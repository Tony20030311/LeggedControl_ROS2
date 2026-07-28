#pragma once
// AgentCore — one robot's slice of ADMMCoordinator::step, driven by an injected Transport
// instead of shared std::maps. The centralized step is already node/edge-separable (each
// node solve reads only its own consensus_target; each edge solve reads only its two
// endpoints; dual update is per-edge local) — that is exactly why the centralized loop
// OpenMP-parallelizes bit-identically. Splitting one process per robot with a barrier per
// phase therefore reproduces ADMMCoordinator::step bit-for-bit (G1), provided every
// per-entity operation keeps the same operands in the same order.
//
// Transport is abstract so the SAME AgentCore runs under two backends:
//   - LoopbackTransport (test): synchronous, no timeout -> bit-identical parity gate.
//   - DdsTransport (admm_agent_node): async with per-hop timeouts -> G4 real distribution.
//
// Implementation in agent_core.cpp. Messages are ROS-agnostic structs here; the node maps
// them to admm_fleet_msgs on the wire.
#include <cstdint>
#include <map>
#include <algorithm>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "legged_upper_control/admm_constants.hpp"
#include "legged_upper_control/admm_coordinator.hpp"  // EdgeKey, consensus_target, EdgeVecs
#include "legged_upper_control/admm_edge_qp.hpp"
#include "legged_upper_control/admm_formation.hpp"
#include "legged_upper_control/admm_node_qp.hpp"
#include "legged_upper_control/admm_rti.hpp"

namespace admm {

// --- wire structs (ROS-agnostic; node wraps into admm_fleet_msgs) ---
struct AgentStateMsg {
    std::uint64_t cycle_id = 0;
    int robot_id = 0;
    Eigen::Vector4d xnow = Eigen::Vector4d::Zero();
    Eigen::VectorXd xibar;   // 6N operating-point plan
    bool reset = false;
};

// GATE 1 — the syntactic filter on the receive path, deliberately loose.
//
// Everything a peer says about itself is believed today: onState stores the message verbatim,
// so a node that keeps talking but lies is never evicted (eviction triggers on SILENCE) and its
// numbers go straight into the consensus and the CBF. This rejects only what a HEALTHY agent
// could never emit. With n=3, wrongly dropping one good message costs a dog, so every threshold
// here carries 3x slack: false negatives are cheap (Gate 2 catches the lie), false positives
// are not.
//
// It also CHAINS the two claimed positions together: xibar's first knot must agree with xnow.
// Both fields are consumed downstream (xnow feeds followSpeed and the corpse anchor fallback,
// xibar feeds the CBF operating point), so leaving either unchecked leaves a free channel for
// a lie. Chaining them here means Gate 2 only has to anchor ONE of them to odom.
//
// NOTE on indexing: xi has no k=0 — px_index(k) = 4*(k-1) starts at k=1 — so xibar's first
// position knot is TS ahead of xnow, not equal to it. That is why the comparison carries a
// velocity allowance rather than being an equality.
struct GateLimits {
    double x_min = -2.0, x_max = 20.0, y_min = -7.0, y_max = 7.0;
    // How far outside the A* map a legitimate plan may reach — and it is a bound on "a
    // plausible world", NOT on the planning map. The fleet is explicitly allowed to operate off
    // the map: a return goal at x=-5 against x_min=-2 is a supported scenario (d_matrix
    // farhome), and the planner clamps into the map while the final leg walks honestly past its
    // edge. Sizing this to the map rejected those legitimate plans and, because each survivor
    // then looked silent to the other, they evicted EACH OTHER and lost the pairwise CBF
    // entirely (measured 2026-07-28, d_0728_075637: 17 mutual rejections, min pair 0.832).
    // So: wide enough that only absurd values fail. Anything plausible-but-wrong is Gate 2's job.
    double arena_slack = 20.0;
    double vel_margin = 3.0;    // multiple of MAX_VX*TS allowed between consecutive plan knots
    // How far from the receiver's own slot a claimed cycle_id may be. The mailbox only keeps 8
    // slots (dds_transport kKeep), so anything outside that window is unusable even when
    // honest; 16 leaves a factor of two for clock skew and scheduling jitter.
    std::uint64_t slot_window = 16;
};

// WHICH check failed, not just that one did. A rejected message is indistinguishable from a
// missing one by design (that is how detection reuses the crash path), so a Gate 1 false
// positive presents as a peer dying — and with both survivors rejecting each other, as the
// pairwise CBF disappearing entirely. Twice on 2026-07-28 the only evidence was a throttled
// "REJECTED" line that named no reason, which is not enough to tell a bug from an attack.
enum class GateReason { kOk = 0, kRoster, kLength, kNonFinite, kArena, kStep,
                        kVel, kAccel, kSlot };
inline const char* gateReasonName(GateReason r) {
    switch (r) {
        case GateReason::kOk: return "ok";
        case GateReason::kRoster: return "roster";
        case GateReason::kLength: return "length";
        case GateReason::kNonFinite: return "non-finite";
        case GateReason::kArena: return "arena";
        case GateReason::kStep: return "step kinematics";
        case GateReason::kVel: return "claimed velocity";
        case GateReason::kAccel: return "plan acceleration";
        case GateReason::kSlot: return "cycle_id window";
    }
    return "?";
}

// worst_margin (optional out): how close the message came to the limit that rejected it, or the
// tightest margin over all checks when it passes. Feeding this back is what makes the
// thresholds calibratable from a clean run instead of argued from the dynamics.
inline GateReason gateCheck(const AgentStateMsg& m, const std::vector<int>& roster,
                            const GateLimits& g, double* worst_margin = nullptr,
                            std::uint64_t now_slot = 0) {
    if (worst_margin) *worst_margin = 0.0;
    if (std::find(roster.begin(), roster.end(), m.robot_id) == roster.end())
        return GateReason::kRoster;
    if (m.xibar.size() != XI_DIM) return GateReason::kLength;
    if (!m.xibar.allFinite() || !m.xnow.allFinite()) return GateReason::kNonFinite;
    // cycle_id must name a slot near NOW. It is not just a label: it keys the mailbox, it
    // advances last_seen_ (so a peer claiming slot+1000 can never be judged silent again), and
    // it drives the prune window (so one such message erases the honest peers' mail as well).
    // A liar needs no position lie at all for that — it is the cheapest possible attack, and
    // the fleet's answer would be a permanent barrier timeout with no eviction and no corpse.
    // now_slot == 0 means "caller has no clock" (unit tests), and the check is skipped.
    if (now_slot != 0) {
        const std::uint64_t lo = now_slot > g.slot_window ? now_slot - g.slot_window : 0;
        if (m.cycle_id < lo || m.cycle_id > now_slot + g.slot_window) {
            if (worst_margin)
                *worst_margin = static_cast<double>(m.cycle_id) - static_cast<double>(now_slot);
            return GateReason::kSlot;
        }
    }
    // Claimed velocity. Only allFinite before, and it is consumed: followSpeed brakes on a
    // peer's claimed v_ahead, so a large negative claim pins a survivor at the follow floor
    // (0.05 m/s) indefinitely — a denial of movement that needs no detectable position error.
    if (std::abs(m.xnow[2]) > MAX_VX * g.vel_margin ||
        std::abs(m.xnow[3]) > MAX_VY * g.vel_margin) {
        if (worst_margin) *worst_margin = std::max(std::abs(m.xnow[2]), std::abs(m.xnow[3]));
        return GateReason::kVel;
    }
    // The ACCELERATION block of xibar, which the position checks never touched. It is what the
    // edge CBF linearises around (rti.cpp: u = hbar - g.(abar_i - abar_j)), so a message whose
    // positions are honest — passing every other check here AND Gate 2's odom residual — can
    // still make the pairwise bound vacuous, switching off the inter-robot barrier silently.
    for (int k = 0; k < N; ++k) {
        if (std::abs(m.xibar[ax_index(k)]) > MAX_AX * g.vel_margin ||
            std::abs(m.xibar[ay_index(k)]) > MAX_AY * g.vel_margin) {
            if (worst_margin)
                *worst_margin = std::max(std::abs(m.xibar[ax_index(k)]),
                                         std::abs(m.xibar[ay_index(k)]));
            return GateReason::kAccel;
        }
    }
    // NO xnow <-> xibar consistency check. It was here, and it had to go: xibar is last cycle's
    // plan SHIFTED by one step while xnow is a fresh measurement, and during a HOLD the plan is
    // deliberately frozen while the body keeps walking off the last published prefix. Measured
    // on honest traffic: 0.597, 0.623, 0.668, 1.335 m. To stop rejecting those the bound would
    // have to sit above 2 m — which is the size of the lie it exists to catch, so there is no
    // threshold that both accepts honest messages and constrains a liar.
    // Consequence, stated rather than hidden: xibar's CONTENT is only self-constrained here
    // (finite / arena / step kinematics / acceleration). It is not anchored to any independent
    // channel — Gate 2 anchors xnow, and nothing anchors the plan. Future work, alongside the
    // ungated EdgeXi/EdgeZ messages.
    const double step = MAX_VX * TS * g.vel_margin;
    double worst = 0.0;
    for (int k = 1; k <= N; ++k) {
        const double x = m.xibar[px_index(k)], y = m.xibar[py_index(k)];
        if (x < g.x_min - g.arena_slack || x > g.x_max + g.arena_slack ||
            y < g.y_min - g.arena_slack || y > g.y_max + g.arena_slack) {
            if (worst_margin) *worst_margin = std::max(std::abs(x), std::abs(y));
            return GateReason::kArena;
        }
        if (k < N) {
            const Eigen::Vector2d a(x, y);
            const Eigen::Vector2d b(m.xibar[px_index(k + 1)], m.xibar[py_index(k + 1)]);
            const double d = (b - a).norm();
            worst = std::max(worst, d / step);
            if (d > step) { if (worst_margin) *worst_margin = d; return GateReason::kStep; }
        }
    }
    if (worst_margin) *worst_margin = worst;   // fraction of the tightest limit actually used
    return GateReason::kOk;
}

inline bool wellFormed(const AgentStateMsg& m, const std::vector<int>& roster,
                       const GateLimits& g) {
    return gateCheck(m, roster, g) == GateReason::kOk;
}
struct EdgeXiMsg {
    std::uint64_t cycle_id = 0;
    int iter = 0;
    EdgeKey edge{0, 0};
    int from_robot = 0;
    Eigen::VectorXd xi, lam;  // this endpoint's node solution + scaled dual, 6N each
};
struct EdgeZMsg {
    std::uint64_t cycle_id = 0;
    int iter = 0;
    EdgeKey edge{0, 0};
    Eigen::VectorXd z_i, z_j;  // both consensus copies, 6N each
};

// Which endpoint solves edge (i,j)'s edge-QP. Balanced round-robin over a 3-dog complete
// graph -> each robot owns exactly one edge. Ownership is a routing choice ONLY; it never
// changes the math (the edge-QP is deterministic in its inputs), so parity holds for any
// consistent rule. Defined in agent_core.cpp.
int edge_owner(const EdgeKey& e);

// Thrown by an async transport (DdsTransport) when a per-hop recv misses its deadline.
// The loopback transport never throws it, so the G1 parity path is unaffected.
struct TransportTimeout {};

// Result of one AgentCore cycle. hold=true means the peer barrier timed out -> the caller
// should NOT publish a new target this cycle (reuse the last one). achieved_rounds is the
// number of ADMM iterations completed before any deadline (the G5 comms metric).
struct StepResult {
    Eigen::VectorXd xi;
    ADMMCoordinator::Hist hist;
    bool hold = false;
    int achieved_rounds = 0;
    // G5 telemetry (output only; the parity gate compares xi, never these).
    std::vector<double> r_prim_edge;  // final ADMM primal residual per adopted edge (my_edges_ order)
    double r_prim = 0.0;              // max over adopted edges (0 if no round completed)
    int n_timeouts = 0;              // per-hop recv deadline misses observed this cycle
    double t_node = 0.0;             // total node-QP solve time this cycle (s)
    double t_edge_solve = 0.0;       // total owned edge-QP solve time this cycle (s)
    // Why the warm start was thrown away THIS cycle, so the node layer can say so out loud.
    // Both clears used to be silent, which is what made the victim3 stall unexplainable from
    // the log: has_prev() gates eviction arming, and nothing recorded that it had gone false.
    // Output only; AgentCore stays ROS-free, the caller does the logging.
    enum WarmCleared { kWarmKept = 0, kWarmNaN = 1, kWarmPeerReset = 2 };
    int warm_cleared = kWarmKept;
    // Diagnostics for the plan-vs-measurement gap that killed the Gate 1 chain check. xibar is
    // broadcast BEFORE this cycle solves and is last cycle's plan shifted one step, so this is
    // "how far the plan I just told everyone about is from where I actually am". It grows while
    // HOLD freezes the plan and the body walks off the published prefix — hold_streak is the
    // suspected driver, recorded next to it so the claim can be checked instead of asserted.
    double chain_margin = 0.0;   // |xibar[k=1] - xnow| of the message broadcast THIS cycle
    int hold_streak = 0;         // consecutive HOLD cycles ending with this one (0 if solved)
    // Largest k=0 keep-out relaxation this cycle bought. Non-zero means a corpse circle was
    // violated at the measured state and the L1 price was paid — the difference between "the
    // keep-out was in the wrong place" and "the geometry left nowhere to stand", which look
    // identical in a separation plot.
    double k0_slack = 0.0;
};

// Transport: send is fire-and-forget; recv blocks until the matching message(s) arrive.
// The loopback backend resolves recv synchronously (all agents stepped in lockstep); the
// DDS backend adds deadlines and returns a HOLD signal on timeout (G4, not this gate).
class Transport {
public:
    virtual ~Transport() = default;
    virtual void send_state(const AgentStateMsg& m) = 0;
    // barrier: block until every peer in `peers` has published its AgentState for `cycle`.
    virtual std::map<int, AgentStateMsg> recv_states(std::uint64_t cycle,
                                                     const std::vector<int>& peers) = 0;
    virtual void send_xi(int to_robot, const EdgeXiMsg& m) = 0;
    // owner side: block until both endpoints' EdgeXi for (edge, iter) arrive.
    virtual std::map<int, EdgeXiMsg> recv_xi(const EdgeKey& edge, std::uint64_t cycle,
                                             int iter) = 0;
    virtual void send_z(int to_robot, const EdgeZMsg& m) = 0;
    // non-owner side: block until my edge's EdgeZ for (edge, iter) arrives.
    virtual EdgeZMsg recv_z(const EdgeKey& edge, std::uint64_t cycle, int iter) = 0;
    // G5 telemetry: per-hop recv deadline misses since the last call (reads-and-resets).
    // Default 0 so synchronous loopback backends (parity/timeout tests) need no override.
    virtual int take_timeouts() { return 0; }
};

class AgentCore {
public:
    // Same subproblem-shaping args as ADMMCoordinator, but this robot only builds the
    // NodeSubproblem for itself and the EdgeSubproblem for the one edge it owns. `formation`
    // is shared read-only (each agent computes the full grad and keeps its own row — the
    // deterministic-replication contract, plan §67).
    AgentCore(int self_id, std::vector<int> dogs, std::vector<EdgeKey> edges,
              const LaplacianFormation* formation, double w_form,
              std::vector<Obstacle> obstacles, std::vector<Wall> walls, int hard_through,
              Transport* transport, double robot_margin = 0.30);

    // One control cycle for THIS robot. Broadcasts AgentState, barriers on peers, runs the
    // P_ITERS node/edge/dual loop over the transport, returns this robot's xi (6N) plus its
    // local Hist. Bit-identical to ADMMCoordinator::step()'s entry for self_id under the
    // loopback transport.
    // slot = the GLOBAL time slot floor(sim_now/TS), shared by all agents; it keys the
    // transport barrier so a HOLD on one agent never desyncs the fleet. The internal
    // cold-start counter (cycle_) is separate and per-agent.
    StepResult step(const Eigen::Vector4d& xnow_self, const Eigen::MatrixXd& xdes_self,
                    std::uint64_t slot);

    int self_id() const { return self_id_; }
    bool has_prev() const { return has_prev_; }
    // Peers' current state {px,py,vx,vy} received over the consensus barrier in the LAST
    // step() (empty before the first). Exposes the data the centralized node reads directly
    // so admm_agent_node can run the same leader-aware followSpeed / safe_prefix locally.
    const std::map<int, Eigen::Vector4d>& peer_xnow() const { return peer_xnow_; }
    // Peers' broadcast predicted trajectory xibar (6N: [px,py,vx,vy,ax,ay]*N) from the LAST
    // step(); positions feed safe_prefix_length. Keyed by robot id, self included.
    const std::map<int, Eigen::VectorXd>& peer_xibar() const { return peer_xibar_; }
    // Move an already-registered obstacle. The node layer owns WHY an obstacle moves (a peer
    // that broadcasts garbage is still walking, so its keep-out has to follow its odom); this
    // is only the route to the node QP, which holds the obstacle set. No rebuild, no cold start
    // — see NodeSubproblem::set_obstacle_pos. Obstacle ORDER is the construction order, so the
    // caller's index is the index it passed to the constructor.
    void set_obstacle_pos(std::size_t i, const Eigen::Vector2d& pos) {
        if (node_) node_->set_obstacle_pos(i, pos);
    }
    std::size_t n_obstacles() const { return node_ ? node_->n_obstacles() : 0; }

private:
    Eigen::VectorXd uncoupled_self_(const Eigen::Vector4d& xnow,
                                    const Eigen::MatrixXd& xdes);
    // full formation gradient from all peers' xibar; returns this robot's (N,2) row block.
    Eigen::MatrixX2d formation_grad_self_(const std::map<int, Eigen::VectorXd>& xibar) const;

    int self_id_;
    std::map<int, Eigen::Vector4d> peer_xnow_;      // cached from last step() (followSpeed)
    std::map<int, Eigen::VectorXd> peer_xibar_;     // cached from last step() (safe_prefix)
    std::vector<int> dogs_;
    std::vector<EdgeKey> edges_;        // full edge list (for neighbor/owner bookkeeping)
    std::vector<EdgeKey> my_edges_;     // edges incident to self_id_
    std::vector<int> neighbors_;
    const LaplacianFormation* formation_;
    double w_form_;
    int hard_through_;
    int nz_, N_;
    Transport* transport_;

    std::unique_ptr<NodeSubproblem> node_;                       // this robot's node QP
    std::map<EdgeKey, std::unique_ptr<EdgeSubproblem>> owned_edge_;  // edges self owns

    // per-edge local consensus state (self's copies only): z^{ij,self}, lam^{ij,self}
    std::map<EdgeKey, Eigen::VectorXd> z_self_, lam_self_;
    // owner-side cache of BOTH consensus copies for edges I own, so a dead edge-QP solve can
    // resend the last good (z_i, z_j) — mirrors the centralized "keep last z" edge_fail path.
    std::map<EdgeKey, std::pair<Eigen::VectorXd, Eigen::VectorXd>> owned_z_;
    // warm start
    bool has_prev_ = false;
    int hold_streak_ = 0;   // consecutive HOLDs; diagnostic only, never gates anything
    std::uint64_t cycle_ = 0;
    Eigen::VectorXd prev_xi_;
    std::map<EdgeKey, Eigen::VectorXd> prev_z_self_, prev_lam_self_;
    std::map<EdgeKey, std::pair<Eigen::VectorXd, Eigen::VectorXd>> prev_owned_z_;
};

}  // namespace admm
