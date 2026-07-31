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
#include <cmath>
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
    // Sender's current roster. Empty means "not stated" and is read as "everyone still in", so
    // the loopback transport used by the parity test may leave it unset without changing a bit.
    std::vector<int> members;
    // Observation-belief-trust (spec 5.2, task 7): what the sender SAW, not what it concluded.
    // Empty ev_peer means "saw nobody this slot", which is also what a pre-evidence sender means,
    // so the loopback parity harness (which never sets these) is unaffected.
    std::vector<int> ev_peer;      // peer ids observed during the slot named by ev_slot
    std::vector<double> ev_pos;    // observed (x,y) per entry, length exactly 2 * ev_peer.size()
    std::uint64_t ev_slot = 0;     // which slot this evidence describes (see AgentState.msg)
};

// A peer's evidence as received (task 7, spec 5.2): copied straight out of AgentStateMsg once
// Gate 1 has already validated it, so the receiver never has to re-check size/roster/duplicates.
struct PeerEvidence {
    std::vector<int> peer;
    std::vector<double> pos;
    std::uint64_t slot = 0;
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
// It does NOT chain the two claimed positions together. That check existed and had to go —
// see the "NO xnow <-> xibar consistency check" note below, where honest traffic was measured
// at 0.597-1.335 m of legitimate divergence. This comment used to claim the chain was here,
// which is worse than having no check at all: both fields are consumed downstream (xnow feeds
// followSpeed and the corpse anchor fallback, xibar feeds the CBF operating point), so a
// reader who trusts the claim believes the plan is anchored when nothing anchors it.
//
// The consequence is real and measured: an attacker can be truthful about xnow, earning
// clean residuals from the belief layer, while drifting its plan knots up to
// MAX_VX*TS*vel_margin = 0.165 m each — which erodes the effective pairwise standoff from
// 1.300 m to 0.354 m, past the 1.22 m knee-to-knee contact distance. Closing it needs one
// more term in the residual (the peer's own plan knot for this slot, differenced against the
// same observation), not a revived Gate 1 bound, because a HOLD legitimately produces the
// same divergence honest traffic showed.
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
                        kVel, kAccel, kSlot, kEvidence };
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
        case GateReason::kEvidence: return "evidence array";
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
    // EVIDENCE ARRAYS (task 7 item 3, spec 5.2). ev_peer/ev_pos are attacker-controlled and
    // deserialise on the thread that services the consensus barrier, on a host also running a
    // 250 Hz whole-body controller, so a malformed pair is dropped the same way any other
    // impossible payload is: the whole message, not a best-effort partial read.
    if (m.ev_pos.size() != 2 * m.ev_peer.size()) return GateReason::kEvidence;
    if (m.ev_peer.size() > roster.size()) return GateReason::kEvidence;
    for (double v : m.ev_pos)
        if (!std::isfinite(v)) return GateReason::kEvidence;
    for (std::size_t a = 0; a < m.ev_peer.size(); ++a) {
        if (std::find(roster.begin(), roster.end(), m.ev_peer[a]) == roster.end())
            return GateReason::kEvidence;
        for (std::size_t b = a + 1; b < m.ev_peer.size(); ++b)
            if (m.ev_peer[a] == m.ev_peer[b]) return GateReason::kEvidence;   // no duplicates
    }
    // ev_slot itself must be windowed too (review finding 1), the same reason cycle_id is below:
    // an unvalidated ev_slot lets a reporter's report about j miss the observation buffer
    // (interp_at returns nullopt -> have_check=false -> smear_llr contributes 0) while the
    // hearsay-about-j term still runs against j's CURRENT claim regardless -- the report costs
    // the reporter nothing while still costing the target. Skipped when there is no evidence to
    // begin with (empty ev_peer is the "saw nobody"/pre-evidence sentinel and may legitimately
    // carry a stale or zero ev_slot). Bounded above by now_slot itself, not now_slot+slot_window:
    // evidence describes something that already happened, so unlike cycle_id (the SENDER's own
    // broadcast slot, which is allowed to run a little ahead on clock skew) it must never be from
    // the receiver's future.
    if (now_slot != 0 && !m.ev_peer.empty()) {
        const std::uint64_t lo = now_slot > g.slot_window ? now_slot - g.slot_window : 0;
        if (m.ev_slot < lo || m.ev_slot > now_slot) {
            if (worst_margin)
                *worst_margin = static_cast<double>(m.ev_slot) - static_cast<double>(now_slot);
            return GateReason::kEvidence;
        }
    }
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

// --- Experiment C fault-injection helpers (Task 10) ---
// Pure functions, testable without a live Transport/node, and NOT on the G1 parity path: the
// loopback transport used by test_distributed_parity never calls any of them. Each is a no-op
// at its stated default, so an honest run that never sets the corresponding parameter is
// byte-identical to a build that never had these functions at all.

// inject_duty_cycle (spec §9 A2-duty arm): is the sender lying THIS slot? period<=0 (default)
// means "yes, always" -- i.e. whatever knob the caller gates on has full, unfiltered control,
// exactly today's behaviour. Otherwise lie for the first half of every `period`-slot window and
// tell the truth for the second half: trust.hpp's credit/penalty asymmetry exists specifically
// so an intermittent liar still gets convicted, and this is the arm that has to prove it (a
// SYMMETRIC accumulator would never cross the threshold against a 50%-duty attacker).
inline bool dutyLying(std::uint64_t slot, int period) {
    if (period <= 0) return true;
    const auto p = static_cast<std::uint64_t>(period);
    return (slot % p) * 2 < p;
}

// inject_fake_evidence / inject_fake_evidence_target (spec §9 A2-smear arm): corrupt what the
// sender claims to have SEEN of `target` by `smear` metres in x and y, leaving every other
// field (in particular its own xnow/xibar, the attacker's honest position) untouched. A no-op
// when smear==0 (inert default) or when the honest evidence pipeline never reported `target`
// this cycle (out of range/occluded that slot) -- this can corrupt a sighting already in the
// message, not manufacture one that was never there.
inline void applyFakeEvidence(std::vector<int>& ev_peer, std::vector<double>& ev_pos,
                              int target, double smear) {
    if (smear == 0.0) return;
    for (std::size_t k = 0; k < ev_peer.size(); ++k)
        if (ev_peer[k] == target) {
            ev_pos[2 * k]     += smear;
            ev_pos[2 * k + 1] += smear;
        }
}

// inject_odom_fake / inject_odom_fake_peer (Task 10 item 1): forge THIS receiver's copy of
// `target`'s independent odom channel by `off` metres in x and y. Exists so the dual-channel
// forgery (claim AND odom) can be switched on together at the experiment trigger: if this were
// live from t=0 (as a launch parameter, the old shape) while the CLAIM lie only starts at the
// trigger, gate2() spends the whole time before the trigger differencing an honest claim
// against an already-forged odometry -- a 0.424 m residual against the 0.30 m gate -- and
// evicts the attacker minutes before the attack begins, making arm A1's numbers meaningless.
// A no-op when off==0 (inert default) or `target` has no odom sample stored yet.
inline void applyOdomFake(std::map<int, Eigen::Vector2d>& odom_by_peer, int target, double off) {
    if (off == 0.0) return;
    const auto it = odom_by_peer.find(target);
    if (it == odom_by_peer.end()) return;
    it->second[0] += off;
    it->second[1] += off;
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
    // Was peer_xnow()/peer_xibar() actually refreshed THIS call to step()? NOT the same thing as
    // n_timeouts==0: an EdgeXi/EdgeZ timeout deep in the ADMM inner loop is caught, breaks the
    // loop, and still returns hold=false with a perfectly fresh peer_xnow_ (it was assigned once,
    // earlier, right after the AgentState barrier). Only the TransportTimeout thrown by THAT
    // earlier barrier (recv_states) returns before the assignment ever runs. A caller judging
    // whether a peer's cached claim is trustworthy (the belief layer) must read THIS flag, not
    // infer it from n_timeouts or from `hold` (hold is also true on the fleet-reset-sync HOLD,
    // where peer_xnow_ is fresh) — see agent_core.cpp for exactly where each is set.
    bool peer_xnow_fresh = false;
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
    // enable_peer_keepout (task 4b spike, design §4/Decision 2026-07-31): when true, adds ONE
    // fixed obstacle slot per peer to THIS agent's own node QP, modelled on corpse_keepout —
    // same D_MIN standoff, same soft_k0 (may be observed already violated). It is the local,
    // unilateral counterpart to edge ownership: edge_owner decides who LINEARIZES a pair's ADMM
    // barrier, but when the owner is the one lying, its own view of that edge is the only view
    // that ever reaches it. This constraint bypasses ownership entirely, the same way the arena
    // obstacle CBF and corpse_keepout already do — each agent fences every peer it can see,
    // regardless of who owns the edge to it.
    // Default false, and the loopback parity harness (test_distributed_parity) never passes
    // true, so `obstacles` there is untouched and node_'s QP is byte-for-byte what it always
    // was — the constraint does not merely happen to be inactive, the code path never runs.
    AgentCore(int self_id, std::vector<int> dogs, std::vector<EdgeKey> edges,
              const LaplacianFormation* formation, double w_form,
              std::vector<Obstacle> obstacles, std::vector<Wall> walls, int hard_through,
              Transport* transport, double robot_margin = 0.30,
              bool enable_peer_keepout = false);

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
    // CONSERVATIVE ANCHORING (spec 4.1): per-peer position corrections, applied to each peer's
    // whole broadcast before the constraint is built, so the barrier protects the distance to a
    // body instead of the distance to a claim. The node computes them (it owns the observation
    // channel and the per-peer `prev` the rate limit needs); AgentCore only applies them.
    // Not set, or no observation of a peer -> that peer is absent from the map -> zero offset and
    // the arithmetic is untouched. That is why the loopback parity harness, which has no
    // observation channel at all, stays bit-identical.
    void set_peer_offsets(const std::map<int, Eigen::Vector2d>& d) { peer_offset_ = d; }
    // Relayed observations received over the LAST step()'s consensus barrier (task 7, spec 5.2),
    // keyed by the reporting peer's id. The loopback parity harness never populates a peer's
    // ev_peer, so this stays empty under it -- one more reason G1 parity is unaffected.
    const std::map<int, PeerEvidence>& peer_evidence() const { return peer_ev_; }
    // THIS agent's own evidence for the broadcast step() is about to send (spec 5.2's ev_slot):
    // observations collected in the PREVIOUS beliefStep call, since send_state happens at the TOP
    // of step() -- before this cycle's own evidence is produced. Must be called before step() for
    // the same timing reason set_peer_offsets is. Default (never called) sends empty/0, matching
    // an old peer and leaving the loopback parity harness untouched.
    void set_evidence(std::vector<int> peer, std::vector<double> pos, std::uint64_t slot) {
        ev_out_peer_ = std::move(peer);
        ev_out_pos_ = std::move(pos);
        ev_out_slot_ = slot;
    }
    // LOCAL PAIRWISE SAFETY NET (task 4b spike): move peer j's dedicated keep-out circle to
    // where THIS agent's own sensor currently believes it is — the same observed, noised
    // position updatePeerOffsets already computes for conservative anchoring (no second
    // observation path). Enforced in THIS agent's own node QP unconditionally, independent of
    // edge_owner: it stands even when the edge to that peer is linearised by the peer itself.
    // A peer absent from `pos` this cycle (no fresh observation) is left wherever it last was —
    // the far-away park default before the first observation ever arrives — the same
    // "no evidence -> no correction" contract set_peer_offsets already implements, and the same
    // "never binds" inactivity an out-of-range arena obstacle already relies on.
    // No-op when enable_peer_keepout was false at construction (peer_keepout_idx_ empty).
    void set_peer_keepout(const std::map<int, Eigen::Vector2d>& pos) {
        for (const auto& kv : pos) {
            const auto it = peer_keepout_idx_.find(kv.first);
            if (it != peer_keepout_idx_.end() && node_) node_->set_obstacle_pos(it->second, kv.second);
        }
    }
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
    std::map<int, Eigen::Vector2d> peer_offset_;    // conservative anchoring; empty = no observations
    std::map<int, PeerEvidence> peer_ev_;           // task 7: peers' evidence from the LAST step()
    // task 7: THIS agent's own evidence for the NEXT broadcast (set_evidence); empty/0 until the
    // node calls set_evidence at least once.
    std::vector<int> ev_out_peer_;
    std::vector<double> ev_out_pos_;
    std::uint64_t ev_out_slot_ = 0;
    // task 4b: peer robot id -> its dedicated keep-out obstacle's index in node_'s obstacle
    // list. Empty when enable_peer_keepout was false at construction (set_peer_keepout no-ops).
    std::map<int, std::size_t> peer_keepout_idx_;
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
