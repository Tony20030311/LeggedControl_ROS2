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
    std::uint64_t cycle_ = 0;
    Eigen::VectorXd prev_xi_;
    std::map<EdgeKey, Eigen::VectorXd> prev_z_self_, prev_lam_self_;
    std::map<EdgeKey, std::pair<Eigen::VectorXd, Eigen::VectorXd>> prev_owned_z_;
};

}  // namespace admm
