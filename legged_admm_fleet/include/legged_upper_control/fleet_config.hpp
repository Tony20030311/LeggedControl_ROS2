#pragma once
// C6 fleet configuration + slot math — C++ mirror of the module-level data and
// functions of scripts/ocs2_fleet_publisher.py (ARENAS / FORMATIONS / DEFAULT_GOALS,
// rot2d / centroid_slot_targets / min_cost_assignment). Single source of truth:
// the pybind module re-exports these so the Python side reads the same literals
// (test_cpp_parity.py test_c6_fleet_*).
#include <Eigen/Dense>

#include <algorithm>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "legged_upper_control/admm_node_qp.hpp"  // Obstacle, Wall

namespace admm {

struct ArenaRect {  // A*-only wall box {center, size}
    Eigen::Vector2d center;
    Eigen::Vector2d size;
};

struct Arena {
    std::vector<Obstacle> obstacles;
    std::vector<Wall> walls;      // empty for arenas without a "walls" key
    std::vector<ArenaRect> rects; // empty for arenas without a "rects" key
    std::map<int, Eigen::Vector2d> goals;
};

const std::map<std::string, Arena>& arenas();
const std::map<std::string, std::vector<Eigen::Vector2d>>& formations();
const std::map<int, Eigen::Vector2d>& default_goals();

// Where to fence off a peer that has just gone silent — the ONE place that decides corpse
// geometry, so it can be unit-tested instead of only observed in a 6-minute Gazebo run.
//
//   pos    : the peer's last broadcast plan terminal knot when that is usable, else its last
//            reported state. The plan knot leads the body, which is where it coasts to: the
//            lower layer keeps executing whatever prefix it already holds.
//   radius : D_MIN - robot_margin, so the realised keep-out (the node QP inflates every
//            obstacle by robot_margin) is exactly D_MIN — a dead dog is held at the same
//            standoff as a live one, no magic constant on top.
//
// Returns nullopt when NO finite position is known. That matters: a NaN centre poisons the
// node QP, the OSQP update guard rejects it, and the whole fleet freezes — which is the exact
// failure this eviction path exists to prevent. A dying agent is disproportionately likely to
// have broadcast garbage, because the operating point goes on the wire BEFORE it is solved for
// (AgentCore::step step 1), and on a cold start that value is an unchecked QP output.
struct CorpseKeepout {
    Eigen::Vector2d pos;
    double radius;
};
// Reaction time budgeted against a peer that is moving and NOT cooperating (experiment C: a
// node evicted for broadcasting lies keeps walking). 2 slots — one to see its new odom, one to
// act on it. Lives here, not in admm_constants.hpp: it is failure-semantics geometry, and no
// barrier, QP or oracle-pinned number depends on it.
inline constexpr double TAU_REACT = 0.2;
// mobile=true when the peer was evicted for LYING rather than for going silent: it is still
// walking, and unlike a corpse it is not cooperating with anyone's CBF. The extra radius is the
// distance it can cover while this agent reacts — the worst case is absorbed into the geometry
// rather than into the barrier, so the CBF form is untouched.
// anchor_knot: WHICH knot of the dead peer's last plan predicts where its body actually stops.
// K_SEND, not N: the lower layer only ever received K_SEND steps of that trajectory and parks
// when it runs out, so the terminal knot overshoots by (N-K_SEND)*TS*v — measured 0.385 m mean
// over 16 evictions against a derived 0.40 m, which put the real body 0.385 m nearer the
// survivors than the keep-out believed (1.30 nominal -> 0.915 m of real clearance on the
// approach side, against a 0.867 m contact line). Parameterised only so the A/B can be run with
// one binary; N is the known-unsafe setting.
// Residual, accepted for now: k_send is not fixed — cycle() truncates the published prefix for
// direction-aware safety and that decision is local, never broadcast, so a receiver cannot know
// a peer sent only 4 steps. Anchoring at K_SEND then overshoots by at most (10-1)*TS*v = 0.36 m.
// Truncation is rare. The root fix is to put the actual k_send in AgentState — one field, worth
// doing together with Phase 2's view_id.
std::optional<CorpseKeepout> corpse_keepout(const Eigen::Vector4d& xnow,
                                            const Eigen::VectorXd& xibar,
                                            double robot_margin, bool mobile = false,
                                            int anchor_knot = K_SEND);

// 2D rotation matrix (world frame).
Eigen::Matrix2d rot2d(double yaw);

// Per-slot world targets so the fleet CENTROID lands on goal_c, V facing yaw.
std::vector<Eigen::Vector2d> centroid_slot_targets(
    const Eigen::Vector2d& goal_c, const std::vector<Eigen::Vector2d>& offsets,
    double yaw);

// Formation name for a fleet of n_live robots out of a roster of n_full. The coordinator
// (which places slot targets) and every agent (which weights the shape cost) must call THIS
// function — if they disagree, the slot targets pull toward one shape while the formation
// gradient pulls toward another and the two fight forever.
// A full fleet keeps whatever shape was configured (`full_shape`, the `formation` param);
// only a degraded fleet falls back to a built-in. "" means "no shape defined": callers must
// then pass a null LaplacianFormation and rely on per-robot goals alone.
// NOTE LaplacianFormation::set_formation("") is a SILENT NO-OP that leaves the previous
// offsets in place, so callers must also check offsets->size() == n_live, not just non-null.
inline std::string shape_for(std::size_t n_live, std::size_t n_full,
                             const std::string& full_shape) {
    if (n_live == n_full) return full_shape;
    if (n_live == 2) return "COL2";  // column: minimal lateral footprint for narrow gaps
    return "";                       // 1 live robot has no shape; ditto anything unsupported
}

// Has a MAJORITY of the OTHER agents dropped robot `i` from their roster? `views` maps each
// robot to the member list it last broadcast (AgentState.members).
//
// The coordinator needs this because heartbeat liveness cannot see an eviction: an agent the
// fleet has blocked keeps broadcasting and reads as alive forever, so it goes on being handed
// formation slots (measured 2026-07-28 — agents saw 2 dogs, the coordinator saw 3).
//
// MAJORITY, not one vote, and that is the security-relevant part: a compromised robot
// broadcasting a roster of only itself excludes every honest dog at once, so a single-vote rule
// would let one attacker delete the honest fleet from the coordinator's view and keep the
// formation to itself. At n=3 the attacker musters one vote against each honest dog — never a
// majority of the two remaining voters — while both honest dogs vote against it.
//
// A robot never votes on itself, and an EMPTY roster abstains rather than voting "everyone is
// in": empty means a pre-members sender, and counting it as a voter would dilute the honest
// majority with a robot that never had an opinion.
inline bool majority_excluded(const std::map<int, std::vector<int>>& views, int i) {
    int voters = 0, votes = 0;
    for (const auto& kv : views) {
        if (kv.first == i || kv.second.empty()) continue;
        ++voters;
        if (std::find(kv.second.begin(), kv.second.end(), i) == kv.second.end()) ++votes;
    }
    return voters > 0 && 2 * votes > voters;
}

// Permutation of slot indices minimising sum ||pos[k]-slot[perm[k]]||^2
// (lexicographic-first on ties, mirroring itertools.permutations + strict <).
std::vector<int> min_cost_assignment(const std::vector<Eigen::Vector2d>& positions,
                                     const std::vector<Eigen::Vector2d>& slots);

}  // namespace admm
