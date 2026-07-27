#pragma once
// C6 fleet configuration + slot math — C++ mirror of the module-level data and
// functions of scripts/ocs2_fleet_publisher.py (ARENAS / FORMATIONS / DEFAULT_GOALS,
// rot2d / centroid_slot_targets / min_cost_assignment). Single source of truth:
// the pybind module re-exports these so the Python side reads the same literals
// (test_cpp_parity.py test_c6_fleet_*).
#include <Eigen/Dense>

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
std::optional<CorpseKeepout> corpse_keepout(const Eigen::Vector4d& xnow,
                                            const Eigen::VectorXd& xibar,
                                            double robot_margin);

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

// Permutation of slot indices minimising sum ||pos[k]-slot[perm[k]]||^2
// (lexicographic-first on ties, mirroring itertools.permutations + strict <).
std::vector<int> min_cost_assignment(const std::vector<Eigen::Vector2d>& positions,
                                     const std::vector<Eigen::Vector2d>& slots);

}  // namespace admm
