#pragma once
// Who to believe. Pure functions only: visibility, the log-odds accumulator that turns
// observation residuals into a belief, and the thresholds that belief crosses. Header-only
// inline, the same way majority_excluded lives in fleet_config.hpp — no node, no ROS, no state,
// so every rule here is unit-testable instead of only observable in a six-minute Gazebo run.
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include "legged_upper_control/admm_node_qp.hpp"  // Obstacle

namespace admm {

// Can an observer at pi see a body at pj? Range gate plus a segment-circle occlusion test
// against the known arena.
//
// The obstacle radius here is the CBF radius (0.30 for a plum pile; the physical pile is 0.20),
// so this over-occludes by 0.10 m. That errs toward ABSTAINING, which is the safe direction:
// abstaining costs latency, a false accusation costs a dog — and a dog wrongly fenced off is
// exactly the failure that zeroes the safety mechanism.
inline bool visible(const Eigen::Vector2d& pi, const Eigen::Vector2d& pj,
                    const std::vector<Obstacle>& obs, double range) {
    const Eigen::Vector2d d = pj - pi;
    if (!pi.allFinite() || !pj.allFinite()) return false;
    const double len = d.norm();
    if (!(len <= range)) return false;   // written so NaN falls through to false
    if (len < 1e-9) return true;
    for (const auto& o : obs) {
        const Eigen::Vector2d w = o.pos - pi;
        const double t = w.dot(d) / (len * len);   // where the foot of the perpendicular lands
        if (t <= 0.0 || t >= 1.0) continue;        // the pile is behind me or past the target
        if ((w - t * d).norm() < o.radius) return false;
    }
    return true;
}

}  // namespace admm
