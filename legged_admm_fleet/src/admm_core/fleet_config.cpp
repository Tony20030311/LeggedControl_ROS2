// C++ mirror of scripts/ocs2_fleet_publisher.py module-level data + slot math.
// Literals MUST stay byte-identical to the Python source (parity gate compares
// the re-exported py dicts against the publisher's until the .py is deleted).
#include "legged_upper_control/fleet_config.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace admm {

const std::map<std::string, Arena>& arenas() {
    static const std::map<std::string, Arena> kArenas = [] {
        std::map<std::string, Arena> a;
        a["obstacles"] = Arena{
            {{{4.0, 0.40}, 0.30}, {{5.5, 0.40}, 0.30}},
            {},
            {},
            {{1, {7.0, 0.0}}, {2, {6.3, 0.5}}, {3, {6.3, -0.5}}},
        };
        Arena plum;
        // Vision60 upscale: arena is A1-native (min peg spacing 1.64 m -> only +0.09 m
        // planner clearance at r_eff 0.30+0.43). Positions (obstacles AND goals) scaled
        // 1.4x about origin -> spacing 2.30 m, +0.42 m clearance == A1's original margin.
        // CBF radius (0.30) is NOT scaled; keep gen_arena_world.py plum in sync.
        for (const auto& p : std::vector<Eigen::Vector2d>{
                 {4.76, 1.54}, {4.76, -1.54}, {6.58, 0.0}, {6.58, 2.94}, {6.58, -2.94},
                 {8.40, 1.54}, {8.40, -1.54}, {10.22, 0.0}, {10.22, 2.94}, {10.22, -2.94}})
            plum.obstacles.push_back({p, 0.30});
        plum.goals = {{1, {12.6, 0.0}}, {2, {11.62, 0.7}}, {3, {11.62, -0.7}}};
        a["plum"] = plum;
        Arena door;
        // Vision60 upscale: A1-native door scaled 1.4x about origin (matching plum). Obstacle
        // positions AND wall/rect geometry AND goals scale; obstacle radii + wall d_safe stay
        // (they are the physical body/clearance, not the layout). Keep gen_arena_world door in sync.
        for (const auto& p : std::vector<Eigen::Vector2d>{
                 {10.5, 3.5}, {11.9, -2.1}, {12.6, 4.9}, {9.8, -4.2}, {11.2, 0.7}})
            door.obstacles.push_back({p, 0.30});  // 5 field cylinders (r_eff 0.60)
        for (const auto& p : std::vector<Eigen::Vector2d>{
                 {8.4, 1.75}, {8.4, 2.45}, {8.4, -1.75}, {8.4, -2.45}})
            door.obstacles.push_back({p, 0.15});  // door posts (r_eff 0.45)
        door.walls = {
            // d_safe stays 0.30 (2026-07-23): tried 0.60 to cover legs, but the door goals put the
            // lead dog at x~13.3 (leg reaches 13.84 < wall face 13.895 -> no leg-wall contact anyway),
            // and 0.60 pushed the last-waypoint V-slot into the wall keep-out -> fleet stalled 2/3.
            // Only worst-case avoidance would clip; the goals never do. Posts/field cyl keep 0.60.
            {{-1.0, 0.0}, {13.895, 0.0}, 0.30},  // right x=14 (normal unscaled, point x1.4)
            {{0.0, -1.0}, {0.0, 6.895}, 0.30},   // top y=7
            {{0.0, 1.0}, {0.0, -6.895}, 0.30},   // bottom y=-7
        };
        door.rects = {
            {{8.4, 4.375}, {0.21, 5.25}},    // wall_left_upper
            {{8.4, -4.375}, {0.21, 5.25}},   // wall_left_lower
            {{14.0, 0.0}, {0.21, 14.0}},     // wall_right
            {{11.2, 7.0}, {5.6, 0.21}},      // wall_top
            {{11.2, -7.0}, {5.6, 0.21}},     // wall_bottom
        };
        door.goals = {{1, {13.02, 0.7}}, {2, {12.04, 1.4}}, {3, {12.04, 0.0}}};
        a["door"] = door;
        return a;
    }();
    return kArenas;
}

const std::map<std::string, std::vector<Eigen::Vector2d>>& formations() {
    static const std::map<std::string, std::vector<Eigen::Vector2d>> kFormations = {
        // Slot spacings must exceed D_MIN=1.0 (Vision60 footprint). "V" is a centroid-
        // centred equilateral triangle (side 1.40 m, apex forward): all three dogs
        // equidistant/equivalent, so a moving goal translates the shape with minimal
        // reassignment (spawn poses in fleet_robots.yaml match these offsets).
        {"V", {{0.808, 0.0}, {-0.404, 0.7}, {-0.404, -0.7}}},
        {"column", {{0.0, 0.0}, {-1.2, 0.0}, {-2.4, 0.0}}},
        {"V_wide", {{0.0, 0.0}, {-1.0, 1.0}, {-1.0, -1.0}}},
    };
    return kFormations;
}

const std::map<int, Eigen::Vector2d>& default_goals() {
    static const std::map<int, Eigen::Vector2d> kGoals = {
        {1, {3.0, 0.0}}, {2, {2.3, 0.5}}, {3, {2.3, -0.5}}};
    return kGoals;
}

Eigen::Matrix2d rot2d(double yaw) {
    // GCC fuses adjacent cos()+sin() into glibc sincos(), whose cos differs from
    // a separate cos() in the last bit; Python calls them separately. Volatile
    // fn pointers force two real libm calls (bit parity, compiler-proof).
    static double (*volatile p_cos)(double) = static_cast<double (*)(double)>(&std::cos);
    static double (*volatile p_sin)(double) = static_cast<double (*)(double)>(&std::sin);
    const double c = p_cos(yaw), s = p_sin(yaw);
    Eigen::Matrix2d r;
    r << c, -s, s, c;
    return r;
}

std::vector<Eigen::Vector2d> centroid_slot_targets(
    const Eigen::Vector2d& goal_c, const std::vector<Eigen::Vector2d>& offsets,
    double yaw) {
    // np.mean(offsets, axis=0): sequential per-component sum, then / n
    const auto n = offsets.size();
    Eigen::Vector2d mean(0.0, 0.0);
    for (const auto& o : offsets) {
        mean(0) += o(0);
        mean(1) += o(1);
    }
    mean(0) /= static_cast<double>(n);
    mean(1) /= static_cast<double>(n);
    const Eigen::Matrix2d R = rot2d(yaw);
    std::vector<Eigen::Vector2d> slots;
    slots.reserve(n);
    for (const auto& o : offsets) {
        const double ox = o(0) - mean(0), oy = o(1) - mean(1);
        // R @ o mirrored per component, then goal_c + ...
        slots.emplace_back(goal_c(0) + (R(0, 0) * ox + R(0, 1) * oy),
                           goal_c(1) + (R(1, 0) * ox + R(1, 1) * oy));
    }
    return slots;
}

std::vector<int> min_cost_assignment(const std::vector<Eigen::Vector2d>& positions,
                                     const std::vector<Eigen::Vector2d>& slots) {
    const int n = static_cast<int>(positions.size());
    std::vector<int> best(n), perm(n);
    std::iota(best.begin(), best.end(), 0);
    std::iota(perm.begin(), perm.end(), 0);
    double bestcost = 1e18;
    do {  // std::next_permutation from identity == itertools lexicographic order
        double c = 0.0;  // Python sum(): sequential
        for (int k = 0; k < n; ++k) {
            const double dx = positions[k](0) - slots[perm[k]](0);
            const double dy = positions[k](1) - slots[perm[k]](1);
            c += dx * dx + dy * dy;  // np.dot 2-vec
        }
        if (c < bestcost) {
            bestcost = c;
            best = perm;
        }
    } while (std::next_permutation(perm.begin(), perm.end()));
    return best;
}

}  // namespace admm
