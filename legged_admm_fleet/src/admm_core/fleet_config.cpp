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
            // V(side 1.40) translated: pairwise spacing 1.40 >= D_MIN=1.3 (dogs stop at
            // zero formation stress). Follower->cyl(5.5,0.40) clearance hypot(0.688,0.30)
            // =0.751 >= r_eff 0.60 (verify_arena margin 0.30).
            {{1, {7.4, 0.0}}, {2, {6.188, 0.7}}, {3, {6.188, -0.7}}},
        };
        Arena plum;
        // Vision60 quincunx weave, 7 rows / 17 piles (row pitch 1.82; A-rows y=+-1.54,
        // B-rows y=0,+-2.94). CBF radius 0.30 per pile (physical pile r=0.20 in the SDF).
        // Clearances (live robot_margin 0.60, so a corridor must span 2*(0.30+0.60)=1.80):
        //   in-row gaps 3.08 (A) / 2.94 (B); min diagonal anywhere hypot(1.82,1.40)=2.296
        //   -> +0.496 slack, corridor half-width 0.248 > A* res 0.15 (a grid path exists).
        //   offline verify (margin 0.20, need >=1.00): +1.30 slack. Keep gen_arena_world in sync.
        for (const auto& p : std::vector<Eigen::Vector2d>{
                 {4.76, 1.54}, {4.76, -1.54}, {6.58, 0.0}, {6.58, 2.94}, {6.58, -2.94},
                 {8.40, 1.54}, {8.40, -1.54}, {10.22, 0.0}, {10.22, 2.94}, {10.22, -2.94},
                 {12.04, 1.54}, {12.04, -1.54}, {13.86, 0.0}, {13.86, 2.94}, {13.86, -2.94},
                 {15.68, 1.54}, {15.68, -1.54}})
            plum.obstacles.push_back({p, 0.30});
        // V(side 1.40) past the last row; follower->last peg(15.68,+-1.54)
        // hypot(1.108,0.84)=1.390 >= 0.90 (live r_eff).
        plum.goals = {{1, {18.0, 0.0}}, {2, {16.788, 0.7}}, {3, {16.788, -0.7}}};
        a["plum"] = plum;
        Arena plum_dense;
        // Denser variant: 7-row quincunx scaled 0.958x -> min diagonal gap 2.20 m (row pitch
        // 1.744, A-rows y=+-1.48, B-rows y=0,+-2.82). Corridor half-width 2.20/2 - 0.90 = 0.20 m
        // (> A* res 0.15). Denser than the 2.30 base but backs off the 2.10 edge where the
        // dog-dog clearance swung to 0.128 m over 5 trials -- 2.20 keeps A* margin + spacing.
        for (const auto& p : std::vector<Eigen::Vector2d>{
                 {4.56, 1.48}, {4.56, -1.48}, {6.31, 0.0}, {6.31, 2.82}, {6.31, -2.82},
                 {8.05, 1.48}, {8.05, -1.48}, {9.79, 0.0}, {9.79, 2.82}, {9.79, -2.82},
                 {11.54, 1.48}, {11.54, -1.48}, {13.28, 0.0}, {13.28, 2.82}, {13.28, -2.82},
                 {15.02, 1.48}, {15.02, -1.48}})
            plum_dense.obstacles.push_back({p, 0.30});
        plum_dense.goals = {{1, {17.2, 0.0}}, {2, {15.988, 0.7}}, {3, {15.988, -0.7}}};
        a["plum_dense"] = plum_dense;
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
        // Followers 0.232 deeper (x 12.04->11.808) so all pairs = 1.400 >= D_MIN=1.3;
        // leader x=13.02 UNCHANGED (:43-46 wall-stall lesson). Follower->cyl(11.2,0.7)
        // hypot(0.608,0.7)=0.927 >= 0.90 (live r_eff).
        door.goals = {{1, {13.02, 0.7}}, {2, {11.808, 1.4}}, {3, {11.808, 0.0}}};
        a["door"] = door;
        return a;
    }();
    return kArenas;
}

const std::map<std::string, std::vector<Eigen::Vector2d>>& formations() {
    static const std::map<std::string, std::vector<Eigen::Vector2d>> kFormations = {
        // Slot spacings must exceed D_MIN=1.3 (Vision60 leg-reach FK, 2026-07-23). "V" is a
        // centroid-centred equilateral triangle (side 1.40 m, apex forward): all three dogs
        // equidistant/equivalent, so a moving goal translates the shape with minimal
        // reassignment (spawn poses in fleet_robots.yaml match these offsets).
        {"V", {{0.808, 0.0}, {-0.404, 0.7}, {-0.404, -0.7}}},
        {"column", {{0.0, 0.0}, {-1.5, 0.0}, {-3.0, 0.0}}},
        {"V_wide", {{0.0, 0.0}, {-1.0, 1.0}, {-1.0, -1.0}}},
        // Two-dog degraded shape (one peer evicted). A COLUMN, not a line abreast:
        // centroid_slot_targets rotates offsets by yaw = atan2(goal - centroid), so +x is the
        // direction of travel -> the pair strings out along the path and its LATERAL footprint
        // is one body wide, which is what fits through the plum-post gaps. Spacing 1.50 > D_MIN
        // 1.30, else the slot targets would fight the inter-agent CBF forever.
        // NOTE: with n=2 the normalized-Laplacian shape cost is identically zero (L_hat is
        // [[1,-1],[-1,1]] regardless of distance), so this shape is held by the coordinator's
        // slot targets alone, not by w_form. That is expected, not a bug.
        {"COL2", {{0.75, 0.0}, {-0.75, 0.0}}},
    };
    return kFormations;
}

std::optional<CorpseKeepout> corpse_keepout(const Eigen::Vector4d& xnow,
                                            const Eigen::VectorXd& xibar,
                                            double robot_margin) {
    CorpseKeepout k;
    const bool plan_ok = xibar.size() > py_index(N) &&
                         std::isfinite(xibar[px_index(N)]) && std::isfinite(xibar[py_index(N)]);
    k.pos = plan_ok ? Eigen::Vector2d(xibar[px_index(N)], xibar[py_index(N)])
                    : xnow.head<2>();
    if (!k.pos.allFinite()) return std::nullopt;
    k.radius = std::max(0.10, D_MIN - robot_margin);
    return k;
}

const std::map<int, Eigen::Vector2d>& default_goals() {
    // V(side 1.40) translated: pairwise spacing 1.40 >= D_MIN=1.3 (was 0.860, statically
    // infeasible vs the inter-agent CBF). g3/g4 override these with live odom anyway.
    static const std::map<int, Eigen::Vector2d> kGoals = {
        {1, {3.0, 0.0}}, {2, {1.788, 0.7}}, {3, {1.788, -0.7}}};
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
