// Trust layer gates: can I see you, and how much does one look move my belief.
// Plain assert/main like the other gates in this directory.
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "legged_upper_control/trust.hpp"

using namespace admm;

namespace {

// One plum pile on the x axis at 6.58 m, CBF radius 0.30 (fleet_config.cpp).
std::vector<Obstacle> pegs() { return {{Eigen::Vector2d(6.58, 0.0), 0.30}}; }

void test_visible_clear_line() {
    assert(visible({4.0, 0.0}, {5.0, 0.0}, {}, 4.0));
}

void test_visible_out_of_range() {
    assert(!visible({0.0, 0.0}, {5.0, 0.0}, {}, 4.0));
}

void test_visible_blocked_by_peg() {
    // Straight through the pile: 5.0 -> 8.0 passes within 0 m of its centre.
    assert(!visible({5.0, 0.0}, {8.0, 0.0}, pegs(), 4.0));
}

void test_visible_past_the_peg_is_not_blocked() {
    // The pile is BEHIND the observer, not between the two dogs.
    assert(visible({7.0, 0.0}, {8.0, 0.0}, pegs(), 4.0));
}

void test_visible_grazing_miss() {
    // Offset 0.4 m > radius 0.30: line of sight clears the pile.
    assert(visible({5.0, 0.4}, {8.0, 0.4}, pegs(), 4.0));
}

void test_visible_nan_is_not_visible() {
    const double nan = std::nan("");
    // Non-finite target position
    assert(!visible({0.0, 0.0}, {nan, 0.0}, {}, 4.0));
    // Non-finite observer position
    assert(!visible({nan, 0.0}, {5.0, 0.0}, {}, 4.0));
}

void test_visible_nan_obstacle_pos_abstains() {
    const double nan = std::nan("");
    const std::vector<Obstacle> bad_obs{{Eigen::Vector2d(nan, 0.0), 0.30}};
    // A non-finite obstacle forces abstention even on a clear sightline.
    assert(!visible({5.0, 0.0}, {8.0, 0.0}, bad_obs, 4.0));
}

void test_visible_nan_obstacle_radius_abstains() {
    const double nan = std::nan("");
    const std::vector<Obstacle> bad_obs{{Eigen::Vector2d(6.58, 0.0), nan}};
    // A non-finite radius forces abstention.
    assert(!visible({5.0, 0.0}, {8.0, 0.0}, bad_obs, 4.0));
}

}  // namespace

int main() {
    test_visible_clear_line();
    test_visible_out_of_range();
    test_visible_blocked_by_peg();
    test_visible_past_the_peg_is_not_blocked();
    test_visible_grazing_miss();
    test_visible_nan_is_not_visible();
    test_visible_nan_obstacle_pos_abstains();
    test_visible_nan_obstacle_radius_abstains();
    std::cout << "test_trust: OK\n";
    return 0;
}
