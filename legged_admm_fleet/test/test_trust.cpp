// Trust layer gates: can I see you, and how much does one look move my belief.
// Plain assert/main like the other gates in this directory.
#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
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

TrustParams params() {
    TrustParams p;              // defaults are the derived values; see trust.hpp
    p.sigma = 0.02;
    return p;
}

void test_llr_zero_crossing_at_half_the_lie() {
    const TrustParams p = params();
    // A residual of exactly d_lie/2 is neutral: below it evidence supports honesty, above it
    // supports lying. Anything else would put the decision boundary somewhere undocumented.
    assert(std::abs(trust_llr(p.d_lie / 2.0, p.sigma, p.d_lie, p.clamp_step)) < 1e-12);
    assert(trust_llr(0.0, p.sigma, p.d_lie, p.clamp_step) > 0.0);
    assert(trust_llr(p.d_lie, p.sigma, p.d_lie, p.clamp_step) < 0.0);
}

void test_llr_is_clamped() {
    const TrustParams p = params();
    assert(std::abs(trust_llr(50.0, p.sigma, p.d_lie, p.clamp_step)) <= p.clamp_step + 1e-12);
    assert(std::abs(trust_llr(0.0, p.sigma, p.d_lie, p.clamp_step)) <= p.clamp_step + 1e-12);
}

void test_trust_has_a_ceiling() {
    const TrustParams p = params();
    double L = 0.0;
    for (int i = 0; i < 1000; ++i) L = trust_step_self(L, p.clamp_step, p);
    assert(L <= p.l_max + 1e-12);   // ten honest minutes must not buy immunity
}

void test_first_hand_evidence_can_evict() {
    const TrustParams p = params();
    double L = p.l_max;             // a peer with a full honest history
    int slots = 0;
    while (!trust_fences_peer(L, p) && slots < 200) {
        L = trust_step_self(L, -p.clamp_step, p);
        ++slots;
    }
    assert(trust_fences_peer(L, p));
    assert(slots <= 10);            // 10 slots = 1.0 s; report the real number in the run log
}

void test_one_relayer_alone_can_never_evict() {
    const TrustParams p = params();
    double C = 0.0;
    for (int i = 0; i < 1000; ++i) C = trust_step_relay(C, -p.clamp_step, p.l_max, p);
    std::map<int, double> relay{{2, C}};
    assert(!trust_fences_peer(trust_total(0.0, relay), p));   // hearsay alone is not a verdict
}

void test_hearsay_cannot_reach_the_eviction_threshold() {
    const TrustParams p = params();
    // The invariant that makes a lone accuser harmless, asserted directly rather than through a
    // particular arithmetic: one relayer's total contribution is capped at l_max, so as long as
    // the ceiling sits above the eviction threshold, no amount of hearsay from one source gets
    // there. Constants may be re-derived; this relationship may not be broken.
    assert(p.l_max < -p.l_evict);
}

void test_first_hand_evidence_still_convicts_with_a_relayer_agreeing() {
    const TrustParams p = params();
    double C = 0.0;
    for (int i = 0; i < 1000; ++i) C = trust_step_relay(C, -p.clamp_step, p.l_max, p);
    std::map<int, double> relay{{2, C}};
    // A corroborating relayer must make conviction faster, never slower, than our own eyes alone.
    int solo = 0, with_help = 0;
    for (double L = 0.0; !trust_fences_peer(L, p) && solo < 200; ++solo)
        L = trust_step_self(L, -p.clamp_step, p);
    for (double L = 0.0; !trust_fences_peer(trust_total(L, relay), p) && with_help < 200; ++with_help)
        L = trust_step_self(L, -p.clamp_step, p);
    assert(with_help <= solo);
    assert(with_help < 200);
}

void test_an_untrusted_relayer_carries_no_weight() {
    const TrustParams p = params();
    double C = 0.0;
    for (int i = 0; i < 1000; ++i) C = trust_step_relay(C, -p.clamp_step, p.l_evict, p);
    assert(std::abs(C) < 1e-9);     // a peer we have already fenced cannot smear anyone
}

void test_belief_decays_toward_neutral() {
    const TrustParams p = params();
    double L = -5.0;
    for (int i = 0; i < 500; ++i) L = trust_step_self(L, 0.0, p);
    assert(std::abs(L) < 1e-3);     // stale evidence expires without an expiry rule
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
    test_llr_zero_crossing_at_half_the_lie();
    test_llr_is_clamped();
    test_trust_has_a_ceiling();
    test_first_hand_evidence_can_evict();
    test_one_relayer_alone_can_never_evict();
    test_hearsay_cannot_reach_the_eviction_threshold();
    test_first_hand_evidence_still_convicts_with_a_relayer_agreeing();
    test_an_untrusted_relayer_carries_no_weight();
    test_belief_decays_toward_neutral();
    std::cout << "test_trust: OK\n";
    return 0;
}
