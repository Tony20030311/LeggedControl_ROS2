// Trust layer gates: can I see you, and how much does one look move my belief.
// Plain assert/main like the other gates in this directory.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>
#include <iostream>
#include <map>
#include <vector>

#include "agent_core.hpp"
#include "legged_upper_control/admm_constants.hpp"  // TS
#include "legged_upper_control/admm_reference.hpp"
#include "legged_upper_control/trust.hpp"

using namespace admm;

namespace {

// One plum pile on the x axis at 6.58 m, CBF radius 0.30 (fleet_config.cpp).
std::vector<Obstacle> pegs() { return {{Eigen::Vector2d(6.58, 0.0), 0.30}}; }

TrustParams params() {
    TrustParams p;              // defaults are the derived values; see trust.hpp
    p.sigma = 0.02;
    return p;
}

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

// --- evidence_plausible / refuted_llr / smear_llr (task 7 items 5-6) ---

void test_evidence_plausible_grazing_sightline_is_not_refuted() {
    const TrustParams p = params();
    const double margin = evidence_margin(p.sigma);
    // Strict visible() blocks this sightline (perpendicular offset 0.20 m < CBF radius 0.30 m).
    // The margin-widened check must NOT refute it: refuting a sightline this close to the edge
    // convicts an honest reporter whose own claimed vantage point is only approximately where it
    // actually was. This is the case that discriminates a shrunk-radius implementation from an
    // "inflate the radius" one -- inflating would refute this even harder, not less.
    assert(!visible({5.0, 0.20}, {8.0, 0.20}, pegs(), 4.0));
    assert(evidence_plausible({5.0, 0.20}, {8.0, 0.20}, pegs(), 4.0, margin));
}

void test_evidence_plausible_still_refutes_straight_through_the_pile() {
    const TrustParams p = params();
    const double margin = evidence_margin(p.sigma);
    // Dead center: no plausible margin should ever rescue a sightline straight through a post.
    assert(!evidence_plausible({5.0, 0.0}, {8.0, 0.0}, pegs(), 4.0, margin));
}

void test_evidence_plausible_range_margin_forgives_the_boundary() {
    const TrustParams p = params();
    const double margin = evidence_margin(p.sigma);
    assert(!visible({0.0, 0.0}, {4.1, 0.0}, {}, 4.0));
    assert(evidence_plausible({0.0, 0.0}, {4.1, 0.0}, {}, 4.0, margin));
    // Far outside even the widened range must still be refuted.
    assert(!evidence_plausible({0.0, 0.0}, {10.0, 0.0}, {}, 4.0, margin));
}

void test_refuted_llr_is_small_negative_and_below_full_step() {
    const TrustParams p = params();
    const double l = refuted_llr(p);
    assert(l < 0.0);
    // Well below a full evidence step: geometry alone is a categorical signal, not a
    // measurement, and must not by itself approach what one real residual can do.
    assert(std::abs(l) < p.clamp_step * 0.5);
}

void test_smear_llr_no_check_contributes_nothing() {
    const TrustParams p = params();
    assert(smear_llr(/*have_check=*/false, {5.0, 0.0}, {0.0, 0.0}, p) == 0.0);
}

void test_smear_llr_uses_the_receivers_own_observation_not_the_targets_claim() {
    const TrustParams p = params();
    // Item 5: the residual is between the RELAYED report and what *I* (the receiver) saw of the
    // target myself -- not against the target's own claim. A big disagreement here is evidence
    // the REPORTER fabricated the sighting, computed with the exact same trust_llr used for
    // first-hand evidence (same rule, different operands).
    const double l = smear_llr(true, Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(0.0, 0.0), p);
    const double expect = trust_llr(1.0, p.sigma, p.d_lie, p.clamp_step, p.credit_ratio);
    assert(std::abs(l - expect) < 1e-12);
    assert(l < 0.0);   // 1.0 m residual is well past d_lie/2 = 0.15: this is a smear, not noise
}

// --- observation channel: interp_at, obs_noise (spec 5.1) ---

void test_interp_brackets_correctly() {
    const std::deque<ObsSample> buf{{0.0, {0.0, 0.0}}, {1.0, {2.0, 4.0}}};
    const auto v = interp_at(buf, 0.5);
    assert(v.has_value());
    assert(std::abs((*v)[0] - 1.0) < 1e-12);
    assert(std::abs((*v)[1] - 2.0) < 1e-12);
}

void test_interp_hits_a_sample_exactly() {
    const std::deque<ObsSample> buf{{1.0, {3.0, 4.0}}, {2.0, {5.0, 6.0}}};
    const auto v = interp_at(buf, 2.0);
    assert(v.has_value());
    assert(std::abs((*v)[0] - 5.0) < 1e-12);
    assert(std::abs((*v)[1] - 6.0) < 1e-12);
}

void test_interp_returns_nullopt_below_the_window() {
    const std::deque<ObsSample> buf{{1.0, {0.0, 0.0}}, {2.0, {1.0, 1.0}}};
    assert(!interp_at(buf, 0.5).has_value());
}

void test_interp_returns_nullopt_above_the_window() {
    // The property the rev-2 audit added: an earlier draft of this plan returned the newest
    // sample here, which INVENTS an observation. If the sensor stream stalls (dropped topic,
    // occlusion, network hiccup) while the peer's claim keeps moving, extrapolating the last
    // truth forward makes the residual against that claim grow with the silence and would
    // convict an honest, merely-unobserved peer.
    const std::deque<ObsSample> buf{{1.0, {0.0, 0.0}}, {2.0, {1.0, 1.0}}};
    assert(!interp_at(buf, 2.5).has_value());
}

void test_interp_empty_buffer_is_no_evidence() {
    const std::deque<ObsSample> buf;
    assert(!interp_at(buf, 0.0).has_value());
}

void test_interp_single_sample_is_only_valid_at_its_own_time() {
    const std::deque<ObsSample> buf{{3.0, {7.0, 8.0}}};
    const auto v = interp_at(buf, 3.0);
    assert(v.has_value());
    assert(std::abs((*v)[0] - 7.0) < 1e-12);
    assert(!interp_at(buf, 3.1).has_value());
    assert(!interp_at(buf, 2.9).has_value());
}

void test_obs_noise_is_deterministic_in_its_inputs() {
    const Eigen::Vector2d a = obs_noise(5, 1, 2, 0.02, 7);
    const Eigen::Vector2d b = obs_noise(5, 1, 2, 0.02, 7);
    assert((a - b).norm() < 1e-15);   // same (slot, observer, target, seed) -> same draw
}

void test_obs_noise_changes_with_seed() {
    // The seed must vary per run (obs_noise_seed_ parameter). A seed fixed at compile time would
    // give every run of one experiment arm the same noise realisation, making a measured
    // false-positive rate n=1 in the one dimension the measurement is supposed to cover.
    const Eigen::Vector2d a = obs_noise(5, 1, 2, 0.02, 7);
    const Eigen::Vector2d b = obs_noise(5, 1, 2, 0.02, 8);
    assert((a - b).norm() > 1e-9);
}

void test_obs_noise_changes_with_slot_and_target() {
    // Not a sequential PRNG draw: a real generator advanced once per visible peer would consume a
    // different number of samples depending on how many peers happen to be visible this slot, so
    // two runs of the identical script would diverge for a reason unrelated to what changed. A
    // pure function of the identifying tuple cannot do that -- checked here by varying each of
    // slot and target independently and confirming the draw actually moves.
    const Eigen::Vector2d base = obs_noise(5, 1, 2, 0.02, 7);
    assert((obs_noise(6, 1, 2, 0.02, 7) - base).norm() > 1e-9);
    assert((obs_noise(5, 1, 3, 0.02, 7) - base).norm() > 1e-9);
}

void test_llr_zero_crossing_at_half_the_lie() {
    const TrustParams p = params();
    // A residual of exactly d_lie/2 is neutral: below it evidence supports honesty, above it
    // supports lying. Anything else would put the decision boundary somewhere undocumented.
    assert(std::abs(trust_llr(p.d_lie / 2.0, p.sigma, p.d_lie, p.clamp_step, p.credit_ratio)) < 1e-12);
    assert(trust_llr(0.0, p.sigma, p.d_lie, p.clamp_step, p.credit_ratio) > 0.0);
    assert(trust_llr(p.d_lie, p.sigma, p.d_lie, p.clamp_step, p.credit_ratio) < 0.0);
}

void test_llr_is_clamped() {
    const TrustParams p = params();
    assert(std::abs(trust_llr(50.0, p.sigma, p.d_lie, p.clamp_step, p.credit_ratio)) <= p.clamp_step + 1e-12);
    assert(std::abs(trust_llr(0.0, p.sigma, p.d_lie, p.clamp_step, p.credit_ratio)) <= p.clamp_step + 1e-12);
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
    printf("test_first_hand_evidence_can_evict: slots=%d final_L=%g fenced=%d\n", slots, L, trust_fences_peer(L, p));
    assert(trust_fences_peer(L, p));
    assert(slots <= 10);            // 10 slots = 1.0 s; report the real number in the run log
}

void test_one_relayer_alone_can_never_evict() {
    const TrustParams p = params();
    double C = 0.0;
    for (int i = 0; i < 1000; ++i) C = trust_step_relay(C, -p.clamp_step, p.l_max, p);
    std::map<int, double> relay{{2, C}};
    assert(!trust_fences_peer(trust_total(0.0, relay, p), p));   // hearsay alone is not a verdict
}

void test_hearsay_cannot_reach_the_eviction_threshold() {
    const TrustParams p = params();
    // The invariant that makes a lone accuser harmless, asserted directly rather than through a
    // particular arithmetic: one relayer's total contribution is capped at l_max, so as long as
    // the ceiling sits above the eviction threshold, no amount of hearsay from one source gets
    // there. Constants may be re-derived; this relationship may not be broken.
    assert(p.l_max < -p.l_evict);
}

void test_relay_sum_capped_not_per_source_at_n5() {
    const TrustParams p = params();
    // Task 7 item 4 / step 3: at N=5, THREE peers can each independently corroborate a lie about
    // one target. Each relayer alone converges to (near) l_max via trust_step_relay's own
    // per-source cap -- exactly the "one relayer alone can never evict" property proven above.
    // The bug this test pins: summing three such per-source-capped contributions WITHOUT capping
    // the TOTAL puts roughly 3*l_max of pure hearsay behind one verdict, which clears l_evict
    // with no first-hand evidence at all and breaks "relayed evidence alone never evicts".
    std::map<int, double> relay;
    for (int src : {2, 3, 4}) {
        double C = 0.0;
        for (int i = 0; i < 1000; ++i) C = trust_step_relay(C, -p.clamp_step, p.l_max, p);
        relay[src] = C;
    }
    double raw_sum = 0.0;
    for (const auto& kv : relay) raw_sum += kv.second;
    printf("test_relay_sum_capped_not_per_source_at_n5: raw_sum=%.2f l_evict=%.2f capped_total=%.2f\n",
           raw_sum, p.l_evict, trust_total(0.0, relay, p));
    // Pins that this scenario actually exercises the failure mode it claims to: the UNCLAMPED
    // sum of 3 relayers crosses the eviction threshold on its own, so a test that passed here
    // even under the old (per-source-only) code would be too weak to mean anything.
    assert(raw_sum < p.l_evict);
    assert(!trust_fences_peer(trust_total(0.0, relay, p), p)
           && "hearsay from 3 relayers alone must not evict, even at N=5");
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
    for (double L = 0.0; !trust_fences_peer(trust_total(L, relay, p), p) && with_help < 200; ++with_help)
        L = trust_step_self(L, -p.clamp_step, p);
    printf("test_first_hand_evidence_still_convicts_with_a_relayer_agreeing: solo=%d with_help=%d ratio=%.1f\n", solo, with_help, solo > 0 ? (double)solo / with_help : 0);
    assert(with_help <= solo);
    assert(with_help < 200);
}

void test_an_untrusted_relayer_carries_no_weight() {
    const TrustParams p = params();
    double C = 0.0;
    for (int i = 0; i < 1000; ++i) C = trust_step_relay(C, -p.clamp_step, p.l_evict, p);
    assert(std::abs(C) < 1e-9);     // a peer we have already fenced cannot smear anyone
}

void test_trust_decays_toward_neutral_only_when_positive() {
    const TrustParams p = params();
    // Task 7 review round 3: "L > 0 decays as before" -- a peer with net TRUST that goes
    // unobserved must still lose that trust over time (any peer can be compromised at any
    // moment), so this half of the old symmetric test survives, just no longer starting negative.
    double L = 4.0;
    for (int i = 0; i < 500; ++i) L = trust_step_self(L, 0.0, p);
    assert(std::abs(L) < 1e-3);
}

void test_suspicion_does_not_decay_without_evidence() {
    const TrustParams p = params();
    // THE TEST THAT WOULD HAVE CAUGHT THE DEFECT. A peer with net SUSPICION (L<0) that goes
    // unobserved must NOT drift back toward neutral: "not visible -> no evidence"
    // (trust_step_observed) has to mean exactly zero evidence, not a decay term that quietly
    // launders suspicion merely because time passed. 50 slots = 5 s hidden (the review's example),
    // long enough that the OLD symmetric rule would have erased ~80% of 6 units of suspicion.
    const double L0 = -6.0;
    double L = L0;
    for (int i = 0; i < 50; ++i) L = trust_step_self(L, 0.0, p);
    assert(L == L0);   // exactly unchanged, not merely "still suspicious"
}

void test_trust_never_diverges_below_the_floor() {
    const TrustParams p = params();
    // log_only mode never actually blocks a peer, so trust_step_self keeps being called past
    // l_evict under sustained adverse evidence -- without a floor, L diverges without bound and
    // recovery time becomes unbounded too.
    double L = 0.0;
    for (int i = 0; i < 1000; ++i) L = trust_step_self(L, -p.clamp_step, p);
    assert(L >= p.l_evict - p.clamp_step - 1e-12);
    assert(L <= p.l_evict - p.clamp_step + 1e-9);   // and it actually REACHES the floor, not just respects it
}

void test_trust_recovers_from_the_floor_in_a_bounded_number_of_slots() {
    const TrustParams p = params();
    // Rehabilitation must stay reachable: no amount of past lying may make recovery impossible,
    // the mirror of "no amount of past honesty buys immunity" (test_trust_has_a_ceiling). From
    // the floor, sustained visible HONEST behaviour (the r=0 credit, clamp_step*credit_ratio per
    // slot) must reach neutral in a small, bounded number of slots -- reported here rather than
    // just bounded, so the number can be quoted instead of recomputed.
    double L = p.l_evict - p.clamp_step;   // the floor itself
    const double credit = p.clamp_step * p.credit_ratio;   // the r=0 honest-evidence llr
    int slots = 0;
    while (L < 0.0 && slots < 200) { L = trust_step_self(L, credit, p); ++slots; }
    printf("test_trust_recovers_from_the_floor_in_a_bounded_number_of_slots: %d slots (%.1f s) "
           "from floor=%.2f to L=%.3f\n", slots, slots * TS, p.l_evict - p.clamp_step, L);
    assert(L >= 0.0);
    assert(slots == 23);   // exact, derived arithmetic: ceil(11.2 / 0.5) = 23 at the defaults
}

void test_neutral_belief_does_not_fence_the_spawn_formation() {
    const TrustParams p = params();
    // Every peer starts neutral. If neutral fenced anyone, three dogs spawning 1.40 m apart
    // would be born violating their own constraint — the one state a hard constraint never
    // recovers from.
    assert(!trust_fences_peer(0.0, p));
    assert(!trust_fences_peer(p.l_evict + 1e-9, p));
    assert(trust_fences_peer(p.l_evict - 1e-9, p));
}

void test_detection_fits_the_safety_margin() {
    const TrustParams p = params();
    // The physics, not a slot count. A lie growing at a constant divergence rate produces NO
    // negative evidence until the residual passes d_lie/2 -- below that trust_llr is still
    // positive, pushing belief UP, not down. So by the time the fence appears the lie has already
    // spent d_lie/2 (the undetectable phase) PLUS (slots+1)*TS*v_div (the detect-and-evict phase),
    // and that SUM must fit inside D_MIN 1.3 minus the 0.867 m contact line = 0.433 m. Raising the
    // assert to make this pass is forbidden: it would ship a detector slower than the damage it
    // prevents.
    int slots = 0;
    for (double L = p.l_max; !trust_fences_peer(L, p) && slots < 200; ++slots)
        L = trust_step_self(L, -p.clamp_step, p);
    const double v_div_covered = (0.433 - p.d_lie / 2.0) / ((slots + 1) * TS);
    std::printf("detection: %d slots, covers divergence up to %.3f m/s\n", slots, v_div_covered);
    assert(v_div_covered >= 0.25);
}

// --- conservative anchoring (spec 4.1): safety that does not wait for a verdict ---

// Sensor-derived, not tuned: 3 sigma of the observation channel, and one slot of robot travel.
constexpr double kDead = 3.0 * 0.02, kRate = MAX_VX * TS;   // 0.06 m, 0.055 m/slot

void test_offset_pulls_a_distant_claim_back_to_the_observation() {
    // A peer claiming to be further away than it is must be relocated onto what we see.
    // The rate limit is deliberately slack here so this test measures the CORRECTION; how fast it
    // may be applied is a separate property (see the ramp test below). A binding rate limit and an
    // exact one-metre answer cannot both be asserted in a single call.
    const Eigen::Vector2d self(0, 0), claimed(3.0, 0), observed(2.0, 0);
    const Eigen::Vector2d d = conservative_offset(self, claimed, observed, kDead, 2.0, {0, 0});
    assert(std::abs(d[0] + 1.0) < 1e-9);   // claim moves 1 m closer, onto the truth
    assert(std::abs(d[1]) < 1e-12);
}

void test_offset_ignores_a_claim_that_is_already_closer() {
    // A peer claiming to be nearer than we see is the ghost case: keep the claim, give it room.
    const Eigen::Vector2d self(0, 0), claimed(2.0, 0), observed(3.0, 0);
    assert(conservative_offset(self, claimed, observed, kDead, kRate, {0, 0}).norm() < 1e-12);
}

void test_offset_has_a_dead_band() {
    // Measurement noise must not jitter a safety constraint.
    const Eigen::Vector2d self(0, 0), claimed(2.0, 0), observed(1.97, 0);
    assert(conservative_offset(self, claimed, observed, kDead, kRate, {0, 0}).norm() < 1e-12);
}

void test_offset_is_rate_limited() {
    // Coming out of occlusion the offset can step. An unlimited step tightens a hard constraint
    // instantly and can make the QP infeasible — the born-violated failure this project knows.
    const Eigen::Vector2d self(0, 0), claimed(5.0, 0), observed(1.0, 0);
    const Eigen::Vector2d d = conservative_offset(self, claimed, observed, kDead, kRate, {0, 0});
    assert(d.norm() <= kRate + 1e-12);
    assert(d[0] < 0.0);             // rate limited, but still in the direction of the truth
}

void test_offset_ramps_onto_the_truth_and_stops_there() {
    // The rate limit is a delay, not a ceiling: a persistent lie is fully corrected within
    // |lie| / rate slots, and the correction then holds instead of drifting past the truth.
    const Eigen::Vector2d self(0, 0), claimed(3.0, 0), observed(2.0, 0);
    Eigen::Vector2d d(0, 0);
    const int need = static_cast<int>(std::ceil(1.0 / kRate));
    for (int i = 0; i < need; ++i)
        d = conservative_offset(self, claimed, observed, kDead, kRate, d);
    printf("test_offset_ramps_onto_the_truth_and_stops_there: %d slots (%.2f s) for 1.00 m\n",
           need, need * TS);
    assert(std::abs(d[0] + 1.0) < 1e-9);
    for (int i = 0; i < 5; ++i)
        d = conservative_offset(self, claimed, observed, kDead, kRate, d);
    assert(std::abs(d[0] + 1.0) < 1e-9);   // no overshoot once it is on the observation
}

void test_anchor_offset_holds_through_a_visibility_flip() {
    // Task 7, review round 2 (a new risk found in the finding-6 fix): updatePeerOffsets must
    // HOLD the last computed offset when a peer becomes unobservable, not drop it to zero.
    // Dropping to zero would let an attacker regain full trust in its raw self-reported claim at
    // exactly the moment (stepping behind a pillar, or beyond range) it has the most incentive
    // to lie -- reopening the judge/constraint asymmetry finding 6 existed to close, on the
    // constraint side. The node has no unit test harness, so this reproduces
    // updatePeerOffsets' exact per-cycle rule directly: off[j] defaults to peer_offset_prev_[j]
    // every cycle, and is overwritten only on a fresh, visible observation.
    const Eigen::Vector2d self(0, 0), claimed(3.0, 0), observed(2.0, 0);   // peer lies by 1 m
    const std::vector<Obstacle> pillar_between{{Eigen::Vector2d(1.5, 0.0), 0.30}};  // blocks the sightline
    Eigen::Vector2d prev(0, 0), off(0, 0);

    // Cycle 1: visible -> a real correction goes into effect (same math as the ramp test above).
    assert(visible(self, observed, {}, 4.0));
    off = conservative_offset(self, claimed, observed, kDead, kRate, prev);
    prev = off;   // updatePeerOffsets' "prev = off[j]" on the fresh path
    const double held_norm = off.norm();
    assert(held_norm > 0.0);

    // Cycle 2: the SAME peer, same claim, same truth -- but a pillar now sits on the sightline.
    // updatePeerOffsets' rule for "not visible" is off[j] = peer_offset_prev_[j] (hold), never
    // touching prev itself.
    assert(!visible(self, observed, pillar_between, 4.0));
    off = prev;
    assert(std::abs(off.norm() - held_norm) < 1e-12);   // THE property under test: it did not vanish
    assert(off.norm() > 0.0);

    // Cycle 3: visible again, peer still lying by the same amount -> ramps FORWARD from where it
    // held, never resets to zero and re-ramps from scratch.
    assert(visible(self, observed, {}, 4.0));
    const Eigen::Vector2d next = conservative_offset(self, claimed, observed, kDead, kRate, prev);
    assert(next.norm() >= held_norm - 1e-12);
}

void test_offset_abstains_on_non_finite_input() {
    const double nan = std::nan("");
    const Eigen::Vector2d self(0, 0), claimed(3.0, 0);
    assert(conservative_offset(self, claimed, {nan, 0.0}, kDead, kRate, {0, 0}).norm() < 1e-12);
}

void test_anchor_translates_the_whole_trajectory() {
    // The property the whole design rests on: every point moves by the same vector, so the
    // differences the HOCBF actually reads are untouched and only the location changes.
    const Eigen::Vector2d d(-0.42, 0.13);
    Eigen::Vector4d xnow(3.0, 1.0, 0.4, -0.1);
    Eigen::VectorXd xibar = Eigen::VectorXd::Zero(XI_DIM);
    for (int k = 1; k <= N; ++k) {       // a moving plan, not a standing one
        xibar[px_index(k)] = 3.0 + 0.05 * k;
        xibar[py_index(k)] = 1.0 - 0.01 * k * k;
        xibar[vx_index(k)] = 0.5;
        xibar[vy_index(k)] = -0.02 * k;
    }
    for (int k = 0; k < N; ++k) { xibar[ax_index(k)] = 0.1; xibar[ay_index(k)] = -0.02; }
    const Eigen::VectorXd before = xibar;
    anchor_peer(d, xnow, xibar);

    assert((xnow.head<2>() - (Eigen::Vector2d(3.0, 1.0) + d)).norm() < 1e-12);
    assert((xnow.tail<2>() - Eigen::Vector2d(0.4, -0.1)).norm() < 1e-12);  // velocity untouched
    for (int k = 1; k <= N; ++k) {
        assert(std::abs(xibar[px_index(k)] - (before[px_index(k)] + d[0])) < 1e-12);
        assert(std::abs(xibar[py_index(k)] - (before[py_index(k)] + d[1])) < 1e-12);
        assert(xibar[vx_index(k)] == before[vx_index(k)]);
        assert(xibar[vy_index(k)] == before[vy_index(k)]);
    }
    for (int k = 0; k < N; ++k) {        // accelerations are what the barrier constrains
        assert(xibar[ax_index(k)] == before[ax_index(k)]);
        assert(xibar[ay_index(k)] == before[ay_index(k)]);
    }
    // and therefore every consecutive difference — the second difference the HOCBF reads — is
    // preserved to rounding (exact in real arithmetic; (a+d)-(b+d) costs a ULP or two in doubles).
    // Substituting one point instead would put a step of |d| in here, twelve orders of magnitude
    // above this tolerance.
    for (int k = 1; k < N; ++k) {
        assert(std::abs((xibar[px_index(k + 1)] - xibar[px_index(k)]) -
                        (before[px_index(k + 1)] - before[px_index(k)])) < 1e-12);
        assert(std::abs((xibar[py_index(k + 1)] - xibar[py_index(k)]) -
                        (before[py_index(k + 1)] - before[py_index(k)])) < 1e-12);
    }
}

void test_anchor_ignores_a_malformed_plan() {
    Eigen::Vector4d xnow(3.0, 1.0, 0.0, 0.0);
    Eigen::VectorXd xibar = Eigen::VectorXd::Zero(4);   // wire data; never index past it
    anchor_peer({-0.4, 0.0}, xnow, xibar);
    assert(xnow[0] == 3.0);   // all or nothing: a plan we cannot translate is left alone
    assert(xibar.isZero());
}

// A scripted peer for exactly one agent. Robot 1 stands still, says so, and hands back its own
// plan as its ADMM iterate. Single-threaded on purpose: agent 2 OWNS edge (1,2) (edge_owner), so
// it solves the edge QP — and therefore builds the CBF — itself, and never waits for a z.
class ScriptedPeer : public Transport {
public:
    ScriptedPeer(int peer, const Eigen::Vector2d& claims) : peer_(peer) {
        xnow_ << claims[0], claims[1], 0.0, 0.0;
        xibar_ = Eigen::VectorXd::Zero(XI_DIM);
        for (int k = 1; k <= N; ++k) {
            xibar_[px_index(k)] = claims[0];
            xibar_[py_index(k)] = claims[1];
        }
    }
    void send_state(const AgentStateMsg&) override {}
    std::map<int, AgentStateMsg> recv_states(std::uint64_t c, const std::vector<int>&) override {
        AgentStateMsg m;
        m.cycle_id = c;
        m.robot_id = peer_;
        m.xnow = xnow_;
        m.xibar = xibar_;
        m.reset = (c == 0);      // cold-start together, or the agent spends the slot announcing
        return {{peer_, m}};
    }
    void send_xi(int, const EdgeXiMsg&) override {}
    std::map<int, EdgeXiMsg> recv_xi(const EdgeKey& e, std::uint64_t c, int it) override {
        EdgeXiMsg m;
        m.cycle_id = c;
        m.iter = it;
        m.edge = e;
        m.from_robot = peer_;
        m.xi = xibar_;
        m.lam = Eigen::VectorXd::Zero(XI_DIM);
        return {{peer_, m}};
    }
    void send_z(int, const EdgeZMsg&) override {}
    EdgeZMsg recv_z(const EdgeKey&, std::uint64_t, int) override {
        assert(false && "the agent owns its only edge; it never waits for a z");
        return {};
    }

private:
    int peer_;
    Eigen::Vector4d xnow_;
    Eigen::VectorXd xibar_;
};

// A scripted peer for an agent that does NOT own its only edge (edge_owner(1,2)==2, so this is
// used with self_id=1). recv_states succeeds normally -- peer_xnow_ gets assigned -- but recv_z
// (the non-owner's wait for the edge round's consensus copy) always times out, simulating the
// EdgeXi/EdgeZ timeout finding 1 is about: one that happens strictly AFTER peer_xnow_ is already
// fresh. take_timeouts() reports a fixed 1, standing in for the counter the real DdsTransport
// increments before throwing.
class EdgeTimeoutPeer : public Transport {
public:
    EdgeTimeoutPeer(int peer, const Eigen::Vector2d& claims) : peer_(peer) {
        xnow_ << claims[0], claims[1], 0.0, 0.0;
        xibar_ = Eigen::VectorXd::Zero(XI_DIM);
        for (int k = 1; k <= N; ++k) {
            xibar_[px_index(k)] = claims[0];
            xibar_[py_index(k)] = claims[1];
        }
    }
    void send_state(const AgentStateMsg&) override {}
    std::map<int, AgentStateMsg> recv_states(std::uint64_t c, const std::vector<int>&) override {
        AgentStateMsg m;
        m.cycle_id = c;
        m.robot_id = peer_;
        m.xnow = xnow_;
        m.xibar = xibar_;
        m.reset = (c == 0);
        return {{peer_, m}};
    }
    void send_xi(int, const EdgeXiMsg&) override {}
    std::map<int, EdgeXiMsg> recv_xi(const EdgeKey&, std::uint64_t, int) override {
        assert(false && "self does not own this edge; recv_xi is never reached");
        return {};
    }
    void send_z(int, const EdgeZMsg&) override {}
    EdgeZMsg recv_z(const EdgeKey&, std::uint64_t, int) override { throw TransportTimeout{}; }
    int take_timeouts() override { return 1; }

private:
    int peer_;
    Eigen::Vector4d xnow_;
    Eigen::VectorXd xibar_;
};

// Walk robot 2 down the x axis, straight at a stationary robot 1, and report how close its BODY
// ever gets to robot 1's TRUE position. Closed loop with perfect tracking (the body lands on the
// first knot of the plan it just solved), because that is what makes the encounter develop the way
// it does in the field: the barrier violation migrates from the soft tail of the horizon to the
// hard front as the gap closes, and the CBF stops the approach. offset = what conservative_offset
// hands AgentCore.
double closest_approach_to_truth(const Eigen::Vector2d& claimed, const Eigen::Vector2d& truth,
                                 const Eigen::Vector2d& offset, Eigen::Vector4d* claim_seen) {
    const std::vector<int> dogs = {1, 2};
    const std::vector<EdgeKey> edges = {{1, 2}};
    ScriptedPeer tp(1, claimed);
    // formation=nullptr: this gate is about the pairwise barrier, and a formation pull has nothing
    // to say about anchoring.
    AgentCore ag(2, dogs, edges, nullptr, 0.0, {}, {}, 1, &tp);
    Eigen::Vector4d xnow(0.0, 0.0, 0.0, 0.0);
    std::map<int, Eigen::Vector2d> off;
    if (offset.norm() > 0.0) off[1] = offset;

    const int CYCLES = 80;   // 8 s at v_cruise 0.30 m/s: long enough to arrive and settle
    double closest = 1e9;
    for (std::uint64_t c = 0; c < static_cast<std::uint64_t>(CYCLES); ++c) {
        Eigen::MatrixX2d wp(2, 2);          // replan from where the body is, as the node does
        wp.row(0) = xnow.head<2>().transpose();
        wp.row(1) = Eigen::RowVector2d(5.0, 0.0);
        const Eigen::MatrixXd xdes = build_reference(xnow.head<2>(), wp);
        ag.set_peer_offsets(off);
        const StepResult r = ag.step(xnow, xdes, c);
        assert(!r.hold && r.xi.allFinite());
        xnow << r.xi[px_index(1)], r.xi[py_index(1)], r.xi[vx_index(1)], r.xi[vy_index(1)];
        closest = std::min(closest, (xnow.head<2>() - truth).norm());
    }
    *claim_seen = ag.peer_xnow().at(1);
    return closest;
}

void test_anchoring_reaches_the_barrier_and_buys_back_the_lie() {
    // Robot 1 says it is at 1.80 m; it is really at 1.45 m — the same shape of lie as the 0.42 m
    // one measured against the fleet, sized so the barrier is active either way and robot 2 is
    // NOT already inside the true keep-out at t=0. Believing the claim lets robot 2 plan to within
    // D_MIN of 1.80, i.e. 0.35 m inside the keep-out around the body that is actually there.
    const Eigen::Vector2d claimed(3.00, 0.0), truth(2.60, 0.0);
    const double lie = (claimed - truth).norm();
    Eigen::Vector4d seen_plain, seen_anchored;
    Eigen::Vector4d seen_honest;
    const double plain = closest_approach_to_truth(claimed, truth, {0.0, 0.0}, &seen_plain);
    const double anchored = closest_approach_to_truth(claimed, truth, truth - claimed, &seen_anchored);
    // What the same encounter looks like when the peer simply tells the truth. This is the
    // reference the correction has to reproduce, and unlike an absolute number it does not depend
    // on how hard this stub harness happens to enforce the barrier.
    const double honest = closest_approach_to_truth(truth, truth, {0.0, 0.0}, &seen_honest);
    printf("anchoring: body's closest approach to the true body  believed=%.3f m  anchored=%.3f m"
           "  honest=%.3f m  (anchored-honest=%.2e, lie=%.2f, D_MIN=%.2f)\n",
           plain, anchored, honest, anchored - honest, lie, D_MIN);
    assert(plain < D_MIN);                     // the lie works when the claim is believed
    assert(anchored > plain + 0.5 * lie);      // most of the lie is bought back, in the QP itself
    assert(anchored >= honest - 0.05);         // the lie buys the attacker essentially nothing
    // The claim itself must survive intact: the belief layer's residual is |claim - observation|,
    // so an anchored peer_xnow() would silently drive that residual to zero and blind the detector
    // this layer is supposed to be independent of.
    assert(std::abs(seen_anchored[0] - claimed[0]) < 1e-12);
    assert(std::abs(seen_plain[0] - claimed[0]) < 1e-12);
}

void test_belief_residual_ignores_the_anchoring_offset() {
    // Task 6, item 6: the belief layer's residual is |observation - peer_xnow()|. If the
    // anchored (corrected) position leaked into peer_xnow(), that residual would read ~0 exactly
    // when a peer starts lying -- the correction would erase the evidence for the thing it is
    // correcting. Apply a FULL correcting offset (claimed -> observed exactly, the strongest case)
    // and confirm the residual computed from peer_xnow() is still the whole 1 m lie, not ~0.
    const std::vector<int> dogs = {1, 2};
    const std::vector<EdgeKey> edges = {{1, 2}};
    const Eigen::Vector2d claimed(3.0, 0.0), observed(2.0, 0.0);   // peer lies by 1 m
    ScriptedPeer tp(1, claimed);
    AgentCore ag(2, dogs, edges, nullptr, 0.0, {}, {}, 1, &tp);
    Eigen::Vector4d xnow(0.0, 0.0, 0.0, 0.0);
    Eigen::MatrixX2d wp(2, 2);
    wp.row(0) = xnow.head<2>().transpose();
    wp.row(1) = Eigen::RowVector2d(5.0, 0.0);
    const Eigen::MatrixXd xdes = build_reference(xnow.head<2>(), wp);
    std::map<int, Eigen::Vector2d> off{{1, observed - claimed}};   // the anchoring offset in full
    ag.set_peer_offsets(off);
    const StepResult r = ag.step(xnow, xdes, 0);
    assert(!r.hold && r.xi.allFinite());
    const double residual = (observed - ag.peer_xnow().at(1).head<2>()).norm();
    assert(std::abs(residual - (observed - claimed).norm()) < 1e-9);   // still 1.0, not ~0
}

void test_peer_xnow_fresh_survives_an_edge_round_timeout() {
    // Review finding 1: res.n_timeouts==0 is NOT the same condition as "peer_xnow_ was refreshed
    // this cycle". An EdgeXi/EdgeZ timeout happens strictly AFTER peer_xnow_ is assigned (right
    // after the AgentState barrier succeeds) and is caught by a handler that just breaks the ADMM
    // loop -- the cycle still returns hold=false with a genuinely fresh peer_xnow_, but
    // n_timeouts is now > 0. self=1 does not own edge (1,2) (edge_owner returns 2), so it waits
    // on recv_z every round; EdgeTimeoutPeer always times that out.
    const std::vector<int> dogs = {1, 2};
    const std::vector<EdgeKey> edges = {{1, 2}};
    EdgeTimeoutPeer tp(2, Eigen::Vector2d(3.0, 0.0));
    AgentCore ag(1, dogs, edges, nullptr, 0.0, {}, {}, 1, &tp);
    Eigen::Vector4d xnow(0.0, 0.0, 0.0, 0.0);
    Eigen::MatrixX2d wp(2, 2);
    wp.row(0) = xnow.head<2>().transpose();
    wp.row(1) = Eigen::RowVector2d(5.0, 0.0);
    const Eigen::MatrixXd xdes = build_reference(xnow.head<2>(), wp);
    const StepResult r = ag.step(xnow, xdes, 0);
    assert(!r.hold);            // an edge-round timeout ends the cycle early, not on a barrier HOLD
    assert(r.n_timeouts > 0);   // the timeout DID happen and DID get counted
    assert(r.peer_xnow_fresh);  // but peer_xnow_ was refreshed before that timeout ever occurred
}

void test_credit_ratio_scales_the_unsaturated_positive_branch() {
    const TrustParams p = params();
    // The existing llr tests all probe residuals (0, d_lie/2, 50) that saturate to +-clamp_step
    // regardless of credit_ratio, so they cannot tell a correct scaling from a dropped one. r=0.149
    // sits just inside d_lie/2=0.15, where the raw (pre-ratio) log-likelihood is a modest 0.75 --
    // comfortably inside (0, clamp_step) both before and after scaling, so nothing here is clamped.
    // Hand computed: raw l = -d_lie*(2r-d_lie)/(2*sigma^2) = -0.30*(0.298-0.30)/(2*0.02^2)
    //              = -0.30*(-0.002)/0.0008 = 0.0006/0.0008 = 0.75
    //              credited  = 0.75 * credit_ratio(0.25) = 0.1875
    // A regression that dropped the scaling would read 0.75 here; one that scaled the PENALTY
    // branch instead of the credit branch would read a negative number.
    const double got = trust_llr(0.149, p.sigma, p.d_lie, p.clamp_step, p.credit_ratio);
    assert(std::abs(got - 0.1875) < 1e-9);
}

void test_credit_ratio_applies_after_the_clamp_not_before() {
    const TrustParams p = params();
    // r=0.149 above cannot discriminate clamp-then-scale from scale-then-clamp: the raw
    // log-likelihood there (0.75) never reaches clamp_step under either order. r=0 does, and it
    // is the residual a peer reports almost every honest slot (clean-flight residual ~0.006, but
    // 0 is the clearest hand computation and the two orders diverge by 4x here, not by rounding).
    // Hand computed: raw l = -d_lie*(2*0-d_lie)/(2*sigma^2) = d_lie^2/(2*sigma^2)
    //              = 0.30^2/(2*0.02^2) = 0.09/0.0008 = 112.5
    // clamp-then-scale (correct):  clamp(112.5, -2, 2) = 2.0;  2.0 * credit_ratio(0.25) = 0.5
    // scale-then-clamp (the bug):  112.5 * 0.25 = 28.125;      clamp(28.125, -2, 2)     = 2.0
    // At this residual the bug makes credit_ratio invisible: any ratio in (0, 1] saturates to the
    // same 2.0, which is exactly the "no-op in practice" defect — every honest cycle sits deep in
    // this saturated regime (sigma is ~0.006-0.02, d_lie is 0.30), so the asymmetry that is
    // supposed to defeat a duty-cycled liar never actually fires under the buggy order.
    const double got = trust_llr(0.0, p.sigma, p.d_lie, p.clamp_step, p.credit_ratio);
    assert(std::abs(got - 0.5) < 1e-9);
}

void test_duty_cycled_lying_is_still_convicted() {
    const TrustParams p = params();
    // Symmetric credit gives an alternating attacker a fixed point near +1.0: it lies half the
    // slots, arbitrarily large, and never approaches the threshold. Credit must be smaller than
    // penalty. One constant, still one rule, no scenario branch.
    double L = 0.0;
    int slots = 0;
    for (; !trust_fences_peer(L, p) && slots < 500; ++slots)
        L = trust_step_self(L, (slots % 2) ? p.clamp_step * p.credit_ratio : -p.clamp_step, p);
    assert(trust_fences_peer(L, p));
}

// --- trust_step_observed (Task 6): the belief accumulator that replaces gate2's single-shot
// verdict for one peer, one slot. Both gating properties below (decay-only-if-unblocked, no
// evidence without a fresh claim) are exactly the two failure modes the task exists to close,
// so each gets its own test rather than being folded into a scenario that could pass by luck.

void test_trust_step_observed_blocked_peer_is_untouched() {
    const TrustParams p = params();
    // Blocking is latched (dds_transport.hpp's block_peer only ever inserts into blocked_; no
    // unblock path exists anywhere). Decaying a convicted peer's belief back toward neutral would
    // make the belief disagree with a transport that is still refusing its messages -- worse than
    // not decaying at all. A blocked peer's L must not move, no matter how damning the evidence.
    const double L = -20.0;   // already deep in convicted territory
    const double got = trust_step_observed(L, /*blocked=*/true, /*claim_fresh=*/true,
                                           /*visible=*/true,
                                           Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(0.0, 0.0),
                                           /*extra_llr=*/-5.0, p);
    assert(got == L);   // blocked overrides even a large second-hand penalty, not just first-hand
}

void test_trust_step_observed_no_fresh_claim_decays_only() {
    const TrustParams p = params();
    // "No fresh claim" is what a HOLD slot looks like from here: peer_xnow_ is frozen (only
    // assigned when this agent's own AgentState barrier actually completes) while last_seen_
    // keeps advancing on every message this process commits, so a frozen claim differenced
    // against a live observation would fabricate a residual nobody produced. That must read
    // exactly like "cannot see you": decay, no evidence -- even with an observed/claimed pair
    // that would otherwise scream "lying".
    const double L = -3.0;
    const double got = trust_step_observed(L, /*blocked=*/false, /*claim_fresh=*/false,
                                           /*visible=*/true,
                                           Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(0.0, 0.0),
                                           /*extra_llr=*/0.0, p);
    assert(std::abs(got - trust_step_self(L, 0.0, p)) < 1e-12);
}

void test_trust_step_observed_not_visible_decays_only() {
    const TrustParams p = params();
    // Task 7 item 1: a peer this agent cannot see right now -- out of range, or behind an
    // obstacle -- must produce NO evidence even with a fresh claim and a damning observed/claimed
    // pair. Abstaining only costs latency; crediting an impossible sightline would let an
    // attacker hide behind a pillar and still be judged, which the design's acceptance criteria
    // (§9.4) explicitly forbid.
    const double L = -3.0;
    const double got = trust_step_observed(L, /*blocked=*/false, /*claim_fresh=*/true,
                                           /*visible=*/false,
                                           Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(0.0, 0.0),
                                           /*extra_llr=*/0.0, p);
    assert(std::abs(got - trust_step_self(L, 0.0, p)) < 1e-12);
}

void test_trust_step_observed_fresh_claim_uses_the_residual() {
    const TrustParams p = params();
    const Eigen::Vector2d observed(0.30, 0.0), claimed(0.0, 0.0);   // r = d_lie exactly
    const double L = 0.0;
    const double got = trust_step_observed(L, /*blocked=*/false, /*claim_fresh=*/true,
                                           /*visible=*/true, observed, claimed, /*extra_llr=*/0.0, p);
    const double expect = trust_step_self(
        L, trust_llr((observed - claimed).norm(), p.sigma, p.d_lie, p.clamp_step, p.credit_ratio), p);
    assert(std::abs(got - expect) < 1e-12);
    assert(got < L);   // r = d_lie sits on the lying side of the zero crossing at d_lie/2
}

void test_trust_step_observed_combines_extra_llr_in_one_step_not_two() {
    const TrustParams p = params();
    // Review finding 3: a peer that is both a first-hand target AND a second-hand reporter this
    // slot must be decayed by lambda ONCE, not twice. Calling trust_step_self a second time would
    // give L2 = lambda*(lambda*L + first) + extra = lambda^2*L + lambda*first + extra, not the
    // single-decay lambda*L + first + extra this function computes. Hand-verify at L=+1.0 (decay
    // asymmetry, round 3, applies only when L>0 -- a negative L here would make decay a no-op
    // either way and the once/twice cases indistinguishable) with no first-hand term
    // (claim_fresh=false) so only the decay-vs-decay^2 difference is visible.
    const double L = 1.0, extra = -0.5;
    const double got = trust_step_observed(L, /*blocked=*/false, /*claim_fresh=*/false,
                                           /*visible=*/false, {0, 0}, {0, 0}, extra, p);
    const double once = p.lambda * L + extra;               // single decay, this function's contract
    const double twice = p.lambda * (p.lambda * L) + extra; // the finding-3 bug: decays L an extra time
    assert(std::abs(got - once) < 1e-12);
    assert(std::abs(got - twice) > 1e-6);   // must actually discriminate the two, not coincide
}

void test_trust_step_observed_extra_llr_applies_without_a_fresh_first_hand_check() {
    const TrustParams p = params();
    // The smear signal is about the REPORTER's credibility and must land even when this agent has
    // no first-hand look at that peer this slot (not visible, or no fresh claim) -- it is an
    // independent channel, not conditioned on the first-hand gates.
    const double L = 0.0, extra = -1.0;
    const double got = trust_step_observed(L, /*blocked=*/false, /*claim_fresh=*/false,
                                           /*visible=*/false, {5.0, 0.0}, {0.0, 0.0}, extra, p);
    const double decayed = (L > 0.0) ? p.lambda * L : L;   // decay asymmetry, round 3
    assert(std::abs(got - (decayed + extra)) < 1e-12);
}


// --- THE ACTOR / REPORTER SPLIT (2026-08-02) -------------------------------------------------
// admm_agent_node keeps two ledgers where it used to keep one: l_act_ takes the first-hand
// position residual and decides eviction, l_rep_ takes the smear check and decides how much this
// peer's relayed testimony counts. The node itself has no unit test, so what is pinned here is
// the exact trust.hpp contract the split leans on -- if any of these four change, the node's
// arithmetic silently changes with them.

// l_act_'s call passes extra_llr = 0. A smear must not move it, however blatant.
void test_split_actor_ledger_ignores_the_smear_term() {
    TrustParams p;
    const Eigen::Vector2d obs(1.0, 0.0), claim(1.0, 0.0);   // position agrees exactly
    const double honest = trust_step_observed(0.0, false, true, true, obs, claim, 0.0, p);
    // Same slot, but the peer also fabricated a sighting worth a full penalty. The ACTOR ledger
    // is called with 0.0 for extra_llr, so it must land in exactly the same place.
    const double with_smear_suppressed =
        trust_step_observed(0.0, false, true, true, obs, claim, 0.0, p);
    assert(std::abs(honest - with_smear_suppressed) < 1e-15
           && "the actor ledger must be identical whether or not the peer smeared");
    assert(honest > 0.0 && "an honest position must still earn credit");
}

// l_rep_'s call passes claim_fresh=false, so the position pair contributes nothing and only the
// smear term lands. Without this the reporter ledger would inherit position credit and a smearer
// with an honest position would never lose its voice -- the original bug, one level down.
void test_split_reporter_ledger_ignores_the_position_residual() {
    TrustParams p;
    const Eigen::Vector2d obs(0.0, 0.0), claim(5.0, 5.0);   // wildly disagreeing position
    const double no_report = trust_step_observed(0.0, false, false, true, obs, claim, 0.0, p);
    assert(std::abs(no_report - 0.0) < 1e-15
           && "with no smear evidence the reporter ledger must not move on position alone");
    const double smeared = trust_step_observed(0.0, false, false, true, obs, claim,
                                               -p.clamp_step, p);
    assert(std::abs(smeared - (-p.clamp_step)) < 1e-12
           && "the reporter ledger must take the smear term and nothing else");
}

// The point of the whole change: a convicted reporter's testimony is worth exactly nothing, not
// merely less. trust_step_relay caps at clamp(l_src, 0, l_max), so a negative reputation gives a
// cap of 0 and the accumulator cannot leave zero however many slots it runs.
void test_split_a_floored_reporter_carries_exactly_zero_testimony() {
    TrustParams p;
    const double floored = p.l_evict - p.clamp_step;        // -11.2, the reporter ledger's floor
    double C = 0.0;
    for (int slot = 0; slot < 200; ++slot)                  // a full-penalty accusation, forever
        C = trust_step_relay(C, -p.clamp_step, floored, p);
    assert(std::abs(C) < 1e-15 && "a floored reporter must contribute exactly zero, not a little");
    // And an honest reporter is untouched by the change: full weight, as before.
    double H = 0.0;
    for (int slot = 0; slot < 200; ++slot)
        H = trust_step_relay(H, -p.clamp_step, p.l_max, p);
    assert(H < -p.l_max + 1e-9 && "an honest reporter must still carry its full weight");
}

// End to end, against the numbers written down in the reviewed derivation
// (docs/superpowers/specs/2026-08-02-actor-reporter-split-derivation.md) BEFORE the change was
// made. Two scenarios decide whether the split did what it claimed:
//   position liar  -- first-hand AND relayed evidence both point down; must still convict
//   smearer        -- own position honest, so both point UP; must NOT convict, and must be mute
void test_split_end_to_end_matches_the_reviewed_derivation() {
    TrustParams p;
    auto run = [&p](double r_pos, bool smears, double* out_total, double* out_rep) {
        double l_act = 0.0, l_rep = 0.0;
        std::map<int, double> relay;
        const Eigen::Vector2d claim(0.0, 0.0);
        const Eigen::Vector2d obs(r_pos, 0.0);              // |obs - claim| == r_pos
        for (int slot = 0; slot < 60; ++slot) {
            l_act = trust_step_observed(l_act, false, true, true, obs, claim, 0.0, p);
            if (smears)
                l_rep = trust_step_observed(l_rep, false, false, true, obs, claim,
                                            -p.clamp_step, p);
            // Every other agent relays what it SAW of this peer, compared against this peer's own
            // claim -- so the relayed residual is r_pos too, not the smear.
            for (int src = 0; src < 4; ++src)
                relay[src] = trust_step_relay(relay[src],
                                              trust_llr(r_pos, p.sigma, p.d_lie, p.clamp_step,
                                                        p.credit_ratio),
                                              p.l_max, p);
        }
        *out_total = trust_total(l_act, relay, p);
        *out_rep = l_rep;
    };
    double total = 0.0, rep = 0.0;

    run(0.4243, false, &total, &rep);                       // a0/a1/a2 position-forgery arm
    assert(total < p.l_evict && "a position liar must still be convicted after the split");
    assert(std::abs(total - (-15.8)) < 1e-6 && "derivation predicted -15.800");

    run(0.02, true, &total, &rep);                          // smear arm: honest position
    assert(total > 0.0 && "a smearer honest about its own position must NOT be convicted");
    assert(std::abs(total - 9.2) < 1e-6 && "derivation predicted +9.200");
    assert(std::abs(rep - (p.l_evict - p.clamp_step)) < 1e-9
           && "and its reporter reputation must sit on the floor, -11.200");
    assert(std::abs(std::clamp(rep, 0.0, p.l_max)) < 1e-15
           && "so its testimony is capped at exactly zero");
}

// --- evict_patience (Task 6, review finding 2): a belief conviction earns shorter patience,
// but that must NOT leak to gate2 or the roster-exclusion path, which share the same transport-
// level blocked() latch and never earned the accumulator's multi-slot debounce.

void test_evict_patience_belief_block_gets_the_short_patience() {
    const int got = evict_patience(/*belief_blocked=*/true, /*blocked=*/true,
                                   /*belief_patience=*/1, /*lying_patience=*/3, /*silence_patience=*/10);
    assert(got == 1);
}

void test_evict_patience_roster_exclusion_or_gate2_keeps_the_long_patience() {
    // Same "blocked" state as a belief conviction, but NOT tagged as one -- this is exactly what
    // a gate2 block or a roster-exclusion block looks like from here. Must fall back to the
    // ORIGINAL (longer) patience, not the one meant only for a debounced belief verdict.
    const int got = evict_patience(/*belief_blocked=*/false, /*blocked=*/true,
                                   /*belief_patience=*/1, /*lying_patience=*/3, /*silence_patience=*/10);
    assert(got == 3);
}

void test_evict_patience_unblocked_peer_gets_ordinary_silence_patience() {
    const int got = evict_patience(/*belief_blocked=*/false, /*blocked=*/false,
                                   /*belief_patience=*/1, /*lying_patience=*/3, /*silence_patience=*/10);
    assert(got == 10);
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
    test_evidence_plausible_grazing_sightline_is_not_refuted();
    test_evidence_plausible_still_refutes_straight_through_the_pile();
    test_evidence_plausible_range_margin_forgives_the_boundary();
    test_refuted_llr_is_small_negative_and_below_full_step();
    test_smear_llr_no_check_contributes_nothing();
    test_smear_llr_uses_the_receivers_own_observation_not_the_targets_claim();
    test_interp_brackets_correctly();
    test_interp_hits_a_sample_exactly();
    test_interp_returns_nullopt_below_the_window();
    test_interp_returns_nullopt_above_the_window();
    test_interp_empty_buffer_is_no_evidence();
    test_interp_single_sample_is_only_valid_at_its_own_time();
    test_obs_noise_is_deterministic_in_its_inputs();
    test_obs_noise_changes_with_seed();
    test_obs_noise_changes_with_slot_and_target();
    test_llr_zero_crossing_at_half_the_lie();
    test_llr_is_clamped();
    test_trust_has_a_ceiling();
    test_first_hand_evidence_can_evict();
    test_one_relayer_alone_can_never_evict();
    test_hearsay_cannot_reach_the_eviction_threshold();
    test_relay_sum_capped_not_per_source_at_n5();
    test_first_hand_evidence_still_convicts_with_a_relayer_agreeing();
    test_an_untrusted_relayer_carries_no_weight();
    test_trust_decays_toward_neutral_only_when_positive();
    test_suspicion_does_not_decay_without_evidence();
    test_trust_never_diverges_below_the_floor();
    test_trust_recovers_from_the_floor_in_a_bounded_number_of_slots();
    test_neutral_belief_does_not_fence_the_spawn_formation();
    test_detection_fits_the_safety_margin();
    test_credit_ratio_scales_the_unsaturated_positive_branch();
    test_credit_ratio_applies_after_the_clamp_not_before();
    test_duty_cycled_lying_is_still_convicted();
    test_offset_pulls_a_distant_claim_back_to_the_observation();
    test_offset_ignores_a_claim_that_is_already_closer();
    test_offset_has_a_dead_band();
    test_offset_is_rate_limited();
    test_offset_ramps_onto_the_truth_and_stops_there();
    test_anchor_offset_holds_through_a_visibility_flip();
    test_offset_abstains_on_non_finite_input();
    test_anchor_translates_the_whole_trajectory();
    test_anchor_ignores_a_malformed_plan();
    test_anchoring_reaches_the_barrier_and_buys_back_the_lie();
    test_belief_residual_ignores_the_anchoring_offset();
    test_peer_xnow_fresh_survives_an_edge_round_timeout();
    test_trust_step_observed_blocked_peer_is_untouched();
    test_trust_step_observed_no_fresh_claim_decays_only();
    test_trust_step_observed_not_visible_decays_only();
    test_trust_step_observed_fresh_claim_uses_the_residual();
    test_trust_step_observed_combines_extra_llr_in_one_step_not_two();
    test_trust_step_observed_extra_llr_applies_without_a_fresh_first_hand_check();
    test_split_actor_ledger_ignores_the_smear_term();
    test_split_reporter_ledger_ignores_the_position_residual();
    test_split_a_floored_reporter_carries_exactly_zero_testimony();
    test_split_end_to_end_matches_the_reviewed_derivation();
    test_evict_patience_belief_block_gets_the_short_patience();
    test_evict_patience_roster_exclusion_or_gate2_keeps_the_long_patience();
    test_evict_patience_unblocked_peer_gets_ordinary_silence_patience();
    std::cout << "test_trust: OK\n";
    return 0;
}
