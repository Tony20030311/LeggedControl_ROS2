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

TrustParams params() {
    TrustParams p;              // defaults are the derived values; see trust.hpp
    p.sigma = 0.02;
    return p;
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

void test_belief_decays_toward_neutral() {
    const TrustParams p = params();
    double L = -5.0;
    for (int i = 0; i < 500; ++i) L = trust_step_self(L, 0.0, p);
    assert(std::abs(L) < 1e-3);     // stale evidence expires without an expiry rule
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
                                           Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(0.0, 0.0), p);
    assert(got == L);
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
                                           Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(0.0, 0.0), p);
    assert(std::abs(got - trust_step_self(L, 0.0, p)) < 1e-12);
}

void test_trust_step_observed_fresh_claim_uses_the_residual() {
    const TrustParams p = params();
    const Eigen::Vector2d observed(0.30, 0.0), claimed(0.0, 0.0);   // r = d_lie exactly
    const double L = 0.0;
    const double got = trust_step_observed(L, /*blocked=*/false, /*claim_fresh=*/true, observed, claimed, p);
    const double expect = trust_step_self(
        L, trust_llr((observed - claimed).norm(), p.sigma, p.d_lie, p.clamp_step, p.credit_ratio), p);
    assert(std::abs(got - expect) < 1e-12);
    assert(got < L);   // r = d_lie sits on the lying side of the zero crossing at d_lie/2
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
    test_first_hand_evidence_still_convicts_with_a_relayer_agreeing();
    test_an_untrusted_relayer_carries_no_weight();
    test_belief_decays_toward_neutral();
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
    test_offset_abstains_on_non_finite_input();
    test_anchor_translates_the_whole_trajectory();
    test_anchor_ignores_a_malformed_plan();
    test_anchoring_reaches_the_barrier_and_buys_back_the_lie();
    test_belief_residual_ignores_the_anchoring_offset();
    test_peer_xnow_fresh_survives_an_edge_round_timeout();
    test_trust_step_observed_blocked_peer_is_untouched();
    test_trust_step_observed_no_fresh_claim_decays_only();
    test_trust_step_observed_fresh_claim_uses_the_residual();
    test_evict_patience_belief_block_gets_the_short_patience();
    test_evict_patience_roster_exclusion_or_gate2_keeps_the_long_patience();
    test_evict_patience_unblocked_peer_gets_ordinary_silence_patience();
    std::cout << "test_trust: OK\n";
    return 0;
}
