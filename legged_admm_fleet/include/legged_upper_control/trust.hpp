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

#include "legged_upper_control/admm_constants.hpp"  // N, XI_DIM, px_index, MAX_VX, TS
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
        if (!o.pos.allFinite() || !std::isfinite(o.radius)) return false;  // abstain on non-finite obstacles
        const Eigen::Vector2d w = o.pos - pi;
        const double t = w.dot(d) / (len * len);   // where the foot of the perpendicular lands
        if (t <= 0.0 || t >= 1.0) continue;        // the pile is behind me or past the target
        if ((w - t * d).norm() < o.radius) return false;
    }
    return true;
}

// --- conservative anchoring (spec section 4.1) ---
//
// The collision barrier is built from what a peer SAYS about itself, so it protects the distance
// to a claim rather than to a body. A peer that misreports its position by 0.42 m closed the
// measured physical gap from 0.433 m to 0.038 m without ever moving abnormally. This corrects the
// geometry BEFORE the constraint is built, so it costs no detection delay: no belief, no
// threshold, no verdict. The belief layer then only decides who to keep cooperating with.
//
// One-sided by construction: a liar cannot make itself look FURTHER away than it is, so a claim is
// only ever pulled TOWARDS us. A claim that is already nearer than what we see is either noise or
// a ghost, and honouring it just leaves more room — the safe direction, and the reason this needs
// no judgement about who is lying.
//
//   dead_band  3 sigma of the observation channel. Below it the correction is noise, and a hard
//              constraint that jitters is worse than one that is slightly stale.
//   rate_limit how far the correction may move in one slot (MAX_VX*TS = one slot of robot travel).
//              Coming out of an occlusion the raw correction can step by metres; applying that at
//              once tightens a hard constraint instantly, which is the born-violated failure this
//              project has already paid for. So the correction RAMPS: a lie of size L is fully
//              anchored after L/rate_limit slots.
//   prev       last slot's offset for this peer. State lives with the caller, so this stays pure.
inline Eigen::Vector2d conservative_offset(const Eigen::Vector2d& self,
                                           const Eigen::Vector2d& claimed,
                                           const Eigen::Vector2d& observed,
                                           double dead_band, double rate_limit,
                                           const Eigen::Vector2d& prev) {
    if (!self.allFinite() || !claimed.allFinite() || !observed.allFinite() || !prev.allFinite())
        return Eigen::Vector2d::Zero();
    // Not "closer to us than claimed" -> no correction at all, and prev is dropped rather than
    // decayed: the anchored position is claimed + offset, so when the claim itself comes back to
    // the truth, dropping the offset keeps that anchored position continuous. Decaying it would
    // instead keep pushing an honest claim towards us for another 8 slots.
    if ((claimed - self).norm() - (observed - self).norm() <= dead_band)
        return Eigen::Vector2d::Zero();
    const Eigen::Vector2d step = (observed - claimed) - prev;
    const double n = step.norm();
    if (!(n > rate_limit) || !(n > 0.0)) return prev + step;
    return prev + step * (rate_limit / n);
}

// Apply one peer's offset to its ENTIRE broadcast: the current state and every knot of the plan.
//
// Translating the whole thing is not tidiness. The HOCBF constraint is a three-point combination
// of the barrier value at consecutive knots (rti.cpp three_point), i.e. a second difference.
// Substituting only the current point makes that difference see a jump that no motion produced —
// an inconsistency of 0.5 m demands roughly 21 m/s^2 of differential acceleration to recover,
// outside the QP's box bounds, so the solver poisons rather than avoids. A uniform translation
// leaves every difference unchanged: the implied velocity and acceleration are identical and only
// the location moves, which is exactly the claim being made ("you are actually over there").
//
// All or nothing on a short plan: wire data reaches here, and a half-translated peer would be a
// geometry nobody derived. Gate 1 already rejects wrong-length messages; this is the backstop.
inline void anchor_peer(const Eigen::Vector2d& d, Eigen::Vector4d& xnow, Eigen::VectorXd& xibar) {
    if (!d.allFinite() || xibar.size() < XI_DIM) return;
    xnow.head<2>() += d;
    for (int k = 1; k <= N; ++k) {
        xibar[px_index(k)] += d[0];
        xibar[py_index(k)] += d[1];
    }
}

// Every constant here is derived, not tuned. Changing one means redoing its derivation.
struct TrustParams {
    // exp(-TS / tau) with TS = 0.1 s and tau = 2 s: how fast evidence about a peer expires.
    // tau is the timescale on which a peer's behaviour can change (it can be compromised at any
    // moment), which is also what makes a separate stale-vote expiry rule unnecessary.
    double lambda = 0.951;
    // Cap on one observation's contribution. With sigma at the clean-flight floor the raw llr is
    // in the hundreds, so a single stumble or dropped scan would decide a verdict by itself.
    double clamp_step = 2.0;
    // Positive contributions (evidence of honesty) are scaled by this before the clamp; negative
    // ones are not. With symmetric credit an attacker that lies on alternating slots has a fixed
    // point near +1.0 — it can lie half the time, arbitrarily large, and never approach the
    // eviction threshold. One constant fixes it, still one rule, no scenario branch: penalty must
    // outweigh credit.
    double credit_ratio = 0.25;
    // Trust ceiling AND the cap on how far one relayer's word can move us. Two jobs, one number:
    // a peer that has been honest all mission must not buy undetected approach time, and
    // hearsay must never on its own reach l_evict (l_max < |l_evict| is the invariant).
    double l_max = 4.6;                     // odds 99:1
    // Wald SPRT thresholds from alpha = 1e-4 (false eviction) and beta = 0.01 (miss):
    // log(beta / (1 - alpha)) = -4.6 to re-admit, log((1 - beta) / alpha) = 9.2 to convict.
    double l_evict = -9.2;
    // No l_rejoin: rejoin is not supported by the transport (blocking is latched in dds_transport
    // with no unblock path anywhere), so a hysteresis threshold for climbing back to neutral would
    // be aspirational. Don't add one without an unblock mechanism to go with it.
    // Measured, not guessed: clean-flight residual distribution (calibration run, Task 8).
    // The simulation value is a FLOOR — real perception adds attitude lever arm, inter-robot
    // drift and surface-vs-origin bias (spec section 4).
    double sigma = 0.02;
    // Smallest lie worth catching. MUST stay under the 0.433 m safety buffer, or a lie exists
    // that is undetectable and still spends the whole margin — the arithmetic that made the old
    // 0.50 gate useless.
    double d_lie = 0.30;
};

// Log-likelihood ratio of HONEST over LYING for one residual, 2D Gaussian noise both sides:
//   log p(r|honest)/p(r|lying) = -d_lie * (2r - d_lie) / (2 sigma^2)
// Zero at r = d_lie/2, positive below, negative above. The positive (honest-looking) branch is
// scaled by credit_ratio AFTER the clamp, not before — see TrustParams::credit_ratio for why
// symmetric credit is unsafe against a duty-cycled liar.
//
// The order is load-bearing, not cosmetic. At any realistic operating residual (sigma is
// 0.006-0.02, d_lie is 0.30) the raw log-likelihood is already in the hundreds, so scale-then-
// clamp computes credit_ratio*raw and then clamps THAT to clamp_step -- for any credit_ratio in
// (0, 1] the result saturates to the same clamp_step as an unscaled penalty, and the asymmetry
// credit_ratio exists to create is invisible in exactly the regime the fleet operates in. Clamp
// first, so the credited value is clamp_step*credit_ratio, a genuinely smaller step than the
// penalty's clamp_step -- see test_credit_ratio_applies_after_the_clamp_not_before for the r=0
// hand computation (0.5 fixed vs 2.0 under the old order).
inline double trust_llr(double r, double sigma, double d_lie, double clamp_step,
                        double credit_ratio) {
    if (!(sigma > 0.0) || !std::isfinite(r)) return 0.0;
    double l = -d_lie * (2.0 * r - d_lie) / (2.0 * sigma * sigma);
    l = std::clamp(l, -clamp_step, clamp_step);
    if (l > 0.0) l *= credit_ratio;
    return l;
}

// First-hand evidence: decay, add, then cap the optimistic side at the ceiling.
inline double trust_step_self(double L, double llr, const TrustParams& p) {
    return std::min(p.lambda * L + llr, p.l_max);
}

inline double trust_prob(double L) { return 1.0 / (1.0 + std::exp(-L)); }

// Second-hand evidence, accumulated per relaying source. Weighted by how much we believe the
// relayer and capped by that same confidence: you cannot be more certain that j lied, on i's
// word, than you are that i is honest. That bound is what makes a lone accuser harmless, and it
// falls out of the likelihood being conditioned on the relayer — it is not a bolted-on rule.
// l_src is OUR log-odds for the relayer; a relayer at or below neutral carries nothing.
inline double trust_step_relay(double C, double llr, double l_src, const TrustParams& p) {
    const double cap = std::clamp(l_src, 0.0, p.l_max);
    const double w = trust_prob(l_src);
    return std::clamp(p.lambda * C + w * llr, -cap, cap);
}

inline double trust_total(double L_self, const std::map<int, double>& relay) {
    double L = L_self;
    for (const auto& kv : relay) L += kv.second;
    return L;
}

// Belief -> geometry, and the ONLY place a peer stops being treated as a cooperating agent.
// FLAT over the trusted range by construction: above the threshold a peer is fenced exactly as
// today (pairwise CBF, no corpse circle). It cannot be otherwise — the fleet spawns 1.40 m apart
// with every belief at neutral, and a circle that grew with doubt would be violated at t = 0,
// the one state a hard constraint never recovers from.
// ponytail: below the threshold this is a step (today's corpse geometry), not a ramp. A ramp
// needs a measurement nobody has taken; add it when one exists.
inline bool trust_fences_peer(double L, const TrustParams& p) { return L < p.l_evict; }

}  // namespace admm
