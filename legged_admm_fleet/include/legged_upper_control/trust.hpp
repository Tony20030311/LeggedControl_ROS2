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
        if (!o.pos.allFinite() || !std::isfinite(o.radius)) return false;  // abstain on non-finite obstacles
        const Eigen::Vector2d w = o.pos - pi;
        const double t = w.dot(d) / (len * len);   // where the foot of the perpendicular lands
        if (t <= 0.0 || t >= 1.0) continue;        // the pile is behind me or past the target
        if ((w - t * d).norm() < o.radius) return false;
    }
    return true;
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
    // Trust ceiling AND the cap on how far one relayer's word can move us. Two jobs, one number:
    // a peer that has been honest all mission must not buy undetected approach time, and
    // hearsay must never on its own reach l_evict (l_max < |l_evict| is the invariant).
    double l_max = 4.6;                     // odds 99:1
    // Wald SPRT thresholds from alpha = 1e-4 (false eviction) and beta = 0.01 (miss):
    // log(beta / (1 - alpha)) = -4.6 to re-admit, log((1 - beta) / alpha) = 9.2 to convict.
    double l_evict = -9.2;
    double l_rejoin = 0.0;                  // hysteresis: climb back to neutral before rejoining
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
// Zero at r = d_lie/2, positive below, negative above. Clamped at both ends.
inline double trust_llr(double r, double sigma, double d_lie, double clamp_step) {
    if (!(sigma > 0.0) || !std::isfinite(r)) return 0.0;
    const double l = -d_lie * (2.0 * r - d_lie) / (2.0 * sigma * sigma);
    return std::clamp(l, -clamp_step, clamp_step);
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
