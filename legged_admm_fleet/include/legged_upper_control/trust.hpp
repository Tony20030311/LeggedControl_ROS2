#pragma once
// Who to believe. Pure functions only: visibility, the log-odds accumulator that turns
// observation residuals into a belief, and the thresholds that belief crosses. Header-only
// inline, the same way majority_excluded lives in fleet_config.hpp — no node, no ROS, no state,
// so every rule here is unit-testable instead of only observable in a six-minute Gazebo run.
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
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

// --- observation channel (spec section 5.1): the input a compromised peer cannot author ---
//
// The node subscribes each peer's stand-in sensor topic and stores raw, un-noised world-frame
// samples here in time order; noise and time-alignment are applied at the point of USE (see
// obs_noise below and admm_agent_node.cpp's per-cycle offset computation), not at the point of
// storage, so the same buffer serves both a clean re-query and a noisy one without duplicating
// samples.
struct ObsSample {
    double t;              // sim time, seconds, when this sample was taken
    Eigen::Vector2d pos;
};

// Linear interpolation of a peer's observed position onto `t_query`, using the pair of samples
// in `buf` (assumed time-ordered, as the node's subscription callback appends them) that bracket
// it. `nullopt` when `t_query` falls outside [buf.front().t, buf.back().t] in EITHER direction --
// not only before the oldest sample but ALSO past the newest one.
//
// The "past the newest sample" case is the one worth spelling out: an earlier draft of this
// design returned the newest sample there, which INVENTS an observation. If the sensor stream
// stalls (dropped topic, an occlusion that never lifts, a crossed range gate) while the peer's
// claim keeps advancing, extrapolating the last-known truth forward makes the residual against
// that claim grow with the silence -- and convicts an honest, merely-unobserved peer exactly the
// way a frozen HOLD claim compared against live odom does elsewhere in this file. Before the
// window is symmetrically wrong for the same reason: absence of evidence must read as absence of
// evidence, not as "yes, still there."
inline std::optional<Eigen::Vector2d> interp_at(const std::deque<ObsSample>& buf, double t_query) {
    if (buf.empty()) return std::nullopt;
    if (t_query < buf.front().t || t_query > buf.back().t) return std::nullopt;
    for (std::size_t i = 1; i < buf.size(); ++i) {
        if (t_query <= buf[i].t) {
            const double span = buf[i].t - buf[i - 1].t;
            const double a = span > 1e-9 ? (t_query - buf[i - 1].t) / span : 0.0;
            return buf[i - 1].pos + a * (buf[i].pos - buf[i - 1].pos);
        }
    }
    return buf.back().pos;   // buf.size() == 1: front == back == t_query, already bounds-checked
}

// Hash-mix a 64-bit state (splitmix64), used below to turn (slot, observer, target, seed) into
// two independent-looking uniform draws without a stateful generator.
inline std::uint64_t obs_noise_mix(std::uint64_t x) {
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

// Sensor noise as a DETERMINISTIC function of (slot, observer, target, seed) -- not a sequential
// PRNG draw. A stateful generator advanced once per visible peer would consume a different
// number of samples depending on how many peers happen to be visible this slot, so two runs of
// the identical script would diverge for a reason that has nothing to do with what changed
// between them (the same determinism discipline that pins the OSQP version for this project).
//
// `seed` must vary PER RUN (the node's obs_noise_seed_ parameter): a seed fixed at compile time
// would give every run of one experiment arm the same noise realisation, making a measured
// false-positive rate n=1 in the one dimension a repeated-run measurement is supposed to cover.
inline Eigen::Vector2d obs_noise(std::uint64_t slot, int observer, int target, double sigma,
                                 int seed) {
    std::uint64_t h = obs_noise_mix(static_cast<std::uint64_t>(seed));
    h = obs_noise_mix(h ^ slot);
    h = obs_noise_mix(h ^ (static_cast<std::uint64_t>(static_cast<std::uint32_t>(observer)) << 32 |
                          static_cast<std::uint32_t>(target)));
    const std::uint64_t h1 = obs_noise_mix(h);
    const std::uint64_t h2 = obs_noise_mix(h1);
    // top 53 bits -> uniform in (0, 1], never exactly 0 (Box-Muller needs log(u1))
    auto to_unit = [](std::uint64_t v) {
        return (static_cast<double>(v >> 11) + 1.0) / (static_cast<double>(1ULL << 53) + 1.0);
    };
    const double u1 = to_unit(h1), u2 = to_unit(h2);
    const double r = sigma * std::sqrt(-2.0 * std::log(u1));
    const double theta = 2.0 * M_PI * u2;
    return Eigen::Vector2d(r * std::cos(theta), r * std::sin(theta));
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
    // This IS the injected noise magnitude (admm::obs_noise), not an independently measured
    // quantity -- calling it "measured" would claim more than a synthetic channel can give. The
    // clean-flight calibration run confirms the resulting residual matches Rayleigh(sigma) with
    // no extra bias (see docs/superpowers/results/2026-07-31-trust-calibration.md), which is the
    // useful check; it does not derive this number from anything independent of itself. The
    // simulation value is a FLOOR — real perception adds attitude lever arm, inter-robot drift
    // and surface-vs-origin bias (spec section 4).
    double sigma = 0.02;
    // Smallest lie worth catching. MUST stay under the 0.433 m safety buffer, or a lie exists
    // that is undetectable and still spends the whole margin — the arithmetic that made the old
    // 0.50 gate useless.
    double d_lie = 0.30;
    // Weight for a geometrically-refuted relayed report (task 7 item 6): "this sightline cannot
    // exist" is a categorical signal, not a measurement, so it must move the REPORTER's belief
    // far less than a real residual would. Smaller than credit_ratio (0.25) on purpose -- no
    // calibration run has measured how often the margin-widened check still misfires against an
    // honest reporter, so this stays conservative until one does.
    double refute_ratio = 0.1;
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

// First-hand evidence: decay, add, then clamp both sides. Task 7 review round 3: decay is NOT
// symmetric in the sign of L.
//   L > 0  (net trust)      decays toward neutral, same as before -- trust must be continually
//                           re-earned, because any peer can be compromised at any moment.
//   L <= 0 (net suspicion)  does NOT decay. "Not visible -> no evidence" (trust_step_observed
//                           passes llr=0 here) must mean exactly zero evidence, not a term that
//                           quietly moves L toward innocence merely because time passed.
//                           Decaying suspicion the same way trust decays let an attacker launder
//                           roughly 80% of 6 units of accumulated suspicion by hiding behind a
//                           pillar for about 3 seconds (30 slots at lambda=0.951) -- clearing
//                           suspicion requires evidence of honest behaviour, and evidence
//                           requires visibility; being unobservable is not proof of innocence.
//                           The same asymmetry TrustParams::credit_ratio already applies to
//                           EVIDENCE (an honest residual counts for 1/4), applied here to the
//                           PASSAGE OF TIME instead -- one rule, not a scenario branch.
// Floor at l_evict - clamp_step (one evidence step below the conviction threshold): the mirror
// of the l_max ceiling. Without it, sustained adverse evidence in log-only mode (blocking never
// actually latches there, so this keeps being called past l_evict) makes L diverge without
// bound, and an unbounded L makes recovery time unbounded too. One evidence step below the
// threshold is far enough that conviction still reads cleanly and close enough that
// rehabilitation stays reachable: recovery from the floor at the honest-credit rate (clamp_step
// * credit_ratio per slot, no decay while L<=0) takes 23 slots (2.3 s) at the derived defaults
// -- see test_trust_recovers_from_the_floor_in_a_bounded_number_of_slots.
inline double trust_step_self(double L, double llr, const TrustParams& p) {
    const double decayed = (L > 0.0) ? p.lambda * L : L;
    return std::clamp(decayed + llr, p.l_evict - p.clamp_step, p.l_max);
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

// Total belief: first-hand plus hearsay, the SUM capped (task 7 item 4), not each source.
// trust_step_relay already caps each entry at our own belief in THAT relayer, but summing
// several such per-source-capped entries unclamped lets N-1 independent accusers each
// contribute up to l_max -- at N >= 4 that alone crosses l_evict on pure hearsay, breaking
// "relayed evidence alone never evicts" for any fleet larger than 3. Clamping the TOTAL
// preserves the invariant for every N.
inline double trust_total(double L_self, const std::map<int, double>& relay, const TrustParams& p) {
    double s = 0.0;
    for (const auto& kv : relay) s += kv.second;
    return L_self + std::clamp(s, -p.l_max, p.l_max);
}

// --- second-hand evidence (spec 5.2/5.3, task 7): agents share what they SAW ---

// How far outside strict geometry a relayed report may still be believed (task 7 item 6). The
// arena is common knowledge -- its posts need no slack -- but the REPORTER's own claimed
// position, used below as its assumed vantage point, carries up to two slots of lag (its own
// broadcast delay plus the one-slot evidence-collection lag ev_slot exists to name), plus
// ordinary observation noise. Refuting tighter than this convicts an honest reporter whose
// sightline merely grazes an obstacle's edge.
inline double evidence_margin(double sigma) { return MAX_VX * 2.0 * TS + 3.0 * sigma; }

// Can this report even be true? `src` is the REPORTER's own claimed position -- never a real
// observation of the reporter, which is the whole point: broadcasting positions instead of
// residuals lets a receiver refute a fabricated report WITHOUT seeing either party. Margin-
// widened on both checks, but in OPPOSITE directions: range is relaxed outward (a report just
// past the nominal edge is not automatically impossible) while the occlusion radius is relaxed
// INWARD -- shrunk, not grown -- because refuting on the bare CBF radius means a sightline that
// merely grazes a pillar's edge is refutable by every honest bystander, on every slot, and a
// handful of those fence an honest robot into mutual evasion (the input that once produced 179
// solver failures and a frozen fleet).
inline bool evidence_plausible(const Eigen::Vector2d& src, const Eigen::Vector2d& obs_pos,
                               const std::vector<Obstacle>& obs, double range, double margin) {
    std::vector<Obstacle> lenient;
    lenient.reserve(obs.size());
    for (const auto& o : obs) lenient.push_back({o.pos, std::max(0.0, o.radius - margin)});
    return visible(src, obs_pos, lenient, range + margin);
}

// llr CONTRIBUTIONS for second-hand evidence, returned rather than applied: one reporter's
// single broadcast can carry evidence about several targets (any N > 3), and each is a separate
// contribution toward the SAME belief update for that reporter. Applying trust_step_self once
// per contribution would decay L by lambda once per TARGET instead of once per SLOT; the caller
// accumulates every contribution for one reporter across its whole message, clamps the sum
// exactly like a first-hand llr, and calls trust_step_self exactly once.

// A relayed report geometry rules out (task 7 item 6). A categorical "this cannot be true"
// signal, not a residual, so it is weighted far below a full evidence step (see
// TrustParams::refute_ratio).
inline double refuted_llr(const TrustParams& p) { return -p.clamp_step * p.refute_ratio; }

// task 7 item 5: the smear check. `have_check` is whether THIS agent independently observed the
// TARGET at the exact slot the report describes (visible, and the observation buffer actually
// covers that instant) -- without one there is nothing to compare against, and this must read
// exactly like every other "no evidence" gate in this file: decay, no term, no exception. With
// one, j's true position at that instant is exactly what was just measured, so disagreement with
// the relayed report is evidence the REPORTER fabricated or misjudged its sighting -- not
// evidence against j. This is the only mechanism that makes a false report about an innocent
// peer cost the liar anything; without it the target's belief drops and the reporter pays
// nothing.
inline double smear_llr(bool have_check, const Eigen::Vector2d& reported,
                        const Eigen::Vector2d& self_observed, const TrustParams& p) {
    if (!have_check) return 0.0;
    return trust_llr((reported - self_observed).norm(), p.sigma, p.d_lie, p.clamp_step, p.credit_ratio);
}

// One peer, one slot: the belief accumulator that replaces gate2's single-shot verdict (task 6).
// Two gates, in order:
//   blocked      Blocking is latched (dds_transport.hpp's block_peer only ever inserts into
//                blocked_; nothing anywhere erases it). Decaying a convicted peer's belief back
//                toward neutral would make the belief disagree with a transport that is still
//                refusing its messages -- worse than not decaying. So L is left untouched, not
//                just un-decayed: this is the ONLY branch that skips trust_step_self entirely.
//   claim_fresh  peer_xnow() (agent_core.cpp) is assigned exactly once per cycle, right after
//                this agent's own AgentState barrier completes, and stays frozen on a barrier-
//                miss HOLD. last_seen_ (dds_transport.hpp) advances on every message this process
//                commits regardless of whether THIS agent's own barrier for the current slot ever
//                finished, so a peer can look fresh by last_seen_ while its cached claim is
//                actually several slots stale. Differencing that frozen claim against a live
//                observation fabricates a residual nobody produced -- "no fresh claim" must read
//                exactly like "cannot see you": decay, no evidence. The caller (admm_agent_node's
//                beliefStep) decides claim_fresh from whether its OWN step() actually refreshed
//                peer_xnow() this slot, not from last_seen_ alone.
// `observed`/`claimed` must both be the RAW, un-anchored values (AgentCore::peer_xnow(), never
// the conservative-anchoring-corrected position) -- see test_belief_residual_ignores_the_
// anchoring_offset for the property this depends on.
//
// `visible_peer` (task 7 item 1): admm::visible() computed by the caller on the RAW ground-truth
// observation, before noise -- occlusion and range are properties of where the body actually is,
// not of the noisy sample. A peer we cannot see produces NO evidence, exactly like a stale claim:
// abstaining costs latency, crediting an impossible sightline would let an attacker hide behind a
// pillar and still be judged, which the design's acceptance criteria explicitly forbid.
//
// `extra_llr` (task 7 review finding 3): a peer is BOTH a first-hand target here AND, in the same
// slot, potentially a second-hand REPORTER whose own credibility just moved (smear_llr /
// refuted_llr, folded together by the caller into one clamped number per reporter per slot).
// Calling trust_step_self a second time for the same peer in the same slot would decay L by
// lambda TWICE (tau effectively halves), so the caller must combine both signals into one llr
// and land here exactly once. 0.0 when the peer generated no second-hand evidence this slot.
inline double trust_step_observed(double L, bool blocked, bool claim_fresh, bool visible_peer,
                                  const Eigen::Vector2d& observed, const Eigen::Vector2d& claimed,
                                  double extra_llr, const TrustParams& p) {
    if (blocked) return L;
    double llr = extra_llr;
    if (claim_fresh && visible_peer)
        llr += trust_llr((observed - claimed).norm(), p.sigma, p.d_lie, p.clamp_step, p.credit_ratio);
    return trust_step_self(L, llr, p);
}

// How many silent slots a peer earns before eviction (task 6, finding 2). A belief conviction
// gets the shortest patience: the accumulator already spent slots deliberating (that IS the
// debounce evict_after_lying's patience exists to provide elsewhere), so stacking more patience
// on top just hands an already-convicted liar extra free approach time. Everything else that
// reaches blocked() -- gate2's odom residual AND the roster-exclusion path in dds_transport.hpp,
// which latches on the FIRST message carrying an excluding roster with none of that debounce --
// keeps the original, longer patience; shortening it too would propagate one bad signal (a
// mistaken roster, or a single-shot gate2 residual bump) to a healthy third party within one
// silent slot instead of three, exactly the failure this codebase has already recorded once
// (two healthy agents evicting each other after one bad signal). Not blocked at all is ordinary
// silence, which gets the longest patience of the three (it might just be a network hiccup).
inline int evict_patience(bool belief_blocked, bool blocked, int belief_patience,
                          int lying_patience, int silence_patience) {
    if (belief_blocked) return belief_patience;
    if (blocked) return lying_patience;
    return silence_patience;
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
