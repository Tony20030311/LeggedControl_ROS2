# Observation-sourced belief layer — experiment matrix results

Plan: `docs/superpowers/plans/2026-07-31-observation-belief-trust-rev2.md`, Task 12.
Design: `docs/superpowers/specs/2026-07-31-observation-belief-trust-design.md`.
Calibration this builds on: `2026-07-31-trust-calibration.md`.

**Status: IN PROGRESS.** Sections marked ⏳ have no data yet and must not be cited.

---

## 0. How to read this document

Three rules, each of which this project has previously got wrong at least once:

1. **Contact is `phys_gap_logger.py`, never centre distance.** Centre-to-centre reads ~0.68 m
   high — three undisturbed dogs are 1.40 m apart by centre and 0.717 m by surface — so a run
   whose centres never dropped below 0.90 m can still have had bodies 0.07 m apart. Every
   "did they touch" number below is a surface-to-surface gap using each dog's real yaw and the
   0.83 × 0.25 m base box. Centre distances appear only where labelled `[proxy]`.
2. **Every distance is a minimum over every logged row**, never a sampled poll. A plum run once
   sat at 0.526 m for 4.3 s and still printed PASS because the poll interval stepped over it.
3. **Every attempt is in the table, including the aborted ones.** Exclusions were pre-registered
   (spec §9 protocol item 7) as infrastructure failure only. `d_run.sh` exit 2 = infrastructure,
   exit 1 = the experiment ran and produced that outcome.

`n = 3` per arm. **No significance is claimed anywhere in this document.** The numbers are
descriptive.

---

## 1. What each arm is, and what the attacker can do

| arm | detector | attacker |
|---|---|---|
| `a0` | none (own verdict computed and logged, never acted on) | dual-channel position forgery |
| `a1` | `gate2()` single-shot EMA odom residual (the pre-existing detector) | dual-channel position forgery |
| `a2` | observation belief layer (this spec) | dual-channel position forgery |
| `smear` | a2 | own position honest; fabricates a sighting of an honest peer |
| `duty` | a2 | position forgery on a 50 % duty cycle |
| `forged_obs` | a2 | position forgery **and** impersonation of a peer's observation channel |

**Dual-channel forgery** means the attacker's broadcast claim *and* every survivor's copy of the
attacker's `/controller/odom` are offset by the same amount, so the two agree. This is the
threat model the design targets: an attacker that owns its estimator as well as its planner
leaves `gate2()` no residual to find, because `gate2()` differences the claim against exactly
that estimator's output. `a2` reads `observed/robotJ` instead — remapped to the Gazebo plugin's
world pose, a channel the compromised process does not author.

Both halves are switched on **in one dispatch** (`scripts/arm_attack.py`), not in sequence. This
is not a detail: with three separate `ros2 param set` calls the halves landed 2.78 s apart and a
survivor blocked the attacker 0.61 s into that single-channel window, so the arm measured the
harness rather than the detector (run `d_0801_035833`, rejected on those grounds and listed in
§9). The achieved arming skew is reported for every run and gated at 0.20 s.

`LIE = 0.30 m`, applied to x and y, i.e. a claim displacement of **0.4243 m** — just inside the
0.433 m at which bodies touch (spec §2). Chosen because it is the largest lie that is not
trivially a collision, and because this project has previously measured a *stationary* lie of
this size eating the body gap down to 0.038 m.

---

## 2. Design envelope, and what the injection actually is

Spec §2 covers divergence up to **`v_div` ≤ 0.31 m/s**, from `0.15 + 0.9·v_div ≤ 0.433`.

⚠️ **The harness injects a STEP, not a ramp.** `inject_fake_offset` goes from 0 to 0.30 m between
one broadcast and the next, so the instantaneous divergence rate is unbounded and the envelope
above — derived for a claim that drifts away from truth at a finite rate — **does not apply to
these runs as a pass/fail criterion**. `v_div` is reported per run as a difference quotient
across the step, labelled as such, and the honest statement is that the step case is *outside*
the envelope the detection budget was sized for.

What protects the fleet in the step case is not the detection budget but **conservative
anchoring** (spec §4.1), which does not depend on detection latency at all. That separation is
the point of reporting both.

**Measured.** Two numbers, because the obvious one is unusable on its own:

| | value | what it means |
|---|---|---|
| `v_div_max` on a **clean run with no attacker** | **0.726 m/s** | the estimator's **noise floor**. Consecutive residuals are independent Rayleigh(σ=0.02) draws one slot apart, so the largest difference quotient anywhere in a run is dominated by noise. |
| §2 design envelope | 0.31 m/s | — |
| `v_div_step` at the confirmed onset (a2 runs) | 4.2 / 4.6 m/s | the step itself |

The first row is the important one: **`v_div_max` is 2.3× the design envelope on a run where
nothing happened**. Any statement of the form "the measured `v_div` exceeded the envelope" is
therefore vacuous — every honest run exceeds it. The envelope was derived for a claim drifting
away from truth at a finite rate and cannot be evaluated against a step injection at all. This is
reported rather than quietly omitted because the plan asks whether the measured `v_div` fell
inside the envelope, and the honest answer is that the question does not apply to this injection.

What does carry over: in the step case the fleet's protection cannot come from the detection
budget, because there is no budget against an unbounded rate. It comes from the two mechanisms
that do not depend on detection latency — conservative anchoring on the live barrier, and (where
the belief layer wins the race) the corpse anchor falling through to an unforged channel.

---

## 3. Arms a0 / a1 / a2

### 3.1 Batch 1 (`g2_logs/matrix_step1`, 9 attempts, all listed)

| # | arm | seed | outcome | body gap surv. | body gap attacker | **ghost offset** | detector blocks | time-to-evict | arrive |
|---|---|---|---|---|---|---|---|---|---|
| 1 | a0 | 21952 | complete | 0.441 | 0.551 | 0.422 | none (by design) | n/a | ✓ |
| 2 | a0 | — | complete | — | — | — | none (by design) | n/a | ✓ |
| 3 | a0 | 20299 | **aborted** | 0.328 | 0.583 | 0.426 | none (by design) | n/a | — |
| 4 | a1 | 1228 | **aborted** | 0.168 | 0.526 | 0.420 | **gate2: none** | n/a | wedged 0.820 |
| 5 | a1 | 25477 | complete | 0.393 | 0.591 | 0.392 | **gate2: none** | n/a | 0.336 |
| 6 | a1 | 13143 | complete | — | — | — | **gate2: none** | n/a | ✓ |
| 7 | a2 | 28112 | complete | 0.387 | 0.534 | **0.009** | belief: agent1, agent3 | **0.50 s (5 slots) both** | 0.097 |
| 8 | a2 | 15866 | complete | 0.406 | 0.537 | **0.007** | belief: agent1, agent3 | **0.50 s (5 slots) both** | 0.280 |
| 9 | a2 | 22610 | **FAILED** | 0.464 | 0.548 | 0.424 | **belief: none** | — | — |

All distances in metres, every one a minimum over every logged row. **Zero false positives in
all nine runs** — no survivor was ever blocked by any detector. Zero WBC deactivations, zero A*
path failures, zero observation-channel drops, in all nine.

**Ghost offset** is the closest the corpse keep-out's centre ever came to the attacker's real
body. It is the number this experiment turns on, and it is not the clearance number.

### 3.2 The lie translates the exclusion disc by exactly the lie

Injected claim displacement: **0.4243 m** (0.30 m applied to x and y). Measured ghost offset in
every run where the belief layer did not convict: **0.392, 0.420, 0.422, 0.424, 0.426 m.** The
corpse keep-out is displaced by the lie to within 4 mm, run after run. This is not a marginal
statistical effect; it is a one-to-one mechanical relationship.

Its consequence is worse than reduced clearance. In run 4 the fleet's own outbound goal fell
0.879 m from the anchor against a 1.63 m effective radius — **inside the exclusion disc, hence
unreachable by construction**. The survivors' centroid was pushed from 0.75 m out to 0.91 m
rather than converging. A lie does not only shrink the margin; it can deny the mission.

In the two runs where the belief layer convicted, the ghost offset was **0.009 and 0.007 m** —
the fence landed on the body instead of 42 cm away from it.

### 3.3 Why — and the honest limit of the claim

⚠️ **This is not conservative anchoring.** `corpseAnchor()` fences the peer's latest *claim* when
that peer is in `exited_` (it announced a roster excluding us) and its `peer_odom_` otherwise.
Conservative anchoring corrects the live pairwise barrier and `set_peer_keepout`; the corpse
keep-out is a different mechanism and is not covered by it. That gap is this project's already
registered **Task 13**, and these numbers are its measurement.

Which branch fires is decided by a **race**. `LIE_DEAF=1` makes the attacker announce a roster
excluding the survivors. If the belief layer convicts first, the peer is blocked for *lying*,
never becomes `exited_`, and the anchor falls through to odom. If roster-exclusion arrives first,
the anchor is the lie.

⚠️ **The harness under-models the attacker on exactly this path.** `applyOdomFake` forges a local
copy inside `gate2()` only, so the member `peer_odom_` stays honest and `corpseAnchor`'s fallback
reads a channel the dual-channel threat model says is compromised. **Every a2 ghost-offset number
here is a lower bound** on what a real dual-channel attacker could do to the corpse keep-out.

### 3.4 A2 loses that race one run in three, and the mechanism is measurable

Run 9 is not a flake and not a borderline residual. Tracing it:

- agent1 got **four** evidence-bearing slots. `l_total` reached **−6.400** against a −9.2
  threshold.
- Roster-exclusion blocked the peer at **0.914 s** after the attack.
- Once blocked, `trust_step_observed` returns `L` unchanged, so the accumulator **froze at
  −4.499 permanently**.

What stole the slots, from the post-onset abstain codes:

| run | outcome | post-onset abstain codes (both survivors) |
|---|---|---|
| 7 | convicted | `{evidence 10, peer_blocked 2}` |
| 8 | convicted | `{evidence 10, peer_blocked 2}` |
| 9 | **froze** | `{evidence 7, no_fresh_claim 2, occluded 1, peer_blocked 4}` |

The pre-attack `no_fresh_claim` rate is 1.0–1.1 % in all three runs, so losing two post-onset
slots is a ~20× local elevation. The cause is the attacker's own deafness stalling the
`AgentState` barrier that sets `claims_fresh` — the gate on collecting evidence at all.

**Going deaf both starves the detector that would convict you and triggers the alternative
eviction path that anchors the keep-out on your lie.**

The belief layer needs 5 consecutive evidence-bearing slots; roster-exclusion lands at ~9 slots;
two lost slots flip the outcome.

⚠️ This race is tight **because this session made it tight, and that is the correct call**. The
attack is now armed in a single dispatch (§1); the previous harness set the deafening several
seconds after the lie, handing the detector a head start no real attacker would grant.
`d_run.sh`'s standing comment that the lie "comfortably beats the deafness clock" is, measured,
false.

### 3.5 Batch 1's acceptance criterion was wrong, and batch 2 re-runs it

Runs 3 and 4 aborted on criteria that did not measure what they claimed:

- Run 3 aborted at a survivor **centre** distance of 0.8890 m against a 0.90 m guard, while the
  true survivor–survivor **body gap** was **0.3284 m**. They did not touch. The guard is a proxy
  this project has already recorded as reading ~0.68 m high.
- Run 4 was scored "never reached the outbound goal" by a bound of 0.65 m derived as half of a
  **1.30 m static** corpse radius. Every LIE arm produces a **mobile** corpse at 1.63 m, so the
  bound should have been 0.815 — against a run that finished at 0.820, a 5 mm miss.

Neither was changed mid-batch: that would have made batch 1's runs mutually incomparable. The
criterion is now "is the goal inside a corpse keep-out", read from the anchor and radius the
agents logged, with the constant deleted rather than re-derived. **Batch 2 re-runs all nine.**
Post-hoc relabelling is not enough — an aborted run never executed its return leg, and that data
does not exist to be recovered.

### 3.6 Batch 2 ⏳

*In flight.*

---

## 4. smear / duty / forged_obs ⏳

*Pending.*

**Registered in advance for `duty`** (so that a surprise is recognisable as one): `dutyLying` is
a fixed 50 % duty cycle — it lies for the first half of every `P`-slot window — so `P` changes
burst length only. With λ = 0.951, a full penalty of −2.0 and a credit of +0.25 × 2.0 = +0.5, the
per-period affine map has fixed points **L = −14.67 (P = 2)** and **L = −12.13 (P = 10)**, both
past `l_evict = −9.2` and both below the −11.2 floor. **Both should convict.** The arm therefore
measures *time*-to-evict and how much of the 0.433 m buffer the intermittent lie spends first,
not whether conviction happens. A non-conviction would falsify the `credit_ratio = 0.25`
derivation.

**`forged_obs` runs in both GID-pin orderings.** The late impostor loses the latch and is
rejected; the first mover wins it and the genuine publisher is dropped instead. The second is
**not a bug to be hidden** — it is the honest boundary of a trust-on-first-use pin and must be
reported as the defence's limit.

---

## 5. Occlusion ⏳

*Pending.*

Acceptance requires a **positive** record: abstain code 5 (`occluded`) from both observers, `L`
numerically unchanged across the window, resumption afterwards. Silence does not count —
throttling, a dead subscription, a freshness guard and an empty queue all look identical to it.

Two things established before running it:

- **A doubly-occluded, in-range spot exists**: a search over the plum pile list found one for
  19 of 22 sampled fleet positions inside the peg field. In-range matters, because abstain code 4
  (`out_of_range`) also comes from `visible()` returning false and would prove nothing about
  occlusion.
- **The probe must run with blocking disarmed.** Any lie above ~0.153 m earns the full −2.0
  penalty (the graded band is only 5.33 mm wide), so conviction lands in 7 slots ≈ 0.7 s — faster
  than the attacker can be walked behind cover. Holding `L` at its ceiling instead is not a
  substitute: a saturated value is unchanged whether or not evidence is arriving, so it cannot
  discriminate. The attacker must be *mid-descent* when it hides.

---

## 6. Stale-vote deadlock — reproduced, and the fix does not cover it

This was scheduled as a scenario to run deliberately. It arrived on its own, in batch 2's first
attempt (`d_0801_044738`), and the answer to "does the fixed-point correction stop the fleet
deadlocking" is **no, not for this input**.

**Chain of events, from the logs:**

1. 59 s after the attack, with the two survivors parked at the outbound goal, agent3 evicted
   robot1 as "silent 10 slots" and agent1 then evicted robot3 as "silent 3 slots". **Neither had
   done anything wrong.** This is the false-positive shape already on this project's record
   (2026-07-28: two healthy agents evicted each other after a single bad broadcast).
2. Both rebuilt solo, leaving the coordinator with `views = {1:[1], 2:[2], 3:[3]}`.
3. `admm::majority_excluded` has **no fixed point** on that input. Transcribed exactly and run:

   | pass | `excluded` | → `next` |
   |---|---|---|
   | 0 | {} | {1,2,3} |
   | 1 | {1,2,3} | {} |
   | 2 | {} | {1,2,3} |
   | 3 | {1,2,3} | {} |

   Period 2, forever. The loop is bounded at `views.size() + 1` = 4 passes and exits holding
   `excluded = {}`, so `excluded_by(i, {})` is **true for every i**.
4. The coordinator saw **zero live dogs** and logged `FLEET WEDGED: goal DROPPED — every dog is
   majority-excluded, so nobody is live` once a second for the rest of the run. The survivors
   never moved again.

The comment above that loop bound says it "keeps a pathological, fully-symmetric input (every
robot's roster contains only itself) from spinning forever instead of returning". It does
return — with the maximally wrong answer. **Converting a hang into a silent wrong answer is not a
fix, and the symmetric input is not hypothetical: step 1 produces it.**

**This is Task 9's code (`e92671c`) — the only task on this branch that never received an
independent review.** The ledger records why: the reviewing attention went to the `NDEBUG`
discovery the same agent surfaced. `test_false_signal.cpp` has eight `majority_excluded` cases —
unanimous 2-0, a 1-1 split, empty-roster abstention, silence, no views at all, an
attacker-plus-misjudgment case at n=4, and the two-pass case that motivated the fixed point — and
**not one with a symmetric roster.** The uncovered case is the one that wedges the fleet.

The intended behaviour is intact: `views = {1:[1,3], 2:[2], 3:[1,3]}` still excludes only
robot2. The defect is confined to inputs with no fixed point.

**Proposed fix (owner decision, not applied):** non-convergence means the ballots are
self-contradictory, and for *liveness* the conservative answer is to exclude **nobody**. A fleet
that keeps planning for a dog that may be gone is recoverable; a wedged fleet is not. One
condition — if the loop exhausts its passes without `next == excluded`, return false — plus a
test for `{1:[1], 2:[2], 3:[3]}`.

### 6.1 No-attacker soak ⏳

*Pending.* Walks a goal continuously; a stationary soak cannot show drift-driven false positives.

---

## 7. Regression ✅

| check | result |
|---|---|
| oracle tests (Python vs C++ parity) | **44 passed** |
| `ctest` | **6/6** (`distributed_parity`, `agent_timeout`, `agent_failover`, `corpse_keepout`, `false_signal`, `trust`) |
| G1 distributed-vs-centralized parity | **`worst max|delta| = 0`, mismatches = 0** |

Run against the branch head that produced the results above. Per this project's `8c6ec4d`
finding, these assertions are live: before that commit `RelWithDebInfo`'s `-DNDEBUG` compiled
them out and "ctest all pass" was not evidence for `test_trust`, `test_false_signal` or
`test_corpse_keepout`.

---

## 8. Communication cost (G5) ⏳

*Pending.* Folded in here rather than run separately, because the trust layer adds fields to
`AgentState` and measuring G5 first would have measured a system about to change.

The measurement is a genuine A/B rather than an analytic estimate: `beliefStep()` is the only
producer of `ev_peer`/`ev_pos`, and it runs only when `obs_gate2 = true`, so arms `a0`/`a1`
broadcast empty evidence arrays and the `a1` → `a2` difference in bytes/second **is** what the
defence costs on the wire.

**Method, and why the attack runs cannot answer it.** `bytes_tx`/`bytes_rx` are read-and-reset by
`take_bytes()` at every `publishStats`, so each `stats.csv` row carries the traffic since that
robot's previous row. The rate is therefore a sum over intervals, not a first-to-last span:
the first row holds everything since node start (all of bring-up), and `g5_logger`'s
subscriptions do not all establish at once — in one run two robots had a 37.5 s hole in their
stream while the third simply started 37.6 s in, which a span-based rate turned into a spurious
1.4× difference between two identically-configured agents. Intervals longer than 1 s are dropped
from both numerator and denominator. After that correction the three agents in a run agree to
0.2 %, with the deafened attacker correctly lower.

⚠️ **The a0/a1/a2 attack runs are not a valid A/B for this.** Traffic is dominated by
`EdgeXi`/`EdgeZ`, which scale with `achieved_rounds`, and the arms converge differently — a2
measured *lower* than a1 in one pair of runs, which is a statement about ADMM convergence and
not about what evidence costs. Worse, the fleet's topology changes mid-run when the attacker is
evicted. The A/B therefore comes from the **`NO_KILL` soaks**, where nothing is evicted, nobody
is deafened, and the only difference between the two runs is whether `beliefStep` is producing
evidence.

⏳ *Soak measurement pending.*

---

## 9. What the harness got wrong, and when

Every number above depends on the harness being right about what it was measuring. It was not,
three times, and each correction changed which runs counted. Recording it here because a results
table that hides its own instrument history is not checkable.

| # | what was wrong | how it was found | effect |
|---|---|---|---|
| 1 | `inject_odom_fake` was **never set at all**. Task 10 built both halves of the dual-channel forgery; the arm selector wired only the claim half. | reading the arm selector against Task 10's report before the first run | every arm would have faced a single-channel attack — the one A1 catches trivially. Acceptance criterion 1 could neither have passed nor failed honestly. |
| 2 | Three `ros2 param set` calls do not land together. Measured skew **2.78 s**. | pilot run `d_0801_035833`: agent1 blocked the attacker 0.61 s into its own single-channel window | "A1 caught the lie on 1/2 survivors" was measuring the harness. Fixed by `arm_attack.py` (one process, one discovery, all requests back to back → **93 µs**). |
| 3 | `KILL_AT_X` was decorative — the attack fired 2.8–3.9 m past where it was staged, by an amount that depends on RTF. | comparing the requested and actual trigger positions across runs | where the attack lands decides what the run tests, so runs were not comparable. Two causes: a pre-arm block that ran while the fleet walked, and `fleet_centroid` costing ~15 s per call. Now fires at **x = 3.125** for a requested 3.0. |
| 4 | The "keep-out blocked" acceptance test, **wrong in both directions**. | batch 1 run 4 wedged at 0.820 against a corrected bound of 0.815 (5 mm); batch 2 run 2 stalled 0.527 m from a 9 m goal with the corpse 4.9 m away | first it forgave any stall below 0.65 m — half a *static* corpse radius, when every lying arm produces a *mobile* one at 1.63 m. Then my replacement asked whether the **goal** was fenced, when the original comment said a **slot** can be. Now: stalled **and** (goal inside a keep-out → mission denied, counted) **or** (centroid inside the formation's published half-extent → arrived). Both read off the run's own geometry; the constant is gone. |

Two measurement bugs in the analysis script were also found and fixed, both of which had produced
plausible-looking numbers:

- **`v_div` pooled both observers** into one time series, so it differenced observer A's residual
  against observer B's a millisecond later — denominator the publish skew, numerator two
  independent noise draws. A clean run with no attacker read **25.1 m/s** against a 0.31 m/s
  design envelope. Per-observer it reads 0.726 m/s, which is the estimator's **noise floor** and
  still 2.3× the envelope — so `v_div_max` can never be compared to §2 at all.
- **A single-sample onset detector** fired on one isolated 0.1160 m residual 2.9 s before the
  attack, turning agent3's true 0.5 s eviction latency into a reported 3.37 s and inventing a
  6.7× spread between two observers of the same attack. Requiring three consecutive
  over-threshold samples — the same shape of evidence the belief layer itself requires — gives
  0.50 s for both.

## 10. Every attempt, including the rejected ones

| # | arm | result | why it is listed |
|---|---|---|---|
| `d_0801_035833` | a1 | **REJECTED — harness** | Arming skew 2.78 s. The three forgery knobs were set by three separate `ros2 param set` processes; agent1's odom half landed 2.78 s after the claim half and it blocked the attacker 0.61 s into that single-channel window. Measured the harness, not the detector. Cause of the `arm_attack.py` rewrite. |
| `d_0801_040522` | a1 | **pilot, accepted as a wiring check only** | Arming skew 93 µs (agents' own log stamps). gate2 caught the lie on 0/2 survivors — the structural miss the design predicts. Not counted toward `n`: it ran before the `KILL_AT_X` staging fix, so its attack fired at x = 5.77 instead of the requested x = 3.0. |

⏳ *Matrix attempts appended as they complete.*
