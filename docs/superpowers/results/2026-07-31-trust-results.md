# Observation-sourced belief layer — experiment matrix results

Plan: `docs/superpowers/plans/2026-07-31-observation-belief-trust-rev2.md`, Task 12.
Design: `docs/superpowers/specs/2026-07-31-observation-belief-trust-design.md`.
Calibration this builds on: `2026-07-31-trust-calibration.md`.

**Status: IN PROGRESS.** Sections marked ⏳ have no data yet and must not be cited.

## Acceptance criteria at a glance

Spec §9's eight criteria, each of which was required to be *able* to fail. Seven have data.

| # | criterion | outcome |
|---|---|---|
| 1 | A2 catches what A1 structurally cannot | ✅ a1: **0 detector-attributed blocks in 5 runs**; a2: convicted in 5 runs, **0.50 s / 5 slots** every time |
| 2 | conservative anchoring keeps the body gap from shrinking | ⚠️ **does not cover the corpse anchor** (registered Task 13). A2's advantage comes from *winning a race*, and the harness under-models the attacker on that path, so its numbers are a **lower bound** |
| 3 | NO_KILL soak, zero false positives | ❌ **a2 partitioned the fleet with no attacker** — not the belief layer (`L` flat at 4.600 vs −9.20) but a correlated barrier stall the *silence* rule resolves by eviction (§6.2). a1's 675.3 s pass had **4 slots of margin** on the same mechanism, and separately is **not** evidence the fleet held separation: all three soaks reached body gaps of 0.18–0.24 m (§6.1) |
| 4 | occlusion: abstention positively recorded, `L` unchanged, resumption | ✅ **and the criterion's wording is wrong** — see §5.1, it should read "suspicion does not decay" |
| 5 | smear: honest peer never evicted, smearer caught | **half, and the recorded reason was wrong.** Target never blocked (3/3 at N=3, 1/1 at N=5) ✅; smearer never convicted ❌ — **not** an N = 3 blind spot but an inequality between three constants, `floor + l_max = −6.6 > −9.2 = l_evict`, so a peer honest about *itself* is structurally unconvictable at **any** N. N = 5 is strictly **worse** (§8b.1) |
| 6 | asymmetric credit convicts an intermittent liar | ❌ **evades when the lying burst is shorter than the conviction depth** (P = 2: 0/3; P = 10: 1/3) |
| 7 | the fixed-point correction stops the stale-vote deadlock | ❌ **it does not** — `majority_excluded` has no fixed point on the symmetric input and the loop bound returns the maximally wrong answer, wedging the fleet (§6) |
| 8 | regression: 44 oracle + ctest + G1 bit-identical | ✅ 44 passed, 6/6, `worst max\|delta\| = 0` |

**Four of the seven settled criteria did not pass.** Each failure is mechanistically traced, and
criteria 6 and 7 plus the caveat on 2 all reduce to **one structural problem**: the belief
accumulator races against roster-exclusion, and whatever loses that race decides where the
corpse keep-out gets anchored.

Criterion 3 is the exception, and it is worth separating: **it failed on something that is not
this branch's**. The belief layer never convicted anybody in that run; a correlated stall in the
G4 consensus barrier made every agent look silent to every other, and the eviction rule — written
for silence as a property of *one* peer — resolved it by letting the first counter to trip evict
the rest (§6.2). It is in this document because the soak found it, not because the trust layer
caused it.

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

### 3.3a Conviction lands in 5 slots, not the 7 the design derives — and the extra 2 are hearsay

The design's debounce arithmetic gives 7 consecutive full-penalty steps
(4.6 → 2.375 → 0.258 → −1.754 → −3.754 → −5.754 → −7.754 → −9.754, crossing at step 7). Measured
conviction is 5 slots. The telemetry says why — agent1's trace from run 8, sim time, one row per
slot:

| t (s) | residual | `l_self` | `l_total` |
|---|---|---|---|
| 69.22 | 0.4521 | 2.875 | 5.269 |
| 69.32 | 0.3771 | 0.734 | 1.031 |
| 69.42 | 0.4164 | −1.302 | −3.000 |
| 69.52 | 0.4115 | −3.302 | −6.897 |
| 69.62 | 0.3985 | −5.302 | **−9.902** ← crosses −9.2 |
| 69.72 | — | −5.302 | −9.677 (abstain = peer_blocked) |

`l_self` falls at 2.000 per slot — exactly `clamp_step`, i.e. the residual is far enough past the
decision boundary to saturate every step, as the 5.33 mm graded band predicts. `l_total` falls at
~3.0 per slot. **The extra ~1.0 per slot is the other survivor's relayed evidence**, and it is
what turns a 7-slot first-hand conviction into a 5-slot one.

That is the second-hand channel doing exactly the job it was built for, and it is also why the
`trust_total` sum clamp matters: the same mechanism that accelerates a true conviction is what a
smear arm tries to abuse, which is what §4's arms exist to test.

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

### 3.6 Batch 2 — stopped after 2 attempts

Stopped deliberately, not lost. Its first attempt hit the fleet-wedging defect (§6) and its
second exposed that the replacement acceptance criterion was still wrong (§9, row 4). Continuing
would have spent ~40 minutes producing runs that a third criterion would supersede. Both attempts
stay in the record (§10).

### 3.7 Batch 3 (`g2_logs/matrix_step1c`, 9 attempts, corrected criterion)

| # | arm | seed | outcome | body gap surv. | **ghost offset** | detector blocks | time-to-evict | arrive |
|---|---|---|---|---|---|---|---|---|
| 1 | a0 | 25024 | complete | 0.350 | 0.418 | none (by design) | n/a | 0.367 |
| 2 | a0 | 7480 | did not converge | 0.367 | 0.420 | none (by design) | n/a | 0.825 |
| 3 | a0 | 16900 | complete | 0.364 | 0.423 | none (by design) | n/a | 0.109 |
| 4 | a1 | 16032 | complete | 0.372 | 0.402 | **gate2: none** | n/a | 0.193 |
| 5 | a1 | — | **infra** | — | — | — | — | WBC deactivated before the kill point |
| 6 | a1 | 25431 | complete | 0.348 | 0.415 | **gate2: none** | n/a | 0.416 |
| 7 | a2 | 29730 | complete | 0.344 | **0.007** | belief: agent1, agent3 | **0.50 s (5 slots) both** | 0.062 |
| 8 | a2 | 7732 | complete | 0.344 | **0.003** | belief: agent1, agent3 | **0.50 s (5 slots) both** | 0.141 |
| 9 | a2 | 3398 | did not converge | 0.404 | **0.008** | belief: agent1, agent3 | **0.50 s (5 slots) both** | 1.087 |

Run 5 is the one pre-registered exclusion class — the known upstream activation race — recorded,
not retried. Runs 2 and 9 did not converge on the outbound goal; **run 9's detection succeeded in
full** (see below), so its failure is a fleet-convergence result, not a defence result.

**Every a2 run in batch 3 convicted**, on both survivors, in exactly 5 slots, with ghost offsets
of 3–8 mm. Zero false positives in all nine.

### 3.8 Combined, and a correction

| arm | n (runs with data) | ghost offset (m) |
|---|---|---|
| a0 | 5 | 0.418, 0.420, 0.422, 0.423, 0.426 |
| a1 | 4 | 0.392, 0.402, 0.415, 0.420 |
| **a2** | 6 | **0.003, 0.007, 0.007, 0.008, 0.009**, and 0.424 |

Nine values from arms where the belief layer never convicted span **0.392–0.426 m**, a 34 mm
spread around an injected displacement of 0.4243 m. Five values from a2 convictions span
**3–9 mm**.

⚠️ **Correction to an earlier reading of batch 1.** On batch 1 alone the belief layer lost the
race to roster-exclusion in 1 of 3 runs, and that was written up as a rate. Batch 3 convicted 3
of 3. The combined figure is **1 loss in 6**, and n = 6 does not support a rate at all — what it
supports is that the failure *mode* is real and reproducible in mechanism (§3.4), not that it
occurs one time in three. Stated as a frequency it would have been an overclaim from three runs.

### 3.9 Non-convergence of the degraded 2-dog mode

Three runs across batches 1 and 3 stopped short of the outbound goal (0.820, 0.825, 1.087 m) and
it is **not** a keep-out effect: in the batch-3 case the corpse anchor was 3.3 m away and both
survivors were clear of it and of every peg (nearest 1.21 m against a 0.90 m effective radius).
What happens is that the two-dog `COL2` column settles **off its slots** — the pair sat 2.44 m
apart against a 1.5 m shape, rear dog 1.9 m back and 1.0 m off-axis.

This is a convergence weakness of the degraded mode, it appears in a0 and a2 alike, and it is
reported rather than absorbed into a looser acceptance criterion. Everything the experiment turns
on — body gap, ghost offset, detector attribution, time-to-evict — is collected up to the abort
point regardless; what a non-converged run loses is only its return leg.

---

## 4. smear / duty / forged_obs

### 4.1 smear — the target is protected, the smearer is not convicted, and the reason is §10

First run (`d_0801_053552`), and it is a substantive result rather than a null one.

**Acceptance criterion 5, first half: holds.** The smear target (robot1) was never blocked by
anything. The observing survivor's belief about it bottomed at `l_self` 3.578 / `l_total` 3.416,
nowhere near the −9.2 threshold. The relay term the smear tried to plant was capped by the
smearer's own collapsing credibility, exactly as designed.

**Second half: the smearer was not convicted — structurally, not by mis-tuning.**

| observer | about | `l_self` min | `l_total` min |
|---|---|---|---|
| agent3 | robot2 (the smearer) | **−11.200** (the floor) | **−7.797** |
| agent1 | robot2 | 2.375 | 6.558 |
| agent3 | robot1 (the target) | 3.578 | 3.416 |

agent3's smear check fired and charged the reporter at the full penalty every slot — its `l_self`
reached the floor. But `l_total = l_self +` the clamped relay sum, and **agent1's relay about
robot2 was positive**, pulling the total back to −7.797 and short of conviction.

**Why only one observer could check it.** `SMEAR_TARGET` defaults to the first survivor, robot1,
and `beliefStep` skips `j == self_id_` — an agent cannot run a smear check on a report about
*itself*. At N = 3, with the target being one of only two possible observers, **exactly one agent
can ever check the smear**, and its conviction is then diluted by the other agent's honest
positive relay.

That is spec §10 verbatim: *"symmetric 1v1: evidence symmetric, third party blind → no eviction;
at N = 3 every close approach is this case."*

**This is the strongest argument for the five-dog runs.** At N = 5 a smeared target still leaves
three agents able to check the report, so the relay sum carries the conviction instead of
cancelling it. **The N = 3 smear arm should be read as measuring the declared blind spot, not as
a defence failure** — and a further N = 5 run should point `SMEAR_TARGET` at a *non-observer*,
which is the case the mechanism was designed for and which N = 3 cannot construct.

Incidental: one `REJECTED AgentState from robot2 — non-finite` (Gate 1) on each survivor, a
single message out of thousands, unrelated to the smear path. Recorded, not explained.

**Reproduced across all three reps** (`d_0801_053552`, `054458`, `054738`):

| | agent3 → smearer | agent3 → target |
|---|---|---|
| `l_self` min | **−11.200 (the floor), all three** | 3.578 / 4.600 / 3.763 |
| `l_total` min | −7.797 / −7.646 / −8.806 | **3.416, identical to three decimals in all three** |

The smear check is not marginal — it **saturates**, every run. And the honest peer's positive
relay holds `l_total` 0.4–1.6 above the threshold, every run. The target's belief does not move
at all. This is a structural outcome, not a tuning question.

### 4.2 duty — the arithmetic is exact, and the prediction asked the wrong question

`P = 2` (`d_0801_055250`). Measured `l_self`, identical on both observers:

| t (s) | residual | `l_self` | model |
|---|---|---|---|
| 59.82 | 0.4057 | 2.875 | — |
| 59.92 | 0.0097 | 3.234 | 0.951·2.875 + 0.5 = **3.234** ✓ |
| 60.02 | 0.4270 | 1.075 | 0.951·3.234 − 2.0 = **1.075** ✓ |
| 60.12 | 0.0155 | 1.523 | 0.951·1.075 + 0.5 = **1.522** ✓ |
| 60.22 | 0.4222 | −0.552 | 0.951·1.523 − 2.0 = **−0.552** ✓ |
| 60.32 | 0.0635 | −0.052 | −0.552 + 0.5 (no decay below 0) = **−0.052** ✓ |
| 60.42 | 0.4433 | −2.052 | −0.052 − 2.0 = **−2.052** ✓ |

**The trust arithmetic reproduces to three decimals at every step.** `credit_ratio = 0.25` and
the decay asymmetry both do exactly what their derivations say: net −1.5 per two-slot period.

**But the registered prediction measured the wrong quantity.** It gave fixed points of −14.67
(P = 2) and −12.13 (P = 10), both past `l_evict` — true given unlimited time, and irrelevant.
Duty halves the accumulation rate (−0.75/slot against −2.0/slot for a continuous liar), so the
belief layer needs ~2.7× longer, and **roster-exclusion arrives at ~11 slots and wins**: `l_self`
stopped at −3.552 and `l_total` at −7.175, neither past −9.2, and the peer was then blocked by
the silence path (abstain → 1).

**So the duty arm's answer is neither "it works" nor "list it as a limitation".** The asymmetry is
correct; the binding constraint is the *same race* as §3.4, and duty simply makes it easier for
the attacker to win by halving how fast evidence accumulates. The duty arm, the a2 race-loss and
the `corpseAnchor` decision are **one structural problem, not three**, and the useful fix is at
the race rather than at `credit_ratio`.

**Across all three P = 2 reps:**

| rep | belief blocked? | ghost offset | verdict |
|---|---|---|---|
| 1 | ✗ (roster-exclusion only) | 0.419 | PASS |
| 2 | ✗ | 0.422 | did not converge (0.529) |
| 3 | ✗ | **0.003** | PASS |

**The duty-cycled liar evaded the belief layer in 3 of 3.** Systematic, not probabilistic: the
halved accumulation rate means the belief layer needs ~2.7× longer than the ~11-slot silence
path, so it loses every time.

**And rep 3's ghost offset is 3 mm despite the belief layer never convicting.** The mechanism is
worth stating: `corpseAnchor` uses `latest_claim` — the *last* `AgentState` received — and
`P = 2` alternates every slot, so whether the keep-out lands on the lie or on the truth depends
entirely on which half of the duty cycle the attacker's final broadcast fell in. Two landed on a
lying slot (0.419, 0.422 — the same value as a0/a1), one on an honest slot (0.003).

That is an **independent confirmation that the keep-out follows the claim**, and it exposes
something worse than the a0/a1 case: an intermittent liar can *choose* the phase of its last
broadcast, i.e. choose whether to leave a 1.63 m exclusion disc in the wrong place. In a0/a1 the
displacement was at least deterministic.

It also sharpens the `corpseAnchor` question (§10, decision 2): keying on belief rather than on
block reason would **not** fix the duty case on its own, because belief never convicts there. The
duty arm argues the anchor question is about *which channel is trusted*, not merely about which
verdict fired.

### 4.3 duty P = 10 convicts, P = 2 does not — the condition is burst length

`P = 10` (`d_0801_060520`), agent1 from onset:

| t (s) | residual | `l_self` | `l_total` |
|---|---|---|---|
| 62.02 | 0.4095 | 2.375 | 4.769 |
| 62.12 | 0.4221 | 0.258 | 0.555 |
| 62.22 | 0.4084 | −1.754 | −3.452 |
| 62.33 | 0.4445 | −3.754 | −7.349 |
| 62.42 | 0.4235 | −5.754 | **−10.354** ← crosses −9.2 |

Belief blocked on both survivors; **ghost offset 0.001 m**.

`P = 10` means **five consecutive lying slots**, so accumulation runs at the full −2.0/slot —
identical to a continuous liar — and conviction lands in exactly the 5 slots §3.3a measured.
`P = 2` alternates every slot, so the lying burst is always **one** slot and the honest slot
immediately claws back +0.5.

> **The vulnerable condition is: lying burst shorter than the conviction depth (5 slots), i.e.
> `P ≤ 8`.**

**Assessment of the registered prediction, stated rather than quietly replaced.** The fixed points
computed in advance (−14.67 for P = 2, −12.13 for P = 10) were **correct**, and both lie past
`l_evict` — P = 2 *would* convict given unlimited time. They were nevertheless the **wrong
quantity to predict**, because a fixed point assumes unlimited time and the run does not have it.
The right predictor is whether the lying burst reaches the conviction depth before the competing
~11-slot eviction path fires.

This also means the fix sketched in the handoff — "add a parameter making release slower than
rise" — is **not** the one the data asks for: `credit_ratio = 0.25` already makes the net
negative (−1.5 per P = 2 period). The binding constraint is the race, for the third time.

### 4.4 duty, complete at n = 3 per period

| period | burst | belief convicted | ghost offset (m) |
|---|---|---|---|
| P = 2 | 1 slot | **0 / 3** | 0.419, 0.422, **0.003** |
| P = 10 | 5 slots | **1 / 3** | **0.001**, 0.423, **0.004** |

⚠️ **Correction.** §4.3 stated "P = 10 convicts" from a single run. At n = 3 it is 1 of 3:
conviction needs five consecutive lying slots *available from the moment evidence starts
accumulating*, so it also requires the trigger to align with the start of a lying burst, not
merely a long enough burst.

**The ghost-offset distribution is the more important result.** Four of the six duty runs landed
at 1–4 mm and only two at 0.42 — and **three of those four had no belief conviction at all**.
`corpseAnchor` fences the *last* `AgentState` received, and a duty-cycled liar's final broadcast
falls in its honest half about half the time. So the keep-out position for an intermittent liar
has three outcomes:

| outcome | runs | anchor lands on | ghost offset |
|---|---|---|---|
| belief convicts | 1 / 6 | odom | mm |
| no conviction, last slot honest | 3 / 6 | an honest claim | mm |
| no conviction, last slot lying | 2 / 6 | **the lie** | 0.42 m |

**This is bad news, not good news.** The attacker *chooses* the phase of its final broadcast. It
does not need to beat the detector at all — it only needs to tell its last lie before being cut
off, which turns that 50 % into 100 %. **The apparent improvement in the duty ghost offsets is an
artefact of an uncontrolled attacker, not a property of the defence**, and it would be a serious
misreading to quote the 1–4 mm figures as duty-arm performance.

### 4.5 forged_obs, late impostor — the pin's coverage case, cleanly

| rep | belief convicted | time-to-evict | ghost offset | impostor samples rejected | false positives |
|---|---|---|---|---|---|
| 1 | agent1 + agent3 | 5 slots | 0.003 | **42,961** | 0 |
| 2 | agent1 + agent3 | 5 slots | 0.005 | **40,318** | 0 |
| 3 | agent1 + agent3 | 5 slots | 0.005 | **39,902** | 0 |

Detection is **indistinguishable from a clean a2 run**. The GID pin rejected every impersonating
sample, the observation channel stayed honest, and the position lie was caught on the same
5-slot schedule. One of the few acceptance criteria that comes out cleanly.

**Quote the counter, never the log.** ~40,000 rejected samples against **88** throttled
`second publisher` WARN lines — a factor of ~460. Task 10 added `n_obs_dropped` on the grounds
that a dying observation channel must reach the run's verdict rather than a throttled line; this
is the proof, and it is the same argument as §9 protocol item 6 arrived at from the opposite
direction.

### 4.6 forged_obs, first mover — ⚠️ NOT ACTUALLY EXECUTED

`FORGED_FIRST=1` produced numbers indistinguishable from §4.5 (belief convicted on both in 5
slots, ghost 0.007, 37,923 samples rejected, zero false positives). **That is not coverage — the
arm did not run the case it was supposed to run.**

Timing, from `d_0801_062505`:

| event | time |
|---|---|
| Gazebo plugin starts publishing `/robot1/hardware/odom` (phase 1) | 06:25:08 |
| impersonating publisher created at agent construction (phase 3) | 06:25:42 |

**34 seconds late.** The observing agent's GID pin was already latched onto the genuine writer,
so the impostor was rejected — the *late-impostor* path. Task 10 described this case as "given a
nonzero value at construction (a launch parameter)", which is true of the *agent's* construction;
but the agent is not the earliest publisher in this bring-up, the simulator's sensor is. **The
launch parameter is necessary and not sufficient.**

**What this does establish, stated positively:** in this system the TOFU pin's weak case requires
an attacker publishing *before the sensor itself exists*. In a real deployment that maps to a
robot compromised and broadcasting before the fleet powers up — a considerably stronger
precondition than "one robot is compromised".

**To run it properly:** start a standalone impostor publisher ahead of the Gazebo launch. Not
done here, because it changes the bring-up order that every completed arm depends on. **Spec §9's
second `forged_obs` case is recorded as NOT EXECUTED**, with the reason and the recipe.

**Registered in advance for `duty`** (so that a surprise is recognisable as one): `dutyLying` is
a fixed 50 % duty cycle — it lies for the first half of every `P`-slot window — so `P` changes
burst length only. With λ = 0.951, a full penalty of −2.0 and a credit of +0.25 × 2.0 = +0.5, the
per-period affine map has fixed points **L = −14.67 (P = 2)** and **L = −12.13 (P = 10)**, both
past `l_evict = −9.2` and both below the −11.2 floor. **Both should convict.** The arm therefore
measures *time*-to-evict and how much of the 0.433 m buffer the intermittent lie spends first,
not whether conviction happens. A non-conviction would falsify the `credit_ratio = 0.25`
derivation.

**Registered in advance for `smear`.** The attacker keeps its own position honest and fabricates a
0.30 m false sighting of survivor 1 (offset in x and y, so a 0.4243 m discrepancy). Each survivor
that independently observed the target at the reported slot runs `smear_llr`, which is
`trust_llr` of that discrepancy charged **to the reporter** — 0.4243 m is far past the 0.15 m
decision boundary, so it saturates at the full −2.0 every checked report, exactly like a position
lie. From the +4.6 ceiling that is 4.6 → 2.375 → 0.259 → −1.754 → −3.754 → −5.754 → −7.754 →
−9.754, i.e. **conviction of the smearer at step 7 on first-hand evidence alone, and sooner once
the two survivors relay about each other** (§3.3a measured that acceleration at ~1.0 per slot).
Meanwhile the relay term the smear is trying to plant on survivor 1 is capped at the smearer's
own credibility, which is collapsing, so it should shrink to nothing.

Predicted: **smearer convicted, target never blocked.** A blocked target would falsify
acceptance criterion 5.

⚠️ This arm **must run with `LIE_DEAF=0`**, and that was found by reading the evidence path rather
than by spending runs on it. A deafened attacker's `peer_xnow()` empties, so its outgoing
`ev_peer` array is empty, and `applyFakeEvidence` corrupts a sighting *already in* the message
rather than fabricating one — the smear channel would transmit nothing and the arm would report
"no honest peer was evicted" for entirely the wrong reason. That is the same shape of error
review round 2 caught once already (I3′, where the smear attacker was also sending the position
lie).

**`forged_obs` runs in both GID-pin orderings.** The late impostor loses the latch and is
rejected; the first mover wins it and the genuine publisher is dropped instead. The second is
**not a bug to be hidden** — it is the honest boundary of a trust-on-first-use pin and must be
reported as the defence's limit.

---

## 4b. The true control — what the belief layer is worth on its own

The recorded 0.557 vs 0.866 comparison was **not** defence versus none: the transport's EXIT rule
fenced the liar in *both* arms, so that pair measured about a second of exposure difference. With
`LIE_DEAF=0` the attacker keeps hearing the fleet, its silence timeout never fires, it never
broadcasts an excluding roster, and **nothing incidental can remove it**. `a0` then has no
defence at all, and `a2` has only the belief layer.

| | `a0` undefended (n = 3) | **`a2` belief layer alone (n = 3)** |
|---|---|---|
| removal events | **0 / 0 / 0** | **4 / 4 / 4** |
| convicted | — | **3 / 3, both survivors each time** |
| time-to-evict | — | **6 slots: 0.57 / 0.60 / 0.56 s** |
| ghost offset | — (no keep-out is ever drawn) | **0.009 / 0.009 / 0.008 m** |
| **survivor body gap** | **0.2727 / 0.2713 / 0.2376 m** | **0.3600 / 0.3519 / 0.3512 m** |
| false positives | 0 | **0** |

**The two groups do not overlap.** `a2`'s minimum (0.3512) exceeds `a0`'s maximum (0.2727) by
78 mm, and `a2`'s spread across three runs is 8.8 mm.

**Detection latency here is 6 slots, one more than the 5 measured with `LIE_DEAF=1`** — and this
is the more honest figure. A deafened attacker stops responding, and its silence accelerates the
survivors' verdict; here it participates in consensus normally and the belief layer must convict
purely on observation residual. **Quote 6 slots for the layer in isolation.**

## 5. Occlusion ✅

Acceptance requires a **positive** record: abstain code 5 (`occluded`) from both observers, `L`
numerically unchanged across the window, resumption afterwards. Silence does not count —
throttling, a dead subscription, a freshness guard and an empty queue all look identical to it.

**Baseline, from the three a2 runs of batch 1** (the only arms with belief telemetry) — 2160
observer-slot rows about the attacker:

| code | rows | share |
|---|---|---|
| 0 `evidence` | 2074 | 96.02 % |
| 5 `occluded` | 54 | 2.50 % |
| 2 `no_fresh_claim` | 24 | 1.11 % |
| 1 `peer_blocked` | 8 | 0.37 % |
| 3 `no_obs_buffer`, 4 `out_of_range` | 0 | — |

**Slots where *both* observers were occluded at once: 0 of 1054.** Incidental occlusion is
common (one observer loses the sightline 2.5 % of the time) but the acceptance condition — every
observer blind simultaneously — **never happens by chance in the peg field.** It has to be
steered, which is what the `occl` arm and `hide_spot.py` are for. `out_of_range` never fires at
all, confirming the calibration finding that `obs_range = 4.0` does not bind at these fleet
spacings.

### 5.1 Result — criterion satisfied, and its wording needs correcting

`d_0801_063709`. Longest window with **every** observer at abstain code 5: **0.60 s / 7 slots**
(t = 109.90–110.50). Inside it:

| observer | belief about the attacker | `l_self` across the window | \|Δ\| |
|---|---|---|---|
| agent3 | had convicted it (log-only, at the floor) | −11.2000 → −11.2000 | **0.000000** |
| agent1 | still trusted it | 4.6000 → 3.4028 | 1.197 |

**One window demonstrates both halves of the designed asymmetry.**

- **Suspicion does not decay while unobservable** — exact zero over seven slots. This is
  `trust.hpp`'s stated rule: *clearing suspicion requires evidence of honest behaviour, and
  evidence requires visibility; being unobservable is not proof of innocence.* **An attacker
  cannot launder suspicion by hiding.**
- **Positive trust does decay**, at λ per slot (4.6 × 0.951⁶ = 3.40, matching to two decimals) —
  *a peer honest all mission must not buy undetected approach time.*

⚠️ **Wording correction for acceptance criterion 4.** The spec asks for "`L` numerically
unchanged across the window". That is true only for a *suspected* peer; trust decaying is the
intended complementary behaviour, not a violation. The criterion should read **"suspicion does
not decay"**. The probe could show both at once only because the two observers happened to hold
opposite beliefs about the same peer — log-only let agent3 convict while agent1 never did.

Abstain histogram: `{evidence 1567, occluded 38, out_of_range 56, no_fresh_claim 6}`. **Both
visibility codes are positively recorded** — the hide spot sometimes took the attacker past
`obs_range` as well as behind cover, and the single `not_visible` code this project split in
review round 2 would have concealed that distinction.

Two further things established before running it:

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

### 6.1 No-attacker soak — a1 ✅

The soak **walks**. A stationary fleet cannot produce a drift-driven false positive at all: the
odom residual the old gate watches and the observation-vs-claim disagreement the belief layer
watches both only move when the dogs do, so a soak that parks three robots and waits proves
nothing. This one flies the same out-and-back legs the rest of the matrix uses, and counts sim
time from `dist.csv` — the clock every other number here is on — rather than wall clock divided
by an assumed RTF.

It asserts on **two** counters, because they are different code paths and only one of them is an
eviction. A detector block that has not yet aged into an eviction is *already* a false positive,
so counting evictions alone would miss precisely the near-misses this criterion exists to find.

| run | arm | sim walked | laps | `EVICT robot` | `REJECTED AgentState` |
|---|---|---|---|---|---|
| `d_0801_072156` | **a1** (gate2) | **675.3 s** | 8 | **0** | **0** |

Zero and zero, over the whole log rather than only the soak window. **Criterion 3, as written,
holds for a1** — it asks about spurious *detector verdicts*, and there were none.

⚠️ **It is not a statement that the fleet held separation, and it must not be quoted as one.**
Two things were wrong with reading it that way, and both are mine:

1. **The run's safety summary predates the soak.** `RESULT` / `TRUE BODY GAP` print *before* the
   soak block, so the a1 run's cheerful "0.257 m worst" is stamped 07:24:28 and describes the
   outbound-and-home leg only. The soak then walked until 07:36:44. Reading every logged row of
   that window instead: **worst centre distance 0.8815 m, worst true body gap 0.1878 m.**
2. **Every soak dipped below the abort threshold; only some got caught.**

| soak | sim | worst `min_pair` | rows below the 0.90 abort | worst **true body gap** | verdict |
|---|---|---|---|---|---|
| a1 `d_0801_072156` | 802.5 s | 0.8815 | 20 / 13828 | **0.1878 m** | passed |
| a2 #1 `d_0801_075028` | 668.6 s | 0.8624 | — | **0.1832 m** | partitioned (§6.2) |
| a2 #2 `d_0801_082228` | 234.6 s | 0.8415 | 21 / 4163 | **0.2421 m** | aborted on the guard |

The guard samples once per **5 s** poll; `dist.csv` is written at **20 Hz**; the excursions last
about **1 s** (a1's ran t = 317.95–318.90). So which run trips the guard is mostly which polls
happen to land — a1's excursion fell between two of them. The three runs' separation behaviour is
the same; their verdicts are not.

This is exactly the hazard §0 rule 2 exists for, and the harness was still sampling. `d_run.sh`
now prints the soak's own worst separation over every logged row. Deliberately **reported, not
enforced**: turning it into a verdict would re-score every earlier run in the matrix against a
stricter rule than the one it was run under, which is an owner decision.

**And read §6.2 before quoting a1 as a clean pass on the eviction question too** — the a2 soak
found a partition mechanism that a1 escaped by four slots.

### 6.2 The a2 soak partitioned the fleet with no attacker in it — and it was not the belief layer

`d_0801_075028`, `ARM=a2 NO_KILL=1`. Two things it was **not**:

- **Not the belief layer.** `L_total` sat flat at **4.600** against `l_evict = −9.20` for the
  entire run. The accumulator never convicted anybody, before or after.
- **Not the WBC.** Zero `Deactivating` lines in `gazebo.log`, unlike `d_0801_074359`.

What actually happened, from `stats.csv`:

| sim t | event |
|---|---|
| 355.622 | all three at `achieved_rounds = 20`, healthy |
| **355.724** | **all three time out in the same slot, each on a *different* barrier phase** — robot1 waiting on `z`, robot2 on `xi`, robot3 on `xi` |
| 355.8 – 356.7 | the phases **lock in**: r1 pinned on `z` at the full 20 ms every slot, r3 on `xi`, r2 on `state`. They never resynchronise |
| ~356.7 | robot2's silence counter reaches 10 first → `EVICT robot1`, `EVICT robot3` |
| ~356.8 | robots 1 and 3 reject robot2's `AgentState` — *"its roster excludes us"*, which is by design |
| ~357.0 | robots 1 and 3 `EVICT robot2 (silent 3 slots)`. The split is now self-sealing |
| 357.156 | robots 1 + 3 resume at 20 rounds as a pair; robot2 runs alone at 0 for the next 117 s |

**It is not clock drift and not the mailbox prune window.** Slot counters stayed in exact lockstep
— `cycle == t / TS` for all three agents, checked at 11 points between t = 300 and t = 470 — so
the 8-slot `pruneOld()` window never came into it.

**And this is what it does to the a1 result.** Both soaks stall fleet-wide; they differ only in
how long:

| soak | episodes where all three were at `achieved_rounds = 0` | total slots | longest |
|---|---|---|---|
| a1 (`d_0801_072156`) | 2 | 7 | **0.50 s / 6 slots** |
| a2 (`d_0801_075028`) | 4 | 22 | **1.20 s / 13 slots** |

The eviction rule trips at **10 slots**. So a1 did not pass because it is immune to this — it
passed with **four slots of margin** on a mechanism that would have partitioned it too. Reporting
the a1 PASS without this table would be reporting the margin as if it were the mechanism.

`n = 1` run per arm, so **no rate is claimed** and nothing here says the belief layer makes stalls
longer. Both soaks ran the *same binaries* — every change made this session is in shell — so the
only configured difference is `obs_gate2`, and one run each cannot separate that from luck.

**The defect is pre-existing and belongs to the G4 barrier, not to this branch.** One missed 20 ms
exchange is enough to leave three agents waiting on three different phases; from there each looks
silent to every other, and the *silence* rule — which assumes silence is a property of one peer —
resolves a *correlated* stall by letting whichever counter trips first evict everybody else.
Registered as a finding rather than fixed here: it is upstream of the trust layer, and a fix is a
change to the consensus barrier, which this project's rules put behind a derivation with the owner.

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

## 8. Communication cost (G5) ✅

*Pending.* Folded in here rather than run separately, because the trust layer adds fields to
`AgentState` and measuring G5 first would have measured a system about to change.

**Two questions, and only one of them is an A/B.** `beliefStep()` is the only producer of
`ev_peer`/`ev_pos` and it runs only when `obs_gate2 = true` (`admm_agent_node.cpp:1405`), so `a1`
broadcasts empty evidence arrays for the whole run and the `a1`/`a2` soak pair isolates the
defence exactly. But the two questions it isolates want different instruments:

- *What does the evidence payload cost?* — **closed form**, because `wire_bytes()` is
  deterministic in the array lengths. No run needed, and no run can beat it for precision.
- *Does the defence cost anything else?* — extra messages, extra ADMM rounds, retries, a fatter
  `xibar`. **That** is what the A/B is for, and nothing else can answer it.

Reporting the first from the A/B would have been a mistake, and the size of that mistake is
measurable — see the noise floor below.

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

#### 8.1 What the payload costs, in closed form

`dds_transport.hpp`'s `wire_bytes()` counts CDR payload — 8 B per `float64`/`uint64`, 4 B per
`int32`, 4 B per array length prefix, 1 B per `bool` — and excludes RTPS/UDP framing. At N = 20,
a 3-robot roster, and both peers observed:

| `AgentState` variant | bytes | vs. pre-spec |
|---|---|---|
| pre-spec peer (no evidence fields at all) | 1037 | — |
| **a1** — fields exist, arrays always empty | 1053 | **+16** (two length prefixes + `ev_slot`) |
| **a2** — 2 peers observed | 1093 | **+56** |

One `AgentState` per agent per cycle at `TS = 0.10 s`, and each agent receives its two peers':

- **tx: +40 B/cycle = +400 B/s** per agent
- **rx: +80 B/cycle = +800 B/s** per agent

The 16 B an `a1` agent pays is the price of the fields *existing*; it is unavoidable once the
message carries them, and it is why `a1` is the right baseline rather than a pre-spec build.

#### 8.2 What that is against, and why the A/B cannot resolve it

Measured from `d_0801_072156` (a1 soak, 772 s of recorded intervals, `achieved_rounds` mean 19.91):

| | robot1 | robot2 | robot3 |
|---|---|---|---|
| tx | 789 123 B/s | 789 223 B/s | 788 941 B/s |
| rx | 799 642 B/s | 799 720 B/s | 799 467 B/s |

So the evidence payload is **+0.051 % tx / +0.100 % rx**. Traffic is dominated by `EdgeXi`/`EdgeZ`
at ~3.9 kB per ADMM round, 20 rounds per cycle, 10 cycles/s.

And that ratio is the finding, because it is also the reason the A/B cannot measure it: **a shift
of 0.01 in mean `achieved_rounds` moves the total by ~390 B/s** — the same size as the entire
effect. The three agents in a single run already differ by 0.02 in that mean. Quoting an `a1` → `a2`
byte delta as "the cost of the defence" would have been quoting ADMM convergence noise with a
decimal point on it.

**Method (unchanged, and it matters).** `bytes_tx`/`bytes_rx` are read-and-reset by `take_bytes()`
at every `publishStats`, so each `stats.csv` row carries the traffic since that robot's previous
row. The rate is a sum over intervals, not a first-to-last span: the first row holds everything
since node start, and `g5_logger`'s subscriptions do not all establish at once — in one run two
robots had a 37.5 s hole while the third started 37.6 s in, which a span-based rate turned into a
spurious 1.4× difference between two identically-configured agents. Intervals longer than 1 s are
dropped from both numerator and denominator.

#### 8.3 A/B result — does the defence cost anything beyond its payload? **No.**

Both soaks ran the **same binaries** (every change made in this session is in shell), nothing is
evicted in either, nobody is deafened, and the only configured difference is `obs_gate2`. Compared
over a **matched sim-time window, t ∈ [100, 350] s**, so that both cover walking laps and the a2
run is read entirely from before its partition (§6.2):

| | a1 `d_0801_072156` | a2 `d_0801_075028` | Δ |
|---|---|---|---|
| tx (mean of 3 agents) | 787 955 B/s | 790 096 B/s | **+2141** |
| rx (mean of 3 agents) | 798 479 B/s | 801 006 B/s | **+2527** |
| `achieved_rounds` (mean) | 19.861 | 19.912 | +0.0514 |

The raw Δ is **5.4× the payload**, and almost all of the excess is convergence, not evidence:

| component of the +2141 B/s tx | B/s | share |
|---|---|---|
| evidence payload (§8.1, closed form) | +400 | 19 % |
| `achieved_rounds` +0.0514, at 3914 B/round | +2011 | 94 % |
| **unexplained residual** | **−269** | **−0.034 % of baseline** |

The residual is **negative and 0.03 % of the total** — indistinguishable from zero at this
instrument's resolution. So the answer to the question the A/B exists for is: the belief layer
costs its payload and **nothing else**. No extra messages, no extra rounds attributable to it.

And this is the concrete form of the warning in §8.2. Had the raw +2141 B/s been reported as "what
the defence costs on the wire", it would have overstated it by a factor of five, and four fifths of
the quoted figure would have been an ADMM convergence difference of 0.05 rounds.

---

## 8b. Five dogs ✅

Owner-approved, and the reason it is worth the runs is criterion 5. The smear arm half-failed at
N = 3 — the target was never blocked (3/3) but the smearer was never convicted — and spec §10
declares that a **blind spot of N = 3 rather than a tuning failure**: every close approach has
only two possible observers, so a symmetric one-against-one disagreement has no third party to
break it, and relayed evidence plus `trust_total`'s sum clamp only become non-trivial at N ≥ 4.
N = 5 is the smallest fleet that can test whether that blind spot actually closes.

**Shape.** `V5` is a centroid-centred regular pentagon, circumradius 1.25 m:

| | value | why it is that value |
|---|---|---|
| side | **1.4695 m** | clears `D_MIN` = 1.30 and the 1.40 m geometry floor, so slot targets do not fight the inter-agent CBF |
| longest diagonal | **2.3776 m** | inside `obs_range` = 4.0, so **every pair is mutually observable** — which is the entire point of running N = 5 |
| centroid | exactly (0, 0) | `centroid_slot_targets` translates the shape without reassignment on the first goal |

Run in the **empty world**: at 2.38 m across, the pentagon cannot pass a 2.20–2.30 m plum-post gap.

**Regression gate before any of it** (the five-dog code had been written and deliberately left
unbuilt until this point):

| check | result |
|---|---|
| oracle tests | **47 passed** (44 + 3 for the generated columns and the pentagon) |
| `ctest` | **6/6** |
| G1 distributed-vs-centralized | **`worst max\|delta\| = 0`, mismatches = 0** |

**First attempt `d_0801_080532` — aborted by a harness bug, and the fleet was blameless.** The
coordinator resolved `shape=V5 -> plan for 5 dog(s)`, all five stood and trotted, and the pentagon
held to within 3 mm of design. The collision guard aborted it anyway on `pairwise 0.8108 < 0.90`:

| | value |
|---|---|
| guard's reading | 0.8108 |
| `x4` at that instant | **0.8108** |
| worst `min_pair` over the entire run | **1.4468** |
| worst true body gap (`phys_gap`) | **0.6007 m** |

`min_pair` is column 8 with three robots and column 12 with five; the reader hardcoded 8, which at
N = 5 is a robot's x coordinate. It cannot fail loudly — a coordinate is a plausible-looking
number — so the guard fires whenever any robot walks through x < 0.9. Fixed by locating the column
by header name (§9 #7).

**Second attempt `d_0801_080904` — PASS.** Same configuration, fixed guard.

| | value |
|---|---|
| verdict | **`D PASS`** — out to 9 m and home, `arrive_dist` 0.382, `mission_denied` 0 |
| coordinator | `shape=V5 -> plan for 5 dog(s)`; `plan covers 5 robot(s)` on **both** legs |
| worst centre distance | 1.2718 (1195 logged rows) |
| worst true body gap | **0.4432 m** (1059 rows) — contact is 0 |
| `EVICT` / `REJECTED` over the whole run | **0 / 0** |

Worst `min_h` is **−0.0726**, i.e. the pentagon dips just inside the `D_MIN` = 1.30 barrier while
turning — the same thing the three-dog V does, and 0.443 m of real body clearance away from
touching anything. Nothing here needed the collision guard's help.

**What this establishes, and what it does not.** It establishes that N = 5 *runs*: the roster,
the generated degraded columns, the pentagon, the coordinator's size guard, the observation range
and both loggers all work at five, and the belief layer produces no false positive over a full
out-and-back. It does **not** yet say anything about the N = 3 blind spot, which needs the smear
arm.

### 8b.1 The smear result — and the N = 3 explanation on record is **wrong**

`d_0801_081301`, `ARM=smear` at N = 5. robot2 keeps its own position honest and fabricates a
0.30 m (0.4243 m displaced) sighting of robot1, with **three** third-party observers that do not
exist at N = 3.

| | result |
|---|---|
| harness verdict | **`D PASS`**, `arrive_dist` 0.266, worst true body gap **0.433 m** |
| criterion 5(a) — smeared peer never blocked | ✅ `plan covers 5 robot(s)` on both legs; robot1 never evicted or rejected |
| criterion 5(b) — smearer convicted | ❌ **no**, and not for the reason on record |
| smear-check coverage | **100 %** — 8667–8715 relayed reports per agent, every one independently checkable |
| `n_refuted` | **0** across all five agents (correct: a 0.42 m displaced position is still geometrically *plausible*) |

**The throttled log cannot answer this, and nearly cost the diagnosis.** `belief robotJ …` is a
single `RCLCPP_INFO_THROTTLE` statement, so within each 1 s window only the first peer in
iteration order prints — always the agent's lowest-id peer. Every agent's view of robot2 is
invisible in the log except agent1's, and agent1 is the smeared peer, which *skips* reports about
itself (`if (j == self_id_ || j == i) continue`). The answer comes from the unthrottled per-peer
telemetry in `stats.csv`'s `tel` column (`peer:residual:L_self:total:abstain`).

**Belief in the smearer, minimum over each run:**

| run | observer | first-hand `L_self` | `total` | threshold |
|---|---|---|---|---|
| N = 3 `d_0801_054738` | agent3 (third party) | **−11.200** | −8.806 | −9.2 |
| N = 3 `d_0801_055013` | agent3 (third party) | **−11.200** | −7.433 | −9.2 |
| N = 5 `d_0801_081301` | agent3 | **−11.200** | **−6.600** | −9.2 |
| N = 5 | agent4 | **−11.200** | **−6.600** | −9.2 |
| N = 5 | agent5 | **−11.200** | **−6.600** | −9.2 |

**The smear check works perfectly.** Every third party drives its first-hand belief to the floor,
at every fleet size. What blocks conviction is the *total*.

**And more witnesses made it worse.** At N = 3 the smearer came within **0.394** of conviction; at
N = 5 it sits **2.600** away. Adding two independent witnesses moved it 2.2 units *further* from
being caught.

#### Why — three constants, and an inequality that cannot be satisfied

```
L_self floor            = l_evict − clamp_step = −9.2 − 2.0 = −11.2   (trust.hpp:273)
max relay sum           = +l_max               =        +4.6          (trust.hpp:298)
worst achievable total  = −11.2 + 4.6          =        −6.6
conviction threshold    = l_evict              =        −9.2
                          −6.6 > −9.2  ⟹  NOT CONVICTABLE
```

`trust_total` adds the first-hand belief to a relay sum clamped to ±`l_max`. The smearer's *own*
position is honest, so every agent's sighting of it corroborates its own claim and every relay
entry about it is **positive**. First-hand suspicion saturates at the floor; corroboration of its
honest self-report is then added on top, up to +4.6.

The measured N = 5 minimum is **−6.600**, matching `floor + l_max` **exactly, to three decimals**,
on all three third parties independently.

So criterion 5(b) **cannot pass with these constants, at any fleet size**. It is not a blind spot
of N = 3 — larger N makes it strictly tighter, because more reporters saturate the +4.6 cap
sooner. At N = 3 the relay sum happened not to saturate, which is the only reason those runs got
within 0.4 of the threshold and made the fleet-size story look plausible.

**A peer that lies only about others, while keeping its own position honest and corroborated, is
structurally immune to conviction.** Making it convictable needs `l_max ≤ clamp_step` (2.0, from
4.6), or a floor below −13.8 — a calibration change to the system under test, so it is registered
as an owner decision rather than applied here. The `l_max = 4.6` cap exists for a stated reason
(trust.hpp:289–294: stopping N−1 hearsay accusers from evicting on pure hearsay at N ≥ 4), so the
two requirements are in direct tension and the resolution is a design call, not a retune.

---

## 9. What the harness got wrong, and when

Every number above depends on the harness being right about what it was measuring. It was not,
seven times, and each correction changed which runs counted. Recording it here because a results
table that hides its own instrument history is not checkable.

| # | what was wrong | how it was found | effect |
|---|---|---|---|
| 1 | `inject_odom_fake` was **never set at all**. Task 10 built both halves of the dual-channel forgery; the arm selector wired only the claim half. | reading the arm selector against Task 10's report before the first run | every arm would have faced a single-channel attack — the one A1 catches trivially. Acceptance criterion 1 could neither have passed nor failed honestly. |
| 2 | Three `ros2 param set` calls do not land together. Measured skew **2.78 s**. | pilot run `d_0801_035833`: agent1 blocked the attacker 0.61 s into its own single-channel window | "A1 caught the lie on 1/2 survivors" was measuring the harness. Fixed by `arm_attack.py` (one process, one discovery, all requests back to back → **93 µs**). |
| 3 | `KILL_AT_X` was decorative — the attack fired 2.8–3.9 m past where it was staged, by an amount that depends on RTF. | comparing the requested and actual trigger positions across runs | where the attack lands decides what the run tests, so runs were not comparable. Two causes: a pre-arm block that ran while the fleet walked, and `fleet_centroid` costing ~15 s per call. Now fires at **x = 3.125** for a requested 3.0. |
| 6 | `form_half_extent` read the latched `/formation/plan` **once**, with a 6 s timeout, and printed **`0`** when the read came up empty — which no distance is ever below, so `walk_until`'s acceptance clause was silently dead. | the a2 soak sat 25 polls at 0.516 m against a half-extent that had accepted an identical **0.517 m** in the a1 run fifteen minutes earlier | a converged fleet standing on station was scored **"never reached the outbound goal"**. The read fails by *run state* — a fresh subscription has to finish discovery, measured past the timeout after a daemon stop — so it looked fixed right up until it wasn't. Now: 3 attempts, print **nothing** on failure, and `walk_until` aborts as infra rather than evaluating a clause it cannot evaluate. |
| 7 | `min_pair` was read from **column 8 of `dist.csv` by position**. That is correct for three robots and is a robot's **x coordinate** for five. | the five-dog run aborted on `pairwise 0.8108 < 0.90` at the instant `x4` was 0.8108, while the run's worst real `min_pair` was 1.4468 | the collision guard cannot fail loudly on this — a coordinate is a plausible-looking separation — so it fires whenever any robot walks through x < 0.9. A pentagon holding shape to 3 mm was failed for a coordinate. Now located by header name; verified unchanged on a three-robot log and correct on a five-robot one. |
| 5 | The WBC-deactivation guard ran **after** phase 8's plan-coverage assertion, so whichever check sat earlier in the file wrote the diagnosis. | `d_0801_074359`: robots 2 and 3 deactivated 17 ms apart on `QP is infeasible`; the run reported `FleetPlan covers 1 robot(s), expected 3` | an **upstream** QP failure was reported as a defence result, exit 1, which the results protocol forbids re-running. Fixed by `wbc_guard()`: classify on whether the run contains an adversary at all (`NO_KILL` → infra; otherwise fail closed, because with an attacker present a felled robot could be the attacker's doing), and check it **first**. |
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

| `d_0801_073719` | a2 soak | **REJECTED — harness** | `form_half_extent` returned empty, `walk_until` read it as a half-extent of 0, and 25 polls at a converged 0.516 m were scored "never reached the outbound goal". The a1 run had accepted an identical 0.517 m fifteen minutes earlier. §9 #6. |
| `d_0801_074359` | a2 soak | **REJECTED — infra (reclassified after the fact, see note)** | Robots 2 and 3 hit `Premature homotopy termination because QP is infeasible` and their controllers deactivated **17 ms apart** at sim t ≈ 115.9, immediately after the phase-8 goal reversal. Their odom stopped, both agents logged *"inputs stale … holding OFF the wire so peers can evict me instead of waiting forever"* — by design — and agent1 evicted them for silence that was entirely real. **Belief never convicted anybody**: totals stayed 4.6–9.2 against `evict < −9.20`. Two robots genuinely fell over; no false positive. |

⚠️ **`d_0801_074359` exited 1, not 2, and the pre-registered rule says exit 2 is the only
automatic exclusion.** I am reclassifying it by hand, so it is worth being exact about what that
does and does not rest on. It rests on log evidence that is not a judgement call — an upstream QP
solver failing in `legged_stack`, which this project does not modify, in a run configured with no
adversary in it at all. It does not rest on the run having been inconvenient. The classifier
itself is now fixed (§9 #5) so that this decision is made by the script from the run's own
configuration rather than by me from its outcome; every later run is classified before anyone
sees what it produced.

| `d_0801_075028` | a2 soak | **FAIL — recorded as an outcome, not excluded** | The fleet partitioned with no attacker in it. One missed 20 ms exchange left the three agents waiting on three different barrier phases and they never resynchronised; robot2's silence counter tripped first and evicted the other two, who then rejected its roster by design and evicted it back. **Belief never convicted anybody** (`L` flat at 4.600 vs −9.20) and there was no WBC deactivation. Full mechanism in §6.2. Its first 355 s of clean three-agent traffic are what §8.3's A/B is computed from. |

| `d_0801_080532` | N=5 a2 `NO_KILL` | **REJECTED — harness** | Collision guard read `min_pair` from column 8 by position, which at N = 5 is `x4`. Aborted on `pairwise 0.8108 < 0.90` at the instant robot4 was at x = 0.8108; the run's worst real `min_pair` was 1.4468 and its worst body gap 0.6007 m. §9 #7. |
| `d_0801_080904` | N=5 a2 `NO_KILL` | **PASS** | `shape=V5 -> plan for 5 dog(s)`, both legs, `arrive_dist` 0.382, worst body gap 0.4432 m, zero evictions, zero blocks. §8b. |
| `d_0801_081301` | N=5 `smear` | **PASS (harness) / criterion 5(b) fails** | Smeared peer never blocked; smearer never convicted, and the telemetry shows why it never could be: `floor + l_max = −6.6 > −9.2`. §8b.1. |

| `d_0801_082228` | a2 soak (repeat) | **FAIL — genuine separation breach, not a partition** | Aborted at sim 234.5 on `pairwise 0.8425 < 0.90`, and this one is real: worst `min_pair` 0.8415, worst true body gap **0.2421 m**. Zero evictions and zero blocks in those 234 s, so no false positive. Does **not** give n = 2 on §6.2's partition — it failed earlier, for a different reason. First soak on the post-rebuild binary; `shape_for(3,3)` is unchanged and COL2 bit-identity is asserted by test. |

⏳ *Matrix attempts appended as they complete.*
