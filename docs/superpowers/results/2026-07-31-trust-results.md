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

⏳ *Measured `v_div` per arm: pending.*

---

## 3. Arms a0 / a1 / a2 ⏳

*Pending — 9 runs in flight.*

Required per run: physical body gap (survivor–survivor and attacker–survivor, per pair),
closest survivor pair over every `dist.csv` row, time-to-evict per survivor, measured `v_div`,
arrival distance, `achieved_rounds`, WBC deactivations, `obs_noise_seed`, arming skew,
communication cost.

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

## 6. Stale-vote deadlock and the no-attacker soak ⏳

*Pending.* The soak walks a goal; a stationary soak cannot show drift-driven false positives.

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

---

## 9. Every attempt, including the rejected ones

| # | arm | result | why it is listed |
|---|---|---|---|
| `d_0801_035833` | a1 | **REJECTED — harness** | Arming skew 2.78 s. The three forgery knobs were set by three separate `ros2 param set` processes; agent1's odom half landed 2.78 s after the claim half and it blocked the attacker 0.61 s into that single-channel window. Measured the harness, not the detector. Cause of the `arm_attack.py` rewrite. |
| `d_0801_040522` | a1 | **pilot, accepted as a wiring check only** | Arming skew 93 µs (agents' own log stamps). gate2 caught the lie on 0/2 survivors — the structural miss the design predicts. Not counted toward `n`: it ran before the `KILL_AT_X` staging fix, so its attack fired at x = 5.77 instead of the requested x = 3.0. |

⏳ *Matrix attempts appended as they complete.*
