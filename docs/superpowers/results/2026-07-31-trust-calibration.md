# Trust-layer calibration (Task 11)

Two clean-flight runs, no attacker, `ARM=a2` (`obs_gate2=true`, belief accumulator armed,
`detection_log_only=false` so it is genuinely blocking — i.e. this measures the same code path
the arm matrix runs against, not a log-only shadow of it), `scripts/d_run.sh` (`NO_KILL=1`, the
existing "nobody silenced, assert zero spurious EVICT/REBUILD" regression branch).

| run | command | `obs_noise_seed` | span | log dir |
|---|---|---|---|---|
| A (open field) | `ARM=a2 NO_KILL=1 GOAL_X=3.0 VICTIM=2 d_run.sh` | 3855 | 61.0 s | `g2_logs/d_0731_181017` |
| B (plum arena) | `ARM=a2 NO_KILL=1 ARENA=plum GOAL_X=9 VICTIM=2 d_run.sh` | 28312 | 144.1 s | `g2_logs/d_0731_181302` |

Both `D PASS`, zero `EVICT`/`REBUILD` lines, `dist_summary.py` clean (contact 0.867 m: 0.0 s in
either run). Telemetry source: `g5_logger.py`'s `stats.csv`, reading `CycleStats.tel_*`/`n_refuted`
(Task 11's unthrottled per-cycle fields — see item 1/4 below), not the old 1 Hz throttled log
lines. Analysis scripts used are throwaway (not committed); the arithmetic is reproduced inline
below so it can be checked without them.

**This is the simulation floor only.** Per spec section 7, it excludes the attitude lever-arm term
(1° → 0.070 m, 3° → 0.21 m), inter-robot odometry drift (0.2–1.0 m over an 18 m path at 1–5 %),
and the surface-vs-origin term (~0.4 m) — all present on real hardware, none present here because
the stand-in observation channel *is* ground truth plus injected noise, with no physical sensor
in the loop.

**Revision note:** sections 2–5 below were corrected after independent review re-derived every
number from the raw logs. Section 1's numbers were confirmed byte-for-byte and are unchanged.
No calibration run was repeated; every fix below is either arithmetic (recomputed from data
already on disk) or a code/harness fix landed for future runs. See each section's "Correction"
note for what changed and why.

## 1. Residual (`obs_sigma`)

Combined over both runs, first-hand evidence rows only (`tel_abstain==0`):

| | value |
|---|---|
| n | 10,704 |
| max | 0.0890 m |
| p99 | 0.0609 m |
| mean | 0.0253 m |

`trust_.sigma` stays at its existing value, **0.02 m** — not changed. Reasoning: the telemetry
residual is `\|observed − claimed\|` where `observed = ground_truth + obs_noise(sigma)` and
`claimed` is the peer's own consensus state, which in an honest, well-tracked clean flight should
equal ground truth. So this residual is expected to be a pure draw from a 2D-Gaussian-noise
(Rayleigh) distribution with scale `sigma`, and that is exactly what was measured — cross-checked
against closed-form Rayleigh(σ=0.02) theory:

| statistic | measured | Rayleigh(σ=0.02) theory | formula |
|---|---|---|---|
| mean | 0.0253 | 0.02507 | `σ·√(π/2)` |
| p99 | 0.0609 | 0.06070 | `σ·√(2·ln 100)` |
| max (n=10,704) | 0.0890 | 0.0862 (median-max) | `σ·√(2·ln n)` |

All three match to within a few percent. This is the meaningful finding: it confirms the claim
tracks ground truth tightly enough that the residual is dominated by the injected sensor-noise
floor alone, with **no detectable extra bias** from consensus/plan lag or the one-slot
`ev_slot`/`interp_at` alignment leaking through. It does *not* mean the calibration is circular in
a useless sense — a broken alignment or a claim that lagged ground truth would have inflated these
numbers well past Rayleigh(0.02), and it did not.

## 2. Assertion: false-positive risk, and the blind spot it trades against (item 3)

**Correction (review round 2):** the first version of this section compared the measured max
residual (0.0890 m) against `d_lie/2 = 0.15 m` and reported a "1.7×" margin as if that were the
false-positive risk. It is not — `max/(d_lie/2)` is not a false-positive measure at all, and the
worry recorded here that a longer mission (n=10⁶) would erode that ratio to "1.4×" would have sent
a future reader chasing a problem that does not exist. Replaced below with the statistic that
actually answers "how likely is an honest peer to get falsely convicted", and — more
importantly — the statistic that answers the question this detector's shape actually raises.

**False-positive risk is astronomically small, not merely comfortable.** `trust_llr`'s raw
log-likelihood saturates at `±clamp_step` outside a narrow band around `d_lie/2`:

```
r_low  = (d_lie − 4σ²/d_lie) / 2 = 0.14733 m   (full +credit for any r below this)
r_high = (d_lie + 4σ²/d_lie) / 2 = 0.15267 m   (full −penalty for any r above this)
band width = r_high − r_low = 4σ²/d_lie = 0.00533 m  (5.3 mm)
```

A single noise-only excursion past `r_low` — i.e. anything that costs an honest peer even one
step of credit instead of full credit — has probability, under Rayleigh(σ=0.02):

```
P(R > r_low) = exp(−r_low² / 2σ²) = 1.64 × 10⁻¹²
```

That single excursion is worth one step of `−clamp_step = −2.0`. Recovering the exact accumulator
rule from `trust.hpp` (asymmetric decay: `L>0` decays each step, `L≤0` does not) and starting from
the clean-flight equilibrium (`L = l_max = 4.6`, the ceiling honest first-hand credit converges
to), conviction (`L < l_evict = −9.2`) needs **seven consecutive full-penalty steps with zero
intervening honest credit**:

```
4.6 → 2.37 → 0.26 → −1.75 → −3.75 → −5.75 → −7.75 → −9.75  (fenced)
```

Seven independent draws each at `1.64e-12` puts the compound probability far below anything worth
quoting a number for; even taking the single-step figure alone as "the" margin, **that is about
twelve orders of magnitude of headroom**, not 1.7×. `sigma`/`d_lie` are not close to a
false-eviction problem for an honest peer under this noise floor.

**The real finding is the blind spot, not the margin.** The 5.3 mm band above is where the
detector is *uncertain*; everywhere below `r_low = 0.1473 m` it gives **full, permanent, undecaying
honest credit** — mathematically indistinguishable from a perfectly honest report, forever, no
matter how much data accumulates (this is a per-residual function, not a sequential test with
memory of "how many times has this exact peer sat just under the line"). That means:

- **Any lie smaller than 0.1473 m is completely undetectable by this mechanism, at any sample
  size.** It isn't merely hard to catch statistically (as the incorrect "1.7×, eroding with n"
  framing implied) — it is invisible by construction.
- `0.1473 / 0.433 = 34%` **of the 0.433 m safety buffer is permanently available to a
  sub-threshold liar with zero detection risk.** This is the same shape of finding as this
  project's recorded "detection threshold exceeds the safety buffer" result (0.50 m gate against a
  0.433 m buffer) — a smaller fraction here (34% vs. that gate consuming the whole buffer), but the
  same defect class: a fixed decision boundary carves out an undetectable region inside the safety
  margin, and a thresholds document that reports only the noise-floor margin without naming that
  region is misleading about what the detector actually guarantees.

**Not raising `d_lie`**, per instruction, and not changing `sigma` — this session's job was to
measure and report, and the blind spot is a property of the `d_lie`/`sigma` *ratio*
(`d_lie/2 ± 2σ²/d_lie`), not a defect either number alone can be blamed for. Closing it needs a
design decision (tighter `d_lie`, a sequential/multi-slot test instead of a per-residual llr, or an
explicit acceptance that a sub-buffer creeping lie is out of scope), which is a spec-level call, not
this run's.

## 3. `refute_ratio` (item 4)

Definition: `refuted_llr(p) = -clamp_step * refute_ratio = -2.0 * 0.1 = -0.2` per event, applied to
the *reporter's* belief when a relayed report about a third peer is judged geometrically
implausible (`evidence_plausible()` false) — the spurious-refutation case this item calibrates,
since no attacker is present in either run.

**Measured: 0 refutations, on any of the 3 agents, across both runs** (10.25 robot-minutes:
3 robots × (61.0 + 144.1) s / 60). Run B (plum arena) is the meaningful one for this measurement —
it produced 134 real first-hand occlusion events (`tel_abstain==4`, peer genuinely hidden behind a
pillar from this agent's own viewpoint) — so grazing sightlines *did* occur during this run, and
the separate, more lenient `evidence_plausible()` check (margin-widened per Task 7 item 6,
specifically built to tolerate a report whose sightline merely grazes a pillar edge) never once
refuted an honest reporter even while real occlusion was common. Run A (open field, no obstacles)
is included for completeness but is a vacuous refutation measurement on its own — there is nothing
to graze — and is not the evidence for this section by itself.

**Correction (review round 2): zero events is not "nothing to derive from" — it is the numerator
of a real bound.** The denominator was already on record, just not in the telemetry: the throttled
"smear-check coverage" line in `admm.log` records `smear_checks_total_`, the cumulative count of
relayed-report entries that *passed* `evidence_plausible()` this run; adding `n_refuted` (already
in `CycleStats`) gives every entry the check was run on at all, refuted or not. Read from both
runs' final coverage lines:

| run | agent1 | agent2 | agent3 |
|---|---|---|---|
| A | 1,939 | 1,933 | 1,932 |
| B | 11,260 | 11,324 | 11,252 |

Total checks performed, both runs: **39,640** (`5,804` from run A + `33,836` from run B), against
**0** refutations. This denominator is now added to the telemetry itself (`CycleStats.
n_evidence_checked`, populated the same read-and-reset way as `n_refuted`) so a future run doesn't
have to reconstruct it from a throttled line.

**Rule-of-three derivation** (zero events observed in N trials → 95% upper confidence bound on the
per-trial rate is `3/N`, since `exp(−3) ≈ 0.05`):

```
N = 39,640 checks, 0 refutations
p_upper = 3 / 39,640 = 7.57e-5 per check                          (95% upper bound)
checks / agent / minute = 39,640 / 10.25 robot-min = 3,865
event_rate_upper = p_upper × 3,865 = 0.293 / agent / minute        (≤ 0.36, consistent w/ review)
llr_rate_upper = event_rate_upper × clamp_step × refute_ratio
              = 0.293 × 2.0 × 0.1 = 0.0585 log-odds / agent / minute
budget (order of magnitude below |l_evict|=9.2) = 0.92
margin = 0.92 / 0.0585 ≈ 15.7×
```

**This is a derivation, not a guess.** `refute_ratio` stays at **0.1** — the measured upper bound
on spurious-refutation harm (0.0585 log-odds/agent/min, 95% confidence) sits a full order of
magnitude (15.7×) below the 0.92 budget, satisfying the calibration instruction on real data rather
than by the absence of any data to check against.

**Limitation, stated plainly:** 39,640 checks is still a two-run, 10.25 robot-minute sample; the
95% bound (`3/N`) is itself conservative for small N and would tighten with a longer or denser run
(`plum_dense`, or the full 18 m plum traverse). That is future work, not something this session's
budget covered.

## 4. `obs_range` invariance (item 5)

**Correction (review round 2): the original n/p99 here were not reproducible from `dist.csv`.**
Root cause: `scripts/_fleet_bringup.sh`'s `trap 'kill -9 $DLPID ...' EXIT` (meant to stop the dist
logger when a run ends) was silently replaced by `d_run.sh`'s own later `trap ... EXIT` (bash traps
don't stack — the last one set for a signal is the only one that fires), so the dist logger was
never actually killed. It kept running past every run's exit, its 20 Hz timer re-writing the last
cached pose forever with no liveness check, appending stale/foreign data to old runs' `dist.csv`
files for as long as the container stayed up (in this session, until a subsequent run's Gazebo
happened to still be feeding the same globally-named `/robotN/controller/odom` topics). Two
different re-reads of the same files at two different wall-clock moments therefore gave two
different, both-wrong answers — this session's original 26,304/1.943 and reviewer's independent
40,017/1.819 are both artifacts of when each recomputation happened to run relative to the leak,
not of anything that happened during either scripted run.

**Fixed**: `$DLPID` added to `d_run.sh`'s own (surviving) trap, so this cannot recur.

**Corrected numbers**, computed over each file's row range at the moment its own script printed
`D PASS` (`head -1225`/`head -3953` of `dist.csv` for runs A/B respectively — reproducible by
anyone, since the files are append-only and these prefixes predate any contamination):

| | value |
|---|---|
| n | 15,528 (3,672 + 11,856) |
| min | 1.094 m |
| mean | 1.347 m |
| p99 | 2.013 m |
| max | 2.312 m |

`min`/`mean`/`max` are unchanged from every prior reading (they were never sensitive to the leak,
consistent with the reviewer's own note that those three agreed); only `n` and `p99` move, and the
conclusion is unaffected — `max` is the load-bearing number either way.

Configured `obs_range = 4.0 m`. Observed maximum (2.312 m, during the plum-arena single-file
weave through the pillar field) sits **1.7× below** the configured range. Formation spacing
(1.40 m nominal) and the plum row pitch (2.2–2.3 m) both show up directly in this distribution —
mean 1.347 m matches nominal spacing, and the max sits right at the plum row-pitch scale, exactly
as the brief predicted.

**Abstain-code elimination check (review round 2 "also" bullet).** `admm::visible()` is both a
range gate and an occlusion test; the original telemetry merged both into one "not_visible" code
(4), so the 134 events recorded in run B could not be told apart by code alone. `tel_abstain` now
splits them (4 = out-of-range, 5 = occluded — see `CycleStats.msg`), but the two calibration runs
predate that split. They don't need it, though: since the peer-distance ceiling measured above
(2.312 m) never approached `obs_range` (4.0 m) in either run, range **cannot** have been the cause
of any of the 134 events — by elimination, every one was occlusion, confirming the original
document's reading was correct in this instance, not merely assumed.

**Verdict: `obs_range` does not bind, in this clean-flight scenario.** Every value ≥ 2.312 m would
have produced identical `visible()` results here, so 4.0 m carries no weight in this result — it
is not false precision, it is deliberately generous headroom over a fictional "sensor" that is
ground truth with no physical limit.

**Caveat (per item 5's own instruction):** this was only checked under clean flight. The scenario
that could make `obs_range` bind — an attacker running away, or a survivor detouring wide around a
corpse keep-out — was not run this session (budget; see below). If a future arm-matrix run shows
peer distances approaching or exceeding 4.0 m, `obs_range` would need to be re-examined and, per
the brief, derived from a named sensor model rather than left at this placeholder.

## 5. Arms verified

- **a2 end-to-end, twice, in Gazebo**: `ARM=a2` correctly set `obs_gate2=true` at launch (confirmed
  both by `d_run.sh`'s own startup line and by `admm_agent_node`'s constructor log, "detector
  armed: belief accumulator"), telemetry populated correctly (`tel_abstain` covers codes 0, 2, 4
  in real operation — codes 1/3 do not occur in a clean run, as expected; nobody is ever blocked
  or has a dead channel), zero spurious eviction over both runs.
- **a0/a1/a2/smear/duty/forged_obs case-mapping**: verified in isolation (bash, no Gazebo) — all
  six values (plus an invalid one) produce the correct `obs_gate2`/`detection_log_only` pair at
  launch; `smear`/`duty`/`forged_obs` were separately dry-run with `pset` mocked to confirm each
  fires the correct Task 10 parameter name(s) and values (`inject_fake_evidence_target`+
  `inject_fake_evidence`, `inject_duty_cycle`, `inject_forged_obs` respectively) in the same
  trigger burst as the position lie — except `smear`, which (review round 2, I3') no longer sends
  the position lie at all, since the smear arm is defined as "the attacker's own position stays
  honest" and sending both convicted the attacker on first-hand residual before the smear channel
  ever accumulated anything.
- **Eviction attribution (review round 2, C1/C2)**: the eviction check no longer treats "an EVICT
  line appeared" as evidence any detector fired. `LIE_DEAF=1` (default) means the deafened victim's
  OWN silence timeout evicts the survivors from *its* roster; each survivor then hits
  `dds_transport.hpp`'s roster-exclusion path and blocks the victim right back — a path that fires
  in ~1.3 s regardless of `ARM`, including for `a0` and regardless of whether gate2/the belief
  layer ever saw the lie. The eviction check now verifies that roster-exclusion path as a plain
  infrastructure sanity check (should always fire given `LIE_DEAF=1`), then separately attributes
  by the block *reason* `dds_transport.hpp` actually logs (`gate2 odom residual` / `belief
  threshold`): hard-enforced for `a0` (must be absent) and `a2` (must be present); recorded, not
  enforced, for `a1` (a miss is the expected control-arm result) and `smear`/`duty`/`forged_obs`
  (these test degradation/evasion, and the run's own point can legitimately be that the attacker
  evades).
- **a1, smear, duty, forged_obs NOT run live in Gazebo** this session — budget (each full arm run
  is ~10 min; two calibration runs plus the bugfix investigation and this review's fixes already
  consumed the available window). Exact commands to run them are in the task report.

## 6. Regression evidence

- `ctest`: 6/6 passed (`distributed_parity`, `agent_timeout`, `agent_failover`, `corpse_keepout`,
  `false_signal`, `trust`).
- G1 parity, printed verbatim: `cycles=6 dogs=3 worst max|delta|=0 mismatches=0` /
  `G1 PASS: distributed AgentCore == centralized ADMMCoordinator (bit-identical)`.
- Python oracle suite: `44 passed`.

## Concerns

1. Both calibration runs are short relative to a full mission (61 s / 144 s vs. the multi-minute
   arm-matrix runs to come) — the residual and distance statistics are solid (thousands of
   samples), but the refutation-rate measurement (item 4) is inherently a rare-event count and
   0.92 confidence in "0 events" over 10 robot-minutes is weaker than the same claim would be over
   an hour.
2. `obs_range`'s "does not bind" verdict is scoped to clean flight only, as stated above.
3. Neither run exercises `abstain` code 1 (peer blocked) or 3 (no observation buffer) — both
   require an actual fault/attack to occur, which a clean-flight calibration run cannot produce by
   definition. The telemetry plumbing for those two codes is exercised by the existing unit tests
   in `test_false_signal.cpp` / `test_trust.cpp` (pure-function level), not by this Gazebo data.
4. **The 34% blind spot (section 2) is the headline finding of this document and is not fixed by
   anything in this task** — it is a property of the `d_lie`/`sigma` ratio the design chose, and
   closing it is a spec-level decision (tighter `d_lie`, a sequential test, or an explicit
   acceptance), not something a calibration run can correct.
5. **Neither run exercises `tel_abstain` codes 4/5 as distinct values** — the split was added in
   review round 2, after both runs; section 4's elimination argument establishes all 134 events
   were code-5-equivalent (occluded) by inference from the peer-distance ceiling, not by direct
   telemetry, since the runs predate the split.
6. `smear`/`duty`/`forged_obs`'s "recorded, not enforced" eviction check (section 5) means those
   three arms currently produce no PASS/FAIL signal on detection at all when actually run — by
   design, per review round 2 — but it also means a real regression in the belief layer's ability
   to catch a duty-cycled or forging attacker would not fail the harness. Whoever reads those
   arms' results needs to look at the recorded `N_BEL` count, not just the exit code.
