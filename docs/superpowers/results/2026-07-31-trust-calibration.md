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

## 2. Assertion: max residual vs `d_lie/2` (item 3)

`d_lie = 0.30` (unchanged), decision boundary `d_lie/2 = 0.15 m`.

- max residual 0.0890 m → **1.7× below** the boundary.
- p99 residual 0.0609 m → **2.5× below** the boundary.

**Verdict: usable, but flagged rather than rubber-stamped.** p99 gives a comfortable margin;
the single-sample max does not — 1.7× is real headroom but is not what "≪" (far below) normally
means, and the brief for this task explicitly asks to report a marginal result rather than paper
over it. The extreme-value cross-check above is the mitigating fact: the max is fully explained by
the noise tail of 10,704 independent draws, not by an unmodeled bias, and the median-max grows
only as `σ·√(2·ln n)` — logarithmically with sample count. Even at n = 10⁶ (a very long multi-run
campaign) the predicted median-max is ≈ 0.105 m, still 1.4× below 0.15 m. So the margin shrinks
slowly and stays positive for any realistic mission length; it does not degrade catastrophically
with more data. **Not raising `d_lie`**, per instruction — if this margin needs to be more
comfortable, the correct lever is a tighter `sigma` (a claim about a better sensor) or a larger
`d_lie`-independent safety factor in a future revision, not this run's job to decide.

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

**Arithmetic and verdict.** The instruction is: accumulated spurious refutation over a full
mission should stay an order of magnitude below `|l_evict| = 9.2`, i.e. under ≈ 0.92. Measured rate
is 0/agent/min, so accumulated harm is 0 — trivially satisfies the bound. `refute_ratio` stays at
**0.1** (unchanged; there is no measured nonzero rate to derive a different number from). As a
sanity bound rather than a measurement: a sustained, undecayed 1 event/agent/minute (already ≫ what
was observed) would need roughly 46 consecutive events with zero offsetting first-hand honest
credit to reach `l_evict` on its own (`9.2 / 0.2 ≈ 46`), i.e. ~46 minutes at that rate — and the
first-hand loop is earning honest credit in parallel the whole time, since `extra_llr` is combined
into the *same* `trust_step_self` call as ordinary first-hand evidence, not applied on a separate,
uncontested channel.

**Limitation, stated plainly:** 10.25 robot-minutes is a small sample for a rare-event rate.
Zero observed events over that window is consistent with keeping `refute_ratio` at 0.1, but it
cannot rule out a rare grazing geometry this specific traverse never produced (single pass through
3 of 7 plum rows). A longer run (`plum_dense`, or the full 18 m plum traverse) would tighten this;
that is future work, not something this session's budget covered.

## 4. `obs_range` invariance (item 5)

Combined peer-to-peer distance, every pairwise sample, both runs:

| | value |
|---|---|
| n | 26,304 |
| min | 1.094 m |
| mean | 1.350 m |
| p99 | 1.943 m |
| max | 2.312 m |

Configured `obs_range = 4.0 m`. Observed maximum (2.312 m, during the plum-arena single-file
weave through the pillar field) sits **1.7× below** the configured range. Formation spacing
(1.40 m nominal) and the plum row pitch (2.2–2.3 m) both show up directly in this distribution —
mean 1.350 m matches nominal spacing, and the max sits right at the plum row-pitch scale, exactly
as the brief predicted.

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
  six values (plus an invalid one) produce the correct `obs_gate2`/`detection_log_only`/
  `expect_evict` combination; `smear`/`duty`/`forged_obs` were separately dry-run with `pset`
  mocked to confirm each fires the correct Task 10 parameter name(s) and values
  (`inject_fake_evidence_target`+`inject_fake_evidence`, `inject_duty_cycle`,
  `inject_forged_obs` respectively) in the same trigger burst as the position lie.
- **a1, smear, duty, forged_obs NOT run live in Gazebo** this session — budget (each full arm run
  is ~10 min; two calibration runs plus the bugfix investigation already consumed the available
  window). Exact commands to run them are in the task report.

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
