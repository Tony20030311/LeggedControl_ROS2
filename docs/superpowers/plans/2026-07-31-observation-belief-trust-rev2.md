# Observation-sourced Belief Trust Layer — Plan, revision 2 (Tasks 3 onward)

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax.

**Supersedes** Tasks 3-9 of `2026-07-31-observation-belief-trust.md`. Tasks 1 and 2 in that file are COMPLETE and unchanged (commits `d116a2c`, `e7883b8`, `7fb4904`, `e7beb87`).

**Why a revision:** four parallel audits and three adversarial verifications found the original plan would have produced an experiment that could not fail, and a safety argument written in the wrong variable. Full findings: `.superpowers/sdd/2026-07-31-observation-belief-trust/audit-actions.md`. Design of record: `docs/superpowers/specs/2026-07-31-observation-belief-trust-design.md` (revision 2, commit `5edd0bd`).

**The reframing that drives this revision:** the pairwise HOCBF is evaluated on the peer's CLAIMED position, so the 0.433 m safety margin is spent by lie DISPLACEMENT, not by anyone's ground speed — a stationary 0.30 lie already ate the measured body gap to 0.038 m. Therefore physical safety is moved OFF the detection path entirely (Task 4), and the belief layer only decides who to cooperate with.

## Global Constraints

Everything in the original plan's Global Constraints still binds. Added:

- **`admm_core/` is untouchable**, including `rti.cpp` (HOCBF coefficients, `edge_h`, gradients) and `D_MIN`. The 44 Python oracle tests pin that math. Anything this revision changes happens where data is ASSEMBLED, never inside the constraint builder.
- **G1 bit-identical parity is the gate for Task 4.** If `test_distributed_parity` stops reporting worst max|Δ| = 0 with no observations present, Task 4 is wrong — stop and report, do not adjust the test.
- **Two comments in the shipped code certify a check that was deleted.** `agent_core.hpp:54-57` and `admm_agent_node.cpp:793` claim Gate 1 chains `xnow` to `xibar`; `agent_core.hpp:148-157` says in its own words that the check "had to go". Fix both comments in whichever task touches them first.
- **Detection latency now has a physics budget**, not a free-form number: `(N_debounce + patience) · TS · v_div` must fit inside 0.433 m. Tests assert the physics, not a slot count.

---

### Task 3: Trust parameters in the node

**Files:** Modify `legged_admm_fleet/src/admm_agent_node.cpp` (parameter block ~line 83-101, member block ~line 1116); modify `legged_admm_fleet/test/test_trust.cpp`.

**Interfaces produced:** node member `admm::TrustParams trust_;` from ROS parameters `trust_lambda`, `trust_clamp`, `trust_credit_ratio`, `trust_l_max`, `trust_l_evict`, `obs_sigma`, `trust_d_lie`, `obs_noise_seed`. Members `std::map<int,double> l_self_;` and `std::map<int, std::map<int,double>> l_relay_;`.

- [ ] **Step 1: Add the invariant tests** to `test_trust.cpp` and call them from `main()`:

```cpp
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
    // The physics, not a slot count. D_MIN 1.3 minus the 0.867 m contact line leaves 0.433 m,
    // and the accumulator plus one slot of eviction patience must fit inside it at the
    // divergence rate the design claims to cover. Raising the assert to make this pass is
    // forbidden: it would ship a detector slower than the damage it prevents.
    int slots = 0;
    for (double L = p.l_max; !trust_fences_peer(L, p) && slots < 200; ++slots)
        L = trust_step_self(L, -p.clamp_step, p);
    const double v_div_covered = 0.433 / ((slots + 1) * TS);
    std::printf("detection: %d slots, covers divergence up to %.3f m/s\n", slots, v_div_covered);
    assert(v_div_covered >= 0.25);
}
```

- [ ] **Step 2: Run and confirm the physics test fails or passes on its own merits** (build+ctest command from the original plan). Report the printed `v_div_covered`. If it is below 0.25, report BLOCKED with the number — do not change the assert.

- [ ] **Step 3: Declare the parameters** after `resid_gate_`:

```cpp
        // Trust layer. Every default is derived (trust.hpp); obs_sigma is the only measured one.
        trust_.lambda        = declare_parameter<double>("trust_lambda", trust_.lambda);
        trust_.clamp_step    = declare_parameter<double>("trust_clamp", trust_.clamp_step);
        trust_.credit_ratio  = declare_parameter<double>("trust_credit_ratio", trust_.credit_ratio);
        trust_.l_max         = declare_parameter<double>("trust_l_max", trust_.l_max);
        trust_.l_evict       = declare_parameter<double>("trust_l_evict", trust_.l_evict);
        trust_.sigma         = declare_parameter<double>("obs_sigma", trust_.sigma);
        trust_.d_lie         = declare_parameter<double>("trust_d_lie", trust_.d_lie);
        obs_noise_seed_      = declare_parameter<int>("obs_noise_seed", 0);
        // The decision boundary is d_lie/2, not d_lie: that is where the log-likelihood crosses
        // zero. It must stay well under the 0.433 m buffer AND well above the measured residual,
        // or the detector either misses damage or convicts honest robots.
        if (!(trust_.d_lie / 2.0 < 0.433))
            throw std::runtime_error("trust_d_lie/2 must stay under the 0.433 m safety buffer");
        if (!(trust_.l_max < -trust_.l_evict))
            throw std::runtime_error("trust_l_max >= |l_evict|: hearsay could evict alone");
```

- [ ] **Step 4: Add `credit_ratio` to `TrustParams`** in `trust.hpp` and apply it in `trust_llr` (positive contributions scaled by `credit_ratio`, default 0.25). Add a test that an alternating lie/truth attacker is still convicted:

```cpp
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
```

- [ ] **Step 5: Full ctest, then commit.**

---

### Task 4: Conservative anchoring — physical safety without waiting for the detector

**Files:** Modify `legged_admm_fleet/src/agent_core.hpp` / `agent_core.cpp` (where `xnow`/`xibar` maps are assembled, `agent_core.cpp:126-129`); modify `legged_admm_fleet/src/admm_agent_node.cpp` (supplies the offsets); modify `legged_admm_fleet/test/test_trust.cpp`.

**Interfaces produced:** `admm::Vector2d admm::conservative_offset(const Eigen::Vector2d& self, const Eigen::Vector2d& claimed, const Eigen::Vector2d& observed, double dead_band, double rate_limit, const Eigen::Vector2d& prev)` in `trust.hpp`; `AgentCore::set_peer_offsets(const std::map<int, Eigen::Vector2d>&)`.

**This is the highest-value task in the revision.** Read spec §4.1 before starting.

- [ ] **Step 1: Write the failing tests** in `test_trust.cpp`:

```cpp
void test_offset_pulls_a_distant_claim_back_to_the_observation() {
    // A peer claiming to be further away than it is must be relocated onto what we see.
    const Eigen::Vector2d self(0, 0), claimed(3.0, 0), observed(2.0, 0);
    const Eigen::Vector2d d = conservative_offset(self, claimed, observed, 0.06, 0.055, {0, 0});
    assert(std::abs(d[0] + 1.0) < 1e-9);   // claim moves 1 m closer, onto the truth
}

void test_offset_ignores_a_claim_that_is_already_closer() {
    // A peer claiming to be nearer than we see is the ghost case: keep the claim, give it room.
    const Eigen::Vector2d self(0, 0), claimed(2.0, 0), observed(3.0, 0);
    assert(conservative_offset(self, claimed, observed, 0.06, 0.055, {0, 0}).norm() < 1e-12);
}

void test_offset_has_a_dead_band() {
    // Measurement noise must not jitter a safety constraint.
    const Eigen::Vector2d self(0, 0), claimed(2.0, 0), observed(1.97, 0);
    assert(conservative_offset(self, claimed, observed, 0.06, 0.055, {0, 0}).norm() < 1e-12);
}

void test_offset_is_rate_limited() {
    // Coming out of occlusion the offset can step. An unlimited step tightens a hard constraint
    // instantly and can make the QP infeasible — the born-violated failure this project knows.
    const Eigen::Vector2d self(0, 0), claimed(5.0, 0), observed(1.0, 0);
    const Eigen::Vector2d d = conservative_offset(self, claimed, observed, 0.06, 0.055, {0, 0});
    assert(d.norm() <= 0.055 + 1e-12);
}
```

- [ ] **Step 2: Run, confirm they fail** (`conservative_offset` undeclared).

- [ ] **Step 3: Implement `conservative_offset`** in `trust.hpp`. Semantics: return zero unless the observation is closer to `self` than the claim by more than `dead_band`; otherwise return `observed - claimed`, clipped so it moves no further than `rate_limit` from `prev`.

- [ ] **Step 4: Apply the offset to the WHOLE peer trajectory** in `agent_core.cpp`, right where `xnow` and `xibar` are assembled from received states. Add `xnow[j] += δ` on the position components AND `δ` to every knot's position in `xibar[j]`. Comment must say why translating the whole trajectory rather than substituting one point: the HOCBF is a three-point difference, so moving only `xnow` manufactures an acceleration that does not exist (a 0.5 m inconsistency needs ~21 m/s² to recover, outside the QP box), while a uniform translation leaves every difference unchanged.

- [ ] **Step 5: Verify G1 parity is bit-identical.** The loopback transport supplies no observations, so the offset map is empty and every δ is zero.

```bash
docker exec legged_stack bash -c "... && ./build/legged_admm_fleet/test_distributed_parity"
```

Expected: worst max|Δ| = 0. **If it is not zero, STOP and report BLOCKED** — the offset is leaking into the no-observation path, and no amount of test adjustment is an acceptable response.

- [ ] **Step 6: Full ctest + the 44 Python oracle tests** (command in the original plan's Task 9 Step 3c). Expected: all pass, because `admm_core/` was not touched.

- [ ] **Step 7: Commit.**

---

### Task 5: Observation channel

**Files:** `admm_agent_node.cpp` (the `peer_truth_` subscription, member block), `launch/admm_fleet.launch.py`, `trust.hpp`, `test_trust.cpp`.

**Interfaces produced:** `admm::ObsSample`, `admm::interp_at`, `admm::obs_noise(slot, i, j, sigma, seed)`; node members `std::map<int, std::deque<admm::ObsSample>> peer_truth_;`, `int obs_window_`, `int obs_noise_seed_`.

- [ ] **Step 1: Tests first** — `interp_at` brackets correctly; **`interp_at` returns `nullopt` when the target time is outside the sample window in EITHER direction** (the original plan's "returns the newest sample" behaviour invents evidence: if the observation stream stops while the claim advances, the residual grows with the silence and evicts an honest peer); `obs_noise` is deterministic in `(slot, i, j, seed)` and **changes when the seed changes**.

- [ ] **Step 2: Implement** both helpers in `trust.hpp`.

- [ ] **Step 3: Rewrite the subscription** with a publisher-GID pin:

```cpp
                [this, j](nav_msgs::msg::Odometry::SharedPtr m, const rclcpp::MessageInfo& info) {
                    // A remap gives no security: the resolved DDS topic is global and any process
                    // on the domain can write it. The DDS writer GUID arrives with every sample
                    // and the sender cannot forge it, so latch the first writer per topic and
                    // drop the rest. Same idea as the transport's impersonation check, stronger.
                    const auto& gid = info.get_rmw_message_info().publisher_gid;
                    auto& pinned = obs_gid_[j];
                    if (!pinned) pinned = gid;
                    else if (std::memcmp(pinned->data, gid.data, RMW_GID_STORAGE_SIZE) != 0) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "[agent%d] second publisher on robot%d's observation topic — dropped",
                            self_id_, j);
                        return;
                    }
                    ...
                }
```

Keep the remap (it is a useful interface seam for real perception) but **delete the comment claiming it makes the topic unforgeable**, and delete the "the publisher is checked below" promise wherever it appears without a check.

- [ ] **Step 4: Launch remappings** as in the original plan's Task 4 Step 6, with the corrected loop variables (`for i in ids`, `int(j) != int(i)`).

- [ ] **Step 5: Full ctest, commit.**

---

### Task 6: Belief replaces the single-shot verdict — WITHOUT deleting it

**Files:** `admm_agent_node.cpp` (`gate2` → `beliefStep`, call sites in `cycle()`).

- [ ] **Step 1: Keep both detectors selectable.** `obs_gate2 == false` runs the existing EMA path unchanged; `true` runs the accumulator. The original plan deleted the EMA, which would have made experiment arm A1 identical to A2 and its acceptance criterion unfalsifiable. Log one line at startup naming the detector compiled and armed.

- [ ] **Step 2: No evidence on a HOLD slot.** `peer_xnow_` is only assigned on the solved path (`agent_core.cpp:129`) while `last_seen_` advances for every accepted message (`dds_transport.hpp:407`), so on a barrier miss an honest peer's frozen claim differenced against a live observation fabricates up to 0.11 m of residual. "No fresh claim" is the same condition as "cannot see you": no evidence, no update.

- [ ] **Step 3: Align to the claim's own slot** — interpolate the observation to `last_seen[j] * admm::TS`, not the current slot.

- [ ] **Step 4: Decay only unblocked peers.** Blocking is terminal (`dds_transport.hpp:272-275` inserts into `blocked_` and nothing anywhere erases), so decaying a convicted peer's belief back toward neutral would make the belief disagree with the transport. Decay every peer that is not blocked, once per slot, before the evidence branch.

- [ ] **Step 5: Eviction patience 1 for belief-triggered blocks** (`evict_after_lying_` 3 → 1). The accumulator already did the debounce work that patience was invented for.

- [ ] **Step 6:** Full ctest, a 60 s honest-fleet Gazebo smoke run with zero evictions, commit.

---

### Task 7: Evidence sharing

**Files:** `admm_fleet_msgs/msg/AgentState.msg`, `agent_core.hpp`/`agent_core.cpp`, `dds_transport.hpp`, `admm_agent_node.cpp`, `trust.hpp`, `test_trust.cpp`.

- [ ] **Step 1:** Message gains `ev_peer`, `ev_pos`, **`ev_slot`** (spec §5.2). `ev_slot` exists because `send_state` is called at the top of `AgentCore::step` while evidence is produced after it returns — without the stamp, slot k broadcasts slot k−1's observations and every relayed term carries 0.055 m of bias.

- [ ] **Step 2: Validate on receive** — length agreement, `ev_peer.size() <= roster size`, ids in the roster, no duplicates. Drop the whole message otherwise (same semantics as Gate 1). Unbounded arrays deserialize on the thread that services the barrier.

- [ ] **Step 3: Cap the relay SUM, not each source** (`trust_total` clamps `Σ relay` to ±`l_max`). Per-source caps let three relayers cross the threshold at N ≥ 4 on pure hearsay. Add a test at N = 5.

- [ ] **Step 4: The smear penalty must land on the smearer.** When we have first-hand evidence of j, feed `‖z_i(j) − z_self(j)‖` through `trust_step_self` for **i**. Without this, a false report about j only pushes j down and costs i nothing — spec §5.2's "every dog is a direct checker" would have no implementation.

- [ ] **Step 5: Plausibility refutation needs a margin.** Refute a relayed observation only when it is outside `range + MAX_VX·2·TS + 3σ` (and likewise for obstacle radii), and weight the penalty well below `clamp_step`. Refuting on the bare geometry convicts honest reporters: in a 17-peg arena a sightline grazing a CBF circle is refuted by everyone but the sender, every slot, and seven such entries fence an honest robot — which then triggers mutual evasion boosts, the input that produced 179 OSQP failures and a frozen fleet.

- [ ] **Step 6:** Full ctest + oracle tests, commit.

---

### Task 8: The plan is part of the claim

**Files:** `admm_agent_node.cpp` (`beliefStep`), `agent_core.hpp` (stale comment), `test_trust.cpp`.

- [ ] **Step 1:** Add one term to the residual: the peer's own plan knot for THIS slot, broadcast k slots ago, differenced against the same observed position, with tolerance set by tracking error. The `xnow`↔`xibar` chain check does not exist (`agent_core.hpp:148-157`), so an attacker can be truthful about now and drift its plan 0.165 m per knot, eroding the effective standoff from 1.300 m to 0.354 m — past the 1.22 m knee-to-knee contact distance.

- [ ] **Step 2:** Correct the two stale comments that claim the chain check exists.

- [ ] **Step 3:** Test that a truthful-now, drifting-plan attacker is convicted. Commit.

---

### Task 9: A fenced robot does not get a vote

**Files:** `legged_admm_fleet/include/legged_upper_control/fleet_config.hpp` (`majority_excluded`), its unit test.

- [ ] **Step 1:** Test first — a roster set where a robot that is itself majority-excluded votes against an honest robot must not remove the honest robot.

- [ ] **Step 2:** One fixed-point pass: compute the excluded set, drop those robots' votes, recompute. This is what actually dissolves the stale-vote deadlock (spec §6); the original plan credited the forgetting factor, which never runs for a blocked peer.

- [ ] **Step 3:** Full ctest (this function is oracle-adjacent — confirm the Python parity suite still passes), commit.

---

### Task 10: The two attackers, wired for a fair comparison

**Files:** `dds_transport.hpp`, `admm_agent_node.cpp`, `launch/admm_fleet.launch.py`.

- [ ] **Step 1: `inject_odom_fake` must be runtime-settable**, not a launch parameter. As a launch parameter it is live from t=0 while the claim forgery fires at the trigger, so the old detector sees a 0.424 m residual against a 0.30 gate and evicts the attacker minutes BEFORE the attack — making arm A1's numbers meaningless.
- [ ] **Step 2:** Add `inject_fake_evidence` / `inject_fake_evidence_target` (smear) and `inject_duty_cycle` (lie on alternating slots) to the send path.
- [ ] **Step 3:** Add `inject_forged_obs` — the attacker also forges the observation channel, for the adversarial control arm.
- [ ] **Step 4:** ctest, commit.

---

### Task 11: Measure properly, then wire the arms

**Files:** `admm_agent_node.cpp` (CSV writer), `scripts/d_run.sh`, `docs/superpowers/results/2026-07-31-trust-calibration.md`.

- [ ] **Step 1: Unthrottled CSV.** One row per (slot, observer, target): `r`, `L_self`, `L_total`, abstain reason code, `v_div`. The 1 Hz throttled log drops ~19 of 20 slots and its throttle state is per call site, so peers alternate — deriving sigma from it samples the tail it is meant to bound. This is the sampling error that already hid a 0.526 m dip in this project.
- [ ] **Step 2: Calibration run**, then record max and p99 residual, the chosen `obs_sigma`, and one line stating this is the simulation floor.
- [ ] **Step 3: Assert the calibration is usable**: measured max residual must be far below `d_lie/2` = 0.15 m. If it is not, report — do not raise `d_lie`.
- [ ] **Step 4: Arm selector** `ARM=a0|a1|a2|smear|duty|forged_obs` setting `OBS_GATE2`, `DETECT_LOG_ONLY`, `LIE_LOGONLY` (a0 only, so the undefended arm survives its own breach), and the injection knobs — all fired in one `pset` burst at the trigger. Make the two-eviction assertion conditional on the arm expecting eviction, and report arrival DISTANCE rather than a boolean.
- [ ] **Step 5: Vary `obs_noise_seed` per run** and record it.
- [ ] **Step 6:** Commit.

---

### Task 12: Run the matrix and report it honestly

**Files:** `docs/superpowers/results/2026-07-31-trust-results.md`.

- [ ] **Step 1:** Arms a0 / a1 / a2, n ≥ 3 each, dual-channel forgery fired in one burst.
- [ ] **Step 2:** smear (with the relay-weighting-disabled positive control), duty, forged_obs arms.
- [ ] **Step 3:** Occlusion check — abstention logged POSITIVELY with a reason code, `L` numerically unchanged across the window, resumption afterwards, `visible()` false for both observers.
- [ ] **Step 4:** Stale-vote deadlock scenario; 600 s no-attacker soak with a walking goal (a stationary soak cannot show drift-driven false positives).
- [ ] **Step 5:** 44 oracle tests + full ctest.
- [ ] **Step 6:** Results document. Required per arm: physical body gap (`phys_gap_logger.py`, the only contact criterion), closest survivor pair over EVERY row of `dist.csv`, time-to-evict per survivor in sim time, measured `v_div`, arrival distance, `achieved_rounds`, WBC deactivations, `obs_noise_seed`, **and the communication cost**. **Table every attempted run including aborted ones with their cause.** No significance claims at n = 3. State the design envelope from spec §2 and whether the measured `v_div` fell inside it.

- [ ] **Step 7: G5 — communication measurement, folded in rather than run separately.**
  This is the project's outstanding gate (`CLAUDE.md`), and it belongs here: the trust layer adds
  fields to `AgentState`, so measuring G5 before it lands would measure a system about to change.
  Nothing new needs building — `dds_transport.hpp` already maintains `bytes_tx_` / `bytes_rx_` and
  computes per-message `wire_bytes()`, and `CycleStats` is already published every cycle and bagged.
  Report per arm: bytes/second per agent, mean `AgentState` payload with and without the evidence
  fields, and the resulting percentage overhead of the defence. That last number answers the
  question a reviewer will certainly ask — what does this defence cost in bandwidth — and it is the
  honest counterweight to the safety benefit.
