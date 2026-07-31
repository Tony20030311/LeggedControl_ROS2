# Observation-sourced Belief Trust Layer — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the single-shot self-reported residual verdict (Gate 2) with a log-odds belief accumulated from evidence the judged peer cannot author, so a peer that forges both its consensus claim and its odometry is still caught.

**Architecture:** A slot-level admission layer at the current `gate2()` site. Each agent compares what it *observes* of a peer against what that peer *claims*, turns the residual into a log-likelihood contribution, decays it, and shares the raw observation so every agent runs the same deterministic accumulator. One belief number drives admission, keep-out geometry, and membership. The ADMM iteration, HOCBF QP, Laplacian coupling and `admm_core` math are untouched.

**Tech Stack:** C++17, Eigen, rclcpp (ROS 2 Jazzy), Gazebo Harmonic, plain `assert`/`main` unit tests registered with ctest, pytest oracle suite (regression only).

**Spec:** `docs/superpowers/specs/2026-07-31-observation-belief-trust-design.md`

## Global Constraints

- **Work only inside `src/legged_fleet/`.** `src/legged_stack`, `src/ocs2`, `src/elevation_mapping_cupy` are read-only.
- **Build inside the container, core-limited** (30 GB RAM will OOM otherwise):
  `docker exec legged_stack bash -c "source /opt/ros/jazzy/setup.bash && source /root/gridmap_ws/install/setup.bash && cd /root/legged_ros2_ws && MAKEFLAGS=-j8 colcon build --executor sequential --packages-select admm_fleet_msgs legged_admm_fleet --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"`
- **Do not modify `admm_core/` math.** New trust code goes in a new header; `fleet_config.cpp` may not change except where a task says so explicitly.
- **G1 bit-identical parity must keep passing** (`test_distributed_parity`, worst max|Δ| = 0).
- **Detection floor must stay below the safety buffer:** `d_lie` < 0.433 m (D_MIN 1.3 − contact line 0.867). Never raise it above.
- **No scenario-specific fallbacks.** Every discrete behaviour must be a reading of the one belief value. If a task seems to need a special case, stop and report instead of adding one.
- **Determinism:** no wall-clock or unseeded RNG in anything that affects a run's trajectory. Noise must be a pure function of `(cycle_id, observer, target)`.
- **Simulation ROS env for every run:** `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; export ROS_DOMAIN_ID=42` (cross-talk from domain 0 has already cost this project days).
- **Process cleanup only with `pkill -9 -x <comm>`** — never `pkill -f` (it self-matches and has killed Gazebo).
- **Uncommitted scaffolding exists** in the working tree (`admm_fleet.launch.py`, `admm_agent_node.cpp`: `obs_gate2`, `obs_range`, `inject_odom_fake*`, `peer_truth_`). Task 4 subsumes and cleans it; do not revert it before then.
- **Contact is measured with `scripts/phys_gap_logger.py`, never centre distance.** Distance claims come from every row of `dist.csv`, never from `walk_until` polling.

---

### Task 1: Visibility predicate

**Files:**
- Create: `legged_admm_fleet/include/legged_upper_control/trust.hpp`
- Create: `legged_admm_fleet/test/test_trust.cpp`
- Modify: `legged_admm_fleet/CMakeLists.txt:96-106`

**Interfaces:**
- Consumes: `admm::Obstacle` (`{Eigen::Vector2d pos; double radius;}`) from `legged_upper_control/admm_node_qp.hpp`.
- Produces: `bool admm::visible(const Eigen::Vector2d& pi, const Eigen::Vector2d& pj, const std::vector<Obstacle>& obs, double range)`.

- [ ] **Step 1: Write the failing test**

Create `legged_admm_fleet/test/test_trust.cpp`:

```cpp
// Trust layer gates: can I see you, and how much does one look move my belief.
// Plain assert/main like the other gates in this directory.
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

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
    assert(!visible({0.0, 0.0}, {nan, 0.0}, {}, 4.0));
}

}  // namespace

int main() {
    test_visible_clear_line();
    test_visible_out_of_range();
    test_visible_blocked_by_peg();
    test_visible_past_the_peg_is_not_blocked();
    test_visible_grazing_miss();
    test_visible_nan_is_not_visible();
    std::cout << "test_trust: OK\n";
    return 0;
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
docker exec legged_stack bash -c "source /opt/ros/jazzy/setup.bash && source /root/gridmap_ws/install/setup.bash && cd /root/legged_ros2_ws && MAKEFLAGS=-j8 colcon build --executor sequential --packages-select legged_admm_fleet --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
```

Expected: FAIL — `trust.hpp: No such file or directory`.

- [ ] **Step 3: Write the minimal implementation**

Create `legged_admm_fleet/include/legged_upper_control/trust.hpp`:

```cpp
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
        const Eigen::Vector2d w = o.pos - pi;
        const double t = w.dot(d) / (len * len);   // where the foot of the perpendicular lands
        if (t <= 0.0 || t >= 1.0) continue;        // the pile is behind me or past the target
        if ((w - t * d).norm() < o.radius) return false;
    }
    return true;
}

}  // namespace admm
```

- [ ] **Step 4: Register the test with CMake**

In `legged_admm_fleet/CMakeLists.txt`, after the `test_false_signal` block (line ~96):

```cmake
add_executable(test_trust test/test_trust.cpp)
target_link_libraries(test_trust admm_core)
```

and after `add_test(NAME false_signal ...)`:

```cmake
add_test(NAME trust COMMAND test_trust)
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
docker exec legged_stack bash -c "source /opt/ros/jazzy/setup.bash && source /root/gridmap_ws/install/setup.bash && cd /root/legged_ros2_ws && MAKEFLAGS=-j8 colcon build --executor sequential --packages-select legged_admm_fleet --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && ./build/legged_admm_fleet/test_trust"
```

Expected: `test_trust: OK`

- [ ] **Step 6: Commit**

```bash
git add legged_admm_fleet/include/legged_upper_control/trust.hpp legged_admm_fleet/test/test_trust.cpp legged_admm_fleet/CMakeLists.txt
git commit -m "Decide what a dog can actually see before letting it testify"
```

---

### Task 2: Belief accumulator

**Files:**
- Modify: `legged_admm_fleet/include/legged_upper_control/trust.hpp`
- Modify: `legged_admm_fleet/test/test_trust.cpp`

**Interfaces:**
- Consumes: nothing from Task 1 beyond the same header.
- Produces:
  - `struct admm::TrustParams { double lambda, clamp_step, l_max, l_evict, l_rejoin, sigma, d_lie; }`
  - `double admm::trust_llr(double r, double sigma, double d_lie, double clamp_step)`
  - `double admm::trust_step_self(double L, double llr, const TrustParams&)`
  - `double admm::trust_step_relay(double C, double llr, double l_src, const TrustParams&)`
  - `double admm::trust_total(double L_self, const std::map<int,double>& relay)`
  - `double admm::trust_prob(double L)`
  - `bool admm::trust_fences_peer(double L, const TrustParams&)`

- [ ] **Step 1: Write the failing tests**

Append to `legged_admm_fleet/test/test_trust.cpp` (before `}  // namespace`):

```cpp
TrustParams params() {
    TrustParams p;              // defaults are the derived values; see trust.hpp
    p.sigma = 0.02;
    return p;
}

void test_llr_zero_crossing_at_half_the_lie() {
    const TrustParams p = params();
    // A residual of exactly d_lie/2 is neutral: below it evidence supports honesty, above it
    // supports lying. Anything else would put the decision boundary somewhere undocumented.
    assert(std::abs(trust_llr(p.d_lie / 2.0, p.sigma, p.d_lie, p.clamp_step)) < 1e-12);
    assert(trust_llr(0.0, p.sigma, p.d_lie, p.clamp_step) > 0.0);
    assert(trust_llr(p.d_lie, p.sigma, p.d_lie, p.clamp_step) < 0.0);
}

void test_llr_is_clamped() {
    const TrustParams p = params();
    assert(std::abs(trust_llr(50.0, p.sigma, p.d_lie, p.clamp_step)) <= p.clamp_step + 1e-12);
    assert(std::abs(trust_llr(0.0, p.sigma, p.d_lie, p.clamp_step)) <= p.clamp_step + 1e-12);
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
```

and add the calls in `main()`:

```cpp
    test_llr_zero_crossing_at_half_the_lie();
    test_llr_is_clamped();
    test_trust_has_a_ceiling();
    test_first_hand_evidence_can_evict();
    test_one_relayer_alone_can_never_evict();
    test_hearsay_cannot_reach_the_eviction_threshold();
    test_first_hand_evidence_still_convicts_with_a_relayer_agreeing();
    test_an_untrusted_relayer_carries_no_weight();
    test_belief_decays_toward_neutral();
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
docker exec legged_stack bash -c "source /opt/ros/jazzy/setup.bash && source /root/gridmap_ws/install/setup.bash && cd /root/legged_ros2_ws && MAKEFLAGS=-j8 colcon build --executor sequential --packages-select legged_admm_fleet --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
```

Expected: FAIL — `'TrustParams' was not declared in this scope`.

- [ ] **Step 3: Write the minimal implementation**

Append inside `namespace admm` in `trust.hpp`:

```cpp
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

// Second-hand evidence, accumulated per relaying source. Weighted by how much we believe the
// relayer and capped by that same confidence: you cannot be more certain that j lied, on i's
// word, than you are that i is honest. That bound is what makes a lone accuser harmless, and it
// falls out of the likelihood being conditioned on the relayer — it is not a bolted-on rule.
// l_src is OUR log-odds for the relayer; a relayer at or below neutral carries nothing.
inline double trust_prob(double L) { return 1.0 / (1.0 + std::exp(-L)); }

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
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
docker exec legged_stack bash -c "source /opt/ros/jazzy/setup.bash && source /root/gridmap_ws/install/setup.bash && cd /root/legged_ros2_ws && MAKEFLAGS=-j8 colcon build --executor sequential --packages-select legged_admm_fleet --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && ./build/legged_admm_fleet/test_trust"
```

Expected: `test_trust: OK`. If `test_first_hand_evidence_can_evict` reports more than 10 slots, do not widen the assert — report the number, because it is the detection latency the paper must state.

- [ ] **Step 5: Commit**

```bash
git add legged_admm_fleet/include/legged_upper_control/trust.hpp legged_admm_fleet/test/test_trust.cpp
git commit -m "Accumulate doubt instead of convicting on one frame"
```

---

### Task 3: Keep-out geometry reads the belief

**Files:**
- Modify: `legged_admm_fleet/src/admm_agent_node.cpp` (constructor parameter block ~line 83-101; `gate2()` ~line 804; corpse construction in `maybeEvict()` ~line 915)
- Modify: `legged_admm_fleet/test/test_trust.cpp`

**Interfaces:**
- Consumes: `admm::TrustParams`, `admm::trust_fences_peer` (Task 2); existing `admm::corpse_keepout` (unchanged).
- Produces: node member `admm::TrustParams trust_;` populated from ROS parameters `trust_lambda`, `trust_clamp`, `trust_l_max`, `trust_l_evict`, `trust_l_rejoin`, `obs_sigma`, `trust_d_lie`.

- [ ] **Step 1: Write the failing test**

Append to `test_trust.cpp` (before `}  // namespace`) and call it from `main()`:

```cpp
void test_neutral_belief_does_not_fence_the_spawn_formation() {
    const TrustParams p = params();
    // Every peer starts at neutral (no evidence either way). If neutral fenced anyone, the
    // three dogs that spawn 1.40 m apart in the V would be born violating their own constraint.
    assert(!trust_fences_peer(0.0, p));
    assert(!trust_fences_peer(p.l_rejoin, p));
    assert(!trust_fences_peer(p.l_evict + 1e-9, p));
    assert(trust_fences_peer(p.l_evict - 1e-9, p));
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run the build+test command from Task 2 Step 4.
Expected: FAIL — the assert on `p.l_evict + 1e-9` fails only if the threshold sense is wrong; if it passes immediately, still complete the remaining steps (the node wiring below is the real deliverable).

- [ ] **Step 3: Declare the trust parameters in the node**

In `admm_agent_node.cpp`, immediately after the `resid_gate_` declaration (~line 91), add:

```cpp
        // Trust layer parameters. Defaults are the derived values in trust.hpp; each one has a
        // derivation, not a tuning history. obs_sigma is the only measured one (Task 8).
        trust_.lambda   = declare_parameter<double>("trust_lambda", trust_.lambda);
        trust_.clamp_step = declare_parameter<double>("trust_clamp", trust_.clamp_step);
        trust_.l_max    = declare_parameter<double>("trust_l_max", trust_.l_max);
        trust_.l_evict  = declare_parameter<double>("trust_l_evict", trust_.l_evict);
        trust_.l_rejoin = declare_parameter<double>("trust_l_rejoin", trust_.l_rejoin);
        trust_.sigma    = declare_parameter<double>("obs_sigma", trust_.sigma);
        trust_.d_lie    = declare_parameter<double>("trust_d_lie", trust_.d_lie);
        if (!(trust_.d_lie < 0.433))
            throw std::runtime_error("trust_d_lie must stay under the 0.433 m safety buffer");
        if (!(trust_.l_max < -trust_.l_evict))
            throw std::runtime_error("trust_l_max >= |l_evict|: one relayer could evict alone");
```

- [ ] **Step 4: Add the member and the include**

Add near the other includes:

```cpp
#include "legged_upper_control/trust.hpp"
```

and next to `double resid_alpha_ = 0.2, resid_gate_ = 0.5;`:

```cpp
    admm::TrustParams trust_;                       // derived constants, see trust.hpp
    std::map<int, double> l_self_;                  // first-hand log-odds per peer, cycle() only
    std::map<int, std::map<int, double>> l_relay_;  // [target][source] second-hand, cycle() only
```

- [ ] **Step 5: Build and run the full test suite**

```bash
docker exec legged_stack bash -c "source /opt/ros/jazzy/setup.bash && source /root/gridmap_ws/install/setup.bash && cd /root/legged_ros2_ws && MAKEFLAGS=-j8 colcon build --executor sequential --packages-select legged_admm_fleet --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && cd build/legged_admm_fleet && ctest --output-on-failure"
```

Expected: 6/6 tests pass, including `distributed_parity`.

- [ ] **Step 6: Commit**

```bash
git add legged_admm_fleet/src/admm_agent_node.cpp legged_admm_fleet/test/test_trust.cpp
git commit -m "Refuse to start with thresholds that would fence the spawn formation"
```

---

### Task 4: Observation channel — own namespace, time alignment, deterministic noise

**Files:**
- Modify: `legged_admm_fleet/src/admm_agent_node.cpp` (`peer_truth_` subscription block ~line 260-285; member block ~line 1116)
- Modify: `legged_admm_fleet/launch/admm_fleet.launch.py:62-72`
- Modify: `legged_admm_fleet/test/test_trust.cpp`

**Interfaces:**
- Consumes: `admm::TrustParams` (Task 2).
- Produces:
  - `std::optional<Eigen::Vector2d> admm::interp_at(const std::deque<admm::ObsSample>&, double t)` in `trust.hpp`
  - `Eigen::Vector2d admm::obs_noise(std::uint64_t slot, int i, int j, double sigma)` in `trust.hpp`
  - `struct admm::ObsSample { double t; Eigen::Vector2d p; }`
  - node member `std::map<int, std::deque<admm::ObsSample>> peer_truth_;` (replaces the `std::map<int, Eigen::Vector2d>` from the uncommitted scaffolding)

- [ ] **Step 1: Write the failing tests**

Append to `test_trust.cpp` and call from `main()`:

```cpp
void test_interp_lands_between_samples() {
    std::deque<ObsSample> s{{1.0, {0.0, 0.0}}, {2.0, {1.0, 0.0}}};
    const auto p = interp_at(s, 1.5);
    assert(p.has_value());
    assert(std::abs((*p)[0] - 0.5) < 1e-12);
}

void test_interp_does_not_extrapolate() {
    std::deque<ObsSample> s{{1.0, {0.0, 0.0}}, {2.0, {1.0, 0.0}}};
    const auto p = interp_at(s, 9.0);
    assert(p.has_value());
    assert(std::abs((*p)[0] - 1.0) < 1e-12);   // newest sample, never a guess past the window
}

void test_interp_empty_is_no_evidence() {
    std::deque<ObsSample> s;
    assert(!interp_at(s, 1.0).has_value());
}

void test_noise_is_deterministic_in_slot_and_pair() {
    const Eigen::Vector2d a = obs_noise(42, 1, 2, 0.02);
    const Eigen::Vector2d b = obs_noise(42, 1, 2, 0.02);
    const Eigen::Vector2d c = obs_noise(43, 1, 2, 0.02);
    assert((a - b).norm() < 1e-15);   // a rerun of the same scenario sees the same noise
    assert((a - c).norm() > 1e-12);   // but it is not a constant offset
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run the build+test command from Task 2 Step 4.
Expected: FAIL — `'interp_at' was not declared in this scope`.

- [ ] **Step 3: Implement both helpers**

Append inside `namespace admm` in `trust.hpp` (add `#include <cstdint>`, `#include <deque>`, `#include <optional>`, `#include <random>` at the top):

```cpp
struct ObsSample {
    double t;              // sim seconds, from the message header stamp
    Eigen::Vector2d p;
};

// Align an observation to the timestamp of the claim it will be compared against. The claim is
// the peer's state at the head of a slot; samples arrive at odom rate in between. At 0.55 m/s an
// unaligned 100 ms is 5.5 cm of pure bias against a 0.30 m decision — a sixth of the budget
// spent on nothing. Outside the window we return the newest sample rather than extrapolating:
// a guess about where a peer went is exactly the kind of invented evidence this layer exists to
// avoid.
inline std::optional<Eigen::Vector2d> interp_at(const std::deque<ObsSample>& s, double t) {
    if (s.empty()) return std::nullopt;
    if (s.size() == 1) return s.back().p;
    for (std::size_t i = s.size() - 1; i > 0; --i) {
        if (s[i - 1].t <= t && t <= s[i].t) {
            const double dt = s[i].t - s[i - 1].t;
            const double a = (dt > 1e-9) ? (t - s[i - 1].t) / dt : 0.0;
            return s[i - 1].p + a * (s[i].p - s[i - 1].p);
        }
    }
    return s.back().p;
}

// Sensor noise as a pure function of (slot, observer, target). A sequential draw would consume a
// number of samples that depends on how many peers happened to be visible that slot, so two runs
// of the same script would diverge — the same determinism discipline that pins the OSQP version.
inline Eigen::Vector2d obs_noise(std::uint64_t slot, int i, int j, double sigma) {
    std::uint64_t h = 1469598103934665603ull;
    for (std::uint64_t v : {slot, static_cast<std::uint64_t>(i), static_cast<std::uint64_t>(j)}) {
        h ^= v;
        h *= 1099511628211ull;
    }
    std::mt19937_64 g(h);
    std::normal_distribution<double> n(0.0, sigma);
    const double a = n(g), b = n(g);
    return {a, b};
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run the build+test command from Task 2 Step 4.
Expected: `test_trust: OK`.

- [ ] **Step 5: Rewrite the observation subscription in the node**

Replace the `peer_truth_subs_` block from the uncommitted scaffolding with:

```cpp
        // OBSERVATION CHANNEL. What THIS agent sees of a peer — the one input a compromised peer
        // does not author. In simulation the stand-in is the Gazebo plugin's world pose, which is
        // published from the model rather than from robot j's estimator.
        //
        // THREAT MODEL, and it is load-bearing: a ROS 2 topic has no publisher authentication, so
        // a compromised robot j CAN publish on this name and hand us its own forgery. The
        // subscription is therefore remapped into the OBSERVER's namespace at launch, and the
        // publisher is checked below. On real hardware this holds for free: the sensor is local
        // hardware and a peer cannot write to it.
        obs_range_ = declare_parameter<double>("obs_range", 4.0);
        obs_window_ = declare_parameter<int>("obs_window", 8);
        for (int j : dogs_) {
            if (j == self_id_) continue;
            peer_truth_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
                "observed/robot" + std::to_string(j), 1,
                [this, j](nav_msgs::msg::Odometry::SharedPtr m) {
                    std::lock_guard<std::mutex> l(mu_);
                    auto& q = peer_truth_[j];
                    q.push_back({rclcpp::Time(m->header.stamp).seconds(),
                                 Eigen::Vector2d(m->pose.pose.position.x,
                                                 m->pose.pose.position.y)});
                    while (static_cast<int>(q.size()) > obs_window_) q.pop_front();
                }, cmd_opts));
        }
```

- [ ] **Step 6: Remap the topic in the launch file**

In `admm_fleet.launch.py`, inside the per-robot `Node(...)` in `_distributed_agents`, add a `remappings` entry that binds each observation subscription to the corresponding Gazebo-published topic:

```python
            remappings=[
                # The observation channel is the observer's own sensor. It is remapped here, in
                # the observer's namespace, so that "what robot i sees of robot j" is a thing
                # robot j has no publisher for. In simulation the stand-in source is the Gazebo
                # plugin's world pose; on hardware this becomes the local perception topic.
                (f'observed/robot{int(j)}', f'/robot{int(j)}/hardware/odom')
                for j in ids if int(j) != int(i)
            ],
```

The enclosing loop is `for i in ids:` (line 41) and `ids` holds the roster entries, so both are cast with `int()` exactly as the existing `'robot_ids': [int(x) for x in ids]` does. Keep the existing `OBS_GATE2` / `OBS_RANGE` / `ODOM_FAKE*` environment parameters exactly as they are in the working tree.

- [ ] **Step 6b: Change the member declarations**

The uncommitted scaffolding declares `std::map<int, Eigen::Vector2d> peer_truth_;`. Replace it, and add the window size:

```cpp
    std::map<int, std::deque<admm::ObsSample>> peer_truth_;  // observation ring, guarded by mu_
    int obs_window_ = 8;   // samples kept per peer; only the pair bracketing the slot is used
```

Add `#include <deque>` to the node's includes.

- [ ] **Step 7: Build and run the whole suite**

```bash
docker exec legged_stack bash -c "source /opt/ros/jazzy/setup.bash && source /root/gridmap_ws/install/setup.bash && cd /root/legged_ros2_ws && MAKEFLAGS=-j8 colcon build --executor sequential --packages-select legged_admm_fleet --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && cd build/legged_admm_fleet && ctest --output-on-failure"
```

Expected: 6/6 pass.

- [ ] **Step 8: Commit**

```bash
git add legged_admm_fleet/include/legged_upper_control/trust.hpp legged_admm_fleet/test/test_trust.cpp legged_admm_fleet/src/admm_agent_node.cpp legged_admm_fleet/launch/admm_fleet.launch.py
git commit -m "Read the peer with our own clock, our own noise, and our own namespace"
```

---

### Task 5: Belief replaces the single-shot verdict

**Files:**
- Modify: `legged_admm_fleet/src/admm_agent_node.cpp` (`gate2()` ~line 804-854; its call sites in `cycle()`)

**Interfaces:**
- Consumes: `trust_`, `l_self_`, `l_relay_` (Task 3); `visible`, `interp_at`, `obs_noise` (Tasks 1, 4).
- Produces: `void AdmmAgentNode::beliefStep(std::uint64_t slot)` replacing `gate2(slot)`; peer blocking still goes through `transport_->block_peer(j, reason)`.

- [ ] **Step 1: Replace the body of `gate2()`**

Rename it to `beliefStep` and replace its body (keep the freshness guard and the `evict_armed_` guard exactly as they are):

```cpp
    // Evidence -> belief -> admission. Replaces the single-shot residual verdict: two survivors
    // reading the same residual at slightly different moments used to reach verdicts 9.5 s and
    // 4.5 m apart (measured 2026-07-29). An accumulator over shared evidence cannot do that.
    void beliefStep(std::uint64_t slot) {
        if (!evict_armed_) return;
        std::map<int, std::deque<admm::ObsSample>> od;
        Eigen::Vector2d self_p(0.0, 0.0);
        {
            std::lock_guard<std::mutex> l(mu_);
            od = peer_truth_;
            self_p = Eigen::Vector2d(odom_.pose.pose.position.x, odom_.pose.pose.position.y);
        }
        const double t_slot = static_cast<double>(slot) * admm::TS;
        const auto seen = transport_->last_seen();
        ev_out_.clear();
        for (const auto& kv : agent_->peer_xnow()) {
            const int j = kv.first;
            if (j == self_id_) continue;
            // FRESHNESS, unchanged from Gate 2: peer_xnow() holds the last COMMITTED claim, which
            // during a barrier-miss HOLD is frozen while that peer's body keeps walking.
            // Differencing a stale claim against a live observation manufactures a residual that
            // grows with the silence. Being quiet is maybeEvict's business.
            const auto sit = seen.find(j);
            if (sit == seen.end() || slot > sit->second + 2) continue;
            const auto qit = od.find(j);
            if (qit == od.end()) continue;
            const auto obs = admm::interp_at(qit->second, t_slot);
            if (!obs) continue;
            // Out of range or behind a pile is NOT evidence of anything. No update at all —
            // abstention is the absence of a term, not a term with a special value.
            if (!admm::visible(self_p, *obs, arena_obs_, obs_range_)) continue;
            const Eigen::Vector2d z = *obs + admm::obs_noise(slot, self_id_, j, trust_.sigma);
            // RESIDUAL = INNOVATION. In simulation the stand-in publishes the base origin, so the
            // observation model h() is the identity and this reduces to a subtraction. With real
            // perception h() predicts what the sensor should see given the claim (body surface,
            // viewing angle) — same definition, one implementation site, no second code path.
            const double r = (z - kv.second.head<2>()).norm();
            double& L = l_self_[j];
            L = admm::trust_step_self(L, admm::trust_llr(r, trust_.sigma, trust_.d_lie,
                                                         trust_.clamp_step), trust_);
            ev_out_.push_back({j, z});   // share the observation, not the verdict (Task 6)
            const double total = admm::trust_total(L, l_relay_[j]);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[agent%d] belief robot%d r=%.3f L_self=%.2f L=%.2f b=%.4f "
                                 "evict<%.1f | gate1_worst=%.3f dropped=%u",
                                 self_id_, j, r, L, total, admm::trust_prob(total),
                                 trust_.l_evict, transport_->gate_worst(),
                                 transport_->dropped(j));
            if (!admm::trust_fences_peer(total, trust_)) continue;
            if (log_only_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "[agent%d] BELIEF would reject robot%d (L=%.2f < %.1f) — "
                                     "log_only, message still accepted",
                                     self_id_, j, total, trust_.l_evict);
            } else {
                // ONE eviction path, unchanged: blocking makes the peer silent to us, and the
                // existing crash-failover timer evicts it. A second verdict route would be the
                // scenario-specific special case this design is not allowed to have.
                transport_->block_peer(j, "belief below evict threshold");
            }
        }
    }
```

- [ ] **Step 2: Add the outbound evidence buffer member**

Next to `l_relay_`:

```cpp
    struct EvidenceOut { int peer; Eigen::Vector2d pos; };
    std::vector<EvidenceOut> ev_out_;   // this slot's observations, cycle() only; sent in Task 6
```

- [ ] **Step 3: Update the call sites**

Rename both `gate2(slot)` call sites in `cycle()` (the SOLVED path and the HOLD path) to `beliefStep(slot)`. Leave `resid_`, `resid_alpha_`, `resid_gate_` and their parameter declarations in place for one commit so a bisect can compare, and delete them in Step 6.

- [ ] **Step 4: Build and run the whole suite**

```bash
docker exec legged_stack bash -c "source /opt/ros/jazzy/setup.bash && source /root/gridmap_ws/install/setup.bash && cd /root/legged_ros2_ws && MAKEFLAGS=-j8 colcon build --executor sequential --packages-select legged_admm_fleet --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && cd build/legged_admm_fleet && ctest --output-on-failure"
```

Expected: 6/6 pass.

- [ ] **Step 5: Smoke-test an honest fleet in Gazebo**

```bash
docker exec legged_stack bash -c "source /root/legged_ros2_ws/src/legged_fleet/legged_admm_fleet/scripts/../../../../source.sh && cd /root/legged_ros2_ws && NO_KILL=1 SOAK=60 bash src/legged_fleet/legged_admm_fleet/scripts/d_run.sh 1"
```

Expected: PASS, zero evictions, zero `BELIEF would reject`. If an honest dog is fenced, do not raise the threshold — capture the log line and report; a false positive here is the failure this design exists to avoid.

- [ ] **Step 6: Delete the dead single-shot path and commit**

Remove `resid_`, `resid_alpha_`, `resid_gate_`, the `odom_residual_alpha` / `odom_residual_gate` parameter declarations and their `param callback` branch.

```bash
git add legged_admm_fleet/src/admm_agent_node.cpp
git commit -m "Convict on accumulated evidence, and delete the frame that used to convict alone"
```

---

### Task 6: Share the evidence

**Files:**
- Modify: `admm_fleet_msgs/msg/AgentState.msg`
- Modify: `legged_admm_fleet/src/agent_core.hpp:34-43`
- Modify: `legged_admm_fleet/src/dds_transport.hpp:97-127` (send), `:433-490` (receive), `:353-356` (`wire_bytes`)
- Modify: `legged_admm_fleet/src/admm_agent_node.cpp` (`beliefStep`, send path)
- Modify: `legged_admm_fleet/test/test_trust.cpp`

**Interfaces:**
- Consumes: `EvidenceOut` (Task 5), `visible` (Task 1), `trust_step_relay` (Task 2).
- Produces: `AgentStateMsg::ev_peer` (`std::vector<int>`), `AgentStateMsg::ev_pos` (`std::vector<double>`, length `2 * ev_peer.size()`); `bool admm::evidence_plausible(const Eigen::Vector2d& src, const Eigen::Vector2d& obs_pos, const std::vector<Obstacle>&, double range)`.

- [ ] **Step 1: Write the failing test**

Append to `test_trust.cpp` and call from `main()`:

```cpp
void test_evidence_from_behind_a_peg_is_refuted_without_seeing_anyone() {
    // Robot i claims to have observed a body at (8,0) while standing at (5,0) with a pile
    // between them. Anyone can compute that this sightline does not exist — no sensor needed.
    assert(!evidence_plausible({5.0, 0.0}, {8.0, 0.0}, pegs(), 4.0));
    assert(evidence_plausible({5.0, 0.4}, {8.0, 0.4}, pegs(), 4.0));
}

void test_evidence_beyond_sensor_range_is_refuted() {
    assert(!evidence_plausible({0.0, 0.0}, {5.0, 0.0}, {}, 4.0));
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run the build+test command from Task 2 Step 4.
Expected: FAIL — `'evidence_plausible' was not declared in this scope`.

- [ ] **Step 3: Add the plausibility predicate**

Append inside `namespace admm` in `trust.hpp`:

```cpp
// Can this report even be true? A relayed observation carries its own geometry: the reporter
// must have been in range and unobstructed. Every agent can check this WITHOUT seeing either
// party, because the arena is known and the reporter's own position is independently observed.
// Broadcasting positions rather than scalar residuals is what makes the check possible at all.
inline bool evidence_plausible(const Eigen::Vector2d& src, const Eigen::Vector2d& obs_pos,
                               const std::vector<Obstacle>& obs, double range) {
    return visible(src, obs_pos, obs, range);
}
```

- [ ] **Step 4: Extend the message and the wire struct**

`admm_fleet_msgs/msg/AgentState.msg`, appended after `members`:

```
int32[]   ev_peer      # peers this sender observed THIS slot
float64[] ev_pos       # observed (x,y) per entry; length is exactly 2 * len(ev_peer).
                       # Positions, not residuals: a residual is unfalsifiable, a position is
                       # constrained by range and by which sightlines the arena allows, so a
                       # receiver can refute a fabricated report without seeing either party.
                       # Empty = "I saw nobody this slot", which is also what a pre-evidence
                       # sender means, so old peers and the loopback test transport are unaffected.
```

`agent_core.hpp`, appended to `AgentStateMsg`:

```cpp
    std::vector<int> ev_peer;      // observed peers this slot
    std::vector<double> ev_pos;    // 2 per entry; empty means "no observations", not "none exist"
```

- [ ] **Step 5: Wire the transport**

In `dds_transport.hpp` `send_state`, after the `members` assignment:

```cpp
        w.ev_peer.assign(s.ev_peer.begin(), s.ev_peer.end());
        w.ev_pos.assign(s.ev_pos.begin(), s.ev_pos.end());
```

In the receive path, next to the `members` copy:

```cpp
        m.ev_peer.assign(w.ev_peer.begin(), w.ev_peer.end());
        m.ev_pos.assign(w.ev_pos.begin(), w.ev_pos.end());
        // Length disagreement is a malformed message, judged the same way Gate 1 judges every
        // other impossible payload: drop the whole thing rather than guess which half is right.
        if (m.ev_pos.size() != 2 * m.ev_peer.size()) {
            drop(m.robot_id, "evidence length mismatch",
                 static_cast<double>(m.ev_pos.size()));
            return;
        }
```

In `wire_bytes(AgentState)`:

```cpp
        return 8 + 4 + 8 + 4 + 4 * 8 + (4 + 8 * w.xibar.size()) + 1
               + (4 + 4 * w.members.size())
               + (4 + 4 * w.ev_peer.size()) + (4 + 8 * w.ev_pos.size());
```

- [ ] **Step 6: Publish this agent's evidence**

In `admm_agent_node.cpp`, where the outgoing `AgentStateMsg` is filled for the cycle head, add:

```cpp
        for (const auto& e : ev_out_) {
            msg.ev_peer.push_back(e.peer);
            msg.ev_pos.push_back(e.pos[0]);
            msg.ev_pos.push_back(e.pos[1]);
        }
```

- [ ] **Step 7: Consume peers' evidence in `beliefStep`**

At the end of `beliefStep`, before the closing brace:

```cpp
        // Second-hand evidence. Weighted and capped by what we independently believe about the
        // REPORTER, so hearsay can raise suspicion but never convict on its own — and a reporter
        // we have already fenced contributes exactly nothing.
        for (const auto& kv : agent_->peer_evidence()) {
            const int i = kv.first;
            if (i == self_id_) continue;
            const auto& ev = kv.second;                    // {ev_peer, ev_pos} as received
            const auto xi = agent_->peer_xnow().find(i);
            if (xi == agent_->peer_xnow().end()) continue;
            const double l_src = admm::trust_total(l_self_[i], l_relay_[i]);
            for (std::size_t k = 0; k < ev.first.size(); ++k) {
                const int j = ev.first[k];
                if (j == self_id_ || j == i) continue;
                const Eigen::Vector2d z(ev.second[2 * k], ev.second[2 * k + 1]);
                if (!z.allFinite()) continue;
                const auto cj = agent_->peer_xnow().find(j);
                if (cj == agent_->peer_xnow().end()) continue;
                // A report that geometry forbids is not weak evidence about j — it is evidence
                // about the REPORTER, which is where the penalty lands.
                if (!admm::evidence_plausible(xi->second.head<2>(), z, arena_obs_, obs_range_)) {
                    l_self_[i] = admm::trust_step_self(l_self_[i], -trust_.clamp_step, trust_);
                    continue;
                }
                const double r = (z - cj->second.head<2>()).norm();
                double& C = l_relay_[j][i];
                C = admm::trust_step_relay(C, admm::trust_llr(r, trust_.sigma, trust_.d_lie,
                                                              trust_.clamp_step), l_src, trust_);
            }
        }
```

`AgentCore` exposes `peer_xnow()` and `peer_xibar()` but nothing else from the received messages, so add one cache beside them in `agent_core.hpp` (next to `peer_xnow_` at line 310), filled at the same place `peer_xnow_` is, and a const accessor:

```cpp
    // Relayed observations, cached exactly like peer_xnow_ so the node reads what arrived rather
    // than a second copy. Loopback senders leave it empty, which is why parity is unaffected.
    std::map<int, std::pair<std::vector<int>, std::vector<double>>> peer_ev_;
```

```cpp
    const std::map<int, std::pair<std::vector<int>, std::vector<double>>>& peer_evidence() const {
        return peer_ev_;
    }
```

- [ ] **Step 8: Build and run the whole suite**

Run the build+ctest command from Task 3 Step 5 with `--packages-select admm_fleet_msgs legged_admm_fleet`.
Expected: 6/6 pass, including `distributed_parity` (the loopback transport leaves the new fields empty, so parity must be unchanged).

- [ ] **Step 9: Commit**

```bash
git add admm_fleet_msgs/msg/AgentState.msg legged_admm_fleet/src/agent_core.hpp legged_admm_fleet/src/dds_transport.hpp legged_admm_fleet/src/admm_agent_node.cpp legged_admm_fleet/include/legged_upper_control/trust.hpp legged_admm_fleet/test/test_trust.cpp
git commit -m "Say what we saw, not what we concluded"
```

---

### Task 7: The two attackers

**Files:**
- Modify: `legged_admm_fleet/src/dds_transport.hpp` (`send_state`)
- Modify: `legged_admm_fleet/src/admm_agent_node.cpp` (parameter declarations)

**Interfaces:**
- Consumes: the `inject_fake_offset` pattern already in `send_state`.
- Produces: parameters `inject_fake_evidence` (double) and `inject_fake_evidence_target` (int); the existing `inject_odom_fake*` from the working tree is kept as-is.

- [ ] **Step 1: Add the smear injection to the send path**

In `dds_transport.hpp::send_state`, after the `inj_fake_offset_` block:

```cpp
        // SMEAR ATTACK. The attacker's own position stays honest; it fabricates what it claims
        // to have SEEN of an honest peer. This is the arm that proves the defence did not open a
        // new hole — a fleet where one broadcast can remove an honest dog would be worse than
        // one with no detector at all.
        if (const double off = inj_fake_evidence_.load(); off != 0.0) {
            const int t = inj_fake_evidence_target_.load();
            for (std::size_t k = 0; k < s.ev_peer.size(); ++k) {
                if (s.ev_peer[k] != t) continue;
                s.ev_pos[2 * k] += off;
                s.ev_pos[2 * k + 1] += off;
            }
        }
```

with members `std::atomic<double> inj_fake_evidence_{0.0}; std::atomic<int> inj_fake_evidence_target_{0};` and a setter matching the existing injection setters.

- [ ] **Step 2: Declare the parameters in the node**

Next to `declare_parameter<double>("inject_fake_offset", 0.0);`:

```cpp
        // Smear knobs, dynamic like the other injections so a sweep can set them on a live fleet.
        declare_parameter<double>("inject_fake_evidence", 0.0);
        declare_parameter<int>("inject_fake_evidence_target", 0);
```

and route both through the existing parameter callback into the transport setter.

- [ ] **Step 3: Build and run the suite**

Run the build+ctest command from Task 3 Step 5.
Expected: 6/6 pass.

- [ ] **Step 4: Commit**

```bash
git add legged_admm_fleet/src/dds_transport.hpp legged_admm_fleet/src/admm_agent_node.cpp
git commit -m "Give the attacker a second weapon: a fabricated eyewitness account"
```

---

### Task 8: Calibrate sigma, then wire the arms

**Files:**
- Modify: `legged_admm_fleet/scripts/d_run.sh`
- Create: `docs/superpowers/results/2026-07-31-trust-calibration.md`

**Interfaces:**
- Consumes: `OBS_GATE2`, `OBS_RANGE`, `ODOM_FAKE`, `ODOM_FAKE_PEER` environment parameters already read by `admm_fleet.launch.py`; `inject_fake_evidence*` (Task 7).
- Produces: `ARM` environment variable accepting `a0` | `a1` | `a2` | `smear`, and a measured `obs_sigma` value recorded in the results file.

- [ ] **Step 1: Run the clean-flight calibration**

```bash
docker exec legged_stack bash -c "source /root/legged_ros2_ws/source.sh && cd /root/legged_ros2_ws && NO_KILL=1 SOAK=120 ARENA=plum bash src/legged_fleet/legged_admm_fleet/scripts/d_run.sh 1"
```

Then extract every `belief robot` line's `r=` value from `g2_logs/<newest>/admm.log` and take max and 99th percentile.

- [ ] **Step 2: Record the calibration**

Create `docs/superpowers/results/2026-07-31-trust-calibration.md` with: the log directory, the number of residual samples, max r, p99 r, the chosen `obs_sigma`, and one sentence stating that this value is the simulation floor and excludes attitude lever arm, inter-robot drift and surface-vs-origin bias.

- [ ] **Step 3: Add the arm selector to `d_run.sh`**

Near the other environment defaults at the top:

```bash
# Experiment arms. One knob, so a run is labelled by what it tests rather than by which four
# variables happened to be exported.
#   a0    no defence at all         (detection disarmed)
#   a1    self-reported gate        (the old channel: peer's own odom)
#   a2    observation belief layer  (this design)
#   smear attacker fabricates eyewitness reports about an honest dog
ARM=${ARM:-a2}
case "$ARM" in
  a0)    export OBS_GATE2=0; DETECT_LOG_ONLY=1 ;;
  a1)    export OBS_GATE2=0; DETECT_LOG_ONLY=0 ;;
  a2)    export OBS_GATE2=1; DETECT_LOG_ONLY=0 ;;
  smear) export OBS_GATE2=1; DETECT_LOG_ONLY=0 ;;
  *)     die "unknown ARM=$ARM (want a0|a1|a2|smear)" ;;
esac
```

Pass `DETECT_LOG_ONLY` through to the `detection_log_only` parameter where the launch command is built, and in the `smear` case set `inject_fake_evidence` / `inject_fake_evidence_target` on the attacker with the existing `pset` helper (which already retries; a single `ros2 param set` timeout has destroyed a ten-minute run before).

- [ ] **Step 4: Verify each arm starts and is labelled**

```bash
for a in a0 a1 a2 smear; do
  docker exec legged_stack bash -c "source /root/legged_ros2_ws/source.sh && cd /root/legged_ros2_ws && ARM=$a GOAL_X=3 bash src/legged_fleet/legged_admm_fleet/scripts/d_run.sh 1" 2>&1 | tail -3
done
```

Expected: each run reaches its verdict line and the log records which arm ran.

- [ ] **Step 5: Commit**

```bash
git add legged_admm_fleet/scripts/d_run.sh docs/superpowers/results/2026-07-31-trust-calibration.md
git commit -m "Measure the noise floor before choosing what counts as a lie"
```

---

### Task 9: Run the matrix and report it honestly

**Files:**
- Create: `docs/superpowers/results/2026-07-31-trust-results.md`

**Interfaces:**
- Consumes: everything above.
- Produces: the results document the paper's experiment section is written from.

- [ ] **Step 1: Run the three arms against the dual-channel attacker**

For each `ARM` in `a0 a1 a2`, three runs:

```bash
docker exec legged_stack bash -c "source /root/legged_ros2_ws/source.sh && cd /root/legged_ros2_ws && ARM=<arm> ARENA=plum LIE=0.30 ODOM_FAKE=0.30 ODOM_FAKE_PEER=1 GOAL_X=18 bash src/legged_fleet/legged_admm_fleet/scripts/d_run.sh 1"
```

`LIE` and `ODOM_FAKE` must match: the attacker forges its claim and its odometry by the same offset, which is what makes arm `a1` structurally blind. Remember the diagonal: `LIE=0.30` is a displacement of 0.30 × √2 = 0.424 m.

- [ ] **Step 2: Run the smear arm**

Three runs with `ARM=smear`, the attacker fabricating observations of an honest survivor:

```bash
docker exec legged_stack bash -c "source /root/legged_ros2_ws/source.sh && cd /root/legged_ros2_ws && ARM=smear ARENA=plum SMEAR=2.0 SMEAR_TARGET=2 GOAL_X=18 bash src/legged_fleet/legged_admm_fleet/scripts/d_run.sh 1"
```

- [ ] **Step 3: Run the occlusion check**

One run where the attacker's path takes it behind a pile while lying: confirm the belief freezes (no update, no eviction) while occluded and resumes when it reappears. The log line to grep is `belief robot` — its absence during occlusion IS the result.

- [ ] **Step 3b: Re-run the stale-vote deadlock scenario**

Spec acceptance 4. The old failure: a frozen attacker's expired roster kept voting, and the
coordinator concluded no dog was alive. The forgetting factor should dissolve this with no
expiry rule anywhere in the code.

```bash
docker exec legged_stack bash -c "source /root/legged_ros2_ws/source.sh && cd /root/legged_ros2_ws && ARM=a2 ARENA=plum LIE=2.0 LIE_CHASE=1 LIE_THEN_KILL=1 CHASE_TARGET=2 CHASE_T=40 CHASE_V=0.55 GOAL_X=18 bash src/legged_fleet/legged_admm_fleet/scripts/d_run.sh 1"
```

Expected: no mutual eviction among survivors, no `coordinator` line reporting zero live dogs.
Record the closest survivor-to-survivor distance from every row of `dist.csv`.

- [ ] **Step 3c: Run the Python oracle regression**

```bash
docker exec legged_stack bash -c "source /opt/ros/jazzy/setup.bash && source /root/legged_ros2_ws/install/setup.bash && cd /root/legged_ros2_ws/src/legged_fleet/legged_admm_fleet && PYTHONPATH=/root/legged_ros2_ws/install/legged_admm_fleet/lib/legged_admm_fleet/python:\$PYTHONPATH python3 -m pytest python/ -q"
```

Expected: all 44 oracle tests pass. The trust layer is outside `admm_core`, so any failure here
means something in the shared headers moved and must be fixed before the results are believable.

- [ ] **Step 4: Run the false-positive soak**

```bash
docker exec legged_stack bash -c "source /root/legged_ros2_ws/source.sh && cd /root/legged_ros2_ws && ARM=a2 NO_KILL=1 SOAK=600 ARENA=plum bash src/legged_fleet/legged_admm_fleet/scripts/d_run.sh 1"
```

Expected: zero evictions over 600 s.

- [ ] **Step 5: Write the results document**

Create `docs/superpowers/results/2026-07-31-trust-results.md` with one table per acceptance criterion from spec section 6, each row citing its log directory. Required columns for the three-arm table: arm, physical body gap from `phys_gap_logger.py` (the only contact criterion), closest survivor-to-survivor distance scanned over every row of `dist.csv`, time-to-evict, mission completed, `achieved_rounds`, WBC deactivations.

State plainly: the time-to-evict for `a2` against the current 0.405 s single-shot number, whether the smear arm removed an honest dog, and any run excluded and why. A run where WBC deactivated is re-run, not reported.

- [ ] **Step 6: Commit**

```bash
git add docs/superpowers/results/2026-07-31-trust-results.md
git commit -m "Report what the three arms actually measured, latency cost included"
```

---

## Deferred, with the trigger that un-defers each

- **Reputation array S (anti-collusion).** Needs two colluding attackers, which needs an honest majority: N ≥ 5. Trigger: the 5-dog spike below shows the rig can carry it.
- **5-dog spike.** Thirty minutes: spawn 5, idle, walk 3 m, record RTF, WBC deactivations, RSS. Trigger: Task 9 complete.
- **FOV cone and real perception.** Trigger: moving from "does an observer-sourced residual work" to "how often can a real sensor see".
- **Continuous keep-out ramp below the threshold.** Trigger: a measurement that says what the ramp should be.
- **Silence-as-evidence** (visible but not reporting). Trigger: an attacker that evades by withholding reports appears in a run.
