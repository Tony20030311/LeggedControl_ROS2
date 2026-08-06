// admm_agent_node — one process per robot (G4, the paper's real distributed deployment).
// Runs a single AgentCore over a DdsTransport; each sim time-slot it builds its own state,
// exchanges the ADMM consensus with peers over DDS, and publishes its 24-D mpc_target.
// Slot assignment (/formation/goal -> /formation/plan) lives in the standalone
// fleet_coordinator_node; each agent just subscribes /formation/plan for its own goal.
//
// Per-dog machinery (x0 / MotionAdapter ADAPT / yaw latch / followSpeed / rescue) mirrors the
// centralized node, but is NOT shared code: the two nodes have different data models (one
// process per robot here vs one process for the whole fleet there), so these are structurally
// distinct implementations, not copy-paste. The only genuinely-identical helper, the 24-D
// target builder, IS shared via fleet_logic.hpp (admm::toTargetMsg).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <utility>
#include <vector>

#include <ocs2_core/misc/LoadData.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>  // rmw_gid_t, RMW_GID_STORAGE_SIZE (observation-channel publisher pin)

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

#include "admm_fleet_msgs/msg/cycle_stats.hpp"
#include "admm_fleet_msgs/msg/fleet_plan.hpp"
#include "dds_transport.hpp"
#include "legged_upper_control/astar_planner.hpp"
#include "legged_upper_control/fleet_logic.hpp"
#include "legged_upper_control/admm_motion_adapter.hpp"
#include "legged_upper_control/admm_qp_common.hpp"
#include "legged_upper_control/admm_reference.hpp"
#include "legged_upper_control/fleet_config.hpp"
#include "legged_upper_control/trust.hpp"

using namespace std::chrono_literals;

namespace {
double clip(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
std::vector<admm::EdgeKey> complete_graph(const std::vector<int>& dogs) {
    std::vector<admm::EdgeKey> e;
    for (size_t a = 0; a < dogs.size(); ++a)
        for (size_t b = a + 1; b < dogs.size(); ++b)
            e.emplace_back(dogs[a], dogs[b]);
    return e;
}
}  // namespace

class AdmmAgentNode : public rclcpp::Node {
public:
    AdmmAgentNode() : rclcpp::Node("admm_agent") {
        self_id_ = declare_parameter<int>("robot_id", 1);
        const auto ids = declare_parameter<std::vector<int64_t>>("robot_ids", {1, 2, 3});
        for (auto v : ids) dogs_.push_back(static_cast<int>(v));
        std::sort(dogs_.begin(), dogs_.end());
        roster_n_ = dogs_.size();  // dogs_ shrinks on eviction; the full roster picks the shape
        edges_ = complete_graph(dogs_);
        v_ = declare_parameter<double>("v", 0.4);
        ts_ = admm::TS;
        w_form_ = declare_parameter<double>("w_form", 0.3);
        hard_through_ = declare_parameter<int>("hard_through", 1);
        robot_margin_ = declare_parameter<double>("robot_margin", 0.60);  // Vision60 URDF FK: covers swing foot 0.542 / hind knee 0.609 (was 0.43 base-box, under-counted legs); obstacle-CBF only
        // experiment D: consecutive missing slots of a peer's AgentState before it is
        // declared dead and evicted (10 slots = 1 s sim). 0 disables eviction.
        evict_after_ = declare_parameter<int>("evict_after_misses", 10);
        // TASK 6: which detector is armed for THIS run. false (default) keeps the existing
        // single-shot EMA residual gate (gate2()) running byte-for-byte unchanged -- it is
        // experiment arm A1's mechanism, and the whole reason it is not deleted is that deleting
        // it would make A1 identical to A2 (belief) and acceptance criterion 1 unfalsifiable
        // (design doc section 9, protocol discipline item 1). true swaps in the belief
        // accumulator (beliefStep()) for every call site gate2() used to own alone. Logged once,
        // loudly, so a results table can cite exactly which detector produced a given run's
        // numbers instead of assuming the binary was armed the way the launch command intended.
        obs_gate2_ = declare_parameter<bool>("obs_gate2", false);
        RCLCPP_INFO(get_logger(), "[agent%d] detector armed: %s", self_id_,
                    obs_gate2_ ? "belief accumulator (beliefStep, obs_gate2=true)"
                               : "single-shot EMA residual gate (gate2, obs_gate2=false)");
        // Silence needs patience — it might be a network hiccup. A lie is POSITIVE evidence and
        // does not: waiting the full 10 slots hands a hostile peer another 0.55 m of approach.
        // This patience covers gate2's odom residual AND the roster-exclusion path in
        // dds_transport.hpp (a peer whose broadcast roster excludes us) -- both latch on the
        // FIRST bad signal, with none of the belief accumulator's multi-slot debounce, so BOTH
        // keep this original, unconditional value regardless of which detector is armed. This
        // codebase has a recorded incident of two healthy agents evicting each other after one
        // bad signal at this exact threshold (2026-07-28, d_0728_072027) -- shortening it here
        // would make that worse, not better.
        evict_after_lying_ = declare_parameter<int>("evict_after_lying", 3);
        // A belief conviction gets ITS OWN, shorter patience: the accumulator already spent ~8
        // slots accumulating that evidence (the debounce depth the patience above exists to
        // provide, done inside the belief itself), so stacking more on top just hands an
        // already-convicted liar more free approach time. Applies ONLY when this node's own
        // belief_blocked_ set names the peer (see beliefStep/maybeEvict) — never to a gate2 or
        // roster-exclusion block, which read evict_after_lying_ above instead.
        evict_after_belief_lying_ = declare_parameter<int>("evict_after_belief_lying", 1);
        corpse_anchor_knot_ = declare_parameter<int>("corpse_anchor_knot", admm::K_SEND);
        if (corpse_anchor_knot_ != admm::K_SEND)
            RCLCPP_WARN(get_logger(),
                        "[agent%d] corpse_anchor_knot=%d, not K_SEND=%d — A/B USE ONLY. The "
                        "terminal knot overshoots where the body actually parks by ~0.39 m, "
                        "leaving 0.92 m of real clearance against a 0.867 m contact line.",
                        self_id_, corpse_anchor_knot_, admm::K_SEND);
        resid_alpha_ = declare_parameter<double>("odom_residual_alpha", 0.2);
        // 0.30, and it MUST stay under the safety buffer. D_MIN 1.3 minus the 0.867 contact line
        // leaves 0.433 m of slack, so a gate at or above that tolerates a lie big enough to spend
        // the whole buffer — undetectable and sufficient to reach contact at the same time. The
        // old 0.5 was exactly that: measured 2026-07-30 with a 0.424 m residual (LIE=0.30, peer
        // cooperating normally, not deaf) the detector fired ZERO times while the liar closed to
        // 0.038 m of body gap. The floor is odom noise, not the gate: clean flight residual is
        // 0.006 m, so 0.30 still clears it 50x.
        resid_gate_ = declare_parameter<double>("odom_residual_gate", 0.30);
        // Trust layer. Every default is derived (trust.hpp); obs_sigma is the only measured one.
        trust_.lambda        = declare_parameter<double>("trust_lambda", trust_.lambda);
        trust_.clamp_step    = declare_parameter<double>("trust_clamp", trust_.clamp_step);
        trust_.credit_ratio  = declare_parameter<double>("trust_credit_ratio", trust_.credit_ratio);
        trust_.l_max         = declare_parameter<double>("trust_l_max", trust_.l_max);
        trust_.l_evict       = declare_parameter<double>("trust_l_evict", trust_.l_evict);
        trust_.sigma         = declare_parameter<double>("obs_sigma", trust_.sigma);
        trust_.d_lie         = declare_parameter<double>("trust_d_lie", trust_.d_lie);
        trust_.refute_ratio  = declare_parameter<double>("trust_refute_ratio", trust_.refute_ratio);
        obs_noise_seed_      = declare_parameter<int>("obs_noise_seed", 0);
        // Task 7 item 1: the range visible() gates first-hand evidence (and, widened, item 6's
        // geometric refutation of relayed reports) on. Same order of magnitude as the arena's own
        // scale (plum piles 2.2-2.3 m apart); 4.0 clears the fleet's own follow_range (1.5) and
        // evade_range (0.5) with room for an honest reporter to actually see its neighbour.
        obs_range_ = declare_parameter<double>("obs_range", 4.0);
        // Does the observation topic need a sensor model applied to it? True for the Gazebo
        // model pose, false for anything that measured something. Set from
        // config/observation_sources.yaml via the launch file; default true keeps every
        // recorded run reproducible.
        synthetic_sensor_ = declare_parameter<bool>("synthetic_sensor_model", true);
        // The decision boundary is d_lie/2, not d_lie: that is where the log-likelihood crosses
        // zero. It must stay well under the 0.433 m buffer AND well above the measured residual,
        // or the detector either misses damage or convicts honest robots.
        if (!(trust_.d_lie / 2.0 < 0.433))
            throw std::runtime_error("trust_d_lie/2 must stay under the 0.433 m safety buffer");
        if (!(trust_.l_max < -trust_.l_evict))
            throw std::runtime_error("trust_l_max >= |l_evict|: hearsay could evict alone");
        // A peer that lies every other slot settles at the fixed point of one lie step (-clamp_step)
        // and one credit step (+clamp_step*credit_ratio) each decayed by lambda:
        //   L* = clamp_step*(credit_ratio - lambda) / (1 - lambda^2)
        // Eviction requires L* < l_evict, i.e. credit_ratio < lambda + l_evict*(1-lambda^2)/clamp_step.
        // At or above that breakeven the duty-cycled liar this parameter exists to catch converges
        // to a positive plateau and is never convicted -- silently, since this is launch-time-only
        // like its siblings above.
        if (!(trust_.credit_ratio <
              trust_.lambda + trust_.l_evict * (1.0 - trust_.lambda * trust_.lambda) / trust_.clamp_step))
            throw std::runtime_error(
                "trust_credit_ratio at/above the duty-cycle breakeven: an alternating liar would never be evicted");
        // ARMED BY DEFAULT. This was true — measure first, block later — and that was right while
        // the gate was being calibrated. It is not right to ship: a fleet that only starts
        // checking when somebody flips a parameter is a fleet that runs unprotected every time
        // nobody remembers, and nothing warns an operator about it. Worse, the demo script only
        // armed it as part of injecting the attack, which quietly handed the fleet advance notice
        // no real deployment gets.
        // The calibration that justified the old default is done: clean residual measured at
        // 0.006 m against a 0.5 m gate, ~80x margin (2026-07-29). Set it true to DISARM — that
        // is the counterfactual arm, and it should be the deliberate act.
        log_only_ = declare_parameter<bool>("detection_log_only", false);
        // Stop broadcasting if our own obs/odom go stale (see ready()). Must be generous
        // enough that an RTF dip is not mistaken for a dead controller.
        input_stale_s_ = declare_parameter<double>("input_stale_s", 1.0);
        // #1 leader-aware follower brake (ported from fleet_centralized_node::followSpeed).
        // Uses peers' xnow from the consensus barrier so a dog slowed by a peg never gets
        // rear-ended by the constant-cruise follower behind it. Same defaults as centralized.
        follow_gain_ = declare_parameter<double>("follow_gain", 0.5);
        follow_desired_ = declare_parameter<double>("follow_desired", 0.72);
        follow_range_ = declare_parameter<double>("follow_range", 1.5);
        follow_floor_ = declare_parameter<double>("follow_floor", 0.05);
        // Evasion boost (see followSpeed): ask for MAX_VX while inside evade_range_ of an
        // untrusted peer's keep-out. Off restores the pre-2026-07-30 constant-reference behaviour.
        evade_boost_ = declare_parameter<bool>("evade_boost", true);
        evade_range_ = declare_parameter<double>("evade_range", 0.5);
        // Task 4b spike: each agent's OWN local pairwise safety net against every peer it can
        // observe, independent of edge_owner (see agent_core.hpp AgentCore ctor). Fixes obstacle
        // slot count at construction, so this is launch-time only — no dynamic re-toggle.
        // Default OFF: this is a feasibility spike, not yet the shipped behaviour.
        enable_peer_keepout_ = declare_parameter<bool>("enable_peer_keepout", false);
        follow_cone_cos_ = std::cos(declare_parameter<double>("follow_cone_deg", 60.0) * M_PI / 180.0);
        // Direction-aware body footprint (Vision60 base box 0.83 x 0.25 -> half 0.415 x 0.125).
        // The inter-agent clearance the node-layer safety enforces depends on how the two long
        // bodies are oriented along the line between them: nose-to-tail (~0.83) vs side-by-side
        // (~0.25). Circular D_MIN can't tell these apart -> it both under-guards head-to-tail and
        // over-freezes side-by-side. drift covers leg swing / tracking slack (calibration knob).
        fp_half_len_ = declare_parameter<double>("footprint_half_length", 0.415);
        fp_half_wid_ = declare_parameter<double>("footprint_half_width", 0.125);
        fp_drift_ = declare_parameter<double>("footprint_drift_margin", 0.10);
        const int deadline_ms = declare_parameter<int>("hop_deadline_ms", 20);
        k_send_ = declare_parameter<int>("k_send", admm::K_SEND);
        r_latch_ = declare_parameter<double>("yaw_latch_r", 0.25);
        latch_margin_ = declare_parameter<double>("yaw_latch_margin", 0.10);

        // OCS2 geometry from reference.info (never hardcoded)
        const std::string ref = declare_parameter<std::string>("reference_file", "");
        if (ref.empty()) throw std::runtime_error("reference_file param required");
        ocs2::loadData::loadCppDataType(ref, "comHeight", com_height_);
        Eigen::VectorXd default_joints(12);  // must be pre-sized for loadEigenMatrix
        ocs2::loadData::loadEigenMatrix(ref, "defaultJointState", default_joints);

        formation_ = std::make_unique<admm::LaplacianFormation>(admm::formations());
        formation_name_ = declare_parameter<std::string>("formation", "V");
        formation_->set_formation(formation_name_);
        adapter_ = std::make_unique<admm::MotionAdapter>(
            com_height_, default_joints, admm::N, ts_, 0.05, 0.2, 24, "path", 5, 0.02,
            admm::K_SEND, true, declare_parameter<double>("yaw_rate_max", 1.2));
        transport_ = std::make_unique<admm::DdsTransport>(
            this, self_id_, dogs_, edges_, std::chrono::milliseconds(deadline_ms));
        // C-experiment fault injection: dynamic params so a sweep can `ros2 param set` levels
        // on a LIVE fleet (no re-bring-up per level). Applied to incoming consensus msgs only.
        inj_drop_p_ = declare_parameter<double>("inject_drop_p", 0.0);
        inj_delay_ms_ = declare_parameter<double>("inject_delay_ms", 0.0);
        inj_jitter_ms_ = declare_parameter<double>("inject_jitter_ms", 0.0);
        // Experiment C, a DIFFERENT injection: this one corrupts what we SEND, so one lie is
        // broadcast once and every receiver judges the same bytes independently.
        declare_parameter<double>("inject_fake_offset", 0.0);
        // Task 10 item 2 (spec §9 A2-smear): fabricate what THIS agent claims to have SEEN of
        // a peer, own position left honest. Same "declare but discard" shape as
        // inject_fake_offset above -- runtime-only, never live from a launch value.
        declare_parameter<double>("inject_fake_evidence", 0.0);
        declare_parameter<int>("inject_fake_evidence_target", 0);
        // Task 10 item 3 (spec §9 A2-duty): period (slots) over which inject_fake_offset lies
        // for the first half and tells the truth for the second. 0 = always lying.
        declare_parameter<int>("inject_duty_cycle", 0);
        // Task 10 item 1: was a LAUNCH parameter (live from t=0); moved to runtime-only so it
        // can switch on together with inject_fake_offset at the experiment trigger (see
        // gate2() and the on-set callback below for why the old shape evicted the attacker
        // minutes before the attack). inject_odom_fake_peer names WHICH peer's odom copy this
        // node forges; inject_odom_fake is the offset in metres, applied identically on every
        // survivor node so they all judge the same forged sample and reach the same verdict.
        declare_parameter<double>("inject_odom_fake", 0.0);
        declare_parameter<int>("inject_odom_fake_peer", 0);
        // Attacker characterization: a Byzantine robot does not run our safety layer for OUR
        // benefit. With this set, peers this agent evicts get NO corpse keep-out in its own
        // QP/A* (eviction and the solo rebuild still happen), so it will genuinely drive at
        // them. Demo/threat-model knob; the fleet's own defence is untouched.
        inj_ignore_corpses_ = declare_parameter<bool>("inject_ignore_corpses", false);
        transport_->set_inject(inj_drop_p_, inj_delay_ms_, inj_jitter_ms_);
        // (the on-set callback is registered at the END of this constructor; see there)
        // Arena obstacle-CBF + A* (mirrors fleet_centralized_node; "" -> empty world). Every
        // agent knows the same 2D-known arena, so obstacle CBF and A* routing are fully local.
        const std::string arena_name = declare_parameter<std::string>("arena", "");
        const bool use_astar = declare_parameter<bool>("use_astar", false);
        const auto& arenas = admm::arenas();
        auto ait = arenas.find(arena_name);
        if (ait != arenas.end()) {
            arena_obs_ = ait->second.obstacles;
            arena_walls_ = ait->second.walls;
            arena_rects_ = ait->second.rects;
        }
        agent_ = std::make_unique<admm::AgentCore>(self_id_, dogs_, edges_, formation_.get(),
                                                   w_form_, arena_obs_, arena_walls_, hard_through_,
                                                   transport_.get(), robot_margin_,
                                                   enable_peer_keepout_);
        // Declared unconditionally: these bounds are "the ground this fleet operates on", which
        // Gate 1 needs whether or not A* is planning on it. (declare_parameter is once-only.)
        const double ax_min = declare_parameter<double>("astar_x_min", 0.0);
        const double ax_max = declare_parameter<double>("astar_x_max", 10.0);
        const double ay_min = declare_parameter<double>("astar_y_min", -5.0);
        const double ay_max = declare_parameter<double>("astar_y_max", 5.0);
        admm::GateLimits gl;
        gl.x_min = ax_min; gl.x_max = ax_max; gl.y_min = ay_min; gl.y_max = ay_max;
        gl.arena_slack = declare_parameter<double>("gate1_arena_slack", 20.0);
        gl.vel_margin = declare_parameter<double>("gate1_vel_margin", 3.0);
        gate_ = gl;
        transport_->set_gate_limits(gl);
        // SELF-CHECK, at bring-up, loudly. The contract above ("goals live inside the map")
        // fixes legitimate trajectories leaving the bounds; it does nothing about the bounds
        // themselves being wrong. The node default for astar_x_max is 10.0 while plum's last
        // post stands at x=15.68, so a launch that forgets to override it leaves Gate 1
        // rejecting every honest message from a dog past x=11 — and because a rejected message
        // is indistinguishable from a missing one, all three would evict each other and lose
        // the pairwise CBF entirely. That must fail at startup, not silently mid-run.
        RCLCPP_INFO(get_logger(), "[agent%d] Gate 1 bounds x[%.2f,%.2f] y[%.2f,%.2f] slack=%.1f",
                    self_id_, gl.x_min, gl.x_max, gl.y_min, gl.y_max, gl.arena_slack);
        for (const auto& ob : arena_obs_)
            if (ob.pos[0] < gl.x_min || ob.pos[0] > gl.x_max ||
                ob.pos[1] < gl.y_min || ob.pos[1] > gl.y_max)
                RCLCPP_ERROR(get_logger(),
                             "[agent%d] MISCONFIGURED: arena obstacle (%.2f,%.2f) lies OUTSIDE "
                             "the map x[%.2f,%.2f] y[%.2f,%.2f] — the fleet must walk where "
                             "Gate 1 will reject it. Fix astar_x_min/x_max/y_min/y_max.",
                             self_id_, ob.pos[0], ob.pos[1],
                             gl.x_min, gl.x_max, gl.y_min, gl.y_max);
        if (use_astar && !arena_obs_.empty()) {
            const double r_astar = declare_parameter<double>("astar_robot_radius", 0.60);
            const double res = declare_parameter<double>("astar_res", 0.15);
            std::vector<admm::AStarCircle> circles;
            for (const auto& o : arena_obs_) circles.push_back({o.pos[0], o.pos[1], o.radius});
            std::vector<admm::AStarRect> rects;
            for (const auto& r : arena_rects_)
                rects.push_back({r.center[0], r.center[1], r.size[0], r.size[1], r_astar});
            planner_.reset(new admm::AStarPlanner(res, r_astar, circles, ax_min, ax_max,
                                                  ay_min, ay_max, /*boundary_margin=*/0.45, rects));
            RCLCPP_INFO(get_logger(), "[agent%d] arena=%s obstacles=%zu A* enabled",
                        self_id_, arena_name.c_str(), arena_obs_.size());
        }

        const std::string ns = "robot" + std::to_string(self_id_);
        // Command/state callbacks live in their OWN reentrant group so the high-rate
        // DdsTransport stream (default group, feeding the blocking worker) can never starve
        // a low-rate goal/plan callback — that starvation randomly left one follower parked.
        cmd_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions cmd_opts;
        cmd_opts.callback_group = cmd_cbg_;
        target_pub_ = create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
            "/" + ns + "/" + ns + "_mpc_target", 1);
        stats_pub_ = create_publisher<admm_fleet_msgs::msg::CycleStats>("/" + ns + "/admm/stats", 10);
        obs_sub_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
            "/" + ns + "/" + ns + "_mpc_observation", 1,
            [this](ocs2_msgs::msg::MpcObservation::SharedPtr m) {
                std::lock_guard<std::mutex> l(mu_); obs_ = *m; has_obs_ = true; t_obs_ = now(); }, cmd_opts);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/" + ns + "/controller/odom", 1,
            [this](nav_msgs::msg::Odometry::SharedPtr m) {
                std::lock_guard<std::mutex> l(mu_);
                odom_ = *m; has_odom_ = true; t_odom_ = now();
                // Task 10 item 4 (inject_forged_obs): republish THIS agent's own real motion
                // onto a peer's global observation topic, impersonating its ground-truth
                // publisher. Piggybacks on this callback purely because it already fires at the
                // stand-in publisher's own high rate -- no separate timer needed.
                if (forged_obs_pub_) forged_obs_pub_->publish(*m);
                if (!has_goal_) {  // stand in place until commanded
                    goal_ = Eigen::Vector2d(m->pose.pose.position.x, m->pose.pose.position.y);
                    has_goal_ = true;
                }
            }, cmd_opts);
        // GATE 2's independent observation channel. A compromised agent owns the planning and
        // comms layer, so everything it SAYS is suspect; it does not own the state estimator,
        // so /robotJ/controller/odom is the one thing about robot J it cannot forge. Without a
        // channel outside the one being attacked there is no detector at all (Pasqualetti et
        // al. 2013: content-only detectors cannot see an undetectable attack).
        // Depth 1: only the latest position matters, and a backlog would age the residual.
        for (int j : dogs_) {
            if (j == self_id_) continue;
            peer_odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
                "/robot" + std::to_string(j) + "/controller/odom", 1,
                [this, j](nav_msgs::msg::Odometry::SharedPtr m) {
                    std::lock_guard<std::mutex> l(mu_);
                    peer_odom_[j] = Eigen::Vector2d(m->pose.pose.position.x,
                                                    m->pose.pose.position.y);
                }, cmd_opts));
        }
        // OBSERVATION CHANNEL — the input a compromised peer cannot author (spec section 5.1).
        // The channel above is still the PEER'S OWN report; an attacker that owns its estimator as
        // well as its planner makes both agree and a detector built only on that channel goes
        // blind (measured 2026-07-30). This one is what THIS robot's sensor sees of the peer.
        // /robotJ/hardware/odom is published by the Gazebo plugin from the model's world pose, not
        // by robot J's estimator or planner, so in simulation it is the closest available stand-in
        // for "my sensor looking at you" (spec section 7: a zero-fidelity stand-in — ground truth
        // + noise + range/occlusion; the noise is added where the sample is USED, in
        // updatePeerOffsets, not here, so the stored buffer stays a clean re-queryable history).
        //
        // Subscribed under a LOCAL name ("observed/robotJ") and remapped at launch to the global
        // topic — a useful seam for swapping in real perception later, but a remap grants no
        // security by itself: the resolved DDS topic is still one global name, and any process on
        // the domain may publish to it regardless of what this node calls it. What actually cannot
        // be forged is the DDS WRITER GUID that arrives with every sample
        // (rclcpp::MessageInfo::get_rmw_message_info().publisher_gid), so the first writer seen on
        // each peer's observation topic is latched and every other writer is dropped — the same
        // idea as the transport's from-topic check (dds_transport.hpp onState), stronger because
        // it does not trust the topic name at all.
        // Windowed by AGE, not sample count (review finding 4): the stand-in publisher runs at
        // ~250 Hz, so the old "30 samples" bound held only ~0.12 s of history -- a little over
        // one slot (TS=0.1s) -- while the smear check (beliefStep) queries roughly TWO slots
        // back (ev_slot's one-cycle broadcast lag). That query landed past the end of the buffer
        // essentially always, so `have_check` was silently false almost every time and a report
        // that could not be checked read identically to one that checked out clean. 0.5 s covers
        // several slots of lag with room for jitter.
        obs_window_s_ = declare_parameter<double>("obs_window_s", 0.5);
        for (int j : dogs_) {
            if (j == self_id_) continue;
            peer_truth_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
                "observed/robot" + std::to_string(j), 1,
                [this, j](nav_msgs::msg::Odometry::SharedPtr m, const rclcpp::MessageInfo& info) {
                    const auto& gid = info.get_rmw_message_info().publisher_gid;
                    std::lock_guard<std::mutex> l(mu_);
                    auto& pinned = obs_gid_[j];
                    if (!pinned) {
                        pinned = gid;
                    } else if (std::memcmp(pinned->data, gid.data, RMW_GID_STORAGE_SIZE) != 0) {
                        // Task 10 item 5: the pin's real guarantee is "a writer arriving AFTER
                        // the channel is established is rejected" -- not "any impostor is
                        // rejected" (an attacker that publishes FIRST wins the latch and the
                        // real publisher is the one dropped here instead). Either way, this
                        // peer's observation channel just went dark to us, and a benign
                        // publisher restart looks identical. The old behaviour was a THROTTLED
                        // warning nothing downstream ever read; count it too, so it can reach
                        // the run's verdict via CycleStats (see publishStats).
                        obs_dropped_this_cycle_.fetch_add(1, std::memory_order_relaxed);
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "[agent%d] second publisher on robot%d's observation topic — dropped "
                            "(that peer's observation channel is now silent to us)",
                            self_id_, j);
                        return;
                    }
                    auto& buf = peer_truth_[j];
                    const double t = now().seconds();
                    buf.push_back({t, Eigen::Vector2d(m->pose.pose.position.x, m->pose.pose.position.y)});
                    while (buf.size() > 1 && t - buf.front().t > obs_window_s_) buf.pop_front();
                }, cmd_opts));
        }
        // Task 10 item 4 (inject_forged_obs): the adversarial control arm for the observation
        // channel's publisher-GID pin (spec §5.1's TOFU boundary). ONE knob, two separately-
        // runnable cases that differ only in WHEN it is set:
        //   (a) late impostor  -- `ros2 param set` AFTER the real Gazebo publisher is already
        //       established. The pin has already latched onto the real writer's GID, so this
        //       loses the race and every sample from this node is rejected -- the defence's
        //       coverage case.
        //   (b) first mover    -- given a nonzero value HERE at construction (a launch
        //       parameter), so this node's publisher exists before the real one and wins the
        //       latch; the genuine publisher is then the one rejected as "the impostor". This
        //       is the pin's honest limit (spec: "guards established-first, not genuine"), and
        //       must be reportable, not a footnote.
        // The initial declared value is READ (unlike inject_fake_offset et al., which discard
        // it) specifically so case (b) is reachable from a launch parameter.
        armForgedObs(declare_parameter<int>("inject_forged_obs", 0));
        // Latch the per-dog goal (transient_local), same discovery-race reason as plan_sub_ below:
        // a short-lived `ros2 topic pub` goal burst can match one agent late and drop its goal for
        // that dog -> it parks at spawn while the others walk. Latching lets a late-matching agent
        // still receive the last goal. Publishers must offer transient_local too (arena/g2 scripts).
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/" + ns + "/goal", rclcpp::QoS(1).transient_local(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
                std::lock_guard<std::mutex> l(mu_);
                goal_ = clampGoal(Eigen::Vector2d(m->pose.position.x, m->pose.position.y));
                has_goal_ = true; yaw_latched_ = false;
            }, cmd_opts);
        // FleetPlan is an event-triggered one-shot; latch it (transient_local) so a robot
        // whose subscription matches the publisher late still receives the current plan —
        // otherwise a nondeterministic discovery race drops it for whichever dog matched last.
        plan_sub_ = create_subscription<admm_fleet_msgs::msg::FleetPlan>(
            "/formation/plan", rclcpp::QoS(1).transient_local(),
            [this](admm_fleet_msgs::msg::FleetPlan::SharedPtr m) {
                std::lock_guard<std::mutex> l(mu_);
                for (size_t k = 0; k < m->robot_ids.size(); ++k)
                    if (m->robot_ids[k] == self_id_) {
                        goal_ = clampGoal(Eigen::Vector2d(m->goals[2 * k], m->goals[2 * k + 1]));
                        has_goal_ = true; yaw_latched_ = false;
                    }
            }, cmd_opts);

        // Registered LAST, on purpose: rclcpp runs the on-set callbacks from declare_parameter
        // too, so a callback that refuses unknown names would make every parameter declared
        // after it throw at construction. Registering here means it only ever sees runtime sets.
        param_cb_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& ps) {
                rcl_interfaces::msg::SetParametersResult r;
                r.successful = true;
                for (const auto& p : ps) {
                    const std::string& n = p.get_name();
                    if (n == "inject_drop_p") inj_drop_p_ = p.as_double();
                    else if (n == "inject_delay_ms") inj_delay_ms_ = p.as_double();
                    else if (n == "inject_jitter_ms") inj_jitter_ms_ = p.as_double();
                    // Runtime speed. Exists for the compromised-robot demo (a hostile peer that
                    // closes at walking pace films as a stroll); MAX_VX 0.55 is the model's own
                    // ceiling, so a set above it buys nothing the state clip won't take back.
                    else if (n == "v") {
                        v_ = p.as_double();
                        RCLCPP_WARN(get_logger(), "[agent%d] reference speed v -> %.2f m/s",
                                    self_id_, v_);
                    }
                    else if (n == "inject_ignore_corpses") {
                        inj_ignore_corpses_ = p.as_bool();
                        RCLCPP_WARN(get_logger(), "[agent%d] corpse keep-outs %s", self_id_,
                                    inj_ignore_corpses_ ? "SUPPRESSED (hostile mode)" : "active");
                    }
                    else if (n == "inject_fake_offset") {
                        transport_->set_fake_offset(p.as_double());
                        RCLCPP_WARN(get_logger(), "[agent%d] COMPROMISED: broadcasting a %.2f m "
                                    "lie about its own position", self_id_, p.as_double());
                    }
                    // Task 10 item 2 (spec §9 A2-smear): fabricate what THIS agent claims to
                    // have SEEN of a peer; own position (inject_fake_offset above) untouched.
                    else if (n == "inject_fake_evidence") {
                        transport_->set_fake_evidence(p.as_double());
                        RCLCPP_WARN(get_logger(), "[agent%d] COMPROMISED: smearing robot%d's "
                                    "reputation by fabricating a %.2f m false sighting of it",
                                    self_id_, inj_fake_evidence_target_, p.as_double());
                    } else if (n == "inject_fake_evidence_target") {
                        inj_fake_evidence_target_ = static_cast<int>(p.as_int());
                        transport_->set_fake_evidence_target(inj_fake_evidence_target_);
                    }
                    // Task 10 item 3 (spec §9 A2-duty): period in slots inject_fake_offset lies
                    // over (first half lying, second half honest). 0 = always lying.
                    else if (n == "inject_duty_cycle") {
                        const int period = static_cast<int>(p.as_int());
                        transport_->set_duty_cycle(period);
                        RCLCPP_WARN(get_logger(), "[agent%d] inject_duty_cycle -> %d slot(s) "
                                    "(0 = always lying whenever inject_fake_offset != 0)",
                                    self_id_, period);
                    }
                    // Task 10 item 1: the odom half of the dual-channel forgery. Must be set in
                    // the SAME pset burst as inject_fake_offset at the experiment trigger, or
                    // gate2() spends the time in between differencing an honest claim against
                    // an already-forged odometry and evicts the attacker early (see agent_core.
                    // hpp's applyOdomFake for the measured number).
                    else if (n == "inject_odom_fake") {
                        inj_odom_fake_ = p.as_double();
                        RCLCPP_WARN(get_logger(), "[agent%d] COMPROMISED: forging robot%d's odom "
                                    "copy by %.2f m", self_id_, inj_odom_fake_peer_.load(),
                                    p.as_double());
                    } else if (n == "inject_odom_fake_peer") {
                        inj_odom_fake_peer_ = static_cast<int>(p.as_int());
                    }
                    // Task 10 item 4: see armForgedObs for the two cases this knob covers.
                    else if (n == "inject_forged_obs") {
                        armForgedObs(static_cast<int>(p.as_int()));
                    }
                    // The detection knobs. These used to fall through to "successful = true"
                    // while changing nothing: `ros2 param set detection_log_only false` reported
                    // OK and left the detector in log-only mode, so the calibration procedure
                    // ends by arming a gate that is not armed. An operator cannot see that.
                    else if (n == "detection_log_only") {
                        log_only_ = p.as_bool();
                        RCLCPP_WARN(get_logger(), "[agent%d] Gate 2 is now %s", self_id_,
                                    log_only_ ? "LOG-ONLY (not blocking)" : "BLOCKING");
                    } else if (n == "odom_residual_gate") {
                        resid_gate_ = p.as_double();
                    } else if (n == "odom_residual_alpha") {
                        resid_alpha_ = p.as_double();
                    } else if (n == "evict_after_lying") {
                        evict_after_lying_ = static_cast<int>(p.as_int());
                    } else if (n == "evict_after_belief_lying") {
                        evict_after_belief_lying_ = static_cast<int>(p.as_int());
                    } else {
                        // Anything else is read once at construction. Say so rather than
                        // accepting it: a silent no-op is worse than a refusal.
                        r.successful = false;
                        r.reason = n + " is not settable at runtime (read once at startup)";
                        RCLCPP_WARN(get_logger(), "[agent%d] REFUSED param '%s' — %s",
                                    self_id_, n.c_str(), r.reason.c_str());
                    }
                }
                transport_->set_inject(inj_drop_p_, inj_delay_ms_, inj_jitter_ms_);
                return r;
            });

        run_ = true;
        worker_ = std::thread([this] { loop(); });
        RCLCPP_INFO(get_logger(), "[admm_agent %d] dogs=%zu edges=%zu v=%.2f deadline=%dms com_h=%.2f",
                    self_id_, dogs_.size(), edges_.size(), v_, deadline_ms, com_height_);
    }

    ~AdmmAgentNode() override {
        run_ = false;
        if (worker_.joinable()) worker_.join();
    }

private:
    bool ready() {
        // size guard: during the WBC activation race the observation can arrive with a
        // short/empty state vector; indexing it (BASE_PX=6, BASE_YAW=9) would be UB.
        // Throttled, not silent: this branch stops the worker dead (no cycle, no broadcast) and
        // used to leave no trace, so "the agent just went quiet" had no entry point in the log.
        if (!(has_obs_ && has_odom_ && has_goal_ && obs_.time != 0.0 &&
              obs_.state.value.size() >= 24)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[agent%d] not ready: obs=%d odom=%d goal=%d t_obs=%.3f nstate=%zu",
                                 self_id_, int(has_obs_), int(has_odom_), int(has_goal_),
                                 obs_.time, obs_.state.value.size());
            return false;
        }
        // FRESHNESS, not just "has arrived once". has_obs_/has_odom_ latch true forever, so a dog
        // whose controller dies (WBC deactivation, controller crash) would keep stepping on its
        // last frozen observation and keep broadcasting AgentState from a position it can no
        // longer leave. Peers would never see silence, never evict it, and the fleet would wait
        // on a heartbeat-alive zombie indefinitely. Going quiet instead self-fences: it turns an
        // undetectable body failure into the plain silence the eviction path already handles.
        // A stamp of exactly 0 means the sample landed before this node had seen its first
        // /clock, so now() was still 0 when we stamped it. Comparing that against a later
        // sim time reports the whole elapsed sim time as "age" and self-fences a perfectly
        // healthy agent at bring-up (observed: "inputs stale 18.27s" == sim time itself).
        // Wait for a real stamp instead; one more sample arrives within a slot.
        if (t_obs_.nanoseconds() == 0 || t_odom_.nanoseconds() == 0) return true;
        const double age = std::max((now() - t_obs_).seconds(), (now() - t_odom_).seconds());
        if (age > input_stale_s_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[agent%d] inputs stale %.2fs (> %.2f) — holding OFF the wire so "
                                 "peers can evict me instead of waiting forever",
                                 self_id_, age, input_stale_s_);
            return false;
        }
        return true;
    }

    // ---- per-slot cycle (worker thread) ----
    void loop() {
        std::uint64_t last = 0;
        while (run_ && rclcpp::ok()) {
            const double t = now().seconds();
            const std::uint64_t slot = static_cast<std::uint64_t>(std::floor(t / ts_));
            // Sim /clock can jump backward (Gazebo reset, or the documented external /clock
            // cross-talk). `last` is monotone, so without re-arming, slot<=last would latch
            // forever and this dog would stop publishing targets. Re-arm on any regression.
            if (slot < last) last = 0;
            if (slot <= last || !snapshotReady()) { std::this_thread::sleep_for(2ms); continue; }
            last = slot;
            cycle(slot);
        }
    }
    bool snapshotReady() {
        std::lock_guard<std::mutex> l(mu_);
        return ready();
    }
    // Half-extent of the body box projected onto unit direction u, given body yaw. Rectangle
    // support: half_length along the forward axis, half_width along the side, + drift margin.
    double footprintSupport(const Eigen::Vector2d& u, double yaw) const {
        const double c = std::cos(yaw), s = std::sin(yaw);
        return fp_half_len_ * std::abs(u[0] * c + u[1] * s)
             + fp_half_wid_ * std::abs(-u[0] * s + u[1] * c) + fp_drift_;
    }
    // Direction-aware minimum center distance between two bodies along their separation dp:
    // both footprints projected toward each other. ~0.93 nose-to-tail, ~0.45 side-by-side.
    double dminEff(const Eigen::Vector2d& dp, double yaw_a, double yaw_b) const {
        const double n = dp.norm();
        if (n < 1e-9) return 2.0 * (fp_half_wid_ + fp_drift_);  // coincident: side-floor
        const Eigen::Vector2d u = dp / n;
        return footprintSupport(u, yaw_a) + footprintSupport(u, yaw_b);
    }
    // Peer heading from its velocity (dogs face travel dir); 0 when ~stopped (no rear-end risk).
    static double velYaw(const Eigen::Vector4d& xnow) {
        const Eigen::Vector2d v = xnow.segment<2>(2);
        return v.norm() > 0.05 ? std::atan2(v[1], v[0]) : 0.0;
    }
    // Leader-aware cruise speed for THIS dog: v_ unless a peer ahead (within cone+range) is
    // slower, then cap at v_ahead + gain*(gap - desired) so we decelerate to match rather than
    // driving into its rear. Peer states are last-cycle (consensus barrier); 1-slot stale is
    // fine at 10 Hz. NOTE: this is the live follower law and it DIVERGES from fleet_centralized_
    // node::followSpeed on purpose — the desired gap here is the direction-aware dminEff() (below),
    // whereas centralized still uses the static follow_desired_ scalar. The distributed agent is
    // the source of truth; the centralized "oracle" is stale. Do NOT downgrade this to match it
    // (regresses live behavior) — update centralized instead. (follow_desired_ is unused here.)
    double followSpeed(const Eigen::Vector2d& pi, const Eigen::Vector2d& goal, double self_yaw) {
        if (follow_gain_ <= 0.0) return v_;
        Eigen::Vector2d tgt = goal;
        if (path_.size() >= 2) {  // travel dir = A* lookahead (route bends away from goal bearing)
            size_t k0 = 0; double dbest = 1e18;
            for (size_t k = 0; k < path_.size(); ++k) {
                const double d = (path_[k] - pi).norm();
                if (d < dbest) { dbest = d; k0 = k; }
            }
            for (size_t k = k0; k < path_.size(); ++k)
                if ((path_[k] - pi).norm() > 0.3) { tgt = path_[k]; break; }
        }
        Eigen::Vector2d dir = tgt - pi;
        const double dn = dir.norm();
        if (dn < 1e-3) return v_;
        dir /= dn;
        double v_eff = v_;
        for (const auto& kv : agent_->peer_xnow()) {
            if (kv.first == self_id_) continue;
            const Eigen::Vector2d d = kv.second.head<2>() - pi;
            const double dist = d.norm();
            if (dist < 1e-3 || dist > follow_range_) continue;
            const double proj = d.dot(dir);
            if (proj <= 0.0 || proj / dist < follow_cone_cos_) continue;  // not ahead / outside cone
            // desired gap is direction-aware: keep clear of a nose-to-tail peer (~0.93), but
            // don't brake for one merely alongside (~0.45) -> no false stall in tight columns.
            const double desired = dminEff(d, self_yaw, velYaw(kv.second));
            const double v_ahead = std::max(0.0, kv.second.segment<2>(2).dot(dir));
            const double v_allow = v_ahead + follow_gain_ * (dist - desired);
            v_eff = std::min(v_eff, std::max(v_allow, follow_floor_));
        }
        // EVASION BOOST. The keep-out around an untrusted peer is sized on the THREAT's top speed
        // (corpse_keepout: + MAX_VX * TAU_REACT) while the reference only ever asks this dog for
        // v_ = 0.4. MAX_VX is a box bound, not a cap on us — the QP would allow 0.55 — but the
        // tracking cost pins us to the reference, so a hostile peer running at MAX_VX closes the
        // margin faster than we open it. Measured 2026-07-29/30: attacker 0.55 vs fleet 0.4 drove
        // the QP to OSQP max-iter 179 times, poisoned the dual to NaN, and ended in a fleet-wide
        // freeze; the same attack with the gap removed produced 6. Ask for the speed the margin
        // was designed against, and only while a keep-out is actually near — cruising the whole
        // fleet at MAX_VX instead was tried and overlapped bodies in 1 run of 3.
        if (!corpses_.empty() && evade_boost_) {
            for (const auto& kv : corpses_)
                for (const auto& o : kv.second)
                    if ((o.pos - pi).norm() < o.radius + evade_range_)
                        return std::max(v_eff, admm::MAX_VX);
        }
        return v_eff;
    }
    // CONSERVATIVE ANCHORING'S OFFSET SOURCE (spec 4.1). Turns this cycle's observation of each
    // peer into the correction AgentCore::step will apply to that peer's WHOLE broadcast before
    // building the barrier (agent_core.cpp, section 2a). Must run BEFORE agent_->step() for this
    // cycle: AgentCore reads peer_offset_ synchronously inside step(), so an offset set after the
    // call would land one cycle late.
    //
    // `claimed` comes from agent_->peer_xnow(), which after the LAST completed step() holds
    // exactly what peer j broadcast in the cycle transport_->last_seen()[j] names (the same
    // one-cycle lag followSpeed already reads peer state under). The observation is interpolated
    // to THAT slot, not to "now": comparing a peer's claim against an observation taken at a
    // different time manufactures a correction nobody's claim asked for, the same reasoning that
    // keeps Gate 2 from differencing a frozen HOLD claim against live odom.
    //
    // No FRESH observation for a peer this cycle (never subscribed, stream stalled, the query
    // lands outside interp_at's window, or -- review round 2 on task 7 finding 6 -- not
    // currently visible) -> HOLD that peer's last-computed offset rather than dropping it.
    //
    // Dropping to zero the instant a peer cannot be observed reopens the exact judge/constraint
    // asymmetry finding 6 closed, on the constraint side: an attacker that steps behind a pillar,
    // or beyond sensing range, would get its raw self-reported claim fully re-trusted at the
    // precise moment it has the most incentive to lie, while set_peer_keepout (same visibility
    // gate, same fix) already HOLDS a peer's keep-out at its last-known position when it drops
    // out of the map handed to it. Holding here makes both mechanisms fail the same way instead
    // of diverging: whichever direction the peer's true behaviour actually goes while unobserved,
    // holding is never worse than zero (the peer keeps lying similarly -> holding is correct; it
    // stops lying -> holding over-corrects, costing a little extra standoff, the conservative
    // direction; it lies MORE while hidden -> holding under-corrects, but strictly less than the
    // zero correction it would otherwise get). No separate expiry: a peer that is genuinely gone
    // for good is the crash-failover/eviction path's job (maybeEvict -> rebuild constructs a
    // fresh AgentCore, which starts this peer's offset over at zero the same as a peer never
    // seen) -- a SEPARATE timer here would duplicate that mechanism for the one case (persistent
    // occlusion of a peer still very much in the consensus) where holding is already bounded-safe
    // by the reasoning above, matching set_peer_keepout's own no-expiry precedent.
    // ONE definition of "what do I observe of peer j at this instant", for both the anchoring
    // path and the belief layer. They must not diverge: an earlier version had anchoring
    // correcting the barrier toward a peer the belief layer had just declared unobservable
    // (task 7 review finding 6), which is safety-positive by accident and indefensible on
    // paper. Sharing the function makes that structural rather than a thing to remember.
    //
    // With a synthetic source the sensor model lives HERE, because the topic carries none:
    // the model pose is a feed of answers until range, occlusion and noise are applied to it.
    // With a real one none of that belongs here. The tracker publishes only the peers it
    // actually resolved, so having a sample IS the visibility test, and the sample already
    // carries the sensor's own error -- adding the stand-in's would count both twice, and the
    // occlusion test would blind the fleet a second time to peers it can plainly see.
    std::optional<Eigen::Vector2d> observedPeer(std::uint64_t slot, int j,
                                                const Eigen::Vector2d& self_p,
                                                const Eigen::Vector2d& raw) const {
        if (!synthetic_sensor_) return raw;
        if (!admm::visible(self_p, raw, arena_obs_, obs_range_)) return std::nullopt;
        return raw + admm::obs_noise(slot, self_id_, j, trust_.sigma, obs_noise_seed_);
    }

    void updatePeerOffsets(std::uint64_t slot, const Eigen::Vector2d& self_p) {
        std::map<int, std::deque<admm::ObsSample>> truth;
        { std::lock_guard<std::mutex> l(mu_); truth = peer_truth_; }
        const auto seen = transport_->last_seen();
        std::map<int, Eigen::Vector2d> off;
        // task 4b: same observation, same slot alignment, same noise -- the local pairwise
        // keep-out is not a second sensor, it is a second constraint reading the first one.
        std::map<int, Eigen::Vector2d> keepout_pos;
        for (const auto& kv : agent_->peer_xnow()) {
            const int j = kv.first;
            if (j == self_id_) continue;
            // BUG FIX (found in review): std::map::operator[]'s "value-init a fresh key"
            // guarantee zero-initializes a scalar -- why resid_[j]/l_act_[j] elsewhere in this
            // file are safe -- but Eigen::Vector2d's default constructor is a deliberate no-op
            // for performance (no EIGEN_INITIALIZE_MATRICES_BY_ZERO/NAN in this build), so a
            // never-written key's operator[] returns whatever bytes were already in the freshly
            // allocated map node, not zero. Verified empirically: 20/20 fresh keys came back
            // (12345.6789, 12345.6789) on this system. Both reads below (this one and `prev` a
            // few lines down) go straight into conservative_offset -> anchor_peer, a live safety
            // constraint, so a garbage seed here corrupted the peer's translated broadcast for
            // one cycle -- every run, at bring-up, before any peer has ever been anchored. Insert
            // the zero EXPLICITLY, once, before anything can read a never-written entry; both
            // later uses of peer_offset_prev_[j] this iteration then hit an already-real entry.
            if (peer_offset_prev_.find(j) == peer_offset_prev_.end())
                peer_offset_prev_.emplace(j, Eigen::Vector2d::Zero());
            // Default: hold. Overwritten below only when this cycle actually produces a fresh,
            // visible observation.
            off[j] = peer_offset_prev_[j];
            const auto tb = truth.find(j);
            if (tb == truth.end()) continue;   // no observation channel heard from j at all
            const auto sit = seen.find(j);
            if (sit == seen.end()) continue;   // no claim to anchor
            const auto obs = admm::interp_at(tb->second, static_cast<double>(sit->second) * ts_);
            if (!obs) continue;   // stream stalled or hasn't started yet: hold, no fresh evidence
            // TASK 7 REVIEW FINDING 6: the same range/occlusion gate beliefStep uses for
            // first-hand evidence, applied here too. Without it, a peer hiding behind a pillar
            // from everyone but its victim was still judged unobservable by the belief layer
            // while this function kept anchoring the barrier and moving the local keep-out to a
            // position it had just declared un-sensed -- safety-positive by accident, but a
            // stand-in sensor model that the paper's own safety mechanism ignores is not
            // defensible. Not visible -> hold (see the function comment for why hold, not drop).
            const auto seen_now = observedPeer(slot, j, self_p, *obs);
            if (!seen_now) continue;   // not observable this cycle -> hold (see the comment above)
            const Eigen::Vector2d noisy = *seen_now;
            // Guaranteed to already exist (seeded to Zero() above if this is j's first
            // iteration ever) -- operator[] here can never again value-init a fresh Eigen key.
            auto& prev = peer_offset_prev_[j];
            off[j] = admm::conservative_offset(self_p, kv.second.head<2>(), noisy,
                                               3.0 * trust_.sigma, admm::MAX_VX * ts_, prev);
            prev = off[j];
            keepout_pos[j] = noisy;
        }
        agent_->set_peer_offsets(off);
        agent_->set_peer_keepout(keepout_pos);
    }
    // THE CONTRACT: goals live inside the A* map. The planner already clamps an outside goal
    // and walks the last leg straight, but leaving the contract implicit is what let a demo
    // send the fleet to x=-5 against x_min=-2 and then produced a class of failures nobody was
    // looking for (2026-07-28). Clamping HERE makes "where may this fleet be told to go" one
    // answer, stated once, and audible when something asks for more.
    Eigen::Vector2d clampGoal(const Eigen::Vector2d& g) const {
        const Eigen::Vector2d c(clip(g[0], gate_.x_min, gate_.x_max),
                                clip(g[1], gate_.y_min, gate_.y_max));
        // Throttled: the plan is republished continuously, so an out-of-map slot goal produced
        // 593 identical lines in one run and buried everything else.
        if ((c - g).squaredNorm() > 1e-12)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                        "[agent%d] goal (%.2f,%.2f) is outside the map x[%.1f,%.1f] y[%.1f,%.1f]"
                        " — clamped to (%.2f,%.2f)",
                        self_id_, g[0], g[1], gate_.x_min, gate_.x_max, gate_.y_min, gate_.y_max,
                        c[0], c[1]);
        return c;
    }

    void cycle(std::uint64_t slot) {
        transport_->set_now_slot(slot);  // Gate 1 bounds a claimed cycle_id against OUR clock
        ocs2_msgs::msg::MpcObservation obs;
        nav_msgs::msg::Odometry odom;
        Eigen::Vector2d goal;
        { std::lock_guard<std::mutex> l(mu_); obs = obs_; odom = odom_; goal = goal_; }

        // x0: odom position + EMA/clamped observed velocity (mirrors centralized x0)
        const auto& s = obs.state.value;
        Eigen::Vector2d P0(odom.pose.pose.position.x, odom.pose.pose.position.y);
        Eigen::Vector2d vraw(s[admm::MOM_LIN_X], s[admm::MOM_LIN_Y]);
        if (!has_vema_) { v_ema_ = vraw; has_vema_ = true; }
        else v_ema_ = 0.25 * vraw + 0.75 * v_ema_;
        Eigen::Vector4d X0(P0[0], P0[1], clip(v_ema_[0], -admm::MAX_VX, admm::MAX_VX),
                           clip(v_ema_[1], -admm::MAX_VY, admm::MAX_VY));

        // Reference polyline: straight P0->goal, or the A* detour when an arena is loaded.
        // Cache per goal (replan only when the goal moves) — cycle() is the sole writer of
        // path_ (single worker thread), so no lock. Mirrors centralized waypoints()/goalCb.
        Eigen::MatrixX2d wp;
        if (planner_ && (goal - path_goal_).norm() > 1e-6) {
            // Ask the planner only about ground it has a map for. A goal outside the map lands
            // on a grid cell outside the array, and nearest_free_candidates only spirals 20
            // cells (3 m) around it, so anything further out returns NO candidates — measured
            // live: a return goal at x=-5 against x_min=-2 produced 68 empty plans, a straight
            // line through the corpse keep-out, and 0.513 m of separation (contact is 0.867).
            const Eigen::Vector2d qgoal(clip(goal[0], planner_->x_min(), planner_->x_max()),
                                        clip(goal[1], planner_->y_min(), planner_->y_max()));
            const auto rg = planner_->find_reachable_goal(P0, qgoal);
            if (rg.path.empty()) {
                // FAIL-SAFE, not a straight line: the line is exactly what crosses a keep-out,
                // linearizes the CBF to l>u and kills the QP. Keep the last valid route (still
                // an obstacle-free corridor) and DON'T latch path_goal_, so the next cycle
                // retries. A stopped dog is a safe failure; a dog steered blind is not.
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                      "[agent%d] A* found NO path to (%.2f,%.2f) from (%.2f,%.2f)"
                                      " — HOLDING (no straight-line fallback)",
                                      self_id_, qgoal[0], qgoal[1], P0[0], P0[1]);
            } else {
                path_ = rg.path;
                // Beyond the map edge there is nothing known to route around, so the final leg
                // to a clamped-away goal is an honest straight line.
                if ((qgoal - goal).squaredNorm() > 1e-12) path_.push_back(goal);
                path_goal_ = goal;
            }
        }
        if (planner_) {
            // No route yet (first goal unplannable) -> stand still. Degenerate two-point
            // polylines are handled in build_reference (guarded 2026-07-25).
            const std::vector<Eigen::Vector2d> hold{P0, P0};
            const auto& src = path_.empty() ? hold : path_;
            wp.resize(static_cast<int>(src.size()), 2);
            for (int k = 0; k < wp.rows(); ++k) wp.row(k) = src[k].transpose();
        } else {
            wp.resize(2, 2);
            wp.row(0) = P0.transpose();
            wp.row(1) = goal.transpose();
        }
        // leader-aware cruise: brake for a slower peer ahead (see followSpeed). Peer xnow is
        // last cycle's (empty on cycle 0 -> returns v_). Ported from the centralized node.
        const double v_cruise = followSpeed(P0, goal, s[admm::BASE_YAW]);
        Eigen::MatrixXd xdes = admm::build_reference(P0, wp, admm::N, admm::TS, v_cruise, 0.80);

        updatePeerOffsets(slot, P0);  // conservative anchoring's offsets for THIS cycle's step()
        // Task 7 / spec 5.2: hand over LAST cycle's beliefStep observations so THIS cycle's
        // step() broadcasts them (send_state runs at the top of step(), before this cycle's own
        // beliefStep has produced anything yet) -- see AgentCore::set_evidence.
        agent_->set_evidence(ev_out_peer_, ev_out_pos_, ev_out_slot_);

        admm::StepResult res;
        const auto t_wall0 = std::chrono::steady_clock::now();
        try {
            res = agent_->step(X0, xdes, slot);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "step failed: %s", e.what());
            // Drain this cycle's read-and-reset telemetry so a partial (aborted) cycle's traffic
            // and latency samples don't carry into the next CycleStats row.
            transport_->take_wait_times(); transport_->take_bytes();
            transport_->take_rx_mean(); transport_->take_stale();
            admm::take_qp_health();
            return;
        }
        const double t_cycle_wall =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t_wall0).count();
        // Did any QP this cycle hand back a non-SOLVED iterate that we accepted anyway?
        // Silent today (callers only test allFinite()); we are measuring whether it ever
        // fires at all before deciding whether it should be treated as a failure.
        if (const auto qh = admm::take_qp_health(); qh.n_nonconverged > 0)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "QP not converged: %lu solve(s) this cycle accepted with "
                                 "osqp status_val=%d (no feasibility guarantee)",
                                 qh.n_nonconverged, qh.last_status_val);
        // Warm start thrown away? Say so. has_prev() gates eviction arming, so a silent clear
        // used to turn into "this agent never evicts anyone" with nothing in the log to explain it.
        // PROBE for the plan-vs-measurement gap. Logged only in the tail (>0.2 m) because that
        // is the part with an open question: the Gate 1 chain check died on values of 0.6–1.3 m,
        // and the suspected driver is HOLD freezing the plan while the body walks off the last
        // published prefix — which caps the drift at prefix*speed ~0.4 m and therefore does NOT
        // explain 1.335. Recording speed alongside separates "drift, still moving" from
        // "parked, so something else". One line per occurrence, no throttle: it is rare.
        if (res.chain_margin > 0.2)
            RCLCPP_INFO(get_logger(),
                        "chainprobe slot=%lu peer=%d margin=%.3f warm_cleared=%d hold=%d "
                        "hold_streak=%d speed=%.3f",
                        static_cast<unsigned long>(slot), self_id_, res.chain_margin,
                        res.warm_cleared, int(res.hold), res.hold_streak,
                        std::hypot(X0[2], X0[3]));
        if (res.warm_cleared != admm::StepResult::kWarmKept)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[agent%d] warm start dropped (%s) — cold-starting consensus",
                                 self_id_,
                                 res.warm_cleared == admm::StepResult::kWarmNaN
                                     ? "non-finite xi" : "peer announced reset");
        if (res.hold || !res.xi.allFinite()) {
            // Detection has to survive the fleet standing still. gate2 used to run only below, on
            // the SOLVED path, so a liar that jams the barrier — announcing reset every slot, or
            // quitting the consensus while it keeps talking — froze every survivor into a
            // permanent HOLD in which the one mechanism that could catch it never executed
            // (measured 2026-07-28, d_0728_095209: "GATE2 would" fired 0 times across a dead run).
            // trackMobileCorpses() moves here for the same reason: a keep-out circle that stops
            // following its body while we are held is fencing yesterday's position.
            // res.peer_xnow_fresh tells detect()/beliefStep whether peer_xnow() was actually
            // refreshed this slot (see beliefStep's comment) -- gate2 ignores the flag entirely.
            // NOT res.n_timeouts==0: an EdgeXi/EdgeZ timeout deep in the ADMM loop also bumps
            // n_timeouts on an otherwise-solved cycle where peer_xnow_ is perfectly fresh.
            detect(slot, res.peer_xnow_fresh, P0);
            trackMobileCorpses();
            if (res.hold) maybeEvict(slot);
            publishStats(slot, res, t_cycle_wall);
            return;
        }
        // Bring-up grace is satisfied HERE — one cycle that actually solved — and never again.
        // It used to be tested inside maybeEvict as has_prev(), which is reachable only on the
        // HOLD path and is cleared by a NaN or a peer's reset announcement. Those clears are
        // exactly what a dying peer causes, and once a peer is gone every later cycle is a
        // barrier HOLD, so has_prev() can never come back: the fleet waits forever for the dead
        // dog it is not allowed to evict. Measured 2026-07-28 (d_0728_045133): 51 straight
        // "NOT ARMED", peer silent 370 slots, zero evictions, run dead.
        evict_armed_ = true;
        detect(slot, res.peer_xnow_fresh, P0);
        trackMobileCorpses();

        // ADAPT: translate xi to estimator frame, build 24-D target, yaw latch, publish.
        Eigen::VectorXd xi_mpc = res.xi;
        const double dx = s[admm::BASE_PX] - P0[0], dy = s[admm::BASE_PY] - P0[1];
        for (int k = 1; k <= admm::N; ++k) {
            xi_mpc[admm::px_index(k)] += dx;
            xi_mpc[admm::py_index(k)] += dy;
        }
        // Direction-aware safe send-prefix (node-layer footprint; ADMM core CBF unchanged).
        // Publish only the longest lead where THIS dog stays clear of every peer's rollout by the
        // ORIENTATION-dependent body clearance (nose-to-tail ~0.93, side-by-side ~0.45) rather
        // than a blind circular D_MIN -> guards the head-to-tail (rear-end) collision without
        // freezing dogs merely passing alongside. Ported from publisher L478-480; peers are last
        // cycle (their broadcast xibar positions, yaw from their velocity).
        Eigen::VectorXd px_i(admm::N), py_i(admm::N);
        for (int k = 1; k <= admm::N; ++k) { px_i[k - 1] = res.xi[admm::px_index(k)]; py_i[k - 1] = res.xi[admm::py_index(k)]; }
        const double self_yaw = s[admm::BASE_YAW];
        struct Peer { Eigen::VectorXd px, py; double yaw; };
        std::vector<Peer> peers;
        const auto& pxnow = agent_->peer_xnow();
        for (const auto& kv : agent_->peer_xibar()) {
            if (kv.first == self_id_ || kv.second.size() <= admm::py_index(admm::N)) continue;
            Peer p; p.px.resize(admm::N); p.py.resize(admm::N);
            for (int k = 1; k <= admm::N; ++k) { p.px[k - 1] = kv.second[admm::px_index(k)]; p.py[k - 1] = kv.second[admm::py_index(k)]; }
            const auto it = pxnow.find(kv.first);
            p.yaw = (it != pxnow.end()) ? velYaw(it->second) : 0.0;
            peers.push_back(std::move(p));
        }
        int k_send = k_send_;
        if (!peers.empty()) {
            int K = 0;
            const int kmax = std::min<int>(k_send_, admm::N);
            for (int m = 0; m < kmax; ++m) {
                bool clear = true;
                for (const auto& p : peers) {
                    const Eigen::Vector2d dp(px_i[m] - p.px[m], py_i[m] - p.py[m]);
                    if (dp.norm() < dminEff(dp, self_yaw, p.yaw)) { clear = false; break; }
                }
                if (!clear) break;
                K = m + 1;
            }
            k_send = std::max(1, K);
        }
        auto out = adapter_->build_target(xi_mpc, obs.time, s[admm::BASE_YAW], "path", k_send);
        Eigen::MatrixXd states = out.states;

        const double d_goal = std::hypot(P0[0] - goal[0], P0[1] - goal[1]);
        if (!yaw_latched_) {
            if (d_goal < r_latch_) { yaw_latched_ = true; yaw_latch_val_ = s[admm::BASE_YAW]; }
        } else if (d_goal > r_latch_ + latch_margin_) {
            yaw_latched_ = false;
        }
        if (yaw_latched_)
            for (int r = 0; r < states.rows(); ++r) states(r, admm::BASE_YAW) = yaw_latch_val_;

        target_pub_->publish(admm::toTargetMsg(out.times, states));
        publishStats(slot, res, t_cycle_wall);
    }

    // Static terrain + every evicted peer's keep-out. This is what the node QP and the A*
    // both consume; arena_obs_ alone is never passed anywhere after bring-up.
    std::vector<admm::Obstacle> allObstacles() const {
        std::vector<admm::Obstacle> all = arena_obs_;
        for (const auto& kv : corpses_) all.insert(all.end(), kv.second.begin(), kv.second.end());
        return all;
    }
    // Where peer j's keep-out sits in that vector — same order as allObstacles(), which is the
    // order the AgentCore was constructed with, so this index is valid for set_obstacle_pos.
    std::size_t corpseIndex(int j) const {
        std::size_t idx = arena_obs_.size();
        for (const auto& kv : corpses_) {
            if (kv.first == j) return idx;
            idx += kv.second.size();
        }
        return idx;
    }

    // A peer evicted for LYING is still walking. Its keep-out must walk with it, or the fleet
    // dodges a ghost and hits the body — the same mistake the corpse anchor already exists to
    // avoid, except here it repeats every slot instead of once.
    // Moving the obstacle in the node QP is free (set_obstacle_pos: values only, no rebuild,
    // no cold start). The A* copy is NOT free — set_circles rebuilds the occupancy grid — so it
    // is refreshed only once the tracked peer has drifted far enough for the cached route to be
    // wrong, and that refresh forces a replan. ponytail: 0.5 m is 3 A* cells; tighten only if a
    // run shows the route stale enough to matter.
    // Where a mobile corpse's keep-out circle is drawn. It follows what we BELIEVE about that
    // peer, and the two ways a peer gets blocked support opposite beliefs:
    //   caught lying (Gate 2)   -> its claim is the thing we disproved, so fence its odom (body)
    //   announced it left (T2)  -> nothing has disproved its position, so fence what it claims
    // Follow what you believe. onState's ordering makes the two exclusive: a peer Gate 2 blocked
    // first can never enter exited_ afterwards, because a blocked message returns before the
    // exit test runs. Returns false when we have no anchor at all — caller decides the fallback.
    bool corpseAnchor(int j, Eigen::Vector2d* out) {
        if (transport_->exited(j) && transport_->latest_claim(j, *out)) return true;
        std::lock_guard<std::mutex> l(mu_);
        const auto it = peer_odom_.find(j);
        if (it == peer_odom_.end()) return false;
        *out = it->second;
        return true;
    }

    void trackMobileCorpses() {
        if (mobile_corpse_.empty()) return;
        bool astar_stale = false;
        for (int j : mobile_corpse_) {
            auto cit = corpses_.find(j);
            Eigen::Vector2d anchor;
            if (cit == corpses_.end() || cit->second.empty() || !corpseAnchor(j, &anchor)) continue;
            cit->second[0].pos = anchor;
            agent_->set_obstacle_pos(corpseIndex(j), anchor);
            auto& last = astar_corpse_[j];
            if ((last - anchor).norm() > 0.5) { last = anchor; astar_stale = true; }
        }
        if (astar_stale && planner_) {
            std::vector<admm::AStarCircle> circles;
            for (const auto& ob : allObstacles()) circles.push_back({ob.pos[0], ob.pos[1], ob.radius});
            planner_->set_circles(circles);
            path_goal_ = Eigen::Vector2d(1e9, 1e9);  // force a replan around where it is NOW
        }
    }

    // Experiment D: a peer whose AgentState has been absent for evict_after_ consecutive
    // slots is dead — remove it from the fleet graph and rebuild the AgentCore over the
    // survivors so consensus (and motion) continues without it. The corpse's last known
    // position becomes a static CBF obstacle (we have no perception; a fallen dog is
    // otherwise invisible). Formation term is dropped post-eviction: a 3-slot shape has
    // no defined meaning for 2 dogs — survivors keep chasing their latched goals with
    // pairwise CBF safety. Rejoin is NOT supported (a returning peer stays ignored).
    // Runs on the worker thread only (same thread as cycle()), so no locking needed.
    // GATE 2 — the detector that actually decides, and the only one that can catch a lie a
    // healthy agent could plausibly have produced. It anchors xnow to odom and nothing else.
    //
    // This comment used to say Gate 1 chains xibar to xnow, so anchoring one anchors both. It
    // does not: that check was removed when honest traffic showed 0.597-1.335 m of legitimate
    // divergence (see agent_core.hpp). Anchoring xnow therefore leaves the PLAN unchecked, and
    // the plan is what the pairwise barrier linearises around — an attacker can be truthful
    // about where it is, earning a clean residual here, while drifting its knots up to
    // MAX_VX*TS*vel_margin = 0.165 m each, which erodes the effective standoff from 1.300 m to
    // 0.354 m, past the 1.22 m knee-to-knee contact distance. Closing it needs one more term in
    // the residual, not a revived Gate 1 bound: a HOLD legitimately produces the same
    // divergence that killed the original check.
    //
    // Low-passed because a single sample cannot separate a lie from odom noise, and armed only
    // after a solved cycle: during bring-up odom and the broadcast state are not yet the same
    // dog's idea of "now" and the residual is meaningless.
    //
    // Exceeding the gate does NOT introduce a new failure mode — it increments the same
    // dropped_ counter Gate 1 uses, the peer stops being committed, its last_seen_ freezes and
    // the crash-failover path evicts it. One mechanism, two entry points.
    // Task 10 item 4 (inject_forged_obs): create the impersonating publisher for `target`'s
    // observation topic the first time it is armed (via launch value at construction, or later
    // via the on-set callback). One target per run -- changing it after a publisher already
    // exists would need a brand new publisher anyway, and the experiment only ever impersonates
    // one peer at a time.
    void armForgedObs(int target) {
        std::lock_guard<std::mutex> l(mu_);
        if (target == 0) return;
        if (target == self_id_) {
            RCLCPP_WARN(get_logger(), "[agent%d] inject_forged_obs=%d ignored (cannot "
                        "impersonate self)", self_id_, target);
            return;
        }
        if (forged_obs_pub_) {
            if (target != forged_obs_target_)
                RCLCPP_WARN(get_logger(), "[agent%d] inject_forged_obs already locked to robot%d "
                            "-- ignoring change to robot%d", self_id_, forged_obs_target_, target);
            return;
        }
        forged_obs_target_ = target;
        forged_obs_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/robot" + std::to_string(target) + "/hardware/odom", rclcpp::QoS(1));
        RCLCPP_WARN(get_logger(), "[agent%d] COMPROMISED: impersonating robot%d's ground-truth "
                    "observation channel — first writer on that topic wins the GID pin",
                    self_id_, target);
    }

    void gate2(std::uint64_t slot) {
        if (!evict_armed_) return;
        std::map<int, Eigen::Vector2d> od;
        { std::lock_guard<std::mutex> l(mu_); od = peer_odom_; }
        // Task 10 item 1: forge THIS node's copy of inject_odom_fake_peer's odom by
        // inject_odom_fake metres. Inert (no-op) unless both are nonzero -- see applyOdomFake.
        admm::applyOdomFake(od, inj_odom_fake_peer_.load(), inj_odom_fake_.load());
        const auto seen = transport_->last_seen();
        for (const auto& kv : agent_->peer_xnow()) {
            const int j = kv.first;
            if (j == self_id_) continue;
            const auto it = od.find(j);
            if (it == od.end()) continue;  // no independent channel yet -> no opinion
            // FRESHNESS. peer_xnow() holds the last COMMITTED claim, which during a barrier-miss
            // HOLD is a frozen copy while that peer's body keeps walking. Differencing a stale
            // claim against live odom manufactures a residual that grows with the silence and
            // would evict an honest dog for being briefly unheard — the same false-positive
            // shape that cost two survivors their pairwise CBF on 2026-07-28. Being quiet is
            // maybeEvict's business; Gate 2 only judges peers that are currently talking.
            const auto sit = seen.find(j);
            if (sit == seen.end() || slot > sit->second + 2) continue;
            const double r = (it->second - kv.second.head<2>()).norm();
            double& R = resid_[j];
            R = (1.0 - resid_alpha_) * R + resid_alpha_ * r;
            // Logged unconditionally: this line IS the calibration procedure. Run the fleet
            // clean with detection_log_only, take max R, set the gate well above it.
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[agent%d] residual robot%d r=%.3f R=%.3f gate=%.2f | "
                                 "gate1_worst=%.3f dropped=%u",
                                 self_id_, j, r, R, resid_gate_,
                                 transport_->gate_worst(), transport_->dropped(j));
            if (R <= resid_gate_) continue;
            if (log_only_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "[agent%d] GATE2 would reject robot%d (R=%.3f > %.2f) — "
                                     "log_only, message still accepted",
                                     self_id_, j, R, resid_gate_);
            } else {
                transport_->block_peer(j, "gate2 odom residual");
            }
        }
    }

    // BELIEF ACCUMULATOR (task 6) — the observation-sourced replacement for gate2's single-shot
    // residual, armed by obs_gate2=true. Same reason gate2 does not compare against a peer's own
    // /controller/odom here: that channel is authored by the same suspect, so a peer that forges
    // both agrees with itself and no residual ever appears. This differences the RAW claim
    // (agent_->peer_xnow(), never the conservative-anchoring-corrected position — see
    // test_belief_residual_ignores_the_anchoring_offset) against peer_truth_, the stand-in sensor
    // channel a compromised planning/comms layer cannot author.
    //
    // claims_fresh: whether agent_->peer_xnow() was actually refreshed THIS slot, i.e. whether
    // this agent's own AgentState barrier completed this cycle. The caller passes
    // res.peer_xnow_fresh directly (AgentCore reports it -- see StepResult::peer_xnow_fresh and
    // agent_core.cpp for exactly which return path sets it). This is deliberately NOT
    // res.n_timeouts==0: an EdgeXi/EdgeZ timeout deep in the ADMM inner loop increments the SAME
    // counter via a catch that just breaks that loop, and still returns hold=false with
    // peer_xnow_ genuinely fresh (it was assigned once, earlier, right after the AgentState
    // barrier) -- n_timeouts>0 there would wrongly zero out evidence on an otherwise-solved
    // cycle. It is also NOT res.hold: hold is equally true on the fleet-reset-sync HOLD, where
    // peer_xnow_ IS fresh (gate2 was moved onto HOLD specifically so a liar that jams the barrier
    // by spamming reset is still judged there -- gating belief off on every hold would silently
    // reopen that exact deadlock for arm A2). transport_->last_seen()[j] cannot substitute for
    // any of this either: it advances on every message this process commits regardless of
    // whether OUR OWN barrier for the current slot ever completed, so a peer talking exactly on
    // time can still show a frozen, stale claim here — differencing that against a live
    // observation fabricates up to 0.11 m of residual against a 0.15 m decision boundary.
    void beliefStep(std::uint64_t slot, bool claims_fresh, const Eigen::Vector2d& self_p) {
        if (!evict_armed_) return;
        std::map<int, std::deque<admm::ObsSample>> truth;
        { std::lock_guard<std::mutex> l(mu_); truth = peer_truth_; }
        const auto seen = transport_->last_seen();
        // Task 11 item 1: this cycle's unthrottled telemetry (CycleStats.tel_*), refilled from
        // scratch every call -- cycle()-thread only, same discipline as ev_out_peer_/ev_out_pos_
        // just below, so no lock needed.
        tel_peer_.clear(); tel_resid_.clear(); tel_l_act_.clear(); tel_l_rep_.clear();
        tel_l_total_.clear(); tel_abstain_.clear();

        // SECOND-HAND EVIDENCE, PART 1: fold in what other peers reported THIS slot (task 7
        // items 4-6, spec 5.2/5.3), run BEFORE the first-hand loop below for two reasons:
        //  - review finding 3: a peer is both a first-hand TARGET (below) and, in the same slot,
        //    potentially a second-hand REPORTER whose own credibility just moved. Accumulating
        //    the reporter's contribution here into `smear_contrib` and feeding it to the SAME
        //    trust_step_observed call below means l_rep_[reporter] is decayed
        //    by lambda exactly once this slot, not twice.
        //  - spec 5.3: w_i (how much we weight a relayer) must be "the PREVIOUS slot's" belief in
        //    that relayer, so trust_step_relay below must read l_rep_[i] before this slot's
        //    first-hand update touches it -- which running this section first guarantees for free.
        // `touched` records every (target, source) pair trust_step_relay actually updated this
        // slot, so the decay pass in part 2 does not decay-then-add the same pair twice.
        std::map<int, double> smear_contrib;      // reporter id -> combined llr about THAT reporter
        std::set<std::pair<int, int>> touched;    // (target, source) pairs updated this slot
        if (claims_fresh) {   // peer_evidence() is frozen on a barrier-miss HOLD -- same "no fresh
                               // data, no evidence" contract as the first-hand loop below; without
                               // this gate a HOLD would replay the SAME frozen report every silent
                               // slot as if it were new (review finding 2's sibling problem).
            const double margin = admm::evidence_margin(trust_.sigma);
            for (const auto& kv : agent_->peer_evidence()) {
                const int i = kv.first;
                if (i == self_id_ || transport_->blocked(i)) continue;   // a fenced peer's word is nothing
                const auto& ev = kv.second;                              // {peer, pos, slot} as received
                const auto xi = agent_->peer_xnow().find(i);
                if (xi == agent_->peer_xnow().end()) continue;            // i itself is gone (evicted)
                double contrib = 0.0;
                for (std::size_t k = 0; k < ev.peer.size(); ++k) {
                    const int j = ev.peer[k];
                    if (j == self_id_ || j == i) continue;
                    const Eigen::Vector2d z(ev.pos[2 * k], ev.pos[2 * k + 1]);
                    if (!z.allFinite()) continue;
                    // Review round 2, I2': the refute_ratio calibration DENOMINATOR -- every
                    // entry that reaches the plausibility test, refuted or not. Incremented
                    // before the call so it counts attempts, not just the ones that passed.
                    ++n_evidence_checked_this_cycle_;
                    // ITEM 6: geometry i itself is bound by, using i's OWN CLAIMED position as
                    // its vantage point -- we do not need to have observed i to run this check,
                    // which is the whole point of broadcasting positions instead of residuals.
                    // Refuted -> the report cannot be about anything real; nothing further is
                    // extracted from it. Lenient occlusion radius = CBF radius (0.30) minus
                    // margin -- narrower than the physical pillar (0.20) whenever margin > 0.10,
                    // so review finding 6's arena/margin numbers are worth re-checking if either
                    // constant is retuned.
                    if (!admm::evidence_plausible(xi->second.head<2>(), z, arena_obs_, obs_range_, margin)) {
                        contrib += admm::refuted_llr(trust_);
                        // Task 11 item 4: refute_ratio calibration input. With no attacker
                        // present every count here is a SPURIOUS refutation (a grazing sightline,
                        // noise, timing skew) against the honest reporter i -- see
                        // docs/superpowers/results/2026-07-31-trust-calibration.md.
                        ++n_refuted_this_cycle_;
                        continue;
                    }
                    // ITEM 5: the smear check. If THIS agent independently observed j at the
                    // exact slot the report is about, compare directly -- j's true position at
                    // that instant is exactly what was just measured, so a disagreement is
                    // evidence the REPORTER fabricated its sighting, not evidence against j.
                    // Coverage of this check is counted (review finding 4): if the buffer never
                    // actually covers ev.slot, `have_check` is always false and a report that
                    // could not be checked would look identical to one that checked out clean.
                    bool have_check = false;
                    Eigen::Vector2d self_obs_j = Eigen::Vector2d::Zero();
                    ++smear_checks_total_;
                    if (const auto tb = truth.find(j); tb != truth.end()) {
                        if (const auto obs = admm::interp_at(tb->second, static_cast<double>(ev.slot) * ts_)) {
                            if (admm::visible(self_p, *obs, arena_obs_, obs_range_)) {
                                self_obs_j = *obs;
                                have_check = true;
                            }
                        }
                    }
                    if (have_check) ++smear_checks_hit_;
                    contrib += admm::smear_llr(have_check, z, self_obs_j, trust_);
                    if (transport_->blocked(j)) continue;   // j already fenced; nothing left to decide
                    const auto cj = agent_->peer_xnow().find(j);
                    if (cj == agent_->peer_xnow().end()) continue;
                    // Hearsay about j itself: corroborated against j's OWN claim, weighted and
                    // capped by how much WE trust i as of LAST slot (trust_step_relay's own
                    // invariant; l_rep_[i] has not been touched yet this slot -- see above).
                    const double r = (z - cj->second.head<2>()).norm();
                    double& C = l_relay_[j][i];
                    C = admm::trust_step_relay(C, admm::trust_llr(r, trust_.sigma, trust_.d_lie,
                                                                  trust_.clamp_step, trust_.credit_ratio),
                                               l_rep_[i], trust_);
                    touched.insert({j, i});
                }
                if (contrib != 0.0)
                    smear_contrib[i] = std::clamp(contrib, -trust_.clamp_step, trust_.clamp_step);
            }
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[agent%d] smear-check coverage: %lu/%lu hit (review finding 4 telemetry)",
                                 self_id_, smear_checks_hit_, smear_checks_total_);
        }
        // SECOND-HAND EVIDENCE, PART 2: decay every relay entry NOT freshly updated above, once
        // per slot -- the same "decay always, evidence only when fresh" discipline l_act_
        // follows below. A blocked reporter's entries are ERASED outright (review finding 2):
        // once fenced, its word about anyone must stop existing, not decay toward neutral over
        // ~2s while still counting toward a target's total -- the same shape of bug as the
        // stale-vote deadlock already on record for this project. Without this, an attacker could
        // earn trust, accuse j for a few slots, go silent about j, and the frozen accusation would
        // keep counting against j for the rest of the mission, surviving the attacker's own
        // eviction.
        for (auto& target_kv : l_relay_) {
            const int j = target_kv.first;
            auto& sources = target_kv.second;
            for (auto it = sources.begin(); it != sources.end();) {
                const int i = it->first;
                if (transport_->blocked(i)) { it = sources.erase(it); continue; }
                if (touched.count({j, i})) { ++it; continue; }
                it->second = admm::trust_step_relay(it->second, 0.0, l_rep_[i], trust_);
                ++it;
            }
        }

        // FIRST-HAND EVIDENCE: gating unchanged, but the llr this slot COMBINES with any
        // second-hand smear_contrib computed above for this same peer as a reporter -- exactly
        // once per peer, exactly once per slot (review finding 3).
        // Task 11 item 1 (review round 2 split occluded/out_of_range into their own codes): the
        // reasons a row can carry no evidence -- see CycleStats.msg's tel_abstain comment, which
        // this numbering must match.
        constexpr std::int32_t kAbstainNone = 0, kAbstainPeerBlocked = 1, kAbstainNoFreshClaim = 2,
                               kAbstainNoObsBuffer = 3, kAbstainOutOfRange = 4, kAbstainOccluded = 5;
        ev_out_peer_.clear();
        ev_out_pos_.clear();
        for (const auto& kv : agent_->peer_xnow()) {
            const int j = kv.first;
            if (j == self_id_) continue;
            const bool blocked = transport_->blocked(j);
            bool fresh = false;
            bool vis = false;
            Eigen::Vector2d observed = Eigen::Vector2d::Zero();
            // Narrowed below as the control flow rules cases out, one assignment per branch --
            // the same nesting as before item 1, so behaviour (fresh/vis/observed/ev_out_*) is
            // byte-for-byte unchanged; only this bookkeeping is new.
            std::int32_t abstain = kAbstainPeerBlocked;
            if (!blocked && claims_fresh) {
                abstain = kAbstainNoObsBuffer;
                const auto sit = seen.find(j);
                const auto tb = truth.find(j);
                if (sit != seen.end() && tb != truth.end()) {
                    // Align to the SLOT THE CLAIM IS FROM, not to "now" -- the same reasoning
                    // updatePeerOffsets and gate2 already use: comparing two different instants
                    // manufactures a discrepancy nobody's claim produced.
                    if (const auto obs =
                            admm::interp_at(tb->second, static_cast<double>(sit->second) * ts_)) {
                        // TASK 7 ITEM 1: visibility on the RAW ground-truth position, before
                        // noise -- occlusion/range are properties of where the body actually is.
                        // Not visible -> no evidence at all, and nothing to share either: the
                        // attacker hiding behind a pillar must not appear in ev_peer/ev_pos.
                        const auto seen_now = observedPeer(slot, j, self_p, *obs);
                        vis = seen_now.has_value();
                        // Review round 2: admm::visible() is both a range gate and an occlusion
                        // test, checked in that order (range first) -- mirrored here rather than
                        // plumbing a reason code through trust.hpp, since this is telemetry only
                        // and visible() itself must stay a single yes/no safety predicate. NaN
                        // (non-finite obs/self_p) falls into out_of_range here too, matching
                        // visible()'s own earliest-exit priority.
                        // Only the synthetic model can name WHY it abstained, because only it
                        // knows the geometry it used. A real sensor that returned nothing has
                        // not told us whether the peer was occluded or out of range, and
                        // kAbstainNoObsBuffer (already set above) says exactly that much.
                        if (synthetic_sensor_)
                            abstain = ((*obs - self_p).norm() <= obs_range_) ? kAbstainOccluded
                                                                            : kAbstainOutOfRange;
                        if (vis) {
                            observed = *seen_now;
                            fresh = true;
                            abstain = kAbstainNone;
                            ev_out_peer_.push_back(j);
                            ev_out_pos_.push_back(observed[0]);
                            ev_out_pos_.push_back(observed[1]);
                        }
                    }
                }
            } else if (!blocked) {
                abstain = kAbstainNoFreshClaim;
            }
            // TWO LEDGERS (see l_act_/l_rep_). Each takes exactly one kind of evidence, and the
            // separation is the whole fix: summing them let a peer's honest self-report buy off
            // its own smearing, because both landed on the same scalar.
            //
            // ACTOR: the first-hand position residual, and nothing else. extra_llr is 0 here --
            // the smear no longer touches what decides eviction.
            double& L = l_act_[j];
            L = admm::trust_step_observed(L, blocked, fresh, vis, observed, kv.second.head<2>(),
                                          0.0, trust_);
            // REPORTER: the smear check, and nothing else. Passed as extra_llr with the position
            // pair suppressed (claim_fresh=false), so trust_step_observed contributes only the
            // smear term while still applying the same decay discipline -- rather than a second
            // hand-rolled accumulator that could drift from trust_step_self's semantics.
            //
            // A peer that reported nothing checkable this slot has no entry in smear_contrib and
            // therefore takes a pure decay step, which is correct: an unverifiable report is not
            // evidence of dishonesty, and silence is not evidence of either. It also means a peer
            // that has never relayed anything sits at 0, where clamp(l_rep, 0, l_max) already
            // gives its testimony zero weight -- Zikratov's "first time ... minimum reputation",
            // and identical to the value l_self_ used to hold at start-up.
            const auto sc = smear_contrib.find(j);
            double& R = l_rep_[j];
            R = admm::trust_step_observed(R, blocked, /*claim_fresh=*/false, vis, observed,
                                          kv.second.head<2>(),
                                          sc != smear_contrib.end() ? sc->second : 0.0, trust_);
            // Task 11 item 1: recorded for EVERY peer, including a blocked one (trust_total is
            // pure and cheap -- the live decision path below still skips it there via `continue`,
            // this is telemetry only). r is NaN whenever abstain != kAbstainNone: the residual
            // literally doesn't exist for those rows, and a 0.0 would misrepresent "no evidence"
            // as "perfect agreement" to whoever calibrates obs_sigma off this column.
            const double total = admm::trust_total(L, l_relay_[j], trust_);
            tel_peer_.push_back(j);
            tel_resid_.push_back(abstain == kAbstainNone
                                      ? (observed - kv.second.head<2>()).norm()
                                      : std::numeric_limits<double>::quiet_NaN());
            tel_l_act_.push_back(L);
            tel_l_rep_.push_back(R);
            tel_l_total_.push_back(total);
            tel_abstain_.push_back(abstain);
            if (blocked) continue;   // already latched off; nothing left to decide
            // Logged unconditionally, same discipline as gate2's residual line: this IS the
            // calibration procedure for trust_.sigma / trust_.d_lie.
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[agent%d] belief robot%d L_self=%.3f total=%.3f fresh=%d vis=%d evict<%.2f",
                                 self_id_, j, L, total, int(fresh), int(vis), trust_.l_evict);
            if (!admm::trust_fences_peer(total, trust_)) continue;
            if (log_only_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "[agent%d] BELIEF would reject robot%d (L=%.3f < %.2f) — "
                                     "log_only, message still accepted",
                                     self_id_, j, total, trust_.l_evict);
            } else {
                transport_->block_peer(j, "belief threshold");
                // Tag WHY this peer is blocked so maybeEvict can give it the shorter, belief-
                // only patience (finding 2) without that patience leaking to a gate2 or
                // roster-exclusion block, which share the same transport-level blocked_ set but
                // never earned the accumulator's multi-slot debounce.
                belief_blocked_.insert(j);
            }
        }
        ev_out_slot_ = slot;   // stamp for the NEXT broadcast (agent_->set_evidence in cycle())
    }

    // Single dispatch point for both of gate2()'s call sites (HOLD path and SOLVED path), so the
    // selector lives in exactly one place instead of being duplicated at each call site. See the
    // constructor's startup log for which detector a given run actually armed.
    void detect(std::uint64_t slot, bool claims_fresh, const Eigen::Vector2d& self_p) {
        if (obs_gate2_) beliefStep(slot, claims_fresh, self_p);
        else gate2(slot);
    }

    void maybeEvict(std::uint64_t slot) {
        if (evict_after_ <= 0 || dogs_.size() < 2) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[agent%d] evict OFF: evict_after=%d dogs=%zu",
                                 self_id_, evict_after_, dogs_.size());
            return;
        }
        // Bring-up grace: don't evict anyone until this agent has completed one good cycle, so a
        // slow discovery at startup is never mistaken for a death. Armed in cycle() on the SOLVED
        // path and latched there — never re-derived here from has_prev(), which a dying peer
        // clears and a missing peer prevents from ever returning (see cycle()).
        const auto seen = transport_->last_seen();
        if (!evict_armed_) {
            std::string s;
            for (const auto& kv : seen)
                s += " " + std::to_string(kv.first) + ":" + std::to_string(kv.second);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[agent%d] evict NOT ARMED (no solved cycle yet) at slot %lu; last_seen%s",
                                 self_id_, static_cast<unsigned long>(slot),
                                 s.empty() ? " (none)" : s.c_str());
            return;
        }
        std::vector<int> dead;
        std::map<int, std::uint64_t> silent;  // per-dead-peer missing-slot count (log)
        for (int j : dogs_) {
            if (j == self_id_) continue;
            const auto it = seen.find(j);
            const std::uint64_t last = (it == seen.end()) ? 0 : it->second;
            // One line, one code path: a peer we have caught lying is silent for a different
            // REASON (we are refusing its messages), and that reason is positive evidence, so
            // it does not earn the patience that a possible network hiccup does.
            //
            // The test is Gate 2's LATCHED verdict, NOT the dropped_ counter. dropped_ also
            // counts Gate 1 rejections, and Gate 1 legitimately rejects a single NaN broadcast
            // from a DYING peer — the crash path, which has its own 10-slot patience. Keying
            // off the counter made one such rejection permanently mark a healthy peer as a
            // liar: measured 2026-07-28 (d_0728_072027), agent2 and agent3 each rejected one
            // NaN message from the other, then evicted each other at the 3-slot threshold,
            // ended up alone with no pairwise CBF at all, and closed to 0.838 m.
            //
            // Three patiences, not two (finding 2): a belief conviction gets the shortest one
            // (belief_blocked_ names it -- the accumulator already debounced it internally);
            // gate2 and roster-exclusion blocks share evict_after_lying_ (both latch on their
            // FIRST bad signal and never earned the belief's multi-slot debounce); anything not
            // blocked at all is ordinary silence.
            const int need = admm::evict_patience(belief_blocked_.count(j) != 0,
                                                  transport_->blocked(j), evict_after_belief_lying_,
                                                  evict_after_lying_, evict_after_);
            if (slot > last && slot - last >= static_cast<std::uint64_t>(need)) {
                dead.push_back(j);
                silent[j] = slot - last;
            }
        }
        if (dead.empty()) {  // holding, but nobody is silent long enough yet — show the counters
            std::string s;
            for (int j : dogs_) {
                if (j == self_id_) continue;
                const auto it = seen.find(j);
                s += " " + std::to_string(j) + ":" +
                     (it == seen.end() ? std::string("never")
                                       : std::to_string(slot > it->second ? slot - it->second : 0));
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[agent%d] HOLD at slot %lu, nobody dead yet (need %d); silent%s",
                                 self_id_, static_cast<unsigned long>(slot), evict_after_, s.c_str());
            return;
        }
        // corpse -> static CBF obstacle. NOT at the kill-time position: the dead dog's
        // lower layer keeps executing its last published trajectory and walks up to ~1 m
        // further before parking (verified live 2026-07-23: ghost at 3.48, body at 4.63,
        // survivor faithfully dodged the ghost and hit the body). Its last broadcast xibar
        // TERMINAL knot predicts where it actually stops — local information the wire
        // already carries — plus an inflated radius for the residual tracking error.
        // COPIES, not references: agent_ is replaced below and these must outlive the old
        // core (a dangling reference here segfaulted both survivors, 2026-07-23 08:36)
        const auto pxnow = agent_->peer_xnow();
        const auto pxibar = agent_->peer_xibar();
        for (int j : dead) {
            const auto itn = pxnow.find(j);
            const auto itb = pxibar.find(j);
            if (itn == pxnow.end()) {
                RCLCPP_WARN(get_logger(), "[agent%d] EVICT robot%d (never seen a state)", self_id_, j);
                continue;
            }
            // Talking, but talking garbage -> it is ALIVE and walking, not a corpse. Its
            // keep-out has to be wider (it is cooperating with nobody) and, below, it has to
            // follow its odom instead of standing where it was last believed to be.
            // Gate 2's latched verdict, for the same reason as `need` above: a peer that emitted
            // one NaN is dying, not lying, and a dying dog's body does stop.
            const bool mobile = transport_->blocked(j);
            // Hostile mode: evict (graph rebuild below, keyed on `dead`) but place no keep-out —
            // structurally the same skip as the !kp path underneath.
            if (inj_ignore_corpses_) {
                RCLCPP_WARN(get_logger(),
                            "[agent%d] EVICT robot%d — keep-out SUPPRESSED (inject_ignore_corpses)",
                            self_id_, j);
                continue;
            }
            // Geometry lives in admm::corpse_keepout (fleet_config) so it is unit-testable:
            // where to fence, how wide, and when NOT to fence at all. See test_corpse_keepout.
            const auto kp = admm::corpse_keepout(
                itn->second, itb != pxibar.end() ? itb->second : Eigen::VectorXd(), robot_margin_,
                mobile, corpse_anchor_knot_);
            if (!kp) {
                RCLCPP_WARN(get_logger(),
                            "[agent%d] EVICT robot%d — no finite position (plan and state both "
                            "non-finite); NO keep-out placed",
                            self_id_, j);
                continue;
            }
            admm::Obstacle o;
            // corpse_keepout anchors on the peer's xibar terminal knot, which for a peer that is
            // still walking and no longer trusted would let the attacker choose where its own
            // keep-out gets drawn — that is the attack, not the defence. corpseAnchor picks the
            // channel that matches WHY we stopped believing it (odom if caught lying, its own
            // claim if it merely announced it left); kp->pos remains the fallback for a corpse
            // that is genuinely silent.
            Eigen::Vector2d anchor;
            o.pos = (mobile && corpseAnchor(j, &anchor)) ? anchor : kp->pos;
            o.radius = kp->radius;
            // The one obstacle kind that can be born already violated: it materialises around
            // wherever the survivors happen to be standing, and the dead peer's body kept
            // walking toward them during the silence window while their pairwise constraint was
            // running on frozen data. Hard at k=0 would mean "infeasible, forever". See Obstacle.
            o.soft_k0 = true;
            corpses_[j].push_back(o);
            if (mobile) { mobile_corpse_.insert(j); astar_corpse_[j] = o.pos; }
            RCLCPP_WARN(get_logger(),
                        "[agent%d] EVICT robot%d (silent %lu slots) — corpse CBF at predicted rest (%.2f,%.2f) r=%.2f (last seen (%.2f,%.2f))",
                        self_id_, j, static_cast<unsigned long>(silent[j]), o.pos[0], o.pos[1],
                        o.radius, itn->second[0], itn->second[1]);
        }
        std::vector<int> survivors;
        for (int j : dogs_)
            if (std::find(dead.begin(), dead.end(), j) == dead.end()) survivors.push_back(j);
        rebuild(survivors, pxnow);
    }

    // Rebuild everything that depends on the member list: the fleet graph, the shape cost,
    // the node QP's obstacle set (which fixes the QP dimensions, hence a fresh AgentCore) and
    // the local A*. Eviction and rejoin both route through here, and because it constructs a
    // NEW AgentCore it also zeroes the consensus duals — a dead peer's stale lambda would
    // otherwise keep applying a constraint force from a robot that no longer exists.
    // Worker-thread only (same thread as cycle()), so no locking.
    // pxnow is a VALUE COPY of the last peer states: agent_ is replaced below, so a reference
    // into the old core would dangle (that segfaulted both survivors, 2026-07-23 08:36).
    void rebuild(std::vector<int> new_dogs, const std::map<int, Eigen::Vector4d>& pxnow) {
        dogs_ = std::move(new_dogs);
        std::sort(dogs_.begin(), dogs_.end());  // complete_graph edge order must match bring-up
        edges_ = complete_graph(dogs_);

        // Shape must agree with what the coordinator lays out (shape_for is the shared rule).
        // set_formation("") is a silent no-op leaving the OLD offsets in place, so the size
        // check below — not a null check — is what actually catches "no shape for this n".
        formation_->set_formation(admm::shape_for(dogs_.size(), roster_n_, formation_name_));
        const auto* off = formation_->current_offsets();
        const bool has_shape = (off != nullptr && off->size() == dogs_.size());

        const auto obs = allObstacles();
        agent_ = std::make_unique<admm::AgentCore>(
            self_id_, dogs_, edges_, has_shape ? formation_.get() : nullptr, w_form_, obs,
            arena_walls_, hard_through_, transport_.get(), robot_margin_, enable_peer_keepout_);

        // The stack's obstacle contract is CBF + A* detour: a straight REFERENCE through a
        // keep-out linearizes into an l>u bound box -> the (now guarded) QP fails loudly and
        // the dog freezes (verified live 08:31). So corpses go to the local A* too.
        // Swap the circles IN PLACE when a planner already exists: rebuilding it would replace
        // the arena-sized bounds from launch (x[-2,20] y[-7,7]) with a box around the current
        // fleet, and a later goal outside that box then yields no path -> straight line through
        // a keep-out -> frozen dog. Only the no-arena case (empty world) has to construct one.
        std::vector<admm::AStarCircle> circles;
        for (const auto& ob : obs) circles.push_back({ob.pos[0], ob.pos[1], ob.radius});
        // circles == arena_obs_ + corpses, so empty means "nothing to route around at all"
        // (empty world, last corpse just dropped) -> go back to the straight-line reference.
        if (circles.empty()) planner_.reset();
        else if (planner_) planner_->set_circles(circles);
        else planner_ = makePlannerFromFleet(circles, pxnow);
        path_goal_ = Eigen::Vector2d(1e9, 1e9);  // force replan against the new obstacle set
        RCLCPP_WARN(get_logger(),
                    "[agent%d] REBUILD -> %zu dog(s), %zu edge(s), shape=%s, %zu obstacle(s) — cold-starting consensus",
                    self_id_, dogs_.size(), edges_.size(),
                    has_shape ? admm::shape_for(dogs_.size(), roster_n_, formation_name_).c_str() : "none",
                    circles.size());
    }

    // Fallback planner for a fleet running WITHOUT an arena (no static obstacles -> no planner
    // was built at bring-up). Box = fleet bounding box + 12 m, generous enough for the goals a
    // demo hands out; an arena run never takes this path.
    std::unique_ptr<admm::AStarPlanner> makePlannerFromFleet(
        const std::vector<admm::AStarCircle>& circles,
        const std::map<int, Eigen::Vector4d>& pxnow) const {
        double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
        for (const auto& pk : pxnow) {
            xmin = std::min(xmin, pk.second[0]); xmax = std::max(xmax, pk.second[0]);
            ymin = std::min(ymin, pk.second[1]); ymax = std::max(ymax, pk.second[1]);
        }
        if (xmin > xmax) { xmin = xmax = ymin = ymax = 0.0; }  // no peer states yet
        std::vector<admm::AStarRect> rects;
        for (const auto& r : arena_rects_)
            rects.push_back({r.center[0], r.center[1], r.size[0], r.size[1], robot_margin_});
        return std::make_unique<admm::AStarPlanner>(0.15, robot_margin_, circles, xmin - 12.0,
                                                    xmax + 12.0, ymin - 12.0, ymax + 12.0,
                                                    /*boundary_margin=*/0.45, rects);
    }

    void publishStats(std::uint64_t slot, const admm::StepResult& res, double t_cycle_wall) {
        admm_fleet_msgs::msg::CycleStats m;
        m.cycle_id = slot;
        m.robot_id = self_id_;
        m.achieved_rounds = res.achieved_rounds;
        m.n_timeouts = res.n_timeouts;
        m.r_prim_edge = res.r_prim_edge;
        m.r_prim_hist = res.hist.r_prim;  // per-round residual -> convergence-round analysis (G5)
        const auto w = transport_->take_wait_times();  // G5 comm-latency: per-cycle recv wait (s)
        m.t_wait_state = w.state;
        m.t_wait_xi = w.xi;
        m.t_wait_z = w.z;
        m.t_cycle_wall = t_cycle_wall;
        m.t_node = res.t_node;
        m.t_edge_solve = res.t_edge_solve;
        const auto b = transport_->take_bytes();  // G5 comm-volume: payload bytes this cycle
        m.bytes_tx = b.tx;
        m.bytes_rx = b.rx;
        m.t_rx_mean = transport_->take_rx_mean();  // G5 one-way latency (mean rx-tx this cycle)
        m.n_stale = transport_->take_stale();
        // Task 10 item 5: observation-channel samples the publisher-GID pin rejected THIS cycle
        // (any peer). Nonzero means a peer's ground-truth channel went dark to us -- late-
        // impostor attack OR a benign publisher restart, operationally indistinguishable -- and
        // the harness must be able to assert on it instead of trusting a run whose channel died
        // silently. See peer_truth_subs_'s subscription callback.
        m.n_obs_dropped = static_cast<int32_t>(obs_dropped_this_cycle_.exchange(0));
        // Task 11 item 1: this cycle's unthrottled belief telemetry (see beliefStep and
        // CycleStats.msg's tel_* comment). Copied, not moved: beliefStep clears and refills these
        // itself at the top of its NEXT call, so an empty-until-refilled window doesn't matter,
        // but a move would leave them in an unspecified state if publishStats ever ran twice
        // between beliefStep calls (it doesn't today, but a copy costs nothing at fleet scale).
        m.tel_peer = tel_peer_;
        m.tel_resid = tel_resid_;
        m.tel_l_act = tel_l_act_;
        m.tel_l_rep = tel_l_rep_;
        m.tel_l_total = tel_l_total_;
        m.tel_abstain = tel_abstain_;
        // Task 11 item 4: same read-and-reset pattern as n_obs_dropped just above.
        m.n_refuted = n_refuted_this_cycle_;
        n_refuted_this_cycle_ = 0;
        // Review round 2, I2': the refute_ratio denominator, same pattern.
        m.n_evidence_checked = n_evidence_checked_this_cycle_;
        n_evidence_checked_this_cycle_ = 0;
        m.hold = res.hold;
        // A finite-guard (NaN xi) cycle returns hold=false but publishes NO target and
        // announces a fleet cold-start next slot; mark it so G5 doesn't read it as a normal cycle.
        m.reset = !res.hold && !res.xi.allFinite();
        stats_pub_->publish(m);
    }

    int self_id_;
    std::vector<int> dogs_;
    std::vector<admm::EdgeKey> edges_;
    std::size_t roster_n_ = 0;                      // launch-time fleet size (dogs_ shrinks on evict)
    std::string formation_name_;                    // configured full-fleet shape (`formation` param)
    double v_, ts_, w_form_, com_height_, r_latch_, latch_margin_;
    double follow_gain_, follow_desired_, follow_range_, follow_floor_, follow_cone_cos_;
    double fp_half_len_, fp_half_wid_, fp_drift_;   // Vision60 body footprint (direction-aware safety)
    int k_send_;
    double inj_drop_p_ = 0.0, inj_delay_ms_ = 0.0, inj_jitter_ms_ = 0.0;  // C-experiment injection
    bool inj_ignore_corpses_ = false;  // hostile mode: evict without fencing (demo/threat model)
    // Task 10 item 2: node-side mirror of DdsTransport's copy, for the WARN log line only (the
    // transport's own atomic is what send_state actually reads). Param-callback-thread only,
    // same non-atomic pattern as inj_drop_p_ above.
    int inj_fake_evidence_target_ = 0;
    bool evade_boost_ = true;          // ask for MAX_VX while evading a keep-out (followSpeed)
    double evade_range_ = 0.5;
    bool enable_peer_keepout_ = false;  // task 4b spike: local pairwise safety net, off by default
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    std::unique_ptr<admm::LaplacianFormation> formation_;
    std::unique_ptr<admm::MotionAdapter> adapter_;
    std::unique_ptr<admm::DdsTransport> transport_;
    std::unique_ptr<admm::AgentCore> agent_;
    std::unique_ptr<admm::AStarPlanner> planner_;   // null unless use_astar && arena has obstacles
    std::vector<admm::Obstacle> arena_obs_;         // STATIC terrain only (arena circles)
    // Evicted peers' keep-outs, keyed by robot id so a returning peer's circle can be
    // dropped again (rejoin). Kept OUT of arena_obs_ precisely so removal is possible —
    // the terrain set must stay pristine across membership changes.
    std::map<int, std::vector<admm::Obstacle>> corpses_;
    std::set<int> mobile_corpse_;                   // evicted for lying -> still walking
    std::map<int, Eigen::Vector2d> astar_corpse_;   // where A* last placed each mobile one
    std::vector<admm::Wall> arena_walls_;
    int hard_through_ = 1;
    double robot_margin_ = 0.60;
    int evict_after_ = 10;                          // slots of peer silence before eviction (0=off)
    int evict_after_lying_ = 3;                     // ...but a CAUGHT liar is positive evidence
    int evict_after_belief_lying_ = 1;              // ...and a BELIEF conviction already debounced itself
    std::set<int> belief_blocked_;                  // peers block_peer'd via beliefStep; worker-thread only
    int corpse_anchor_knot_ = admm::K_SEND;         // A/B knob; K_SEND is the safe setting
    // Gate 2 state. Budget for evict_after_lying_: the CBF has D_MIN 1.3 - contact 0.867 =
    // 0.433 m of slack, a hostile peer closes at MAX_VX 0.55 m/s, so the whole detect-and-evict
    // path has 0.79 s ~ 8 slots. The low-pass itself spends ~2 of them, leaving 6; 3 keeps a
    // factor of two. Worst danger window = 2 + 3 = 5 slots = 0.5 s = 0.28 m closed, inside 0.433.
    admm::GateLimits gate_;                         // map bounds + Gate 1 thresholds
    std::map<int, Eigen::Vector2d> peer_odom_;      // guarded by mu_ (written from cmd_cbg_)
    // OBSERVATION CHANNEL (spec 5.1): raw, un-noised, time-ordered samples per peer, guarded by
    // mu_ (written from cmd_cbg_, the subscription's callback group). Noise is added at the point
    // of use (updatePeerOffsets), not here, so this buffer stays a clean, re-queryable history.
    std::map<int, std::deque<admm::ObsSample>> peer_truth_;
    std::map<int, std::optional<rmw_gid_t>> obs_gid_;  // first writer GID/peer topic (latched); guarded by mu_
    double obs_window_s_ = 0.5;                      // seconds of history kept per peer (declared with the subscription)
    std::map<int, Eigen::Vector2d> peer_offset_prev_;  // conservative_offset rate-limit state; cycle()-thread only
    std::map<int, double> resid_;                   // low-passed |odom - claimed|, cycle() only
    double resid_alpha_ = 0.2, resid_gate_ = 0.5;
    bool log_only_ = true;                          // measure before you block; see calibration
    // Belief layer (trust.hpp). Every default is derived. obs_gate2_ selects gate2() vs
    // beliefStep() at every call site (see detect()); l_act_/l_rep_ are its per-peer state.
    admm::TrustParams trust_;
    bool obs_gate2_ = false;                        // false=gate2() (A1), true=beliefStep() (A2)
    // TWO LEDGERS, NOT ONE (2026-08-02). These used to be a single `l_self_` that carried three
    // roles at once: what a peer did, how good a witness it is, and how much its testimony counts.
    // A peer that lies only about OTHERS while reporting its own position honestly then became
    // unconvictable at any fleet size -- everyone's observation corroborates its own claim, so the
    // relay term is positive and offsets first-hand suspicion that has already saturated at the
    // floor. Measured: l_self floors at l_evict - clamp_step = -11.2, the relay sum caps at
    // +l_max = 4.6, and -11.2 + 4.6 = -6.6 never reaches l_evict = -9.2 (§8b.1 of the results;
    // three independent N=5 observers each measured exactly -6.600).
    //
    // Zikratov, Lebedev & Gurtov, "Trust and Reputation Mechanisms for Multi-agent Robotic
    // Systems" (LNCS 8638, pp. 106-120) -- this project's own reference for the trust layer --
    // keeps them apart by construction: Definition 1's TRUST is about the object of the vote,
    // Definition 2's REPUTATION is about the voter, and its conclusion asks an agent "not only to
    // execute functions for serving a target but also to give a correct feedback on the actions of
    // other robots". Two requirements, two variables.
    //
    // The split follows the harm, which is the reason it is safe to stop trying to convict a
    // smearer: lying about where YOU are shrinks the barrier and can put a body into another
    // robot, so it must be evictable. Lying about where SOMEONE ELSE is cannot -- the barrier uses
    // positions, and this peer's own position is honest -- so the proportionate answer is to stop
    // counting its testimony, not to remove a working robot and leave a keep-out disc behind.
    std::map<int, double> l_act_;   // ACTOR trust: first-hand position residual only. Drives
                                    // eviction via trust_total(l_act_[j], l_relay_[j]).
    std::map<int, double> l_rep_;   // REPORTER reputation: the smear check only. Weights and caps
                                    // this peer's relayed testimony; never evicts on its own.
    // l_relay_[j][i]: peer i's relayed contribution to OUR belief about peer j (trust_step_relay),
    // so "all hearsay about j" is the single map l_relay_[j] that trust_total consumes directly.
    std::map<int, std::map<int, double>> l_relay_;
    // Task 11 item 1: this cycle's unthrottled per-peer telemetry (CycleStats.tel_*), written
    // and cleared by beliefStep() only -- cycle()-thread only, same as ev_out_peer_/ev_out_pos_.
    // Stay empty for the whole run under obs_gate2=false (gate2() never touches them).
    std::vector<std::int32_t> tel_peer_;
    std::vector<double> tel_resid_, tel_l_act_, tel_l_rep_, tel_l_total_;
    std::vector<std::int32_t> tel_abstain_;
    // Task 11 item 4: relayed reports refuted as geometrically implausible THIS cycle (the
    // refute_ratio calibration input). Same thread as tel_*_ above -- no atomic needed, unlike
    // obs_dropped_this_cycle_ below, which is written from a subscription callback thread.
    int n_refuted_this_cycle_ = 0;
    // Review round 2, I2': the denominator for n_refuted_this_cycle_ above -- every
    // evidence_plausible() attempt this cycle, refuted or not.
    int n_evidence_checked_this_cycle_ = 0;
    int obs_noise_seed_ = 0;                        // must vary per run -- see admm::obs_noise
    // Task 7 item 1: sensing range visible() gates first-hand evidence on, and (widened) the
    // geometric refutation of relayed reports (item 6).
    double obs_range_ = 4.0;
    bool synthetic_sensor_ = true;   // apply range/occlusion/noise to the observation ourselves
    // Review finding 4 telemetry: how often the smear check (beliefStep) actually had a
    // buffered self-observation to compare a relayed report against, vs how often it merely
    // abstained -- cumulative, logged throttled, so the smear experiment arm can prove the
    // check ran instead of a silent 0/0.
    std::uint64_t smear_checks_total_ = 0, smear_checks_hit_ = 0;
    // Task 7 / spec 5.2: this agent's own evidence, produced by the LAST beliefStep call and
    // broadcast at the TOP of the NEXT step() (see cycle()'s set_evidence call and AgentState.msg
    // for why ev_slot must accompany it). cycle()-thread only, same as ev_out_slot_.
    std::vector<int> ev_out_peer_;
    std::vector<double> ev_out_pos_;
    std::uint64_t ev_out_slot_ = 0;
    // Task 10 item 1: dual-channel forgery (see gate2()/applyOdomFake and the on-set callback).
    // Atomic: read on the worker thread (gate2), written from the executor thread (param cb).
    std::atomic<double> inj_odom_fake_{0.0};
    std::atomic<int> inj_odom_fake_peer_{0};
    // Task 10 item 4 (inject_forged_obs): guarded by mu_ (armed from construction OR the
    // executor-thread param callback; read from the executor-thread odom_sub_ callback).
    int forged_obs_target_ = 0;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr forged_obs_pub_;
    // Task 10 item 5: observation-channel samples this cycle rejected by the publisher-GID pin
    // (peer_truth_subs_ callback, executor thread) -- read-and-reset once per cycle into
    // CycleStats::n_obs_dropped (see publishStats) so a channel going dark reaches the run's
    // verdict instead of only a throttled log line nothing downstream reads.
    std::atomic<std::uint32_t> obs_dropped_this_cycle_{0};
    double input_stale_s_ = 1.0;                    // own obs/odom age that self-fences us
    std::vector<admm::ArenaRect> arena_rects_;      // A*-only wall boxes (door)
    std::vector<Eigen::Vector2d> path_;             // cached A* route; cycle()-thread only
    Eigen::Vector2d path_goal_{1e9, 1e9};           // goal the cache was planned for (force 1st plan)

    rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr target_pub_;
    rclcpp::Publisher<admm_fleet_msgs::msg::CycleStats>::SharedPtr stats_pub_;
    rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obs_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> peer_odom_subs_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> peer_truth_subs_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<admm_fleet_msgs::msg::FleetPlan>::SharedPtr plan_sub_;  // gets this dog's goal
    // reentrant group for this agent's own command/state callbacks (obs/odom/goal/plan) so the
    // high-rate DdsTransport stream can't starve a low-rate goal/plan callback. (Fleet-wide slot
    // assignment now lives in the standalone fleet_coordinator_node, not here.)
    rclcpp::CallbackGroup::SharedPtr cmd_cbg_;

    std::mutex mu_;
    ocs2_msgs::msg::MpcObservation obs_;
    nav_msgs::msg::Odometry odom_;
    Eigen::Vector2d goal_{0, 0}, v_ema_{0, 0};
    bool has_obs_ = false, has_odom_ = false, has_goal_ = false, has_vema_ = false;
    rclcpp::Time t_obs_{0, 0, RCL_ROS_TIME}, t_odom_{0, 0, RCL_ROS_TIME};  // last arrival (freshness)
    bool evict_armed_ = false;   // latched bring-up grace; see maybeEvict
    // Callbacks (executor threads) clear this under mu_; the worker read-modify-writes it WITHOUT
    // mu_ -> atomic to avoid a torn bool. yaw_latch_val_ is worker-thread-only, needs no atomic.
    std::atomic<bool> yaw_latched_{false};
    double yaw_latch_val_ = 0.0;

    std::atomic<bool> run_{false};
    std::thread worker_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdmmAgentNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
