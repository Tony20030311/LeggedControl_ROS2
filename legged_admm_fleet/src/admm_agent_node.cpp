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
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <ocs2_core/misc/LoadData.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

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
        transport_->set_inject(inj_drop_p_, inj_delay_ms_, inj_jitter_ms_);
        param_cb_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& ps) {
                for (const auto& p : ps) {
                    if (p.get_name() == "inject_drop_p") inj_drop_p_ = p.as_double();
                    else if (p.get_name() == "inject_delay_ms") inj_delay_ms_ = p.as_double();
                    else if (p.get_name() == "inject_jitter_ms") inj_jitter_ms_ = p.as_double();
                }
                transport_->set_inject(inj_drop_p_, inj_delay_ms_, inj_jitter_ms_);
                rcl_interfaces::msg::SetParametersResult r;
                r.successful = true;
                return r;
            });
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
                                                   transport_.get(), robot_margin_);
        if (use_astar && !arena_obs_.empty()) {
            const double r_astar = declare_parameter<double>("astar_robot_radius", 0.60);
            const double res = declare_parameter<double>("astar_res", 0.15);
            const double ax_min = declare_parameter<double>("astar_x_min", 0.0);
            const double ax_max = declare_parameter<double>("astar_x_max", 10.0);
            const double ay_min = declare_parameter<double>("astar_y_min", -5.0);
            const double ay_max = declare_parameter<double>("astar_y_max", 5.0);
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
                if (!has_goal_) {  // stand in place until commanded
                    goal_ = Eigen::Vector2d(m->pose.pose.position.x, m->pose.pose.position.y);
                    has_goal_ = true;
                }
            }, cmd_opts);
        // Latch the per-dog goal (transient_local), same discovery-race reason as plan_sub_ below:
        // a short-lived `ros2 topic pub` goal burst can match one agent late and drop its goal for
        // that dog -> it parks at spawn while the others walk. Latching lets a late-matching agent
        // still receive the last goal. Publishers must offer transient_local too (arena/g2 scripts).
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/" + ns + "/goal", rclcpp::QoS(1).transient_local(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
                std::lock_guard<std::mutex> l(mu_);
                goal_ = Eigen::Vector2d(m->pose.position.x, m->pose.position.y);
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
                        goal_ = Eigen::Vector2d(m->goals[2 * k], m->goals[2 * k + 1]);
                        has_goal_ = true; yaw_latched_ = false;
                    }
            }, cmd_opts);

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
        return v_eff;
    }
    void cycle(std::uint64_t slot) {
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
        if (res.warm_cleared != admm::StepResult::kWarmKept)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[agent%d] warm start dropped (%s) — cold-starting consensus",
                                 self_id_,
                                 res.warm_cleared == admm::StepResult::kWarmNaN
                                     ? "non-finite xi" : "peer announced reset");
        if (res.hold || !res.xi.allFinite()) {
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

    // Experiment D: a peer whose AgentState has been absent for evict_after_ consecutive
    // slots is dead — remove it from the fleet graph and rebuild the AgentCore over the
    // survivors so consensus (and motion) continues without it. The corpse's last known
    // position becomes a static CBF obstacle (we have no perception; a fallen dog is
    // otherwise invisible). Formation term is dropped post-eviction: a 3-slot shape has
    // no defined meaning for 2 dogs — survivors keep chasing their latched goals with
    // pairwise CBF safety. Rejoin is NOT supported (a returning peer stays ignored).
    // Runs on the worker thread only (same thread as cycle()), so no locking needed.
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
            if (slot > last && slot - last >= static_cast<std::uint64_t>(evict_after_)) {
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
            // Geometry lives in admm::corpse_keepout (fleet_config) so it is unit-testable:
            // where to fence, how wide, and when NOT to fence at all. See test_corpse_keepout.
            const auto kp = admm::corpse_keepout(
                itn->second, itb != pxibar.end() ? itb->second : Eigen::VectorXd(), robot_margin_);
            if (!kp) {
                RCLCPP_WARN(get_logger(),
                            "[agent%d] EVICT robot%d — no finite position (plan and state both "
                            "non-finite); NO keep-out placed",
                            self_id_, j);
                continue;
            }
            admm::Obstacle o;
            o.pos = kp->pos;
            o.radius = kp->radius;
            // The one obstacle kind that can be born already violated: it materialises around
            // wherever the survivors happen to be standing, and the dead peer's body kept
            // walking toward them during the silence window while their pairwise constraint was
            // running on frozen data. Hard at k=0 would mean "infeasible, forever". See Obstacle.
            o.soft_k0 = true;
            corpses_[j].push_back(o);
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
            arena_walls_, hard_through_, transport_.get(), robot_margin_);

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
    std::vector<admm::Wall> arena_walls_;
    int hard_through_ = 1;
    double robot_margin_ = 0.60;
    int evict_after_ = 10;                          // slots of peer silence before eviction (0=off)
    double input_stale_s_ = 1.0;                    // own obs/odom age that self-fences us
    std::vector<admm::ArenaRect> arena_rects_;      // A*-only wall boxes (door)
    std::vector<Eigen::Vector2d> path_;             // cached A* route; cycle()-thread only
    Eigen::Vector2d path_goal_{1e9, 1e9};           // goal the cache was planned for (force 1st plan)

    rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr target_pub_;
    rclcpp::Publisher<admm_fleet_msgs::msg::CycleStats>::SharedPtr stats_pub_;
    rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obs_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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
