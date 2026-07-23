// fleet_centralized_node.cpp — rclcpp port of ocs2_fleet_publisher_node.cpp (ROS1).
// Centralized ADMM-CBF-DMPC fleet publisher: the debug mode, the thesis "centralized
// baseline", and the parity oracle for the distributed agents (P3/P4).
// Behaviour-equivalent to the ROS1 node; differences:
//   * robot namespaces parameterized (default robot1..robot3, Vision60 legged_stack)
//   * odometry from /{robot}/controller/odom (estimator ground_truth) instead of Gazebo GT
//   * com_height/default_joints loaded from OCS2 reference.info (A1 hardcode removed)
//   * killCppTargets() dropped — handled by base_target_command:=false at launch
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <ocs2_msgs/msg/mpc_state.hpp>
#include <ocs2_msgs/msg/mpc_input.hpp>
#include <ocs2_core/misc/LoadData.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "legged_upper_control/admm_constants.hpp"
#include "legged_upper_control/admm_reference.hpp"
#include "legged_upper_control/admm_motion_adapter.hpp"
#include "legged_upper_control/admm_formation.hpp"
#include "legged_upper_control/astar_planner.hpp"
#include "legged_upper_control/fleet_config.hpp"
#include "legged_upper_control/admm_coordinator.hpp"
#include "legged_upper_control/admm_node_qp.hpp"  // Obstacle, Wall
#include "legged_upper_control/fleet_logic.hpp"   // admm::toTargetMsg (shared 24-D target builder)

// OSQP's osqp.h (pulled in via admm_qp_common.hpp) re-#defines RHO as a macro after
// admm_constants.hpp #undef'd it; undo it here so admm::RHO resolves in this TU.
#ifdef RHO
#undef RHO
#endif

using admm::BASE_PX; using admm::BASE_PY; using admm::BASE_YAW;
using admm::MOM_LIN_X; using admm::MOM_LIN_Y;

namespace {
Eigen::MatrixX2d to_mat(const std::vector<Eigen::Vector2d>& v) {
  Eigen::MatrixX2d m(static_cast<int>(v.size()), 2);
  for (int k = 0; k < static_cast<int>(v.size()); ++k) { m(k, 0) = v[k][0]; m(k, 1) = v[k][1]; }
  return m;
}
double clip(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
}  // namespace

class FleetCentralizedNode : public rclcpp::Node {
 public:
  FleetCentralizedNode() : rclcpp::Node("fleet_centralized") {
    const auto ids = declare_parameter<std::vector<int64_t>>("robot_ids", {1, 2, 3});
    for (int64_t i : ids) dogs_.push_back(static_cast<int>(i));
    prefix_ = declare_parameter<std::string>("robot_prefix", "robot");
    for (size_t a = 0; a < dogs_.size(); ++a)
      for (size_t b = a + 1; b < dogs_.size(); ++b)
        edges_.push_back({dogs_[a], dogs_[b]});  // complete graph
    N_ = admm::N; ts_ = admm::TS;
    v_ = declare_parameter<double>("v", 0.15);
    formation_name_ = declare_parameter<std::string>("formation", "V");
    w_form_ = declare_parameter<double>("w_form", 0.3);  // loose default: single-file through tight gaps
    arena_name_ = declare_parameter<std::string>("arena", "");

    // Vision60 target geometry from OCS2 reference.info — never hardcoded (was A1-hardcoded)
    const auto reference_file = declare_parameter<std::string>("reference_file", "");
    if (reference_file.empty()) throw std::runtime_error("Missing required parameter: reference_file");
    Eigen::VectorXd default_joints(12);
    ocs2::loadData::loadCppDataType(reference_file, "comHeight", com_height_);
    ocs2::loadData::loadEigenMatrix(reference_file, "defaultJointState", default_joints);

    // arena selection: "" (or unknown) -> empty world (no obstacles, DEFAULT_GOALS)
    const auto& arenas = admm::arenas();
    auto ait = arenas.find(arena_name_);
    std::vector<admm::Obstacle> arena_obs;
    std::vector<admm::Wall> arena_walls;
    std::map<int, Eigen::Vector2d> arena_goals = admm::default_goals();
    if (ait != arenas.end()) {
      arena_obs = ait->second.obstacles;
      arena_walls = ait->second.walls;
      arena_rects_ = ait->second.rects;
      arena_goals = ait->second.goals;
    }
    for (int i : dogs_) {
      const Eigen::Vector2d ag = arena_goals.count(i) ? arena_goals[i] : Eigen::Vector2d(0, 0);
      double gx = declare_parameter<double>("goal" + std::to_string(i) + "_x", ag[0]);
      double gy = declare_parameter<double>("goal" + std::to_string(i) + "_y", ag[1]);
      goal_[i] = Eigen::Vector2d(gx, gy);
    }

    formation_.reset(new admm::LaplacianFormation(admm::formations()));
    formation_->set_formation(formation_name_);
    int hard_through = declare_parameter<int>("hard_through", 1);  // diag: # of hard edge-CBF steps (k=0..hard_through-1)
    double robot_margin = declare_parameter<double>("robot_margin", 0.60);  // Vision60 URDF FK: covers swing foot 0.542 / hind knee 0.609 (was 0.43 base-box); obstacle CBF inflation: r_eff = radius + this
    coord_.reset(new admm::ADMMCoordinator(admm::P_ITERS, admm::RHO, dogs_, edges_,
                                           formation_.get(), w_form_, arena_obs, arena_walls,
                                           hard_through, /*parallel=*/false, robot_margin));
    int lookahead = declare_parameter<int>("lookahead_m", 5);
    double pos_eps = declare_parameter<double>("pos_eps", 0.02);
    double yaw_rate_max = declare_parameter<double>("yaw_rate_max", 1.2);
    adapter_.reset(new admm::MotionAdapter(com_height_, default_joints, N_, ts_,
                                           /*v_freeze=*/0.05, /*ema_alpha=*/0.2, /*input_dim=*/24,
                                           /*yaw_mode=*/"path", lookahead, pos_eps,
                                           /*k_send=*/admm::K_SEND, /*has_max_yaw_rate=*/true,
                                           yaw_rate_max));

    r_latch_ = declare_parameter<double>("yaw_latch_r", 0.25);
    latch_margin_ = declare_parameter<double>("yaw_latch_margin", 0.10);
    send_cap_ = declare_parameter<int>("k_send", admm::K_SEND);
    d_safe_ = admm::D_MIN + declare_parameter<double>("send_margin", 0.0);

    // #1 leader-aware follower speed (ACC/car-following): a follower must not out-run a
    // slower dog ahead of it. follow_gain<=0 disables. desired 0.72 < nominal V gap 0.86
    // so it never brakes in same-speed formation; only bites when the dog ahead slows.
    follow_gain_ = declare_parameter<double>("follow_gain", 0.5);
    follow_desired_ = declare_parameter<double>("follow_desired", 0.72);
    follow_range_ = declare_parameter<double>("follow_range", 1.5);
    double cone_deg = declare_parameter<double>("follow_cone_deg", 60.0);
    follow_cone_cos_ = std::cos(cone_deg * M_PI / 180.0);
    // ACC cap floor: never brake below this. A PARKED dog ahead would otherwise pin the
    // cap at ~0 forever (passby deadlock) -- the floor keeps the reference advancing so
    // the CBF slides the dog around laterally, while still shedding 75% of approach speed.
    follow_floor_ = declare_parameter<double>("follow_floor", 0.05);

    use_astar_ = declare_parameter<bool>("use_astar", false);
    r_astar_ = declare_parameter<double>("astar_robot_radius", 0.60);
    astar_res_ = declare_parameter<double>("astar_res", 0.15);
    ax_min_ = declare_parameter<double>("astar_x_min", 0.0);
    ax_max_ = declare_parameter<double>("astar_x_max", 10.0);
    ay_min_ = declare_parameter<double>("astar_y_min", -5.0);
    ay_max_ = declare_parameter<double>("astar_y_max", 5.0);
    if (use_astar_ && !coord_->obstacles().empty()) {
      std::vector<admm::AStarCircle> circles;
      for (const auto& o : coord_->obstacles())
        circles.push_back({o.pos[0], o.pos[1], o.radius});
      std::vector<admm::AStarRect> rects;
      for (const auto& r : arena_rects_)
        rects.push_back({r.center[0], r.center[1], r.size[0], r.size[1], r_astar_});
      planner_.reset(new admm::AStarPlanner(astar_res_, r_astar_, circles, ax_min_, ax_max_,
                                            ay_min_, ay_max_, /*boundary_margin=*/0.45, rects));
    }

    // settle-short rescue (detect + reassign + detour). Detector always logs; actions
    // gated by ~rescue. Fingerprint Gazebo-validated on the ROS1 stack.
    rescue_ = declare_parameter<bool>("rescue", true);
    rescue_stall_s_ = declare_parameter<double>("rescue_stall_s", 5.0);
    rescue_v_eps_ = declare_parameter<double>("rescue_v_eps", 0.02);
    rescue_slot_err_ = declare_parameter<double>("rescue_slot_err", 0.5);
    rescue_hyst_ = declare_parameter<double>("rescue_hyst", 0.05);
    rescue_cooldown_s_ = declare_parameter<double>("rescue_cooldown_s", 10.0);

    for (int i : dogs_) {
      const std::string ns = prefix_ + std::to_string(i);
      pub_[i] = create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
          "/" + ns + "/" + ns + "_mpc_target", 1);
      obs_subs_.push_back(create_subscription<ocs2_msgs::msg::MpcObservation>(
          "/" + ns + "/" + ns + "_mpc_observation", 1,
          [this, i](ocs2_msgs::msg::MpcObservation::SharedPtr m) { obs_[i] = *m; has_obs_[i] = true; }));
      odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
          "/" + ns + "/controller/odom", 1,
          [this, i](nav_msgs::msg::Odometry::SharedPtr m) { gt_[i] = *m; has_gt_[i] = true; }));
      goal_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
          "/" + ns + "/goal", 1,
          [this, i](geometry_msgs::msg::PoseStamped::SharedPtr m) { goalCb(*m, i); }));
    }
    fgoal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/formation/goal", 1,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) { formationGoalCb(*m); });

    viz_ = declare_parameter<bool>("viz_markers", true);
    viz_frame_ = declare_parameter<std::string>("viz_frame", "world");
    if (viz_) marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/formation/admm_markers", 1);

    csv_path_ = declare_parameter<std::string>("log_csv", "");
    if (!csv_path_.empty()) {
      csv_.open(csv_path_.c_str());
      if (csv_.is_open()) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "# fleet formation=%s w_form=%.1f v=%.3f\n",
                      formation_name_.c_str(), w_form_, v_);
        csv_ << buf << "t,";
        for (size_t d = 0; d < dogs_.size(); ++d)
          for (size_t c = 0; c < kDbgCols.size(); ++c)
            csv_ << kDbgCols[c] << dogs_[d] << ((d + 1 == dogs_.size() && c + 1 == kDbgCols.size()) ? "" : ",");
        csv_ << "\n"; csv_.flush();
      }
    }
    const auto hpath = declare_parameter<std::string>("log_hist_csv", "");
    if (!hpath.empty()) {
      hist_csv_.open(hpath.c_str());
      if (hist_csv_.is_open()) hist_csv_ << "# ADMM residuals; row = t, r_prim[0..P-1], r_dual[0..P-1], planchg_pos[0..P-1](m), planchg_vel[0..P-1](m/s)\n";
    }

    // node-clock timer: with use_sim_time the 10 Hz tick follows Gazebo /clock (RTF-immune)
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(ts_),
                                  [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "[fleet_centralized] robots=%zu formation=%s w_form=%.1f v=%.2f arena=%s obstacles=%zu com_h=%.2f",
                dogs_.size(), formation_name_.c_str(), w_form_, v_,
                arena_name_.empty() ? "(empty)" : arena_name_.c_str(), coord_->obstacles().size(), com_height_);
  }

 private:
  static const std::vector<std::string> kDbgCols;

  void goalCb(const geometry_msgs::msg::PoseStamped& m, int i) {
    goal_[i] = Eigen::Vector2d(m.pose.position.x, m.pose.position.y);
    path_.erase(i); yaw_latched_[i] = false;
    RCLCPP_INFO(get_logger(), "[fleet_centralized] %s%d new goal (%.2f,%.2f)", prefix_.c_str(), i, goal_[i][0], goal_[i][1]);
    have_slots_ = false;  // per-dog mode: no slots, rescue disarmed
  }

  void formationGoalCb(const geometry_msgs::msg::PoseStamped& m) {
    for (int i : dogs_) if (!has_gt_[i]) { RCLCPP_WARN(get_logger(), "[fleet_centralized] /formation/goal ignored: no state"); return; }
    Eigen::Vector2d goal_c(m.pose.position.x, m.pose.position.y);
    std::vector<Eigen::Vector2d> pos;
    Eigen::Vector2d centroid(0, 0);
    for (int i : dogs_) {
      Eigen::Vector2d p(gt_[i].pose.pose.position.x, gt_[i].pose.pose.position.y);
      pos.push_back(p); centroid += p;
    }
    centroid /= static_cast<double>(dogs_.size());
    Eigen::Vector2d d = goal_c - centroid;
    double yaw = (d.norm() > 0.25) ? std::atan2(d[1], d[0]) : last_formation_yaw_;
    last_formation_yaw_ = yaw;
    const auto* offsets = formation_->current_offsets();
    if (offsets == nullptr || offsets->size() != dogs_.size()) {
      RCLCPP_WARN(get_logger(), "[fleet_centralized] /formation/goal ignored: no formation offsets"); return;
    }
    slots_ = admm::centroid_slot_targets(goal_c, *offsets, yaw);
    assign_ = admm::min_cost_assignment(pos, slots_);
    // a new goal is a clean slate for the rescue state: rescued_until_ too, else a per-dog
    // lockout earned under the PREVIOUS goal silently skips this dog for up to 2*cooldown.
    have_slots_ = true; stall_since_d_.clear(); cooldown_until_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    rescued_until_.clear();
    for (size_t k = 0; k < dogs_.size(); ++k) {
      goal_[dogs_[k]] = slots_[assign_[k]]; path_.erase(dogs_[k]); yaw_latched_[dogs_[k]] = false;
    }
    RCLCPP_INFO(get_logger(), "[fleet_centralized] /formation/goal centroid (%.2f,%.2f) yaw=%.2f", goal_c[0], goal_c[1], yaw);
  }

  bool ready() const {
    for (int i : dogs_) {
      auto o = has_obs_.find(i); auto g = has_gt_.find(i);
      if (o == has_obs_.end() || !o->second) return false;
      if (g == has_gt_.end() || !g->second) return false;
      if (obs_.at(i).time == 0.0) return false;
      // activation-race guard: a short/empty state vector would be UB at BASE_PX/BASE_YAW
      if (obs_.at(i).state.value.size() < 24) return false;
    }
    return true;
  }

  // world X0 (px,py,vx,vy) + P0 (px,py): odom position + EMA/clamped obs velocity.
  void x0(int i, Eigen::Vector4d& X0, Eigen::Vector2d& P0) {
    const auto& s = obs_[i].state.value;
    const auto& gp = gt_[i].pose.pose.position;
    P0 = Eigen::Vector2d(gp.x, gp.y);
    Eigen::Vector2d vraw(s[MOM_LIN_X], s[MOM_LIN_Y]);
    if (!has_vema_[i]) { v_ema_[i] = vraw; has_vema_[i] = true; }
    else v_ema_[i] = 0.25 * vraw + 0.75 * v_ema_[i];
    double vx0 = clip(v_ema_[i][0], -admm::MAX_VX, admm::MAX_VX);
    double vy0 = clip(v_ema_[i][1], -admm::MAX_VY, admm::MAX_VY);
    X0 = Eigen::Vector4d(P0[0], P0[1], vx0, vy0);
  }

  Eigen::MatrixX2d waypoints(int i, const Eigen::Vector2d& P0i) {
    if (path_.find(i) != path_.end()) return to_mat(path_[i]);
    if (!planner_) return to_mat({P0i, goal_[i]});
    auto rg = planner_->find_reachable_goal(P0i, goal_[i]);
    path_[i] = rg.path.empty() ? std::vector<Eigen::Vector2d>{P0i, goal_[i]} : rg.path;
    return to_mat(path_[i]);
  }

  // #1 leader-aware follower cruise speed. Returns v_ unless a dog ahead of i (within the
  // travel-direction cone and follow_range) is slower: then ACC law caps i's speed at
  // v_ahead + gain*(gap - desired) so i decelerates to match rather than driving into it.
  double followSpeed(int i, const std::map<int, Eigen::VectorXd>& xnow) {
    if (follow_gain_ <= 0.0) return v_;
    const Eigen::Vector2d pi = xnow.at(i).head<2>();
    // travel direction = next reference waypoint when a path exists (an A* route or a
    // rescue detour bends away from the straight goal bearing).
    Eigen::Vector2d tgt = goal_[i];
    const auto it = path_.find(i);
    if (it != path_.end() && it->second.size() >= 2) {
      // project onto the path first (nearest vertex), THEN look ahead from there
      size_t k0 = 0; double dbest = 1e18;
      for (size_t k = 0; k < it->second.size(); ++k) {
        const double d = (it->second[k] - pi).norm();
        if (d < dbest) { dbest = d; k0 = k; }
      }
      for (size_t k = k0; k < it->second.size(); ++k)
        if ((it->second[k] - pi).norm() > 0.3) { tgt = it->second[k]; break; }
    }
    Eigen::Vector2d dir = tgt - pi;
    const double dn = dir.norm();
    if (dn < 1e-3) return v_;  // at goal: no travel direction to define "ahead"
    dir /= dn;
    double v_eff = v_;
    for (int j : dogs_) {
      if (j == i) continue;
      const Eigen::Vector2d d = xnow.at(j).head<2>() - pi;
      const double dist = d.norm();
      if (dist < 1e-3 || dist > follow_range_) continue;
      const double proj = d.dot(dir);
      if (proj <= 0.0 || proj / dist < follow_cone_cos_) continue;  // not ahead / outside cone
      const double v_ahead = std::max(0.0, xnow.at(j).segment<2>(2).dot(dir));
      const double v_allow = v_ahead + follow_gain_ * (dist - follow_desired_);
      v_eff = std::min(v_eff, std::max(v_allow, follow_floor_));
    }
    return v_eff;
  }

  // Settle-short deadlock detector. PER-DOG stall: dog i is stuck when it sits still
  // while > rescue_slot_err off its slot. Detector always logs; actions gated by ~rescue.
  void rescueTick(const std::map<int, Eigen::VectorXd>& xnow, const rclcpp::Time& now) {
    if (!have_slots_) return;
    int cand = -1; double cerr = -1.0;
    for (int i : dogs_) {
      const double e = (xnow.at(i).head<2>() - goal_[i]).norm();
      const bool stuck = xnow.at(i).segment<2>(2).norm() < rescue_v_eps_ && e > rescue_slot_err_;
      if (!stuck) { stall_since_d_.erase(i); continue; }
      if (!stall_since_d_.count(i)) { stall_since_d_[i] = now; continue; }
      // per-dog rescue cooldown: a freshly-detoured dog steps aside so the OTHER half of
      // a wedged pair gets routed too
      if (rescued_until_.count(i) && now < rescued_until_.at(i)) continue;
      if ((now - stall_since_d_[i]).seconds() >= rescue_stall_s_ && e > cerr) { cerr = e; cand = i; }
    }
    if (cand < 0 || (have_cooldown_ && now < cooldown_until_)) return;
    std::string errs;
    for (int i : dogs_) {
      char b[16]; std::snprintf(b, sizeof(b), "%.2f", (xnow.at(i).head<2>() - goal_[i]).norm());
      errs += std::string(b) + (i == dogs_.back() ? "" : ",");
    }
    RCLCPP_WARN(get_logger(), "[rescue] FIRE errs=%s", errs.c_str());
    cooldown_until_ = now + rclcpp::Duration::from_seconds(rescue_cooldown_s_); have_cooldown_ = true;
    stall_since_d_.clear();
    if (!rescue_) return;
    // stage-1: re-match dogs->slots from CURRENT positions. Hysteresis: only accept a
    // strictly better matching (squared-distance margin) so symmetric ties never thrash.
    std::vector<Eigen::Vector2d> pos;
    for (int i : dogs_) pos.push_back(xnow.at(i).head<2>());
    const std::vector<int> a_new = admm::min_cost_assignment(pos, slots_);
    double c_old = 0.0, c_new = 0.0;
    for (size_t k = 0; k < dogs_.size(); ++k) {
      c_old += (pos[k] - slots_[assign_[k]]).squaredNorm();
      c_new += (pos[k] - slots_[a_new[k]]).squaredNorm();
    }
    if (a_new != assign_ && c_new < c_old - rescue_hyst_) {
      for (size_t k = 0; k < dogs_.size(); ++k) {
        // ONLY touch dogs whose slot actually changed (unlatching a parked dog re-arms
        // the yaw-runaway the goal-proximity latch exists to prevent).
        if (a_new[k] == assign_[k]) continue;
        goal_[dogs_[k]] = slots_[a_new[k]]; path_.erase(dogs_[k]); yaw_latched_[dogs_[k]] = false;
      }
      assign_ = a_new;
      RCLCPP_WARN(get_logger(), "[rescue] stage-1 reassign (cost %.2f -> %.2f)", c_old, c_new);
      return;
    }
    // stage-2: symmetric deadlock -> give the WORST dog a reference that routes AROUND
    // its parked teammates (BVC-style detour). No path even after a start nudge =
    // genuinely fenced out -> hold and log (retry is free after the cooldown).
    const int worst = cand;
    const double mate_r = std::max(0.30, admm::D_MIN - r_astar_);
    std::vector<admm::AStarCircle> circles;
    for (const auto& o : coord_->obstacles())
      circles.push_back({o.pos[0], o.pos[1], o.radius});
    Eigen::Vector2d pw = xnow.at(worst).head<2>(), nearest_mate = pw;
    double dmin_mate = 1e9;
    for (int j : dogs_) {
      if (j == worst) continue;
      const Eigen::Vector2d pj = xnow.at(j).head<2>();
      circles.push_back({pj[0], pj[1], mate_r});
      const double d = (pj - pw).norm();
      if (d < dmin_mate) { dmin_mate = d; nearest_mate = pj; }
    }
    std::vector<admm::AStarRect> rects;
    for (const auto& r : arena_rects_)
      rects.push_back({r.center[0], r.center[1], r.size[0], r.size[1], r_astar_});
    const admm::AStarPlanner local(astar_res_, r_astar_, circles, ax_min_, ax_max_,
                                   ay_min_, ay_max_, /*boundary_margin=*/0.45, rects);
    std::vector<Eigen::Vector2d> p = local.plan(pw, goal_[worst]);
    if (p.empty() && dmin_mate < 1e9) {
      // start cell may sit inside a teammate's inflated disc -> nudge start outward
      const Eigen::Vector2d away = (pw - nearest_mate).normalized();
      const Eigen::Vector2d s2 = nearest_mate + away * (mate_r + r_astar_ + astar_res_);
      p = local.plan(s2, goal_[worst]);
      if (!p.empty()) p.insert(p.begin(), pw);
    }
    if (p.empty()) { RCLCPP_WARN(get_logger(), "[rescue] fenced out %s%d (no path)", prefix_.c_str(), worst); return; }
    path_[worst] = p; yaw_latched_[worst] = false;
    rescued_until_[worst] = now + rclcpp::Duration::from_seconds(2.0 * rescue_cooldown_s_);
    RCLCPP_WARN(get_logger(), "[rescue] stage-2 detour %s%d wp=%zu", prefix_.c_str(), worst, p.size());
  }

  void tick() {
    if (!ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[fleet_centralized] waiting for all obs+odom ...");
      return;
    }
    std::map<int, Eigen::VectorXd> xnow;
    std::map<int, Eigen::Vector2d> P0;
    std::map<int, Eigen::MatrixXd> xdes;
    for (int i : dogs_) {
      Eigen::Vector4d X0; Eigen::Vector2d p0; x0(i, X0, p0);
      xnow[i] = X0; P0[i] = p0;
    }
    rescueTick(xnow, now());
    for (int i : dogs_) {
      // waypoints() WRITES path_, followSpeed() READS it — sequence them explicitly.
      const Eigen::MatrixX2d wp = waypoints(i, P0[i]);
      xdes[i] = admm::build_reference(P0[i], wp, admm::N, admm::TS,
                                      followSpeed(i, xnow), 0.80);
    }

    std::map<int, Eigen::VectorXd> xi;
    admm::ADMMCoordinator::Hist hist;
    try {
      auto res = coord_->step(xnow, xdes);
      xi = res.first; hist = res.second;
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[fleet_centralized] coord.step failed: %s", e.what());
      return;
    }

    if (hist_csv_.is_open()) {
      char b[32]; std::snprintf(b, sizeof(b), "%.4f,", obs_[dogs_[0]].time); hist_csv_ << b;
      for (size_t k = 0; k < hist.r_prim.size(); ++k) { std::snprintf(b, sizeof(b), "%.6g", hist.r_prim[k]); hist_csv_ << b << ","; }
      for (size_t k = 0; k < hist.r_dual.size(); ++k) { std::snprintf(b, sizeof(b), "%.6g", hist.r_dual[k]); hist_csv_ << b << ","; }
      for (size_t k = 0; k < hist.r_pos.size(); ++k) { std::snprintf(b, sizeof(b), "%.6g", hist.r_pos[k]); hist_csv_ << b << ","; }
      for (size_t k = 0; k < hist.r_vel.size(); ++k) { std::snprintf(b, sizeof(b), "%.6g", hist.r_vel[k]); hist_csv_ << b << (k + 1 < hist.r_vel.size() ? "," : ""); }
      hist_csv_ << "\n"; hist_csv_.flush();
    }

    for (int i : dogs_)
      if (!xi[i].allFinite()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[fleet_centralized] ADMM xi non-finite -> HOLD");
        return;
      }

    std::map<int, Eigen::VectorXd> wpx, wpy;
    for (int i : dogs_) {
      Eigen::VectorXd px(admm::N), py(admm::N);
      for (int k = 1; k <= admm::N; ++k) { px[k - 1] = xi[i][admm::px_index(k)]; py[k - 1] = xi[i][admm::py_index(k)]; }
      wpx[i] = px; wpy[i] = py;
    }
    if (viz_) publishMarkers(wpx, wpy);

    std::map<int, std::vector<double>> dbg;  // per-dog instrumentation for CSV
    for (int i : dogs_) {
      const auto& s = obs_[i].state.value;
      Eigen::VectorXd xi_mpc = xi[i];
      double dx = s[BASE_PX] - P0[i][0], dy = s[BASE_PY] - P0[i][1];
      for (int k = 1; k <= admm::N; ++k) { xi_mpc[admm::px_index(k)] += dx; xi_mpc[admm::py_index(k)] += dy; }
      std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> others;
      for (int j : dogs_) if (j != i) others.push_back({wpx[j], wpy[j]});
      int k_send = admm::safe_prefix_length(wpx[i], wpy[i], others, d_safe_, send_cap_);
      auto out = adapter_->build_target(xi_mpc, obs_[i].time, s[BASE_YAW], "path", k_send);
      Eigen::MatrixXd states = out.states;  // (ns, 24)

      double d_goal = std::hypot(P0[i][0] - goal_[i][0], P0[i][1] - goal_[i][1]);
      if (!yaw_latched_[i]) {
        if (d_goal < r_latch_) { yaw_latched_[i] = true; yaw_latch_val_[i] = s[BASE_YAW]; }
      } else if (d_goal > r_latch_ + latch_margin_) {
        yaw_latched_[i] = false;
      }
      if (yaw_latched_[i])
        for (int r = 0; r < states.rows(); ++r) states(r, BASE_YAW) = yaw_latch_val_[i];

      pub_[i]->publish(admm::toTargetMsg(out.times, states));
      dbg[i] = {gt_[i].pose.pose.position.x, gt_[i].pose.pose.position.y,
                (double)s[BASE_PX], (double)s[BASE_PY], dx, dy,
                (double)s[MOM_LIN_X], (double)s[MOM_LIN_Y], xnow[i][2], xnow[i][3],
                states(0, BASE_PX), states(0, BASE_PY), (double)s[BASE_YAW],
                states(0, BASE_YAW), (double)k_send,
                goal_[i][0], goal_[i][1]};
    }
    logCsv(dbg);
  }

  void logCsv(const std::map<int, std::vector<double>>& dbg) {
    if (!csv_.is_open()) return;
    char b[32]; std::snprintf(b, sizeof(b), "%.4f", obs_[dogs_[0]].time); csv_ << b;
    for (int i : dogs_) {
      for (double v : dbg.at(i)) { std::snprintf(b, sizeof(b), "%.5f", v); csv_ << "," << b; }
    }
    csv_ << "\n"; csv_.flush();
  }

  void publishMarkers(const std::map<int, Eigen::VectorXd>& wpx, const std::map<int, Eigen::VectorXd>& wpy) {
    visualization_msgs::msg::MarkerArray arr;
    int j = 0;
    for (const auto& o : coord_->obstacles()) {
      std::vector<Eigen::Vector3d> pts;
      for (int k = 0; k <= 24; ++k) {
        double a = 2 * M_PI * k / 24.0;
        pts.emplace_back(o.pos[0] + o.radius * std::cos(a), o.pos[1] + o.radius * std::sin(a), 0.05);
      }
      arr.markers.push_back(lineMarker("obstacles", j++, pts, 0.55, 0.55, 0.55, 0.03));
    }
    int w = 0;  // walls: rect obstacle outlines
    for (const auto& r : arena_rects_) {
      const double hx = r.size[0] / 2.0, hy = r.size[1] / 2.0;
      std::vector<Eigen::Vector3d> pts;
      pts.emplace_back(r.center[0] - hx, r.center[1] - hy, 0.05);
      pts.emplace_back(r.center[0] + hx, r.center[1] - hy, 0.05);
      pts.emplace_back(r.center[0] + hx, r.center[1] + hy, 0.05);
      pts.emplace_back(r.center[0] - hx, r.center[1] + hy, 0.05);
      pts.emplace_back(r.center[0] - hx, r.center[1] - hy, 0.05);
      arr.markers.push_back(lineMarker("walls", w++, pts, 0.85, 0.85, 0.85, 0.05));
    }
    static const std::map<int, Eigen::Vector3d> rgb = {
        {1, {0.90, 0.20, 0.20}}, {2, {0.20, 0.80, 0.30}}, {3, {0.20, 0.45, 0.95}}};
    std::map<int, Eigen::Vector2d> pos;
    for (int i : dogs_) pos[i] = Eigen::Vector2d(gt_[i].pose.pose.position.x, gt_[i].pose.pose.position.y);
    for (int i : dogs_) {
      Eigen::Vector3d c = rgb.count(i) ? rgb.at(i) : Eigen::Vector3d(0.6, 0.6, 0.6);
      std::vector<Eigen::Vector3d> roll;
      for (int k = 0; k < wpx.at(i).size(); ++k) roll.emplace_back(wpx.at(i)[k], wpy.at(i)[k], 0.1);
      arr.markers.push_back(lineMarker("rollout", i, roll, c[0], c[1], c[2], 0.05));
      arr.markers.push_back(sphereMarker("dog", i, pos[i], c, 0.22));
      arr.markers.push_back(sphereMarker("slot", i, goal_[i], c, 0.12));
    }
    std::vector<Eigen::Vector3d> ep;
    for (size_t a = 0; a < dogs_.size(); ++a)
      for (size_t b = a + 1; b < dogs_.size(); ++b) {
        ep.emplace_back(pos[dogs_[a]][0], pos[dogs_[a]][1], 0.1);
        ep.emplace_back(pos[dogs_[b]][0], pos[dogs_[b]][1], 0.1);
      }
    auto fm = lineMarker("formation", 0, ep, 1.0, 0.85, 0.1, 0.03);
    fm.type = visualization_msgs::msg::Marker::LINE_LIST;
    arr.markers.push_back(fm);
    marker_pub_->publish(arr);
  }

  visualization_msgs::msg::Marker lineMarker(const std::string& ns, int id,
                                             const std::vector<Eigen::Vector3d>& pts,
                                             double r, double g, double b, double w) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = viz_frame_;
    m.lifetime = rclcpp::Duration::from_seconds(3 * ts_);
    m.ns = ns; m.id = id; m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD; m.scale.x = w; m.pose.orientation.w = 1.0;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0;
    for (const auto& p : pts) { geometry_msgs::msg::Point q; q.x = p[0]; q.y = p[1]; q.z = p[2]; m.points.push_back(q); }
    return m;
  }

  visualization_msgs::msg::Marker sphereMarker(const std::string& ns, int id, const Eigen::Vector2d& xy,
                                               const Eigen::Vector3d& c, double d) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = viz_frame_;
    m.lifetime = rclcpp::Duration::from_seconds(3 * ts_);
    m.ns = ns; m.id = id; m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = d; m.pose.orientation.w = 1.0;
    m.pose.position.x = xy[0]; m.pose.position.y = xy[1]; m.pose.position.z = 0.1;
    m.color.r = c[0]; m.color.g = c[1]; m.color.b = c[2]; m.color.a = 1.0;
    return m;
  }

  std::vector<int> dogs_;
  std::string prefix_;
  std::vector<admm::EdgeKey> edges_;
  int N_; double ts_, v_, com_height_{0.5}, w_form_;
  std::string formation_name_, arena_name_, viz_frame_, csv_path_;
  std::map<int, Eigen::Vector2d> goal_;
  std::unique_ptr<admm::LaplacianFormation> formation_;
  std::unique_ptr<admm::ADMMCoordinator> coord_;
  std::unique_ptr<admm::MotionAdapter> adapter_;
  std::unique_ptr<admm::AStarPlanner> planner_;
  std::vector<admm::ArenaRect> arena_rects_;
  double last_formation_yaw_ = 0.0, r_latch_, latch_margin_, d_safe_;
  double follow_gain_, follow_desired_, follow_range_, follow_cone_cos_, follow_floor_;
  bool rescue_{true}, have_slots_{false}, have_cooldown_{false};
  double rescue_stall_s_, rescue_v_eps_, rescue_slot_err_, rescue_hyst_, rescue_cooldown_s_;
  double r_astar_, astar_res_, ax_min_, ax_max_, ay_min_, ay_max_;
  std::vector<Eigen::Vector2d> slots_;
  std::vector<int> assign_;
  std::map<int, rclcpp::Time> stall_since_d_, rescued_until_;
  rclcpp::Time cooldown_until_;
  int send_cap_; bool use_astar_, viz_;
  std::map<int, bool> yaw_latched_; std::map<int, double> yaw_latch_val_;
  std::map<int, std::vector<Eigen::Vector2d>> path_;
  std::map<int, ocs2_msgs::msg::MpcObservation> obs_; std::map<int, bool> has_obs_;
  std::map<int, nav_msgs::msg::Odometry> gt_; std::map<int, bool> has_gt_;
  std::map<int, Eigen::Vector2d> v_ema_; std::map<int, bool> has_vema_;
  std::map<int, rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr> pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::vector<rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr> obs_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> goal_subs_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fgoal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream csv_, hist_csv_;
};

const std::vector<std::string> FleetCentralizedNode::kDbgCols = {
    "gt_x", "gt_y", "estpx", "estpy", "dx", "dy", "obsvx", "obsvy",
    "x0vx", "x0vy", "tgt1x", "tgt1y", "seed", "cmd", "ksend", "slotx", "sloty"};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FleetCentralizedNode>());
  rclcpp::shutdown();
  return 0;
}
