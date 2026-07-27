// fleet_coordinator_node — standalone formation slot allocator (extracted from the dog1-hosted
// fleet-logic that used to live in admm_agent_node). Subscribes /formation/goal + every dog's
// odom, assigns each dog a formation slot (centroid_slot_targets + min_cost_assignment), and
// publishes the latched /formation/plan that the distributed agents subscribe to.
//
// This is centralized TASK ALLOCATION (one node decides slots), but on its own process so no dog
// is special. It is NOT the distributed control — the per-robot ADMM/CBF/MPC in admm_agent_node
// stays fully distributed. Because /formation/plan is latched (transient_local), a coordinator
// crash leaves the last plan standing: the fleet holds its assignment and keeps running; only
// NEW goals and rescue reassignment pause until it restarts.
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "admm_fleet_msgs/msg/agent_state.hpp"
#include "admm_fleet_msgs/msg/fleet_plan.hpp"
#include "legged_upper_control/admm_formation.hpp"
#include "legged_upper_control/fleet_config.hpp"

using namespace std::chrono_literals;

class FleetCoordinatorNode : public rclcpp::Node {
public:
    FleetCoordinatorNode() : rclcpp::Node("fleet_coordinator") {
        const auto ids = declare_parameter<std::vector<int64_t>>("robot_ids", {1, 2, 3});
        for (auto v : ids) dogs_.push_back(static_cast<int>(v));
        prefix_ = declare_parameter<std::string>("robot_prefix", "robot");
        formation_name_ = declare_parameter<std::string>("formation", "V");
        formation_ = std::make_unique<admm::LaplacianFormation>(admm::formations());
        formation_->set_formation(formation_name_);
        // settle-short rescue params (match admm_agent_node / fleet_centralized_node defaults)
        rescue_v_eps_ = declare_parameter<double>("rescue_v_eps", 0.02);
        rescue_slot_err_ = declare_parameter<double>("rescue_slot_err", 0.5);
        rescue_stall_s_ = declare_parameter<double>("rescue_stall_s", 5.0);
        rescue_hyst_ = declare_parameter<double>("rescue_hyst", 0.05);
        rescue_cooldown_s_ = declare_parameter<double>("rescue_cooldown_s", 10.0);

        // latched (transient_local) so a late-joining agent still gets the current plan.
        plan_pub_ = create_publisher<admm_fleet_msgs::msg::FleetPlan>(
            "/formation/plan", rclcpp::QoS(1).transient_local());
        // Rebroadcast every 1 s: latching covers late-joiners, and a steady republish guarantees
        // every dog picks up its slot within ~1 s even if its plan callback was transiently missed.
        plan_republish_timer_ = create_wall_timer(1s, [this] {
            std::lock_guard<std::mutex> pl(plan_mu_);
            if (has_plan_) plan_pub_->publish(last_plan_);
        });
        for (int j : dogs_)
            odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
                "/" + prefix_ + std::to_string(j) + "/controller/odom", 1,
                [this, j](nav_msgs::msg::Odometry::SharedPtr m) {
                    std::lock_guard<std::mutex> l(fl_mu_);
                    fl_pos_[j] = Eigen::Vector2d(m->pose.pose.position.x, m->pose.pose.position.y);
                    fl_vel_[j] = Eigen::Vector2d(m->twist.twist.linear.x, m->twist.twist.linear.y);
                }));
        // Liveness heartbeat. AgentCore::step() broadcasts AgentState as its FIRST action, so
        // this arrives even on a cycle that later throws — unlike /admm/stats, which cycle()
        // skips entirely on an exception and would read as "dead" after a couple of bad cycles.
        // Same source the agents use for eviction, so both sides agree on who is alive.
        alive_timeout_ = declare_parameter<double>("alive_timeout", 2.0);
        for (int j : dogs_)
            state_subs_.push_back(create_subscription<admm_fleet_msgs::msg::AgentState>(
                "/" + prefix_ + std::to_string(j) + "/admm/state", rclcpp::QoS(10).reliable(),
                [this, j](admm_fleet_msgs::msg::AgentState::SharedPtr) {
                    std::lock_guard<std::mutex> l(fl_mu_);
                    fl_alive_[j] = now();
                }));
        fgoal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/formation/goal", 1, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
                onFormationGoal(Eigen::Vector2d(m->pose.position.x, m->pose.position.y));
            });
        if (declare_parameter<bool>("rescue", true))
            rescue_timer_ = create_wall_timer(1s, [this] { rescueTick_(); });
        RCLCPP_INFO(get_logger(), "[fleet_coordinator] dogs=%zu formation=%s prefix=%s",
                    dogs_.size(), formation_name_.c_str(), prefix_.c_str());
    }

private:
    // Robots whose ADMM agent is still speaking AND whose odom we have. Liveness comes from
    // /robotN/admm/state, NOT odom: freezing or killing an agent leaves the lower-level
    // controller publishing odom forever, so odom-based liveness never notices the failure.
    // Caller holds fl_mu_.
    std::vector<int> liveDogs() const {
        std::vector<int> live;
        const auto t = now();
        for (int i : dogs_) {
            if (!fl_pos_.count(i)) continue;  // see notReady_: caller must refuse first
            const auto it = fl_alive_.find(i);
            // NEVER heard from = still starting up, not dead. Controllers come up a phase
            // before the agents, so there is a window where odom exists but /admm/state does
            // not; calling that dead would lay out a degraded shape for a merely-late fleet.
            // Mirrors the agents' own bring-up grace (maybeEvict arms only after has_prev()).
            if (it == fl_alive_.end() || (t - it->second).seconds() < alive_timeout_)
                live.push_back(i);
        }
        return live;
    }

    // "No odom yet" and "dead" are different and must not be conflated. Odom is the very first
    // thing a healthy robot produces, so a missing one means the fleet is still coming up —
    // the pre-liveness behaviour (refuse the goal outright) is correct there, and degrading the
    // formation instead would hand a merely-late fleet a 2-dog shape. Only a STALE heartbeat
    // on a robot that is otherwise reporting odom means "dead" and licenses degrading.
    // Caller holds fl_mu_. Returns the first not-ready id, or -1.
    int notReady_() const {
        for (int i : dogs_)
            if (!fl_pos_.count(i)) return i;
        return -1;
    }

    void onFormationGoal(const Eigen::Vector2d& goal_c) {
        std::lock_guard<std::mutex> l(fl_mu_);
        onFormationGoalLocked_(goal_c);
    }

    // Caller holds fl_mu_.
    void onFormationGoalLocked_(const Eigen::Vector2d& goal_c) {
        last_goal_ = goal_c;
        has_goal_ = true;
        if (const int nr = notReady_(); nr >= 0) {
            RCLCPP_WARN(get_logger(), "coordinator: missing pos %d", nr);  // pre-liveness behaviour
            return;
        }
        const std::vector<int> live = liveDogs();
        if (live.empty()) {
            RCLCPP_WARN(get_logger(), "[fleet_coordinator] /formation/goal ignored: no live dogs");
            return;
        }
        // A single survivor has no formation to hold — send it straight at the commanded
        // centroid. Without this the shape lookup below returns "" (shape_for(1)), which
        // set_formation() treats as a silent no-op, leaving the previous 3-slot offsets in
        // place; the size guard then rejects the goal and the last live dog never gets one.
        if (live.size() == 1) {
            fl_live_ = live;
            fl_slots_ = {goal_c};
            fl_assign_ = {0};
            fl_have_slots_ = true;
            stall_since_.clear();
            have_cooldown_ = false;
            publishPlan_();
            RCLCPP_INFO(get_logger(), "[fleet_coordinator] /formation/goal (%.2f,%.2f) -> solo plan for robot%d",
                        goal_c[0], goal_c[1], live[0]);
            return;
        }

        std::vector<Eigen::Vector2d> pos;
        Eigen::Vector2d centroid(0, 0);
        for (int i : live) {
            pos.push_back(fl_pos_[i]);
            centroid += fl_pos_[i];
        }
        centroid /= static_cast<double>(live.size());
        const Eigen::Vector2d d = goal_c - centroid;
        // goal ~ centroid (no clear bearing): hold the current heading, not +x — else the
        // three dogs needlessly rotate/reshuffle to face +x. Mirrors centralized formationGoalCb.
        const double yaw = (d.norm() > 0.25) ? std::atan2(d[1], d[0]) : last_formation_yaw_;
        last_formation_yaw_ = yaw;
        // Shape follows the LIVE count, via the same rule every agent uses (admm::shape_for),
        // so the slot targets and the agents' formation gradient never pull toward different
        // shapes. Leaving this at the 3-slot "V" after a death makes the size guard below
        // reject every goal — symptom: "new goal does nothing", one WARN line.
        formation_->set_formation(admm::shape_for(live.size(), dogs_.size(), formation_name_));
        const auto* off = formation_->current_offsets();
        if (off == nullptr || off->size() != live.size()) {  // no shape defined for this count
            RCLCPP_WARN(get_logger(),
                        "[fleet_coordinator] /formation/goal ignored: no %zu-slot formation offsets",
                        live.size());
            return;
        }
        fl_live_ = live;
        fl_slots_ = admm::centroid_slot_targets(goal_c, *off, yaw);
        fl_assign_ = admm::min_cost_assignment(pos, fl_slots_);
        fl_have_slots_ = true;
        stall_since_.clear();   // fresh goal: reset rescue stall timers
        have_cooldown_ = false;  // ...and clear a rescue cooldown earned under the PREVIOUS goal,
                                 // else that lockout silently skips reassignment under the new goal
        publishPlan_();          // caller holds fl_mu_
        RCLCPP_INFO(get_logger(), "[fleet_coordinator] /formation/goal (%.2f,%.2f) yaw=%.2f shape=%s -> plan for %zu dog(s)",
                    goal_c[0], goal_c[1], yaw,
                    admm::shape_for(live.size(), dogs_.size(), formation_name_).c_str(), live.size());
    }

    // Build + publish the FleetPlan from the current slots/assignment. Caller holds fl_mu_.
    // Only LIVE robots appear: an id omitted here simply keeps whatever goal it last latched,
    // which is what we want for a robot that is not executing plans any more.
    void publishPlan_() {
        admm_fleet_msgs::msg::FleetPlan pm;
        for (size_t k = 0; k < fl_live_.size(); ++k) {
            pm.robot_ids.push_back(fl_live_[k]);
            pm.goals.push_back(fl_slots_[fl_assign_[k]][0]);
            pm.goals.push_back(fl_slots_[fl_assign_[k]][1]);
        }
        { std::lock_guard<std::mutex> pl(plan_mu_); last_plan_ = pm; has_plan_ = true; }
        plan_pub_->publish(pm);
    }

    // Settle-short rescue (stage-1). A dog that sits still (|v|<eps) far from its slot (>slot_err)
    // for >stall_s is stuck: re-match dogs to slots from CURRENT positions and, if strictly cheaper
    // (hysteresis), republish. Breaks the 180deg formation-reversal gridlock.
    void rescueTick_() {
        std::lock_guard<std::mutex> l(fl_mu_);
        if (!fl_have_slots_ || fl_live_.empty()) return;
        // fl_assign_ is indexed by position in fl_live_ as it stood at the last assignment.
        // If the live set has changed since, those indices are stale — liveSetTick_ will
        // reassign; skip this tick rather than read the wrong slot.
        if (fl_live_ != liveDogs()) return;
        for (int i : fl_live_) if (!fl_pos_.count(i) || !fl_vel_.count(i)) return;
        const auto now = this->now();
        int cand = -1;
        double cerr = -1.0;
        for (size_t k = 0; k < fl_live_.size(); ++k) {
            const int i = fl_live_[k];
            const double e = (fl_pos_[i] - fl_slots_[fl_assign_[k]]).norm();
            const bool stuck = fl_vel_[i].norm() < rescue_v_eps_ && e > rescue_slot_err_;
            if (!stuck) { stall_since_.erase(i); continue; }
            if (!stall_since_.count(i)) { stall_since_[i] = now; continue; }
            if ((now - stall_since_[i]).seconds() >= rescue_stall_s_ && e > cerr) { cerr = e; cand = i; }
        }
        if (cand < 0 || (have_cooldown_ && now < cooldown_until_)) return;
        cooldown_until_ = now + rclcpp::Duration::from_seconds(rescue_cooldown_s_);
        have_cooldown_ = true;
        stall_since_.clear();
        std::vector<Eigen::Vector2d> pos;
        for (int i : fl_live_) pos.push_back(fl_pos_[i]);
        const auto a_new = admm::min_cost_assignment(pos, fl_slots_);
        double c_old = 0.0, c_new = 0.0;
        for (size_t k = 0; k < fl_live_.size(); ++k) {
            c_old += (pos[k] - fl_slots_[fl_assign_[k]]).squaredNorm();
            c_new += (pos[k] - fl_slots_[a_new[k]]).squaredNorm();
        }
        if (a_new != fl_assign_ && c_new < c_old - rescue_hyst_) {
            fl_assign_ = a_new;
            publishPlan_();
            RCLCPP_WARN(get_logger(), "[rescue] stage-1 reassign (cost %.2f -> %.2f)", c_old, c_new);
        } else {
            RCLCPP_WARN(get_logger(), "[rescue] stall (worst err %.2f) but no cheaper matching", cerr);
        }
    }

    std::vector<int> dogs_;
    std::string prefix_, formation_name_;
    std::unique_ptr<admm::LaplacianFormation> formation_;
    double rescue_v_eps_, rescue_slot_err_, rescue_stall_s_, rescue_hyst_, rescue_cooldown_s_;

    rclcpp::Publisher<admm_fleet_msgs::msg::FleetPlan>::SharedPtr plan_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fgoal_sub_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
    std::vector<rclcpp::Subscription<admm_fleet_msgs::msg::AgentState>::SharedPtr> state_subs_;
    rclcpp::TimerBase::SharedPtr plan_republish_timer_, rescue_timer_;

    std::mutex fl_mu_, plan_mu_;
    std::map<int, Eigen::Vector2d> fl_pos_, fl_vel_;
    admm_fleet_msgs::msg::FleetPlan last_plan_;
    bool has_plan_ = false;
    std::map<int, rclcpp::Time> fl_alive_;   // last /robotN/admm/state arrival, per robot
    double alive_timeout_ = 2.0;             // must exceed the agents' evict_after_misses * TS
    std::vector<int> fl_live_;               // robots the CURRENT fl_slots_/fl_assign_ cover
    std::vector<Eigen::Vector2d> fl_slots_;
    std::vector<int> fl_assign_;
    bool fl_have_slots_ = false;
    Eigen::Vector2d last_goal_{0, 0};        // replayed when the live set changes (liveSetTick_)
    bool has_goal_ = false;
    double last_formation_yaw_ = 0.0;
    std::map<int, rclcpp::Time> stall_since_;
    rclcpp::Time cooldown_until_;
    bool have_cooldown_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FleetCoordinatorNode>());
    rclcpp::shutdown();
    return 0;
}
