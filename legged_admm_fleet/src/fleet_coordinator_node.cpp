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
    void onFormationGoal(const Eigen::Vector2d& goal_c) {
        std::lock_guard<std::mutex> l(fl_mu_);
        std::vector<Eigen::Vector2d> pos;
        Eigen::Vector2d centroid(0, 0);
        for (int i : dogs_) {
            if (!fl_pos_.count(i)) { RCLCPP_WARN(get_logger(), "coordinator: missing pos %d", i); return; }
            pos.push_back(fl_pos_[i]);
            centroid += fl_pos_[i];
        }
        centroid /= static_cast<double>(dogs_.size());
        const Eigen::Vector2d d = goal_c - centroid;
        // goal ~ centroid (no clear bearing): hold the current heading, not +x — else the
        // three dogs needlessly rotate/reshuffle to face +x. Mirrors centralized formationGoalCb.
        const double yaw = (d.norm() > 0.25) ? std::atan2(d[1], d[0]) : last_formation_yaw_;
        last_formation_yaw_ = yaw;
        const auto* off = formation_->current_offsets();
        if (off == nullptr || off->size() != dogs_.size()) {  // invalid `formation` param -> no offsets
            RCLCPP_WARN(get_logger(), "[fleet_coordinator] /formation/goal ignored: no formation offsets");
            return;
        }
        fl_slots_ = admm::centroid_slot_targets(goal_c, *off, yaw);
        fl_assign_ = admm::min_cost_assignment(pos, fl_slots_);
        fl_have_slots_ = true;
        stall_since_.clear();   // fresh goal: reset rescue stall timers
        have_cooldown_ = false;  // ...and clear a rescue cooldown earned under the PREVIOUS goal,
                                 // else that lockout silently skips reassignment under the new goal
        publishPlan_();          // caller holds fl_mu_
        RCLCPP_INFO(get_logger(), "[fleet_coordinator] /formation/goal (%.2f,%.2f) yaw=%.2f -> plan published",
                    goal_c[0], goal_c[1], yaw);
    }

    // Build + publish the FleetPlan from the current slots/assignment. Caller holds fl_mu_.
    void publishPlan_() {
        admm_fleet_msgs::msg::FleetPlan pm;
        for (size_t k = 0; k < dogs_.size(); ++k) {
            pm.robot_ids.push_back(dogs_[k]);
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
        if (!fl_have_slots_) return;
        for (int i : dogs_) if (!fl_pos_.count(i) || !fl_vel_.count(i)) return;
        const auto now = this->now();
        int cand = -1;
        double cerr = -1.0;
        for (size_t k = 0; k < dogs_.size(); ++k) {
            const int i = dogs_[k];
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
        for (int i : dogs_) pos.push_back(fl_pos_[i]);
        const auto a_new = admm::min_cost_assignment(pos, fl_slots_);
        double c_old = 0.0, c_new = 0.0;
        for (size_t k = 0; k < dogs_.size(); ++k) {
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
    rclcpp::TimerBase::SharedPtr plan_republish_timer_, rescue_timer_;

    std::mutex fl_mu_, plan_mu_;
    std::map<int, Eigen::Vector2d> fl_pos_, fl_vel_;
    admm_fleet_msgs::msg::FleetPlan last_plan_;
    bool has_plan_ = false;
    std::vector<Eigen::Vector2d> fl_slots_;
    std::vector<int> fl_assign_;
    bool fl_have_slots_ = false;
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
