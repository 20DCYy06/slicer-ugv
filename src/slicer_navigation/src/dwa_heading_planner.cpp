#include <slicer_navigation/start_heading.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <angles/angles.h>
#include <base_local_planner/goal_functions.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace slicer_navigation {

class HeadingDWAPlanner : public nav_core::BaseLocalPlanner {
 public:
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap) override {
    if (costmap_) return;
    tf_ = tf;
    costmap_ = costmap;
    ros::NodeHandle nh("~/" + name);
    double rate, tolerance, settle, timeout, kp, kd;
    nh.param("start_heading_rate", rate, .45);
    nh.param("start_heading_tolerance", tolerance, .12);
    nh.param("start_heading_settle", settle, .25);
    nh.param("start_heading_timeout", timeout, 16.);
    nh.param("start_heading_min_distance", min_distance_, .20);
    nh.param("start_heading_lookahead", lookahead_, .20);
    nh.param("start_heading_kp", kp, 1.6);
    nh.param("start_heading_kd", kd, .25);
    for (double value : {rate, tolerance, settle, timeout, min_distance_, lookahead_, kp, kd})
      if (!std::isfinite(value) || value <= 0.)
        throw std::invalid_argument("start heading parameters must be positive and finite");
    heading_.reset(new StartHeading(rate, tolerance, settle, timeout, kp, kd));
    nh.param("goal_approach_decel", approach_decel_, .60);
    nh.param("goal_approach_response", approach_response_, .25);
    if (!std::isfinite(approach_decel_) || approach_decel_ <= 0. ||
        !std::isfinite(approach_response_) || approach_response_ < 0.)
      throw std::invalid_argument("invalid goal approach deceleration or response time");
    std::string odom_topic;
    nh.param<std::string>("odom_topic", odom_topic, "odom");
    ros::NodeHandle node;
    odometry_ = node.subscribe(odom_topic, 1, &HeadingDWAPlanner::onOdom, this);
    delegate_.initialize(name, tf, costmap);
    local_plan_preview_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
    ros::NodeHandle move_base("~");
    goals_ = move_base.subscribe("goal", 5, &HeadingDWAPlanner::onGoal, this);
  }

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override {
    if (plan.empty() || !delegate_.setPlan(plan)) return false;
    const auto& target = plan.back();
    const auto epoch = goal_epoch_.load();
    if (!has_goal_ || epoch != plan_epoch_ || differentGoal(target)) {
      goal_ = target;
      has_goal_ = true;
      plan_epoch_ = epoch;
      aimed_ = aligned_ = false;
      last_drive_ = geometry_msgs::Twist();
      last_forward_output_ = 0.;
    }
    if (!aimed_) start_path_ = plan;
    return true;
  }

  bool computeVelocityCommands(geometry_msgs::Twist& command) override {
    if (aligned_) return cruise(command);
    command = geometry_msgs::Twist();
    geometry_msgs::PoseStamped robot;
    if (!has_goal_ || !costmap_->getRobotPose(robot)) return false;

    const auto now = ros::WallTime::now().toSec();
    if (!aimed_) {
      const double distance = distanceToGoal(robot);
      if (!std::isfinite(distance)) return false;
      if (distance <= min_distance_) {
        aligned_ = true;
        return cruise(command);
      }
      std::vector<geometry_msgs::PoseStamped> local_path;
      if (!base_local_planner::transformGlobalPlan(*tf_, start_path_, robot,
              *costmap_->getCostmap(), costmap_->getGlobalFrameID(), local_path)) return false;
      bearing_ = pathStartTangent(local_path.begin(), local_path.end(), [](const auto& pose) {
          return std::make_pair(pose.pose.position.x, pose.pose.position.y);
        }, lookahead_);
      if (!std::isfinite(bearing_)) {
        ROS_WARN_THROTTLE(2., "RTK start heading needs a local path tangent");
        return false;
      }
      nav_msgs::Path preview;
      preview.header = robot.header;
      preview.poses = std::move(local_path);
      local_plan_preview_.publish(preview);
      ROS_INFO("RTK start heading: current %.3f, local tangent %.3f, error %.3f rad",
               tf2::getYaw(robot.pose.orientation), bearing_,
               angles::shortest_angular_distance(tf2::getYaw(robot.pose.orientation), bearing_));
      heading_->start(now);
      aimed_ = true;
    }

    const auto odom = freshOdometry();
    if (!odom) {
      ROS_WARN_THROTTLE(2., "RTK start heading needs fresh odometry");
      return false;
    }
    const auto& twist = odom->twist.twist;
    const bool stopped = std::hypot(twist.linear.x, twist.linear.y) <= .025 &&
                         std::abs(twist.angular.z) <= .06;
    const double error = angles::shortest_angular_distance(tf2::getYaw(robot.pose.orientation), bearing_);
    if (!std::isfinite(error) || !std::isfinite(twist.angular.z)) return false;
    const auto step = heading_->update(error, twist.angular.z, stopped, now);
    if (step.failed) {
      ROS_WARN_THROTTLE(2., "RTK initial turn did not settle within its time limit");
      return false;
    }
    command.angular.z = step.angular;
    aligned_ = step.ready;
    return true;
  }

  bool isGoalReached() override {
    if (!aligned_ || !delegate_.isGoalReached()) return false;
    has_goal_ = false;
    return true;
  }

 private:
  nav_msgs::Odometry::ConstPtr freshOdometry() {
    std::lock_guard<std::mutex> lock(odom_lock_);
    if (!last_odom_ || last_odom_->header.stamp.isZero()) return {};
    const double age = (ros::Time::now() - last_odom_->header.stamp).toSec();
    return age >= 0. && age <= .5 ? last_odom_ : nav_msgs::Odometry::ConstPtr{};
  }

  double distanceToGoal(const geometry_msgs::PoseStamped& robot) const {
    auto target = goal_;
    if (target.header.frame_id != robot.header.frame_id) {
      target.header.stamp = ros::Time(0);
      try {
        target = tf_->transform(target, robot.header.frame_id, ros::Duration(.02));
      } catch (const tf2::TransformException& error) {
        ROS_WARN_THROTTLE(2., "RTK navigation needs goal TF: %s", error.what());
        return std::numeric_limits<double>::quiet_NaN();
      }
    }
    return std::hypot(target.pose.position.x - robot.pose.position.x,
                      target.pose.position.y - robot.pose.position.y);
  }

  bool cruise(geometry_msgs::Twist& command) {
    const bool planned = delegate_.computeVelocityCommands(command);
    const auto now = ros::WallTime::now();
    if (!planned) {
      // Brake on the last arc through a local planning miss. Each command
      // still goes through the Guard, and the bridge expires at zero speed.
      if (last_drive_.linear.x <= 0.) return false;
      constexpr double braking_rate = 1.0;
      constexpr double maximum_duration = .8;
      const double elapsed = (now - last_drive_at_).toSec();
      const double initial_speed = std::min(last_drive_.linear.x, braking_rate * maximum_duration);
      const double braking_speed = initial_speed - braking_rate * elapsed;
      if (elapsed <= 0. || braking_speed <= 0.) return false;
      const auto odom = freshOdometry();
      if (!odom) return false;
      const double measured = odom->twist.twist.linear.x;
      if (!std::isfinite(measured)) return false;
      const double speed = std::min({braking_speed, measured, last_forward_output_});
      if (speed <= 0.) return false;
      command = geometry_msgs::Twist();
      command.linear.x = speed;
      command.angular.z = last_drive_.angular.z * speed / last_drive_.linear.x;
    }
    if (command.linear.x <= 0.) {
      last_drive_ = command;
      last_forward_output_ = 0.;
      return true;
    }
    geometry_msgs::PoseStamped robot;
    if (!costmap_->getRobotPose(robot)) return false;
    const double distance = distanceToGoal(robot);
    if (!std::isfinite(distance)) return false;

    // Solve distance = speed * response + speed² / (2 * deceleration).
    // Reduce both components to keep the collision-checked arc's curvature.
    const double margin = approach_decel_ * approach_response_;
    const double cap = std::sqrt(margin * margin + 2. * approach_decel_ * distance) - margin;
    if (command.linear.x > cap) {
      command.angular.z *= cap / command.linear.x;
      command.linear.x = cap;
    }
    last_forward_output_ = command.linear.x;
    if (planned) {
      last_drive_ = command;
      last_drive_at_ = now;
    } else {
      ROS_INFO_THROTTLE(1., "RTK DWA planning gap: braking on the last arc");
    }
    return true;
  }

  bool differentGoal(const geometry_msgs::PoseStamped& target) const {
    return target.header.frame_id != goal_.header.frame_id ||
        std::hypot(target.pose.position.x - goal_.pose.position.x,
                   target.pose.position.y - goal_.pose.position.y) > 1e-6 ||
        std::abs(angles::shortest_angular_distance(tf2::getYaw(target.pose.orientation),
                                                   tf2::getYaw(goal_.pose.orientation))) > 1e-6;
  }

  void onGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr&) { ++goal_epoch_; }

  void onOdom(const nav_msgs::Odometry::ConstPtr& sample) {
    std::lock_guard<std::mutex> lock(odom_lock_);
    last_odom_ = sample;
  }

  dwa_local_planner::DWAPlannerROS delegate_;
  std::mutex odom_lock_;
  nav_msgs::Odometry::ConstPtr last_odom_;
  std::unique_ptr<StartHeading> heading_;
  ros::Subscriber goals_, odometry_;
  ros::Publisher local_plan_preview_;
  tf2_ros::Buffer* tf_ = nullptr;
  costmap_2d::Costmap2DROS* costmap_ = nullptr;
  geometry_msgs::PoseStamped goal_;
  geometry_msgs::Twist last_drive_;
  ros::WallTime last_drive_at_;
  double last_forward_output_ = 0.;

  std::vector<geometry_msgs::PoseStamped> start_path_;
  std::atomic<unsigned> goal_epoch_{0};
  unsigned plan_epoch_ = 0;
  double min_distance_ = .2, lookahead_ = .2, bearing_ = 0.;
  double approach_decel_ = .60, approach_response_ = .25;
  bool has_goal_ = false, aimed_ = false, aligned_ = false;
};

}  // namespace slicer_navigation

PLUGINLIB_EXPORT_CLASS(slicer_navigation::HeadingDWAPlanner, nav_core::BaseLocalPlanner)
