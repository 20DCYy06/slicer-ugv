#include <slicer_navigation/start_heading.h>
#include <slicer_navigation/field_commitment.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <angles/angles.h>
#include <base_local_planner/goal_functions.h>
#include <costmap_2d/cost_values.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Time.h>
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
    nh.param("start_heading_lookahead", lookahead_, 1.35);
    nh.param("start_heading_kp", kp, 1.6);
    nh.param("start_heading_kd", kd, .25);
    for (double value : {rate, tolerance, settle, timeout, min_distance_, lookahead_, kp, kd})
      if (!std::isfinite(value) || value <= 0.)
        throw std::invalid_argument("start heading parameters must be positive and finite");
    if (lookahead_ < 1.2 || lookahead_ > 1.5 || tolerance >= 10.*kHeadingPi/180.)
      throw std::invalid_argument("start heading expects 1.2–1.5 m lookahead and tolerance below 10 degrees");
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
    nh.param("start_commitment", commitment_enabled_, false);
    if (commitment_enabled_) {
      commitment_path_ = nh.advertise<nav_msgs::Path>("commitment_path", 1);
      global_grid_sub_ = ros::NodeHandle("~").subscribe("global_costmap/costmap", 1, &HeadingDWAPlanner::onGlobalGrid, this);
      guard_stop_sub_ = node.subscribe("/slicer_nav_arc_guard/commitment_stop", 1, &HeadingDWAPlanner::onGuardStop, this);
    }
    local_plan_preview_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
    ros::NodeHandle move_base("~");
    goals_ = move_base.subscribe("current_goal", 5, &HeadingDWAPlanner::onGoal, this);
  }

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override {
    if (plan.empty()) return false;
    const auto& target = plan.back();
    const auto epoch = goal_epoch_.load();
    if (!has_goal_ || epoch != plan_epoch_ || differentGoalPosition(target)) {
      goal_ = target;
      has_goal_ = true;
      plan_epoch_ = epoch;
      clearAlignment();
    }
    if (aligned_ && !heading_lock_.active()) return delegate_.setPlan(plan);
    // Keep the first accepted path in DWA through alignment and initial travel.
    // Save only the newest replan for the handoff.
    pending_plan_ = plan;
    return true;
  }

  bool computeVelocityCommands(geometry_msgs::Twist& command) override {
    command = geometry_msgs::Twist();
    if (has_goal_ && plan_epoch_ != goal_epoch_.load()) return true;
    if (aligned_ && !heading_lock_.active()) return cruise(command);
    geometry_msgs::PoseStamped robot;
    if (!has_goal_ || !costmap_->getRobotPose(robot)) return false;

    const auto now = ros::WallTime::now().toSec();
    if (!heading_lock_.active()) {
      if (pending_plan_.empty()) return false;
      const double distance = distanceToGoal(robot);
      if (!std::isfinite(distance)) return false;
      if (distance <= min_distance_) {
        if (!usePendingPlan()) return false;
        aligned_ = true;
        return cruise(command);
      }
      if (!acquireHeading(robot, now)) return false;
    }

    const auto odom = freshOdometry();
    if (!odom) {
      ROS_WARN_THROTTLE(2., "RTK start heading needs fresh odometry");
      return false;
    }
    const auto& odom_pose = odom->pose.pose;
    const HeadingPoint odom_position{odom_pose.position.x, odom_pose.position.y};
    const bool finite_position = std::isfinite(odom_position.first) && std::isfinite(odom_position.second);
    if (aligned_) {
      if (!finite_position || odom->header.frame_id != forward_odom_frame_) return false;
      if (commitment_enabled_) return committedCruise(robot, command);
      if (heading_lock_.travelledForward(odom_position, .5)) {
        if (!usePendingPlan()) return false;
        heading_lock_.clear();
        ROS_INFO("RTK start heading released after 0.50 m of forward odometry");
        return cruise(command);
      }
    }
    if (!commitment_enabled_ && hardObstacleAhead(robot, heading_lock_.yaw())) {
      ROS_WARN("RTK start heading released for a hard obstacle within 1.50 m; waiting for a new plan");
      clearAlignment();
      return true;  // Publish zero before any replacement alignment.
    }
    if (aligned_) return cruise(command);

    const auto& twist = odom->twist.twist;
    const bool stopped = std::hypot(twist.linear.x, twist.linear.y) <= .025 &&
                         std::abs(twist.angular.z) <= .06;
    const double error = angles::shortest_angular_distance(tf2::getYaw(robot.pose.orientation), heading_lock_.yaw());
    if (!std::isfinite(error) || !std::isfinite(twist.angular.z)) return false;
    const auto step = heading_->update(error, twist.angular.z, stopped, now);
    if (step.failed) {
      ROS_WARN_THROTTLE(2., "RTK initial turn did not settle within its time limit");
      return false;
    }
    command.angular.z = step.angular;
    if (step.ready) {
      const double odom_yaw = tf2::getYaw(odom_pose.orientation);
      if (odom->header.frame_id.empty() || !std::isfinite(odom_yaw) || !finite_position) return false;
      if (commitment_enabled_ && !chooseCommitmentAnchor(robot)) return true;
      heading_lock_.beginForward(odom_position, odom_yaw);
      forward_odom_frame_ = odom->header.frame_id;
      aligned_ = true;
    }
    return true;
  }

  bool isGoalReached() override {
    if (plan_epoch_ != goal_epoch_.load() || !aligned_ || !delegate_.isGoalReached()) return false;
    has_goal_ = false;
    clearAlignment();
    return true;
  }

 private:
  bool usePendingPlan() {
    if (pending_plan_.empty()) return true;
    if (!delegate_.setPlan(pending_plan_)) return false;
    pending_plan_.clear();
    return true;
  }

  void clearAlignment() {
    clearCommitment();
    heading_lock_.clear();
    aligned_ = false;
    pending_plan_.clear();
    last_drive_ = geometry_msgs::Twist();
    last_forward_output_ = 0.;
  }

  bool acquireHeading(const geometry_msgs::PoseStamped& robot, double now) {
    std::vector<geometry_msgs::PoseStamped> local_path;
    if (!base_local_planner::transformGlobalPlan(*tf_, pending_plan_, robot, *costmap_->getCostmap(),
            costmap_->getGlobalFrameID(), local_path)) return false;
    const auto& position = robot.pose.position;
    const double yaw = pathLookaheadChord(local_path.begin(), local_path.end(), [](const auto& pose) {
        return HeadingPoint{pose.pose.position.x, pose.pose.position.y};
      }, {position.x, position.y}, lookahead_);
    if (!std::isfinite(yaw)) {
      ROS_WARN_THROTTLE(2., "RTK start heading needs a local lookahead chord");
      return false;
    }
    if (hardObstacleAhead(robot, yaw)) {
      ROS_WARN_THROTTLE(2., "RTK start heading candidate has a hard obstacle within 1.50 m");
      return false;
    }
    if (commitment_enabled_) locked_plan_ = pending_plan_;
    if (!usePendingPlan()) return false;
    heading_lock_.acquire(yaw);
    heading_->start(now);
    nav_msgs::Path preview;
    preview.header = robot.header;
    preview.poses = std::move(local_path);
    local_plan_preview_.publish(preview);
    const double robot_yaw = tf2::getYaw(robot.pose.orientation);
    ROS_INFO("RTK start heading locked: current %.3f, %.2f m chord %.3f, error %.3f rad",
             robot_yaw, lookahead_, yaw, angles::shortest_angular_distance(robot_yaw, yaw));
    return true;
  }

  void onGlobalGrid(const nav_msgs::OccupancyGrid::ConstPtr& grid) {
    std::lock_guard<std::mutex> lock(commitment_lock_);
    global_grid_ = grid;
  }

  void onGuardStop(const std_msgs::Time::ConstPtr& reply) {
    std::lock_guard<std::mutex> lock(commitment_lock_);
    guard_stopped_path_ = reply->data;
  }

  void clearCommitment() {
    if (!commitment_enabled_) return;
    if (!commitment_started_.isZero()) commitment_path_.publish(nav_msgs::Path{});
    commitment_started_ = ros::Time();
    blocked_since_ = ros::WallTime();
    locked_plan_.clear();
  }

  geometry_msgs::PoseStamped latestTransform(geometry_msgs::PoseStamped pose,
                                              const std::string& frame) const {
    pose.header.stamp = ros::Time();
    return pose.header.frame_id == frame ? pose : tf_->transform(pose, frame, ros::Duration(.02));
  }

  bool chooseCommitmentAnchor(const geometry_msgs::PoseStamped& robot) {
    nav_msgs::OccupancyGrid::ConstPtr grid;
    {
      std::lock_guard<std::mutex> lock(commitment_lock_);
      grid = global_grid_;
    }
    const auto& frame = locked_plan_.front().header.frame_id;
    const auto age = grid ? (ros::Time::now() - grid->header.stamp).toSec() : 10.;
    if (!grid || grid->header.frame_id != frame || grid->header.stamp.isZero() ||
        age < -.1 || age > 1.2 || !std::isfinite(grid->info.resolution) || grid->info.resolution <= 0. ||
        grid->data.size() != static_cast<std::size_t>(grid->info.width) * grid->info.height) {
      ROS_WARN_THROTTLE(2., "RTK first segment needs a current global occupancy grid");
      return false;
    }
    try {
      const auto pose = latestTransform(robot, frame);
      const Point2 position{pose.pose.position.x, pose.pose.position.y};
      std::vector<Point2> route;
      for (const auto& p : locked_plan_) route.emplace_back(p.pose.position.x, p.pose.position.y);
      route = remainingRoute(route, position, .75);
      route.insert(route.begin(), position);
      const auto& origin = grid->info.origin.position;
      const auto sam = routeSamples(route, grid->info.resolution / 2., routeLength(route));
      const auto known = [&](Point2 p) {
        const double x = std::floor((p.x - origin.x)/grid->info.resolution);
        const double y = std::floor((p.y - origin.y)/grid->info.resolution);
        if (!(x >= 0. && y >= 0. && x < grid->info.width && y < grid->info.height)) return false;
        const int cost = grid->data[static_cast<unsigned>(y)*grid->info.width + static_cast<unsigned>(x)];
        return cost >= 0 && cost < 100;
      };
      const auto anchor = commitmentAnchor(sam, known, .5);
      anchor_ = pose;
      anchor_.pose.position.x = anchor.x;
      anchor_.pose.position.y = anchor.y;
      commitment_started_ = ros::Time::now();
      ROS_INFO("RTK commitment anchor %.2f %.2f in %s", anchor.x, anchor.y, frame.c_str());
      return true;
    } catch (const tf2::TransformException&) {
      ROS_WARN_THROTTLE(2., "RTK commitment anchor needs current global TF");
      return false;
    }
  }

  bool committedCruise(const geometry_msgs::PoseStamped& robot, geometry_msgs::Twist& command) {
    try {
      const auto anchor = latestTransform(anchor_, robot.header.frame_id);
      const Point2 position{robot.pose.position.x, robot.pose.position.y};
      if (norm(position - Point2{anchor.pose.position.x, anchor.pose.position.y}) < .8) {
        if (!usePendingPlan()) return false;
        clearCommitment();
        heading_lock_.clear();
        ROS_INFO("RTK commitment anchor reached; latest route accepted");
        return cruise(command);
      }
      const auto path_tf = tf_->lookupTransform(robot.header.frame_id, locked_plan_.front().header.frame_id,
                                                 ros::Time(), ros::Duration(.02));
      std::vector<Point2> route;
      for (const auto& p : locked_plan_) {
        geometry_msgs::PoseStamped transformed;
        tf2::doTransform(p, transformed, path_tf);
        route.emplace_back(transformed.pose.position.x, transformed.pose.position.y);
      }
      route = remainingRoute(route, position, .75);
      if (route.size() < 2) {
        blocked_since_ = ros::WallTime();
        return true;
      }
      route.insert(route.begin(), position);
      nav_msgs::Path path;
      path.header = robot.header;
      path.header.stamp = ros::Time::now();
      const auto samples = routeSamples(route, .05, 1.2);
      double yaw = tf2::getYaw(robot.pose.orientation);
      for (std::size_t i = 0; i < samples.size(); ++i) {
        if (i+1 < samples.size()) {
          const auto d = samples[i+1] - samples[i];
          if (norm(d) > 1e-6) yaw = std::atan2(d.y, d.x);
        }
        auto p = robot;
        p.header = path.header;
        p.pose.position.x = samples[i].x;
        p.pose.position.y = samples[i].y;
        tf2::Quaternion q;
        q.setRPY(0., 0., yaw);
        p.pose.orientation = tf2::toMsg(q);
        path.poses.push_back(p);
      }
      commitment_path_.publish(path);
      ros::Time stop;
      {
        std::lock_guard<std::mutex> lock(commitment_lock_);
        stop = guard_stopped_path_;
      }
      const double stop_age = (path.header.stamp - stop).toSec();
      if (stop >= commitment_started_ && stop_age >= 0. && stop_age <= .35) {
        const ros::WallTime now = ros::WallTime::now();
        if (blocked_since_.isZero()) blocked_since_ = now;
        if ((now - blocked_since_).toSec() > 2.) {
          ROS_WARN("RTK commitment released after persistent Guard hard stop");
          clearAlignment();
        }
        return true;
      }
      blocked_since_ = ros::WallTime();
      return cruise(command);
    } catch (const tf2::TransformException&) {
      blocked_since_ = ros::WallTime();
      ROS_WARN_THROTTLE(2., "RTK committed route needs current TF");
      return true;
    }
  }

  bool hardObstacleAhead(const geometry_msgs::PoseStamped& robot, double yaw) const {
    auto* grid = costmap_->getCostmap();
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
    const auto& position = robot.pose.position;
    return headingCorridorBlocked({position.x, position.y}, yaw, 1.5, grid->getResolution() / 2.,
        [&](double x, double y) {
          unsigned mx, my;
          if (!grid->worldToMap(x, y, mx, my)) return false;
          const auto cost = grid->getCost(mx, my);
          // Unknown and ordinary inflation never pick a different turn.
          // The downstream Guard still checks the actual footprint sweep.
          return cost == costmap_2d::LETHAL_OBSTACLE ||
                 cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        });
  }

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

  bool differentGoalPosition(const geometry_msgs::PoseStamped& target) const {
    return target.header.frame_id != goal_.header.frame_id ||
        std::hypot(target.pose.position.x - goal_.pose.position.x,
                   target.pose.position.y - goal_.pose.position.y) > 1e-6;
  }

  void onGoal(const geometry_msgs::PoseStamped::ConstPtr&) { ++goal_epoch_; }

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

  AlignmentHeadingLock heading_lock_;
  std::vector<geometry_msgs::PoseStamped> pending_plan_;
  bool commitment_enabled_ = false;
  std::mutex commitment_lock_;
  ros::Subscriber global_grid_sub_, guard_stop_sub_;
  ros::Publisher commitment_path_;
  nav_msgs::OccupancyGrid::ConstPtr global_grid_;
  geometry_msgs::PoseStamped anchor_;
  ros::Time commitment_started_, guard_stopped_path_;
  ros::WallTime blocked_since_;
  std::vector<geometry_msgs::PoseStamped> locked_plan_;
  std::string forward_odom_frame_;
  std::atomic<unsigned> goal_epoch_{0};
  unsigned plan_epoch_ = 0;
  double min_distance_ = .2, lookahead_ = 1.35;
  double approach_decel_ = .60, approach_response_ = .25;
  bool has_goal_ = false, aligned_ = false;
};

}  // namespace slicer_navigation

PLUGINLIB_EXPORT_CLASS(slicer_navigation::HeadingDWAPlanner, nav_core::BaseLocalPlanner)
