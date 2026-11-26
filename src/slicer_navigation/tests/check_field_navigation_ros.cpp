#include <cmath>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

geometry_msgs::PoseStamped at(double x, double y, double yaw=0.) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0., 0., yaw);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

void configureCostmap(ros::NodeHandle& node, const std::string& name) {
  node.setParam(name + "/global_frame", "map");
  node.setParam(name + "/robot_base_frame", "base_link");
  node.setParam(name + "/update_frequency", 0.);
  node.setParam(name + "/publish_frequency", 0.);
  node.setParam(name + "/transform_tolerance", .7);
  node.setParam(name + "/robot_radius", .20);
  node.setParam(name + "/plugins", std::vector<std::string>{});
}

void fill(costmap_2d::Costmap2D* grid, int expensive_side = 0) {
  for (unsigned y = 0; y < grid->getSizeInCellsY(); ++y)
    for (unsigned x = 0; x < grid->getSizeInCellsX(); ++x) {
      double px, py;
      grid->mapToWorld(x, y, px, py);
      unsigned char value = costmap_2d::FREE_SPACE;
      if (expensive_side && std::abs(px) < .75 && std::abs(py) < .8)
        value = costmap_2d::LETHAL_OBSTACLE;
      else if (expensive_side && std::abs(px) < 1.5 && py*expensive_side > .8)
        value = 205;
      grid->setCost(x, y, value);
    }
}

int routeSide(const nav_msgs::Path& path) {
  for (const auto& pose : path.poses) {
    const auto& p = pose.pose.position;
    if (std::abs(p.x) < .5 && std::abs(p.y) > .5) return p.y > 0. ? 1 : -1;
  }
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rtk_field_nav_probe");
  ros::NodeHandle node, local("~");
  bool isolated = false;
  node.getParam("/rtk_field_test_isolated", isolated);
  require(isolated, "this test needs its own ROS master");
  configureCostmap(local, "local");
  configureCostmap(local, "global");
  local.setParam("dwa/odom_topic", "/field_test/odom");
  local.setParam("dwa/start_commitment", true);
  local.setParam("dwa/xy_goal_tolerance", .20);
  local.setParam("dwa/yaw_goal_tolerance", .30);
  local.setParam("GlobalPlanner/continuity_cost/enabled", true);
  local.setParam("GlobalPlanner/continuity_cost/weight", 0.);
  local.setParam("GlobalPlanner/continuity_cost/obstacle_weight", .70);
  local.setParam("GlobalPlanner/task_side_hysteresis", true);
  tf2_ros::Buffer tf;
  tf.setUsingDedicatedThread(true);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  const auto odom = node.advertise<nav_msgs::Odometry>("/field_test/odom", 1, true);
  const auto grid_pub = local.advertise<nav_msgs::OccupancyGrid>("global_costmap/costmap", 1, true);
  const auto goal_pub = local.advertise<geometry_msgs::PoseStamped>("current_goal", 1);
  nav_msgs::OccupancyGrid observed;
  observed.info.width = observed.info.height = 200;
  observed.info.resolution = .05;
  observed.info.origin.position.x = observed.info.origin.position.y = -5.;
  observed.info.origin.orientation.w = 1.;
  observed.data.resize(200*200);
  for (unsigned y=0; y<200; ++y)
    for (unsigned x=160; x<200; ++x) observed.data[y*200+x] = -1;
  const auto pose = [&](double x, double y=0.) {
    const auto p = at(x, y);
    geometry_msgs::TransformStamped t;
    t.header = p.header;
    t.child_frame_id = "base_link";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation = p.pose.orientation;
    tf.setTransform(t, "field_fixture", false);
    nav_msgs::Odometry m;
    m.header = p.header;
    m.child_frame_id = t.child_frame_id;
    m.pose.pose = p.pose;
    odom.publish(m);
    observed.header = p.header;
    grid_pub.publish(observed);
    ros::WallDuration(.10).sleep();
  };
  pose(0.);
  costmap_2d::Costmap2DROS local_grid("local", tf), global_grid("global", tf);
  for (auto* map : {&local_grid, &global_grid}) {
    map->stop();
    map->getCostmap()->resizeMap(200, 200, .05, -5., -5.);
    fill(map->getCostmap());
  }
  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> local_plugins("nav_core", "nav_core::BaseLocalPlanner");
  auto dwa = local_plugins.createInstance("slicer_navigation/DWAPlannerROS");
  dwa->initialize("dwa", &tf, &local_grid);
  std::mutex lock;
  nav_msgs::Path commitment, raw;
  const auto path_sub = local.subscribe<nav_msgs::Path>("dwa/commitment_path", 1,
      [&](const nav_msgs::Path::ConstPtr& m) { std::lock_guard<std::mutex> guard(lock); commitment = *m; });
  const auto raw_sub = local.subscribe<nav_msgs::Path>("GlobalPlanner/plan_raw", 1,
      [&](const nav_msgs::Path::ConstPtr& m) { std::lock_guard<std::mutex> guard(lock); raw = *m; });
  pose(0.);
  const std::vector<geometry_msgs::PoseStamped> first{at(0.,0.), at(1.35,0.), at(4.,0.)};
  const std::vector<geometry_msgs::PoseStamped> opposite{at(0.,0.), at(2.,-1.), first.back()};
  geometry_msgs::Twist command;
  require(dwa->setPlan(first) && dwa->computeVelocityCommands(command), "stationary robot acquires known path");
  pose(0.);
  dwa->computeVelocityCommands(command);
  require(dwa->setPlan(opposite), "background candidate accepted into pending slot");
  pose(1.6);
  dwa->computeVelocityCommands(command);
  ros::WallDuration(.15).sleep();
  {
    std::lock_guard<std::mutex> guard(lock);
    require(commitment.poses.size() > 10, "planned the protected 1.2 m prefix");
    for (const auto& p : commitment.poses)
      require(std::abs(p.pose.position.y) < .02, "pending route must not replace committed y=0 route");
  }
  pose(1.8);
  dwa->computeVelocityCommands(command);
  ros::WallDuration(.15).sleep();
  {
    std::lock_guard<std::mutex> guard(lock);
    require(commitment.poses.empty(), "anchor radius releases commitment");
  }
  std::cout << "PASS protected route stays fixed at x=1.6 and hands over at x=1.8\n";

  const auto guarded = node.advertise<std_msgs::Time>("/slicer_nav_arc_guard/commitment_stop", 1);
  pose(0.);
  require(dwa->setPlan({at(0.,0.), at(1.35,0.), at(3.9,0.)}), "new commitment test plan");
  require(dwa->computeVelocityCommands(command), "new commitment acquired");
  auto begin_stop = ros::WallTime();
  bool cleared = false;
  for (int i = 0; i < 32; ++i) {
    pose(0.);
    std_msgs::Time signal;
    {
      std::lock_guard<std::mutex> guard(lock);
      signal.data = commitment.header.stamp;
    }
    if (!signal.data.isZero()) {
      if (begin_stop.isZero()) begin_stop = ros::WallTime::now();
      guarded.publish(signal);
      ros::WallDuration(.025).sleep();
    } else if (!begin_stop.isZero()) {
      const double duration = (ros::WallTime::now() - begin_stop).toSec();
      require(duration > 2. && duration < 2.6, "persistent hard stop releases only after two seconds");
      std::cout << "PASS only sustained Guard hard-stop feedback releases commitment at "
                << duration << " s\n";
      cleared = true;
      break;
    }
    dwa->computeVelocityCommands(command);
    if (!begin_stop.isZero())
      require(command.linear.x == 0. && command.angular.z == 0., "confirmed Guard stop remains zero");
  }
  require(cleared, "continuous hard stop must hand over the route");

  pose(1.8);
  goal_pub.publish(at(1.98,0.,.25));
  ros::WallDuration(.15).sleep();
  require(dwa->computeVelocityCommands(command) && command.linear.x == 0. && command.angular.z == 0.,
          "accepted goal suppresses old commands before the new plan");
  require(dwa->setPlan({at(1.8,0.), at(1.98,0.,.25)}), "near goal accepted");
  require(dwa->computeVelocityCommands(command) && dwa->isGoalReached(),
          "0.18 m and 0.25 rad qualify in one control cycle");
  std::cout << "PASS installed DWA 0.20 m / 0.30 rad final tolerance\n";

  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> global_plugins("nav_core", "nav_core::BaseGlobalPlanner");
  auto planner = global_plugins.createInstance("slicer_navigation/BSplineGlobalPlanner");
  planner->initialize("GlobalPlanner", &global_grid);
  for (int i = 0; i < 25 && raw_sub.getNumPublishers() == 0; ++i) ros::WallDuration(.10).sleep();
  require(raw_sub.getNumPublishers() == 1, "raw path ROS subscriber is connected");
  ros::WallDuration(.20).sleep();
  const auto start = at(-2.5, 0.), goal = at(2.5, 0.);
  std::vector<geometry_msgs::PoseStamped> plan;
  for (int expensive : {1, -1, 1, -1}) {
    fill(global_grid.getCostmap(), expensive);
    require(planner->makePlan(start, goal, plan), "Dijkstra found an obstacle bypass");
    nav_msgs::Path accepted;
    accepted.poses = plan;
    ros::WallDuration(.15).sleep();
    std::lock_guard<std::mutex> guard(lock);
    std::cout << "Raw side " << routeSide(raw) << " accepted side " << routeSide(accepted)
              << " expensive side " << expensive << " raw samples " << raw.poses.size() << std::endl;
    require(routeSide(raw) == -expensive, "synthetic map must alternate true raw planner preference");
    require(routeSide(accepted) == -1, "a feasible equal-length bypass keeps the originally accepted side");
  }
  std::cout << "PASS real Dijkstra alternates raw preference while task side remains RIGHT\n";
}
