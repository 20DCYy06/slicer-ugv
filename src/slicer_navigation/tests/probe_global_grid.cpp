// Read a captured OccupancyGrid and exercise the installed planner without a robot.
#include <ros/ros.h>
#include <global_planner/planner_core.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>

int main(int argc, char** argv) {
  ros::init(argc, argv, "slicer_global_grid_probe");
  if (argc < 2) return 2;
  std::ifstream input(argv[1]);
  unsigned width, height;
  double resolution, ox, oy, sx, sy, sa, gx, gy, ga;
  if (!(input >> width >> height >> resolution >> ox >> oy >> sx >> sy >> sa >> gx >> gy >> ga)) return 2;
  costmap_2d::Costmap2D map(width, height, resolution, ox, oy, 0);
  for (unsigned y=0; y<height; ++y) for (unsigned x=0; x<width; ++x) {
    int value;
    if (!(input >> value)) return 2;
    // Inverse of Costmap2DPublisher's lossy OccupancyGrid translation.
    unsigned char cost = value < 0 ? 255 : value == 100 ? 254 : value == 99 ? 253 :
                         value == 0 ? 0 : std::min(252, 1 + (value-1)*251/97);
    map.setCost(x, y, cost);
  }
  geometry_msgs::PoseStamped start, goal;
  start.header.frame_id = goal.header.frame_id = "map";
  start.pose.position.x = sx; start.pose.position.y = sy;
  goal.pose.position.x = gx; goal.pose.position.y = gy;
  start.pose.orientation.z = std::sin(sa/2); start.pose.orientation.w = std::cos(sa/2);
  goal.pose.orientation.z = std::sin(ga/2); goal.pose.orientation.w = std::cos(ga/2);
  ros::NodeHandle nh("~");
  for (int mode=0; mode<4; ++mode) {
    std::string name = "probe_" + std::to_string(mode);
    nh.setParam(name+"/use_dijkstra", true);
    nh.setParam(name+"/use_grid_path", true);
    nh.setParam(name+"/allow_unknown", true);
    nh.setParam(name+"/old_navfn_behavior", bool(mode & 1));
    nh.setParam(name+"/use_quadratic", !(mode & 2));
    nh.setParam(name+"/cost_factor", 3.0);
    nh.setParam(name+"/lethal_cost", 253);
    nh.setParam(name+"/neutral_cost", 50);
    costmap_2d::Costmap2D copy(map);
    global_planner::GlobalPlanner planner(name, &copy, "map");
    std::vector<geometry_msgs::PoseStamped> path;
    auto before = ros::WallTime::now();
    bool ok = planner.makePlan(start, goal, path);
    std::cout << "GRID_PROBE old_seed=" << bool(mode & 1) << " quadratic=" << !(mode & 2)
              << " ok=" << ok << " poses=" << path.size()
              << " seconds=" << (ros::WallTime::now()-before).toSec() << std::endl;
  }
  return 0;
}
