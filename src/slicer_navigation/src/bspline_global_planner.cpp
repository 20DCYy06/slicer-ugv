#include <slicer_navigation/bspline_core.h>
#include <slicer_navigation/field_commitment.h>
#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/thread/locks.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#include <sstream>

namespace slicer_navigation {
// move_base直接把makePlan返回值交给TEB::setPlan。话题只作显示，绝不假装中继能替换参考轨迹。
class BSplineGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  void initialize(std::string name,costmap_2d::Costmap2DROS* costmap) override {
    if(raw_) return;
    costmap_ros_=costmap;
    ros::NodeHandle nh("~/GlobalPlanner"),raw_nh("~/GlobalPlannerRaw");
    XmlRpc::XmlRpcValue parameters;
    if(nh.getParam("",parameters)) raw_nh.setParam("",parameters);
    nh.param("smoothing/enabled",enabled_,true);
    nh.param("smoothing/rdp_epsilon",epsilon_,.03);
    nh.param("smoothing/control_point_spacing",control_spacing_,.20);
    nh.param("smoothing/sample_spacing",sample_spacing_,.05);
    nh.param("smoothing/max_refinements",refinements_,2);
    nh.param("smoothing/max_cost",max_cost_,252);
    nh.param("smoothing/observed_prefix", smooth_observed_prefix_, false);
    nh.param("continuity_cost/enabled", cost_field_enabled_, false);
    nh.param("allow_unknown", allow_unknown_, true);
    nh.param("continuity_cost/weight", cost_options_.weight, 4.);
    nh.param("continuity_cost/decay_length_m", cost_options_.decay_length_m, 2.);
    nh.param("continuity_cost/obstacle_weight", cost_options_.obstacle_weight, .70);
    nh.param("continuity_cost/obstacle_soft_radius_m", cost_options_.obstacle_soft_radius_m, .60);
    nh.param("continuity_cost/deviation_cap_m", cost_options_.deviation_cap_m, 1.);
    nh.param("continuity_cost/min_clearance_m", cost_options_.min_clearance_m, .23);
    nh.param("continuity_cost/rejoin_distance_m", cost_options_.rejoin_distance_m, .30);
    nh.param("task_side_hysteresis", side_enabled_, false);
    if(!std::isfinite(epsilon_) || epsilon_<0. || !std::isfinite(control_spacing_) ||
       control_spacing_<=0. || !std::isfinite(sample_spacing_) || sample_spacing_<=0. ||
       refinements_<0 || max_cost_<0 || max_cost_>252 ||
       (cost_field_enabled_ && !validContinuityOptions(cost_options_)))
      throw std::invalid_argument("invalid GlobalPlanner smoothing or continuity cost configuration");
    raw_.reset(new global_planner::GlobalPlanner());
    if (cost_field_enabled_) {
      // Noetic's Dijkstra reserves < lethal_cost-1 for known traversable
      // cells and adds neutral_cost + cost_factor*byte to the potential.
      const auto configure = [](ros::NodeHandle& planner, bool grid_traceback) {
        planner.setParam("lethal_cost", 254);
        planner.setParam("neutral_cost", 50);
        planner.setParam("cost_factor", .8);
        planner.setParam("use_dijkstra", true);
        planner.setParam("use_grid_path", grid_traceback);
        planner.setParam("old_navfn_behavior", true);
      };
      ros::NodeHandle grid_nh("~/GlobalPlannerGridFallback");
      if (parameters.valid()) grid_nh.setParam("", parameters);
      configure(raw_nh, false);
      configure(grid_nh, true);
      search_grid_ = snapshot();
      const auto frame = costmap_ros_->getGlobalFrameID();
      raw_->initialize("GlobalPlannerRaw", &search_grid_, frame);
      grid_fallback_.reset(new global_planner::GlobalPlanner());
      grid_fallback_->initialize("GlobalPlannerGridFallback", &search_grid_, frame);
      ros::NodeHandle node("~");
      goals_ = node.subscribe<geometry_msgs::PoseStamped>("current_goal", 1,
          [this](const geometry_msgs::PoseStamped::ConstPtr&) { ++reference_epoch_; });
      cancels_ = node.subscribe<actionlib_msgs::GoalID>("cancel", 1,
          [this](const actionlib_msgs::GoalID::ConstPtr&) { ++reference_epoch_; });
      registration_ = node.subscribe("/geographic/status", 1, &BSplineGlobalPlanner::registrationState, this);
    } else {
      raw_->initialize("GlobalPlannerRaw", costmap_ros_);
    }
    raw_pub_=nh.advertise<nav_msgs::Path>("plan_raw",1);
    smooth_pub_=nh.advertise<nav_msgs::Path>("plan",1);
    status_pub_=nh.advertise<std_msgs::String>("smoothing_status",1,true);
    ROS_INFO("B-spline global planner ready: Dijkstra -> cubic spline, step=%.3fm",sample_spacing_);
  }

  bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override {
    std::lock_guard<std::mutex> lock(plan_lock_);
    plan.clear();
    costmap_2d::Costmap2D grid;
    std::vector<std::uint8_t> hard;
    const auto epoch = reference_epoch_.load();
    const auto frame = costmap_ros_->getGlobalFrameID();
    const auto feasible = [&](const Point2& p, bool provisional) {
      unsigned int x, y;
      return std::isfinite(p.x) && std::isfinite(p.y) && grid.worldToMap(p.x, p.y, x, y)
          && !hard[y * grid.getSizeInCellsX() + x]
          && (provisional || grid.getCost(x, y) != costmap_2d::NO_INFORMATION);
    };
    std::vector<Point2> retained_suffix;
    if (cost_field_enabled_) {
      grid = snapshot();
      double inscribed, circumscribed;
      costmap_2d::calculateMinAndMaxDistances(costmap_ros_->getRobotFootprint(), inscribed, circumscribed);
      // Reserve half a cell diagonal so a pose within an admissible cell
      // has the required centre-to-hit clearance, not only at its centre.
      const double clearance = std::max(cost_options_.min_clearance_m, circumscribed)
                             + grid.getResolution() * std::sqrt(.5);
      hard = hardContinuityMask(grid.getCharMap(), grid.getSizeInCellsX(), grid.getSizeInCellsY(),
                                grid.getResolution(), clearance);
      if (start.header.frame_id != frame || goal.header.frame_id != frame ||
          !feasible(point(start), allow_unknown_) || !feasible(point(goal), allow_unknown_))
        return reject(goal, "blocked or out-of-bounds start/goal");
      std::vector<Point2> reference;
      if (epoch == previous_epoch_ && sameGoal(goal)
          && (frame != "map" || map_reference_valid_.load())) {
        retained_suffix = remainingRoute(previous_path_, point(start), cost_options_.rejoin_distance_m);
        reference = observedReference(retained_suffix, grid.getResolution() * .5, feasible);
      } else if (side_enabled_) {
        task_side_.reset(point(start), point(goal));
      }
      search_grid_ = grid;
      auto* costs = search_grid_.getCharMap();
      if (cost_options_.obstacle_weight > 0.)
        preferObstacleDistance(grid.getCharMap(), costs, grid.getSizeInCellsX(), grid.getSizeInCellsY(),
                               grid.getResolution(), clearance, cost_options_.obstacle_soft_radius_m);
      encodeContinuityCosts(costs, hard, grid.getSizeInCellsX(), grid.getSizeInCellsY(),
          grid.getResolution(), {grid.getOriginX(), grid.getOriginY()}, point(start),
          reference, cost_options_, costs);
    }
    std::vector<geometry_msgs::PoseStamped> original;
    bool found = raw_ && raw_->makePlan(start, goal, original) && !original.empty();
    if (cost_field_enabled_) {
      const auto checked = [&] {
        // In old Navfn coordinates the traceback indexes cell centres from
        // the map origin and appends the exact goal twice. Avoid gratuitous
        // subcell turns at both real poses when a checked shortcut exists.
        const auto terminal = original.back();
        original.resize(original.size() > 2 ? original.size() - 2 : 0);
        const double half_cell = grid.getResolution() * .5;
        for (auto& pose : original) {
          pose.pose.position.x += half_cell;
          pose.pose.position.y += half_cell;
        }
        const auto can_connect = [&](const auto& a, const auto& b) {
          return segmentFeasible(point(a), point(b), half_cell, allow_unknown_, feasible);
        };
        if (!original.empty() && can_connect(start, original.size() > 1 ? original[1] : terminal))
          original.erase(original.begin());
        original.insert(original.begin(), start);
        if (original.size() > 1 && can_connect(original[original.size()-2], terminal))
          original.pop_back();
        original.push_back(terminal);
        return pathFeasible(original, half_cell, feasible);
      };
      if (!found || !checked()) {
        // Retry the identical objective once with cell-centre traceback
        // when interpolation crossed the hard mask. No older route is reused.
        found = grid_fallback_->makePlan(start, goal, original) && !original.empty() && checked();
      }
      if (!found) return reject(goal, "no valid traceback on the current cost field");
    } else if (!found) {
      return false;
    }
    publish(raw_pub_,original);
    if (cost_field_enabled_ && side_enabled_) {
      std::vector<Point2> candidate;
      for (const auto& pose : original) candidate.push_back(point(pose));
      if (retained_suffix.size() >= 2) {
        retained_suffix.insert(retained_suffix.begin(), point(start));
        if (task_side_.keep(candidate, routeLength(retained_suffix))) {
          std::vector<geometry_msgs::PoseStamped> retained;
          retained.reserve(retained_suffix.size());
          for (const auto& p : retained_suffix) {
            auto pose = start;
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;
            retained.push_back(pose);
          }
          retained.back() = original.back();
          if (pathFeasible(retained, grid.getResolution() * .5, feasible)) {
            plan = std::move(retained);
            status("task side retained: opposite path below 25% advantage");
            return deliver(goal, plan, epoch);
          }
        }
      }
      task_side_.accept(candidate);
    }
    if(!enabled_ || original.size()<2) {
      plan=original;
      status("raw: disabled or stationary");
      return deliver(goal, plan, epoch);
    }
    if (!cost_field_enabled_) grid = snapshot();
    const auto footprint = cost_field_enabled_ ? std::vector<geometry_msgs::Point>{} : costmap_ros_->getRobotFootprint();
    base_local_planner::CostmapModel model(grid);
    double inscribed=0.,circumscribed=0.;
    if (!cost_field_enabled_)
      costmap_2d::calculateMinAndMaxDistances(footprint,inscribed,circumscribed);
    auto safe=[&](const PathSample& s) {
      // The field's circumscribed envelope is already heading-independent.
      // Smooth observed space and preserve the provisional raw tail.
      if (cost_field_enabled_) return feasible(s.position, false);
      unsigned int mx,my;
      if(!grid.worldToMap(s.position.x,s.position.y,mx,my) || grid.getCost(mx,my)>max_cost_) return false;
      // 使用既有实车足迹，不再增加额外padding。密集检查不是速度出口的二次拦截。
      if(model.footprintCost(s.position.x,s.position.y,s.yaw,footprint,inscribed,circumscribed)<0.) return false;
      // CostmapModel主要光栅化轮廓边缘；补查足迹内部，避免小障碍完全落在多边形内却漏检。
      std::vector<Point2> polygon;
      unsigned int left=grid.getSizeInCellsX(),right=0,bottom=grid.getSizeInCellsY(),top=0;
      const double c=std::cos(s.yaw),sn=std::sin(s.yaw);
      for(const auto& p:footprint) {
        const Point2 q{s.position.x+c*p.x-sn*p.y,s.position.y+sn*p.x+c*p.y};
        if(!grid.worldToMap(q.x,q.y,mx,my)) return false;
        polygon.push_back(q);
        left=std::min(left,mx); right=std::max(right,mx);
        bottom=std::min(bottom,my); top=std::max(top,my);
      }
      for(unsigned int y=bottom;y<=top;++y) for(unsigned int x=left;x<=right;++x) {
        if(grid.getCost(x,y)<costmap_2d::LETHAL_OBSTACLE) continue;
        double wx,wy; grid.mapToWorld(x,y,wx,wy);
        bool positive=false,negative=false;
        for(size_t i=0;i<polygon.size();++i) {
          const auto a=polygon[i],b=polygon[(i+1)%polygon.size()];
          const double cross=(b.x-a.x)*(wy-a.y)-(b.y-a.y)*(wx-a.x);
          positive=positive || cross>1e-12; negative=negative || cross< -1e-12;
        }
        if(!(positive && negative)) return false;  // 当前实车采用凸多边形足迹。
      }
      return true;
    };
    std::vector<Point2> points;
    for(const auto& pose:original) points.emplace_back(pose.pose.position.x,pose.pose.position.y);
    // 精确保留规划器的有效终点（兼容其目标容差）和调用方起点。
    points.front()={start.pose.position.x,start.pose.position.y};
    size_t smooth_count = points.size();
    double endpoint_yaw = tf2::getYaw(original.back().pose.orientation);
    if (smooth_observed_prefix_) {
      // Retain the provisional route to the true goal, but smooth its known
      // prefix as the observed area advances around the next corner.
      for (size_t i = 1; i < points.size(); ++i) {
        const Point2 tangent = points[i] - points[i-1];
        const double yaw = i+1 == points.size() ? endpoint_yaw : std::atan2(tangent.y, tangent.x);
        if (!safe({points[i], yaw, 1.})) {
          smooth_count = i;
          const Point2 last = i > 1 ? points[i-1] - points[i-2] : Point2{0., 0.};
          endpoint_yaw = std::atan2(last.y, last.x);
          break;
        }
      }
    }
    const std::vector<Point2> known_prefix(points.begin(), points.begin() + smooth_count);
    SmoothResult smooth;
    const bool endpoints = smooth_count >= 2 &&
        safe({known_prefix.front(), tf2::getYaw(start.pose.orientation), 0.}) &&
        safe({known_prefix.back(), endpoint_yaw, 1.});
    if(endpoints && smoothPath(known_prefix,epsilon_,control_spacing_,sample_spacing_,
                              grid.getResolution()*.5,refinements_,safe,smooth)) {
      for(const auto& sample:smooth.path) {
        auto pose=original.front();
        pose.header.stamp=ros::Time::now();
        pose.pose.position.x=sample.position.x; pose.pose.position.y=sample.position.y;
        pose.pose.position.z=0.;
        pose.pose.orientation.x=pose.pose.orientation.y=0.;
        pose.pose.orientation.z=std::sin(sample.yaw*.5);
        pose.pose.orientation.w=std::cos(sample.yaw*.5);
        plan.push_back(pose);
      }
      // 端点姿态保留任务语义；中间朝向采用解析切线，不覆盖任务最终yaw。
      plan.front() = start;
      if (smooth_count == original.size()) plan.back() = original.back();
      else plan.insert(plan.end(), original.begin() + smooth_count, original.end());
      std::ostringstream out;
      out<<"smoothed raw="<<original.size()<<" known="<<smooth_count<<" output="<<plan.size()
         <<" refinement="<<smooth.refinement<<" length="<<smooth.length;
      status(out.str());
    } else {
      plan=original;
      status("raw fallback: spline corner/footprint collision or nonregular curve");
      ROS_INFO_THROTTLE(5.,"B-spline: retaining original global path where smoothing is unsafe");
    }
    if (cost_field_enabled_ && !pathFeasible(plan, grid.getResolution() * .5, feasible)) {
      plan = original;
      status("raw fallback: smoothed splice crossed hard geometry");
    }
    return deliver(goal, plan, epoch);
  }
 private:
  costmap_2d::Costmap2D snapshot() const {
    auto* live = costmap_ros_->getCostmap();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*live->getMutex());
    return *live;
  }

  static Point2 point(const geometry_msgs::PoseStamped& pose) {
    return {pose.pose.position.x, pose.pose.position.y};
  }

  bool sameGoal(const geometry_msgs::PoseStamped& goal) const {
    return goal.header.frame_id == previous_goal_.header.frame_id &&
        goal.header.stamp == previous_goal_.header.stamp &&
        norm(point(goal) - point(previous_goal_)) < 1e-6 &&
        std::abs(std::remainder(tf2::getYaw(goal.pose.orientation) -
            tf2::getYaw(previous_goal_.pose.orientation), 2. * std::acos(-1.))) < 1e-6;
  }

  void registrationState(const std_msgs::String::ConstPtr& message) {
    try {
      std::istringstream stream(message->data);
      boost::property_tree::ptree status;
      boost::property_tree::read_json(stream, status);
      const auto generation = status.get<unsigned long>("generation");
      const auto registered = status.get<bool>("registered");
      const auto rebuild = status.get("costmap_rebuild_recommended", false);
      std::lock_guard<std::mutex> lock(registration_lock_);
      if (generation != generation_ || registered != registered_ || rebuild != rebuild_) ++reference_epoch_;
      generation_ = generation;
      registered_ = registered;
      rebuild_ = rebuild;
      map_reference_valid_ = registered && !rebuild;
    } catch (const boost::property_tree::ptree_error&) {
      ROS_WARN_THROTTLE(5., "Continuity-cost received unreadable registration status");
      map_reference_valid_ = false;
      ++reference_epoch_;
    }
  }

  template<class Feasible>
  static bool segmentFeasible(const Point2& a, const Point2& b, double spacing,
                              bool provisional, Feasible feasible) {
    const Point2 delta = b - a;
    const int steps = std::max(1, static_cast<int>(std::ceil(norm(delta) / spacing)));
    for (int i = 0; i <= steps; ++i)
      if (!feasible(a + delta * (static_cast<double>(i) / steps), provisional)) return false;
    return true;
  }

  template<class Feasible>
  bool pathFeasible(const std::vector<geometry_msgs::PoseStamped>& path,
                    double spacing, Feasible feasible) const {
    if (path.size() < 2) return false;
    for (std::size_t i = 1; i < path.size(); ++i)
      if (!segmentFeasible(point(path[i-1]), point(path[i]), spacing, allow_unknown_, feasible)) return false;
    return true;
  }

  template<class Feasible>
  static std::vector<Point2> observedReference(const std::vector<Point2>& path,
                                               double spacing, Feasible feasible) {
    if (path.size() < 2 || !feasible(path.front(), false)) return {};
    std::vector<Point2> reference{path.front()};
    for (std::size_t i = 1; i < path.size(); ++i) {
      if (!segmentFeasible(path[i-1], path[i], spacing, false, feasible)) break;
      reference.push_back(path[i]);
    }
    return reference.size() >= 2 ? reference : std::vector<Point2>{};
  }

  bool reject(const geometry_msgs::PoseStamped& goal, const std::string& reason) {
    previous_path_.clear();
    publish(smooth_pub_, {}, goal.header);
    status("no plan: " + reason);
    ROS_WARN_STREAM_THROTTLE(2., "Continuity-cost rejected: " << reason);
    return false;
  }

  bool deliver(const geometry_msgs::PoseStamped& goal,
               const std::vector<geometry_msgs::PoseStamped>& path, std::uint64_t epoch) {
    if (cost_field_enabled_) {
      previous_epoch_ = epoch;
      previous_goal_ = goal;
      previous_path_.clear();
      for (const auto& pose : path) previous_path_.push_back(point(pose));
    }
    publish(smooth_pub_, path);
    return true;
  }

  void publish(ros::Publisher& pub, const std::vector<geometry_msgs::PoseStamped>& poses,
               const std_msgs::Header& empty_header = {}) {
    nav_msgs::Path message;
    message.header = poses.empty() ? empty_header : poses.front().header;
    message.header.stamp=ros::Time::now(); message.poses=poses; pub.publish(message);
  }
  void status(const std::string& text) {
    std_msgs::String message; message.data=text; status_pub_.publish(message);
  }
  costmap_2d::Costmap2DROS* costmap_ros_=nullptr;
  costmap_2d::Costmap2D search_grid_;
  std::mutex plan_lock_, registration_lock_;
  std::unique_ptr<global_planner::GlobalPlanner> raw_, grid_fallback_;
  ros::Publisher raw_pub_,smooth_pub_,status_pub_;
  bool enabled_=true, smooth_observed_prefix_=false;
  double epsilon_=.03,control_spacing_=.2,sample_spacing_=.05;
  ContinuityCostOptions cost_options_;
  bool cost_field_enabled_=false, allow_unknown_=true, side_enabled_=false;
  SideHysteresis task_side_;
  std::atomic<std::uint64_t> reference_epoch_{0};
  std::atomic<bool> map_reference_valid_{true};
  std::uint64_t previous_epoch_=0;
  unsigned long generation_=0;
  bool registered_=false, rebuild_=false;
  ros::Subscriber goals_, cancels_, registration_;
  geometry_msgs::PoseStamped previous_goal_;
  std::vector<Point2> previous_path_;
  int refinements_=2,max_cost_=252;
};
}  // namespace slicer_navigation
PLUGINLIB_EXPORT_CLASS(slicer_navigation::BSplineGlobalPlanner,nav_core::BaseGlobalPlanner)
