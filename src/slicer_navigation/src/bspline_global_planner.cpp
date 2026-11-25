#include <slicer_navigation/clearance_preference.h>
#include <slicer_navigation/bspline_core.h>
#include <slicer_navigation/route_continuity.h>
#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/thread/locks.hpp>
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
    nh.param("continuity/extra_cost", reuse_cost_, 0.);
    nh.param("continuity/join_distance", join_distance_, .30);
    nh.param("continuity/preferred_clearance", preferred_clearance_, 0.);
    ros::NodeHandle private_node("~");
    private_node.param("global_costmap/inflation_layer/cost_scaling_factor", clearance_decay_, 4.5);
    int lethal;
    nh.param("lethal_cost", lethal, 253);
    preferred_cap_ = std::max(0, std::min(lethal - 2, 252));
    if(!std::isfinite(epsilon_) || epsilon_<0. || !std::isfinite(control_spacing_) ||
       control_spacing_<=0. || !std::isfinite(sample_spacing_) || sample_spacing_<=0. ||
       refinements_<0 || max_cost_<0 || max_cost_>252 || !std::isfinite(reuse_cost_) ||
       reuse_cost_<0. || !std::isfinite(join_distance_) || join_distance_<=0. ||
       !std::isfinite(preferred_clearance_) || preferred_clearance_<0. || preferred_clearance_>2. ||
       !std::isfinite(clearance_decay_) || clearance_decay_<=0.)
      throw std::invalid_argument("invalid GlobalPlanner/smoothing configuration");
    raw_.reset(new global_planner::GlobalPlanner());
    if (reuse_cost_ > 0.) {
      nh.param("allow_unknown", allow_unknown_, true);
      nh.param("continuity/brake_deceleration", braking_, .20);
      double limit_x, limit_trans;
      private_node.param("DWAPlannerROS/max_vel_x", limit_x, .8);
      private_node.param("DWAPlannerROS/max_vel_trans", limit_trans, .8);
      max_speed_ = std::min(limit_x, limit_trans);
      private_node.param("DWAPlannerROS/sim_time", preview_, 1.05);
      ros::NodeHandle guard("/slicer_nav_arc_guard");
      guard.param("compensation_horizon", minimum_time_, .8);
      guard.param("command_timeout", reaction_, .50);
      for (double value : {limit_x, limit_trans, preview_, minimum_time_, reaction_, braking_})
        if (!std::isfinite(value) || value <= 0.)
          throw std::invalid_argument("invalid RTK continuity motion envelope");
      std::string odom;
      private_node.param<std::string>("DWAPlannerROS/odom_topic", odom, "odom");
      odometry_ = ros::NodeHandle().subscribe(odom, 1, &BSplineGlobalPlanner::onOdom, this);
    }
    if (copyPlanningGrid()) {
      preferred_grid_ = snapshot();
      raw_->initialize("GlobalPlannerRaw", &preferred_grid_, costmap_ros_->getGlobalFrameID());
      bool use_grid_path;
      nh.param("use_grid_path", use_grid_path, false);
      if (reuse_cost_ > 0. && !use_grid_path) {
        ros::NodeHandle grid_nh("~/GlobalPlannerGridFallback");
        if (parameters.valid()) grid_nh.setParam("", parameters);
        grid_nh.setParam("use_grid_path", true);
        grid_fallback_.reset(new global_planner::GlobalPlanner());
        grid_fallback_->initialize("GlobalPlannerGridFallback", &preferred_grid_, costmap_ros_->getGlobalFrameID());
      }
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
    if (copyPlanningGrid()) {
      grid = snapshot();
      preferred_grid_ = grid;
      if (preferred_clearance_ > 0.) {
        double inscribed, circumscribed;
        costmap_2d::calculateMinAndMaxDistances(costmap_ros_->getRobotFootprint(), inscribed, circumscribed);
        preferClearance(preferred_grid_.getCharMap(), preferred_grid_.getSizeInCellsX(),
                        preferred_grid_.getSizeInCellsY(), preferred_grid_.getResolution(),
                        preferred_clearance_, inscribed, clearance_decay_, preferred_cap_);
      }
    }
    Plan original;
    if (!raw_ || !raw_->makePlan(start, goal, original)) original.clear();
    if (!original.empty()) publish(raw_pub_, original);
    if (reuse_cost_ == 0.) {
      if (original.empty()) return false;
      if (!enabled_ || original.size() < 2) {
        plan = original;
        publish(smooth_pub_, plan);
        status("raw: disabled or stationary");
        return true;
      }
    }
    if (!copyPlanningGrid()) grid = snapshot();
    auto footprint=costmap_ros_->getRobotFootprint();
    base_local_planner::CostmapModel model(grid);
    double inscribed=0.,circumscribed=0.;
    costmap_2d::calculateMinAndMaxDistances(footprint,inscribed,circumscribed);
    const auto center_cost = [&](const Point2& p) -> int {
      unsigned int mx, my;
      return grid.worldToMap(p.x, p.y, mx, my) ? grid.getCost(mx, my) : -1;
    };
    const auto footprint_issue = [&](const Point2& p, double yaw) {
      unsigned int mx, my;
      // 使用既有实车足迹，不再增加额外padding。密集检查不是速度出口的二次拦截。
      const double value = model.footprintCost(p.x, p.y, yaw, footprint, inscribed, circumscribed);
      if (value < 0.) {
        if (value == -3.) return RouteIssue::Outside;
        return value == -2. ? RouteIssue::Unknown : RouteIssue::Collision;
      }
      // CostmapModel主要光栅化轮廓边缘；补查足迹内部，避免小障碍完全落在多边形内却漏检。
      std::vector<Point2> polygon;
      unsigned int left=grid.getSizeInCellsX(),right=0,bottom=grid.getSizeInCellsY(),top=0;
      const double c=std::cos(yaw),sn=std::sin(yaw);
      for(const auto& vertex:footprint) {
        const Point2 q{p.x+c*vertex.x-sn*vertex.y,p.y+sn*vertex.x+c*vertex.y};
        if(!grid.worldToMap(q.x,q.y,mx,my)) return RouteIssue::Outside;
        polygon.push_back(q);
        left=std::min(left,mx); right=std::max(right,mx);
        bottom=std::min(bottom,my); top=std::max(top,my);
      }
      for(unsigned int y=bottom;y<=top;++y) for(unsigned int x=left;x<=right;++x) {
        const unsigned char cell = grid.getCost(x,y);
        if(cell<costmap_2d::LETHAL_OBSTACLE) continue;
        double wx,wy; grid.mapToWorld(x,y,wx,wy);
        bool positive=false,negative=false;
        for(size_t i=0;i<polygon.size();++i) {
          const auto a=polygon[i],b=polygon[(i+1)%polygon.size()];
          const double cross=(b.x-a.x)*(wy-a.y)-(b.y-a.y)*(wx-a.x);
          positive=positive || cross>1e-12; negative=negative || cross< -1e-12;
        }
        if(!(positive && negative))
          return cell == costmap_2d::NO_INFORMATION ? RouteIssue::Unknown : RouteIssue::Collision;
      }
      return RouteIssue::None;
    };
    auto safe = [&](const PathSample& sample) {
      return cellIssue(center_cost(sample.position), max_cost_) == RouteIssue::None
          && footprint_issue(sample.position, sample.yaw) == RouteIssue::None;
    };
    const double horizon = executableHorizon();
    const auto assess = [&](const Plan& route) {
      if (route.size() < 2) return inspectRoute(pointsOf(route), grid.getResolution() * .5, horizon,
                                              max_cost_, allow_unknown_, center_cost, footprint_issue);
      const auto initial = footprint_issue(point(start), tf2::getYaw(start.pose.orientation));
      if (initial != RouteIssue::None) {
        RouteCheck check;
        check.near = {initial, 0., center_cost(point(start))};
        return check;
      }
      return inspectRoute(pointsOf(route), grid.getResolution() * .5, horizon,
                          max_cost_, allow_unknown_, center_cost, footprint_issue);
    };
    if (reuse_cost_ > 0.) {
      normalizeStart(start, original);
      const bool reused = selectRoute(start, goal, horizon, grid.getResolution(), assess, original);
      if (original.empty()) {
        previous_path_.clear();
        publish(smooth_pub_, plan, start.header);
        status("failed: no checked route to target");
        return false;
      }
      if (reused || !enabled_) {
        plan = original;
        return deliver(goal, plan);
      }
    }

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
    if (reuse_cost_ > 0.) {
      if (!assess(plan).valid()) {
        plan = original;
        status("raw fallback: smoothed route failed executable-prefix validation");
      }
      // The raw route already passed the same checker during selection.
      return deliver(goal, plan);
    }
    publish(smooth_pub_,plan);
    return true;
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

  using Plan = std::vector<geometry_msgs::PoseStamped>;

  static void normalizeStart(const geometry_msgs::PoseStamped& start, Plan& path) {
    if (path.size() == 1) path.insert(path.begin(), start);
    else if (!path.empty()) path.front() = start;
  }

  bool copyPlanningGrid() const { return preferred_clearance_ > 0. || reuse_cost_ > 0.; }

  void onOdom(const nav_msgs::Odometry::ConstPtr& odom) {
    if (odom->child_frame_id != costmap_ros_->getBaseFrameID()) return;
    const double speed = std::hypot(odom->twist.twist.linear.x, odom->twist.twist.linear.y);
    if (!std::isfinite(speed) || odom->header.stamp.isZero()) return;
    std::lock_guard<std::mutex> lock(odom_lock_);
    observed_speed_ = speed;
    odom_stamp_ = odom->header.stamp;
  }

  double executableHorizon() {
    std::lock_guard<std::mutex> lock(odom_lock_);
    const double age = (ros::Time::now() - odom_stamp_).toSec();
    const double speed = !odom_stamp_.isZero() && age >= 0. && age <= .5 ? observed_speed_ : max_speed_;
    return checkedHorizon(max_speed_, preview_, speed, minimum_time_, reaction_, braking_);
  }

  static std::vector<Point2> pointsOf(const Plan& path) {
    std::vector<Point2> points;
    points.reserve(path.size());
    for (const auto& p : path) points.push_back(point(p));
    return points;
  }

  bool sameGoal(const geometry_msgs::PoseStamped& goal) const {
    return goal.header.frame_id == previous_goal_.header.frame_id
        && norm(point(goal) - point(previous_goal_)) < 1e-6
        && std::abs(std::remainder(tf2::getYaw(previous_goal_.pose.orientation) -
                                  tf2::getYaw(goal.pose.orientation), 2. * std::acos(-1.))) < 1e-6;
  }

  Plan reconnect(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                 double& gap) const {
    if (previous_path_.size() < 2 || !sameGoal(goal)) return {};
    // Join at the closest position on a segment, not at a discretized waypoint.
    gap = std::numeric_limits<double>::infinity();
    size_t nearest = 0;
    Point2 join;
    for (size_t i = 1; i < previous_path_.size(); ++i) {
      const Point2 p = point(previous_path_[i-1]);
      const Point2 delta = point(previous_path_[i]) - p;
      const double squared = dot(delta, delta);
      if (squared < 1e-12) continue;
      const double along = std::max(0., std::min(1., dot(point(start) - p, delta) / squared));
      const Point2 candidate = p + delta * along;
      const double distance = norm(point(start) - candidate);
      if (distance < gap) {
        gap = distance;
        nearest = i;
        join = candidate;
      }
    }
    if (!nearest || gap > join_distance_) return {};
    Plan path{start};
    if (gap > 1e-6 && norm(join - point(previous_path_[nearest])) > 1e-6) {
      auto projected = previous_path_[nearest];
      projected.pose.position.x = join.x;
      projected.pose.position.y = join.y;
      path.push_back(projected);
    }
    path.insert(path.end(), previous_path_.begin() + nearest, previous_path_.end());
    for (auto& pose : path) pose.header.stamp = start.header.stamp;
    return path;
  }

  static Plan prefixTo(const Plan& path, double limit) {
    double distance = 0.;
    Plan prefix{path.front()};
    for (size_t i = 1; i < path.size(); ++i) {
      const Point2 direction = point(path[i]) - point(path[i-1]);
      const double length = norm(direction);
      if (length < 1e-9) continue;
      if (distance + length >= limit) {
        auto anchor = path[i];
        const Point2 p = point(path[i-1]) + direction * ((limit - distance) / length);
        anchor.pose.position.x = p.x;
        anchor.pose.position.y = p.y;
        tf2::Quaternion rotation;
        rotation.setRPY(0., 0., std::atan2(direction.y, direction.x));
        anchor.pose.orientation = tf2::toMsg(rotation);
        prefix.push_back(anchor);
        return prefix;
      }
      distance += length;
      prefix.push_back(path[i]);
    }
    return {};
  }

  static bool advancesThroughJoin(const Plan& prefix, const Plan& suffix, double resolution) {
    if (suffix.size() < 2) return false;
    const Point2 join = point(prefix.back());
    const Point2 incoming = join - point(prefix[prefix.size()-2]);
    const Point2 outgoing = routePoint(pointsOf(suffix), resolution * 2.) - join;
    if (dot(incoming, outgoing) <= 0.) return false;
    // Reject a detour that immediately sends the car back through the committed corridor.
    const auto committed = pointsOf(prefix);
    const double sample = resolution * .5;
    for (size_t i = 1; i < suffix.size(); ++i) {
      const Point2 from = point(suffix[i-1]);
      const Point2 delta = point(suffix[i]) - from;
      const int steps = std::max(1, static_cast<int>(std::ceil(norm(delta) / sample)));
      for (int j = 1; j <= steps; ++j) {
        const Point2 p = from + delta * (double(j) / steps);
        if (norm(p - join) <= resolution * 2.) continue;
        for (size_t k = 1; k < committed.size(); ++k)
          if (segmentDistance(p, committed[k-1], committed[k]) < resolution) return false;
      }
    }
    return true;
  }

  static void explain(const char* label, const RouteCheck& route) {
    const RouteBlock block = route.near.issue != RouteIssue::None ? route.near : route.far;
    if (block.issue != RouteIssue::None)
      ROS_INFO_THROTTLE(2., "B-spline %s: %s %s at %.2f m (center cost %d)", label,
                        route.near.issue != RouteIssue::None ? "executable prefix" : "suffix",
                        issueName(block.issue), block.distance, block.cost);
  }

  template<class Assess>
  bool selectRoute(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                   double horizon, double resolution, Assess assess, Plan& candidate) {
    auto new_route = assess(candidate);
    if (!new_route.valid()) explain("new route rejected", new_route);
    double gap = 0.;
    Plan previous = reconnect(start, goal, gap);
    if (previous.empty()) {
      if (!previous_path_.empty() && sameGoal(goal) && gap > join_distance_)
        ROS_INFO_THROTTLE(2., "B-spline previous route rejected: %s at %.2f m", issueName(RouteIssue::Rejoin), gap);
    } else {
      const auto old = assess(previous);
      if (old.valid() && (!new_route.valid()
          || preferPreviousRoute(old.cost, new_route.cost, reuse_cost_)
          || !sameCorridor(pointsOf(previous), pointsOf(candidate), horizon, join_distance_))) {
        candidate = std::move(previous);
        status("retained: checked route to same target");
        return true;
      }
      if (!old.valid()) explain("previous route rejected", old);
      if (old.near.issue == RouteIssue::None && old.far.issue != RouteIssue::None) {
        // Route repair is bounded to two seams. Each candidate is a complete
        // route to the original goal and passes the same checks as a fresh plan.
        const double latest = std::min(old.observed - resolution,
                                       horizon + max_speed_ * reaction_);
        for (const double seam : {latest, horizon}) {
          if (seam < horizon || seam >= old.far.distance - resolution) continue;
          Plan joined = prefixTo(previous, seam), suffix;
          if (joined.size() < 2 || !raw_->makePlan(joined.back(), goal, suffix)
              || !advancesThroughJoin(joined, suffix, resolution)) continue;
          // Upstream traceback may not start at the exact requested pose.
          suffix.front() = joined.back();
          joined.insert(joined.end(), suffix.begin() + 1, suffix.end());
          if (assess(joined).valid()) {
            candidate = std::move(joined);
            status("repaired: checked prefix and new suffix to same target");
            return true;
          }
          if (std::abs(latest - horizon) < resolution) break;
        }
      }
    }
    if (!new_route.valid()) {
      candidate.clear();
      // Continuous traceback sometimes crosses an excluded cell between valid
      // potential samples. Retry once with the upstream cell-center traceback.
      if (grid_fallback_ && grid_fallback_->makePlan(start, goal, candidate)) {
        normalizeStart(start, candidate);
        const auto grid_route = assess(candidate);
        if (grid_route.valid()) {
          ROS_INFO_THROTTLE(2., "B-spline: using a checked grid traceback after gradient failure");
          return false;
        }
        explain("grid traceback rejected", grid_route);
      }
      candidate.clear();
    }
    return false;
  }

  bool deliver(const geometry_msgs::PoseStamped& goal, const Plan& path) {
    previous_goal_ = goal;
    previous_path_ = path;
    publish(smooth_pub_, path);
    return true;
  }

  void publish(ros::Publisher& pub, const Plan& poses, const std_msgs::Header& empty_header = {}) {
    nav_msgs::Path message;
    message.header = poses.empty() ? empty_header : poses.front().header;
    message.header.stamp = ros::Time::now();
    message.poses = poses; pub.publish(message);
  }
  void status(const std::string& text) {
    std_msgs::String message; message.data=text; status_pub_.publish(message);
  }
  costmap_2d::Costmap2DROS* costmap_ros_=nullptr;
  costmap_2d::Costmap2D preferred_grid_;
  std::mutex plan_lock_, odom_lock_;
  ros::Subscriber odometry_;
  ros::Time odom_stamp_;
  double observed_speed_ = 0.;
  std::unique_ptr<global_planner::GlobalPlanner> raw_, grid_fallback_;
  ros::Publisher raw_pub_,smooth_pub_,status_pub_;
  bool enabled_=true, smooth_observed_prefix_=false;
  double epsilon_=.03,control_spacing_=.2,sample_spacing_=.05;
  double reuse_cost_=0., join_distance_=.30, preferred_clearance_=0., clearance_decay_=4.5;
  int preferred_cap_=173;
  bool allow_unknown_ = true;
  double braking_ = .20, max_speed_ = .8, preview_ = 1.05, minimum_time_ = .8, reaction_ = .50;
  geometry_msgs::PoseStamped previous_goal_;
  std::vector<geometry_msgs::PoseStamped> previous_path_;
  int refinements_=2,max_cost_=252;
};
}  // namespace slicer_navigation
PLUGINLIB_EXPORT_CLASS(slicer_navigation::BSplineGlobalPlanner,nav_core::BaseGlobalPlanner)
