#include <slicer_navigation/bspline_core.h>
#include <slicer_navigation/route_continuity.h>
#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/thread/locks.hpp>
#include <memory>
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
    if(!std::isfinite(epsilon_) || epsilon_<0. || !std::isfinite(control_spacing_) ||
       control_spacing_<=0. || !std::isfinite(sample_spacing_) || sample_spacing_<=0. ||
       refinements_<0 || max_cost_<0 || max_cost_>252 || !std::isfinite(reuse_cost_) ||
       reuse_cost_<0. || !std::isfinite(join_distance_) || join_distance_<=0.)
      throw std::invalid_argument("invalid GlobalPlanner/smoothing configuration");
    raw_.reset(new global_planner::GlobalPlanner());
    raw_->initialize("GlobalPlannerRaw",costmap_ros_);
    raw_pub_=nh.advertise<nav_msgs::Path>("plan_raw",1);
    smooth_pub_=nh.advertise<nav_msgs::Path>("plan",1);
    status_pub_=nh.advertise<std_msgs::String>("smoothing_status",1,true);
    ROS_INFO("B-spline global planner ready: Dijkstra -> cubic spline, step=%.3fm",sample_spacing_);
  }

  bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override {
    plan.clear();
    std::vector<geometry_msgs::PoseStamped> original;
    if(!raw_ || !raw_->makePlan(start,goal,original) || original.empty()) return false;
    publish(raw_pub_,original);
    if((!enabled_ && reuse_cost_ == 0.) || original.size()<2) {
      plan=original; publish(smooth_pub_,plan); status("raw: disabled or stationary"); return true;
    }
    // 仅短暂锁住全局地图制作快照，曲线拟合不占用实时代价地图更新线程。
    costmap_2d::Costmap2D grid;
    {
      auto* live=costmap_ros_->getCostmap();
      boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*live->getMutex());
      grid=*live;
    }
    if (reuse_cost_ > 0.) retainRoute(start, goal, grid, original);
    if (!enabled_) {
      plan = original;
      publish(smooth_pub_, plan);
      return true;
    }
    auto footprint=costmap_ros_->getRobotFootprint();
    base_local_planner::CostmapModel model(grid);
    double inscribed=0.,circumscribed=0.;
    costmap_2d::calculateMinAndMaxDistances(footprint,inscribed,circumscribed);
    auto safe=[&](const PathSample& s) {
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
    publish(smooth_pub_,plan);
    return true;
  }
 private:
  static Point2 point(const geometry_msgs::PoseStamped& pose) {
    return {pose.pose.position.x, pose.pose.position.y};
  }

  void retainRoute(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                   const costmap_2d::Costmap2D& grid,
                   std::vector<geometry_msgs::PoseStamped>& current) {
    // A newer destination never inherits the route or terminal orientation of an older goal.
    if (!previous_path_.empty() && goal.header.frame_id == previous_goal_.header.frame_id &&
        norm(point(goal) - point(previous_goal_)) < 1e-6 &&
        std::abs(std::remainder(tf2::getYaw(previous_goal_.pose.orientation) -
                               tf2::getYaw(goal.pose.orientation), 2. * std::acos(-1.))) < 1e-6) {
      const auto nearest = std::min_element(previous_path_.begin(), previous_path_.end(),
          [&](const auto& a, const auto& b) {
            return norm(point(a) - point(start)) < norm(point(b) - point(start));
          });
      const double gap = norm(point(*nearest) - point(start));
      if (gap <= join_distance_) {
        const auto cost_at = [&](const Point2& p) -> int {
          unsigned int x, y;
          return grid.worldToMap(p.x, p.y, x, y) ? grid.getCost(x, y) : -1;
        };
        std::vector<Point2> retained{point(start)}, alternative;
        for (auto it = nearest; it != previous_path_.end(); ++it) retained.push_back(point(*it));
        for (const auto& pose : current) alternative.push_back(point(pose));
        const double spacing = grid.getResolution() * .5;
        if (preferPreviousRoute(routeCost(retained, spacing, gap, cost_at, max_cost_),
                                routeCost(alternative, spacing, -1., cost_at, max_cost_), reuse_cost_)) {
          std::vector<geometry_msgs::PoseStamped> joined{start};
          if (gap > 1e-6) joined.insert(joined.end(), nearest, previous_path_.end());
          else joined.insert(joined.end(), nearest + 1, previous_path_.end());
          if (joined.size() >= 2) {
            for (auto& pose : joined) pose.header.stamp = start.header.stamp;
            current = std::move(joined);
            ROS_DEBUG("Retaining the validated route around the same side of the obstacle");
          }
        }
      }
    }
    previous_goal_ = goal;
    previous_path_ = current;
  }

  void publish(ros::Publisher& pub,const std::vector<geometry_msgs::PoseStamped>& poses) {
    nav_msgs::Path message; message.header=poses.front().header;
    message.header.stamp=ros::Time::now(); message.poses=poses; pub.publish(message);
  }
  void status(const std::string& text) {
    std_msgs::String message; message.data=text; status_pub_.publish(message);
  }
  costmap_2d::Costmap2DROS* costmap_ros_=nullptr;
  std::unique_ptr<global_planner::GlobalPlanner> raw_;
  ros::Publisher raw_pub_,smooth_pub_,status_pub_;
  bool enabled_=true, smooth_observed_prefix_=false;
  double epsilon_=.03,control_spacing_=.2,sample_spacing_=.05;
  double reuse_cost_=0., join_distance_=.30;
  geometry_msgs::PoseStamped previous_goal_;
  std::vector<geometry_msgs::PoseStamped> previous_path_;
  int refinements_=2,max_cost_=252;
};
}  // namespace slicer_navigation
PLUGINLIB_EXPORT_CLASS(slicer_navigation::BSplineGlobalPlanner,nav_core::BaseGlobalPlanner)
