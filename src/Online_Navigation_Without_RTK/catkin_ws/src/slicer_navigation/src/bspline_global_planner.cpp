#include <slicer_navigation/bspline_core.h>
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
    if(!std::isfinite(epsilon_) || epsilon_<0. || !std::isfinite(control_spacing_) ||
       control_spacing_<=0. || !std::isfinite(sample_spacing_) || sample_spacing_<=0. ||
       refinements_<0 || max_cost_<0 || max_cost_>252)
      throw std::invalid_argument("invalid GlobalPlanner/smoothing configuration");
    raw_.reset(new global_planner::GlobalPlanner());
    raw_->initialize("GlobalPlannerRaw",costmap_ros_);
    raw_pub_=nh.advertise<nav_msgs::Path>("plan_raw",1);
    smooth_pub_=nh.advertise<nav_msgs::Path>("plan",1);
    status_pub_=nh.advertise<std_msgs::String>("smoothing_status",1,true);
    ROS_INFO("B-spline global planner ready: Dijkstra -> cubic spline -> TEB, step=%.3fm",sample_spacing_);
  }

  bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override {
    plan.clear();
    std::vector<geometry_msgs::PoseStamped> original;
    if(!raw_ || !raw_->makePlan(start,goal,original) || original.empty()) return false;
    publish(raw_pub_,original);
    if(!enabled_ || original.size()<2) {
      plan=original; publish(smooth_pub_,plan); status("raw: disabled or stationary"); return true;
    }
    // 仅短暂锁住全局地图制作快照，曲线拟合不占用实时代价地图更新线程。
    costmap_2d::Costmap2D grid;
    {
      auto* live=costmap_ros_->getCostmap();
      boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*live->getMutex());
      grid=*live;
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
    SmoothResult smooth;
    const bool endpoints=safe({points.front(),tf2::getYaw(start.pose.orientation),0.}) &&
                         safe({points.back(),tf2::getYaw(original.back().pose.orientation),1.});
    if(endpoints && smoothPath(points,epsilon_,control_spacing_,sample_spacing_,
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
      plan.front()=start; plan.back()=original.back();
      std::ostringstream out;
      out<<"smoothed raw="<<original.size()<<" output="<<plan.size()
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
  bool enabled_=true;
  double epsilon_=.03,control_spacing_=.2,sample_spacing_=.05;
  int refinements_=2,max_cost_=252;
};
}  // namespace slicer_navigation
PLUGINLIB_EXPORT_CLASS(slicer_navigation::BSplineGlobalPlanner,nav_core::BaseGlobalPlanner)
