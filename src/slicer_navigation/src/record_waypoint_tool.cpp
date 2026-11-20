// 地图平面录点：不依赖 GPU 深度拾取，不发送导航目标或底盘速度。
#include <cmath>
#include <sstream>
#include <OgrePlane.h>
#include <OgreVector3.h>
#include <QCursor>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <rviz/tool.h>
#include <rviz/display_context.h>
#include <rviz/geometry.h>
#include <rviz/viewport_mouse_event.h>

namespace slicer_navigation
{
class RecordWaypointTool : public rviz::Tool
{
public:
  RecordWaypointTool() { shortcut_key_ = 'p'; }
  void onInitialize() override
  {
    setName("Record Waypoint");
    setCursor(QCursor(Qt::CrossCursor));
    publisher_ = node_.advertise<geometry_msgs::PointStamped>("/clicked_point", 100);
  }
  void activate() override
  {
    pressed_ = false;
    setStatus("Click the map plane to RECORD a waypoint. Repeated clicks stay active. No motion goal is sent.");
  }
  void deactivate() override { pressed_ = false; }
  int processMouseEvent(rviz::ViewportMouseEvent& event) override
  {
    if (event.leftDown()) pressed_ = true;
    if (!event.leftUp()) return 0;
    const bool clicked = pressed_;
    pressed_ = false;
    if (!clicked) return 0;
    // 用当前固定坐标系的 z=0 平面求交；白色自由区和无渲染实体的区域也能点中。
    Ogre::Plane ground(Ogre::Vector3::UNIT_Z, 0.f);
    Ogre::Vector3 position;
    if (!rviz::getPointOnPlaneFromWindowXY(event.viewport, ground, event.x, event.y, position) ||
        !std::isfinite(position.x) || !std::isfinite(position.y))
    {
      setStatus("Cannot intersect map plane: switch to a top-down view.");
      ROS_WARN("Record Waypoint: view does not intersect the map plane; click NOT sent");
      return 0;
    }
    if (context_->getFixedFrame() != "map")
    {
      setStatus("Recording disabled: set RViz Fixed Frame to map first.");
      return 0;
    }
    if (!publisher_.getNumSubscribers())
    {
      setStatus("Recorder is OFFLINE. Run run_navigation.sh record in your SSH terminal, then click again.");
      ROS_WARN("Record Waypoint: no /clicked_point subscriber; start recorder before clicking");
      return 0;
    }
    geometry_msgs::PointStamped message;
    message.header.frame_id = context_->getFixedFrame().toStdString();
    message.header.stamp = ros::Time::now();
    message.point.x = position.x;
    message.point.y = position.y;
    message.point.z = 0.;
    publisher_.publish(message);
    std::ostringstream text;
    text << "Click sent: x=" << position.x << " y=" << position.y
         << ". Check terminal Recorded message for disk-save confirmation.";
    setStatus(QString::fromStdString(text.str()));
    return Render;  // 不返回 Finished，允许持续连续打点。
  }
private:
  ros::NodeHandle node_;
  ros::Publisher publisher_;
  bool pressed_ = false;
};
}  // namespace slicer_navigation
PLUGINLIB_EXPORT_CLASS(slicer_navigation::RecordWaypointTool, rviz::Tool)
