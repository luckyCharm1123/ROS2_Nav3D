#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "rviz_3d_nav_goal_tool/goal_tool.hpp"

namespace rviz_3d_nav_goal_tool
{

Goal3DTool::Goal3DTool()
{
  shortcut_key_ = 'g';

  topic_property_ = new rviz_common::properties::StringProperty( "Topic", "goal",
                                        "The topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );
}

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName( "3D Nav Goal" );
  
  // 在 RViz2 中获取节点用于发布消息
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      topic_property_->getStdString(), 1);
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
{
  RCLCPP_INFO(node_->get_logger(), "3D Goal Set");
  
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = fixed_frame;
  goal.header.stamp = node_->get_clock()->now();
  
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  
  goal.pose.orientation.x = quat.x();
  goal.pose.orientation.y = quat.y();
  goal.pose.orientation.z = quat.z();
  goal.pose.orientation.w = quat.w();

  RCLCPP_INFO(node_->get_logger(), "Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f", 
      fixed_frame.c_str(),
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
      goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
      
  pub_->publish(goal);
}

} // end namespace

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz_3d_nav_goal_tool::Goal3DTool, rviz_common::Tool )