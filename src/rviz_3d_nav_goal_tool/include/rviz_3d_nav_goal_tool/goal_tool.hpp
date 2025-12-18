#ifndef RVIZ_GOAL_TOOL_H
#define RVIZ_GOAL_TOOL_H

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rviz_common/properties/string_property.hpp>

#include "pose_tool.hpp"

namespace rviz_3d_nav_goal_tool
{

class Goal3DTool: public Pose3DTool
{
Q_OBJECT
public:
  Goal3DTool();
  virtual ~Goal3DTool() {}
  virtual void onInitialize() override;

protected:
  virtual void onPoseSet(double x, double y, double z, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;

  rviz_common::properties::StringProperty* topic_property_;
};

}

#endif