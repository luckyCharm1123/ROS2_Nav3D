#ifndef RVIZ_POSE_TOOL_H
#define RVIZ_POSE_TOOL_H

#include <QCursor>
#include <vector>

#include <rviz_common/tool.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>
#include <OgreVector3.h>

namespace rviz_3d_nav_goal_tool
{

class Pose3DTool: public rviz_common::Tool
{
public:
  Pose3DTool();
  virtual ~Pose3DTool();

  virtual void onInitialize() override;

  virtual void activate() override;
  virtual void deactivate() override;

  virtual int processMouseEvent( rviz_common::ViewportMouseEvent& event ) override;

protected:
  virtual void onPoseSet(double x, double y, double z, double theta) = 0;

  rviz_rendering::Arrow* arrow_;
  std::vector<rviz_rendering::Arrow*> arrow_array;

  enum State
  {
    Position,
    Orientation,
    Height
  };
  State state_;

  Ogre::Vector3 pos_;
};

}

#endif