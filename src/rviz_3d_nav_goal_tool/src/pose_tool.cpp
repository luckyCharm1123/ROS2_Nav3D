#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <OgreCamera.h>

#include <rviz_rendering/geometry.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/display_context.hpp>

#include "rviz_3d_nav_goal_tool/pose_tool.hpp"

namespace rviz_3d_nav_goal_tool
{

bool getPointOnPlaneFromWindowXY(Ogre::Viewport* viewport,
                                 Ogre::Plane& plane,
                                 int window_x, int window_y,
                                 Ogre::Vector3& intersection_out)
{
  int width = viewport->getActualWidth();
  int height = viewport->getActualHeight();

  Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay(
      (float)window_x / (float)width, (float)window_y / (float)height);

  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(plane);
  if (intersection.first)
  {
    intersection_out = mouse_ray.getPoint(intersection.second);
    return true;
  }

  return false;
}

Pose3DTool::Pose3DTool()
  : rviz_common::Tool()
  , arrow_( NULL )
{
}

Pose3DTool::~Pose3DTool()
{
  delete arrow_;
  for(auto* a : arrow_array) delete a;
}

void Pose3DTool::onInitialize()
{
  arrow_ = new rviz_rendering::Arrow( scene_manager_, NULL, 2.0f, 0.2f, 0.5f, 0.35f );
  arrow_->setColor( 0.0f, 1.0f, 0.0f, 1.0f );
  arrow_->getSceneNode()->setVisible( false );
}

void Pose3DTool::activate()
{
  setStatus( "Click and drag mouse to set position/orientation." );
  state_ = Position;
}

void Pose3DTool::deactivate()
{
  arrow_->getSceneNode()->setVisible( false );
  for(auto* a : arrow_array) delete a;
  arrow_array.clear();
}

int Pose3DTool::processMouseEvent( rviz_common::ViewportMouseEvent& event )
{
  int flags = 0;
  static double initz;
  static double prevz;
  static double prevangle;
  const double z_scale = 50;
  const double z_interval = 0.5;
  Ogre::Quaternion orient_x = Ogre::Quaternion( Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y );

  if( event.leftDown() )
  {
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    auto viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
    if( getPointOnPlaneFromWindowXY( viewport,
                                     ground_plane,
                                     event.x, event.y, intersection ))
    {
      pos_ = intersection;
      arrow_->setPosition( pos_ );
      state_ = Orientation;
      flags |= Render;
    }
  }
  else if( event.type == QEvent::MouseMove && event.left() )
  {
    if( state_ == Orientation )
    {
      Ogre::Vector3 cur_pos;
      Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
      auto viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
      if( getPointOnPlaneFromWindowXY( viewport,
                                       ground_plane,
                                       event.x, event.y, cur_pos ))
      {
        double angle = atan2( cur_pos.y - pos_.y, cur_pos.x - pos_.x );
        arrow_->getSceneNode()->setVisible( true );
        arrow_->setOrientation( Ogre::Quaternion( Ogre::Radian(angle), Ogre::Vector3::UNIT_Z ) * orient_x );
        if ( event.right() )
          state_ = Height;
        initz = pos_.z;
        prevz = event.y;
        prevangle = angle;
        flags |= Render;
      }
    }
    if ( state_ == Height )
    {
      double z = event.y;
      double dz = z - prevz;
      prevz = z;
      pos_.z -= dz / z_scale;
      arrow_->setPosition( pos_ );
      
      for (auto* a : arrow_array) delete a;
      arrow_array.clear();
      
      int cnt = ceil( fabs(initz - pos_.z) / z_interval );
      for (int k = 0; k < cnt; k++)
      {
        rviz_rendering::Arrow* arrow__;
        arrow__ = new rviz_rendering::Arrow( scene_manager_, NULL, 0.5f, 0.1f, 0.0f, 0.1f );
        arrow__->setColor( 0.0f, 1.0f, 0.0f, 1.0f );
        arrow__->getSceneNode()->setVisible( true );       
        Ogre::Vector3 arr_pos = pos_;
        arr_pos.z = initz - ( (initz - pos_.z > 0)?1:-1 ) * k * z_interval;
        arrow__->setPosition( arr_pos );
        arrow__->setOrientation( Ogre::Quaternion( Ogre::Radian(prevangle), Ogre::Vector3::UNIT_Z ) * orient_x );
        arrow_array.push_back(arrow__); 
      }
      flags |= Render;    
    }
  }
  else if( event.leftUp() )
  {
    if( state_ == Orientation || state_ == Height)
    {
      for (auto* a : arrow_array) delete a;
      arrow_array.clear();
      onPoseSet(pos_.x, pos_.y, pos_.z, prevangle);
      flags |= (Finished|Render);
    }
  }

  return flags;
}

}