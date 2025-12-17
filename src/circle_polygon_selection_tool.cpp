#include "circle_polygon_selection_tool.h"

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

const static unsigned POINTS = 1000;

namespace rviz_polygon_selection_tool
{
CirclePolygonSelectionTool::CirclePolygonSelectionTool() : PolygonSelectionTool()
{
  shortcut_key_ = 'c';

  radius_property_ =
      new rviz_common::properties::FloatProperty("Tool Radius", 0.210f, "The radius of the tool circle display",
                                                 getPropertyContainer(), SLOT(updateToolVisualization()), this);

  color_property_ =
      new rviz_common::properties::ColorProperty("Color", QColor(255, 255, 255), "The color of the tool visualization",
                                                 getPropertyContainer(), SLOT(updateToolVisualization()), this);
}

Ogre::MovableObject* CirclePolygonSelectionTool::createToolVisualization()
{
  Ogre::ManualObject* manual = scene_manager_->createManualObject();

  // Set the type of manual object
  manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

  // Loop over an arbitrary number of points in the circle
  const float& radius = radius_property_->getFloat();

  for (unsigned i = 0; i < POINTS; ++i)
  {
    float angle = static_cast<float>(i) / static_cast<float>(POINTS) * 2.0f * M_PI;
    float x = radius * std::cos(angle);
    float y = radius * std::sin(angle);

    manual->position(x, y, 0.0f);
    manual->index(i);
    manual->colour(color_property_->getOgreColor());
  }

  // Add a line strip from the last index to the first
  manual->index(0);
  manual->end();

  return manual;
}

}  // namespace rviz_polygon_selection_tool

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_polygon_selection_tool::CirclePolygonSelectionTool, rviz_common::Tool)
