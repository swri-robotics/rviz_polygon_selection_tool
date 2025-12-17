#pragma once

#include "rviz_polygon_selection_tool.h"

namespace rviz_polygon_selection_tool
{
class CirclePolygonSelectionTool : public PolygonSelectionTool
{
  Q_OBJECT

public:
  CirclePolygonSelectionTool();

protected:
  virtual Ogre::MovableObject* createToolVisualization() override;

  rviz_common::properties::FloatProperty* radius_property_;
  rviz_common::properties::ColorProperty* color_property_;
};

}  // namespace rviz_polygon_selection_tool
