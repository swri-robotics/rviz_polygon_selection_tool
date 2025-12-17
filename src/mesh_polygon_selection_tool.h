#pragma once

#include "rviz_polygon_selection_tool.h"

namespace rviz_common::properties
{
class StringProperty;
}

namespace rviz_polygon_selection_tool
{
class MeshPolygonSelectionTool : public PolygonSelectionTool
{
  Q_OBJECT

public:
  MeshPolygonSelectionTool();
  virtual ~MeshPolygonSelectionTool() override;

public Q_SLOTS:
  void updateColor();

protected:
  virtual Ogre::MovableObject* createToolVisualization() override;

  rviz_common::properties::StringProperty* mesh_file_;
  rviz_common::properties::ColorProperty* color_property_;
  Ogre::MaterialPtr material_;
};

}  // namespace rviz_polygon_selection_tool
