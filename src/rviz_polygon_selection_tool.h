#include <rviz_polygon_selection_tool/srv/get_selection.hpp>

#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <rviz_common/tool.hpp>
#include <rclcpp/service.hpp>

namespace rviz_rendering
{
  class MaterialManager;
  class MovableText;
} // namespace rviz_rendering

namespace rviz_common
{
namespace properties
{
class BoolProperty;
class ColorProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

class OgreManualObject;

namespace rviz_polygon_selection_tool
{
class PolygonSelectionTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  PolygonSelectionTool();
  virtual ~PolygonSelectionTool();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void newPolygon();
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
  int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel) override;

public Q_SLOTS:
  void updatePointsColor();
  void updatePointsSize();
  void updateLinesColor();
  void updateVisual();
  void updateTextVisibility();
  void updateTextSize();

private:
  void callback(const srv::GetSelection::Request::SharedPtr, const srv::GetSelection::Response::SharedPtr res);
  void updateText();
  void removeDisplays();

  rviz_common::properties::BoolProperty* lasso_mode_property_;
  rviz_common::properties::BoolProperty* close_loop_property_;
  rviz_common::properties::ColorProperty* pt_color_property_;
  rviz_common::properties::ColorProperty* line_color_property_;
  rviz_common::properties::FloatProperty* pt_size_property_;
  rviz_common::properties::BoolProperty* text_visibility_property_;
  rviz_common::properties::FloatProperty* text_size_property_;
  rviz_common::properties::FloatProperty* points_gap_;

  rclcpp::Service<srv::GetSelection>::SharedPtr server_;

  /** @brief */
  std::vector<std::vector<Ogre::Vector3>> points_;

  // Visualizations
  Ogre::SceneNode* points_node_;
  Ogre::SceneNode* lines_node_;
  Ogre::SceneNode* text_node_;
  Ogre::MaterialPtr points_material_;
  Ogre::MaterialPtr lines_material_;
};

}  // namespace rviz_polygon_selection_tool
