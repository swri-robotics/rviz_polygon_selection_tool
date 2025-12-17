#pragma once

#include <rviz_polygon_selection_tool/srv/get_selection.hpp>

#include <OgreMaterial.h>
#include <OgreVector.h>
#include <rviz_common/tool.hpp>
#include <rclcpp/service.hpp>

#include "version_check.hpp"
#ifdef CALLBACK_GROUP_SUPPORTED
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>
#endif

namespace rviz_rendering
{
class MaterialManager;
class MovableText;
}  // namespace rviz_rendering

namespace rviz_common
{
namespace properties
{
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
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
  void updateRenderAsOverlay();
  void updatePatchSize();
  void updateToolVisualization();

protected:
  virtual Ogre::MovableObject* createToolVisualization();
  static void updateMaterialColor(Ogre::MaterialPtr material, const QColor& color,
                                  const bool override_self_illumination = true);

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
  rviz_common::properties::FloatProperty* points_gap_size_property_;
  rviz_common::properties::BoolProperty* render_as_overlay_property_;
  rviz_common::properties::IntProperty* patch_size_property_;

#ifdef CALLBACK_GROUP_SUPPORTED
  std::thread executor_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::CallbackGroup::SharedPtr executor_callback_group_;
#endif

  rclcpp::Service<srv::GetSelection>::SharedPtr server_;

  /** @brief Polygon points in 3D space*/
  std::vector<std::vector<Ogre::Vector3>> points_;

  // Visualizations
  Ogre::SceneNode* points_node_{ nullptr };
  Ogre::SceneNode* lines_node_{ nullptr };
  Ogre::SceneNode* text_node_{ nullptr };
  Ogre::MaterialPtr points_material_{ nullptr };
  Ogre::MaterialPtr lines_material_{ nullptr };
  QCursor std_cursor_;
  QCursor hit_cursor_;
  Ogre::SceneNode* cursor_node_{ nullptr };
  Ogre::MovableObject* cursor_object_{ nullptr };
};

}  // namespace rviz_polygon_selection_tool
