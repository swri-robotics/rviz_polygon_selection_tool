#include <rviz_polygon_selection_tool/srv/get_selection.hpp>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz_common/tool.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/interaction/view_picker.hpp>
#include <rviz_rendering/material_manager.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rclcpp/service.hpp>

namespace rviz_polygon_selection_tool
{
class PolygonSelectionTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  PolygonSelectionTool();

  void onInitialize() override;

  void activate() override{};

  void deactivate() override{};

  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

public Q_SLOTS:
  void updatePtsColor();

  void updateLinesColor();

private:
  void callback(const srv::GetSelection::Request::SharedPtr, const srv::GetSelection::Response::SharedPtr res);

  void updateVisual();

  void updatePtsColor(const int r, const int g, const int b, const int a);

  void updateLinesColor(const int r, const int g, const int b, const int a);

  rviz_common::properties::BoolProperty* lasso_mode_property_;
  rviz_common::properties::ColorProperty* lasso_pt_color_property_;
  rviz_common::properties::ColorProperty* lasso_line_color_property_;
  rviz_common::properties::FloatProperty* lasso_pt_size_property_;
  rclcpp::Service<srv::GetSelection>::SharedPtr server_;
  std::vector<Ogre::Vector3> points_;
  Ogre::ManualObject* pts_vis_{ nullptr };
  Ogre::ManualObject* lines_vis_{ nullptr };
  Ogre::MaterialPtr pts_material_{ nullptr };
  Ogre::MaterialPtr lines_material_{ nullptr };
};

}  // namespace rviz_polygon_selection_tool
