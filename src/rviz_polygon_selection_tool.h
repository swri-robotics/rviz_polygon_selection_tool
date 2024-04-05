#include <rviz_polygon_selection_tool/srv/get_selection.hpp>

#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <rviz_common/tool.hpp>
#include <rclcpp/service.hpp>
#include <qevent.h>

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

  void onInitialize() override;
  void activate() override{};
  void deactivate() override{};
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
  int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel) override;

public Q_SLOTS:
  void updatePtsColor();
  void updatePtsSize();
  void updateLinesColor();
  void updateVisual();

private:
  static constexpr int num_selections_ = 3;
  int sel = 0;
  void callback(const srv::GetSelection::Request::SharedPtr, const srv::GetSelection::Response::SharedPtr res);

  rviz_common::properties::BoolProperty* lasso_mode_property_;
  rviz_common::properties::BoolProperty* close_loop_property_;
  rviz_common::properties::ColorProperty* pt_color_property_;
  rviz_common::properties::ColorProperty* line_color_property_;
  rviz_common::properties::FloatProperty* pt_size_property_;

  rclcpp::Service<srv::GetSelection>::SharedPtr server_[num_selections_];

  std::vector<Ogre::Vector3> points_[num_selections_];
  Ogre::ManualObject* pts_vis_[num_selections_]{ nullptr };
  Ogre::ManualObject* lines_vis_[num_selections_]{ nullptr };
  Ogre::MaterialPtr pts_material_{ nullptr };
  Ogre::MaterialPtr lines_material_{ nullptr };
};

}  // namespace rviz_polygon_selection_tool
