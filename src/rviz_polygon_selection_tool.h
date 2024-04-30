#include <rviz_polygon_selection_tool/srv/get_selection.hpp>

#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <rviz_common/tool.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/node.hpp>
#include <qevent.h>
#include <vector>
#include <geometry_msgs/msg/polygon_stamped.hpp>

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
  void newPolygon();
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
  int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel) override;

public Q_SLOTS:
  void updatePtsColor();
  void updatePtsSize();
  void updateLinesColor();
  void updateVisual();

private:
  int index = 0;
  rclcpp::Node::SharedPtr node;
  void callback(const srv::GetSelection::Request::SharedPtr, const srv::GetSelection::Response::SharedPtr res);

  rviz_common::properties::BoolProperty* lasso_mode_property_;
  rviz_common::properties::BoolProperty* close_loop_property_;
  rviz_common::properties::ColorProperty* pt_color_property_;
  rviz_common::properties::ColorProperty* line_color_property_;
  rviz_common::properties::FloatProperty* pt_size_property_;

  rclcpp::Service<srv::GetSelection>::SharedPtr server_;

  std::vector<std::vector<Ogre::Vector3>> points_;
  std::vector<Ogre::ManualObject*> pts_vis_;
  std::vector<Ogre::ManualObject*> lines_vis_;
  Ogre::MaterialPtr pts_material_;
  Ogre::MaterialPtr lines_material_;
};

}  // namespace rviz_polygon_selection_tool
