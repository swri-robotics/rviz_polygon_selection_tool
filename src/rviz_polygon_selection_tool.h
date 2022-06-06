//#include <rviz_polygon_selection_tool/srv/get_selection.hpp>
#include <rviz_polygon_selection_tool/msg/selection.hpp>

#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <rviz_common/tool.hpp>
//#include <rclcpp/service.hpp>
#include <rclcpp/publisher.hpp>

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

public Q_SLOTS:
  void updatePtsColor();
  void updatePtsSize();
  void updateLinesColor();

private:
  //  void callback(const srv::GetSelection::Request::SharedPtr, const srv::GetSelection::Response::SharedPtr res);
  void publish();
  void updateVisual();
  void updatePtsColor(const int r, const int g, const int b, const int a);
  void updateLinesColor(const int r, const int g, const int b, const int a);

  rviz_common::properties::BoolProperty* lasso_mode_property_;
  rviz_common::properties::ColorProperty* pt_color_property_;
  rviz_common::properties::ColorProperty* line_color_property_;
  rviz_common::properties::FloatProperty* pt_size_property_;

  //  rclcpp::Service<srv::GetSelection>::SharedPtr server_;
  rclcpp::Publisher<msg::Selection>::SharedPtr publisher_;

  std::vector<Ogre::Vector3> points_;
  Ogre::ManualObject* pts_vis_{ nullptr };
  Ogre::ManualObject* lines_vis_{ nullptr };
  Ogre::MaterialPtr pts_material_{ nullptr };
  Ogre::MaterialPtr lines_material_{ nullptr };
};

}  // namespace rviz_polygon_selection_tool
