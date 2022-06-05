#include "rviz_polygon_selection_tool.h"

namespace rviz_polygon_selection_tool
{
PolygonSelectionTool::PolygonSelectionTool() : rviz_common::Tool()
{
  shortcut_key_ = 'p';
}

void PolygonSelectionTool::onInitialize()
{
  rclcpp::Node::SharedPtr node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  server_ = node->create_service<srv::GetSelection>(
      "get_selection", std::bind(&PolygonSelectionTool::callback, this, std::placeholders::_1, std::placeholders::_2));

  pts_vis_ = scene_manager_->createManualObject("points");
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(pts_vis_);
  pts_material_ = rviz_rendering::MaterialManager::createMaterialWithLighting("points_material");
  pts_material_->setPointSize(5.0);
  PolygonSelectionTool::updatePtsColor(255, 255, 255, 255);

  lines_vis_ = scene_manager_->createManualObject("lines");

  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(lines_vis_);
  lines_material_ = rviz_rendering::MaterialManager::createMaterialWithLighting("lines_material");
  PolygonSelectionTool::updateLinesColor(0, 0, 0, 255);

  lasso_mode_property_ = new rviz_common::properties::BoolProperty(
      "Lasso mode", true, "Toggle between lasso and discrete click mode", getPropertyContainer());

  lasso_pt_color_property_ = new rviz_common::properties::ColorProperty(
      "Point Color", Qt::white, "Color of the points", getPropertyContainer(), SLOT(updatePtsColor()), this);

  lasso_line_color_property_ = new rviz_common::properties::ColorProperty(
      "Line Color", Qt::black, "Color of the line", getPropertyContainer(), SLOT(updateLinesColor()), this);

  lasso_pt_size_property_ =
      new rviz_common::properties::FloatProperty("Point Size", 5.0, "Size of clicked points", getPropertyContainer());
}

int PolygonSelectionTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
  // Collect the point
  if (event.leftUp() || (event.left() && lasso_mode_property_->getBool()))
  {
    Ogre::Vector3 position;
    if (context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, position))
    {
      points_.push_back(position);
      updateVisual();
    }
  }
  else if (event.middleUp())
  {
    // Clear the selection
    points_.clear();
    lines_vis_->clear();
    pts_vis_->clear();
  }
  else if (event.rightUp())
  {
    return rviz_common::Tool::Finished;
  }

  return rviz_common::Tool::Render;
}

void PolygonSelectionTool::updatePtsColor()
{
  auto q_color = lasso_pt_color_property_->getColor();
  int r, g, b, a;
  q_color.getRgb(&r, &g, &b, &a);
  PolygonSelectionTool::updatePtsColor(r, g, b, a);
}

void PolygonSelectionTool::updateLinesColor()
{
  auto q_color = lasso_line_color_property_->getColor();
  int r, g, b, a;
  q_color.getRgb(&r, &g, &b, &a);
  PolygonSelectionTool::updateLinesColor(r, g, b, a);
}

void PolygonSelectionTool::callback(const srv::GetSelection::Request::SharedPtr,
                                    const srv::GetSelection::Response::SharedPtr res)
{
  res->selection.reserve(points_.size());
  for (const Ogre::Vector3& pt : points_)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.frame_id = context_->getFixedFrame().toStdString();
    msg.point.x = pt.x;
    msg.point.y = pt.y;
    msg.point.z = pt.z;
    res->selection.push_back(msg);
  }
}

void PolygonSelectionTool::updateVisual()
{
  pts_material_->setPointSize(lasso_pt_size_property_->getFloat());

  // Add the points to the display when not in lasso mode
  if (!lasso_mode_property_->getBool())
  {
    pts_vis_->clear();
    pts_vis_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
    for (std::size_t i = 0; i < points_.size(); ++i)
    {
      pts_vis_->position(points_[i]);
    }
    pts_vis_->end();

    // Set the custom material
    pts_vis_->setMaterial(0, pts_material_);
  }

  // Add the polygon lines
  if (points_.size() > 1)
  {
    lines_vis_->clear();
    lines_vis_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    for (std::size_t i = 0; i < points_.size() - 1; ++i)
    {
      lines_vis_->position(points_.at(i));
      lines_vis_->position(points_.at(i + 1));
    }

    // Close the polygon
    if (points_.size() > 2)
    {
      lines_vis_->position(points_.back());
      lines_vis_->position(points_.front());
    }

    lines_vis_->end();

    lines_vis_->setMaterial(0, lines_material_);
  }
}

void PolygonSelectionTool::updatePtsColor(const int r, const int g, const int b, const int a)
{
  pts_material_->setDiffuse(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f,
                            static_cast<float>(b) / 255.0f, static_cast<float>(a) / 255.0f);
  pts_material_->setSpecular(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f,
                             static_cast<float>(b) / 255.0f, static_cast<float>(a) / 255.0f);
  pts_material_->setAmbient(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f,
                            static_cast<float>(b) / 255.0f);
}

void PolygonSelectionTool::updateLinesColor(const int r, const int g, const int b, const int a)
{
  lines_material_->setDiffuse(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f,
                              static_cast<float>(b) / 255.0f, static_cast<float>(a) / 255.0f);
  pts_material_->setSpecular(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f,
                             static_cast<float>(b) / 255.0f, static_cast<float>(a) / 255.0f);
  pts_material_->setAmbient(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f,
                            static_cast<float>(b) / 255.0f);
}

}  // namespace rviz_polygon_selection_tool

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_polygon_selection_tool::PolygonSelectionTool, rviz_common::Tool)
