#include "rviz_polygon_selection_tool.h"

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/interaction/view_picker.hpp>
#include <rviz_rendering/material_manager.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

static void updateMaterialColor(Ogre::MaterialPtr material, const QColor& color)
{
  qreal r, g, b, a;
  color.getRgbF(&r, &g, &b, &a);
  material->setDiffuse(r, g, b, a);
  material->setSpecular(r, g, b, a);
  material->setAmbient(r, g, b);
}

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

  // Add the points visualization
  pts_vis_ = scene_manager_->createManualObject("points");
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(pts_vis_);
  pts_material_ = rviz_rendering::MaterialManager::createMaterialWithLighting("points_material");

  // Add the lines visualization
  lines_vis_ = scene_manager_->createManualObject("lines");
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(lines_vis_);
  lines_material_ = rviz_rendering::MaterialManager::createMaterialWithLighting("lines_material");

  // Add the properties
  lasso_mode_property_ = new rviz_common::properties::BoolProperty(
      "Lasso mode", true, "Toggle between lasso and discrete click mode", getPropertyContainer());

  close_loop_property_ = new rviz_common::properties::BoolProperty("Close loop", true,
                                                                   "Close the polygon with a line between the last and "
                                                                   "first points",
                                                                   getPropertyContainer(), SLOT(updateVisual()), this);

  pt_color_property_ = new rviz_common::properties::ColorProperty("Point Color", Qt::black, "Color of the points",
                                                                  getPropertyContainer(), SLOT(updatePtsColor()), this);

  line_color_property_ = new rviz_common::properties::ColorProperty(
      "Line Color", Qt::black, "Color of the line", getPropertyContainer(), SLOT(updateLinesColor()), this);

  pt_size_property_ = new rviz_common::properties::FloatProperty("Point Size", 5.0, "Size of clicked points",
                                                                 getPropertyContainer(), SLOT(updatePtsSize()), this);

  // Update the materials
  updatePtsSize();
  updatePtsColor();
  updateLinesColor();
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
  return updateMaterialColor(pts_material_, pt_color_property_->getColor());
}

void PolygonSelectionTool::updateLinesColor()
{
  return updateMaterialColor(lines_material_, line_color_property_->getColor());
}

void PolygonSelectionTool::updatePtsSize()
{
  pts_material_->setPointSize(pt_size_property_->getFloat());
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
  pts_material_->setPointSize(pt_size_property_->getFloat());

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
    if (points_.size() > 2 && close_loop_property_->getBool())
    {
      lines_vis_->position(points_.back());
      lines_vis_->position(points_.front());
    }

    lines_vis_->end();

    lines_vis_->setMaterial(0, lines_material_);
  }
}

}  // namespace rviz_polygon_selection_tool

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_polygon_selection_tool::PolygonSelectionTool, rviz_common::Tool)
