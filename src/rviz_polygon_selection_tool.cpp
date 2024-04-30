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
  node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  server_ = node->create_service<srv::GetSelection>(
        "get_selection", std::bind(&PolygonSelectionTool::callback, this, std::placeholders::_1, std::placeholders::_2));
  pts_material_ = rviz_rendering::MaterialManager::createMaterialWithLighting("points_material");
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

  line_color_property_ = new rviz_common::properties::ColorProperty("Line Color", Qt::black, "Color of the line", 
                                                                  getPropertyContainer(), SLOT(updateLinesColor()), this);

  pt_size_property_ = new rviz_common::properties::FloatProperty("Point Size", 5.0, "Size of clicked points",
                                                                  getPropertyContainer(), SLOT(updatePtsSize()), this);
  newPolygon();
}

void PolygonSelectionTool::newPolygon()
{
  // Add the points visualization
  pts_vis_.push_back(scene_manager_->createManualObject("points_" + std::to_string(index)));
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(pts_vis_[index]);

  // Add the lines visualization
  lines_vis_.push_back(scene_manager_->createManualObject("lines_" + std::to_string(index)));
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(lines_vis_[index]);

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
      // Check if the index is within the range of points_
      if (index < int(points_.size())) {
        // Insert position into the inner vector at the current index
        points_[index].emplace_back(position);
      } else {
        // Create a new inner vector and insert position into it
        points_.emplace_back(1, position); // Create a vector with 1 element (position)
      }
      updateVisual();
    }
  }
  else if (event.middleUp())
  {
    // Clear the selection
    points_[index].clear();
    lines_vis_[index]->clear();
    pts_vis_[index]->clear();
  }
  else if (event.rightUp())
  {
    // return rviz_common::Tool::Finished;
    index += 1;
    newPolygon();
  }
  return rviz_common::Tool::Render;
}

int PolygonSelectionTool::processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel)
{
  switch (event->key() )
  {
    case Qt::Key_Delete:
      // Check if the vector is not null and clear its data
      if (!points_.empty()) {
        for (std::vector<Ogre::Vector3>& innerVec : points_) {
            // Check if the inner vector is not null and clear its data
            innerVec.clear();
        }
        points_.clear();
      }

      for (size_t i = 0; i < pts_vis_.size(); ++i) {
        if (pts_vis_[i]) {
          Ogre::SceneNode* parentNode = pts_vis_[i]->getParentSceneNode();
          if (parentNode) {
              // Detach the ManualObject from its parent scene node
              parentNode->detachObject(pts_vis_[i]);
              // Remove the ManualObject from the scene manager
              scene_manager_->destroyManualObject(pts_vis_[i]);
              // Remove the corresponding scene node from the scene manager
              scene_manager_->destroySceneNode(parentNode);
          }
        }
      }
      pts_vis_.clear();

      for (size_t i = 0; i < lines_vis_.size(); ++i) {
        if (lines_vis_[i]) {
          Ogre::SceneNode* parentNode = lines_vis_[i]->getParentSceneNode();
          if (parentNode) {
              // Detach the ManualObject from its parent scene node
              parentNode->detachObject(lines_vis_[i]);
              // Remove the ManualObject from the scene manager
              scene_manager_->destroyManualObject(lines_vis_[i]);
              // Remove the corresponding scene node from the scene manager
              scene_manager_->destroySceneNode(parentNode);
          }
        }
      }
      lines_vis_.clear();
      
      index = 0;
      newPolygon();
      break;
  }
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
  res->selection.reserve((index+1)); //reserve to the amount of polygons drawn in rviz

  for (int i = 0; i <= index; ++i) {
    res->selection.push_back(geometry_msgs::msg::PolygonStamped()); // Initialize each element in the selection array
    if(points_[i].empty()) continue;
    res->selection[i].header.frame_id = context_->getFixedFrame().toStdString();
    for (const Ogre::Vector3& pt : points_[i])
    {
      geometry_msgs::msg::Point32 msg;
      msg.x = pt.x;
      msg.y = pt.y;
      msg.z = pt.z;
      res->selection[i].polygon.points.push_back(msg);
    }
  }
}
  
void PolygonSelectionTool::updateVisual()
{
  pts_material_->setPointSize(pt_size_property_->getFloat());
  // Add the points to the display when not in lasso mode
  if (!lasso_mode_property_->getBool())
  {
    pts_vis_[index]->clear();
    pts_vis_[index]->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
    for (std::size_t i = 0; i < points_[index].size(); ++i)
    {
      pts_vis_[index]->position(points_[index][i]);
    }
    pts_vis_[index]->end();
    // Set the custom material
    pts_vis_[index]->setMaterial(0, pts_material_);
  }
  // Add the polygon lines
  if (points_[index].size() > 1)
  {
    lines_vis_[index]->clear();
    lines_vis_[index]->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    for (std::size_t i = 0; i < points_[index].size() - 1; ++i)
    {
      lines_vis_[index]->position(points_[index].at(i));
      lines_vis_[index]->position(points_[index].at(i + 1));
    }

    // Close the polygon
    if (points_[index].size() > 2 && close_loop_property_->getBool())
    {
      lines_vis_[index]->position(points_[index].back());
      lines_vis_[index]->position(points_[index].front());
    }

    lines_vis_[index]->end();

    lines_vis_[index]->setMaterial(0, lines_material_);
    }
}

}  // namespace rviz_polygon_selection_tool

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_polygon_selection_tool::PolygonSelectionTool, rviz_common::Tool)
