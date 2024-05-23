#include "rviz_polygon_selection_tool.h"

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <QKeyEvent>
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/interaction/view_picker.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_rendering/material_manager.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

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

  text_visibility_property_ = new rviz_common::properties::BoolProperty("Show Text", true, "Toggles the visibility of the text display", 
                                                                  getPropertyContainer(), SLOT(updateTextVisibility()), this);

  text_size_property_ = new rviz_common::properties::FloatProperty("Text Size", 0.015, "Height of the text display (m)", 
                                                                  getPropertyContainer(), SLOT(updateTextSize()), this);

  points_gap_ = new rviz_common::properties::FloatProperty("Point Generation Gap", 0.002, "Separation between adjacent points in a polygon (m)",
                                                           getPropertyContainer());
  
  start_text_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  
  newPolygon();
}

void PolygonSelectionTool::newPolygon()
{
  // Add a new empty selection vector
  points_.push_back({});
  std::size_t index = points_.size() - 1;

  // Add the points visualization
  Ogre::ManualObject* pts = scene_manager_->createManualObject("points_" + std::to_string(index));
  pts_vis_.push_back(pts);
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(pts);

  // Add the lines visualization
  Ogre::ManualObject* lines = scene_manager_->createManualObject("lines_" + std::to_string(index));
  lines_vis_.push_back(lines);
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(lines);

  // Add the text
  auto* text = new rviz_rendering::MovableText("not initialized", "Liberation Sans", 0.05, Ogre::ColourValue::Blue);
  text->setVisible(false);
  text->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_ABOVE);
  texts_.push_back(text);

  Ogre::SceneNode* node = start_text_node_->createChildSceneNode();
  node->attachObject(text);
  text_node_.push_back(node);

  // Update the materials
  updatePtsSize();
  updatePtsColor();
  updateLinesColor();
  updateTextSize();
}

int PolygonSelectionTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
  // Collect the point
  if (event.leftUp() || (event.left() && lasso_mode_property_->getBool()))
  {
    Ogre::Vector3 position;
    if (context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, position))
    {
      if(lasso_mode_property_->getBool() && !points_.back().empty())
      {
        const Ogre::Vector3& last_point = points_.back().back();
        if(last_point.squaredDistance(position) < std::pow(points_gap_->getFloat(), 2.0))
          return rviz_common::Tool::Render;
      }
      points_.back().push_back(position);
      updateVisual();
    }
  }
  else if (event.middleUp())
  {
    // Clear the selection
    if (points_.back().size() > 1)
    {
      points_.back().clear();
      lines_vis_.back()->clear();
      pts_vis_.back()->clear();
    }
  }
  else if (event.rightUp())
  {
    if (!points_.back().empty())
    {
      updateText();
      newPolygon();
    }
  }
  return rviz_common::Tool::Render;
}

// int PolygonSelectionTool::processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* /*panel*/)
// {
//   switch (event->key() )
//   {
//     case Qt::Key_Delete:
//       // Check if the vector is not null and clear its data
//       if (!points_.empty())
//       {
//         for (std::vector<Ogre::Vector3>& innerVec : points_) {
//             // Check if the inner vector is not null and clear its data
//             innerVec.clear();
//         }
//         points_.clear();
//       }

//       for (size_t i = 0; i < pts_vis_.size(); ++i)
//       {
//         if (pts_vis_[i]) {
//           Ogre::SceneNode* parentNode = pts_vis_[i]->getParentSceneNode();
//           if (parentNode) {
//               // Detach the ManualObject from its parent scene node
//               parentNode->detachObject(pts_vis_[i]);
//               // Remove the ManualObject from the scene manager
//               scene_manager_->destroyManualObject(pts_vis_[i]);
//               // Remove the corresponding scene node from the scene manager
//               scene_manager_->destroySceneNode(parentNode);
//           }
//         }
//       }
//       pts_vis_.clear();

//       for (size_t i = 0; i < lines_vis_.size(); ++i)
//       {
//         if (lines_vis_[i])
//         {
//           Ogre::SceneNode* parentNode = lines_vis_[i]->getParentSceneNode();
//           if (parentNode) {
//               // Detach the ManualObject from its parent scene node
//               parentNode->detachObject(lines_vis_[i]);
//               // Remove the ManualObject from the scene manager
//               scene_manager_->destroyManualObject(lines_vis_[i]);
//               // Remove the corresponding scene node from the scene manager
//               scene_manager_->destroySceneNode(parentNode);
//           }
//         }
//       }
//       lines_vis_.clear();

//       for (Ogre::SceneNode* node : text_node_)
//       {
//         node->detachAllObjects();
//       }
//       text_node_.clear();
//       texts_.clear();

      
//       index_ = 0;
//       newPolygon();
//       break;
//   }
// }

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

void PolygonSelectionTool::updateTextSize()
{
  const float height = text_size_property_->getFloat();
  for (rviz_rendering::MovableText* text : texts_)
  {
    text->setCharacterHeight(height);
  }
}

void PolygonSelectionTool::updateTextVisibility()
{
  const bool text_visible = text_visibility_property_->getBool();
  for (Ogre::SceneNode* node : text_node_)
  {
    node->setVisible(text_visible);
  }
  text_size_property_->setHidden(!text_visible);
}

void PolygonSelectionTool::updateText()
{
  const bool show = points_.back().size() > 1;
  text_node_.back()->setVisible(show);
  if (show)
  {
    // Update the caption of the text
    texts_.back()->setCaption("Polygon#" + std::to_string(texts_.size() - 1));

    // Set the position of the text node as the center of the points
    Ogre::Vector3 text_pos(0.0, 0.0, 0.0);
    for (const Ogre::Vector3& point : points_.back())
      text_pos += point;
    text_pos /= static_cast<Ogre::Real>(points_.back().size());

    text_node_.back()->setPosition(text_pos);
  }
}

void PolygonSelectionTool::callback(const srv::GetSelection::Request::SharedPtr /*req*/,
                                    const srv::GetSelection::Response::SharedPtr res)
{
  res->selection.reserve(points_.size());
  for (std::size_t i = 0; i < points_.size(); ++i)
  {
    // Skip selections with fewer than 3 points
    if(points_[i].size() < 3) continue;

    geometry_msgs::msg::PolygonStamped polygon;
    polygon.header.frame_id = context_->getFixedFrame().toStdString();
    for (const Ogre::Vector3& pt : points_[i])
    {
      geometry_msgs::msg::Point32 msg;
      msg.x = pt.x;
      msg.y = pt.y;
      msg.z = pt.z;
      polygon.polygon.points.push_back(msg);
    }

    res->selection.push_back(polygon);
  }
}
  
void PolygonSelectionTool::updateVisual()
{
  pts_material_->setPointSize(pt_size_property_->getFloat());
  // Add the points to the display when not in lasso mode
  if (!lasso_mode_property_->getBool())
  {
    pts_vis_.back()->clear();
    pts_vis_.back()->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
    for (std::size_t i = 0; i < points_.back().size(); ++i)
    {
      pts_vis_.back()->position(points_.back()[i]);
    }
    pts_vis_.back()->end();
    // Set the custom material (Ignore this: class "Ogre::ManualObject" has no member "setMaterial")
    pts_vis_.back()->setMaterial(0, pts_material_);
  }
  // Add the polygon lines
  if (int(points_.back().size()) > 1)
  {
    lines_vis_.back()->clear();
    lines_vis_.back()->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    for (std::size_t i = 0; i < points_.back().size() - 1; ++i)
    {
      lines_vis_.back()->position(points_.back().at(i));
      lines_vis_.back()->position(points_.back().at(i + 1));
    }

    // Close the polygon
    if (int(points_.back().size()) > 2 && close_loop_property_->getBool())
    {
      lines_vis_.back()->position(points_.back().back());
      lines_vis_.back()->position(points_.back().front());
    }

    lines_vis_.back()->end();
    // Set the custom material (Ignore this: class "Ogre::ManualObject" has no member "setMaterial")
    lines_vis_.back()->setMaterial(0, lines_material_);
  }
}

}  // namespace rviz_polygon_selection_tool

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_polygon_selection_tool::PolygonSelectionTool, rviz_common::Tool)
