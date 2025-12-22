#include "rviz_polygon_selection_tool.h"

#include <Eigen/Dense>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <OgreCamera.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <QKeyEvent>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/interaction/view_picker.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_rendering/material_manager.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

static Eigen::Matrix3f createMatrix(const Eigen::Vector3f& norm)
{
  Eigen::Matrix3f mat(Eigen::Matrix3f::Identity());
  mat.col(2) = norm;

  // Create plane from point normal
  Eigen::Hyperplane<float, 3> plane(norm, Eigen::Vector3f(0, 0, 0));

  // If the normal and global x-axis are not closely aligned
  if (std::abs(norm.dot(Eigen::Vector3f::UnitX())) < 0.90f)
  {
    // Project the global x-axis onto the plane to generate the x-axis
    Eigen::Vector3f x_axis = plane.projection(Eigen::Vector3f::UnitX()).normalized();
    mat.col(0) = x_axis;
    mat.col(1) = norm.cross(x_axis);
  }
  else
  {
    // Project the global y-axis onto the plane to generate the y-axis
    Eigen::Vector3f y_axis = plane.projection(Eigen::Vector3f::UnitY()).normalized();
    mat.col(0) = y_axis.cross(norm);
    mat.col(1) = y_axis;
  }

  return mat;
}

static Ogre::Quaternion estimateNormal(const std::vector<Ogre::Vector3>& points, const Ogre::Vector3& camera_norm)
{
  Eigen::MatrixXf data;
  data.resize(points.size(), 3);

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    data.row(i) = Eigen::Map<const Eigen::Vector3f>(points[i].ptr());
  }

  Eigen::MatrixXf centered = data.rowwise() - data.colwise().mean();

  // Use principal component analysis to the get eigenvectors
  Eigen::MatrixXf cov = centered.transpose() * centered;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);

  // Get the eigenvector associated with the smallest eigenvalue
  // (should be Z-axis, assuming points are relatively planar)
  Eigen::Vector3f norm = eig.eigenvectors().col(0);
  norm.normalize();

  Eigen::Vector3f camera_normal;
  camera_normal << camera_norm.x, camera_norm.y, camera_norm.z;
  camera_normal.normalize();

  if (norm.dot(camera_normal) < 0)
  {
    norm *= -1;
  }

  // Create an arbitrary orientation matrix with the normal being in the direction of the smallest eigenvector
  Eigen::Matrix3f mat = createMatrix(norm);

  Eigen::Quaternionf q(mat);  // Eigen::AngleAxisf(0.0, evecs.col(2)));
  Ogre::Quaternion out;
  out.w = q.w();
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();

  return out;
}

namespace rviz_polygon_selection_tool
{
void PolygonSelectionTool::updateMaterialColor(Ogre::MaterialPtr material, const QColor& color,
                                               const bool override_self_illumination)
{
  qreal r, g, b, a;
  color.getRgbF(&r, &g, &b, &a);
  material->setDiffuse(r, g, b, a);
  material->setSpecular(r, g, b, a);
  material->setAmbient(r, g, b);
  if (override_self_illumination)
    material->setSelfIllumination(r, g, b);
}

PolygonSelectionTool::PolygonSelectionTool() : rviz_common::Tool()
{
  shortcut_key_ = 'p';
}

PolygonSelectionTool::~PolygonSelectionTool()
{
#ifdef CALLBACK_GROUP_SUPPORTED
  executor_.cancel();
  executor_thread_.join();
#endif

  // Remove materials
  Ogre::MaterialManager::getSingleton().remove(points_material_);
  Ogre::MaterialManager::getSingleton().remove(lines_material_);

  // Remove displays
  removeDisplays();

  // Remove top level scene nodes
  scene_manager_->getRootSceneNode()->removeAndDestroyChild(points_node_);
  scene_manager_->getRootSceneNode()->removeAndDestroyChild(lines_node_);
  scene_manager_->getRootSceneNode()->removeAndDestroyChild(text_node_);

  if (cursor_node_->getParentSceneNode())
    cursor_node_->getParentSceneNode()->removeChild(cursor_node_);
  scene_manager_->destroySceneNode(cursor_node_);
}

void PolygonSelectionTool::onInitialize()
{
  // Add the properties
  service_name_property_ = new rviz_common::properties::StringProperty(
      "Service name", "get_selection", "The name of the service to get the selection points", getPropertyContainer(),
      SLOT(updateServiceName()), this);

  lasso_mode_property_ = new rviz_common::properties::BoolProperty(
      "Lasso mode", true, "Toggle between lasso and discrete click mode", getPropertyContainer());

  close_loop_property_ = new rviz_common::properties::BoolProperty("Close loop", true,
                                                                   "Close the polygon with a line between the last and "
                                                                   "first points",
                                                                   getPropertyContainer(), SLOT(updateVisual()), this);

  pt_color_property_ = new rviz_common::properties::ColorProperty(
      "Point Color", Qt::black, "Color of the points", getPropertyContainer(), SLOT(updatePointsColor()), this);

  line_color_property_ = new rviz_common::properties::ColorProperty(
      "Line Color", Qt::black, "Color of the line", getPropertyContainer(), SLOT(updateLinesColor()), this);

  pt_size_property_ = new rviz_common::properties::FloatProperty(
      "Point Size", 5.0, "Size of clicked points", getPropertyContainer(), SLOT(updatePointsSize()), this);

  text_visibility_property_ =
      new rviz_common::properties::BoolProperty("Show Text", true, "Toggles the visibility of the text display",
                                                getPropertyContainer(), SLOT(updateTextVisibility()), this);

  text_size_property_ = new rviz_common::properties::FloatProperty(
      "Text Size", 0.015, "Height of the text display (m)", getPropertyContainer(), SLOT(updateTextSize()), this);

  points_gap_size_property_ = new rviz_common::properties::FloatProperty(
      "Point Generation Gap", 0.002, "Separation between adjacent points in a polygon (m)", getPropertyContainer());

  render_as_overlay_property_ = new rviz_common::properties::BoolProperty(
      "Render as Overlay", false, "Render the polygon points and lines on top of all other geometry",
      getPropertyContainer(), SLOT(updateRenderAsOverlay()), this);

  patch_size_property_ =
      new rviz_common::properties::IntProperty("Patch size", 15, "Patch size (pixels) for cursor normal estimation",
                                               getPropertyContainer(), nullptr, nullptr, 3);

  // Configure the ROS service interface
  updateServiceName();

  // Configure the displays
  static int count = 0;
  points_material_ =
      rviz_rendering::MaterialManager::createMaterialWithLighting("points_material_" + std::to_string(count));
  lines_material_ =
      rviz_rendering::MaterialManager::createMaterialWithLighting("lines_material_" + std::to_string(count));
  ++count;

  points_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  lines_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  text_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  newPolygon();

  // Initialize the scene node
  cursor_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Create the visual tool object
  cursor_object_ = createToolVisualization();

  // Attach the tool visualization to the scene
  cursor_node_->attachObject(cursor_object_);
  cursor_node_->setVisible(false);

  // Set the cursors
  hit_cursor_ = cursor_;
  std_cursor_ = rviz_common::getDefaultCursor();
}

void PolygonSelectionTool::activate()
{
  cursor_node_->setVisible(true);
}

void PolygonSelectionTool::deactivate()
{
  updateText();
  cursor_node_->setVisible(false);
}

void PolygonSelectionTool::newPolygon()
{
  // Add a new empty selection vector
  points_.push_back({});

  // Add the points visualization
  Ogre::ManualObject* points = scene_manager_->createManualObject();
  points_node_->attachObject(points);

  // Add the lines visualization
  Ogre::ManualObject* lines = scene_manager_->createManualObject();
  lines_node_->attachObject(lines);

  // Add the text
  std::string caption = "#" + std::to_string(points_.size());
  auto* text = new rviz_rendering::MovableText(caption, "Liberation Sans", text_size_property_->getFloat(),
                                               Ogre::ColourValue::Blue);
  text->setVisible(false);
  text->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_ABOVE);

  // Attach the text to a movable child scene node underneath the main text scene node
  text_node_->createChildSceneNode()->attachObject(text);

  // Update the materials
  updatePointsSize();
  updatePointsColor();
  updateLinesColor();
  updateTextSize();
  updateRenderAsOverlay();
}

void PolygonSelectionTool::removeDisplays()
{
  // Remove the displays
  // Points
  {
    std::vector<Ogre::MovableObject*> objects = points_node_->getAttachedObjects();
    points_node_->removeAndDestroyAllChildren();

    // Delete the objects attached to this node
    for (auto object : objects)
      scene_manager_->destroyManualObject(object->getName());
  }

  // Lines
  {
    std::vector<Ogre::MovableObject*> objects = lines_node_->getAttachedObjects();
    lines_node_->removeAndDestroyAllChildren();

    // Delete the objects attached to this node
    for (Ogre::MovableObject* object : objects)
      scene_manager_->destroyManualObject(object->getName());
  }

  // Text
  for (Ogre::Node* child : text_node_->getChildren())
  {
    auto* child_scene = dynamic_cast<Ogre::SceneNode*>(child);
    std::vector<Ogre::MovableObject*> objects = child_scene->getAttachedObjects();
    child_scene->removeAndDestroyAllChildren();

    for (Ogre::MovableObject* object : objects)
      delete object;
  }
  text_node_->removeAndDestroyAllChildren();
}

int PolygonSelectionTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
  // Set the visibility of this node off so the selection manager won't choose a point on our cursor mesh in the point
  // and patch
  cursor_node_->setVisible(false);

  Ogre::Vector3 position;
  bool got_point = context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, position);

  std::vector<Ogre::Vector3> points;
  const unsigned patch_size = static_cast<unsigned>(patch_size_property_->getInt());
  bool got_patch =
      context_->getViewPicker()->get3DPatch(event.panel, event.x, event.y, patch_size, patch_size, true, points);

  // Revisualize the cursor node
  cursor_node_->setVisible(true);

  // Set the cursor
  if (got_point && got_patch)
  {
    setCursor(hit_cursor_);

    // Update the cursor position
    Ogre::Vector3 camera_direction =
        event.panel->getViewController()->getCamera()->getParentSceneNode()->getLocalAxes().GetColumn(2);
    camera_direction *= -1.0;
    Ogre::Quaternion q = estimateNormal(points, camera_direction);
    cursor_node_->setOrientation(q);
    cursor_node_->setPosition(position);
  }
  else
  {
    setCursor(std_cursor_);
  }

  // Collect the point
  if (event.leftUp() || (event.left() && lasso_mode_property_->getBool()))
  {
    if (got_point)
    {
      if (!points_.back().empty())
      {
        const Ogre::Vector3& last_point = points_.back().back();
        if (last_point.squaredDistance(position) < std::pow(points_gap_size_property_->getFloat(), 2.0))
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
      // Clear the points data
      points_.back().clear();

      // Clear the points visualization
      {
        auto* points = dynamic_cast<Ogre::ManualObject*>(points_node_->getAttachedObjects().back());
        if (points)
          points->clear();
      }

      // Clear the lines visualization
      {
        auto* lines = dynamic_cast<Ogre::ManualObject*>(lines_node_->getAttachedObjects().back());
        if (lines)
          lines->clear();
      }

      // Make the text object invisible
      {
        auto* text_node = dynamic_cast<Ogre::SceneNode*>(text_node_->getChildren().back());
        if (text_node)
          text_node->setVisible(false);
      }
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

int PolygonSelectionTool::processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* /*panel*/)
{
  switch (event->key())
  {
    case Qt::Key_Delete:
    case Qt::Key_Backspace:
      points_.clear();
      removeDisplays();
      newPolygon();
      break;
    default:
      break;
  }

  return rviz_common::Tool::Render;
}

void PolygonSelectionTool::updateServiceName()
{
  const std::string service_name = service_name_property_->getStdString();

  rclcpp::Node::SharedPtr node = context_->getRosNodeAbstraction().lock()->get_raw_node();

#ifdef CALLBACK_GROUP_SUPPORTED
  executor_.cancel();
  if (executor_thread_.joinable())
    executor_thread_.join();

  executor_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  executor_.add_callback_group(executor_callback_group_, node->get_node_base_interface());

#ifdef QOS_REQUIRED_IN_SERVICE
  rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT, 1),
                  rmw_qos_profile_services_default);
  server_ = node->create_service<srv::GetSelection>(
      service_name, std::bind(&PolygonSelectionTool::callback, this, std::placeholders::_1, std::placeholders::_2), qos,
      executor_callback_group_);
#else
  server_ = node->create_service<srv::GetSelection>(
      service_name, std::bind(&PolygonSelectionTool::callback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, executor_callback_group_);
#endif

  executor_thread_ = std::thread([&]() { executor_.spin(); });
#else
  server_ = node->create_service<srv::GetSelection>(
      service_name, std::bind(&PolygonSelectionTool::callback, this, std::placeholders::_1, std::placeholders::_2));
#endif
}

void PolygonSelectionTool::updatePointsColor()
{
  return updateMaterialColor(points_material_, pt_color_property_->getColor());
}

void PolygonSelectionTool::updateLinesColor()
{
  return updateMaterialColor(lines_material_, line_color_property_->getColor());
}

void PolygonSelectionTool::updatePointsSize()
{
  points_material_->setPointSize(pt_size_property_->getFloat());
}

void PolygonSelectionTool::updateTextSize()
{
  const float height = text_size_property_->getFloat();
  for (Ogre::Node* child : text_node_->getChildren())
  {
    auto child_scene = dynamic_cast<Ogre::SceneNode*>(child);
    if (child_scene)
    {
      for (Ogre::MovableObject* child_scene_object : child_scene->getAttachedObjects())
      {
        auto* text = dynamic_cast<rviz_rendering::MovableText*>(child_scene_object);
        if (text)
          text->setCharacterHeight(height);
      }
    }
  }
}

void PolygonSelectionTool::updateTextVisibility()
{
  const bool text_visible = text_visibility_property_->getBool();
  for (Ogre::Node* child : text_node_->getChildren())
  {
    auto* child_scene = dynamic_cast<Ogre::SceneNode*>(child);
    if (child_scene)
      child_scene->setVisible(text_visible);
  }
  text_size_property_->setHidden(!text_visible);
}

void PolygonSelectionTool::updateText()
{
  const bool show = points_.back().size() > 1;
  if (!show)
    return;

  // Get the last child scene node from the top-level text scene node
  auto* last_child_scene = dynamic_cast<Ogre::SceneNode*>(text_node_->getChildren().back());
  if (!last_child_scene || last_child_scene->numAttachedObjects() != 1)
    return;

  // Get the first attached object of the last child scene object
  auto* text = dynamic_cast<rviz_rendering::MovableText*>(last_child_scene->getAttachedObject(0));
  if (text == nullptr)
    return;

  text->setVisible(true);

  // Set the position of the text node as the center of the points
  Ogre::Vector3 text_pos(0.0, 0.0, 0.0);
  for (const Ogre::Vector3& point : points_.back())
    text_pos += point;
  text_pos /= static_cast<Ogre::Real>(points_.back().size());

  last_child_scene->setPosition(text_pos);
}

void PolygonSelectionTool::updateRenderAsOverlay()
{
  const bool render_as_overlay = render_as_overlay_property_->getBool();
  Ogre::RenderQueueGroupID render_id =
      render_as_overlay ? Ogre::RenderQueueGroupID::RENDER_QUEUE_OVERLAY : Ogre::RenderQueueGroupID::RENDER_QUEUE_MAIN;

  // Lines
  lines_material_->setDepthCheckEnabled(!render_as_overlay);
  lines_material_->setDepthWriteEnabled(!render_as_overlay);
  for (auto object : lines_node_->getAttachedObjects())
  {
    auto lines = dynamic_cast<Ogre::ManualObject*>(object);
    if (!lines)
      continue;
    lines->setRenderQueueGroup(render_id);
  }

  // Points
  points_material_->setDepthCheckEnabled(!render_as_overlay);
  points_material_->setDepthWriteEnabled(!render_as_overlay);
  for (auto object : points_node_->getAttachedObjects())
  {
    auto points = dynamic_cast<Ogre::ManualObject*>(object);
    if (!points)
      continue;
    points->setRenderQueueGroup(render_id);
  }

  // Text
  for (Ogre::Node* child : text_node_->getChildren())
  {
    auto scene_node = dynamic_cast<Ogre::SceneNode*>(child);
    if (!scene_node)
      continue;

    for (auto object : scene_node->getAttachedObjects())
    {
      auto text = dynamic_cast<rviz_rendering::MovableText*>(object);
      if (!text)
        continue;

      text->setRenderQueueGroup(render_id);
      text->getMaterial()->setDepthCheckEnabled(!render_as_overlay);
      text->getMaterial()->setDepthWriteEnabled(!render_as_overlay);
    }
  }
}

void PolygonSelectionTool::updateToolVisualization()
{
  // Remove and destroy the first tool visualization from the scene node
  scene_manager_->destroyMovableObject(cursor_object_);

  // Create the new tool visualization
  cursor_object_ = createToolVisualization();

  // Attach the new tool visualization to the scene node
  cursor_node_->attachObject(cursor_object_);
  cursor_node_->setVisible(false);
}

Ogre::MovableObject* PolygonSelectionTool::createToolVisualization()
{
  return scene_manager_->createManualObject();
}

void PolygonSelectionTool::callback(const srv::GetSelection::Request::SharedPtr /*req*/,
                                    const srv::GetSelection::Response::SharedPtr res)
{
  res->selection.reserve(points_.size());
  for (std::size_t i = 0; i < points_.size(); ++i)
  {
    // Skip selections with fewer than 3 points
    if (points_[i].size() < 3)
      continue;

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
  points_material_->setPointSize(pt_size_property_->getFloat());

  Ogre::ManualObject* points = dynamic_cast<Ogre::ManualObject*>(points_node_->getAttachedObjects().back());
  if (!points)
    return;

  Ogre::ManualObject* lines = dynamic_cast<Ogre::ManualObject*>(lines_node_->getAttachedObjects().back());
  if (!lines)
    return;

  // Add the points to the display when not in lasso mode
  if (!lasso_mode_property_->getBool())
  {
    points->clear();
    points->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
    for (std::size_t i = 0; i < points_.back().size(); ++i)
    {
      points->position(points_.back()[i]);
    }
    points->end();
    // Set the custom material (Ignore this: class "Ogre::ManualObject" has no member "setMaterial")
    points->setMaterial(0, points_material_);
  }

  // Add the polygon lines
  if (int(points_.back().size()) > 1)
  {
    lines->clear();
    lines->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    for (std::size_t i = 0; i < points_.back().size() - 1; ++i)
    {
      lines->position(points_.back().at(i));
      lines->position(points_.back().at(i + 1));
    }

    // Close the polygon
    if (int(points_.back().size()) > 2 && close_loop_property_->getBool())
    {
      lines->position(points_.back().back());
      lines->position(points_.back().front());
    }

    lines->end();
    // Set the custom material (Ignore this: class "Ogre::ManualObject" has no member "setMaterial")
    lines->setMaterial(0, lines_material_);
  }
}

}  // namespace rviz_polygon_selection_tool

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_polygon_selection_tool::PolygonSelectionTool, rviz_common::Tool)
