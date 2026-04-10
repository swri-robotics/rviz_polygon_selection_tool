#include "mesh_polygon_selection_tool.h"

#include <OgreEntity.h>
#include <OgreMesh.h>
#include <OgreMaterialManager.h>
#include <OgreMovableObject.h>
#include <OgreSceneManager.h>
#include <OgreSubEntity.h>
#include <rviz_rendering/mesh_loader.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/string_property.hpp>

const static std::string COLOR_NAME = "mesh_cursor_tool_color";
const static std::string DEFAULT_MESH_RESOURCE = "package://rviz_polygon_selection_tool/resources/default.stl";

#if !defined(RCLCPP_VERSION_MAJOR) || (RCLCPP_VERSION_MAJOR <= 28)
Ogre::MeshPtr loadMesh(const std::string& resource, const rviz_common::DisplayContext*)
{
  // Attempt to load the mesh
  Ogre::MeshPtr mesh = rviz_rendering::loadMeshFromResource(resource);
  if (!mesh)
  {
    // ROS_WARN("Loading default mesh...");

    // Load a default mesh
    mesh = rviz_rendering::loadMeshFromResource(DEFAULT_MESH_RESOURCE);
  }

  return mesh;
}
#else
#include <rviz_default_plugins/ros_resource_retriever.hpp>
#include <resource_retriever/retriever.hpp>
#include <rviz_common/display_context.hpp>

Ogre::MeshPtr loadMesh(const std::string& resource, rviz_common::DisplayContext* context)
{
  // Attempt to load the mesh
  resource_retriever::RetrieverVec plugins;
  if (context != nullptr)
  {
    std::shared_ptr node_ptr = context->getRosNodeAbstraction().lock();
    if (node_ptr != nullptr)
    {
      plugins.push_back(std::make_shared<RosResourceRetriever>(context->getRosNodeAbstraction()));
    }
  }
  for (const auto& plugin : resource_retriever::default_plugins())
  {
    plugins.push_back(plugin);
  }
  resource_retriever::Retriever ret = resource_retriever::Retriever(plugins);

  Ogre::MeshPtr mesh = rviz_rendering::loadMeshFromResource(&ret, resource);
  if (!mesh)
  {
    // ROS_WARN("Loading default mesh...");

    // Load a default mesh
    mesh = rviz_rendering::loadMeshFromResource(&ret, DEFAULT_MESH_RESOURCE);
  }

  return mesh;
}
#endif

namespace rviz_polygon_selection_tool
{
MeshPolygonSelectionTool::MeshPolygonSelectionTool() : PolygonSelectionTool()
{
  shortcut_key_ = 'm';

  mesh_file_ = new rviz_common::properties::StringProperty(
      "Mesh Filename", QString(DEFAULT_MESH_RESOURCE.c_str()), "The mesh resource to display as a cursor",
      getPropertyContainer(), SLOT(updateToolVisualization()), this);

  material_ = Ogre::MaterialManager::getSingletonPtr()->create(COLOR_NAME,
                                                               Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  color_property_ =
      new rviz_common::properties::ColorProperty("Color", QColor(255, 255, 255), "The color of the tool visualization",
                                                 getPropertyContainer(), SLOT(updateColor()), this);
  updateColor();
}

MeshPolygonSelectionTool::~MeshPolygonSelectionTool()
{
  Ogre::MaterialManager::getSingletonPtr()->remove(COLOR_NAME);
}

Ogre::MovableObject* MeshPolygonSelectionTool::createToolVisualization()
{
  Ogre::Entity* entity = scene_manager_->createEntity(loadMesh(mesh_file_->getStdString(), context_));
  for (unsigned i = 0; i < entity->getNumSubEntities(); ++i)
  {
    Ogre::SubEntity* sub = entity->getSubEntity(i);
    sub->setMaterial(material_);
  }

  return entity;
}

void MeshPolygonSelectionTool::updateColor()
{
  updateMaterialColor(material_, color_property_->getColor(), false);
}

}  // namespace rviz_polygon_selection_tool

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_polygon_selection_tool::MeshPolygonSelectionTool, rviz_common::Tool)
