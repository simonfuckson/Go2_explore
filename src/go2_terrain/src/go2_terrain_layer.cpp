#include "go2_terrain/go2_terrain_layer.hpp"
#include "go2_terrain/terrain_algorithms.hpp"

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/thread/locks.hpp>
#include <costmap_2d/cost_values.h>

namespace go2_terrain
{

void Go2TerrainLayer::onInitialize()
{
  reloadConfiguration(true);
}

void Go2TerrainLayer::validateLayerContext() const
{
  if (layered_costmap_->isRolling())
  {
    throw std::runtime_error(name_ +
                             ": saved terrain is forbidden on a rolling costmap");
  }
  if (layered_costmap_->getGlobalFrameID() != "map")
  {
    throw std::runtime_error(name_ +
                             ": saved terrain requires global_frame=map");
  }
}

void Go2TerrainLayer::loadTerrain(const std::string& metadata_file)
{
  if (metadata_file.empty())
  {
    throw std::runtime_error(name_ + ": metadata_file is required when enabled");
  }

  TerrainMetadata metadata = loadTerrainMetadata(metadata_file);
  const boost::filesystem::path metadata_path(metadata_file);
  const boost::filesystem::path layer_path =
      metadata_path.parent_path() / metadata.cost.file;
  std::vector<std::uint8_t> costs =
      readUint8Layer(layer_path.string(), metadata.geometry.cellCount());

  metadata_ = std::move(metadata);
  terrain_costs_ = std::move(costs);
  metadata_file_ = metadata_file;
  terrain_loaded_ = true;

  ROS_INFO("%s loaded %zux%zu terrain costs from %s",
           name_.c_str(),
           metadata_.geometry.width,
           metadata_.geometry.height,
           layer_path.string().c_str());
}

void Go2TerrainLayer::reloadConfiguration(bool initial_load)
{
  ros::NodeHandle private_nh("~/" + name_);
  bool requested_enabled = initial_load ? true : enabled_;
  std::string requested_metadata = metadata_file_;
  private_nh.param("enabled", requested_enabled, requested_enabled);
  private_nh.param<std::string>("metadata_file",
                                requested_metadata,
                                requested_metadata);

  current_ = false;
  geometry_validated_ = false;
  if (requested_enabled)
  {
    // Set this before disk I/O. If an explicit reload fails and its caller
    // catches the exception, the old complete terrain data remains loaded and
    // is still scheduled for reapplication rather than silently disappearing.
    update_state_.requestFullUpdate();
  }
  if (requested_enabled)
  {
    validateLayerContext();
    // reset() is also the explicit configuration reload boundary. Loading
    // into temporaries in loadTerrain keeps the last complete layer intact if
    // the replacement metadata or binary layer is invalid.
    loadTerrain(requested_metadata);
  }
  enabled_ = requested_enabled;

  if (!enabled_)
  {
    // Noetic skips disabled plugins in both update phases. resetLayers()
    // clears the master before invoking reset(), so a disabled layer must not
    // wait for a callback that will never arrive.
    update_state_.clearFullUpdate();
  }
  current_ = !update_state_.fullUpdateRequired();
  if (!enabled_)
  {
    ROS_WARN("%s is disabled", name_.c_str());
  }
}

void Go2TerrainLayer::matchSize()
{
  geometry_validated_ = false;
  if (enabled_)
  {
    update_state_.requestFullUpdate();
  }
  else
  {
    // Resizing the master creates a fresh grid, so no old disabled-layer
    // costs remain to clear.
    update_state_.clearFullUpdate();
  }
  current_ = !update_state_.fullUpdateRequired();
}

void Go2TerrainLayer::reset()
{
  costmap_2d::Costmap2D* master_grid = layered_costmap_->getCostmap();
  if (master_grid == nullptr)
  {
    throw std::runtime_error(name_ + ": cannot reset without a master costmap");
  }
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(
      *master_grid->getMutex());
  reloadConfiguration(false);
}

void Go2TerrainLayer::validateMasterGeometry(
    const costmap_2d::Costmap2D& master_grid) const
{
  GridGeometry master;
  master.width = master_grid.getSizeInCellsX();
  master.height = master_grid.getSizeInCellsY();
  master.resolution = master_grid.getResolution();
  master.origin_x = master_grid.getOriginX();
  master.origin_y = master_grid.getOriginY();
  master.origin_yaw = 0.0;

  std::string error;
  if (!geometryMatches(metadata_.geometry, master, 1e-5, &error))
  {
    throw std::runtime_error(name_ + ": terrain/map geometry mismatch: " + error);
  }
}

void Go2TerrainLayer::updateBounds(double,
                                   double,
                                   double,
                                   double* min_x,
                                   double* min_y,
                                   double* max_x,
                                   double* max_y)
{
  if (!update_state_.fullUpdateRequired())
  {
    // The layer contributes no bounds during steady state. updateCosts still
    // receives and repairs the union dirty window requested by static,
    // obstacle, recovery, or inflation layers after the master resets it.
    return;
  }

  const costmap_2d::Costmap2D* master_grid = layered_costmap_->getCostmap();
  if (master_grid == nullptr || master_grid->getSizeInCellsX() == 0 ||
      master_grid->getSizeInCellsY() == 0)
  {
    current_ = false;
    ROS_ERROR_THROTTLE(2.0, "%s cannot request full bounds for an empty map",
                       name_.c_str());
    return;
  }
  const double origin_x = master_grid->getOriginX();
  const double origin_y = master_grid->getOriginY();
  *min_x = std::min(*min_x, origin_x);
  *min_y = std::min(*min_y, origin_y);
  *max_x = std::max(
      *max_x,
      origin_x + master_grid->getSizeInCellsX() * master_grid->getResolution());
  *max_y = std::max(
      *max_y,
      origin_y + master_grid->getSizeInCellsY() * master_grid->getResolution());
}

void Go2TerrainLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                  int min_i,
                                  int min_j,
                                  int max_i,
                                  int max_j)
{
  if (!enabled_)
  {
    update_state_.acknowledgeWindow(
        min_i, min_j, max_i, max_j,
        master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY());
    current_ = !update_state_.fullUpdateRequired();
    return;
  }
  if (!terrain_loaded_)
  {
    current_ = false;
    throw std::runtime_error(name_ + ": enabled without loaded terrain data");
  }
  if (!geometry_validated_)
  {
    validateMasterGeometry(master_grid);
    geometry_validated_ = true;
  }

  const int width = static_cast<int>(metadata_.geometry.width);
  const int height = static_cast<int>(metadata_.geometry.height);
  const int bounded_min_i = std::max(0, min_i);
  const int bounded_min_j = std::max(0, min_j);
  const int bounded_max_i = std::min(width, max_i);
  const int bounded_max_j = std::min(height, max_j);

  for (int y = bounded_min_j; y < bounded_max_j; ++y)
  {
    for (int x = bounded_min_i; x < bounded_max_i; ++x)
    {
      const std::size_t index = static_cast<std::size_t>(y) *
                                    metadata_.geometry.width +
                                static_cast<std::size_t>(x);
      const unsigned char master_value = master_grid.getCost(x, y);
      const unsigned char merged =
          mergeTerrainCost(master_value, terrain_costs_[index]);
      if (merged != master_value)
      {
        master_grid.setCost(x, y, merged);
      }
    }
  }

  update_state_.acknowledgeWindow(
      min_i, min_j, max_i, max_j,
      master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY());
  current_ = !update_state_.fullUpdateRequired();
}

}  // namespace go2_terrain

PLUGINLIB_EXPORT_CLASS(go2_terrain::Go2TerrainLayer, costmap_2d::Layer)
