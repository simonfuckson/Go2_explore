#pragma once

#include <string>
#include <vector>

#include <costmap_2d/layer.h>

#include "go2_terrain/static_layer_update.hpp"
#include "go2_terrain/terrain_grid.hpp"

namespace go2_terrain
{

class Go2TerrainLayer : public costmap_2d::Layer
{
public:
  Go2TerrainLayer() = default;

  void onInitialize() override;
  void matchSize() override;
  void reset() override;
  void updateBounds(double robot_x,
                    double robot_y,
                    double robot_yaw,
                    double* min_x,
                    double* min_y,
                    double* max_x,
                    double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid,
                   int min_i,
                   int min_j,
                   int max_i,
                   int max_j) override;

private:
  void validateLayerContext() const;
  void validateMasterGeometry(const costmap_2d::Costmap2D& master_grid) const;
  void loadTerrain(const std::string& metadata_file);
  void reloadConfiguration(bool initial_load);

  TerrainMetadata metadata_;
  std::vector<std::uint8_t> terrain_costs_;
  std::string metadata_file_;
  bool geometry_validated_ = false;
  bool terrain_loaded_ = false;
  StaticLayerUpdateState update_state_;
};

}  // namespace go2_terrain
