#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace go2_terrain
{

constexpr float kUnknownTerrain = -3.402823466e+38F;

struct GridGeometry
{
  std::size_t width = 0;
  std::size_t height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;

  std::size_t cellCount() const { return width * height; }
  bool valid() const;
};

struct TerrainGrid
{
  GridGeometry geometry;
  std::vector<float> elevation;
  std::vector<float> slope_deg;
  std::vector<float> roughness;
  std::vector<float> step;
  std::vector<std::uint8_t> cost;
  std::vector<std::uint8_t> confidence;

  void resize(const GridGeometry& geometry_in);
  bool valid() const;
};

struct LayerDescriptor
{
  std::string file;
  std::string type;
  std::string unit;
};

struct TerrainMetadata
{
  std::string format;
  int version = 0;
  std::string export_id;
  std::string frame_id;
  GridGeometry geometry;
  std::string image_file;
  double base_to_floor_m = 0.0;
  LayerDescriptor elevation;
  LayerDescriptor slope;
  LayerDescriptor roughness;
  LayerDescriptor step;
  LayerDescriptor cost;
  LayerDescriptor confidence;
};

bool isKnown(float value);
bool nearlyEqual(double lhs, double rhs, double tolerance);
bool geometryMatches(const GridGeometry& lhs,
                     const GridGeometry& rhs,
                     double tolerance,
                     std::string* error);

void writeFloatLayer(const std::string& path,
                     const std::vector<float>& values);
std::vector<float> readFloatLayer(const std::string& path,
                                  std::size_t expected_count);
void writeUint8Layer(const std::string& path,
                     const std::vector<std::uint8_t>& values);
std::vector<std::uint8_t> readUint8Layer(const std::string& path,
                                        std::size_t expected_count);

TerrainMetadata loadTerrainMetadata(const std::string& path);
void writeTerrainMetadata(const std::string& path,
                          const TerrainMetadata& metadata,
                          const std::string& source_pcd,
                          const std::string& trajectory_pcd,
                          const std::string& generated_utc,
                          const std::string& checksum_file,
                          const std::vector<std::pair<std::string, double>>& parameters);

}  // namespace go2_terrain
