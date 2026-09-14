#include "go2_terrain/terrain_grid.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace go2_terrain
{
namespace
{

bool validExportId(const std::string& value)
{
  if (value.empty() || value.size() > 128 ||
      !std::isalnum(static_cast<unsigned char>(value.front())))
  {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](char character) {
    const unsigned char byte = static_cast<unsigned char>(character);
    return std::isalnum(byte) || character == '_' || character == '-';
  });
}

LayerDescriptor parseLayer(const YAML::Node& layers,
                           const std::string& name,
                           const std::string& expected_type)
{
  const YAML::Node node = layers[name];
  if (!node || !node["file"] || !node["type"])
  {
    throw std::runtime_error("Terrain metadata is missing layer: " + name);
  }

  LayerDescriptor result;
  result.file = node["file"].as<std::string>();
  result.type = node["type"].as<std::string>();
  result.unit = node["unit"] ? node["unit"].as<std::string>() : std::string();
  if (result.type != expected_type)
  {
    throw std::runtime_error("Unsupported terrain layer type for " + name +
                             ": " + result.type);
  }
  return result;
}

void emitLayer(YAML::Emitter* emitter,
               const std::string& name,
               const LayerDescriptor& layer)
{
  *emitter << YAML::Key << name << YAML::Value << YAML::BeginMap;
  *emitter << YAML::Key << "file" << YAML::Value << layer.file;
  *emitter << YAML::Key << "type" << YAML::Value << layer.type;
  *emitter << YAML::Key << "unknown" << YAML::Value
           << (layer.type == "uint8" ? "255" : "nan");
  if (!layer.unit.empty())
  {
    *emitter << YAML::Key << "unit" << YAML::Value << layer.unit;
  }
  *emitter << YAML::EndMap;
}

}  // namespace

bool GridGeometry::valid() const
{
  return width > 0 && height > 0 && std::isfinite(resolution) &&
         resolution > 0.0 && std::isfinite(origin_x) &&
         std::isfinite(origin_y) && std::isfinite(origin_yaw);
}

void TerrainGrid::resize(const GridGeometry& geometry_in)
{
  geometry = geometry_in;
  const std::size_t count = geometry.cellCount();
  const float unknown = std::numeric_limits<float>::quiet_NaN();
  elevation.assign(count, unknown);
  slope_deg.assign(count, unknown);
  roughness.assign(count, unknown);
  step.assign(count, unknown);
  cost.assign(count, 255);
  confidence.assign(count, 255);
}

bool TerrainGrid::valid() const
{
  const std::size_t count = geometry.cellCount();
  return geometry.valid() && elevation.size() == count &&
         slope_deg.size() == count && roughness.size() == count &&
         step.size() == count && cost.size() == count &&
         confidence.size() == count;
}

bool isKnown(float value)
{
  return std::isfinite(value) && value != kUnknownTerrain;
}

bool nearlyEqual(double lhs, double rhs, double tolerance)
{
  return std::fabs(lhs - rhs) <= tolerance;
}

bool geometryMatches(const GridGeometry& lhs,
                     const GridGeometry& rhs,
                     double tolerance,
                     std::string* error)
{
  std::ostringstream message;
  if (lhs.width != rhs.width || lhs.height != rhs.height)
  {
    message << "cell dimensions differ: terrain=" << lhs.width << "x"
            << lhs.height << " master=" << rhs.width << "x" << rhs.height;
  }
  else if (!nearlyEqual(lhs.resolution, rhs.resolution, tolerance))
  {
    message << "resolution differs: terrain=" << lhs.resolution
            << " master=" << rhs.resolution;
  }
  else if (!nearlyEqual(lhs.origin_x, rhs.origin_x, tolerance) ||
           !nearlyEqual(lhs.origin_y, rhs.origin_y, tolerance))
  {
    message << "origin differs: terrain=[" << lhs.origin_x << ", "
            << lhs.origin_y << "] master=[" << rhs.origin_x << ", "
            << rhs.origin_y << "]";
  }
  else if (!nearlyEqual(lhs.origin_yaw, rhs.origin_yaw, tolerance))
  {
    message << "origin yaw differs: terrain=" << lhs.origin_yaw
            << " master=" << rhs.origin_yaw;
  }
  else
  {
    if (error)
    {
      error->clear();
    }
    return true;
  }

  if (error)
  {
    *error = message.str();
  }
  return false;
}

void writeFloatLayer(const std::string& path,
                     const std::vector<float>& values)
{
  static_assert(sizeof(float) == 4, "Terrain format requires 32-bit float");
  const std::uint16_t endian_test = 1;
  if (*reinterpret_cast<const std::uint8_t*>(&endian_test) != 1)
  {
    throw std::runtime_error("Terrain float32_le writer requires a little-endian host");
  }

  std::ofstream output(path.c_str(), std::ios::binary | std::ios::trunc);
  if (!output)
  {
    throw std::runtime_error("Cannot create terrain layer: " + path);
  }
  if (!values.empty())
  {
    output.write(reinterpret_cast<const char*>(values.data()),
                 static_cast<std::streamsize>(values.size() * sizeof(float)));
  }
  if (!output)
  {
    throw std::runtime_error("Failed writing terrain layer: " + path);
  }
}

std::vector<float> readFloatLayer(const std::string& path,
                                  std::size_t expected_count)
{
  std::ifstream input(path.c_str(), std::ios::binary | std::ios::ate);
  if (!input)
  {
    throw std::runtime_error("Cannot open terrain layer: " + path);
  }

  const std::streamoff size = input.tellg();
  const std::streamoff expected =
      static_cast<std::streamoff>(expected_count * sizeof(float));
  if (size != expected)
  {
    std::ostringstream message;
    message << "Terrain layer has wrong byte size: " << path << " got="
            << size << " expected=" << expected;
    throw std::runtime_error(message.str());
  }

  input.seekg(0, std::ios::beg);
  std::vector<float> values(expected_count);
  if (expected_count > 0)
  {
    input.read(reinterpret_cast<char*>(values.data()), expected);
  }
  if (!input)
  {
    throw std::runtime_error("Failed reading terrain layer: " + path);
  }
  return values;
}

void writeUint8Layer(const std::string& path,
                     const std::vector<std::uint8_t>& values)
{
  std::ofstream output(path.c_str(), std::ios::binary | std::ios::trunc);
  if (!output)
  {
    throw std::runtime_error("Cannot create terrain layer: " + path);
  }
  if (!values.empty())
  {
    output.write(reinterpret_cast<const char*>(values.data()),
                 static_cast<std::streamsize>(values.size()));
  }
  if (!output)
  {
    throw std::runtime_error("Failed writing terrain layer: " + path);
  }
}

std::vector<std::uint8_t> readUint8Layer(const std::string& path,
                                        std::size_t expected_count)
{
  std::ifstream input(path.c_str(), std::ios::binary | std::ios::ate);
  if (!input)
  {
    throw std::runtime_error("Cannot open terrain layer: " + path);
  }
  const std::streamoff size = input.tellg();
  if (size != static_cast<std::streamoff>(expected_count))
  {
    std::ostringstream message;
    message << "Terrain layer has wrong byte size: " << path << " got="
            << size << " expected=" << expected_count;
    throw std::runtime_error(message.str());
  }
  input.seekg(0, std::ios::beg);
  std::vector<std::uint8_t> values(expected_count);
  if (expected_count > 0)
  {
    input.read(reinterpret_cast<char*>(values.data()),
               static_cast<std::streamsize>(expected_count));
  }
  if (!input)
  {
    throw std::runtime_error("Failed reading terrain layer: " + path);
  }
  return values;
}

TerrainMetadata loadTerrainMetadata(const std::string& path)
{
  const YAML::Node root = YAML::LoadFile(path);
  TerrainMetadata result;
  result.format = root["format"].as<std::string>();
  result.version = root["version"].as<int>();
  result.export_id = root["export_id"].as<std::string>();
  result.frame_id = root["frame_id"].as<std::string>();

  if (result.format != "go2_terrain_2p5d" || result.version != 1)
  {
    throw std::runtime_error("Unsupported terrain metadata format/version: " + path);
  }
  if (result.frame_id != "map")
  {
    throw std::runtime_error("Terrain metadata frame_id must be map: " + path);
  }
  if (!validExportId(result.export_id))
  {
    throw std::runtime_error("Terrain metadata export_id is missing or unsafe: " +
                             path);
  }

  const YAML::Node map = root["map"];
  const YAML::Node origin = map["origin"];
  if (!map || !origin || !origin.IsSequence() || origin.size() != 3)
  {
    throw std::runtime_error("Terrain metadata has invalid map geometry: " + path);
  }
  result.geometry.width = map["width"].as<std::size_t>();
  result.geometry.height = map["height"].as<std::size_t>();
  result.geometry.resolution = map["resolution"].as<double>();
  result.geometry.origin_x = origin[0].as<double>();
  result.geometry.origin_y = origin[1].as<double>();
  result.geometry.origin_yaw = origin[2].as<double>();
  result.image_file = map["image"].as<std::string>();
  result.base_to_floor_m = root["base_to_floor_m"].as<double>();
  if (!result.geometry.valid())
  {
    throw std::runtime_error("Terrain metadata geometry is not finite: " + path);
  }

  const YAML::Node layers = root["layers"];
  result.elevation = parseLayer(layers, "elevation", "float32_le");
  result.slope = parseLayer(layers, "slope", "float32_le");
  result.roughness = parseLayer(layers, "roughness", "float32_le");
  result.step = parseLayer(layers, "step", "float32_le");
  result.cost = parseLayer(layers, "cost", "uint8");
  result.confidence = parseLayer(layers, "confidence", "uint8");
  return result;
}

void writeTerrainMetadata(
    const std::string& path,
    const TerrainMetadata& metadata,
    const std::string& source_pcd,
    const std::string& trajectory_pcd,
    const std::string& generated_utc,
    const std::string& checksum_file,
    const std::vector<std::pair<std::string, double>>& parameters)
{
  if (!metadata.geometry.valid())
  {
    throw std::runtime_error("Cannot write invalid terrain geometry");
  }
  if (!validExportId(metadata.export_id))
  {
    throw std::runtime_error("Cannot write missing or unsafe terrain export_id");
  }

  YAML::Emitter output;
  output << YAML::BeginMap;
  output << YAML::Key << "format" << YAML::Value << "go2_terrain_2p5d";
  output << YAML::Key << "version" << YAML::Value << 1;
  output << YAML::Key << "export_id" << YAML::Value << metadata.export_id;
  output << YAML::Key << "frame_id" << YAML::Value << metadata.frame_id;
  output << YAML::Key << "generated_utc" << YAML::Value << generated_utc;
  output << YAML::Key << "source_pcd" << YAML::Value << source_pcd;
  output << YAML::Key << "trajectory_pcd" << YAML::Value << trajectory_pcd;
  output << YAML::Key << "base_to_floor_m" << YAML::Value
         << metadata.base_to_floor_m;
  output << YAML::Key << "checksum_file" << YAML::Value << checksum_file;

  output << YAML::Key << "map" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "image" << YAML::Value << metadata.image_file;
  output << YAML::Key << "width" << YAML::Value << metadata.geometry.width;
  output << YAML::Key << "height" << YAML::Value << metadata.geometry.height;
  output << YAML::Key << "resolution" << YAML::Value
         << metadata.geometry.resolution;
  output << YAML::Key << "origin" << YAML::Value << YAML::Flow
         << YAML::BeginSeq << metadata.geometry.origin_x
         << metadata.geometry.origin_y << metadata.geometry.origin_yaw
         << YAML::EndSeq;
  output << YAML::EndMap;

  output << YAML::Key << "layers" << YAML::Value << YAML::BeginMap;
  emitLayer(&output, "elevation", metadata.elevation);
  emitLayer(&output, "slope", metadata.slope);
  emitLayer(&output, "roughness", metadata.roughness);
  emitLayer(&output, "step", metadata.step);
  emitLayer(&output, "cost", metadata.cost);
  emitLayer(&output, "confidence", metadata.confidence);
  output << YAML::EndMap;

  output << YAML::Key << "parameters" << YAML::Value << YAML::BeginMap;
  for (const auto& parameter : parameters)
  {
    output << YAML::Key << parameter.first << YAML::Value << parameter.second;
  }
  output << YAML::EndMap;
  output << YAML::EndMap;

  std::ofstream file(path.c_str(), std::ios::trunc);
  if (!file)
  {
    throw std::runtime_error("Cannot create terrain metadata: " + path);
  }
  file << output.c_str() << "\n";
  if (!file)
  {
    throw std::runtime_error("Failed writing terrain metadata: " + path);
  }
}

}  // namespace go2_terrain
