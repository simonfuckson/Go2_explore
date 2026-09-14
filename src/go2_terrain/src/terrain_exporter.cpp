#include <ros/ros.h>

#include <yaml-cpp/yaml.h>


#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <iostream>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>

#include "go2_terrain/terrain_algorithms.hpp"
#include "go2_terrain/terrain_grid.hpp"
#include "go2_terrain/offline_surface.hpp"
#include "go2_terrain/export_transaction.hpp"

namespace go2_terrain
{
namespace
{

constexpr std::uint8_t kUnknownImage = 205;
constexpr std::uint8_t kFreeImage = 254;
constexpr std::uint8_t kOccupiedImage = 0;
constexpr double kPi = 3.14159265358979323846;

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

struct MapDefinition
{
  GridGeometry geometry;
  std::string input_image_path;
  double occupied_threshold = 0.65;
  double free_threshold = 0.196;
  int negate = 0;
};

struct ExportParameters {
  double required_resolution=0.05, map_padding_m=0.50;
  double base_search_radius=1.0, base_min_height=0.20, base_max_height=0.55;
  int base_min_points=20;
  double obstacle_min_height=0.05, obstacle_max_height=1.50;
  int min_obstacle_points=2, minimum_ground_cells=100;
  double trajectory_free_radius=0.18, maximum_trajectory_gap=0.50;
  double obstacle_inflation_m=0.03, ground_free_dilation=0.10;
  double minimum_trajectory_ground_ratio=0.80;
  double minimum_trajectory_free_ratio=0.95;
  double minimum_trajectory_reachable_ratio=0.95;
  CostParameters cost;
  SurfaceParameters surface;
};

std::size_t cellIndex(int x, int y, const GridGeometry& geometry)
{
  return static_cast<std::size_t>(y) * geometry.width +
         static_cast<std::size_t>(x);
}

bool inside(int x, int y, const GridGeometry& geometry)
{
  return x >= 0 && y >= 0 && x < static_cast<int>(geometry.width) &&
         y < static_cast<int>(geometry.height);
}

bool pointToCell(double x,
                 double y,
                 const GridGeometry& geometry,
                 int* grid_x,
                 int* grid_y)
{
  const int gx = static_cast<int>(
      std::floor((x - geometry.origin_x) / geometry.resolution));
  const int gy = static_cast<int>(
      std::floor((y - geometry.origin_y) / geometry.resolution));
  if (!inside(gx, gy, geometry))
  {
    return false;
  }
  *grid_x = gx;
  *grid_y = gy;
  return true;
}

std::string readPgmToken(std::istream* input)
{
  std::string token;
  char c = '\0';
  while (input->get(c))
  {
    if (c == '#')
    {
      input->ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      continue;
    }
    if (!std::isspace(static_cast<unsigned char>(c)))
    {
      token.push_back(c);
      break;
    }
  }
  while (input->get(c))
  {
    if (std::isspace(static_cast<unsigned char>(c)))
    {
      if (c == '\r' && input->peek() == '\n')
      {
        input->get(c);
      }
      break;
    }
    token.push_back(c);
  }
  return token;
}

std::pair<std::size_t, std::size_t> readPgmDimensions(
    const std::string& path)
{
  std::ifstream input(path.c_str(), std::ios::binary);
  if (!input)
  {
    throw std::runtime_error("Cannot open map image: " + path);
  }
  const std::string magic = readPgmToken(&input);
  if (magic != "P5" && magic != "P2")
  {
    throw std::runtime_error("Only P5/P2 PGM map images are supported: " + path);
  }
  const long width = std::stol(readPgmToken(&input));
  const long height = std::stol(readPgmToken(&input));
  const long maximum = std::stol(readPgmToken(&input));
  if (width <= 0 || height <= 0 || maximum <= 0 || maximum > 255)
  {
    throw std::runtime_error("Invalid PGM header: " + path);
  }
  return {static_cast<std::size_t>(width),
          static_cast<std::size_t>(height)};
}

MapDefinition loadMapDefinition(const std::string& map_yaml)
{
  const YAML::Node root = YAML::LoadFile(map_yaml);
  MapDefinition result;
  result.geometry.resolution = root["resolution"].as<double>();
  const YAML::Node origin = root["origin"];
  if (!origin || !origin.IsSequence() || origin.size() != 3)
  {
    throw std::runtime_error("map.yaml origin must have three values");
  }
  result.geometry.origin_x = origin[0].as<double>();
  result.geometry.origin_y = origin[1].as<double>();
  result.geometry.origin_yaw = origin[2].as<double>();
  if (std::fabs(result.geometry.origin_yaw) > 1e-9)
  {
    throw std::runtime_error("Rotated map.yaml origins are not supported");
  }
  result.negate = root["negate"] ? root["negate"].as<int>() : 0;
  if (result.negate != 0)
  {
    throw std::runtime_error(
        "map.yaml negate must be 0; terrain export writes standard occupancy pixels");
  }
  result.occupied_threshold =
      root["occupied_thresh"] ? root["occupied_thresh"].as<double>() : 0.65;
  result.free_threshold =
      root["free_thresh"] ? root["free_thresh"].as<double>() : 0.196;

  const boost::filesystem::path yaml_path(map_yaml);
  const boost::filesystem::path image(root["image"].as<std::string>());
  result.input_image_path = image.is_absolute()
                                ? image.string()
                                : (yaml_path.parent_path() / image).string();
  const auto dimensions = readPgmDimensions(result.input_image_path);
  result.geometry.width = dimensions.first;
  result.geometry.height = dimensions.second;
  if (!result.geometry.valid())
  {
    throw std::runtime_error("Invalid geometry in map.yaml");
  }
  return result;
}

MapDefinition deriveMapDefinition(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                  const ExportParameters& parameters)
{
  double minimum_x = std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const auto& point : cloud.points)
  {
    if (!pcl::isFinite(point))
    {
      continue;
    }
    minimum_x = std::min(minimum_x, static_cast<double>(point.x));
    minimum_y = std::min(minimum_y, static_cast<double>(point.y));
    maximum_x = std::max(maximum_x, static_cast<double>(point.x));
    maximum_y = std::max(maximum_y, static_cast<double>(point.y));
  }
  if (!std::isfinite(minimum_x) || !std::isfinite(minimum_y) ||
      !std::isfinite(maximum_x) || !std::isfinite(maximum_y))
  {
    throw std::runtime_error("public_map.pcd contains no finite XYZ points");
  }

  MapDefinition result;
  result.geometry.resolution = parameters.required_resolution;
  result.geometry.origin_x = minimum_x - parameters.map_padding_m;
  result.geometry.origin_y = minimum_y - parameters.map_padding_m;
  result.geometry.origin_yaw = 0.0;
  const double padded_maximum_x = maximum_x + parameters.map_padding_m;
  const double padded_maximum_y = maximum_y + parameters.map_padding_m;
  result.geometry.width = static_cast<std::size_t>(std::ceil(
                              (padded_maximum_x - result.geometry.origin_x) /
                              result.geometry.resolution)) +
                          1U;
  result.geometry.height = static_cast<std::size_t>(std::ceil(
                               (padded_maximum_y - result.geometry.origin_y) /
                               result.geometry.resolution)) +
                           1U;
  if (!result.geometry.valid())
  {
    throw std::runtime_error("Failed to derive valid map geometry from public map");
  }
  return result;
}

float median(std::vector<float> values)
{
  if (values.empty())
  {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  float result = values[middle];
  if (values.size() % 2 == 0)
  {
    const auto lower = std::max_element(values.begin(), values.begin() + middle);
    result = 0.5F * (result + *lower);
  }
  return result;
}

double estimateBaseToFloor(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const pcl::PointCloud<pcl::PointXYZ>& trajectory,
    const ExportParameters& parameters)
{
  if (trajectory.empty())
  {
    throw std::runtime_error(
        "traversed_path_map.pcd is empty; base-to-floor cannot be estimated safely");
  }
  const pcl::PointXYZ& start = trajectory.front();
  if (!pcl::isFinite(start)) throw std::runtime_error("Invalid first trajectory point");
  const double radius_squared =
      parameters.base_search_radius * parameters.base_search_radius;
  const double bin_width = 0.01;
  const int bin_count = static_cast<int>(std::ceil(
      (parameters.base_max_height - parameters.base_min_height) / bin_width)) + 1;
  std::vector<std::vector<float>> bins(static_cast<std::size_t>(bin_count));

  for (const auto& point : cloud.points)
  {
    if (!pcl::isFinite(point))
    {
      continue;
    }
    const double dx = point.x - start.x;
    const double dy = point.y - start.y;
    if (dx * dx + dy * dy > radius_squared)
    {
      continue;
    }
    // /odom_nav deliberately flattens Z. FAST-LIO starts with base_link at
    // map Z=0, so only trajectory XY is used and initial floor is measured
    // below that zero reference.
    const double height = -static_cast<double>(point.z);
    if (height < parameters.base_min_height ||
        height > parameters.base_max_height)
    {
      continue;
    }
    const int bin = std::max(
        0,
        std::min(bin_count - 1,
                 static_cast<int>((height - parameters.base_min_height) /
                                  bin_width)));
    bins[static_cast<std::size_t>(bin)].push_back(static_cast<float>(height));
  }

  std::size_t best = 0;
  for (std::size_t i = 1; i < bins.size(); ++i)
  {
    if (bins[i].size() > bins[best].size())
    {
      best = i;
    }
  }
  if (bins[best].size() < static_cast<std::size_t>(parameters.base_min_points))
  {
    std::ostringstream message;
    message << "Only " << bins[best].size()
            << " consistent floor samples near trajectory start; need "
            << parameters.base_min_points;
    throw std::runtime_error(message.str());
  }

  std::vector<float> support;
  for (int offset = -1; offset <= 1; ++offset)
  {
    const int index = static_cast<int>(best) + offset;
    if (index >= 0 && index < bin_count)
    {
      support.insert(support.end(), bins[static_cast<std::size_t>(index)].begin(),
                     bins[static_cast<std::size_t>(index)].end());
    }
  }
  const double result = median(std::move(support));
  if (!std::isfinite(result) || result < parameters.base_min_height ||
      result > parameters.base_max_height)
  {
    throw std::runtime_error("Estimated base-to-floor height is outside safe bounds");
  }
  return result;
}

void markDisk(double world_x,
              double world_y,
              double radius_m,
              const GridGeometry& geometry,
              std::vector<std::uint8_t>* mask)
{
  int center_x = 0;
  int center_y = 0;
  if (!pointToCell(world_x, world_y, geometry, &center_x, &center_y))
  {
    return;
  }
  const int radius_cells =
      std::max(0, static_cast<int>(std::ceil(radius_m / geometry.resolution)));
  const double radius_sq = radius_m * radius_m +
                           geometry.resolution * geometry.resolution * 0.5;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy)
  {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
    {
      const int x = center_x + dx;
      const int y = center_y + dy;
      if (!inside(x, y, geometry))
      {
        continue;
      }
      const double metric_x = dx * geometry.resolution;
      const double metric_y = dy * geometry.resolution;
      if (metric_x * metric_x + metric_y * metric_y <= radius_sq)
      {
        (*mask)[cellIndex(x, y, geometry)] = 1;
      }
    }
  }
}

void markSegment(double start_x,
                 double start_y,
                 double end_x,
                 double end_y,
                 double radius_m,
                 const GridGeometry& geometry,
                 std::vector<std::uint8_t>* mask)
{
  const double distance = std::hypot(end_x - start_x, end_y - start_y);
  const double spacing = std::max(geometry.resolution * 0.5, 0.01);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
  for (int step = 0; step <= steps; ++step)
  {
    const double ratio = static_cast<double>(step) /
                         static_cast<double>(steps);
    markDisk(start_x + ratio * (end_x - start_x),
             start_y + ratio * (end_y - start_y),
             radius_m,
             geometry,
             mask);
  }
}

std::vector<std::uint8_t> buildTrajectoryMask(
    const std::vector<PlanarPoint>& trajectory,
    double radius_m,
    double maximum_gap_m,
    const GridGeometry& geometry)
{
  std::vector<std::uint8_t> mask(geometry.cellCount(), 0U);
  for (std::size_t i = 0; i < trajectory.size(); ++i)
  {
    markDisk(trajectory[i].x, trajectory[i].y, radius_m, geometry, &mask);
    if (i == 0)
    {
      continue;
    }
    const double segment_length = std::hypot(
        trajectory[i].x - trajectory[i - 1].x,
        trajectory[i].y - trajectory[i - 1].y);
    if (segment_length <= maximum_gap_m)
    {
      markSegment(trajectory[i - 1].x,
                  trajectory[i - 1].y,
                  trajectory[i].x,
                  trajectory[i].y,
                  radius_m,
                  geometry,
                  &mask);
    }
  }
  return mask;
}

class Sha256
{
public:
  Sha256() { reset(); }

  void update(const std::uint8_t* data, std::size_t length)
  {
    for (std::size_t i = 0; i < length; ++i)
    {
      block_[block_size_++] = data[i];
      bit_length_ += 8;
      if (block_size_ == 64)
      {
        transform();
        block_size_ = 0;
      }
    }
  }

  std::string finish()
  {
    block_[block_size_++] = 0x80;
    if (block_size_ > 56)
    {
      while (block_size_ < 64)
      {
        block_[block_size_++] = 0;
      }
      transform();
      block_size_ = 0;
    }
    while (block_size_ < 56)
    {
      block_[block_size_++] = 0;
    }
    for (int i = 7; i >= 0; --i)
    {
      block_[block_size_++] =
          static_cast<std::uint8_t>((bit_length_ >> (i * 8)) & 0xffU);
    }
    transform();

    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (const std::uint32_t value : state_)
    {
      output << std::setw(8) << value;
    }
    return output.str();
  }

private:
  static std::uint32_t rotateRight(std::uint32_t value, std::uint32_t count)
  {
    return (value >> count) | (value << (32U - count));
  }

  void reset()
  {
    state_ = {{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
               0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U}};
    block_.fill(0);
    block_size_ = 0;
    bit_length_ = 0;
  }

  void transform()
  {
    static const std::uint32_t constants[64] = {
        0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
        0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
        0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
        0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
        0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
        0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
        0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
        0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
        0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
        0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
        0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
        0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
        0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
        0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
        0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
        0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};
    std::uint32_t words[64] = {};
    for (int i = 0; i < 16; ++i)
    {
      words[i] = (static_cast<std::uint32_t>(block_[i * 4]) << 24U) |
                 (static_cast<std::uint32_t>(block_[i * 4 + 1]) << 16U) |
                 (static_cast<std::uint32_t>(block_[i * 4 + 2]) << 8U) |
                 static_cast<std::uint32_t>(block_[i * 4 + 3]);
    }
    for (int i = 16; i < 64; ++i)
    {
      const std::uint32_t s0 = rotateRight(words[i - 15], 7) ^
                               rotateRight(words[i - 15], 18) ^
                               (words[i - 15] >> 3U);
      const std::uint32_t s1 = rotateRight(words[i - 2], 17) ^
                               rotateRight(words[i - 2], 19) ^
                               (words[i - 2] >> 10U);
      words[i] = words[i - 16] + s0 + words[i - 7] + s1;
    }

    std::uint32_t a = state_[0];
    std::uint32_t b = state_[1];
    std::uint32_t c = state_[2];
    std::uint32_t d = state_[3];
    std::uint32_t e = state_[4];
    std::uint32_t f = state_[5];
    std::uint32_t g = state_[6];
    std::uint32_t h = state_[7];
    for (int i = 0; i < 64; ++i)
    {
      const std::uint32_t sum1 = rotateRight(e, 6) ^ rotateRight(e, 11) ^
                                 rotateRight(e, 25);
      const std::uint32_t choice = (e & f) ^ ((~e) & g);
      const std::uint32_t temp1 = h + sum1 + choice + constants[i] + words[i];
      const std::uint32_t sum0 = rotateRight(a, 2) ^ rotateRight(a, 13) ^
                                 rotateRight(a, 22);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temp2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }
    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<std::uint32_t, 8> state_;
  std::array<std::uint8_t, 64> block_;
  std::size_t block_size_ = 0;
  std::uint64_t bit_length_ = 0;
};

std::string sha256File(const std::string& path)
{
  std::ifstream input(path.c_str(), std::ios::binary);
  if (!input)
  {
    throw std::runtime_error("Cannot checksum file: " + path);
  }
  Sha256 sha;
  std::array<char, 1024 * 1024> buffer;
  while (input)
  {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = input.gcount();
    if (count > 0)
    {
      sha.update(reinterpret_cast<const std::uint8_t*>(buffer.data()),
                 static_cast<std::size_t>(count));
    }
  }
  if (!input.eof())
  {
    throw std::runtime_error("Failed while checksumming file: " + path);
  }
  return sha.finish();
}

std::map<std::string, std::string> readChecksumFile(const std::string& path)
{
  std::ifstream input(path.c_str());
  if (!input)
  {
    throw std::runtime_error("Cannot read checksum file: " + path);
  }
  std::map<std::string, std::string> result;
  std::string hash;
  std::string file;
  while (input >> hash >> file)
  {
    if (hash.size() != 64 || file.empty() || file.find('/') != std::string::npos ||
        file.find('\\') != std::string::npos || file == "." || file == "..")
    {
      throw std::runtime_error("Malformed checksum entry in: " + path);
    }
    if (!std::all_of(hash.begin(), hash.end(), [](char c) {
          return std::isxdigit(static_cast<unsigned char>(c)) != 0;
        }))
    {
      throw std::runtime_error("Checksum is not hexadecimal in: " + path);
    }
    std::transform(hash.begin(), hash.end(), hash.begin(), [](char c) {
      return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    });
    if (!result.emplace(file, hash).second)
    {
      throw std::runtime_error("Duplicate checksum entry for " + file);
    }
  }
  return result;
}

void verifyMappingSnapshot(const std::string& marker,
                           const std::string& public_map,
                           const std::string& trajectory)
{
  const std::map<std::string, std::string> checksums = readChecksumFile(marker);
  const std::set<std::string> expected = {
      "public_map.pcd", "traversed_path_map.pcd"};
  if (checksums.size() != expected.size())
  {
    throw std::runtime_error(
        "mapping_snapshot.sha256 must contain exactly the two mapping inputs");
  }
  for (const auto& file : expected)
  {
    const auto found = checksums.find(file);
    if (found == checksums.end())
    {
      throw std::runtime_error("Mapping snapshot is missing: " + file);
    }
    const std::string& path =
        file == "public_map.pcd" ? public_map : trajectory;
    if (boost::filesystem::path(path).filename().string() != file)
    {
      throw std::runtime_error("Mapping input basename does not match marker: " +
                               path);
    }
    if (sha256File(path) != found->second)
    {
      throw std::runtime_error("Mapping snapshot checksum mismatch: " + file);
    }
  }
}

std::string utcNow()
{
  const std::time_t now = std::time(nullptr);
  std::tm utc = {};
  gmtime_r(&now, &utc);
  char text[32] = {};
  std::strftime(text, sizeof(text), "%Y-%m-%dT%H:%M:%SZ", &utc);
  return text;
}

void writePgm(const std::string& path,
              const std::vector<std::uint8_t>& grid,
              const GridGeometry& geometry)
{
  std::ofstream output(path.c_str(), std::ios::binary | std::ios::trunc);
  if (!output)
  {
    throw std::runtime_error("Cannot create map image: " + path);
  }
  output << "P5\n" << geometry.width << " " << geometry.height << "\n255\n";
  for (int y = static_cast<int>(geometry.height) - 1; y >= 0; --y)
  {
    output.write(
        reinterpret_cast<const char*>(grid.data() + cellIndex(0, y, geometry)),
        static_cast<std::streamsize>(geometry.width));
  }
  if (!output)
  {
    throw std::runtime_error("Failed writing map image: " + path);
  }
}

void writeMapYaml(const std::string& path,
                  const MapDefinition& map,
                  const std::string& image_file)
{
  std::ofstream output(path.c_str(), std::ios::trunc);
  if (!output)
  {
    throw std::runtime_error("Cannot create map YAML: " + path);
  }
  output << "image: " << image_file << "\n";
  output << std::fixed << std::setprecision(9);
  output << "resolution: " << map.geometry.resolution << "\n";
  output << "origin: [" << map.geometry.origin_x << ", "
         << map.geometry.origin_y << ", " << map.geometry.origin_yaw << "]\n";
  output << "negate: " << map.negate << "\n";
  output << "occupied_thresh: " << map.occupied_threshold << "\n";
  output << "free_thresh: " << map.free_threshold << "\n";
}

void writePreview(const std::string& path,
                  const TerrainGrid& terrain,
                  const std::vector<std::uint8_t>& occupancy)
{
  std::ofstream output(path.c_str(), std::ios::binary | std::ios::trunc);
  if (!output)
  {
    throw std::runtime_error("Cannot create terrain preview: " + path);
  }
  output << "P6\n" << terrain.geometry.width << " "
         << terrain.geometry.height << "\n255\n";
  for (int y = static_cast<int>(terrain.geometry.height) - 1; y >= 0; --y)
  {
    for (int x = 0; x < static_cast<int>(terrain.geometry.width); ++x)
    {
      const std::size_t index = cellIndex(x, y, terrain.geometry);
      std::array<std::uint8_t, 3> pixel = {{90, 90, 90}};
      if (occupancy[index] == kOccupiedImage)
      {
        pixel = {{0, 0, 0}};
      }
      else if (terrain.cost[index] == 254)
      {
        pixel = {{220, 35, 35}};
      }
      else if (terrain.cost[index] != 255)
      {
        const double ratio = std::min(1.0, terrain.cost[index] / 80.0);
        pixel = {{static_cast<std::uint8_t>(40 + 200 * ratio),
                  static_cast<std::uint8_t>(190 - 100 * ratio), 45}};
      }
      else if (occupancy[index] == kFreeImage)
      {
        pixel = {{220, 220, 220}};
      }
      output.write(reinterpret_cast<const char*>(pixel.data()), 3);
    }
  }
}

}  // namespace

class TerrainExporter
{
public:
  TerrainExporter() : private_nh_("~")
  {
    loadParameters();
  }

  void run()
  {
    const boost::filesystem::path map_dir(map_dir_);
    if (!boost::filesystem::is_directory(map_dir))
    {
      throw std::runtime_error("Map directory does not exist: " + map_dir_);
    }
    if (!boost::filesystem::is_regular_file(input_pcd_) ||
        !boost::filesystem::is_regular_file(trajectory_pcd_) ||
        !boost::filesystem::is_regular_file(mapping_snapshot_))
    {
      throw std::runtime_error(
          "public_map.pcd, traversed_path_map.pcd, and mapping_snapshot.sha256 are required");
    }
    verifyMappingSnapshot(mapping_snapshot_, input_pcd_, trajectory_pcd_);
    const std::string initial_snapshot_digest =
        sha256File(mapping_snapshot_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory(
        new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(input_pcd_, *cloud) != 0 || cloud->empty())
    {
      throw std::runtime_error("Cannot load nonempty public map: " + input_pcd_);
    }
    if (pcl::io::loadPCDFile(trajectory_pcd_, *trajectory) != 0 ||
        trajectory->empty())
    {
      throw std::runtime_error("Cannot load nonempty trajectory: " +
                               trajectory_pcd_);
    }
    ROS_INFO("Terrain input: map_points=%zu trajectory_points=%zu",
             cloud->size(), trajectory->size());

    MapDefinition map;
    const bool has_input_map =
        boost::filesystem::is_regular_file(input_map_yaml_);
    if (has_input_map)
    {
      map = loadMapDefinition(input_map_yaml_);
      if (!nearlyEqual(map.geometry.resolution,
                       parameters_.required_resolution,
                       1e-9))
      {
        std::ostringstream error;
        error << "Map resolution must be " << parameters_.required_resolution
              << " m, got " << map.geometry.resolution;
        throw std::runtime_error(error.str());
      }
      ROS_INFO("Using existing map.yaml geometry: %zux%zu",
               map.geometry.width, map.geometry.height);
    }
    else
    {
      map = deriveMapDefinition(*cloud, parameters_);
      ROS_INFO("Derived map geometry from public_map.pcd: %zux%zu",
               map.geometry.width, map.geometry.height);
    }

    // Absolute-Z occupancy is used for geometry only. Copying its black
    // pixels would reintroduce the exact ramp-as-obstacle defect.
    const double base_to_floor =
        estimateBaseToFloor(*cloud, *trajectory, parameters_);
    ROS_INFO("Estimated base_link-to-floor height: %.3f m", base_to_floor);

    TerrainGrid terrain;
    std::vector<std::uint8_t> occupancy;
    pcl::PointCloud<pcl::PointXYZI> ground_diagnostic;
    pcl::PointCloud<pcl::PointXYZI> obstacle_diagnostic;
    reconstruct(*cloud,
                *trajectory,
                map.geometry,
                base_to_floor,
                &terrain,
                &occupancy,
                &ground_diagnostic,
                &obstacle_diagnostic);

    const boost::filesystem::path stage =
        map_dir / boost::filesystem::unique_path(".go2_terrain_stage_%%%%-%%%%");
    boost::filesystem::create_directory(stage);
    try
    {
      verifyMappingSnapshot(mapping_snapshot_, input_pcd_, trajectory_pcd_);
      if (sha256File(mapping_snapshot_) != initial_snapshot_digest)
      {
        throw std::runtime_error(
            "Mapping snapshot changed while terrain reconstruction was running");
      }
      writeOutputs(stage,
                   map,
                   terrain,
                   occupancy,
                   ground_diagnostic,
                   obstacle_diagnostic,
                   base_to_floor);
      validateStage(stage, map.geometry);
      verifyMappingSnapshot(mapping_snapshot_, input_pcd_, trajectory_pcd_);
      if (sha256File(mapping_snapshot_) != initial_snapshot_digest)
      {
        throw std::runtime_error(
            "Mapping snapshot changed before terrain assets could be committed");
      }
      commitOutputs(stage, map_dir);
      boost::filesystem::remove_all(stage);
    }
    catch (...)
    {
      boost::filesystem::remove_all(stage);
      throw;
    }

    ROS_INFO("Terrain export committed: %s", map_dir_.c_str());
  }

private:
  void loadParameters()
  {
    private_nh_.param<std::string>("map_dir", map_dir_, std::string());
    if (map_dir_.empty())
    {
      throw std::runtime_error("~map_dir is required");
    }
    private_nh_.param<std::string>("export_id", export_id_, std::string());
    if (!validExportId(export_id_))
    {
      throw std::runtime_error(
          "~export_id is required and must contain only letters, numbers, '_' or '-'");
    }
    const boost::filesystem::path root(map_dir_);
    private_nh_.param<std::string>(
        "input_pcd", input_pcd_, (root / "public_map.pcd").string());
    private_nh_.param<std::string>(
        "trajectory_pcd",
        trajectory_pcd_,
        (root / "traversed_path_map.pcd").string());
    private_nh_.param<std::string>(
        "input_map_yaml", input_map_yaml_, (root / "map.yaml").string());
    private_nh_.param<std::string>(
        "mapping_snapshot",
        mapping_snapshot_,
        (root / "mapping_snapshot.sha256").string());

    auto param = [this](const std::string& name, double* value) {
      private_nh_.param(name, *value, *value);
    };
    auto int_param = [this](const std::string& name, int* value) {
      private_nh_.param(name, *value, *value);
    };
    param("resolution", &parameters_.required_resolution);
    param("map_padding_m", &parameters_.map_padding_m);
    param("base_height/search_radius", &parameters_.base_search_radius);
    param("base_height/min", &parameters_.base_min_height);
    param("base_height/max", &parameters_.base_max_height);
    int_param("base_height/min_points", &parameters_.base_min_points);
    param("obstacle/min_height", &parameters_.obstacle_min_height);
    param("obstacle/max_height", &parameters_.obstacle_max_height);
    int_param("obstacle/min_points", &parameters_.min_obstacle_points);
    int_param("minimum_ground_cells", &parameters_.minimum_ground_cells);
    param("occupancy/trajectory_free_radius", &parameters_.trajectory_free_radius);
    param("occupancy/max_trajectory_gap", &parameters_.maximum_trajectory_gap);
    param("occupancy/obstacle_inflation_m", &parameters_.obstacle_inflation_m);
    param("occupancy/ground_free_dilation", &parameters_.ground_free_dilation);
    param("quality/minimum_trajectory_ground_ratio", &parameters_.minimum_trajectory_ground_ratio);
    param("quality/minimum_trajectory_free_ratio", &parameters_.minimum_trajectory_free_ratio);
    param("quality/minimum_trajectory_reachable_ratio", &parameters_.minimum_trajectory_reachable_ratio);
    param("cost/flat_slope_deg", &parameters_.cost.flat_slope_deg);
    param("cost/lethal_slope_deg", &parameters_.cost.lethal_slope_deg);
    param("cost/dilation_m", &parameters_.cost.dilation_m);
    double min_cost=parameters_.cost.minimum_cost, max_cost=parameters_.cost.maximum_soft_cost;
    param("cost/minimum_cost", &min_cost); param("cost/maximum_soft_cost", &max_cost);
    parameters_.cost.minimum_cost=min_cost; parameters_.cost.maximum_soft_cost=max_cost;
    int cluster=parameters_.cost.minimum_lethal_cluster_cells;
    int_param("cost/minimum_lethal_cluster_cells", &cluster);
    parameters_.cost.minimum_lethal_cluster_cells=cluster;
    private_nh_.param("surface/candidate_percentile", parameters_.surface.candidate_percentile, parameters_.surface.candidate_percentile);
    private_nh_.param("surface/seed_radius_m", parameters_.surface.seed_radius_m, parameters_.surface.seed_radius_m);
    private_nh_.param("surface/seed_height_tolerance_m", parameters_.surface.seed_height_tolerance_m, parameters_.surface.seed_height_tolerance_m);
    private_nh_.param("surface/trusted_seed_height_tolerance_m", parameters_.surface.trusted_seed_height_tolerance_m, parameters_.surface.trusted_seed_height_tolerance_m);
    private_nh_.param("surface/pmf_max_window_size", parameters_.surface.pmf_max_window_size, parameters_.surface.pmf_max_window_size);
    private_nh_.param("surface/pmf_slope", parameters_.surface.pmf_slope, parameters_.surface.pmf_slope);
    private_nh_.param("surface/pmf_initial_distance_m", parameters_.surface.pmf_initial_distance_m, parameters_.surface.pmf_initial_distance_m);
    private_nh_.param("surface/pmf_max_distance_m", parameters_.surface.pmf_max_distance_m, parameters_.surface.pmf_max_distance_m);
    private_nh_.param("surface/pmf_base", parameters_.surface.pmf_base, parameters_.surface.pmf_base);
    private_nh_.param("surface/pmf_exponential", parameters_.surface.pmf_exponential, parameters_.surface.pmf_exponential);
    private_nh_.param("surface/validation_radius_m", parameters_.surface.validation_radius_m, parameters_.surface.validation_radius_m);
    private_nh_.param("surface/validation_min_candidates", parameters_.surface.validation_min_candidates, parameters_.surface.validation_min_candidates);
    private_nh_.param("surface/plane_inlier_tolerance_m", parameters_.surface.plane_inlier_tolerance_m, parameters_.surface.plane_inlier_tolerance_m);
    private_nh_.param("surface/plane_min_inlier_ratio", parameters_.surface.plane_min_inlier_ratio, parameters_.surface.plane_min_inlier_ratio);
    private_nh_.param("surface/plane_min_spread_m", parameters_.surface.plane_min_spread_m, parameters_.surface.plane_min_spread_m);
    private_nh_.param("surface/candidate_plane_tolerance_m", parameters_.surface.candidate_plane_tolerance_m, parameters_.surface.candidate_plane_tolerance_m);
    private_nh_.param("surface/plane_max_rmse_m", parameters_.surface.plane_max_rmse_m, parameters_.surface.plane_max_rmse_m);
    private_nh_.param("surface/max_ground_slope_deg", parameters_.surface.max_ground_slope_deg, parameters_.surface.max_ground_slope_deg);
    private_nh_.param("surface/connect_radius_m", parameters_.surface.connect_radius_m, parameters_.surface.connect_radius_m);
    private_nh_.param("surface/connection_plane_tolerance_m", parameters_.surface.connection_plane_tolerance_m, parameters_.surface.connection_plane_tolerance_m);
    private_nh_.param("surface/connection_max_normal_delta_deg", parameters_.surface.connection_max_normal_delta_deg, parameters_.surface.connection_max_normal_delta_deg);
    private_nh_.param("surface/max_ground_step_m", parameters_.surface.max_ground_step_m, parameters_.surface.max_ground_step_m);
    private_nh_.param("surface/surface_fit_radius_m", parameters_.surface.surface_fit_radius_m, parameters_.surface.surface_fit_radius_m);
    private_nh_.param("surface/surface_observation_radius_m", parameters_.surface.surface_observation_radius_m, parameters_.surface.surface_observation_radius_m);
    private_nh_.param("surface/surface_gap_fill_radius_m", parameters_.surface.surface_gap_fill_radius_m, parameters_.surface.surface_gap_fill_radius_m);
    private_nh_.param("surface/surface_min_candidates", parameters_.surface.surface_min_candidates, parameters_.surface.surface_min_candidates);
    private_nh_.param("surface/surface_min_component_cells", parameters_.surface.surface_min_component_cells, parameters_.surface.surface_min_component_cells);
    private_nh_.param("surface/surface_min_component_fraction", parameters_.surface.surface_min_component_fraction, parameters_.surface.surface_min_component_fraction);
    private_nh_.param("surface/min_connected_ground_cells", parameters_.surface.min_connected_ground_cells, parameters_.surface.min_connected_ground_cells);
    private_nh_.param("surface/wall_search_radius_m", parameters_.surface.wall_search_radius_m, parameters_.surface.wall_search_radius_m);
    private_nh_.param("surface/wall_max_nearest_m", parameters_.surface.wall_max_nearest_m, parameters_.surface.wall_max_nearest_m);
    private_nh_.param("surface/wall_min_inlier_ratio", parameters_.surface.wall_min_inlier_ratio, parameters_.surface.wall_min_inlier_ratio);
    private_nh_.param("surface/wall_min_ground_cells", parameters_.surface.wall_min_ground_cells, parameters_.surface.wall_min_ground_cells);
    if (parameters_.base_min_height<0.20 || parameters_.base_max_height>0.55 ||
        parameters_.base_min_height>=parameters_.base_max_height ||
        parameters_.min_obstacle_points<1 || parameters_.minimum_ground_cells<100)
      throw std::runtime_error("Invalid base-height or minimum support limits");
    for (double ratio: {parameters_.minimum_trajectory_ground_ratio,
                        parameters_.minimum_trajectory_free_ratio,
                        parameters_.minimum_trajectory_reachable_ratio})
      if (!std::isfinite(ratio) || ratio<=0 || ratio>1)
        throw std::runtime_error("Terrain quality ratios must be finite and in (0,1]");
    if (parameters_.minimum_trajectory_ground_ratio<0.80 ||
        parameters_.minimum_trajectory_free_ratio<0.95 ||
        parameters_.minimum_trajectory_reachable_ratio<0.95 ||
        !std::isfinite(parameters_.ground_free_dilation) || parameters_.ground_free_dilation<0 ||
        parameters_.ground_free_dilation>0.10 || !std::isfinite(parameters_.maximum_trajectory_gap) ||
        parameters_.maximum_trajectory_gap<=0 || parameters_.maximum_trajectory_gap>0.50)
      throw std::runtime_error("Export quality/free-space limits are outside the validated profile");
    if (parameters_.required_resolution!=0.05 || parameters_.surface.max_ground_slope_deg!=35.0 ||
        parameters_.cost.flat_slope_deg!=8.0 || parameters_.cost.lethal_slope_deg!=30.0 ||
        cluster!=4 || min_cost!=15 || max_cost!=80 || parameters_.cost.dilation_m!=0.20 ||
        parameters_.obstacle_min_height!=0.05 || parameters_.obstacle_max_height!=1.50 ||
        parameters_.trajectory_free_radius!=0.18 || parameters_.obstacle_inflation_m!=0.03)
      throw std::runtime_error("Terrain settings differ from the approved Go2 profile");
  }

  void reconstruct(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                   const pcl::PointCloud<pcl::PointXYZ>& trajectory,
                   const GridGeometry& geometry, double base_to_floor,
                   TerrainGrid* terrain, std::vector<std::uint8_t>* occupancy,
                   pcl::PointCloud<pcl::PointXYZI>* ground_diagnostic,
                   pcl::PointCloud<pcl::PointXYZI>* obstacle_diagnostic)
  {
    std::vector<PlanarPoint> path;
    for (const auto& point: trajectory) if (pcl::isFinite(point)) path.push_back({point.x,point.y});
    if (path.empty()) throw std::runtime_error("No finite trajectory points");
    auto p=parameters_.surface;
    p.cell_size=geometry.resolution;
    p.seed_x=path.front().x; p.seed_y=path.front().y; p.seed_ground_z=-base_to_floor;
    p.min_obstacle_relative_height_m=parameters_.obstacle_min_height;
    p.max_obstacle_relative_height_m=parameters_.obstacle_max_height;
    ROS_INFO("Reconstruction revision 2: distributed PMF anchors + continuous surface + observed wall recovery");
    auto result=reconstructSurface(cloud,geometry,p);
    *terrain=std::move(result.terrain);
    *ground_diagnostic=std::move(result.ground);
    *obstacle_diagnostic=std::move(result.obstacles);
    if (ground_diagnostic->size()<static_cast<std::size_t>(parameters_.minimum_ground_cells))
      throw std::runtime_error("Insufficient reconstructed ground; old export retained");
    terrain->cost=buildSlopeCostLayer(terrain->slope_deg,geometry,parameters_.cost);
    const std::size_t count=geometry.cellCount();
    std::vector<std::uint8_t> ground(count,0), obstacles(count,0), expanded, blocked;
    for (std::size_t i=0;i<count;++i) {
      ground[i]=isKnown(terrain->elevation[i]);
      obstacles[i]=result.obstacle_count[i]>=static_cast<unsigned>(parameters_.min_obstacle_points) ||
                   result.measured_step[i];
    }
    // Bounded free completion only around measured/validated ground. The wall
    // recovery reference is deliberately absent from this mask.
    expanded=ground;
    const int radius=std::floor(parameters_.ground_free_dilation/geometry.resolution+1e-9);
    for (int y=0;y<static_cast<int>(geometry.height);++y)
      for (int x=0;x<static_cast<int>(geometry.width);++x) if (ground[cellIndex(x,y,geometry)])
        for (int dy=-radius;dy<=radius;++dy) for (int dx=-radius;dx<=radius;++dx)
          if (dx*dx+dy*dy<=radius*radius && inside(x+dx,y+dy,geometry))
            expanded[cellIndex(x+dx,y+dy,geometry)]=1;
    const auto walked=buildTrajectoryMask(path,parameters_.trajectory_free_radius,
                                         parameters_.maximum_trajectory_gap,geometry);
    dilateBinaryMaskMetric(obstacles,&blocked,geometry,parameters_.obstacle_inflation_m);
    occupancy->assign(count,kUnknownImage);
    for (std::size_t i=0;i<count;++i) {
      if (expanded[i] || walked[i]) (*occupancy)[i]=kFreeImage;
      if (result.unresolved_vertical[i]) (*occupancy)[i]=kUnknownImage;
      if (blocked[i]) (*occupancy)[i]=kOccupiedImage; // measured obstacle always wins
    }

    // Flood fill actual PGM free cells, never "known" cells: occupied is known too.
    std::vector<std::uint8_t> reached(count,0);
    std::queue<std::size_t> queue;
    int start_x,start_y;
    if (pointToCell(path.front().x,path.front().y,geometry,&start_x,&start_y)) {
      const auto start=cellIndex(start_x,start_y,geometry);
      if ((*occupancy)[start]==kFreeImage) { reached[start]=1; queue.push(start); }
    }
    while (!queue.empty()) {
      const auto i=queue.front(); queue.pop();
      const int x=i%geometry.width,y=i/geometry.width;
      for (int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
        if ((!dx&&!dy) || !inside(x+dx,y+dy,geometry)) continue;
        const auto n=cellIndex(x+dx,y+dy,geometry);
        if (reached[n] || (*occupancy)[n]!=kFreeImage) continue;
        if (dx&&dy && ((*occupancy)[cellIndex(x+dx,y,geometry)]!=kFreeImage ||
                      (*occupancy)[cellIndex(x,y+dy,geometry)]!=kFreeImage)) continue;
        reached[n]=1; queue.push(n);
      }
    }
    std::size_t on_ground=0,free=0,reachable=0;
    for (const auto& point:path) {
      int x,y; if (!pointToCell(point.x,point.y,geometry,&x,&y)) continue;
      const auto i=cellIndex(x,y,geometry);
      on_ground+=ground[i]!=0; free+=(*occupancy)[i]==kFreeImage; reachable+=reached[i]!=0;
    }
    const double ground_ratio=double(on_ground)/path.size();
    const double free_ratio=double(free)/path.size();
    const double reachable_ratio=double(reachable)/path.size();
    quality_report_="reconstruction_revision: 2\n";
    std::ostringstream report;
    report << std::setprecision(10) << "ground_cells: " << ground_diagnostic->size()
           << "\nrecovered_wall_cells: " << result.recovered_wall_cells
           << "\nunresolved_vertical_cells: " << result.unresolved_vertical_cells
           << "\nmeasured_step_cells: " << std::count(result.measured_step.begin(),result.measured_step.end(),1)
           << "\ntrajectory_points: " << path.size()
           << "\ntrajectory_ground_ratio: " << ground_ratio
           << "\ntrajectory_free_ratio: " << free_ratio
           << "\ntrajectory_reachable_ratio: " << reachable_ratio << "\n";
    quality_report_+=report.str();
    ROS_INFO_STREAM("Terrain export quality:\n" << quality_report_);
    if (ground_ratio<parameters_.minimum_trajectory_ground_ratio ||
        free_ratio<parameters_.minimum_trajectory_free_ratio ||
        reachable_ratio<parameters_.minimum_trajectory_reachable_ratio)
      throw std::runtime_error("Terrain quality failed: insufficient ground/free/reachable trajectory; "
                               "inspect measured obstacles or rescan gaps. Original map retained.");
  }

  void writeOutputs(
      const boost::filesystem::path& stage,
      const MapDefinition& map,
      const TerrainGrid& terrain,
      const std::vector<std::uint8_t>& occupancy,
      const pcl::PointCloud<pcl::PointXYZI>& ground_diagnostic,
      const pcl::PointCloud<pcl::PointXYZI>& obstacle_diagnostic,
      double base_to_floor) const
  {
    const std::string map_pgm = "map.pgm";
    const std::string map_yaml = "map.yaml";
    const std::string elevation = "terrain_elevation.f32";
    const std::string slope = "terrain_slope.f32";
    const std::string roughness = "terrain_roughness.f32";
    const std::string step = "terrain_step.f32";
    const std::string cost = "terrain_cost.u8";
    const std::string confidence = "terrain_confidence.u8";
    const std::string ground_pcd = "terrain_ground.pcd";
    const std::string obstacle_pcd = "terrain_obstacles.pcd";
    const std::string preview = "terrain_preview.ppm";
    const std::string checksums = "terrain_checksums.sha256";

    writePgm((stage / map_pgm).string(), occupancy, terrain.geometry);
    writeMapYaml((stage / map_yaml).string(), map, map_pgm);
    writeFloatLayer((stage / elevation).string(), terrain.elevation);
    writeFloatLayer((stage / slope).string(), terrain.slope_deg);
    writeFloatLayer((stage / roughness).string(), terrain.roughness);
    writeFloatLayer((stage / step).string(), terrain.step);
    writeUint8Layer((stage / cost).string(), terrain.cost);
    writeUint8Layer((stage / confidence).string(), terrain.confidence);
    auto save_cloud=[](const std::string& path, const pcl::PointCloud<pcl::PointXYZI>& cloud) {
      if (cloud.empty()) {
        // PCL rejects an empty cloud; an obstacle-free map is nevertheless valid.
        std::ofstream output(path, std::ios::binary);
        output << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
                  "TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 0\nHEIGHT 1\n"
                  "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 0\nDATA binary\n";
        output.close();
        if (!output) throw std::runtime_error("Failed writing empty diagnostic PCD");
      } else if (pcl::io::savePCDFileBinaryCompressed(path,cloud)!=0)
        throw std::runtime_error("Failed writing terrain diagnostic PCD: "+path);
    };
    save_cloud((stage/ground_pcd).string(),ground_diagnostic);
    save_cloud((stage/obstacle_pcd).string(),obstacle_diagnostic);
    writePreview((stage / preview).string(), terrain, occupancy);
    std::ofstream quality((stage/"terrain_quality.yaml").string());
    quality << quality_report_;
    quality.close();
    if (!quality) throw std::runtime_error("Failed writing terrain quality report");

    TerrainMetadata metadata;
    metadata.format = "go2_terrain_2p5d";
    metadata.version = 1;
    metadata.export_id = export_id_;
    metadata.frame_id = "map";
    metadata.geometry = terrain.geometry;
    metadata.image_file = map_pgm;
    metadata.base_to_floor_m = base_to_floor;
    metadata.elevation = {elevation, "float32_le", "m"};
    metadata.slope = {slope, "float32_le", "deg"};
    metadata.roughness = {roughness, "float32_le", "m"};
    metadata.step = {step, "float32_le", "m"};
    metadata.cost = {cost, "uint8", "cost"};
    metadata.confidence = {confidence, "uint8", "percent"};
    std::vector<std::pair<std::string, double>> exported_parameters = {
        {"reconstruction_revision", 2.0},
        {"ground_max_slope_deg", parameters_.surface.max_ground_slope_deg},
        {"obstacle_min_height_m", parameters_.obstacle_min_height},
        {"obstacle_max_height_m", parameters_.obstacle_max_height},
        {"trajectory_free_radius_m", parameters_.trajectory_free_radius},
        {"obstacle_inflation_m", parameters_.obstacle_inflation_m},
        {"preserve_existing_map", 0.0},
        {"minimum_trajectory_ground_ratio", parameters_.minimum_trajectory_ground_ratio},
        {"minimum_trajectory_free_ratio", parameters_.minimum_trajectory_free_ratio},
        {"minimum_trajectory_reachable_ratio", parameters_.minimum_trajectory_reachable_ratio},
        {"wall_search_radius_m", parameters_.surface.wall_search_radius_m},
        {"wall_max_nearest_m", parameters_.surface.wall_max_nearest_m},
        {"wall_min_inlier_ratio", parameters_.surface.wall_min_inlier_ratio},
        {"ground_free_dilation_m", parameters_.ground_free_dilation},
        {"flat_slope_deg", parameters_.cost.flat_slope_deg},
        {"lethal_slope_deg", parameters_.cost.lethal_slope_deg},
        {"minimum_slope_cost", parameters_.cost.minimum_cost},
        {"maximum_soft_slope_cost", parameters_.cost.maximum_soft_cost},
        {"slope_cost_dilation_m", parameters_.cost.dilation_m},
        {"minimum_lethal_cluster_cells",
         static_cast<double>(parameters_.cost.minimum_lethal_cluster_cells)}};
    exported_parameters.emplace_back("surface_candidate_percentile", parameters_.surface.candidate_percentile);
    exported_parameters.emplace_back("surface_seed_radius_m", parameters_.surface.seed_radius_m);
    exported_parameters.emplace_back("surface_seed_height_tolerance_m", parameters_.surface.seed_height_tolerance_m);
    exported_parameters.emplace_back("surface_trusted_seed_height_tolerance_m", parameters_.surface.trusted_seed_height_tolerance_m);
    exported_parameters.emplace_back("surface_pmf_max_window_size", parameters_.surface.pmf_max_window_size);
    exported_parameters.emplace_back("surface_pmf_slope", parameters_.surface.pmf_slope);
    exported_parameters.emplace_back("surface_pmf_initial_distance_m", parameters_.surface.pmf_initial_distance_m);
    exported_parameters.emplace_back("surface_pmf_max_distance_m", parameters_.surface.pmf_max_distance_m);
    exported_parameters.emplace_back("surface_pmf_base", parameters_.surface.pmf_base);
    exported_parameters.emplace_back("surface_pmf_exponential", parameters_.surface.pmf_exponential);
    exported_parameters.emplace_back("surface_validation_radius_m", parameters_.surface.validation_radius_m);
    exported_parameters.emplace_back("surface_validation_min_candidates", parameters_.surface.validation_min_candidates);
    exported_parameters.emplace_back("surface_plane_inlier_tolerance_m", parameters_.surface.plane_inlier_tolerance_m);
    exported_parameters.emplace_back("surface_plane_min_inlier_ratio", parameters_.surface.plane_min_inlier_ratio);
    exported_parameters.emplace_back("surface_plane_min_spread_m", parameters_.surface.plane_min_spread_m);
    exported_parameters.emplace_back("surface_candidate_plane_tolerance_m", parameters_.surface.candidate_plane_tolerance_m);
    exported_parameters.emplace_back("surface_plane_max_rmse_m", parameters_.surface.plane_max_rmse_m);
    exported_parameters.emplace_back("surface_max_ground_slope_deg", parameters_.surface.max_ground_slope_deg);
    exported_parameters.emplace_back("surface_connect_radius_m", parameters_.surface.connect_radius_m);
    exported_parameters.emplace_back("surface_connection_plane_tolerance_m", parameters_.surface.connection_plane_tolerance_m);
    exported_parameters.emplace_back("surface_connection_max_normal_delta_deg", parameters_.surface.connection_max_normal_delta_deg);
    exported_parameters.emplace_back("surface_max_ground_step_m", parameters_.surface.max_ground_step_m);
    exported_parameters.emplace_back("surface_surface_fit_radius_m", parameters_.surface.surface_fit_radius_m);
    exported_parameters.emplace_back("surface_surface_observation_radius_m", parameters_.surface.surface_observation_radius_m);
    exported_parameters.emplace_back("surface_surface_gap_fill_radius_m", parameters_.surface.surface_gap_fill_radius_m);
    exported_parameters.emplace_back("surface_surface_min_candidates", parameters_.surface.surface_min_candidates);
    exported_parameters.emplace_back("surface_surface_min_component_cells", parameters_.surface.surface_min_component_cells);
    exported_parameters.emplace_back("surface_surface_min_component_fraction", parameters_.surface.surface_min_component_fraction);
    exported_parameters.emplace_back("surface_min_connected_ground_cells", parameters_.surface.min_connected_ground_cells);
    exported_parameters.emplace_back("surface_wall_search_radius_m", parameters_.surface.wall_search_radius_m);
    exported_parameters.emplace_back("surface_wall_max_nearest_m", parameters_.surface.wall_max_nearest_m);
    exported_parameters.emplace_back("surface_wall_min_inlier_ratio", parameters_.surface.wall_min_inlier_ratio);
    exported_parameters.emplace_back("surface_wall_min_ground_cells", parameters_.surface.wall_min_ground_cells);
    writeTerrainMetadata(
        (stage / "terrain_2p5d.yaml").string(),
        metadata,
        boost::filesystem::path(input_pcd_).filename().string(),
        boost::filesystem::path(trajectory_pcd_).filename().string(),
        utcNow(),
        checksums,
        exported_parameters);

    const std::vector<std::string> checksummed = {
        map_yaml, map_pgm, elevation, slope, roughness, step, cost, confidence,
        ground_pcd, obstacle_pcd, preview, "terrain_quality.yaml", "terrain_2p5d.yaml"};
    std::ofstream checksum_output((stage / checksums).string(), std::ios::trunc);
    if (!checksum_output)
    {
      throw std::runtime_error("Cannot create terrain checksum file");
    }
    for (const auto& file : checksummed)
    {
      checksum_output << sha256File((stage / file).string()) << "  " << file
                      << "\n";
    }
    const std::vector<std::pair<std::string, std::string>> source_files = {
        {input_pcd_, boost::filesystem::path(input_pcd_).filename().string()},
        {trajectory_pcd_,
         boost::filesystem::path(trajectory_pcd_).filename().string()},
        {mapping_snapshot_, "mapping_snapshot.sha256"}};
    for (const auto& source : source_files)
    {
      checksum_output << sha256File(source.first) << "  " << source.second
                      << "\n";
    }
    checksum_output.close();
    if (!checksum_output)
    {
      throw std::runtime_error("Failed writing terrain checksum file");
    }
  }

  void validateStage(const boost::filesystem::path& stage,
                     const GridGeometry& expected_geometry) const
  {
    const TerrainMetadata metadata =
        loadTerrainMetadata((stage / "terrain_2p5d.yaml").string());
    if (metadata.export_id != export_id_)
    {
      throw std::runtime_error("Staged terrain export_id does not match request");
    }
    std::string geometry_error;
    if (!geometryMatches(metadata.geometry,
                         expected_geometry,
                         1e-9,
                         &geometry_error))
    {
      throw std::runtime_error("Staged metadata geometry mismatch: " +
                               geometry_error);
    }
    if (metadata.base_to_floor_m < parameters_.base_min_height ||
        metadata.base_to_floor_m > parameters_.base_max_height)
    {
      throw std::runtime_error("Staged base-to-floor estimate is outside limits");
    }

    const MapDefinition staged_map =
        loadMapDefinition((stage / "map.yaml").string());
    if (!geometryMatches(staged_map.geometry,
                         expected_geometry,
                         1e-9,
                         &geometry_error))
    {
      throw std::runtime_error("Staged map geometry mismatch: " + geometry_error);
    }

    const std::vector<LayerDescriptor> float_layers = {
        metadata.elevation,
        metadata.slope,
        metadata.roughness,
        metadata.step};
    for (const auto& layer : float_layers)
    {
      const std::vector<float> values = readFloatLayer(
          (stage / layer.file).string(), metadata.geometry.cellCount());
      for (const float value : values)
      {
        if (std::isinf(value))
        {
          throw std::runtime_error("Staged float layer contains infinity: " +
                                   layer.file);
        }
      }
    }
    const std::vector<std::uint8_t> costs = readUint8Layer(
        (stage / metadata.cost.file).string(), metadata.geometry.cellCount());
    const std::vector<std::uint8_t> confidence = readUint8Layer(
        (stage / metadata.confidence.file).string(),
        metadata.geometry.cellCount());
    for (const std::uint8_t value : confidence)
    {
      if (value != 255 && value > 100)
      {
        throw std::runtime_error(
            "Staged confidence layer contains a value above 100");
      }
    }

    const std::set<std::string> required = {
        "map.yaml",
        "map.pgm",
        metadata.elevation.file,
        metadata.slope.file,
        metadata.roughness.file,
        metadata.step.file,
        metadata.cost.file,
        metadata.confidence.file,
        "terrain_ground.pcd",
        "terrain_obstacles.pcd",
        "terrain_preview.ppm",
        "terrain_quality.yaml",
        "terrain_2p5d.yaml",
        boost::filesystem::path(input_pcd_).filename().string(),
        boost::filesystem::path(trajectory_pcd_).filename().string(),
        "mapping_snapshot.sha256"};
    const std::map<std::string, std::string> checksums = readChecksumFile(
        (stage / "terrain_checksums.sha256").string());
    for (const auto& file_name : required)
    {
      const auto found = checksums.find(file_name);
      if (found == checksums.end())
      {
        throw std::runtime_error("Missing staged checksum entry: " + file_name);
      }
      boost::filesystem::path file_path = stage / file_name;
      if (file_name == boost::filesystem::path(input_pcd_).filename().string())
      {
        file_path = input_pcd_;
      }
      else if (file_name ==
               boost::filesystem::path(trajectory_pcd_).filename().string())
      {
        file_path = trajectory_pcd_;
      }
      else if (file_name == "mapping_snapshot.sha256")
      {
        file_path = mapping_snapshot_;
      }
      if (!boost::filesystem::is_regular_file(file_path) ||
          sha256File(file_path.string()) != found->second)
      {
        throw std::runtime_error("Staged checksum mismatch: " + file_name);
      }
    }
    if (boost::filesystem::file_size(stage / "terrain_ground.pcd") < 100 ||
        boost::filesystem::file_size(stage / "terrain_obstacles.pcd") < 100 ||
        boost::filesystem::file_size(stage / "terrain_preview.ppm") < 100)
    {
      throw std::runtime_error("One or more staged diagnostic assets are empty");
    }
    (void)costs;
    ROS_INFO("Validated all staged map, terrain, diagnostic, and checksum assets");
  }

  void commitOutputs(const boost::filesystem::path& stage,
                     const boost::filesystem::path& destination) const
  {
    // Metadata is the commit marker and is therefore always installed last.
    const std::vector<std::string> files = {
        "map.pgm",
        "map.yaml",
        "terrain_elevation.f32",
        "terrain_slope.f32",
        "terrain_roughness.f32",
        "terrain_step.f32",
        "terrain_cost.u8",
        "terrain_confidence.u8",
        "terrain_ground.pcd",
        "terrain_obstacles.pcd",
        "terrain_preview.ppm",
        "terrain_quality.yaml",
        "terrain_checksums.sha256",
        "terrain_2p5d.yaml"};
    commitExportFiles(stage, destination, files);
  }

  ros::NodeHandle private_nh_;
  std::string map_dir_;
  std::string export_id_;
  std::string input_pcd_;
  std::string trajectory_pcd_;
  std::string input_map_yaml_;
  std::string mapping_snapshot_;
  ExportParameters parameters_;
  std::string quality_report_;
};

}  // namespace go2_terrain

int main(int argc, char** argv)
{
  ros::init(argc, argv, "go2_terrain_exporter");
  try
  {
    go2_terrain::TerrainExporter exporter;
    exporter.run();
  }
  catch (const std::exception& error)
  {
    // A one-shot required roslaunch node may terminate before rosconsole has
    // flushed its final line. stderr keeps the fail-closed reason visible to
    // both operators and deployment logs.
    std::cerr << "Terrain export failed: " << error.what() << std::endl;
    ROS_FATAL("Terrain export failed: %s", error.what());
    return 1;
  }
  return 0;
}
