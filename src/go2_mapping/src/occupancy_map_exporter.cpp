#include <ros/ros.h>



#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>

#include <pcl/point_types.h>



#include <algorithm>

#include <cctype>

#include <cmath>

#include <cerrno>

#include <cstdio>

#include <cstdint>

#include <cstring>

#include <fstream>

#include <iomanip>

#include <limits>

#include <string>

#include <stdexcept>

#include <vector>



class PcdToPgm

{

public:

  PcdToPgm()

      : pnh_("~")

  {

    pnh_.param<std::string>(

        "input_pcd",

        input_pcd_,

        std::string());

    pnh_.param<std::string>(

        "output_pgm",

        output_pgm_,

        std::string());



    pnh_.param<std::string>(

        "output_yaml",

        output_yaml_,

        std::string());

    pnh_.param<std::string>("export_id", export_id_, std::string());

    pnh_.param<std::string>(

        "export_receipt",

        export_receipt_,

        std::string());

    if (input_pcd_.empty() || output_pgm_.empty() || output_yaml_.empty() ||
        export_id_.empty() || export_receipt_.empty())
    {
      throw std::runtime_error(
          "input_pcd, output_pgm, output_yaml, export_id, and "
          "export_receipt parameters are required");
    }

    if (!validExportId(export_id_))
    {
      throw std::runtime_error("export_id contains unsafe characters");
    }



    pnh_.param<double>("resolution", resolution_, 0.05);

    pnh_.param<double>("padding_m", padding_m_, 0.50);



    // Ground evidence.

    pnh_.param<double>("floor_min_z", floor_min_z_, -0.15);

    pnh_.param<double>("floor_max_z", floor_max_z_, 0.08);



    // Obstacles relevant to GO2 body collision.

    pnh_.param<double>("obstacle_min_z", obstacle_min_z_, 0.08);

    pnh_.param<double>("obstacle_max_z", obstacle_max_z_, 0.70);



    // Fill small holes in floor observations.

    pnh_.param<double>("free_dilation_m", free_dilation_m_, 0.15);



    // Robot safety inflation.

    pnh_.param<double>("obstacle_inflation_m", obstacle_inflation_m_, 0.30);



    pnh_.param<int>("min_floor_points", min_floor_points_, 1);

    pnh_.param<int>("min_obstacle_points", min_obstacle_points_, 1);



    pnh_.param<int>("unknown_value", unknown_value_, 205);

    pnh_.param<int>("free_value", free_value_, 254);

    pnh_.param<int>("occupied_value", occupied_value_, 0);



    run();

  }



private:

  static bool validExportId(const std::string& value)

  {

    if (value.empty() || value.size() > 128)

      return false;

    for (const unsigned char ch : value)

    {

      if (!std::isalnum(ch) && ch != '-' && ch != '_' && ch != '.')

        return false;

    }

    return true;

  }

  static bool finitePoint(const pcl::PointXYZI& p)

  {

    return std::isfinite(p.x) &&

           std::isfinite(p.y) &&

           std::isfinite(p.z);

  }



  int index(int x, int y) const

  {

    return y * width_ + x;

  }



  bool inside(int x, int y) const

  {

    return x >= 0 && x < width_ &&

           y >= 0 && y < height_;

  }



  void dilateMask(const std::vector<uint8_t>& input,

                  std::vector<uint8_t>& output,

                  int radius_cells)

  {

    output = input;



    if (radius_cells <= 0)

      return;



    for (int y = 0; y < height_; ++y)

    {

      for (int x = 0; x < width_; ++x)

      {

        if (!input[index(x, y)])

          continue;



        for (int dy = -radius_cells; dy <= radius_cells; ++dy)

        {

          for (int dx = -radius_cells; dx <= radius_cells; ++dx)

          {

            if (dx * dx + dy * dy >

                radius_cells * radius_cells)

              continue;



            const int nx = x + dx;

            const int ny = y + dy;



            if (inside(nx, ny))

              output[index(nx, ny)] = 1;

          }

        }

      }

    }

  }



  void writePgm(const std::vector<uint8_t>& grid)

  {

    std::ofstream out(output_pgm_.c_str(),

                      std::ios::out | std::ios::binary);



    if (!out)

      throw std::runtime_error(

          "Cannot open output PGM: " + output_pgm_);



    out << "P5\n";

    out << width_ << " " << height_ << "\n";

    out << "255\n";



    // PGM first row is image top.

    // ROS occupancy coordinates use origin at lower-left.

    // Therefore flip Y while writing.

    for (int image_y = height_ - 1;

         image_y >= 0;

         --image_y)

    {

      for (int x = 0; x < width_; ++x)

      {

        const uint8_t v = grid[index(x, image_y)];

        out.write(reinterpret_cast<const char*>(&v), 1);

      }

    }



    out.close();

    if (!out)

      throw std::runtime_error(

          "Failed while writing output PGM: " + output_pgm_);

  }



  void writeYaml()

  {

    std::ofstream out(output_yaml_.c_str());



    if (!out)

      throw std::runtime_error(

          "Cannot open output YAML: " + output_yaml_);



    // map_server resolves a relative image path against YAML directory.

    const std::size_t slash = output_pgm_.find_last_of('/');

    std::string image_name = output_pgm_;



    if (slash != std::string::npos)

      image_name = output_pgm_.substr(slash + 1);



    out << "image: " << image_name << "\n";



    out << std::fixed << std::setprecision(6);



    out << "resolution: " << resolution_ << "\n";



    out << "origin: ["

        << min_x_ << ", "

        << min_y_ << ", 0.0]\n";



    out << "negate: 0\n";

    out << "occupied_thresh: 0.65\n";

    out << "free_thresh: 0.196\n";



    out.close();

    if (!out)

      throw std::runtime_error(

          "Failed while writing output YAML: " + output_yaml_);

  }

  void validateTemporaryMapOutputs(const std::string& pgm_path,
                                   const std::string& yaml_path,
                                   std::size_t expected_pixels) const
  {
    std::ifstream pgm(pgm_path.c_str(), std::ios::in | std::ios::binary);
    std::string magic;
    int width = 0;
    int height = 0;
    int maximum = 0;
    if (!(pgm >> magic >> width >> height >> maximum) || magic != "P5" ||
        width != width_ || height != height_ || maximum != 255)
    {
      throw std::runtime_error("Temporary PGM header validation failed: " +
                               pgm_path);
    }
    char separator = '\0';
    if (!pgm.get(separator) || separator != '\n')
    {
      throw std::runtime_error("Temporary PGM header is not terminated: " +
                               pgm_path);
    }
    const std::streamoff data_start = pgm.tellg();
    pgm.seekg(0, std::ios::end);
    const std::streamoff data_end = pgm.tellg();
    if (data_start < 0 || data_end < data_start ||
        static_cast<std::size_t>(data_end - data_start) != expected_pixels)
    {
      throw std::runtime_error("Temporary PGM payload validation failed: " +
                               pgm_path);
    }

    std::ifstream yaml(yaml_path.c_str());
    std::string first_line;
    if (!std::getline(yaml, first_line))
    {
      throw std::runtime_error("Temporary YAML validation failed: " +
                               yaml_path);
    }
    const std::size_t slash = output_pgm_.find_last_of('/');
    const std::string image_name =
        slash == std::string::npos ? output_pgm_
                                   : output_pgm_.substr(slash + 1);
    if (first_line != "image: " + image_name)
    {
      throw std::runtime_error("Temporary YAML image reference is invalid: " +
                               yaml_path);
    }
  }

  static bool fileExists(const std::string& path)
  {
    std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
    return static_cast<bool>(input);
  }

  static void copyFile(const std::string& source,
                       const std::string& destination)
  {
    std::ifstream input(source.c_str(), std::ios::in | std::ios::binary);
    std::ofstream output(destination.c_str(),
                         std::ios::out | std::ios::binary | std::ios::trunc);
    if (!input || !output)
    {
      std::remove(destination.c_str());
      throw std::runtime_error("Cannot create export rollback backup: " +
                               destination);
    }
    output << input.rdbuf();
    output.close();
    if (input.bad() || !output)
    {
      std::remove(destination.c_str());
      throw std::runtime_error("Failed while writing export rollback backup: " +
                               destination);
    }
  }

  void writeMapOutputsAtomically(const std::vector<uint8_t>& grid)
  {
    const std::string final_pgm = output_pgm_;
    const std::string final_yaml = output_yaml_;
    const std::string temporary_pgm = final_pgm + ".tmp." + export_id_;
    const std::string temporary_yaml = final_yaml + ".tmp." + export_id_;
    const std::string backup_pgm = final_pgm + ".bak." + export_id_;
    const std::string backup_yaml = final_yaml + ".bak." + export_id_;
    if (fileExists(backup_pgm) || fileExists(backup_yaml))
    {
      throw std::runtime_error(
          "Refusing to overwrite an existing export rollback backup for " +
          export_id_);
    }
    std::remove(temporary_pgm.c_str());
    std::remove(temporary_yaml.c_str());
    try
    {
      output_pgm_ = temporary_pgm;
      writePgm(grid);
      output_pgm_ = final_pgm;
      output_yaml_ = temporary_yaml;
      writeYaml();
      output_yaml_ = final_yaml;
      validateTemporaryMapOutputs(temporary_pgm, temporary_yaml, grid.size());
    }
    catch (...)
    {
      output_pgm_ = final_pgm;
      output_yaml_ = final_yaml;
      std::remove(temporary_pgm.c_str());
      std::remove(temporary_yaml.c_str());
      throw;
    }

    const bool had_original_pgm = fileExists(final_pgm);
    const bool had_original_yaml = fileExists(final_yaml);
    bool backed_up_pgm = false;
    bool backed_up_yaml = false;
    try
    {
      if (had_original_pgm)
      {
        copyFile(final_pgm, backup_pgm);
        backed_up_pgm = true;
      }
      if (had_original_yaml)
      {
        copyFile(final_yaml, backup_yaml);
        backed_up_yaml = true;
      }
    }
    catch (...)
    {
      if (backed_up_pgm)
        std::remove(backup_pgm.c_str());
      if (backed_up_yaml)
        std::remove(backup_yaml.c_str());
      std::remove(temporary_pgm.c_str());
      std::remove(temporary_yaml.c_str());
      throw;
    }

    bool committed_pgm = false;
    bool committed_yaml = false;
    std::string commit_error;
    if (std::rename(temporary_pgm.c_str(), final_pgm.c_str()) != 0)
    {
      const int error_number = errno;
      commit_error = "Cannot commit output PGM '" + final_pgm + "': " +
                     std::strerror(error_number);
    }
    else
    {
      committed_pgm = true;
      if (std::rename(temporary_yaml.c_str(), final_yaml.c_str()) != 0)
      {
        const int error_number = errno;
        commit_error = "Cannot commit output YAML '" + final_yaml + "': " +
                       std::strerror(error_number);
      }
      else
      {
        committed_yaml = true;
      }
    }

    if (!commit_error.empty())
    {
      std::string rollback_error;
      const auto append_rollback_error = [&rollback_error](
          const std::string& message) {
        if (!rollback_error.empty())
          rollback_error += "; ";
        rollback_error += message;
      };

      if (committed_pgm)
      {
        if (had_original_pgm)
        {
          if (std::rename(backup_pgm.c_str(), final_pgm.c_str()) != 0)
          {
            const int error_number = errno;
            append_rollback_error("PGM restore failed; backup retained at '" +
                                  backup_pgm + "': " +
                                  std::strerror(error_number));
          }
          else
          {
            backed_up_pgm = false;
          }
        }
        else if (std::remove(final_pgm.c_str()) != 0 && errno != ENOENT)
        {
          const int error_number = errno;
          append_rollback_error("new PGM removal failed: " +
                                std::string(std::strerror(error_number)));
        }
      }
      if (committed_yaml)
      {
        if (had_original_yaml)
        {
          if (std::rename(backup_yaml.c_str(), final_yaml.c_str()) != 0)
          {
            const int error_number = errno;
            append_rollback_error("YAML restore failed; backup retained at '" +
                                  backup_yaml + "': " +
                                  std::strerror(error_number));
          }
          else
          {
            backed_up_yaml = false;
          }
        }
        else if (std::remove(final_yaml.c_str()) != 0 && errno != ENOENT)
        {
          const int error_number = errno;
          append_rollback_error("new YAML removal failed: " +
                                std::string(std::strerror(error_number)));
        }
      }

      // An uncommitted destination was never changed; discard only its exact
      // rollback copy. A failed restoration deliberately retains its backup.
      if (!committed_pgm && backed_up_pgm)
      {
        std::remove(backup_pgm.c_str());
        backed_up_pgm = false;
      }
      if (!committed_yaml && backed_up_yaml)
      {
        std::remove(backup_yaml.c_str());
        backed_up_yaml = false;
      }
      std::remove(temporary_pgm.c_str());
      std::remove(temporary_yaml.c_str());
      if (!rollback_error.empty())
        commit_error += "; rollback incomplete: " + rollback_error;
      throw std::runtime_error(commit_error);
    }

    if (backed_up_pgm && std::remove(backup_pgm.c_str()) != 0)
    {
      ROS_WARN("Committed map but could not remove PGM rollback backup: %s",
               backup_pgm.c_str());
    }
    if (backed_up_yaml && std::remove(backup_yaml.c_str()) != 0)
    {
      ROS_WARN("Committed map but could not remove YAML rollback backup: %s",
               backup_yaml.c_str());
    }
  }



  void writeExportReceipt()

  {

    const std::string temporary = export_receipt_ + ".tmp." + export_id_;

    std::ofstream out(temporary.c_str(), std::ios::out | std::ios::trunc);

    if (!out)

      throw std::runtime_error(

          "Cannot open temporary export receipt: " + temporary);

    out << export_id_ << "\n";

    out.close();

    if (!out)

    {

      std::remove(temporary.c_str());

      throw std::runtime_error(

          "Failed while writing export receipt: " + temporary);

    }

    if (std::rename(temporary.c_str(), export_receipt_.c_str()) != 0)

    {

      std::remove(temporary.c_str());

      throw std::runtime_error(

          "Cannot commit export receipt: " + export_receipt_);

    }

  }



  void run()

  {

    ROS_INFO("Loading PCD: %s", input_pcd_.c_str());



    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(

        new pcl::PointCloud<pcl::PointXYZI>());



    if (pcl::io::loadPCDFile<pcl::PointXYZI>(

            input_pcd_, *cloud) != 0)

    {

      throw std::runtime_error("Failed to load PCD: " + input_pcd_);

    }



    ROS_INFO("Loaded PCD points=%zu", cloud->size());



    if (cloud->empty())

    {

      throw std::runtime_error("Input PCD is empty: " + input_pcd_);

    }



    double xmin = std::numeric_limits<double>::max();

    double ymin = std::numeric_limits<double>::max();



    double xmax = -std::numeric_limits<double>::max();

    double ymax = -std::numeric_limits<double>::max();



    size_t finite_count = 0;



    for (const auto& p : cloud->points)

    {

      if (!finitePoint(p))

        continue;



      xmin = std::min(xmin, static_cast<double>(p.x));

      xmax = std::max(xmax, static_cast<double>(p.x));



      ymin = std::min(ymin, static_cast<double>(p.y));

      ymax = std::max(ymax, static_cast<double>(p.y));



      ++finite_count;

    }



    if (finite_count == 0)

    {

      throw std::runtime_error("No finite points in PCD: " + input_pcd_);

    }



    min_x_ = xmin - padding_m_;

    min_y_ = ymin - padding_m_;



    const double max_x = xmax + padding_m_;

    const double max_y = ymax + padding_m_;



    width_ =

        static_cast<int>(

            std::ceil((max_x - min_x_) / resolution_)) + 1;



    height_ =

        static_cast<int>(

            std::ceil((max_y - min_y_) / resolution_)) + 1;



    ROS_INFO(

        "Map bounds x=[%.3f %.3f] y=[%.3f %.3f]",

        min_x_, max_x, min_y_, max_y);



    ROS_INFO(

        "Map size=%d x %d resolution=%.3f",

        width_, height_, resolution_);



    const size_t n =

        static_cast<size_t>(width_) *

        static_cast<size_t>(height_);



    std::vector<int> floor_count(n, 0);

    std::vector<int> obstacle_count(n, 0);



    for (const auto& p : cloud->points)

    {

      if (!finitePoint(p))

        continue;



      const int gx =

          static_cast<int>(

              std::floor((p.x - min_x_) / resolution_));



      const int gy =

          static_cast<int>(

              std::floor((p.y - min_y_) / resolution_));



      if (!inside(gx, gy))

        continue;



      const int id = index(gx, gy);



      if (p.z >= floor_min_z_ &&

          p.z <= floor_max_z_)

      {

        floor_count[id]++;

      }



      if (p.z >= obstacle_min_z_ &&

          p.z <= obstacle_max_z_)

      {

        obstacle_count[id]++;

      }

    }



    std::vector<uint8_t> floor_mask(n, 0);

    std::vector<uint8_t> obstacle_mask(n, 0);



    for (size_t i = 0; i < n; ++i)

    {

      if (floor_count[i] >= min_floor_points_)

        floor_mask[i] = 1;



      if (obstacle_count[i] >= min_obstacle_points_)

        obstacle_mask[i] = 1;

    }



    const int free_radius =

        static_cast<int>(

            std::round(free_dilation_m_ / resolution_));



    const int obstacle_radius =

        static_cast<int>(

            std::round(obstacle_inflation_m_ / resolution_));



    std::vector<uint8_t> free_dilated;

    std::vector<uint8_t> obstacle_inflated;



    dilateMask(

        floor_mask,

        free_dilated,

        free_radius);



    dilateMask(

        obstacle_mask,

        obstacle_inflated,

        obstacle_radius);



    std::vector<uint8_t> image(

        n,

        static_cast<uint8_t>(unknown_value_));



    // Free evidence first.

    for (size_t i = 0; i < n; ++i)

    {

      if (free_dilated[i])

        image[i] =

            static_cast<uint8_t>(free_value_);

    }



    // Obstacles always override free evidence.

    for (size_t i = 0; i < n; ++i)

    {

      if (obstacle_inflated[i])

        image[i] =

            static_cast<uint8_t>(occupied_value_);

    }



    writeMapOutputsAtomically(image);

    // This marker is the commit record for this invocation and must be last.
    writeExportReceipt();



    size_t free_cells = 0;

    size_t occ_cells = 0;

    size_t unknown_cells = 0;



    for (const auto& v : image)

    {

      if (v == static_cast<uint8_t>(free_value_))

        ++free_cells;

      else if (v == static_cast<uint8_t>(occupied_value_))

        ++occ_cells;

      else

        ++unknown_cells;

    }



    ROS_INFO("PGM saved: %s", output_pgm_.c_str());

    ROS_INFO("YAML saved: %s", output_yaml_.c_str());

    ROS_INFO("Export receipt saved: %s", export_receipt_.c_str());



    ROS_INFO(

        "cells free=%zu occupied=%zu unknown=%zu",

        free_cells,

        occ_cells,

        unknown_cells);



    ros::shutdown();

  }



private:

  ros::NodeHandle pnh_;



  std::string input_pcd_;

  std::string output_pgm_;

  std::string output_yaml_;

  std::string export_id_;

  std::string export_receipt_;



  double resolution_;

  double padding_m_;



  double floor_min_z_;

  double floor_max_z_;



  double obstacle_min_z_;

  double obstacle_max_z_;



  double free_dilation_m_;

  double obstacle_inflation_m_;



  int min_floor_points_;

  int min_obstacle_points_;



  int unknown_value_;

  int free_value_;

  int occupied_value_;



  int width_;

  int height_;



  double min_x_;

  double min_y_;

};



int main(int argc, char** argv)

{

  ros::init(argc, argv, "pcd_to_pgm");



  try

  {

    PcdToPgm converter;

  }

  catch (const std::exception& e)

  {

    ROS_FATAL("pcd_to_pgm exception: %s", e.what());

    return 1;

  }



  return 0;

}
