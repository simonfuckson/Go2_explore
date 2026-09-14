#pragma once

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>
#include <openssl/evp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

namespace go2_mapping {

inline std::string temporaryPcdPath(const std::string& output_path) {
  return output_path + ".tmp";
}

inline void discardPreparedPcd(const std::string& output_path) {
  boost::system::error_code ignored;
  boost::filesystem::remove(
      boost::filesystem::path(temporaryPcdPath(output_path)), ignored);
}

template <typename PointT>
bool prepareBinaryPcd(const std::string& output_path,
                      const pcl::PointCloud<PointT>& cloud,
                      std::string* error) {
  try {
    const boost::filesystem::path output(output_path);
    if (output.has_parent_path()) {
      boost::filesystem::create_directories(output.parent_path());
    }
    discardPreparedPcd(output_path);
    const std::string temporary = temporaryPcdPath(output_path);
    if (pcl::io::savePCDFileBinary(temporary, cloud) != 0) {
      *error = "PCL failed to write " + temporary;
      return false;
    }
    return true;
  } catch (const std::exception& exception) {
    *error = exception.what();
    return false;
  }
}

// The temporary file is in the destination directory, so POSIX rename is an
// atomic replacement. This is the final operation used by manual, automatic,
// and shutdown saves on the Ubuntu target.
inline bool commitPreparedPcd(const std::string& output_path,
                              std::string* error) {
  const std::string temporary = temporaryPcdPath(output_path);
  if (std::rename(temporary.c_str(), output_path.c_str()) != 0) {
    const int error_number = errno;
    *error = "rename(" + temporary + ", " + output_path + ") failed: " +
             std::strerror(error_number);
    return false;
  }
  return true;
}

inline bool sha256File(const std::string& path, std::string* digest,
                       std::string* error) {
  error->clear();
  std::ifstream input(path, std::ios::in | std::ios::binary);
  if (!input) {
    *error = "Cannot open for SHA-256: " + path;
    return false;
  }
  EVP_MD_CTX* context = EVP_MD_CTX_new();
  if (context == nullptr) {
    *error = "EVP_MD_CTX_new failed";
    return false;
  }
  bool success = EVP_DigestInit_ex(context, EVP_sha256(), nullptr) == 1;
  char buffer[1024 * 1024];
  while (success && input.good()) {
    input.read(buffer, sizeof(buffer));
    const std::streamsize count = input.gcount();
    if (count > 0 &&
        EVP_DigestUpdate(context, buffer, static_cast<std::size_t>(count)) !=
            1) {
      success = false;
    }
  }
  if (input.bad()) {
    success = false;
    *error = "Read failed while hashing: " + path;
  }
  unsigned char bytes[EVP_MAX_MD_SIZE];
  unsigned int byte_count = 0;
  if (success && EVP_DigestFinal_ex(context, bytes, &byte_count) != 1) {
    success = false;
  }
  EVP_MD_CTX_free(context);
  if (!success || byte_count != 32U) {
    if (error->empty()) {
      *error = "OpenSSL SHA-256 failed for " + path;
    }
    return false;
  }
  std::ostringstream hex;
  hex << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < byte_count; ++index) {
    hex << std::setw(2) << static_cast<unsigned int>(bytes[index]);
  }
  *digest = hex.str();
  return true;
}

inline bool buildSnapshotManifest(const std::string& public_map_path,
                                  const std::string& trajectory_path,
                                  std::string* contents,
                                  std::string* error) {
  std::string map_digest;
  std::string trajectory_digest;
  if (!sha256File(public_map_path, &map_digest, error) ||
      !sha256File(trajectory_path, &trajectory_digest, error)) {
    return false;
  }
  const std::string map_name =
      boost::filesystem::path(public_map_path).filename().string();
  const std::string trajectory_name =
      boost::filesystem::path(trajectory_path).filename().string();
  if (map_name != "public_map.pcd" ||
      trajectory_name != "traversed_path_map.pcd") {
    *error = "Snapshot requires public_map.pcd and traversed_path_map.pcd";
    return false;
  }
  *contents = map_digest + "  " + map_name + "\n" + trajectory_digest +
              "  " + trajectory_name + "\n";
  return true;
}

inline bool invalidateSnapshotManifest(const std::string& manifest_path,
                                       std::string* error) {
  error->clear();
  const std::string temporary = temporaryPcdPath(manifest_path);
  const auto remove_if_present = [error](const std::string& path,
                                         const char* description) {
    errno = 0;
    if (std::remove(path.c_str()) == 0 || errno == ENOENT) {
      return true;
    }
    const int error_number = errno;
    *error = std::string("Failed to remove ") + description + " " + path +
             ": " + std::strerror(error_number);
    return false;
  };
  if (!remove_if_present(manifest_path, "committed snapshot manifest") ||
      !remove_if_present(temporary, "temporary snapshot manifest")) {
    return false;
  }
  return true;
}

inline void discardSnapshotManifest(const std::string& manifest_path) {
  std::string ignored;
  invalidateSnapshotManifest(manifest_path, &ignored);
}

inline bool prepareSnapshotManifest(const std::string& manifest_path,
                                    const std::string& contents,
                                    std::string* error) {
  try {
    const boost::filesystem::path output(manifest_path);
    if (output.has_parent_path()) {
      boost::filesystem::create_directories(output.parent_path());
    }
    const std::string temporary = temporaryPcdPath(manifest_path);
    boost::system::error_code ignored;
    boost::filesystem::remove(boost::filesystem::path(temporary), ignored);
    std::ofstream stream(temporary,
                         std::ios::out | std::ios::binary | std::ios::trunc);
    stream.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    stream.flush();
    stream.close();
    if (!stream) {
      *error = "Failed to write snapshot manifest: " + temporary;
      return false;
    }
    return true;
  } catch (const std::exception& exception) {
    *error = exception.what();
    return false;
  }
}

inline bool commitPreparedSnapshotManifest(const std::string& manifest_path,
                                           std::string* error) {
  return commitPreparedPcd(manifest_path, error);
}

template <typename PointT>
bool writeMappingSnapshot(const std::string& public_map_path,
                          const std::string& trajectory_path,
                          const std::string& manifest_path,
                          const pcl::PointCloud<PointT>& map,
                          const pcl::PointCloud<PointT>& trajectory,
                          std::string* message) {
  if (map.empty() || trajectory.empty()) {
    *message = "Mapping snapshot requires non-empty map and trajectory clouds";
    return false;
  }
  if (!prepareBinaryPcd(public_map_path, map, message)) {
    return false;
  }
  if (!prepareBinaryPcd(trajectory_path, trajectory, message)) {
    discardPreparedPcd(public_map_path);
    return false;
  }

  // Removing the prior manifest changes the snapshot state to invalid before
  // either committed PCD changes. The new manifest is the final commit marker.
  if (!invalidateSnapshotManifest(manifest_path, message)) {
    discardPreparedPcd(public_map_path);
    discardPreparedPcd(trajectory_path);
    return false;
  }
  if (!commitPreparedPcd(trajectory_path, message)) {
    discardPreparedPcd(public_map_path);
    discardPreparedPcd(trajectory_path);
    return false;
  }
  if (!commitPreparedPcd(public_map_path, message)) {
    discardPreparedPcd(public_map_path);
    return false;
  }

  std::string manifest_contents;
  if (!buildSnapshotManifest(public_map_path, trajectory_path,
                             &manifest_contents, message) ||
      !prepareSnapshotManifest(manifest_path, manifest_contents, message) ||
      !commitPreparedSnapshotManifest(manifest_path, message)) {
    discardSnapshotManifest(manifest_path);
    return false;
  }
  *message = "Committed " + std::to_string(map.size()) + " map points and " +
             std::to_string(trajectory.size()) +
             " trajectory points via snapshot manifest " + manifest_path;
  return true;
}

struct SnapshotWriteResult {
  bool had_work = false;
  bool success = false;
  uint64_t mutation_sequence = 0;
  std::string message;
};

template <typename PointT>
class AsyncSnapshotWriter {
 public:
  using Cloud = pcl::PointCloud<PointT>;

  AsyncSnapshotWriter() = default;
  AsyncSnapshotWriter(const AsyncSnapshotWriter&) = delete;
  AsyncSnapshotWriter& operator=(const AsyncSnapshotWriter&) = delete;

  ~AsyncSnapshotWriter() {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  bool start(const std::string& public_map_path,
             const std::string& trajectory_path,
             const std::string& manifest_path, Cloud map, Cloud trajectory,
             uint64_t mutation_sequence, std::string* error) {
    if (worker_.joinable()) {
      *error = "A mapping snapshot writer is already active or awaiting reap";
      return false;
    }
    completed_.store(false, std::memory_order_relaxed);
    result_ = SnapshotWriteResult();
    try {
      worker_ = std::thread(
          [this, public_map_path, trajectory_path, manifest_path,
           map = std::move(map), trajectory = std::move(trajectory),
           mutation_sequence]() mutable {
            SnapshotWriteResult result;
            result.had_work = true;
            result.mutation_sequence = mutation_sequence;
            try {
              result.success = writeMappingSnapshot(
                  public_map_path, trajectory_path, manifest_path, map,
                  trajectory, &result.message);
            } catch (const std::exception& exception) {
              result.success = false;
              result.message =
                  std::string("Unhandled mapping snapshot exception: ") +
                  exception.what();
            } catch (...) {
              result.success = false;
              result.message = "Unhandled non-standard mapping snapshot "
                               "exception";
            }
            result_ = std::move(result);
            completed_.store(true, std::memory_order_release);
          });
    } catch (const std::exception& exception) {
      *error = std::string("Failed to start mapping snapshot writer: ") +
               exception.what();
      return false;
    }
    return true;
  }

  bool busy() const { return worker_.joinable(); }

  bool reapIfComplete(SnapshotWriteResult* result) {
    if (!worker_.joinable() ||
        !completed_.load(std::memory_order_acquire)) {
      return false;
    }
    worker_.join();
    *result = result_;
    result_ = SnapshotWriteResult();
    completed_.store(false, std::memory_order_relaxed);
    return true;
  }

  SnapshotWriteResult wait() {
    if (!worker_.joinable()) {
      return SnapshotWriteResult();
    }
    worker_.join();
    SnapshotWriteResult result = result_;
    result_ = SnapshotWriteResult();
    completed_.store(false, std::memory_order_relaxed);
    return result;
  }

 private:
  std::thread worker_;
  std::atomic<bool> completed_{false};
  SnapshotWriteResult result_;
};

}  // namespace go2_mapping
