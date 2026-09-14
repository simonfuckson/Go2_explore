#include "go2_terrain/export_transaction.hpp"
#include <stdexcept>
#include <exception>

namespace go2_terrain {
void commitExportFiles(const boost::filesystem::path& stage,
                       const boost::filesystem::path& destination,
                       const std::vector<std::string>& files,
                       const std::function<void(std::size_t)>& before_replace) {
  namespace fs=boost::filesystem;
  // Check all targets before mutating any. Hard links keep recovery inexpensive
  // and preserve the old bytes when each staged file is renamed over its target.
  for (const auto& name:files) {
    if (fs::path(name).filename().string()!=name || !fs::is_regular_file(stage/name))
      throw std::runtime_error("Invalid staged export asset: "+name);
    if (fs::exists(destination/name) && (!fs::is_regular_file(destination/name) ||
                                        fs::is_symlink(destination/name)))
      throw std::runtime_error("Invalid export destination: "+name);
  }
  const auto recovery=destination/fs::unique_path(".go2_terrain_recovery_%%%%-%%%%");
  fs::create_directory(recovery);
  std::vector<std::string> replaced;
  try {
    for (const auto& name:files) if (fs::exists(destination/name))
      fs::create_hard_link(destination/name,recovery/name);
    for (const auto& name:files) {
      if (before_replace) before_replace(replaced.size());
      fs::rename(stage/name,destination/name);
      replaced.push_back(name);
    }
  } catch (...) {
    const auto failure=std::current_exception();
    try {
      for (auto it=replaced.rbegin();it!=replaced.rend();++it) {
        if (fs::exists(recovery/(*it))) fs::rename(recovery/(*it),destination/(*it));
        else fs::remove(destination/(*it));
      }
      fs::remove_all(recovery);
    } catch (...) {
      throw std::runtime_error("Export failed and rollback needs recovery from "+recovery.string());
    }
    std::rethrow_exception(failure);
  }
  // A cleanup error after a successful commit must not masquerade as a failed
  // export. A retained recovery directory contains only the previous assets.
  boost::system::error_code error;
  fs::remove_all(recovery,error);
}
}
