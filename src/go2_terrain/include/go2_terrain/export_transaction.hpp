#pragma once
#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include <functional>

namespace go2_terrain {
// The caller holds the workspace lock. Metadata must be the final file.
// On a normal I/O failure restore every old file. If restoration itself fails,
// preserve the recovery directory and report its path. Crash recovery is manual.
void commitExportFiles(const boost::filesystem::path& stage,
                       const boost::filesystem::path& destination,
                       const std::vector<std::string>& files,
                       const std::function<void(std::size_t)>& before_replace = {});
}
