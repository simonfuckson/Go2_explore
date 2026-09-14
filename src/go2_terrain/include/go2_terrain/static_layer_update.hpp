#pragma once

#include <cstddef>

namespace go2_terrain
{

// Tracks whether a static layer must contribute the complete master-grid
// bounds. A partial callback never consumes the request, which prevents a
// resize/reset race from leaving part of the terrain layer unapplied.
class StaticLayerUpdateState
{
public:
  explicit StaticLayerUpdateState(bool full_update_required = false)
    : full_update_required_(full_update_required)
  {
  }

  void requestFullUpdate() { full_update_required_ = true; }
  void clearFullUpdate() { full_update_required_ = false; }
  bool fullUpdateRequired() const { return full_update_required_; }

  bool acknowledgeWindow(int min_i,
                         int min_j,
                         int max_i,
                         int max_j,
                         std::size_t width,
                         std::size_t height)
  {
    if (!full_update_required_)
    {
      return false;
    }
    if (width == 0 || height == 0 || min_i > 0 || min_j > 0 ||
        max_i < static_cast<int>(width) ||
        max_j < static_cast<int>(height))
    {
      return false;
    }
    full_update_required_ = false;
    return true;
  }

private:
  bool full_update_required_;
};

}  // namespace go2_terrain
