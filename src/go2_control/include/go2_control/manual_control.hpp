#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <array>

namespace go2_control {
struct RemoteSample { float lx=0, ly=0, rx=0, ry=0; uint16_t keys=0; };
// Unitree SDK2 example/state_machine/gamepad.hpp, 40-byte layout. Decode via
// memcpy rather than an unaligned/aliased packed-struct cast (ARM64).
inline bool decodeLowStateRemote(const std::array<uint8_t,40>& bytes, RemoteSample& s) {
  if (bytes[0] != 0x55 || bytes[1] != 0x51) return false;
  s.keys = static_cast<uint16_t>(bytes[2]) | (static_cast<uint16_t>(bytes[3]) << 8);
  auto read_float = [&](int offset) {
    uint32_t value=0;
    for(int i=0;i<4;++i) value |= static_cast<uint32_t>(bytes[offset+i]) << (8*i);
    float result; std::memcpy(&result,&value,sizeof(result)); return result;
  };
  s.lx=read_float(4); s.rx=read_float(8); s.ry=read_float(12); s.ly=read_float(20);
  return std::isfinite(s.lx) && std::isfinite(s.ly) && std::isfinite(s.rx) && std::isfinite(s.ry);
}
enum class RemoteEvent { kNeutral, kManualMotion, kHardTakeover };
// Guarded by remote_mutex_. The supervisor owns the retained navigation goal.
struct ManualControlState {
  bool resumable = false;
  bool active = false;
  double last_sample = -1.0;
  double neutral_since = -1.0;
  double max_axis = 0.0;
  uint16_t keys = 0;

  void pause(bool controlled) {
    if (controlled) resumable = true;
    neutral_since = -1.0;
  }
  void cancelResume() { resumable = false; }
  RemoteEvent observe(double lx, double ly, double rx, double ry,
                      uint16_t buttons, double now, bool controlled) {
    const bool gap = last_sample < 0.0 || now-last_sample > 0.50;
    last_sample = now;
    keys = buttons;
    const bool finite = std::isfinite(lx) && std::isfinite(ly) &&
                        std::isfinite(rx) && std::isfinite(ry);
    max_axis = std::max({std::fabs(lx),std::fabs(ly),std::fabs(rx),std::fabs(ry)});
    if (!finite || max_axis > 1.05 || buttons != 0) {
      active = true; neutral_since = -1.0; resumable = false;
      return RemoteEvent::kHardTakeover;
    }
    // A physical joystick is substantially noisier around center than cmd_vel.
    if (max_axis > 0.12) {
      active = true; pause(controlled);
      return RemoteEvent::kManualMotion;
    }
    active = false;
    if (gap || neutral_since < 0.0) neutral_since = now;
    return RemoteEvent::kNeutral;
  }
  bool neutralAndFresh(double now) const {
    return !active && last_sample >= 0.0 && now-last_sample <= 0.50 &&
        now >= last_sample && neutral_since >= 0.0 && now-neutral_since >= 1.0;
  }
  bool canResume(double now) const { return resumable && neutralAndFresh(now); }
};
}  // namespace go2_control
