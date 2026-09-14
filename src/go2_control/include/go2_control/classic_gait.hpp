#pragma once
#include <cstdint>

namespace go2_control {
struct ClassicWalkResult {
  int32_t zero_result = -1;
  int32_t classic_result = -1;
  int32_t first_classic_result = -1;
  int32_t reset_result = -1;
  bool reset_attempted = false;
  bool cancelled = false;
  bool accepted() const {
    return !cancelled && zero_result == 0 && classic_result == 0 &&
        (!reset_attempted || reset_result == 0);
  }
};

// Run only while disarmed. Never substitutes controller selection, StandUp,
// legacy SwitchGait, or an unacknowledged request for ClassicWalk(true).
// A generic -1 is not success and has no documented "already selected"
// meaning. At most one acknowledged off/on preparation is attempted while
// disarmed. Transport/lease errors never trigger a gait transition retry.
template <typename SportClient, typename Permitted, typename Settle>
ClassicWalkResult requestClassicWalk(SportClient& client,
                                     Permitted permitted, Settle settle) {
  ClassicWalkResult result;
  if (!permitted()) { result.cancelled = true; return result; }
  result.zero_result = client.Move(0.0f, 0.0f, 0.0f);
  if (result.zero_result != 0) return result;
  if (!permitted()) { result.cancelled = true; return result; }
  result.classic_result = result.first_classic_result = client.ClassicWalk(true);
  if (result.classic_result == -1 && permitted()) {
    result.reset_attempted = true;
    result.reset_result = client.ClassicWalk(false);
    if (result.reset_result != 0) return result;
    settle();
    if (!permitted()) { result.cancelled = true; return result; }
    result.classic_result = client.ClassicWalk(true);
  }
  if (!permitted()) result.cancelled = true;
  return result;
}

template <typename SportClient>
ClassicWalkResult requestClassicWalk(SportClient& client) {
  return requestClassicWalk(client, [] { return true; }, [] {});
}

enum class JoystickRequest { kNotJoystick, kReleased, kTakeover };
inline JoystickRequest joystickRequest(int64_t api, bool valid, bool on) {
  if (api != 1027) return JoystickRequest::kNotJoystick;
  return valid && !on ? JoystickRequest::kReleased : JoystickRequest::kTakeover;
}

inline bool changesSportMode(int64_t api) {
  switch (api) {
    case 1001: case 1002: case 1003: case 1004: case 1005: case 1006:
    case 1007: case 1009: case 1010: case 1011: case 1016: case 1017:
    case 1019: case 1020: case 1022: case 1023: case 1028:
    case 1029: case 1030: case 1031: case 1032: case 1036:
    case 1061: case 1062: case 1063: case 2041: case 2043: case 2044:
    case 2045: case 2046: case 2047: case 2048: case 2049: case 2050:
    case 2051: case 2054: case 2058:
      return true;
    default: return false;
  }
}
}  // namespace go2_control
