# Local integration changes

Upstream repository: `https://github.com/url-kaist/patchwork-plusplus-ros`

Pinned upstream commit: `f8c070bf2774b2f3ef622644a511bdfe3f2f27bb`

License: GPL-3.0

The GO2 workspace makes only these integration changes:

- Export upstream `include/` from `catkin_package` so a clean dependent build
  can include `patchworkpp/patchworkpp.hpp`.
- Add `go2_patchwork_wrapper_node`. It keeps subscriber and result publisher
  queues at one, does not latch results, validates the measured 0.43-0.59 m
  sensor-height envelope, requires the `terrain_sensor` frame, preserves scan
  timestamps, and throttles per-frame statistics to one log every five seconds.
- Guard the upstream near-ring elevation/flatness threshold lookup before
  indexing the four-entry adaptive arrays. Upstream evaluated those indexes
  for all fourteen rings before checking `is_near_zone`, which is undefined
  behavior; the condition order is corrected without changing near/far-ring
  classification semantics.
- Limit CMake to the online GO2 wrapper and use the system
  `ros-noetic-jsk-recognition-msgs` package. Upstream demos and the embedded
  JSK source stay in the source snapshot for traceability but are not built.

Apart from the bounds-order correction above, the Patchwork++ estimator math
is unchanged. The GO2 runtime launches the bounded wrapper, not `demo`.
