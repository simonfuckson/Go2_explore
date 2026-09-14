# Offline reconstruction provenance

`src/offline_surface.cpp` adapts the distributed PMF / robust plane surface
reconstruction from `WheelTech/wheeltec_stack/wheeltec_map_tools/src/terrain_reclassify.cpp`
in https://github.com/AADCL/ugv, commit
`ad23ac64cde045f26a6dcd5f80d899302ee0fa79`, the implementation identified by the
user-provided `wheeltech_pointcloud_v51.pdf` Route 1.

The upstream package manifest at this revision declares its license as `TODO`.
This adaptation does not claim that upstream material is BSD-3-Clause. The
existing package LICENSE covers the pre-existing Go2 code; upstream licensing
must be resolved before redistributing the adapted source under a blanket license.

Go2-specific changes: native library API, shared grid geometry and six-layer
output, measured base-height anchoring, 35-degree reconstruction, start-connected
surface filtering, coherent measured step edges, obstacle-only wall references,
quality checks, transactional rollback and regression fixtures.

The independent runtime Patchwork++ package and its GPL-3.0 boundary are unchanged.
