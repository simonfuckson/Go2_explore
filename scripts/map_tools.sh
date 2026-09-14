#!/usr/bin/env bash
set -eo pipefail

WS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
MAP_SCHEMA_MARKER=".go2_terrain_mapping_v1"
TERRAIN_METADATA="terrain_2p5d.yaml"
GO2_INTERFACE="${GO2_INTERFACE:-eth0}"
STACK_LOCK_FILE="/tmp/go2_nav_ws_nvidia.stack.lock"
source /opt/ros/noetic/setup.bash
if [[ ! -f "${WS_ROOT}/devel/setup.bash" ]]; then
  echo "Workspace is not built. Run: ${WS_ROOT}/build_workspace.sh" >&2
  exit 2
fi
source "${WS_ROOT}/devel/setup.bash"
set -u

command_name="${1:-help}"
map_name="${2:-site01}"
[[ "${command_name}" == export-map ]] || { echo 'Use run_go2_explore for session commands; this helper only exports maps.' >&2; exit 2; }

require_map_name() {
  if [[ ! "${map_name}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]]; then
    echo "Invalid map name '${map_name}'. Use letters, numbers, '_' or '-', and start with a letter or number." >&2
    exit 2
  fi
}

map_dir() {
  printf '%s/maps/%s' "${WS_ROOT}" "${map_name}"
}

map_has_terrain_evidence() {
  local directory="${1:-$(map_dir)}"
  [[ -e "${directory}/${MAP_SCHEMA_MARKER}" ]] ||
    [[ -e "${directory}/mapping_snapshot.sha256" ]] ||
    [[ -e "${directory}/traversed_path_map.pcd" ]] ||
    compgen -G "${directory}/terrain_*" >/dev/null
}

new_export_id() {
  printf '%s-%s-%s-%s' "$(date -u +%Y%m%dT%H%M%SZ)" "$$" \
    "${RANDOM}" "${RANDOM}"
}

require_navigation_map() {
  local directory
  directory="$(map_dir)"
  for file in public_map.pcd map.pgm map.yaml; do
    if [[ ! -s "${directory}/${file}" ]]; then
      echo "Map '${map_name}' is incomplete: missing or empty ${directory}/${file}" >&2
      exit 5
    fi
  done
}

terrain_mode_for_map() {
  local directory
  directory="$(map_dir)"

  if [[ -f "${directory}/${TERRAIN_METADATA}" ]]; then
    if ! rosrun go2_terrain validate_terrain_map.py --map-dir "${directory}" --quiet; then
      echo "Map '${map_name}' declares terrain mode, but its terrain assets failed validation." >&2
      echo "Refusing to silently fall back to legacy navigation." >&2
      return 2
    fi
    printf 'terrain'
    return 0
  fi

  if map_has_terrain_evidence "${directory}"; then
    echo "Map '${map_name}' is a terrain-enabled/new mapping session, but ${TERRAIN_METADATA} is missing." >&2
    echo "Run: run_go2 export-map ${map_name}" >&2
    return 2
  fi

  printf 'legacy'
}

prepare_new_mapping_directory() {
  local directory marker_tmp
  directory="$(map_dir)"
  mkdir -p "${directory}"

  if [[ -s "${directory}/public_map.pcd" ]] || \
      [[ -s "${directory}/map.yaml" ]] || \
      [[ -s "${directory}/map.pgm" ]]; then
    echo "Refusing to overwrite existing map '${map_name}'. Choose a new map name." >&2
    exit 5
  fi

  marker_tmp="${directory}/${MAP_SCHEMA_MARKER}.tmp.$$"
  printf 'format: go2_terrain_mapping\nversion: 1\ncreated_utc: %s\n' \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)" >"${marker_tmp}"
  mv -f "${marker_tmp}" "${directory}/${MAP_SCHEMA_MARKER}"
}

best_effort_disarm() {
  local service services topic
  # Cancel through both the supervised public action and the remapped internal
  # server. The second path still works if the supervisor itself is unhealthy.
  for topic in /move_base/cancel /move_base_internal/cancel; do
    timeout 2 rostopic pub -1 "${topic}" actionlib_msgs/GoalID '{}' \
      >/dev/null 2>&1 || true
  done
  services="$(timeout 2 rosservice list 2>/dev/null || true)"
  for service in /go2_sdk_bridge_real/enable /go2_sdk_bridge_mock/enable; do
    if grep -qx "${service}" <<<"${services}"; then
      timeout 2 rosservice call "${service}" "data: false" \
        >/dev/null 2>&1 || true
    fi
  done
  for topic in /cmd_vel_nav /cmd_vel_safe; do
    timeout 2 rostopic pub -1 "${topic}" geometry_msgs/Twist '{}' \
      >/dev/null 2>&1 || true
  done
}

resolve_control_service() {
  local services real_present=false mock_present=false
  services="$(timeout 3 rosservice list 2>/dev/null || true)"
  if grep -qx '/go2_sdk_bridge_real/enable' <<<"${services}"; then
    real_present=true
  fi
  if grep -qx '/go2_sdk_bridge_mock/enable' <<<"${services}"; then
    mock_present=true
  fi
  if [[ "${real_present}" == true && "${mock_present}" == true ]]; then
    echo "Both real and mock SDK bridge services are active; refusing ambiguous control." >&2
    return 2
  fi
  if [[ "${real_present}" == true ]]; then
    printf '%s' '/go2_sdk_bridge_real/enable'
    return 0
  fi
  if [[ "${mock_present}" == true ]]; then
    printf '%s' '/go2_sdk_bridge_mock/enable'
    return 0
  fi
  echo "No GO2 SDK bridge enable service is available." >&2
  return 1
}

wait_for_bool_topic() {
  local topic="$1"
  local expected="$2"
  local timeout_seconds="$3"
  local deadline output value
  deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS <= deadline )); do
    output="$(timeout 1 rostopic echo -n 1 "${topic}" 2>/dev/null || true)"
    value="$(awk '/^data:/ {print tolower($2); exit}' <<<"${output}")"
    if [[ "${value}" == "${expected}" ]]; then
      return 0
    fi
    sleep 0.10
  done
  return 1
}

acquire_stack_lock() {
  exec 9>"${STACK_LOCK_FILE}"
  if ! flock -n 9; then
    echo "Another run_go2 mapping, export-map, or navigation process owns the stack lock." >&2
    echo "Stop that process cleanly before retrying." >&2
    exit 3
  fi
}

require_exclusive_stack() {
  local active_nodes
  local conflicting_nodes
  local conflicting_launches
  conflicting_launches="$(pgrep -af '/opt/ros/noetic/bin/[r]oslaunch (go2_bringup (mapping|navigation)\.launch|go2_(mapping|terrain) export_[^ ]+\.launch)' || true)"
  if [[ -n "${conflicting_launches}" ]]; then
    echo "Refusing to start a second GO2 stack. Existing launch process:" >&2
    printf '%s\n' "${conflicting_launches}" >&2
    echo "A process marked T/Tl was suspended with Ctrl+Z. Resume it with fg and stop it with Ctrl+C before retrying." >&2
    exit 3
  fi
  active_nodes="$(rosnode list 2>/dev/null || true)"
  conflicting_nodes="$(printf '%s\n' "${active_nodes}" | grep -E '^/(laserMapping|livox_lidar_publisher2|go2_tf_manager|go2_pose_adapter|cloud_to_base|cloud_for_costmap|cloud_world_to_odom|go2_map_builder|go2_map_exporter|go2_terrain_exporter|go2_map_loader|go2_map_server|go2_ndt_localizer|go2_localization_guard|go2_navigation_supervisor|go2_system_monitor|go2_terrain_cloud_adapter|go2_navigation_patchworkpp|go2_terrain_guard|go2_velocity_shaper|move_base|go2_sdk_bridge_real|go2_sdk_bridge_mock)$' || true)"
  if [[ -n "${conflicting_nodes}" ]]; then
    echo "Refusing to start a second GO2 stack. Conflicting ROS nodes are already running:" >&2
    printf '%s\n' "${conflicting_nodes}" >&2
    echo "Stop the existing mapping/navigation launch with Ctrl+C, verify that these nodes disappear, then retry." >&2
    exit 3
  fi
}

case "${command_name}" in
  mapping)
    require_map_name
    acquire_stack_lock
    require_exclusive_stack
    prepare_new_mapping_directory
    exec roslaunch go2_bringup mapping.launch map_name:="${map_name}" \
      map_root:="${WS_ROOT}/maps" rviz:="${RVIZ:-false}"
    ;;
  save-map)
    save_response="$(timeout 30 rosservice call /go2_map_builder/save_map)"
    printf '%s\n' "${save_response}"
    if ! grep -qE '^success: ([Tt]rue|true)$' <<<"${save_response}"; then
      echo "Map save failed; export-map is not safe to run." >&2
      exit 5
    fi
    ;;
  export-map)
    require_map_name
    acquire_stack_lock
    require_exclusive_stack
    [[ ! -L "$(map_dir)" ]] || { echo 'Refusing a symlink map directory.' >&2; exit 5; }
    for file in map.pgm map.yaml terrain_2p5d.yaml; do
      [[ ! -e "$(map_dir)/${file}" ]] || { echo "Refusing to overwrite exported map asset: $(map_dir)/${file}" >&2; exit 5; }
    done
    terrain_export=false
    if map_has_terrain_evidence "$(map_dir)"; then
      terrain_export=true
      for file in public_map.pcd traversed_path_map.pcd mapping_snapshot.sha256; do
        if [[ ! -s "$(map_dir)/${file}" ]]; then
          echo "Cannot export terrain map: missing or empty $(map_dir)/${file}" >&2
          exit 5
        fi
      done
    fi

    # Keep legacy export for old maps and as a geometry template for revision 2.
    # Terrain maps classify obstacles relative to the reconstructed floor;
    # copying legacy absolute-Z occupancy would turn ramps black again.
    if [[ "${terrain_export}" == true ]]; then
      echo "Preparing map geometry; revision 2 will reconstruct terrain and recover measured walls."
    else
      echo "Legacy map detected; exporting the original 2D occupancy map only."
    fi
    legacy_export_id="$(new_export_id)"
    legacy_export_receipt="$(map_dir)/.go2_legacy_export_receipt"
    legacy_output_args=()
    baseline_pgm=""
    baseline_yaml=""
    if [[ "${terrain_export}" == true ]]; then
      # Keep the committed map set untouched until the terrain exporter
      # has validated and committed every output. Hidden baseline files are
      # transaction-local inputs and are removed on every normal/error exit.
      baseline_pgm="$(map_dir)/.go2_occupancy_baseline_${legacy_export_id}.pgm"
      baseline_yaml="$(map_dir)/.go2_occupancy_baseline_${legacy_export_id}.yaml"
      legacy_export_receipt="$(map_dir)/.go2_occupancy_baseline_${legacy_export_id}.receipt"
      legacy_output_args+=(output_pgm:="${baseline_pgm}")
      legacy_output_args+=(output_yaml:="${baseline_yaml}")
      trap 'rm -f -- "${baseline_pgm}" "${baseline_yaml}" "${legacy_export_receipt}"' EXIT
    fi
    if ! roslaunch go2_mapping export_occupancy.launch map_name:="${map_name}" \
        map_root:="${WS_ROOT}/maps" \
        export_id:="${legacy_export_id}" \
        export_receipt:="${legacy_export_receipt}" \
        "${legacy_output_args[@]}"; then
      echo "Legacy occupancy map exporter failed." >&2
      exit 5
    fi
    if [[ ! -f "${legacy_export_receipt}" ]] ||
        [[ "$(head -n 1 "${legacy_export_receipt}")" != "${legacy_export_id}" ]]; then
      echo "Legacy map export did not commit export_id ${legacy_export_id}." >&2
      exit 5
    fi
    if [[ "${terrain_export}" == true ]]; then
      for file in "${baseline_pgm}" "${baseline_yaml}"; do
        if [[ ! -s "${file}" ]]; then
          echo "Occupancy staging failed: missing or empty ${file}" >&2
          exit 5
        fi
      done
      echo "Staged occupancy baseline validated; committed map remains unchanged."
    else
      for file in map.pgm map.yaml; do
        if [[ ! -s "$(map_dir)/${file}" ]]; then
          echo "Legacy map export failed: missing or empty $(map_dir)/${file}" >&2
          exit 5
        fi
      done
      echo "Occupancy map validated: $(map_dir)"
    fi

    if [[ "${terrain_export}" == true ]]; then
      terrain_export_id="$(new_export_id)"
      if ! roslaunch go2_terrain export_terrain.launch map_name:="${map_name}" \
          map_root:="${WS_ROOT}/maps" \
          input_map_yaml:="${baseline_yaml}" \
          export_id:="${terrain_export_id}"; then
        echo "Terrain map exporter failed for export_id ${terrain_export_id}." >&2
        echo "Inspect the error and any .go2_terrain_recovery_* directory before retrying." >&2
        exit 5
      fi
      # roslaunch may return zero after a required one-shot node exits with an
      # error. Treat the validated, checksum-covered asset set as the commit
      # result rather than trusting the roslaunch process status alone.
      if ! rosrun go2_terrain validate_terrain_map.py \
          --map-dir "$(map_dir)" \
          --expected-export-id "${terrain_export_id}" --quiet; then
        echo "Terrain map export did not commit export_id ${terrain_export_id}." >&2
        exit 5
      fi
      rm -f -- "${baseline_pgm}" "${baseline_yaml}" "${legacy_export_receipt}"
      trap - EXIT
      echo "Revision 2 terrain map, occupancy, quality report and checksums validated: $(map_dir)"
    fi
    ;;
  navigation)
    require_map_name
    acquire_stack_lock
    require_navigation_map
    require_exclusive_stack
    terrain_mode="$(terrain_mode_for_map)" || exit $?
    use_real=false
    if [[ "${3:-}" == "--real" ]]; then
      use_real=true
    fi
    echo "Starting ${terrain_mode} navigation for map '${map_name}'."
    exec roslaunch go2_bringup navigation.launch map_name:="${map_name}" \
      map_root:="${WS_ROOT}/maps" \
      terrain_enabled:="$([[ "${terrain_mode}" == terrain ]] && echo true || echo false)" \
      terrain_metadata:="$(map_dir)/${TERRAIN_METADATA}" \
      use_real_sdk:="${use_real}" network_interface:="${GO2_INTERFACE}" \
      rviz:="${RVIZ:-false}"
    ;;
  enable)
    if ! control_service="$(resolve_control_service)"; then
      best_effort_disarm
      exit 4
    fi
    echo "Clearing old navigation goals and costmaps before enabling control..."
    if ! reset_response="$(timeout 10 rosservice call /go2_navigation_supervisor/reset)"; then
      echo "Navigation reset service transport failed; forcing control disabled." >&2
      best_effort_disarm
      exit 4
    fi
    printf '%s\n' "${reset_response}"
    if ! grep -qE '^success: ([Tt]rue|true)$' <<<"${reset_response}"; then
      echo "Navigation reset failed; forcing control disabled." >&2
      best_effort_disarm
      exit 4
    fi
    if ! enable_response="$(timeout 15 rosservice call "${control_service}" "data: true")"; then
      echo "Control enable service transport failed; forcing control disabled." >&2
      best_effort_disarm
      exit 4
    fi
    printf '%s\n' "${enable_response}"
    if ! grep -qE '^success: ([Tt]rue|true)$' <<<"${enable_response}"; then
      echo "Control enable was rejected by ${control_service}." >&2
      best_effort_disarm
      exit 4
    fi
    if ! wait_for_bool_topic /go2/control/enabled true 5; then
      echo "Enable service succeeded, but /go2/control/enabled did not become true." >&2
      best_effort_disarm
      exit 4
    fi
    if ! wait_for_bool_topic /navigation/ready true 5; then
      echo "Control was enabled, but the complete navigation safety gate is not ready; disarming." >&2
      best_effort_disarm
      exit 4
    fi
    echo "Control enabled. Publish a fresh 2D Nav Goal; old goals are not resumed."
    ;;
  disable)
    if ! control_service="$(resolve_control_service)"; then
      best_effort_disarm
      exit 4
    fi
    if ! disable_response="$(timeout 15 rosservice call "${control_service}" "data: false")"; then
      echo "Control disable service transport failed; sending best-effort stop." >&2
      best_effort_disarm
      exit 4
    fi
    printf '%s\n' "${disable_response}"
    if ! grep -qE '^success: ([Tt]rue|true)$' <<<"${disable_response}"; then
      echo "Control disable service was rejected; sending best-effort stop." >&2
      best_effort_disarm
      exit 4
    fi
    if ! wait_for_bool_topic /go2/control/enabled false 5; then
      echo "Disable succeeded, but disabled state was not confirmed; sending best-effort stop." >&2
      best_effort_disarm
      exit 4
    fi
    echo "Control disabled."
    ;;
  reset-navigation)
    if ! reset_response="$(timeout 10 rosservice call /go2_navigation_supervisor/reset)"; then
      echo "Navigation reset service transport failed; forcing control disabled." >&2
      best_effort_disarm
      exit 4
    fi
    printf '%s\n' "${reset_response}"
    if ! grep -qE '^success: ([Tt]rue|true)$' <<<"${reset_response}"; then
      echo "Navigation reset failed; forcing control disabled." >&2
      best_effort_disarm
      exit 4
    fi
    ;;
  status)
    show_topic() {
      local label="$1"
      local topic="$2"
      echo "=== ${label} (${topic}) ==="
      timeout 3 rostopic echo -n 1 "${topic}" || echo "unavailable"
    }
    show_topic "System ready" /go2/system/ready
    show_topic "Localization state" /localization/state
    show_topic "Localization OK" /localization/ok
    show_topic "Control enabled" /go2/control/enabled
    show_topic "Navigation ready" /navigation/ready
    echo "=== Internal move_base action server ==="
    rosparam get /go2_navigation_supervisor/move_base_internal_connected 2>/dev/null || echo "unavailable"
    show_topic "GO2 diagnostics" /go2/diagnostics
    echo "=== Mapping snapshot/capacity ==="
    for parameter in capacity_ok capacity_error snapshot_writer_busy snapshot_dirty last_snapshot_status fine_candidate_voxels fine_candidate_rejections; do
      value="$(rosparam get "/go2_map_builder/${parameter}" 2>/dev/null || true)"
      printf '%s: %s\n' "${parameter}" "${value:-unavailable}"
    done
    echo "=== Map/terrain mode ==="
    active_terrain_mode="$(rosparam get /go2/terrain/map_mode 2>/dev/null || true)"
    echo "${active_terrain_mode:-unavailable}"
    if [[ "${active_terrain_mode}" == "terrain" ]]; then
      echo "=== Terrain asset status ==="
      active_metadata="$(rosparam get /go2/terrain/metadata_file 2>/dev/null || true)"
      if [[ -n "${active_metadata}" ]]; then
        rosrun go2_terrain validate_terrain_map.py \
          --map-dir "$(dirname "${active_metadata}")" || true
      else
        echo "terrain metadata parameter unavailable"
      fi
      show_topic "Terrain healthy" /terrain/healthy
      show_topic "Terrain runtime (includes output_rate_hz and ground_ratio)" \
        /terrain/status
    else
      echo "Terrain runtime: not used by this legacy map."
    fi
    ;;
  chassis-status)
    echo "=== GO2 battery ==="
    rostopic echo -n 1 /go2/battery_state
    echo "=== GO2 SDK diagnostics ==="
    rostopic echo -n 1 /go2/diagnostics
    ;;
  chassis-self-test)
    test_log="/tmp/go2_chassis_self_test_latest.log"
    rosrun go2_control go2_sdk_official_motion_test_node \
      "${GO2_INTERFACE}" --execute-straight-test 2>&1 | tee "${test_log}"
    echo "Self-test log: ${test_log}"
    ;;
  chassis-standing-test)
    test_log="/tmp/go2_chassis_standing_test_latest.log"
    rosrun go2_control go2_sdk_official_motion_test_node \
      "${GO2_INTERFACE}" --execute-standing-test 2>&1 | tee "${test_log}"
    echo "Standing test log: ${test_log}"
    ;;
  *)
    cat <<'USAGE'
Usage:
  run_go2 mapping <map_name>
  run_go2 save-map
  run_go2 export-map <map_name>
  run_go2 navigation <map_name>          # dry-run, mock SDK
  run_go2 navigation <map_name> --real   # real SDK starts disabled
  run_go2 enable
  run_go2 disable
  run_go2 reset-navigation
  run_go2 status
  run_go2 chassis-status
  run_go2 chassis-self-test             # 0.3 m official SDK straight test
  run_go2 chassis-standing-test         # already standing; direct 0.3 m test

Set RVIZ=true before mapping/navigation to start RViz.
Set GO2_INTERFACE only if the GO2 DDS interface is not eth0.
USAGE
    ;;
esac
