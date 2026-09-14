#!/usr/bin/env python3
"""Validate a committed GO2 2.5D terrain map before navigation starts."""

import argparse
import hashlib
import math
import os
import re
import struct
import sys

import yaml


LAYER_TYPES = {
    "elevation": ("float32_le", 4),
    "slope": ("float32_le", 4),
    "roughness": ("float32_le", 4),
    "step": ("float32_le", 4),
    "cost": ("uint8", 1),
    "confidence": ("uint8", 1),
}

EXPORT_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,127}$")

LOCKED_PARAMETERS = {
    "ground_max_slope_deg": 35.0,
    "obstacle_min_height_m": 0.05,
    "obstacle_max_height_m": 1.50,
    "trajectory_free_radius_m": 0.18,
    "obstacle_inflation_m": 0.03,
    "preserve_existing_map": 1.0,
    "maximum_reanchor_height_from_initial_m": 0.65,
    "minimum_traced_trajectory_ratio": 0.80,
    "minimum_ground_observation_ratio": 0.30,
    "minimum_largest_ground_component_ratio": 0.60,
    "minimum_ground_to_baseline_free_ratio": 0.08,
    "minimum_trajectory_corridor_known_ratio": 0.95,
    "flat_slope_deg": 8.0,
    "lethal_slope_deg": 30.0,
    "minimum_slope_cost": 15.0,
    "maximum_soft_slope_cost": 80.0,
    "slope_cost_dilation_m": 0.20,
    "minimum_lethal_cluster_cells": 4.0,
}


class ValidationError(RuntimeError):
    pass


def load_yaml(path):
    try:
        with open(path, "r", encoding="utf-8") as stream:
            value = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ValidationError("cannot read YAML {}: {}".format(path, exc))
    if not isinstance(value, dict):
        raise ValidationError("{} must contain a YAML mapping".format(path))
    return value


def safe_asset_path(map_dir, name):
    if not isinstance(name, str) or not name:
        raise ValidationError("asset name must be a non-empty string")
    if os.path.isabs(name):
        raise ValidationError("absolute asset path is not allowed: {}".format(name))
    root = os.path.realpath(map_dir)
    path = os.path.realpath(os.path.join(root, name))
    if os.path.commonpath([root, path]) != root:
        raise ValidationError("asset escapes the map directory: {}".format(name))
    return path


def read_pgm_size(path):
    try:
        with open(path, "rb") as stream:
            data = stream.read()
    except OSError as exc:
        raise ValidationError("cannot read PGM {}: {}".format(path, exc))

    def next_token(offset):
        while offset < len(data):
            if data[offset] in b" \t\r\n":
                offset += 1
                continue
            if data[offset] == ord("#"):
                newline = data.find(b"\n", offset)
                offset = len(data) if newline < 0 else newline + 1
                continue
            break
        start = offset
        while offset < len(data) and data[offset] not in b" \t\r\n#":
            offset += 1
        if start == offset:
            return None, offset
        return data[start:offset], offset

    tokens = []
    offset = 0
    for _ in range(4):
        token, offset = next_token(offset)
        if token is None:
            break
        tokens.append(token)
    if len(tokens) != 4 or tokens[0] not in (b"P5", b"P2"):
        raise ValidationError("unsupported or malformed PGM: {}".format(path))
    try:
        width, height, maximum = map(int, tokens[1:4])
    except ValueError as exc:
        raise ValidationError("invalid PGM dimensions: {}".format(exc))
    if width <= 0 or height <= 0 or maximum <= 0 or maximum > 65535:
        raise ValidationError("invalid PGM header values")
    expected_pixels = width * height
    if tokens[0] == b"P5":
        if offset >= len(data) or data[offset] not in b" \t\r\n":
            raise ValidationError("binary PGM has no raster separator")
        if data[offset:offset + 2] == b"\r\n":
            offset += 2
        else:
            offset += 1
        bytes_per_pixel = 1 if maximum < 256 else 2
        actual_bytes = len(data) - offset
        expected_bytes = expected_pixels * bytes_per_pixel
        if actual_bytes != expected_bytes:
            raise ValidationError(
                "binary PGM raster size {} != {}".format(
                    actual_bytes, expected_bytes
                )
            )
    else:
        pixels = []
        while True:
            token, offset = next_token(offset)
            if token is None:
                break
            try:
                value = int(token)
            except ValueError as exc:
                raise ValidationError("invalid ASCII PGM pixel: {}".format(exc))
            if value < 0 or value > maximum:
                raise ValidationError("ASCII PGM pixel exceeds maxval")
            pixels.append(value)
        if len(pixels) != expected_pixels:
            raise ValidationError(
                "ASCII PGM pixel count {} != {}".format(
                    len(pixels), expected_pixels
                )
            )
    return width, height


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_checksums(path):
    entries = {}
    try:
        with open(path, "r", encoding="ascii") as stream:
            for line_number, raw in enumerate(stream, 1):
                line = raw.strip()
                if not line:
                    continue
                fields = line.split(None, 1)
                if len(fields) != 2 or len(fields[0]) != 64:
                    raise ValidationError(
                        "malformed checksum line {} in {}".format(line_number, path)
                    )
                name = fields[1].lstrip("*")
                if name in entries:
                    raise ValidationError("duplicate checksum entry: {}".format(name))
                try:
                    int(fields[0], 16)
                except ValueError:
                    raise ValidationError("invalid SHA256 for {}".format(name))
                entries[name] = fields[0].lower()
    except OSError as exc:
        raise ValidationError("cannot read checksums: {}".format(exc))
    return entries


def require_number(mapping, key):
    value = mapping.get(key)
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValidationError("{} must be finite numeric".format(key))
    return float(value)


def validate(map_dir, expected_export_id=None):
    map_dir = os.path.realpath(map_dir)
    metadata_path = os.path.join(map_dir, "terrain_2p5d.yaml")
    metadata = load_yaml(metadata_path)

    if metadata.get("format") != "go2_terrain_2p5d":
        raise ValidationError("unsupported terrain format")
    if metadata.get("version") != 1:
        raise ValidationError("unsupported terrain version")
    export_id = metadata.get("export_id")
    if not isinstance(export_id, str) or not EXPORT_ID_PATTERN.fullmatch(export_id):
        raise ValidationError("terrain export_id is missing or unsafe")
    if expected_export_id is not None:
        if not EXPORT_ID_PATTERN.fullmatch(expected_export_id):
            raise ValidationError("expected export_id is unsafe")
        if export_id != expected_export_id:
            raise ValidationError(
                "terrain export_id {} != expected {}".format(
                    export_id, expected_export_id
                )
            )
    if metadata.get("frame_id") != "map":
        raise ValidationError("terrain frame_id must be map")

    map_info = metadata.get("map")
    if not isinstance(map_info, dict):
        raise ValidationError("metadata.map is required")
    width = int(require_number(map_info, "width"))
    height = int(require_number(map_info, "height"))
    resolution = require_number(map_info, "resolution")
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValidationError("map dimensions and resolution must be positive")
    if abs(resolution - 0.05) > 1e-9:
        raise ValidationError("terrain resolution must be the locked 0.05 m")
    if abs(width - require_number(map_info, "width")) > 1e-9 or \
            abs(height - require_number(map_info, "height")) > 1e-9:
        raise ValidationError("map width/height must be integers")
    origin = map_info.get("origin")
    if not isinstance(origin, list) or len(origin) != 3 or not all(
            isinstance(v, (int, float)) and math.isfinite(float(v)) for v in origin):
        raise ValidationError("metadata.map.origin must contain three finite values")

    base_to_floor = require_number(metadata, "base_to_floor_m")
    if not 0.20 <= base_to_floor <= 0.55:
        raise ValidationError("base_to_floor_m is outside [0.20, 0.55]")
    if metadata.get("checksum_file") != "terrain_checksums.sha256":
        raise ValidationError("checksum_file must be terrain_checksums.sha256")

    parameters = metadata.get("parameters")
    if not isinstance(parameters, dict):
        raise ValidationError("metadata.parameters is required")
    revision = parameters.get("reconstruction_revision", 1)
    if revision not in (1, 2):
        raise ValidationError("unsupported reconstruction_revision")
    locked = dict(LOCKED_PARAMETERS)
    if revision == 2:
        for name in list(locked):
            if name.startswith("minimum_") and name not in (
                    "minimum_slope_cost", "minimum_lethal_cluster_cells"):
                del locked[name]
        del locked["maximum_reanchor_height_from_initial_m"]
        locked["preserve_existing_map"] = 0.0
        for name, minimum in (("minimum_trajectory_ground_ratio", 0.80),
                              ("minimum_trajectory_free_ratio", 0.95),
                              ("minimum_trajectory_reachable_ratio", 0.95)):
            if not minimum <= require_number(parameters, name) <= 1:
                raise ValidationError("invalid revision 2 quality limit: " + name)
        quality = load_yaml(os.path.join(map_dir, "terrain_quality.yaml"))
        if quality.get("reconstruction_revision") != 2:
            raise ValidationError("quality report revision mismatch")
        for metric in ("ground", "free", "reachable"):
            name = "trajectory_{}_ratio".format(metric)
            if not require_number(parameters, "minimum_" + name) <= require_number(quality, name) <= 1:
                raise ValidationError("failed export quality metric: " + name)
    for name, expected in locked.items():
        actual = require_number(parameters, name)
        if abs(actual - expected) > 1e-9:
            raise ValidationError(
                "locked terrain parameter {} must be {} (got {})".format(
                    name, expected, actual
                )
            )

    source_pcd = metadata.get("source_pcd")
    trajectory_pcd = metadata.get("trajectory_pcd")
    if source_pcd != "public_map.pcd" or trajectory_pcd != "traversed_path_map.pcd":
        raise ValidationError(
            "terrain sources must be public_map.pcd and traversed_path_map.pcd"
        )
    mapping_snapshot_name = "mapping_snapshot.sha256"
    mapping_snapshot_path = safe_asset_path(map_dir, mapping_snapshot_name)
    mapping_snapshot = parse_checksums(mapping_snapshot_path)
    expected_sources = {source_pcd, trajectory_pcd}
    if set(mapping_snapshot) != expected_sources:
        raise ValidationError(
            "mapping_snapshot.sha256 must contain exactly the two terrain sources"
        )
    for name, expected in mapping_snapshot.items():
        source_path = safe_asset_path(map_dir, name)
        if not os.path.isfile(source_path) or sha256_file(source_path) != expected:
            raise ValidationError("mapping snapshot checksum mismatch: {}".format(name))

    map_yaml_path = os.path.join(map_dir, "map.yaml")
    map_yaml = load_yaml(map_yaml_path)
    if map_yaml.get("negate", 0) != 0:
        raise ValidationError("map.yaml negate must be 0")
    image_name = map_yaml.get("image")
    if image_name != "map.pgm" or map_info.get("image") != "map.pgm":
        raise ValidationError(
            "metadata and map.yaml image must both be exactly map.pgm"
        )
    image_path = safe_asset_path(map_dir, image_name)
    pgm_width, pgm_height = read_pgm_size(image_path)
    if (pgm_width, pgm_height) != (width, height):
        raise ValidationError("PGM dimensions do not match terrain metadata")
    if abs(float(map_yaml.get("resolution", -1.0)) - resolution) > 1e-9:
        raise ValidationError("map.yaml resolution does not match terrain metadata")
    yaml_origin = map_yaml.get("origin")
    if not isinstance(yaml_origin, list) or len(yaml_origin) != 3 or any(
            abs(float(a) - float(b)) > 1e-6 for a, b in zip(yaml_origin, origin)):
        raise ValidationError("map.yaml origin does not match terrain metadata")

    layers = metadata.get("layers")
    if not isinstance(layers, dict):
        raise ValidationError("metadata.layers is required")
    required_assets = {
        "map.yaml",
        "map.pgm",
        "terrain_2p5d.yaml",
        "terrain_ground.pcd",
        "terrain_obstacles.pcd",
        "terrain_preview.ppm",
        source_pcd,
        trajectory_pcd,
        mapping_snapshot_name,
    }
    cell_count = width * height
    if revision == 2:
        required_assets.add("terrain_quality.yaml")
    for layer_name, (expected_type, bytes_per_cell) in LAYER_TYPES.items():
        layer = layers.get(layer_name)
        if not isinstance(layer, dict):
            raise ValidationError("missing layer metadata: {}".format(layer_name))
        if layer.get("type") != expected_type:
            raise ValidationError(
                "layer {} type must be {}".format(layer_name, expected_type)
            )
        asset_name = layer.get("file")
        asset_path = safe_asset_path(map_dir, asset_name)
        try:
            actual_size = os.path.getsize(asset_path)
        except OSError as exc:
            raise ValidationError("cannot stat layer {}: {}".format(layer_name, exc))
        expected_size = cell_count * bytes_per_cell
        if actual_size != expected_size:
            raise ValidationError(
                "layer {} size {} != {}".format(layer_name, actual_size, expected_size)
            )
        required_assets.add(asset_name)

        if expected_type == "float32_le":
            with open(asset_path, "rb") as stream:
                while True:
                    chunk = stream.read(4 * 16384)
                    if not chunk:
                        break
                    values = struct.unpack("<{}f".format(len(chunk) // 4), chunk)
                    if any(math.isinf(value) for value in values):
                        raise ValidationError("layer {} contains infinity".format(layer_name))
        elif layer_name == "confidence":
            with open(asset_path, "rb") as stream:
                if any(value != 255 and value > 100 for value in stream.read()):
                    raise ValidationError("confidence contains a value above 100")

    checksums_path = os.path.join(map_dir, "terrain_checksums.sha256")
    checksums = parse_checksums(checksums_path)
    missing = sorted(required_assets.difference(checksums))
    if missing:
        raise ValidationError("checksums missing required assets: {}".format(", ".join(missing)))
    for name, expected in checksums.items():
        asset_path = safe_asset_path(map_dir, name)
        if not os.path.isfile(asset_path):
            raise ValidationError("checksummed asset is missing: {}".format(name))
        actual = sha256_file(asset_path)
        if actual != expected:
            raise ValidationError("checksum mismatch: {}".format(name))

    return {
        "map_dir": map_dir,
        "width": width,
        "height": height,
        "resolution": resolution,
        "base_to_floor_m": base_to_floor,
        "export_id": export_id,
        "reconstruction_revision": revision,
        "checksums": len(checksums),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", required=True)
    parser.add_argument("--expected-export-id")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    try:
        result = validate(args.map_dir, args.expected_export_id)
    except (ValidationError, OSError, ValueError, TypeError) as exc:
        print("terrain map validation failed: {}".format(exc), file=sys.stderr)
        return 2
    if not args.quiet:
        print(
            "terrain map valid: {width}x{height} @ {resolution:.3f} m, "
            "base-floor={base_to_floor_m:.3f} m, export-id={export_id}, "
            "revision={reconstruction_revision}, checksums={checksums}".format(**result)
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
