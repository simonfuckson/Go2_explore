#!/usr/bin/env python3

import hashlib
import importlib.util
import os
import struct
import tempfile
import unittest

import yaml


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), "scripts", "validate_terrain_map.py"
)
SPEC = importlib.util.spec_from_file_location("validate_terrain_map", SCRIPT)
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


class TerrainMapValidatorTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = self.temp.name
        self.width = 2
        self.height = 2
        self.layer_files = {
            "elevation": ("terrain_elevation.f32", "float32_le"),
            "slope": ("terrain_slope.f32", "float32_le"),
            "roughness": ("terrain_roughness.f32", "float32_le"),
            "step": ("terrain_step.f32", "float32_le"),
            "cost": ("terrain_cost.u8", "uint8"),
            "confidence": ("terrain_confidence.u8", "uint8"),
        }
        self._write_valid_map()

    def tearDown(self):
        self.temp.cleanup()

    def _write(self, name, content, mode="wb"):
        path = os.path.join(self.root, name)
        with open(path, mode) as stream:
            stream.write(content)

    def _write_yaml(self, name, value):
        self._write(name, yaml.safe_dump(value, sort_keys=False), "w")

    def _write_checksums(self):
        names = [
            "map.yaml",
            "map.pgm",
            "terrain_2p5d.yaml",
            "terrain_ground.pcd",
            "terrain_obstacles.pcd",
            "terrain_preview.ppm",
            "public_map.pcd",
            "traversed_path_map.pcd",
            "mapping_snapshot.sha256",
        ] + [
            value[0] for value in self.layer_files.values()
        ]
        lines = []
        if os.path.isfile(os.path.join(self.root, "terrain_quality.yaml")):
            names.append("terrain_quality.yaml")
        for name in names:
            with open(os.path.join(self.root, name), "rb") as stream:
                lines.append("{}  {}\n".format(hashlib.sha256(stream.read()).hexdigest(), name))
        self._write("terrain_checksums.sha256", "".join(lines), "w")

    def _write_valid_map(self):
        self._write("public_map.pcd", b"synthetic-static-map")
        self._write("traversed_path_map.pcd", b"synthetic-driven-path")
        source_lines = []
        for name in ("public_map.pcd", "traversed_path_map.pcd"):
            with open(os.path.join(self.root, name), "rb") as stream:
                source_lines.append(
                    "{}  {}\n".format(hashlib.sha256(stream.read()).hexdigest(), name)
                )
        self._write("mapping_snapshot.sha256", "".join(source_lines), "w")
        self._write("map.pgm", b"P5\n2 2\n255\n\xfe\xfe\x00\xcd")
        self._write_yaml(
            "map.yaml",
            {
                "image": "map.pgm",
                "resolution": 0.05,
                "origin": [-1.0, -2.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
        )
        float_data = struct.pack("<4f", 0.0, 0.1, float("nan"), 0.2)
        for layer, (name, layer_type) in self.layer_files.items():
            self._write(name, float_data if layer_type == "float32_le" else bytes([0, 10, 20, 255]))
        self._write("terrain_ground.pcd", b"synthetic-ground")
        self._write("terrain_obstacles.pcd", b"synthetic-obstacles")
        self._write("terrain_preview.ppm", b"P6\n1 1\n255\n\x00\x00\x00")
        self._write_yaml(
            "terrain_2p5d.yaml",
            {
                "format": "go2_terrain_2p5d",
                "version": 1,
                "export_id": "test-export-001",
                "frame_id": "map",
                "source_pcd": "public_map.pcd",
                "trajectory_pcd": "traversed_path_map.pcd",
                "base_to_floor_m": 0.35,
                "checksum_file": "terrain_checksums.sha256",
                "parameters": dict(VALIDATOR.LOCKED_PARAMETERS),
                "map": {
                    "resolution": 0.05,
                    "width": 2,
                    "height": 2,
                    "origin": [-1.0, -2.0, 0.0],
                    "image": "map.pgm",
                },
                "layers": {
                    layer: {"file": value[0], "type": value[1]}
                    for layer, value in self.layer_files.items()
                },
            },
        )
        self._write_checksums()

    def test_valid_map(self):
        result = VALIDATOR.validate(self.root)
        self.assertEqual(result["width"], 2)
        self.assertEqual(result["height"], 2)
        self.assertEqual(result["export_id"], "test-export-001")

    def _revision_two(self):
        metadata = VALIDATOR.load_yaml(os.path.join(self.root, "terrain_2p5d.yaml"))
        metadata["parameters"].update(reconstruction_revision=2, preserve_existing_map=0,
            minimum_trajectory_ground_ratio=.8, minimum_trajectory_free_ratio=.95,
            minimum_trajectory_reachable_ratio=.95)
        self._write_yaml("terrain_2p5d.yaml", metadata)
        self._write_yaml("terrain_quality.yaml", dict(reconstruction_revision=2,
            trajectory_ground_ratio=1, trajectory_free_ratio=1, trajectory_reachable_ratio=1))
        self._write_checksums()

    def test_accepts_revision_two_and_preserves_legacy_validation(self):
        self.assertEqual(VALIDATOR.validate(self.root)["reconstruction_revision"], 1)
        self._revision_two()
        self.assertEqual(VALIDATOR.validate(self.root)["reconstruction_revision"], 2)

    def test_rejects_revision_two_unreachable_path(self):
        self._revision_two()
        quality = VALIDATOR.load_yaml(os.path.join(self.root, "terrain_quality.yaml"))
        quality["trajectory_reachable_ratio"] = .3
        self._write_yaml("terrain_quality.yaml", quality)
        self._write_checksums()
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "quality metric"):
            VALIDATOR.validate(self.root)

    def test_rejects_unknown_reconstruction_revision(self):
        self._revision_two()
        metadata = VALIDATOR.load_yaml(os.path.join(self.root, "terrain_2p5d.yaml"))
        metadata["parameters"]["reconstruction_revision"] = 3
        self._write_yaml("terrain_2p5d.yaml", metadata)
        self._write_checksums()
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "unsupported reconstruction"):
            VALIDATOR.validate(self.root)

    def test_accepts_expected_export_id(self):
        result = VALIDATOR.validate(self.root, "test-export-001")
        self.assertEqual(result["export_id"], "test-export-001")

    def test_rejects_stale_export_id(self):
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "!= expected"):
            VALIDATOR.validate(self.root, "new-export-002")

    def test_rejects_missing_export_id(self):
        metadata = VALIDATOR.load_yaml(os.path.join(self.root, "terrain_2p5d.yaml"))
        del metadata["export_id"]
        self._write_yaml("terrain_2p5d.yaml", metadata)
        self._write_checksums()
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "export_id"):
            VALIDATOR.validate(self.root)

    def test_rejects_wrong_layer_size(self):
        self._write("terrain_cost.u8", bytes([0, 1, 2]))
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "size"):
            VALIDATOR.validate(self.root)

    def test_rejects_checksum_mismatch(self):
        self._write("map.pgm", b"changed")
        with self.assertRaises(VALIDATOR.ValidationError):
            VALIDATOR.validate(self.root)

    def test_rejects_truncated_binary_pgm(self):
        self._write("map.pgm", b"P5\n2 2\n255\n\xfe\xfe\x00")
        self._write_checksums()
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "raster size"):
            VALIDATOR.validate(self.root)

    def test_rejects_map_image_in_subdirectory(self):
        os.mkdir(os.path.join(self.root, "nested"))
        self._write("nested/map.pgm", b"P5\n2 2\n255\n\xfe\xfe\x00\xcd")
        map_yaml = VALIDATOR.load_yaml(os.path.join(self.root, "map.yaml"))
        map_yaml["image"] = "nested/map.pgm"
        self._write_yaml("map.yaml", map_yaml)
        metadata = VALIDATOR.load_yaml(os.path.join(self.root, "terrain_2p5d.yaml"))
        metadata["map"]["image"] = "nested/map.pgm"
        self._write_yaml("terrain_2p5d.yaml", metadata)
        self._write_checksums()
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "exactly map.pgm"):
            VALIDATOR.validate(self.root)

    def test_rejects_asset_outside_map_directory(self):
        metadata = VALIDATOR.load_yaml(os.path.join(self.root, "terrain_2p5d.yaml"))
        metadata["layers"]["cost"]["file"] = "../terrain_cost.u8"
        self._write_yaml("terrain_2p5d.yaml", metadata)
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "escapes"):
            VALIDATOR.validate(self.root)

    def test_rejects_changed_mapping_snapshot_source(self):
        self._write("public_map.pcd", b"new-static-map")
        self._write_checksums()
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "mapping snapshot"):
            VALIDATOR.validate(self.root)

    def test_rejects_unapproved_terrain_profile(self):
        metadata = VALIDATOR.load_yaml(os.path.join(self.root, "terrain_2p5d.yaml"))
        metadata["parameters"]["lethal_slope_deg"] = 31.0
        self._write_yaml("terrain_2p5d.yaml", metadata)
        self._write_checksums()
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "locked terrain parameter"):
            VALIDATOR.validate(self.root)

    def test_rejects_map_without_export_quality_contract(self):
        metadata = VALIDATOR.load_yaml(os.path.join(self.root, "terrain_2p5d.yaml"))
        del metadata["parameters"]["minimum_ground_observation_ratio"]
        self._write_yaml("terrain_2p5d.yaml", metadata)
        self._write_checksums()
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "must be finite numeric"):
            VALIDATOR.validate(self.root)

    def test_rejects_negated_occupancy_map(self):
        map_yaml = VALIDATOR.load_yaml(os.path.join(self.root, "map.yaml"))
        map_yaml["negate"] = 1
        self._write_yaml("map.yaml", map_yaml)
        self._write_checksums()
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "negate must be 0"):
            VALIDATOR.validate(self.root)


if __name__ == "__main__":
    unittest.main()
