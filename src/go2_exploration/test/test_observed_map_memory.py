import importlib.util
from pathlib import Path
import unittest
import numpy as np

path = Path(__file__).resolve().parents[1]/'scripts/observed_map_memory.py'
spec = importlib.util.spec_from_file_location('observed_memory', str(path))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ObservationMemoryTests(unittest.TestCase):
    def setUp(self):
        self.memory = module.OccupancyMemory(.1, 2., 10000)

    def cell(self, x, y):
        return self.memory.cells[int(np.floor(y/.1))-self.memory.y0,
                                 int(np.floor(x/.1))-self.memory.x0]

    def test_unknown_does_not_erase_old_obstacle_or_free(self):
        self.memory.integrate([.05, .15], [.05, .05], [100, 0])
        self.assertFalse(self.memory.integrate([.05, .15], [.05, .05], [-1, -1]))
        self.assertEqual(self.cell(.05, .05), 100)
        self.assertEqual(self.cell(.15, .05), 0)
        self.assertEqual(self.cell(.25, .05), -1)

    def test_fresh_observation_clears_and_remarks_dynamic_obstacle(self):
        for value in (100, 0, 100):
            self.memory.integrate([.05], [.05], [value])
            self.assertEqual(self.cell(.05, .05), value)

    def test_rolling_window_moves_but_memory_stays(self):
        self.memory.integrate([.05], [.05], [100])
        self.memory.integrate([-2.05, 3.05], [-2.05, 3.05], [0, 100])
        self.assertEqual(self.cell(.05, .05), 100)
        self.assertEqual(self.cell(-2.05, -2.05), 0)
        self.assertEqual(self.cell(3.05, 3.05), 100)

    def test_collision_wins_when_cells_project_to_same_destination(self):
        self.memory.integrate([.01, .02], [.01, .02], [100, 0])
        self.assertEqual(self.cell(.05, .05), 100)

    def test_invalid_or_oversized_input_keeps_existing_map(self):
        original = self.memory.cells.copy()
        for x, y, value in [([0.], [0.], [99]), ([float('nan')], [0.], [0]),
                            ([10000.], [10000.], [0])]:
            with self.assertRaises(ValueError):
                self.memory.integrate(x, y, value)
            np.testing.assert_array_equal(self.memory.cells, original)

    def test_ground_support_is_local_front_only_and_never_a_ray(self):
        output = module.ground_support([[1.,0.,-.3],[-1.,0.,-.3],[2.,1.,float('nan')]])
        self.assertGreater(len(output),0)
        self.assertTrue((output[:,0]>.8).all())
        self.assertTrue((output[:,0]<1.2).all())
        self.assertEqual(len(module.ground_support([[-1.,0.,-.3]])),0)

    def test_history_and_age_survive_rear_blindness_and_expansion(self):
        self.memory.integrate([.05],[.05],[0],10.)
        self.memory.integrate([.05],[.05],[-1],20.)
        self.memory.integrate([3.05],[3.05],[0],30.)
        iy,ix = int(np.floor(.05/.1))-self.memory.y0,int(np.floor(.05/.1))-self.memory.x0
        self.assertEqual(self.memory.cells[iy,ix],0)
        self.assertEqual(self.memory.last_seen[iy,ix],10.)

    def test_turning_observes_former_rear_without_erasing_front_history(self):
        from types import SimpleNamespace as S
        patch = module.ground_support([[1.,0.,-.3]])
        translation = S(x=0.,y=0.,z=.3)
        front = module.transform_points(patch,S(rotation=S(x=0.,y=0.,z=0.,w=1.),translation=translation))
        back = module.transform_points(patch,S(rotation=S(x=0.,y=0.,z=1.,w=0.),translation=translation))
        self.memory.integrate(front[:,0],front[:,1],np.zeros(len(front)),10.)
        self.assertEqual(self.cell(-1.05,-.05),-1)
        self.memory.integrate(back[:,0],back[:,1],np.zeros(len(back)),20.)
        self.assertEqual(self.cell(-1.05,-.05),0)
        self.assertEqual(self.cell(1.05,.05),0)

    def test_latched_coverage_is_not_mutated_into_age(self):
        from nav_msgs.msg import OccupancyGrid
        import rospy
        from unittest.mock import Mock
        node=module.MemoryNode.__new__(module.MemoryNode)
        node.Grid=OccupancyGrid;node.frame='map';node.started=rospy.Time(1)
        node.memory=module.OccupancyMemory(.1,2,10000)
        node.coverage=module.OccupancyMemory(.1,2,10000)
        node.coverage.integrate([.05],[.05],[0],10.)
        node.coverage_stamp=rospy.Time(10)
        node.map_pub=Mock();node.metadata_pub=Mock();node.coverage_pub=Mock();node.coverage_age_pub=Mock()
        node.publish(rospy.Time(20))
        binary=node.coverage_pub.publish.call_args[0][0]
        age=node.coverage_age_pub.publish.call_args[0][0]
        self.assertIsNot(binary,age)
        self.assertEqual(set(binary.data),{-1,0})
        self.assertEqual(set(age.data),{-1,10})


class CoverageTimingTests(unittest.TestCase):
    """Exercise callbacks and TF timestamps without ROS transport or hardware."""
    def setUp(self):
        import rospy
        import threading
        from unittest.mock import Mock, patch
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.point_cloud2 import create_cloud_xyz32
        from std_msgs.msg import Header, String
        self.ros = rospy
        self.node = module.MemoryNode.__new__(module.MemoryNode)
        n = self.node
        n.ros, n.Grid, n.String = rospy, OccupancyGrid, String
        n.frame = 'map'
        n.front, n.rear, n.half_width = .35, .35, .155
        n.support_grid = dict(resolution=.15, min_x=-5., min_y=-4.)
        n.support_range = dict(min_horizontal_range=.35, max_horizontal_range=5.)
        n.memory = module.OccupancyMemory(.05, 4., 100000)
        n.coverage = module.OccupancyMemory(.05, 4., 100000)
        n.ground_lock = threading.Lock()
        n.ground = create_cloud_xyz32(Header(frame_id='terrain_sensor', stamp=rospy.Time.from_sec(99.9)),
                                      [(1., 0., -.3)])
        n.last_ground_stamp = n.coverage_stamp = n.last_stamp = rospy.Time(0)
        n.started = rospy.Time(99)
        for name in ('map_pub', 'metadata_pub', 'coverage_pub', 'coverage_age_pub', 'status_pub'):
            setattr(n, name, Mock())
        self.pose = TransformStamped()
        self.pose.header.stamp = rospy.Time.from_sec(99.998)
        self.pose.transform.rotation.w = 1.
        self.exact = TransformStamped()
        self.exact.transform.rotation.w = 1.
        self.exact.transform.translation.z = .3
        n.tf = Mock(lookup_transform=Mock(side_effect=lambda target, source, stamp, timeout:
                     self.pose if source == 'base_link' else self.exact))
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'map'
        self.grid.header.stamp = rospy.Time(100)
        self.grid.info.resolution = .05
        self.grid.info.width = self.grid.info.height = 2
        self.grid.info.origin.orientation.w = 1.
        self.grid.data = [0, 100, -1, 0]
        for p in (patch.object(rospy.Time, 'now', return_value=rospy.Time(100)),
                  patch.object(rospy, 'logwarn_throttle')):
            p.start()
            self.addCleanup(p.stop)

    def test_footprint_uses_fresh_pose_stamp_and_ground_keeps_exact_stamp(self):
        self.node.receive(self.grid)
        calls = self.node.tf.lookup_transform.call_args_list
        base = [c for c in calls if c.args[1] == 'base_link']
        ground = [c for c in calls if c.args[1] == 'terrain_sensor']
        self.assertEqual(base[0].args[2], self.ros.Time(0))
        self.assertEqual(ground[0].args[2], self.node.ground.header.stamp)
        n = self.node
        iy, ix = -n.coverage.y0, -n.coverage.x0
        self.assertAlmostEqual(n.coverage.last_seen[iy, ix], 99.998)
        self.assertEqual(n.last_stamp, self.grid.header.stamp)
        self.assertTrue(n.status_pub.publish.call_args.args[0].data.startswith('ready:'))

    def test_stale_future_and_unstamped_base_tf_do_not_publish_ready(self):
        for stamp in (99.49, 100.11, 0.):
            with self.subTest(stamp=stamp):
                self.pose.header.stamp = self.ros.Time.from_sec(stamp)
                self.node.receive(self.grid)
                self.node.map_pub.publish.assert_not_called()
                self.assertEqual(self.node.last_stamp, self.ros.Time(0))
                self.assertEqual(self.node.memory.generation, 0)
                self.assertEqual(self.node.coverage.generation, 0)

    def test_failed_frame_can_be_retried_without_partial_commit(self):
        import tf2_ros
        original = self.node.tf.lookup_transform.side_effect
        self.node.tf.lookup_transform.side_effect = tf2_ros.ExtrapolationException('not available yet')
        self.node.receive(self.grid)
        self.assertEqual(self.node.last_stamp, self.ros.Time(0))
        self.assertEqual(self.node.memory.generation, 0)
        self.node.tf.lookup_transform.side_effect = original
        self.node.receive(self.grid)
        self.node.map_pub.publish.assert_called_once()
        self.assertEqual(self.node.memory.generation, 1)
        self.assertEqual(self.node.last_stamp, self.grid.header.stamp)

    def test_stale_ground_does_not_gain_coverage_from_fresh_pose(self):
        self.node.ground.header.stamp = self.ros.Time(99)
        self.node.receive(self.grid)
        self.node.map_pub.publish.assert_not_called()
        self.assertEqual(self.node.coverage.generation, 0)


if __name__ == '__main__':
    unittest.main()
