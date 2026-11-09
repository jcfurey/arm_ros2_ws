"""Validate motion inputs and preview invalidation independently of a running arm."""
import unittest
from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

from arm_bringup.motion_panel import MotionPanel, joint_targets, matches_start, normalize_pose, scale_value, scene_signature


class MotionInputs(unittest.TestCase):
    def test_speed_rejects_nonfinite_and_out_of_range_values(self):
        self.assertEqual(scale_value("0.25"), 0.25)
        for value in ("nan", "inf", "0", "-1", "1.1", "bad"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                scale_value(value)

    def test_joint_targets_require_every_joint_once_and_obey_limits(self):
        limits = {"a": (-1, 1), "b": (-2, 2)}
        self.assertEqual(joint_targets(["b", "a"], [1.5, 0.5], ["a", "b"], limits), [("a", 0.5), ("b", 1.5)])
        for names, values in [(["a"], [0]), (["a", "a"], [0, 0]), (["a", "c"], [0, 0]),
                              (["a", "b"], [2, 0]), (["a", "b"], [0, float("nan")]), (["a", "b"], [0])]:
            with self.subTest(names=names, values=values), self.assertRaises(ValueError):
                joint_targets(names, values, ["a", "b"], limits)

    def test_pose_normalization_preserves_input_and_rejects_bad_frames(self):
        message = PoseStamped()
        message.header.frame_id = "world"
        message.pose.orientation.w = 2.0
        result = normalize_pose(message)
        self.assertEqual(result.orientation.w, 1.0)
        self.assertEqual(message.pose.orientation.w, 2.0)
        message.header.frame_id = "camera"
        with self.assertRaises(ValueError):
            normalize_pose(message)
        message.header.frame_id = "world"
        message.pose.orientation.w = 0.0
        with self.assertRaises(ValueError):
            normalize_pose(message)
        message.pose.orientation.w = 1.0
        message.pose.position.x = float("inf")
        with self.assertRaises(ValueError):
            normalize_pose(message)

    def test_saved_start_rejects_motion_missing_joints_and_nan(self):
        self.assertTrue(matches_start({"a": 0.1}, {"a": 0.11}))
        for current in ({}, {"a": 0.2}, {"a": float("nan")}):
            self.assertFalse(matches_start({"a": 0.1}, current))
        self.assertFalse(matches_start({}, {}))


class SceneInvalidation(unittest.TestCase):
    def setUp(self):
        self.panel = MotionPanel.__new__(MotionPanel)
        self.panel.scene_check_pending, self.panel.generation = False, 1
        self.panel.saved = {"name": "ready", "scene": b"current"}
        self.current = b"current"
        self.panel.snapshot = Mock(side_effect=lambda name, then, **_kwargs: then(self.current))
        self.panel.invalidate = Mock()

    def scene(self, x=0.0):
        message = PlanningScene(is_diff=True)
        message.robot_state.is_diff = True
        obj = CollisionObject(id="obstacle", operation=CollisionObject.ADD)
        obj.header.frame_id = "world"
        obj.pose.position.x = x
        obj.pose.orientation.w = 1.0
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.1, 0.1, 0.1])]
        message.world.collision_objects = [obj]
        return message

    def test_repeated_scene_and_robot_only_updates_keep_preview_valid(self):
        message = self.scene()
        self.panel.scene_changed(message)
        message.world.collision_objects[0].header.stamp.sec = 123
        self.panel.scene_changed(message)
        self.panel.scene_changed(PlanningScene(is_diff=True))
        self.panel.invalidate.assert_not_called()

    def test_changed_and_removed_obstacles_invalidate_preview(self):
        self.current = b"changed"
        self.panel.scene_changed(self.scene(0.5))
        self.panel.invalidate.assert_called_once()
        self.panel.invalidate.reset_mock()
        message = self.scene()
        message.world.collision_objects[0].operation = CollisionObject.REMOVE
        self.panel.scene_changed(message)
        self.panel.invalidate.assert_called_once()

    def test_full_scene_can_remove_old_obstacles(self):
        self.current = b"empty"
        self.panel.scene_changed(PlanningScene(is_diff=False))
        self.panel.invalidate.assert_called_once()

    def test_unused_part_transform_jitter_does_not_change_the_signature(self):
        scene = self.scene()
        transform = TransformStamped(child_frame_id="parts/pick_cube")
        transform.header.frame_id = "world"
        transform.transform.rotation.w = 1.0
        scene.fixed_frame_transforms = [transform]
        expected = scene_signature(scene)
        scene.fixed_frame_transforms[0].transform.translation.x += 1e-6
        self.assertEqual(scene_signature(scene), expected)
        scene.world.collision_objects[0].pose.position.x += 0.1
        self.assertNotEqual(scene_signature(scene), expected)

    def test_transforms_referenced_by_geometry_are_part_of_the_signature(self):
        scene = self.scene()
        scene.world.collision_objects[0].header.frame_id = "fixture_frame"
        scene.fixed_frame_transforms = [TransformStamped(child_frame_id="fixture_frame")]
        expected = scene_signature(scene)
        scene.fixed_frame_transforms[0].transform.translation.x += 0.1
        self.assertNotEqual(scene_signature(scene), expected)


if __name__ == "__main__":
    unittest.main()
