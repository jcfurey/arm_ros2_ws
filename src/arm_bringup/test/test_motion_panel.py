"""Validate motion inputs, preview invalidation and the stuck-operation watchdog without a running arm."""
import importlib.util
import math
from pathlib import Path
import time
import unittest
from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes, PlanningScene
import rclpy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from arm_bringup.motion_panel import (MOVING, LatestJointStates, MotionPanel, joint_targets, matches_start,
                                      normalize_pose, scale_value, scene_signature)

# workcell_ui is installed as an executable, not a module.
_spec = importlib.util.spec_from_file_location(
    "workcell_ui", Path(__file__).resolve().parents[1] / "scripts" / "workcell_ui.py")
workcell_ui = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(workcell_ui)


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
        self.panel.ui = Mock(busy=False)
        self.panel.scene_sent, self.panel.generation = -math.inf, 1
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

    def test_an_unchanged_scene_keeps_the_preview(self):
        self.panel.check_scene()
        self.panel.check_scene()
        self.assertEqual(self.panel.snapshot.call_count, 2)
        self.panel.invalidate.assert_not_called()

    def test_a_changed_scene_clears_the_preview(self):
        self.current = b"changed"
        self.panel.check_scene()
        self.panel.invalidate.assert_called_once_with("Collision scene changed; plan again")

    def test_nothing_is_asked_without_a_preview_or_during_an_operation(self):
        self.panel.ui.busy = True
        self.panel.check_scene()
        self.panel.ui.busy, self.panel.saved = False, None
        self.panel.check_scene()
        self.panel.snapshot.assert_not_called()

    def test_an_unanswered_query_blocks_the_next_one_only_for_a_while(self):
        self.panel.snapshot = Mock()  # MoveIt never answers
        self.panel.check_scene()
        self.panel.check_scene()
        self.assertEqual(self.panel.snapshot.call_count, 1)
        self.panel.scene_sent = time.monotonic() - 6.0
        self.panel.check_scene()
        self.assertEqual(self.panel.snapshot.call_count, 2)

    def test_an_answer_for_a_replaced_preview_is_ignored(self):
        answers = []
        self.panel.snapshot = Mock(side_effect=lambda name, then, **_kwargs: answers.append(then))
        self.panel.check_scene()
        self.panel.saved, self.panel.generation = {"name": "sweep", "scene": b"newer"}, 2
        answers[0](b"changed")
        self.panel.invalidate.assert_not_called()

    def test_a_failed_query_clears_the_preview(self):
        self.panel.snapshot = Mock(side_effect=lambda name, then, on_error, **_kwargs: on_error("no answer"))
        self.panel.check_scene()
        self.panel.invalidate.assert_called_once_with("no answer")

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

    def test_a_live_occupancy_map_is_not_part_of_the_signature(self):
        scene = self.scene()
        expected = scene_signature(scene)
        scene.world.octomap.header.frame_id = "world"
        scene.world.octomap.octomap.resolution = 0.04
        scene.world.octomap.octomap.data = [1, 2, 3]
        self.assertEqual(scene_signature(scene), expected)
        scene.world.octomap.octomap.data = [3, 2, 1, 0]
        self.assertEqual(scene_signature(scene), expected)
        self.assertEqual(list(scene.world.octomap.octomap.data), [3, 2, 1, 0])


class PreviewBookkeeping(unittest.TestCase):
    def setUp(self):
        self.panel = MotionPanel.__new__(MotionPanel)
        self.panel.ui = Mock(busy=False)
        self.panel.preview_pub = Mock()
        self.panel.generation, self.panel.saved, self.panel.invalid_reason = 1, None, "earlier"

    def test_discard_explains_only_a_preview_that_existed(self):
        self.panel.discard("Speed changed; plan again")
        self.assertEqual(self.panel.invalid_reason, "")
        self.panel.saved = {"name": "ready"}
        self.panel.discard("Speed changed; plan again")
        self.assertEqual(self.panel.invalid_reason, "Speed changed; plan again")
        self.assertIsNone(self.panel.saved)
        self.assertEqual(self.panel.generation, 3)

    def test_joint_motion_is_noticed_and_a_moved_arm_clears_the_preview(self):
        positions = {"a": 0.5}
        self.panel.joint_states = Mock(read=Mock(return_value=positions))
        self.panel.moved_from, self.panel.last_motion = {}, 0.0
        self.panel.poll_joints()  # the first reading counts
        first = self.panel.last_motion
        self.assertGreater(first, 0.0)
        self.panel.last_motion = 0.0
        positions["a"] += MOVING / 2
        self.panel.poll_joints()
        self.assertEqual(self.panel.last_motion, 0.0)
        positions["a"] += MOVING
        self.panel.poll_joints()
        self.assertGreater(self.panel.last_motion, 0.0)
        positions["b"] = float("nan")  # a joint without a reading is not motion
        self.panel.poll_joints()
        self.panel.last_motion = 0.0
        self.panel.poll_joints()
        self.assertEqual(self.panel.last_motion, 0.0)

        self.panel.invalidate = Mock()
        self.panel.saved = {"start": {"a": positions["a"]}}
        self.panel.poll_joints()
        self.panel.invalidate.assert_not_called()
        positions["a"] += 0.02
        self.panel.poll_joints()
        self.panel.invalidate.assert_called_once()


class JointStateReader(unittest.TestCase):
    def test_messages_are_taken_on_request_without_spinning(self):
        rclpy.init()
        try:
            owner = rclpy.create_node("reader_owner")
            source = rclpy.create_node("reader_source")
            publisher = source.create_publisher(JointState, "/reader_test/joint_states", 5)
            reader = LatestJointStates(owner, "/reader_test/joint_states")
            self.assertEqual(reader.read(), {})
            end = time.monotonic() + 10
            while not reader.read() and time.monotonic() < end:
                publisher.publish(JointState(name=["a", "b"], position=[0.1, 0.2]))
                time.sleep(0.05)
            self.assertEqual(reader.read(), {"a": 0.1, "b": 0.2})
            publisher.publish(JointState(name=["b"], position=[0.3]))
            end = time.monotonic() + 10
            while reader.read()["b"] != 0.3 and time.monotonic() < end:
                time.sleep(0.05)
            self.assertEqual(reader.read(), {"a": 0.1, "b": 0.3})
            reader.destroy()
            owner.destroy_node()
            source.destroy_node()
        finally:
            rclpy.shutdown()


class StuckOperations(unittest.TestCase):
    def setUp(self):
        self.ui = workcell_ui.WorkcellUI.__new__(workcell_ui.WorkcellUI)
        self.ui.busy, self.ui.stop_requested, self.ui.goal, self.ui.epoch = False, False, None, 0
        self.ui.progress = time.monotonic()
        self.ui.motion = Mock(last_motion=0.0)
        self.ui.stop_pub = Mock()
        self.ui.get_logger = Mock()
        self.ui.finish = Mock()

    def test_answers_count_as_progress_and_late_ones_are_dropped(self):
        answered = Mock()
        callback = self.ui.guard(answered)
        self.ui.progress = 0.0
        callback("first")
        answered.assert_called_once_with("first")
        self.assertGreater(self.ui.progress, 0.0)
        self.ui.epoch += 1
        callback("late")
        answered.assert_called_once_with("first")

    def test_an_idle_ui_and_a_recent_answer_are_left_alone(self):
        self.ui.progress = 0.0
        self.ui.watchdog()
        self.ui.begin("ready")
        self.ui.watchdog()
        self.ui.finish.assert_not_called()

    def test_a_silent_operation_is_given_up_and_its_goal_cancelled(self):
        self.ui.begin("ready")
        self.ui.goal = Mock()
        self.ui.progress = time.monotonic() - workcell_ui.ANSWER_TIMEOUT - 1
        self.ui.watchdog()
        self.assertEqual(self.ui.epoch, 1)
        self.ui.goal.cancel_goal_async.assert_called_once()
        self.ui.stop_pub.publish.assert_called_once()
        self.ui.motion.clear.assert_called_once()
        name, code = self.ui.finish.call_args.args
        self.assertEqual((name, code), ("ready", MoveItErrorCodes.TIMED_OUT))
        self.assertIn("MoveIt has not answered", self.ui.finish.call_args.kwargs["reason"])

    def test_a_moving_arm_is_progress(self):
        self.ui.begin("ready")
        self.ui.progress = time.monotonic() - workcell_ui.ANSWER_TIMEOUT - 1
        self.ui.motion.last_motion = time.monotonic()
        self.ui.watchdog()
        self.ui.finish.assert_not_called()

    def test_stop_shortens_the_wait(self):
        self.ui.begin("ready")
        self.ui.progress = time.monotonic() - workcell_ui.STOP_TIMEOUT - 1
        self.ui.watchdog()
        self.ui.finish.assert_not_called()
        self.ui.stop_requested = True
        self.ui.watchdog()
        self.ui.finish.assert_called_once()


if __name__ == "__main__":
    unittest.main()
