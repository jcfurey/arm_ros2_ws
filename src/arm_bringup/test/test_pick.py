"""Check the pick geometry: gripper kinematics, grasp yaws, and grasp heights."""
import math
import unittest

from arm_bringup.pick import (FINGER_REACH, KNUCKLE_CLEARANCE, PAD_OFFSET, TCP_ABOVE_BENCH, PickPlace,
                              drive_for_gap, grasp_yaws, half_extents)
from arm_bringup.scene import SceneObject, quaternion


def part(shape, dimensions, yaw=0.0):
    pose = (0.0, 0.0, 0.0, *quaternion((0.0, 0.0, yaw)))
    return SceneObject(id="sdf/part/body/solid", shape=shape, dimensions=dimensions, pose=pose,
                       color=(1, 1, 1, 1), model="part")


def same_angles(first, second):
    """Equal as sets of directions, ignoring order and multiples of 2 pi."""
    wrap = lambda angles: sorted(round(math.atan2(math.sin(a), math.cos(a)), 6) % round(2 * math.pi, 6)
                                 for a in angles)  # noqa: E731
    return wrap(first) == wrap(second)


class PickTests(unittest.TestCase):
    def test_drive_for_gap_inverts_the_four_bar(self):
        a, b = FINGER_REACH
        for theta in (0.0, 0.2, 0.5, 0.8):
            gap = PAD_OFFSET + a * math.cos(theta) - b * math.sin(theta)
            self.assertAlmostEqual(drive_for_gap(gap), theta, places=6)
        # Beyond the stroke the command saturates at the joint limits.
        self.assertEqual(drive_for_gap(1.0), 0.0)
        self.assertEqual(drive_for_gap(-1.0), 0.85)

    def test_rectangular_box_closes_across_its_narrow_faces(self):
        # Narrow along x: the pads (link_tcp y) must point along the box's x axis.
        narrow_x = part("box", (0.02, 0.06, 0.04), yaw=0.3)
        self.assertTrue(same_angles(grasp_yaws([narrow_x], narrow_x.pose), [0.3 - math.pi / 2, 0.3 + math.pi / 2]))
        narrow_y = part("box", (0.06, 0.02, 0.04), yaw=0.3)
        self.assertTrue(same_angles(grasp_yaws([narrow_y], narrow_y.pose), [0.3, 0.3 + math.pi]))
        self.assertAlmostEqual(half_extents([narrow_y])[1], 0.01)

    def test_square_parts_offer_four_yaws(self):
        cube = part("box", (0.04, 0.04, 0.04), yaw=0.3)
        self.assertTrue(same_angles(grasp_yaws([cube], cube.pose), [0.3 + k * math.pi / 2 for k in range(4)]))
        cylinder = part("cylinder", (0.08, 0.02))
        self.assertEqual(len(grasp_yaws([cylinder], cylinder.pose)), 4)
        self.assertEqual(half_extents([cylinder]), (0.04, 0.02))

    def test_grasp_height_keeps_tips_and_knuckles_clear(self):
        picker = PickPlace(0.75, {"open": {"drive_joint": 0.005}, "close": {"drive_joint": 0.85}})
        for centre, half_height in ((0.77, 0.02), (0.79, 0.04)):
            steps = picker.steps((0.0, 0.1, centre, 0.0, 0.0, 0.0, 1.0), half_height, 0.02, (0.1, 0.0), [0.0])
            names = [step[0] for step in steps]
            self.assertEqual(names, ["open", "approach", "descend", "grasp", "attach", "lift", "carry", "lower",
                                     "detach", "release", "retreat"])
            z = dict((step[0], step) for step in steps)["descend"][2][0][2]
            self.assertGreaterEqual(z, 0.75 + TCP_ABOVE_BENCH - 1e-9)
            self.assertGreaterEqual(z, centre + half_height - KNUCKLE_CLEARANCE - 1e-9)


if __name__ == "__main__":
    unittest.main()
