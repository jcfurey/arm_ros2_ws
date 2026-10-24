"""Validate SDF pose composition and reject geometry the planner cannot mirror."""
import math
from pathlib import Path
import tempfile
import unittest

from arm_bringup.scene import compose, inverse, load_world, quaternion


class SceneTests(unittest.TestCase):
    def parse(self, body):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "world.sdf"
            path.write_text(f'<sdf version="1.10"><world name="test">{body}</world></sdf>')
            return load_world(path)

    def test_rotated_model_link_and_collision_frames(self):
        scene = self.parse(f'''
          <frame name="robot_mount" attached_to="world"><pose relative_to="world">0 0 0.75 0 0 0</pose></frame>
          <model name="fixture"><static>true</static><pose>1 2 3 0 0 {math.pi/2}</pose>
            <link name="body"><pose>1 0 0 0 0 0</pose>
              <collision name="solid"><pose>0 2 0 0 0 {math.pi/2}</pose>
                <geometry><box><size>1 2 3</size></box></geometry>
              </collision>
            </link>
          </model>''')
        obj = scene.objects[0]
        self.assertEqual(obj.id, "sdf/fixture/body/solid")
        for actual, expected in zip(obj.pose, (-1, 3, 3, 0, 0, 1, 0)):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(scene.mount[2], 0.75)

    def test_unsupported_geometry_cannot_disappear_silently(self):
        for geometry in ['<mesh><uri>model://unknown</uri></mesh>', '<heightmap/>']:
            with self.subTest(geometry=geometry), self.assertRaisesRegex(ValueError, "Unsupported"):
                self.parse(f'<model name="obstacle"><static>true</static><link name="body"><collision name="solid"><geometry>{geometry}</geometry></collision></link></model>')

    def test_free_single_link_model_is_a_movable_part(self):
        scene = self.parse('''
          <model name="part"><pose>1 0 0.5 0 0 0</pose>
            <link name="body"><pose>0 0 0.1 0 0 0</pose>
              <collision name="solid"><geometry><box><size>0.04 0.04 0.04</size></box></geometry></collision>
            </link>
          </model>''')
        self.assertEqual(scene.objects, ())
        part = scene.movable[0]
        self.assertEqual((part.id, part.model), ("sdf/part/body/solid", "part"))
        for actual, expected in zip(part.pose, (1, 0, 0.6, 0, 0, 0, 1)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(part.model_offset, (0, 0, 0.1, 0, 0, 0, 1)):
            self.assertAlmostEqual(actual, expected)

    def test_inverse_undoes_compose(self):
        pose = (0.3, -0.2, 1.1) + quaternion((0.4, -0.7, 2.0))
        other = (-1.0, 0.5, 0.25) + quaternion((-0.3, 0.2, 0.9))
        for actual, expected in zip(compose(compose(pose, other), inverse(other)), pose):
            self.assertAlmostEqual(actual, expected)

    def test_articulated_moving_models_cannot_be_synchronized(self):
        with self.assertRaisesRegex(ValueError, "Moving model"):
            self.parse('<model name="moving"><static>false</static><link name="a"/><link name="b"/></model>')

    def test_external_frame_references_cannot_be_misplaced(self):
        with self.assertRaisesRegex(ValueError, "relative to their parent"):
            self.parse('<model name="obstacle"><static>true</static><pose relative_to="unknown">0 0 0 0 0 0</pose></model>')


if __name__ == "__main__":
    unittest.main()
