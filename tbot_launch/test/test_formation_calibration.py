"""Dependency-free geometry and controller tests for the ROS script."""
import ast
import math
from pathlib import Path
import time
import unittest
from types import SimpleNamespace

SOURCE = Path(__file__).resolve().parents[1] / 'scripts' / 'formation_calibration.py'
module = ast.parse(SOURCE.read_text())
selected = [n for n in module.body if isinstance(n, ast.FunctionDef) and
            n.name in ('wrap_angle', 'formation_targets')]
context = {'math': math}
exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), 'exec'), context)


class GeometryTest(unittest.TestCase):
    def test_v_simulation(self):
        t = context['formation_targets']('v_formation', (2.5, 0.8, -math.pi), .25, .25, .15)
        self.assertEqual(len(t), 2)
        for result, desired in zip(t, ((2.75, .65), (2.75, .95))):
            self.assertAlmostEqual(result[0], desired[0], places=8)
            self.assertAlmostEqual(result[1], desired[1], places=8)

    def test_platoon_scenario_four(self):
        t = context['formation_targets']('platoon', (-.5, .8, -math.pi), .25, .25, .15)
        self.assertEqual(len(t), 3)
        for i, state in enumerate(t, start=1):
            self.assertAlmostEqual(state[0], -.5 + .25*i)
            self.assertAlmostEqual(state[1], .8)
            self.assertAlmostEqual(abs(state[2]), math.pi)

    def test_rotation(self):
        t = context['formation_targets']('v_formation', (0, 0, math.pi/2), .25, .25, .15)
        self.assertAlmostEqual(t[0][0], -.15)
        self.assertAlmostEqual(t[0][1], -.25)

    def test_invalid_mode(self):
        with self.assertRaises(ValueError):
            context['formation_targets']('invalid', (0, 0, 0), .25, .25, .15)


if __name__ == '__main__':
    unittest.main()

class DecentralizedSourceTest(unittest.TestCase):
    def test_single_robot_instance(self):
        classes = {n.name: n for n in module.body if isinstance(n, ast.ClassDef)}
        cal = classes['FormationCalibration']
        init = next(n for n in cal.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        src = ast.get_source_segment(SOURCE.read_text(), init)
        self.assertIn('targets[self.follower_id - 1]', src)
        self.assertIn("get('pose_topic', '')", src)
        self.assertIn("get('cmd_topic', '')", src)
        self.assertNotIn('self.robots', src)

    def test_launch_one_node_and_correct_mocap_topics(self):
        import xml.etree.ElementTree as ET
        launch = ET.parse(SOURCE.parents[1] / 'launch' / 'formation_calibration.launch')
        root = launch.getroot()
        self.assertEqual(len(root.findall('node')), 1)
        args = {x.get('name'): x.get('default') for x in root.findall('arg')}
        self.assertEqual(args['position_tolerance'], '0.02')
        self.assertEqual(args['f1_mocap'], '/vrpn_client_node/bot162/pose')
        self.assertEqual(args['f2_mocap'], '/vrpn_client_node/bot149/pose')
        self.assertEqual(args['f3_mocap'], '/vrpn_client_node/bot165/pose')
        self.assertEqual(args['follower_id'], '0')
