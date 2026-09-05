"""Offline checks: source assets, map pairing, launch defaults and TF ownership."""
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml
from PIL import Image
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / 'src/robot_rl_description'
NAVIGATION = ROOT / 'src/robot_rl_navigation2'


class WarehouseIntegrationTest(unittest.TestCase):
    def test_all_model_uris_resolve_locally(self):
        models = DESCRIPTION / 'models/warehouse'
        files = [DESCRIPTION / 'world/warehouse/small_warehouse.world',
                 *models.rglob('*.sdf')]
        for path in files:
            for uri in ET.parse(path).iter('uri'):
                value = (uri.text or '').strip()
                if value.startswith('model://'):
                    self.assertTrue((models / value[8:]).exists(), (path, value))

    def test_map_and_patrol_points(self):
        metadata = yaml.safe_load((NAVIGATION / 'maps/warehouse_map.yaml').read_text())
        image = Image.open(NAVIGATION / 'maps' / metadata['image'])
        self.assertEqual(image.size, (318, 439))
        self.assertEqual(metadata['resolution'], 0.05)
        self.assertEqual(metadata['origin'], [-7.56, -10.8, 0])
        config = yaml.safe_load((ROOT / 'src/autopatrol_robot/config/warehouse_patrol.yaml').read_text())
        values = config['patrol_node']['ros__parameters']['target_points']
        self.assertEqual(len(values) % 3, 0)
        for x, y, _ in zip(values[::3], values[1::3], values[2::3]):
            col = int((x - metadata['origin'][0]) / metadata['resolution'])
            row = image.height - 1 - int((y - metadata['origin'][1]) / metadata['resolution'])
            self.assertTrue(0 <= col < image.width and 0 <= row < image.height)
            self.assertGreater(image.getpixel((col, row)), 250)

    def test_scene_pairing_and_tf_authority(self):
        spec = importlib.util.spec_from_file_location('navigation_launch', NAVIGATION / 'launch/navigation2.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for scene, slam in [('warehouse', 'false'), ('room', 'false'), ('room', 'true')]:
            context = LaunchContext()
            context.launch_configurations.update(scene=scene, slam=slam)
            description = module.generate_launch_description()
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            self.assertEqual(Path(context.launch_configurations['map']).name,
                             'warehouse_map.yaml' if scene == 'warehouse' else 'room.yaml')
            self.assertEqual(Path(context.launch_configurations['world']).name,
                             'small_warehouse.world' if scene == 'warehouse' else 'custom_room.world')
            self.assertEqual(float(context.launch_configurations['spawn_z']),
                             0.08 if scene == 'warehouse' else 0.002)
            gazebo = next(a for a in description.entities if isinstance(a, IncludeLaunchDescription))
            value = dict(gazebo.launch_arguments)['publish_map_tf'].perform(context)
            self.assertEqual(value, str(scene == 'room' and slam == 'false'))


if __name__ == '__main__':
    unittest.main()
