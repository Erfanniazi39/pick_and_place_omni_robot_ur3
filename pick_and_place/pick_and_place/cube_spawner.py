#!/usr/bin/env python3
import random
import subprocess
import time

import rclpy
from rclpy.node import Node


class CubeSpawner(Node):
    def __init__(self):
        super().__init__('cube_spawner')
        self.get_logger().info('Cube spawner node started')

        self.colors = {
            'red': {'r': 1.0, 'g': 0.0, 'b': 0.0},
            'green': {'r': 0.0, 'g': 1.0, 'b': 0.0},
            'blue': {'r': 0.0, 'g': 0.0, 'b': 1.0},
        }

        color_list = list(self.colors.keys())
        random.shuffle(color_list)
        self.get_logger().info(f'Cube arrangement: {color_list}')

        self.spawn_cubes(color_list)

    def spawn_cubes(self, colors):
        """Spawn one small cube on table and three big cubes at different positions."""
        try:
            cube_size = 0.03
            table_size_x = 0.60
            table_size_y = 0.60
            table_size_z = 0.20

            # Spawn the large table block first.
            base_x = 0.7
            base_y = 0.0
            table_center_z = table_size_z / 2.0
            cube_center_z = table_size_z + (cube_size / 2.0)

            table_sdf = self.create_table_sdf(
                base_x,
                base_y,
                table_center_z,
                table_size_x,
                table_size_y,
                table_size_z,
            )
            self.spawn_model_gz(table_sdf, 'table', 0)
            self.get_logger().info(
                f'Spawned table at ({base_x:.2f}, {base_y:.2f}, {table_center_z:.2f}) '
                f'size=({table_size_x:.2f}, {table_size_y:.2f}, {table_size_z:.2f})'
            )
            time.sleep(0.5)

            # Spawn ONE small cube on the table with random color
            small_cube_color = random.choice(list(self.colors.keys()))
            cube_sdf = self.create_cube_sdf(small_cube_color, base_x, base_y, cube_center_z, cube_size)
            self.spawn_model_gz(cube_sdf, f'small_{small_cube_color}', 0)
            self.get_logger().info(
                f'Spawned small {small_cube_color} cube at ({base_x:.2f}, {base_y:.2f}, {cube_center_z:.2f})'
            )
            time.sleep(0.5)

            # Spawn THREE big cubes at different positions (left, behind, right)
            # with non-repeating random colors.
            big_colors = list(self.colors.keys())
            random.shuffle(big_colors)
            big_cube_positions = [
              {'color': big_colors[0], 'x': 0.0, 'y': 1.7, 'label': 'left'},
              {'color': big_colors[1], 'x': -1.7, 'y': 0.0, 'label': 'behind'},
              {'color': big_colors[2], 'x': 0.0, 'y': -1.7, 'label': 'right'},
            ]
            
            big_cube_size = 0.4
            big_cube_z = big_cube_size / 4.0

            for pos_info in big_cube_positions:
                big_cube_sdf = self.create_big_cube_sdf(
                    color=pos_info['color'],
                    x=pos_info['x'],
                    y=pos_info['y'],
                    z=big_cube_z,
                    size_x=big_cube_size,
                    size_y=big_cube_size,
                    size_z=big_cube_size/2,
                )
                self.spawn_model_gz(big_cube_sdf, f'big_{pos_info["color"]}', 0)
                self.get_logger().info(
                    f'Spawned big {pos_info["color"]} cube at ({pos_info["x"]:.2f}, {pos_info["y"]:.2f}, {big_cube_z:.2f}) [{pos_info["label"]}]'
                )
                time.sleep(0.5)

        except Exception as e:
            self.get_logger().error(f'Error spawning cubes: {e}')

    def create_cube_sdf(self, color, x, y, z, cube_size):
        """Create SDF model for a small colored cube."""
        color_rgb = self.colors[color]

        sdf_string = f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.10\">
  <model name=\"cube_{color}_{int(time.time() * 1000) % 10000}\">
    <static>false</static>
    <link name=\"link\">
      <inertial>
        <mass>0.03</mass>
        <inertia>
          <ixx>0.0002</ixx>
          <iyy>0.0002</iyy>
          <izz>0.0002</izz>
        </inertia>
      </inertial>
      <collision name=\"collision\">
        <geometry>
          <box>
            <size>{cube_size} {cube_size} {cube_size}</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>5.0</mu>
              <mu2>5.0</mu2>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>10.0</kd>
              <max_vel>0.05</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name=\"visual\">
        <geometry>
          <box>
            <size>{cube_size} {cube_size} {cube_size}</size>
          </box>
        </geometry>
        <material>
          <ambient>{color_rgb['r']} {color_rgb['g']} {color_rgb['b']} 1.0</ambient>
          <diffuse>{color_rgb['r']} {color_rgb['g']} {color_rgb['b']} 1.0</diffuse>
          <specular>0.5 0.5 0.5 1.0</specular>
        </material>
      </visual>
    </link>
    <pose>{x} {y} {z} 0 0 0</pose>
  </model>
</sdf>"""
        return sdf_string

    def create_table_sdf(self, x, y, z, size_x, size_y, size_z):
        """Create SDF model for a large static table cube."""
        sdf_string = f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.10\">
  <model name=\"table_block_{int(time.time() * 1000) % 10000}\">
    <static>true</static>
    <link name=\"link\">
      <collision name=\"collision\">
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>3.0</mu>
              <mu2>3.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name=\"visual\">
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.55 0.40 0.25 1.0</ambient>
          <diffuse>0.55 0.40 0.25 1.0</diffuse>
          <specular>0.1 0.1 0.1 1.0</specular>
        </material>
      </visual>
    </link>
    <pose>{x} {y} {z} 0 0 0</pose>
  </model>
</sdf>"""
        return sdf_string

    def create_big_cube_sdf(self, color, x, y, z, size_x, size_y, size_z):
        """Create SDF model for a large static colored cube."""
        color_rgb = self.colors[color]
        sdf_string = f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.10\">
  <model name=\"big_cube_{color}_{int(time.time() * 1000) % 10000}\">
    <static>true</static>
    <link name=\"link\">
      <collision name=\"collision\">
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
      </collision>
      <visual name=\"visual\">
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
        <material>
          <ambient>{color_rgb['r']} {color_rgb['g']} {color_rgb['b']} 1.0</ambient>
          <diffuse>{color_rgb['r']} {color_rgb['g']} {color_rgb['b']} 1.0</diffuse>
          <specular>0.3 0.3 0.3 1.0</specular>
        </material>
      </visual>
    </link>
    <pose>{x} {y} {z} 0 0 0</pose>
  </model>
</sdf>"""
        return sdf_string

    def spawn_model_gz(self, sdf_string, model_tag, idx):
        """Spawn model in Gazebo using gz service."""
        try:
            sdf_escaped = sdf_string.replace('"', '\\"').replace('\n', ' ')
            cmd = [
                'gz', 'service', '-s', '/world/default/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', f'sdf: "{sdf_escaped}"',
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                self.get_logger().info(f'Successfully spawned {model_tag}_{idx}')
            else:
                self.get_logger().warn(f'Spawn returned code {result.returncode}')
                if result.stderr:
                    self.get_logger().warn(result.stderr.strip())

        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Spawn service timeout for {model_tag}_{idx}')
        except Exception as e:
            self.get_logger().error(f'Error calling spawn service: {e}')


def main(args=None):
    rclpy.init(args=args)
    cube_spawner = CubeSpawner()

    try:
        rclpy.spin(cube_spawner)
    except KeyboardInterrupt:
        pass
    finally:
        cube_spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
