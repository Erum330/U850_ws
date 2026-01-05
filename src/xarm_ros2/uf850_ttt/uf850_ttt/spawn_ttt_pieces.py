#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.msg import EntityFactory
from geometry_msgs.msg import Pose, Point, Quaternion


class TTTPieceSpawner(Node):
    def __init__(self):
        super().__init__('ttt_spawner')

        self.spawn_client = self.create_client(SpawnEntity, '/world/ttt_world/create')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /world/ttt_world/create ...')

        # Table top is z=0.75
        # Piece height ~0.04 => center ~0.77
        self.spawn_group("o_piece", y_pos=0.35, color="0 0 1 1", is_o=True)   # blue O
        self.spawn_group("x_piece", y_pos=-0.35, color="1 0 0 1", is_o=False) # red X

        self.get_logger().info("Done spawning pieces.")

    def generate_sdf(self, name: str, is_o: bool, color: str) -> str:
        geometry = (
            '<cylinder><radius>0.02</radius><length>0.04</length></cylinder>'
            if is_o else
            '<box><size>0.04 0.04 0.04</size></box>'
        )

        # Good grasp behavior usually comes from:
        # - moderate friction (2~5)
        # - not-crazy kp/kd (huge kp can cause explosions)
        # - reasonable mass/inertia
        return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <gravity>true</gravity>
      <self_collide>false</self_collide>

      <inertial>
        <mass>0.06</mass>
        <inertia>
          <ixx>0.00003</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00003</iyy><iyz>0</iyz><izz>0.00003</izz>
        </inertia>
      </inertial>

      <collision name="c">
        <geometry>{geometry}</geometry>
        <surface>
          <friction>
            <ode>
              <mu>3.0</mu>
              <mu2>3.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>2e5</kp>
              <kd>10.0</kd>
              <min_depth>0.001</min_depth>
              <max_vel>0.1</max_vel>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="v">
        <geometry>{geometry}</geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

    def spawn_one(self, name: str, x: float, y: float, z: float, is_o: bool, color: str):
        req = SpawnEntity.Request()
        req.entity_factory = EntityFactory()
        req.entity_factory.name = name
        req.entity_factory.sdf = self.generate_sdf(name, is_o, color)

        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # keep upright
        req.entity_factory.pose = pose

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            self.get_logger().error(f"Spawn failed (no response): {name}")
            return False

        res = future.result()
        if hasattr(res, "success") and not res.success:
            self.get_logger().error(f"Spawn failed: {name} | msg={getattr(res, 'status_message', '')}")
            return False

        self.get_logger().info(f"Spawned: {name}")
        return True

    def spawn_group(self, base_name: str, y_pos: float, color: str, is_o: bool):
        for i in range(5):
            name = f"{base_name}_{i}"
            x_pos = 0.2 + (i * 0.1)
            # center at ~0.77, a tiny bit above to settle
            self.spawn_one(name, x=x_pos, y=y_pos, z=0.775, is_o=is_o, color=color)


def main():
    rclpy.init()
    node = TTTPieceSpawner()
    # give Gazebo a moment, then exit
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
