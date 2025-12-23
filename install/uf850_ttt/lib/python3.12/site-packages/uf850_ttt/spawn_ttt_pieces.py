#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.msg import EntityFactory
from geometry_msgs.msg import Pose, Point, Quaternion
import time

class TTTPieceSpawner(Node):
    def __init__(self):
        super().__init__('ttt_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/world/ttt_world/create')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        # Increased Y distance for better robot arm clearance
        self.spawn_group("o_piece", 0.5, "0 0 1 1", True)   
        time.sleep(1.0)
        self.spawn_group("x_piece", -0.5, "1 0 0 1", False) 

    def generate_sdf(self, name, is_o, color):
        if is_o:
            # Blue O - Standing Cylinder
            geometry = '<cylinder><radius>0.02</radius><length>0.04</length></cylinder>'
        else:
            # Red X - We use a Box for the collision to make the vacuum seal easy,
            # but you can see it as an X by its shape.
            geometry = '<box><size>0.04 0.04 0.01</size></box>'
            
        return f'''<?xml version="1.0" ?>
        <sdf version="1.9">
          <model name="{name}">
            <static>false</static>
            <link name="link">
              <inertial>
                <mass>0.05</mass>
                <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia>
              </inertial>
              <collision name="collision">
                <geometry>{geometry}</geometry>
              </collision>
              <visual name="visual">
                <geometry>{geometry}</geometry>
                <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
              </visual>
            </link>
          </model>
        </sdf>'''

    def spawn_group(self, base_name, y_pos, color, is_o):
        for i in range(5):
            name = f"{base_name}_{i}"
            x_pos = 0.4 + (i * 0.12) # Spread out 12cm apart
            
            req = SpawnEntity.Request()
            req.entity_factory = EntityFactory()
            req.entity_factory.name = name
            req.entity_factory.sdf = self.generate_sdf(name, is_o, color)
            req.entity_factory.pose = Pose()
            req.entity_factory.pose.position = Point(x=x_pos, y=y_pos, z=0.8)
            
            # Rotation: Make them stand on their edge
            if not is_o:
                # Rotate 90 degrees on X-axis so the "flat" side faces the robot
                req.entity_factory.pose.orientation = Quaternion(x=0.7071068, y=0.0, z=0.0, w=0.7071068)
            else:
                req.entity_factory.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            self.get_logger().info(f'Spawning {name}...')
            future = self.spawn_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    node = TTTPieceSpawner()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()