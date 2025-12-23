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

        # Robot O's on Left (y=0.35), Human X's on Right (y=-0.35)
        self.spawn_group("o_piece", 0.35, "0 0 1 1", True)   
        time.sleep(1.0)
        self.spawn_group("x_piece", -0.35, "1 0 0 1", False) 

    def generate_sdf(self, name, is_o, color):
        geometry = '<cylinder><radius>0.02</radius><length>0.04</length></cylinder>' if is_o else \
                   '<box><size>0.04 0.04 0.01</size></box>'
        return f'''<?xml version="1.0" ?><sdf version="1.9"><model name="{name}"><link name="link">
            <collision name="c"><geometry>{geometry}</geometry></collision>
            <visual name="v"><geometry>{geometry}</geometry><material><ambient>{color}</ambient></material></visual>
            </link></model></sdf>'''

    def spawn_group(self, base_name, y_pos, color, is_o):
        for i in range(5):
            name = f"{base_name}_{i}"
            x_pos = 0.2 + (i * 0.1) # Spaced 10cm apart for gripper clearance
            req = SpawnEntity.Request()
            req.entity_factory = EntityFactory()
            req.entity_factory.name = name
            req.entity_factory.sdf = self.generate_sdf(name, is_o, color)
            req.entity_factory.pose = Pose()
            req.entity_factory.pose.position = Point(x=x_pos, y=y_pos, z=0.8)
            if not is_o: # Orient X's to stand vertically
                req.entity_factory.pose.orientation = Quaternion(x=0.7071068, y=0.0, z=0.0, w=0.7071068)
            self.spawn_client.call_async(req)

def main():
    rclpy.init()
    node = TTTPieceSpawner()
    time.sleep(2.0) # Ensure async calls complete
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()