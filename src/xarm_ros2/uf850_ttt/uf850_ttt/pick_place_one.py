#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose, Point, Quaternion

class OnePickPlace(Node):
    def __init__(self):
        super().__init__("pick_place_one")

        # ----------------------------
        # TARGET PIECE: Piece 4 (Furthest from robot)
        # ----------------------------
        self.target_piece_name = "o_piece_4"
        self.grid_center = Point(x=0.4, y=0.0, z=0.771) 

        # Service to teleport the piece
        self.set_pose_client = self.create_client(SetEntityPose, '/world/ttt_world/set_pose')

        # --- JOINT STATES (Indices 1-6) ---
        # The "PICK" joints you gave me are the final grasp position (down on the piece)
        self.pick_joints = [0.51394, -1.09971, -1.68162, 0.00007, -0.58175, 0.51380]
        
        # The "DROP" joints you gave me are the final release position (down on the grid)
        self.drop_joints = [-0.00310, -0.56648, -0.66113, 0.00007, -0.09444, -0.00313]

        # Calculate Approach/Lift (Modify joint2 and joint3 for height)
        # Higher Z usually means joint2 and joint3 are less negative/closer to zero
        self.approach_joints = [0.51394, -0.80000, -1.20000, 0.00007, -0.58175, 0.51380]
        self.lift_joints = [-0.00310, -0.30000, -0.40000, 0.00007, -0.09444, -0.00313]

        self.gripper_close = 0.51
        self.gripper_open  = 0.0

        # Action clients
        self.arm_ac = ActionClient(self, FollowJointTrajectory, "/uf850_traj_controller/follow_joint_trajectory")
        self.gripper_ac = ActionClient(self, FollowJointTrajectory, "/uf850_gripper_traj_controller/follow_joint_trajectory")

        self.get_logger().info(f"Initialized. Targeting {self.target_piece_name}")
        self.arm_ac.wait_for_server()
        self.gripper_ac.wait_for_server()

        self._worker = threading.Thread(target=self._run_sequence, daemon=True)
        self._worker.start()

    def teleport_piece(self, piece_name, position):
        if not self.set_pose_client.service_is_ready():
            return
        req = SetEntityPose.Request()
        req.entity.name = piece_name
        req.pose.position = position
        req.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.set_pose_client.call_async(req)

    def _run_sequence(self):
        # 1. Start with Open Gripper
        self.get_logger().info("Opening Gripper...")
        self.send_gripper(self.gripper_open, 1.0)
        
        # 2. Move to APPROACH point (above the piece)
        self.get_logger().info("Moving to Approach position...")
        self.send_arm(self.approach_joints, 4.0)
        
        # 3. Descend to PICK point
        self.get_logger().info("Descending to grasp...")
        self.send_arm(self.pick_joints, 2.0)
        
        # 4. CLOSE Gripper
        self.get_logger().info("Closing Gripper...")
        self.send_gripper(self.gripper_close, 2.0)
        time.sleep(1.5) # Let physics settle

        # 5. LIFT up (Avoid dragging piece on table)
        self.get_logger().info("Lifting piece...")
        self.send_arm(self.approach_joints, 2.0)

        # 6. Move to DROP location (Above grid center)
        self.get_logger().info("Moving to Grid Center...")
        self.send_arm(self.lift_joints, 4.0)

        # 7. Lower to Grid Surface
        self.get_logger().info("Lowering for release...")
        self.send_arm(self.drop_joints, 2.0)

        # 8. OPEN and TELEPORT
        self.get_logger().info(f"Releasing and Snapping {self.target_piece_name}...")
        self.send_gripper(self.gripper_open, 1.0)
        self.teleport_piece(self.target_piece_name, self.grid_center)

        # 9. Clear the area
        self.send_arm(self.lift_joints, 2.0)
        self.get_logger().info("✅ Task Complete!")

    def send_arm(self, joints, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in joints]
        pt.time_from_start.sec = int(duration)
        goal.trajectory.points = [pt]
        return self._send_and_wait(self.arm_ac, goal)

    def send_gripper(self, pos, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["drive_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(pos)]
        pt.time_from_start.sec = int(duration)
        goal.trajectory.points = [pt]
        return self._send_and_wait(self.gripper_ac, goal)

    def _send_and_wait(self, ac, goal):
        future = ac.send_goal_async(goal)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        res_future = future.result().get_result_async()
        while rclpy.ok() and not res_future.done():
            time.sleep(0.1)
        return True

def main():
    rclpy.init()
    node = OnePickPlace()
    rclpy.spin(node)

if __name__ == "__main__":
    main()