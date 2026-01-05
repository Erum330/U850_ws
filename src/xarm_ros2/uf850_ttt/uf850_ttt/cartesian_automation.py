#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

import tf2_ros
import tf_transformations


class TTTPieceDetector(Node):
    def __init__(self):
        super().__init__('ttt_piece_detector')

        # -------- Parameters (edit via launch / params)
        self.declare_parameter('image_topic', '/ttt_camera/image')
        self.declare_parameter('camera_info_topic', '/ttt_camera/camera_info')
        self.declare_parameter('base_frame', 'world')        # world / link_base / panda_link0 etc
        self.declare_parameter('camera_frame', 'ttt_camera') # MUST exist in TF
        self.declare_parameter('target_z', 0.775)            # plane height in base frame
        self.declare_parameter('min_area', 80)               # contour area threshold

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.target_z = float(self.get_parameter('target_z').value)
        self.min_area = int(self.get_parameter('min_area').value)

        # -------- Camera intrinsics (filled from CameraInfo)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # -------- ROS I/O
        self.bridge = CvBridge()

        self.sub_info = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_cb, 10
        )
        self.sub_img = self.create_subscription(
            Image, self.image_topic, self.image_cb, 10
        )

        self.pub_red = self.create_publisher(PointStamped, '/ttt/piece_red', 10)
        self.pub_blue = self.create_publisher(PointStamped, '/ttt/piece_blue', 10)

        # -------- TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"TTT Piece Detector started\n"
            f"  image_topic: {self.image_topic}\n"
            f"  camera_info_topic: {self.camera_info_topic}\n"
            f"  base_frame: {self.base_frame}\n"
            f"  camera_frame: {self.camera_frame}\n"
            f"  target_z: {self.target_z}"
        )

    def camera_info_cb(self, msg: CameraInfo):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def image_cb(self, msg: Image):
        if any(v is None for v in [self.fx, self.fy, self.cx, self.cy]):
            self.get_logger().warn("Waiting for CameraInfo (intrinsics)...")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ---- HSV masks (tuned for bright sim colors)
        # Red wraps around hue=0, so use two ranges
        red1 = cv2.inRange(hsv, np.array([0, 120, 80]), np.array([10, 255, 255]))
        red2 = cv2.inRange(hsv, np.array([170, 120, 80]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(red1, red2)

        blue_mask = cv2.inRange(hsv, np.array([95, 150, 80]), np.array([140, 255, 255]))

        # Clean masks
        red_mask = self.clean_mask(red_mask)
        blue_mask = self.clean_mask(blue_mask)

        # Detect + publish
        self.detect_and_publish(frame, red_mask, color_name="red", pub=self.pub_red)
        self.detect_and_publish(frame, blue_mask, color_name="blue", pub=self.pub_blue)

        # Debug view (optional)
        cv2.imshow("TTT Piece Detection", frame)
        cv2.waitKey(1)

    def clean_mask(self, mask):
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)
        return mask

    def detect_and_publish(self, frame, mask, color_name, pub):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return

        # Take largest blob
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area < self.min_area:
            return

        x, y, w, h = cv2.boundingRect(cnt)
        u = x + w // 2
        v = y + h // 2

        # draw
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.circle(frame, (u, v), 3, (0, 255, 255), -1)
        cv2.putText(frame, f"{color_name}", (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Pixel -> 3D on plane using TF
        pt_base = self.pixel_to_base(u, v)
        if pt_base is None:
            return

        # publish PointStamped
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        out.point.x = float(pt_base[0])
        out.point.y = float(pt_base[1])
        out.point.z = float(pt_base[2])
        pub.publish(out)

        self.get_logger().info(f"{color_name}: {out.point.x:.3f}, {out.point.y:.3f}, {out.point.z:.3f}")

    def pixel_to_base(self, u, v):
        """
        1) Make ray in camera frame
        2) Transform ray origin + direction to base frame via TF camera_frame -> base_frame
        3) Intersect with plane z=target_z in base frame
        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed ({self.base_frame} <- {self.camera_frame}): {e}")
            return None

        trans = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ])

        rot = [
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ]

        T = tf_transformations.quaternion_matrix(rot)
        R = T[:3, :3]         # rotation cam->base
        cam_origin = trans    # camera origin in base

        # Ray direction in camera coords (Z forward)
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        ray_cam = np.array([x, y, 1.0])
        ray_cam = ray_cam / np.linalg.norm(ray_cam)

        # Rotate into base
        ray_base = R @ ray_cam

        # Intersect with plane Z = target_z
        if abs(ray_base[2]) < 1e-6:
            return None  # ray parallel to plane

        s = (self.target_z - cam_origin[2]) / ray_base[2]
        if s <= 0:
            return None  # intersection behind camera

        pt = cam_origin + s * ray_base
        return pt


def main(args=None):
    rclpy.init(args=args)
    node = TTTPieceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
