# obb_node.py (수정본)
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, Bool
import tf_transformations
from ultralytics import YOLO
import math


class OrientationEstimator(Node):
    def __init__(self):
        super().__init__('orientation_estimator')

        self.bridge = CvBridge()

        # ---- Subscriber ----
        self.rgb_sub = self.create_subscription(Image, '/rgb', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth', self.depth_callback, 10)
        self.cam_sub = self.create_subscription(CameraInfo, '/camera_info', self.cam_callback, 10)

        # ---- Publisher ----
        self.pose_pub = self.create_publisher(PoseStamped, "/object_pose", 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, "/target_pose", 10)  # ★ 추가
        self.marker_pub = self.create_publisher(Marker, "/object_marker", 10)
        self.matrix_pub = self.create_publisher(Float32MultiArray, "/rotation_matrix", 10)
        self.move_pub = self.create_publisher(Bool, '/moverobot', 10)

        # ---- Data ----
        self.rgb = None
        self.depth = None
        self.cam_info = None

        self.minangle = 10.0
        self.maxangle = 70.0

        self.model = YOLO("/home/rokey/ros2_ws/best.pt")

        # ---- 안정화(디바운스) 파라미터 ----
        self.defect_need = 5   # 불량 판정 연속 프레임 수
        self.ok_need = 5       # 정상 판정 연속 프레임 수

        self.defect_count = 0
        self.ok_count = 0
        self.current_defect = False  # 현재 moverobot 상태

        self.get_logger().info("OrientationEstimator initialized (debounced moverobot + latched /target_pose)")

    def cam_callback(self, msg):
        self.cam_info = msg

    def depth_callback(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        if self.depth is None or self.cam_info is None:
            return

        obb = self.run_yolo_obb(self.rgb)

        if obb is None:
            # 감지 안 되면 카운트 리셋 + (원하면 일정 프레임 후 정상으로 복귀)
            self.defect_count = 0
            self.ok_count += 1
            self._maybe_publish_state(defect_now=False, pose_xyzrpy=None, whd=None)
            self.delete_marker()
            return

        xc, yc, w, h, theta = obb

        depth_value = self.get_depth(xc, yc)
        if depth_value is None:
            self.defect_count = 0
            self.ok_count += 1
            self._maybe_publish_state(defect_now=False, pose_xyzrpy=None, whd=None)
            self.delete_marker()
            return

        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]
        cy = self.cam_info.k[5]

        # 2D OBB -> 3D 포인트들
        vertices_2d = self.get_obb_vertices(xc, yc, w, h, theta)
        vertices_3d = []
        for px, py in vertices_2d:
            d = self.get_depth(px, py)
            if d is None:
                continue
            Xv = (px - cx) * d / fx
            Yv = (py - cy) * d / fy
            Zv = d
            vertices_3d.append([Xv, Yv, Zv])

        if len(vertices_3d) < 3:
            self.defect_count = 0
            self.ok_count += 1
            self._maybe_publish_state(defect_now=False, pose_xyzrpy=None, whd=None)
            self.delete_marker()
            return

        vertices_3d = np.array(vertices_3d)

        # plane normal -> roll/pitch
        normal = self.fit_plane_normal(vertices_3d)
        pitch, roll = self.compute_pitch_roll_from_normal(normal)

        # yaw 보정
        raw_yaw = theta
        offset_radian = math.pi / 2
        corrected_yaw = raw_yaw - offset_radian
        corrected_yaw = self._normalize_radian(corrected_yaw)
        yaw = corrected_yaw * -1

        # 중심점 3D
        X = (xc - cx) * depth_value / fx
        Y = (yc - cy) * depth_value / fy
        Z = depth_value

        # rotation matrix publish
        R = self.compute_rotation_matrix(roll, pitch, yaw)
        mat_msg = Float32MultiArray()
        mat_msg.data = R.flatten().astype(np.float32).tolist()
        self.matrix_pub.publish(mat_msg)

        # object_pose는 계속 publish (기존 유지)
        self.publish_pose(X, Y, Z, roll, pitch, yaw)
        self.publish_marker(X, Y, Z, roll, pitch, yaw, w, h, depth_value)

        # ---- 불량 판정(기존 로직 기반) ----
        yaw_deg = math.degrees(yaw)
        ok_now = (self.minangle < yaw_deg < self.maxangle)
        defect_now = not ok_now

        if defect_now:
            self.defect_count += 1
            self.ok_count = 0
        else:
            self.ok_count += 1
            self.defect_count = 0

        # 디바운스 반영 + 상태 publish (+ 불량 전환 시 target_pose 1회 발행)
        self._maybe_publish_state(
            defect_now=defect_now,
            pose_xyzrpy=(X, Y, Z, roll, pitch, yaw),
            whd=(w, h, depth_value)
        )

    def _maybe_publish_state(self, defect_now: bool, pose_xyzrpy, whd):
        """
        defect_now: 현재 프레임의 판정
        - defect_count/ok_count가 임계치 넘을 때만 moverobot 상태 변경
        - False->True로 바뀌는 순간 target_pose를 1회 발행(= latch)
        """
        # 상태 전환 조건
        want_defect = self.current_defect

        if defect_now and self.defect_count >= self.defect_need:
            want_defect = True
        if (not defect_now) and self.ok_count >= self.ok_need:
            want_defect = False

        # 상태가 그대로면 moverobot만 계속 쏘지 말고(필요시 여기서 계속 publish도 가능)
        if want_defect == self.current_defect:
            # 그래도 moverobot을 계속 publish하고 싶으면 아래 주석 해제
            # msg = Bool(); msg.data = self.current_defect; self.move_pub.publish(msg)
            return

        # 상태가 바뀌는 순간
        self.current_defect = want_defect
        move_msg = Bool()
        move_msg.data = self.current_defect
        self.move_pub.publish(move_msg)

        if self.current_defect:
            # False -> True로 바뀐 순간: target_pose latch 1회 publish
            if pose_xyzrpy is not None:
                X, Y, Z, roll, pitch, yaw = pose_xyzrpy
                self.publish_target_pose(X, Y, Z, roll, pitch, yaw)
                self.get_logger().info("DEFECT confirmed -> /moverobot=True, latched /target_pose published")
        else:
            self.get_logger().info("OK confirmed -> /moverobot=False")

    def publish_target_pose(self, x, y, z, roll, pitch, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"  # 기존 pose와 동일 프레임
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = \
            float(q[0]), float(q[1]), float(q[2]), float(q[3])
        self.target_pose_pub.publish(msg)

    # ---- 이하 함수들은 기존 그대로 사용(생략 없이 두면 됨) ----
    def _normalize_radian(self, radian):
        radian = radian % (2 * math.pi)
        if radian > math.pi:
            radian -= (2 * math.pi)
        return radian

    def run_yolo_obb(self, img):
        results = self.model(img, verbose=False)
        if len(results) == 0 or results[0].obb is None or len(results[0].obb) == 0:
            return None
        data = results[0].obb.data.cpu().numpy()[0]
        xc, yc, w, h, theta, conf, cls = data
        return float(xc), float(yc), float(w), float(h), float(theta)

    def get_obb_vertices(self, xc, yc, w, h, theta):
        hw = w / 2
        hh = h / 2
        corners = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]])
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        R = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
        rot = corners @ R.T
        rot[:, 0] += xc
        rot[:, 1] += yc
        return rot

    def get_depth(self, x, y):
        h, w = self.depth.shape
        x = int(x)
        y = int(y)
        if x < 2 or x >= w - 2 or y < 2 or y >= h - 2:
            return None
        patch = self.depth[y-2:y+3, x-2:x+3]
        d = np.median(patch)
        if d == 0 or np.isnan(d):
            return None
        if d > 10:
            d = d / 1000.0
        return float(d)

    def fit_plane_normal(self, pts):
        p0, p1, p2 = pts[0], pts[1], pts[2]
        v1 = p1 - p0
        v2 = p2 - p0
        n = np.cross(v1, v2)
        norm = np.linalg.norm(n)
        return n / norm if norm > 0 else np.array([0, 0, 1])

    def compute_pitch_roll_from_normal(self, n):
        nx, ny, nz = n
        pitch = np.arctan2(-nx, np.sqrt(ny * ny + nz * nz))
        roll = np.arctan2(ny, nz)
        return float(pitch), float(roll)

    def compute_rotation_matrix(self, roll, pitch, yaw):
        cx = np.cos(roll); sx = np.sin(roll)
        cy = np.cos(pitch); sy = np.sin(pitch)
        cz = np.cos(yaw); sz = np.sin(yaw)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def publish_pose(self, x, y, z, roll, pitch, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = \
            float(q[0]), float(q[1]), float(q[2]), float(q[3])
        self.pose_pub.publish(msg)

    def publish_marker(self, x, y, z, roll, pitch, yaw, w, h, depth):
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obb"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = \
            float(q[0]), float(q[1]), float(q[2]), float(q[3])

        marker.scale.x = max(0.01, float(w) / 1000.0)
        marker.scale.y = max(0.01, float(h) / 1000.0)
        marker.scale.z = max(0.05, float(depth) / 2.0)

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime.sec = 0

        self.marker_pub.publish(marker)

    def delete_marker(self):
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obb"
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = OrientationEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
