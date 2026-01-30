# move_joint.py (수정본)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import numpy as np
import math


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TestROS2Bridge(Node):
    def __init__(self):
        super().__init__("test_ros2bridge")

        self.pub = self.create_publisher(JointState, "/joint_command", 10)

        self.sub_move = self.create_subscription(Bool, "/moverobot", self.move_cb, 10)
        self.sub_target = self.create_subscription(PoseStamped, "/target_pose", self.target_cb, 10)

        self.msg = JointState()
        self.msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.msg.position = [0.0] * 6

        self.home = np.zeros(6, dtype=np.float64)

        # “그럴듯한” 데모용 기본 포즈들(라디안) - 필요하면 여기만 조금씩 튜닝
        self.approach_base = np.array([1.45, -0.55, 1.10, -1.15, 1.50, 0.0], dtype=np.float64)
        self.pick_base     = np.array([1.55, -0.70, 1.30, -1.25, 1.60, 0.0], dtype=np.float64)
        self.retreat_base  = np.array([1.40, -0.45, 1.00, -1.05, 1.45, 0.0], dtype=np.float64)

        # /target_pose latch 저장
        self.latched_target = None  # PoseStamped

        # 시퀀스 제어
        self.sequence_active = False
        self.sequence_step = 0
        self.step_timer = None

        # 현재 publish할 목표
        self.target_pose = self.home.copy()

        # 20Hz publish
        self.timer = self.create_timer(0.05, self.pub_cb)

        self.get_logger().info("move_joint ready: /moverobot + /target_pose -> joint sequence")

    def target_cb(self, msg: PoseStamped):
        self.latched_target = msg

    def move_cb(self, msg: Bool):
        defect = bool(msg.data)

        if defect:
            # 불량 확정: 시퀀스 시작(중복 시작 방지)
            if not self.sequence_active:
                print("target object detected!\nstarting to make an approach")
                self.start_sequence()
        else:
            # 정상 확정: 시퀀스 중지 + HOME
            self.stop_sequence()
            self.target_pose = self.home.copy()

    def pub_cb(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.position = self.target_pose.tolist()
        self.pub.publish(self.msg)

    def apply_target_hint(self, base_pose: np.ndarray) -> np.ndarray:
        """
        진짜 IK 대신, /target_pose의 (x,y)를 이용해 joint_1(베이스 회전)만 “살짝” 보정.
        발표에서 '물체 위치에 따라 반응'이 보이게 하는 용도.
        """
        pose = base_pose.copy()

        if self.latched_target is None:
            return pose

        x = float(self.latched_target.pose.position.x)
        y = float(self.latched_target.pose.position.y)

        # 카메라 프레임 기준이더라도 '연출'로는 충분: atan2로 방향만 뽑아서 약하게 적용
        yaw = math.atan2(y, x)
        yaw = clamp(yaw, -math.radians(90), math.radians(90))

        gain = 0.6  # 0~1.0 사이로 조절(크면 과하게 회전)
        pose[0] = clamp(gain * yaw, -math.pi, math.pi)

        return pose

    def start_sequence(self):
        self.stop_sequence()
        self.sequence_active = True
        self.sequence_step = 0
        self.advance_step()

    def advance_step(self):
        if not self.sequence_active:
            return

        # 단계별 포즈
        if self.sequence_step == 0:
            self.target_pose = self.apply_target_hint(self.approach_base)
            self._schedule_next(1.2)
        elif self.sequence_step == 1:
            self.target_pose = self.apply_target_hint(self.pick_base)
            self._schedule_next(1.2)
        elif self.sequence_step == 2:
            self.target_pose = self.apply_target_hint(self.retreat_base)
            # 마지막은 유지 (또는 일정 시간 뒤 HOME 복귀하고 싶으면 여기서 스케줄 추가)
            self.sequence_active = False
            self.step_timer = None
            return

        self.sequence_step += 1

    def _schedule_next(self, sec: float):
        # one-shot 타이머 형태로 쓰기 위해 매번 새로 만들고, 콜백에서 cancel
        self.step_timer = self.create_timer(sec, self._step_timer_cb)

    def _step_timer_cb(self):
        if self.step_timer is not None:
            self.step_timer.cancel()
            self.step_timer = None
        self.advance_step()

    def stop_sequence(self):
        self.sequence_active = False
        self.sequence_step = 0
        if self.step_timer is not None:
            self.step_timer.cancel()
            self.step_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = TestROS2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
