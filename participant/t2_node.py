from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import LowCmd, LowState

from .t2_collision import Go1Collision
from .t2_safety import (
    HOLD_AFTER_NS, JOINT_NAMES, Motor, command_pose, finite_motor, hold_command,
    limited_command, safe_motion, valid_pose,
)


MOTOR_COUNT = len(JOINT_NAMES)
PUBLISH_PERIOD_S = 0.01


@dataclass(frozen=True)
class Request:
    received_ns: int
    motors: tuple


@dataclass(frozen=True)
class Check:
    request: Request
    start: tuple
    epoch: int
    result: Future


def to_lowcmd(motors):
    # создаём новое сообщение: неиспользуемые 8 приводов остаются нулевыми.
    message = LowCmd()
    for output, motor in zip(message.motor_cmd, motors):
        output.mode = motor.mode
        output.q, output.dq = motor.q, motor.dq
        output.kp, output.kd, output.tau = motor.kp, motor.kd, motor.tau
    return message


class SafetyNode(Node):
    def __init__(self, collision):
        super().__init__("go1_t2_safety", parameter_overrides=[
            Parameter("use_sim_time", value=True),
        ])
        self._collision = collision
        self._initial_pose = None
        self._goal = None
        self._hold = None
        self._active = None
        self._accepted_ns = None
        self._prev_tick_ns = None
        self._epoch = 0
        self._pending = None
        self._checking = None

        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LowCmd, "/lowcmd_bare", self._on_command, latest)
        self._state_sub = self.create_subscription(LowState, "/lowstate", self._on_state, latest)
        self._publish = self.create_publisher(LowCmd, "/lowcmd", QoSProfile(depth=1))
        self._worker = ThreadPoolExecutor(max_workers=1)
        # при use_sim_time ROS берёт время таймера из /clock.
        self.create_timer(PUBLISH_PERIOD_S, self._tick)

    def _on_state(self, message):
        if self._goal is None and self._initial_pose is None:
            self._initial_pose = tuple(m.q for m in message.motor_state[:MOTOR_COUNT])

    def _on_command(self, message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return
        motors = tuple(Motor(m.mode, m.q, m.dq, m.kp, m.kd, m.tau) for m in message.motor_cmd)
        # NaN или Inf портят всё сообщение, даже если пришли в неиспользуемом элементе.
        if not all(finite_motor(motor) for motor in motors):
            return
        self._pending = Request(now_ns, motors[:MOTOR_COUNT])

    def _start_hold(self):
        pose = self._initial_pose
        if pose is not None and valid_pose(pose) and self._collision.is_safe(pose):
            self._goal = command_pose(pose)
        else:
            self._goal = self._collision.home
        self._hold = to_lowcmd(hold_command(self._goal))
        self.destroy_subscription(self._state_sub)
        self._state_sub = None

    def _fresh(self, received_ns, now_ns):
        return received_ns is not None and 0 <= now_ns - received_ns < HOLD_AFTER_NS

    def _check(self, request, start):
        candidate = limited_command(request.motors, start)
        end = tuple(motor.q for motor in candidate)
        # если цель не меняется, она уже прошла проверку при предыдущей публикации.
        if start == end or safe_motion(start, end, self._collision.is_safe):
            return candidate
        return None

    def _finish_check(self, now_ns):
        check = self._checking
        if check is None or not check.result.done():
            return
        self._checking = None
        if (check.epoch != self._epoch or check.start != self._goal
                or not self._fresh(check.request.received_ns, now_ns)):
            return
        try:
            candidate = check.result.result()
        except Exception as error:
            self.get_logger().error(f"T2 ошибка проверки коллизий: {error}", throttle_duration_sec=1.0)
            candidate = None
        if candidate is None:
            # запрещённый переход не меняет цель и сразу включает удержание.
            self._active = None
            self._accepted_ns = None
            return
        self._goal = tuple(motor.q for motor in candidate)
        self._hold = to_lowcmd(hold_command(self._goal))
        self._active = to_lowcmd(candidate)
        self._accepted_ns = check.request.received_ns

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self._prev_tick_ns is not None and now_ns < self._prev_tick_ns:
            # при перезапуске часов старые результаты больше применять нельзя.
            self._epoch += 1
            self._pending = None
            self._accepted_ns = None
            self._active = None
        self._prev_tick_ns = now_ns
        if now_ns <= 0:
            return
        if self._goal is None:
            self._start_hold()

        self._finish_check(now_ns)
        if not self._fresh(self._accepted_ns, now_ns):
            self._active = None
        self._publish.publish(self._active if self._active is not None else self._hold)

        # долгая проверка перехода не прерывает выходной поток. в очереди только свежая цель.
        if self._checking is None and self._pending is not None:
            request, self._pending = self._pending, None
            if self._fresh(request.received_ns, now_ns):
                future = self._worker.submit(self._check, request, self._goal)
                self._checking = Check(request, self._goal, self._epoch, future)

    def destroy_node(self):
        self._worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = SafetyNode(Go1Collision())
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
