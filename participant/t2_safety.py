from dataclasses import dataclass
import math
from struct import Struct


JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint"
    for leg in ("FR", "FL", "RR", "RL")
    for joint in ("hip", "thigh", "calf")
)
Q_MIN = (-0.863, -0.686, -2.818) * 4
Q_MAX = (0.863, 4.501, -0.888) * 4
MAX_STEP = math.radians(5)
COLLISION_STEP = math.radians(0.1)
HOLD_AFTER_NS = 450_000_000
SERVO_MODE = 10
FLOAT32 = Struct("<f")
Q_ROUNDING_MARGIN = 1e-6  # Больше шага float32 во всём диапазоне углов Go1.


@dataclass(frozen=True)
class Motor:
    mode: int
    q: float
    dq: float
    kp: float
    kd: float
    tau: float


def finite_motor(motor):
    return all(math.isfinite(value) for value in (
        motor.q, motor.dq, motor.kp, motor.kd, motor.tau,
    ))


def valid_pose(pose):
    return len(pose) == len(JOINT_NAMES) and all(
        lower <= angle <= upper for angle, lower, upper in zip(pose, Q_MIN, Q_MAX)
    )


def clamp(value, lower, upper):
    return min(max(value, lower), upper)


def float32(value):
    return FLOAT32.unpack(FLOAT32.pack(value))[0]


def command_pose(pose):
    # ROS передаёт q как float32, поэтому округляем и проверяем границы.
    output = []
    for angle, lower, upper in zip(pose, Q_MIN, Q_MAX):
        rounded = float32(clamp(angle, lower, upper))
        if rounded < lower:
            rounded = float32(lower + Q_ROUNDING_MARGIN)
        elif rounded > upper:
            rounded = float32(upper - Q_ROUNDING_MARGIN)
        output.append(rounded)
    return tuple(output)


def max_step(start, end):
    return max(abs(b - a) for a, b in zip(start, end))


def limited_command(motors, previous_q):
    target = command_pose(tuple(motor.q for motor in motors))
    distance = max_step(previous_q, target)
    if distance > MAX_STEP:
        # Один множитель для всех суставов сохраняет направление движения.
        fraction = (MAX_STEP - Q_ROUNDING_MARGIN) / distance
        target = command_pose(tuple(
            a + fraction * (b - a) for a, b in zip(previous_q, target)
        ))
    if not valid_pose(target) or max_step(previous_q, target) > MAX_STEP:
        raise ValueError("Joint target exceeds limits after rounding")

    return tuple(
        Motor(motor.mode, q, clamp(motor.dq, -20.0, 20.0),
              clamp(motor.kp, 0.0, 1000.0), clamp(motor.kd, 0.0, 100.0),
              clamp(motor.tau, -100.0, 100.0))
        for motor, q in zip(motors, target)
    )


def motion_samples(start, end):
    distance = max_step(start, end)
    if distance == 0.0:
        yield start
        return
    intervals = math.ceil(distance / COLLISION_STEP)
    yield start
    for step in range(1, intervals):
        t = step / intervals
        yield tuple(a + (b - a) * t for a, b in zip(start, end))
    yield end


def safe_motion(start, end, is_safe_pose):
    return all(is_safe_pose(pose) for pose in motion_samples(start, end))


def hold_command(pose):
    # держим последнюю принятую цель измеренные углы сюда не подставляем.
    return tuple(Motor(SERVO_MODE, q, 0.0, 40.0, 2.0, 0.0) for q in pose)
