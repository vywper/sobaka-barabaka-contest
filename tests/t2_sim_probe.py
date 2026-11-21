import csv
import math
from pathlib import Path
import random
import time

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from unitree_go.msg import LowCmd, LowState


MIN_Q = (-0.863, -0.686, -2.818) * 4
MAX_Q = (0.863, 4.501, -0.888) * 4
MODEL_DIR = Path("/opt/go1_model")

"""подаёт тестовые команды и оценивает /lowcmd в публичной сцене lowlevel."""

def pose_of(message):
    return tuple(m.q for m in message.motor_cmd[:12])


def matching_pose(left, right):
    return max(abs(a - b) for a, b in zip(left, right)) < 1e-6


def make_command(pose, *, dq=0.25, tau=0.5, kp=30.0, kd=2.0):
    message = LowCmd()
    for motor, angle in zip(message.motor_cmd, pose):
        motor.mode, motor.q = 10, angle
        motor.dq, motor.tau, motor.kp, motor.kd = dq, tau, kp, kd
    return message


def holding(message, goal):
    return matching_pose(pose_of(message), goal) and all(
        (m.mode, m.dq, m.tau, m.kp, m.kd) == (10, 0.0, 0.0, 40.0, 2.0)
        for m in message.motor_cmd[:12]
    )


class Probe(Node):
    def __init__(self):
        super().__init__("go1_t2_probe")
        self.now = 0.0
        self.seen_state = False
        self.samples = []
        self.last_input = 0.0
        self.publisher = self.create_publisher(LowCmd, "/lowcmd_bare", 1)
        self.create_subscription(Clock, "/clock", self.update_clock, qos_profile_sensor_data)
        self.create_subscription(LowState, "/lowstate", self.update_state, qos_profile_sensor_data)
        self.create_subscription(LowCmd, "/lowcmd", self.collect, 100)

    def update_clock(self, message):
        self.now = message.clock.sec + 1e-9 * message.clock.nanosec

    def update_state(self, _message):
        self.seen_state = True

    def collect(self, message):
        if self.now > 0:
            self.samples.append((self.now, message))

    def wait_for(self, predicate, wall_timeout):
        deadline = time.monotonic() + wall_timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise AssertionError("Тайм аут ожидания условия")
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_for(self, seconds, command=None):
        end = self.now + seconds
        next_input = self.now
        start_index = len(self.samples)

        def finished():
            nonlocal next_input
            if self.now >= end:
                return True
            if command is not None and self.now >= next_input:
                self.publisher.publish(command)
                self.last_input = self.now
                next_input = self.now + 0.018
            return False

        self.wait_for(finished, max(10, 5 * seconds))
        return self.samples[start_index:]


class GeometryOracle:
    """Независимая проверка выхода через полный прямой расчёт MuJoCo."""
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "scene.xml"))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        with (MODEL_DIR / "t2_collision_pairs.csv").open(newline="") as stream:
            self.pairs = {
                tuple(sorted((int(row["geom1_id"]), int(row["geom2_id"]))))
                for row in csv.DictReader(stream) if row["diagnostic_policy"] == "checked"
            }

    def collides(self, angles):
        self.data.qpos[7:] = angles
        mujoco.mj_forward(self.model, self.data)
        return any(
            contact.dist < -0.001 and tuple(sorted(contact.geom)) in self.pairs
            for contact in self.data.contact
        )

    def find_collision(self):
        randomizer = random.Random(2)
        for _ in range(500):
            pose = tuple(randomizer.uniform(a, b) for a, b in zip(MIN_Q, MAX_Q))
            if self.collides(pose):
                return pose
        raise AssertionError("Expected a colliding target within the joint ranges")

    def validate_stream(self, samples):
        previous = None
        for _, message in samples:
            for motor, lower, upper in zip(message.motor_cmd[:12], MIN_Q, MAX_Q):
                assert lower <= motor.q <= upper, (motor.q, lower, upper)
                assert -20 <= motor.dq <= 20 and -100 <= motor.tau <= 100
                assert 0 <= motor.kp <= 1000 and 0 <= motor.kd <= 100
            # У неиспользуемых приводов все числовые поля тоже остаются конечными.
            assert all((m.mode, m.q, m.dq, m.tau, m.kp, m.kd) == (0, 0, 0, 0, 0, 0)
                       for m in message.motor_cmd[12:])
            current = pose_of(message)
            if previous is None:
                assert not self.collides(current), current
            elif previous != current:
                distance = max(abs(a - b) for a, b in zip(previous, current))
                assert distance <= math.radians(5), distance
                intervals = math.ceil(distance / math.radians(0.1))
                for pose in np.linspace(previous, current, intervals + 1):
                    assert not self.collides(pose), tuple(pose)
            previous = current


def verify_hold(samples, target, last_input):
    holding_after = [stamp for stamp, message in samples if holding(message, target)]
    assert holding_after and holding_after[0] - last_input <= 0.5, (holding_after, last_input)
    assert samples[-1][0] - last_input >= 0.5
    assert all(holding(message, target) for stamp, message in samples if stamp >= holding_after[0])
    print(f"Удержание включилось через {holding_after[0] - last_input:.3f} с по /clock")


def verify(probe, oracle, unsafe):
    probe.wait_for(lambda: probe.seen_state and probe.samples
                   and probe.publisher.get_subscription_count() > 0, 45)
    start = probe.now
    home = pose_of(probe.samples[-1][1])
    assert holding(probe.samples[-1][1], home)
    target = (home[0] + 0.02, *home[1:])
    active = probe.run_for(0.35, make_command(target))
    assert any(matching_pose(pose_of(message), target) and message.motor_cmd[0].tau == 0.5
               for _, message in active), "Safe input was not forwarded"

    # После обрыва входа цель должна остаться прежней, даже если робот сместился.
    last_input = probe.last_input
    verify_hold(probe.run_for(0.65), target, last_input)

    resumed = (target[0] - 0.04, *target[1:])
    active = probe.run_for(0.3, make_command(resumed))
    assert any(matching_pose(pose_of(message), resumed) and message.motor_cmd[0].tau == 0.5
               for _, message in active), "Did not resume after hold"
    last_input = probe.last_input
    invalid = make_command(target, tau=float("inf"))
    invalid_samples = probe.run_for(0.65, invalid)
    assert all(matching_pose(pose_of(message), resumed) for _, message in invalid_samples)
    verify_hold(invalid_samples, resumed, last_input)

    extreme = (10.0, *resumed[1:])
    corrected = probe.run_for(0.25, make_command(extreme, dq=40.0, tau=120.0, kp=1500.0, kd=150.0))
    assert any(
        (m.dq, m.tau, m.kp, m.kd) == (20.0, 100.0, 1000.0, 100.0)
        for _, message in corrected for m in message.motor_cmd[:12]
    ), "Out-of-range input was not corrected"
    assert any(not matching_pose(pose_of(message), resumed) for _, message in corrected)

    blocked = probe.run_for(2.75, make_command(unsafe, kp=10.0))
    last = blocked[-1][1]
    # Вход продолжает идти, но блок остановил переход у границы столкновения.
    assert holding(last, pose_of(last)), "Collision request did not trigger hold"
    assert not matching_pose(pose_of(last), unsafe), "Unsafe goal reached the actuator"

    if probe.now < start + 5.15:
        probe.run_for(start + 5.15 - probe.now)
    window = [stamp for stamp, _ in probe.samples if start <= stamp < start + 5.0]
    max_gap = max(b - a for a, b in zip([start, *window], [*window, start + 5.0]))
    rate = len(window) / 5.0
    assert rate >= 50 and max_gap <= 0.1, (rate, max_gap)
    print(f"/lowcmd: {rate:.1f} Гц за 5 с, максимальный интервал {max_gap * 1000:.1f} мс")
    oracle.validate_stream(probe.samples)
    print(f"Проверены диапазоны и переходы для {len(probe.samples)} выходных сообщений")


def main():
    # Загружаем геометрию до подписки, чтобы подготовка модели не прерывала замер частоты.
    oracle = GeometryOracle()
    unsafe = oracle.find_collision()
    rclpy.init()
    probe = Probe()
    try:
        verify(probe, oracle, unsafe)
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
