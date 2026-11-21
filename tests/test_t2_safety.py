from dataclasses import replace
import math
import unittest

from participant.t2_safety import (
    COLLISION_STEP, MAX_STEP, Motor, Q_MAX, Q_MIN, command_pose, finite_motor,
    hold_command, limited_command, max_step, motion_samples, safe_motion, valid_pose,
)


HOME = command_pose((0.0, 0.9, -1.8) * 4)
SAFE = hold_command(HOME)


class SafetyTests(unittest.TestCase):
    def test_safe_command_passes_unchanged(self):
        command = (replace(SAFE[0], q=0.03125, dq=0.5, tau=-1.0), *SAFE[1:])
        self.assertEqual(limited_command(command, HOME), command)

    def test_all_joint_limits_survive_float32_conversion(self):
        for bounds in (Q_MIN, Q_MAX):
            rounded = command_pose(bounds)
            self.assertTrue(valid_pose(rounded))
            self.assertLess(max_step(rounded, bounds), 2e-6)

    def test_large_command_is_clipped_in_one_step(self):
        for sign in (-1, 1):
            raw = tuple(Motor(0, 1e30 * sign, 1e30 * sign, 1e30 * sign,
                              1e30 * sign, 1e30 * sign) for _ in HOME)
            result = limited_command(raw, HOME)
            goal = tuple(m.q for m in result)
            self.assertTrue(valid_pose(goal))
            self.assertLessEqual(max_step(HOME, goal), MAX_STEP)
            self.assertAlmostEqual(max_step(HOME, goal), MAX_STEP, places=5)
            self.assertTrue(all(m.mode == 0 and m.dq == 20 * sign and m.tau == 100 * sign
                                and m.kp == max(0, 1000 * sign)
                                and m.kd == max(0, 100 * sign) for m in result))

    def test_each_numeric_field_rejects_nonfinite_values(self):
        for field in ("q", "dq", "kp", "kd", "tau"):
            for value in (math.nan, math.inf, -math.inf):
                with self.subTest(field=field, value=value):
                    self.assertFalse(finite_motor(replace(SAFE[0], **{field: value})))

    def test_unsafe_middle_is_rejected_even_when_endpoints_pass(self):
        end = (0.08, *HOME[1:])
        samples = list(motion_samples(HOME, end))
        self.assertEqual(samples[0], HOME)
        self.assertEqual(samples[-1], end)
        self.assertTrue(all(max_step(a, b) <= COLLISION_STEP + 1e-12
                            for a, b in zip(samples, samples[1:])))
        unsafe_middle = lambda pose: not 0.039 <= pose[0] <= 0.041
        self.assertTrue(unsafe_middle(HOME) and unsafe_middle(end))
        self.assertFalse(safe_motion(HOME, end, unsafe_middle))


if __name__ == "__main__":
    unittest.main()
