import numpy as np
import rclpy
from rclpy.node import Node

from localization.motion_model import MotionModel
from localization.test import TEST_PARTICLES, TEST_MOTION_MODEL_ODOM, TEST_MOTION_MODEL_RESULTS


class MotionModelTest(Node):
    def __init__(self):
        super().__init__('particle_filter')
        self.motion_model = MotionModel(self)

        self.particles = np.array(TEST_PARTICLES)
        self.odom = np.array(TEST_MOTION_MODEL_ODOM)
        self.expected = np.array(TEST_MOTION_MODEL_RESULTS)

        self.tol = 0.1

    def test_evaluate_motion_model(self):
        actual = self.motion_model.evaluate(self.particles, self.odom)
        assert np.allclose(self.expected,
                           actual,
                           rtol=self.tol), f"Expected {self.expected}, got {actual}"

        self.get_logger().info("Motion model test passed!")


def main(args=None):
    rclpy.init(args=args)
    motion_model_test = MotionModelTest()
    motion_model_test.test_evaluate_motion_model()
    rclpy.spin(motion_model_test)
    rclpy.shutdown()
