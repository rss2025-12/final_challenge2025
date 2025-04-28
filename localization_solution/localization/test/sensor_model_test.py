import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from localization.sensor_model import SensorModel
from localization.test import TEST_PRECOMPUTED_TABLE, TEST_SENSOR_MODEL_OUTPUT_PROBABILITIES, \
    TEST_SENSOR_MODEL_INPUT_SCANS, \
    TEST_PARTICLES_2, TEST_MAP_ARRAY


class Args:
    alpha_hit = 0.74
    alpha_short = 0.07
    alpha_max = 0.07
    alpha_rand = 0.12
    sigma_hit = 8.0
    table_width = 201

    tolerance = 0.01


class SensorModelTest(Node):
    def __init__(self):
        # Hack: particle filter
        super().__init__('particle_filter')
        self.sensor_model = SensorModel(self)

        # overwrite alphas
        self.sensor_model.alpha_hit = Args.alpha_hit
        self.sensor_model.alpha_short = Args.alpha_short
        self.sensor_model.alpha_max = Args.alpha_max
        self.sensor_model.alpha_rand = Args.alpha_rand
        self.sensor_model.sigma_hit = Args.sigma_hit
        self.sensor_model.table_width = Args.table_width

        self.tol = Args.tolerance

        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_cb,
            1)

    def map_cb(self, msg):
        self.sensor_model.map_callback(msg)
        self.sensor_model.precompute_sensor_model()

        self.test_map_callback()
        self.test_precompute()
        self.test_evaluate()
        self.get_logger().info("All tests passed!")

    def test_all(self):

        try:
            self.test_precompute()
        except AssertionError:
            self.get_logger().error("Precompute test failed :(")
        try:
            self.test_evaluate()
        except AssertionError:
            self.get_logger().error("Evaluate test failed :(")

        try:
            self.test_map_callback()
        except AssertionError:
            self.get_logger().error("Map callback test failed :(")

        self.get_logger().info("All tests passed :)")

    def test_precompute(self):
        expected = np.array(TEST_PRECOMPUTED_TABLE)
        actual = self.sensor_model.sensor_model_table

        assert actual.shape == expected.shape, f"Wrong shape of the precomputed table. Expected {expected.shape}, got {actual.shape}"

        assert np.allclose(actual, expected, atol=self.tol), "Wrong values in the precomputed table"

        self.get_logger().info("Precompute test passed :)")

    def test_evaluate(self):
        expected_probabilities = np.array(TEST_SENSOR_MODEL_OUTPUT_PROBABILITIES)
        actual_probabilities = self.sensor_model.evaluate(
            np.array(TEST_PARTICLES_2), np.array(TEST_SENSOR_MODEL_INPUT_SCANS))

        assert np.allclose(expected_probabilities,
                           actual_probabilities,
                           rtol=self.tol), f"Expected {expected_probabilities}, got {actual_probabilities}"

        self.get_logger().info("Evaluate test passed :)")

    def test_map_callback(self):
        expected_map = np.array(TEST_MAP_ARRAY)
        actual_map = self.sensor_model.map

        assert np.allclose(expected_map, actual_map, atol=self.tol), "Map does not match exactly"

        self.get_logger().info("Map callback test passed :)")


def main(args=None):
    rclpy.init(args=args)
    pf = SensorModelTest()
    pf.get_logger().info("Waiting for map, running the racecar simulator in a few seconds...")
    rclpy.spin(pf)
    rclpy.shutdown()
