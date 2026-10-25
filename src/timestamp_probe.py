
import time
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped


class TimestampProbe(Node):

    def __init__(self):
        super().__init__("timestamp_probe")

        self.camera_count = 0
        self.ekf_count = 0

        self.camera_sub = self.create_subscription(
            Image,
            "/vehicles/beamng_bridge/sensors/camera_L/colour",
            self.camera_callback,
            qos_profile_sensor_data
        )

        self.ekf_sub = self.create_subscription(
            PoseStamped,
            "/adas/ekf/pose",
            self.ekf_callback,
            qos_profile_sensor_data
        )

    def measure_age(self, stamp):
        stamp_ns = (
            stamp.sec * 1_000_000_000
            + stamp.nanosec
        )

        now_ns = time.time_ns()

        return (now_ns - stamp_ns) / 1e6

    def camera_callback(self, msg):
        self.camera_count += 1

        if self.camera_count % 10 == 0:
            age = self.measure_age(msg.header.stamp)
            print(f"CAMERA received: age = {age:.2f} ms")

    def ekf_callback(self, msg):
        self.ekf_count += 1

        if self.ekf_count % 30 == 0:
            age = self.measure_age(msg.header.stamp)
            print(f"EKF received: age = {age:.2f} ms")


def main():
    rclpy.init()
    node = TimestampProbe()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()