import math

import board
import busio
import adafruit_lsm9ds0
import rclpy
import sensor_msgs.msg
from rclpy.node import Node


class LSM9DS0Node(Node):
    def __init__(self):
        super().__init__("lsm9ds0_node")

        # Logger
        self.logger = self.get_logger()

        # Parameters
        self.declare_parameter("frame_id", "imu_lsm9ds0")
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.frame_id = frame_id

        self.declare_parameter("pub_rate", 50)
        pub_rate = self.get_parameter("pub_rate").get_parameter_value().integer_value
        self.pub_rate = pub_rate

        # IMU instance
        i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_lsm9ds0.LSM9DS0_I2C(i2c)

        # Publishers
        self.imu_pub_ = self.create_publisher(sensor_msgs.msg.Imu, "/imu/data_raw", 10)
        self.mag_pub_ = self.create_publisher(
            sensor_msgs.msg.MagneticField, "/imu/mag_raw", 10
        )
        self.pub_clk_ = self.create_timer(1 / self.pub_rate, self.publish_cback)

    def publish_cback(self):
        imu_msg = sensor_msgs.msg.Imu()
        mag_msg = sensor_msgs.msg.MagneticField()

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        ax, ay, az = self.imu.acceleration
        gx, gy, gz = self.imu.gyro
        mx, my, mz = self.imu.magnetic
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)
        imu_msg.orientation_covariance[0] = -1

        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = mx / 1e4
        mag_msg.magnetic_field.y = my / 1e4
        mag_msg.magnetic_field.z = mz / 1e4

        self.imu_pub_.publish(imu_msg)
        self.mag_pub_.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    lsm9ds0_node = LSM9DS0Node()
    rclpy.spin(lsm9ds0_node)

    lsm9ds0_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
