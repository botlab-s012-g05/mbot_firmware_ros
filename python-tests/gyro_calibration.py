
import rclpy
import time
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


CALIBRATION_SECONDS = 10
SAMPLE_PERIOD = 0.01
SAMPLE_COUNT = int(CALIBRATION_SECONDS / SAMPLE_PERIOD)


class GyroCalibration(Node):

    def __init__(self):
        super().__init__('gyro_calibration')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos_profile
        )
        
        self.gyro_samples = []
        self.start_time = None
        self.calibration_complete = False

        self.get_logger().info(
            'Place the robot on a stable, level surface and keep it still.'
        )
        self.get_logger().info('Starting in 3 seconds...')

        time.sleep(3)

        self.start_time = time.monotonic()

    def imu_callback(self, msg):

        if self.calibration_complete:
            return

        # Elapsed time
        elapsed_time = time.monotonic() - self.start_time

        # Extract angular velocity (rad/s)
        gyro = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]

        self.gyro_samples.append([elapsed_time] + gyro)

        if elapsed_time >= CALIBRATION_SECONDS:
            self.calculate_calibration()

    def calculate_calibration(self):

        self.calibration_complete = True

        data = np.array(self.gyro_samples)

        times = data[:, 0]
        gyro = data[:, 1:4]

        print('\nNumber of samples:', len(times))

        print('\nLeast-squares fit:')
        print('gyro(t) = slope * t + bias\n')

        # Design matrix:
        # [t0  1]
        # [t1  1]
        # ...
        A = np.column_stack((times, np.ones(len(times))))

        for axis in range(3):

            # Least Squares solution
            slope, bias = np.linalg.lstsq(
                A,
                gyro[:, axis],
                rcond=None
            )[0]

            bias_deg = np.degrees(bias)

            print(f'Axis {"XYZ"[axis]}:')
            print(f'  Bias       = {bias:+.8f} rad/s')
            print(f'  Bias       = {bias_deg:+.6f} deg/s')
            print(f'  Drift slope = {slope:+.8f} rad/s^2')
            print()

        print('Calibration complete.')
        print('Apply:')
        print('gyro_corrected = gyro_measured - bias')


def main(args=None):

    rclpy.init(args=args)

    node = GyroCalibration()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
