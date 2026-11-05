import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = Node('test_turning')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    publish_rate = 0.05  # 20 Hz

    def ramp_velocity(start, goal, ramp_time):
        start_time = time.monotonic()

        while True:
            elapsed = time.monotonic() - start_time
            ratio = min(elapsed / ramp_time, 1.0)

            velocity = start + (goal - start) * ratio

            msg = Twist()
            msg.angular.z = velocity
            pub.publish(msg)

            if ratio >= 1.0:
                break

            time.sleep(publish_rate)

    def hold_velocity(velocity, duration):
        start_time = time.monotonic()

        while time.monotonic() - start_time < duration:
            msg = Twist()
            msg.angular.z = velocity
            pub.publish(msg)
            time.sleep(publish_rate)

    def stop(duration):
        hold_velocity(0.0, duration)

    try:
        print("Starting rotational PID test...")

        # Initial stop
        stop(1.0)

        # Clockwise: 0 -> -1.0 rad/s
        ramp_velocity(0.0, -1.0, 2.0)
        hold_velocity(-1.0, 4.0)
        ramp_velocity(-1.0, 0.0, 2.0)

        # Pause between directions
        stop(1.0)

        # Counterclockwise: 0 -> +1.0 rad/s
        ramp_velocity(0.0, 1.0, 2.0)
        hold_velocity(1.0, 4.0)
        ramp_velocity(1.0, 0.0, 2.0)

        # Final stop
        stop(1.0)

        print("Rotational test completed.")

    finally:
        pub.publish(Twist())
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()