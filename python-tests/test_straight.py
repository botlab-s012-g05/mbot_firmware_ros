import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = Node('test_straight')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    publish_rate = 0.05  # 20 Hz

    def ramp_velocity(start, goal, ramp_time):
        start_time = time.monotonic()

        while True:
            elapsed = time.monotonic() - start_time
            ratio = min(elapsed / ramp_time, 1.0)

            velocity = start + (goal - start) * ratio

            msg = Twist()
            msg.linear.x = velocity
            pub.publish(msg)

            if ratio >= 1.0:
                break

            time.sleep(publish_rate)

    def hold_velocity(velocity, duration):
        start_time = time.monotonic()

        while time.monotonic() - start_time < duration:
            msg = Twist()
            msg.linear.x = velocity
            pub.publish(msg)
            time.sleep(publish_rate)

    def stop(duration):
        hold_velocity(0.0, duration)

    try:
        print("Starting straight-line PID test...")

        # Initial stop
        stop(1.0)

        # Ramp up: 0 -> 0.5 m/s
        ramp_velocity(0.0, 0.5, 2.0)

        # Hold: 0.5 m/s for 4 seconds
        hold_velocity(0.5, 4.0)

        # Ramp down: 0.5 -> 0 m/s
        ramp_velocity(0.5, 0.0, 2.0)

        # Final stop
        stop(1.0)

        print("Straight-line test completed.")

    finally:
        pub.publish(Twist())
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
