import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
import math
from geometry_msgs.msg import Point


class SignalPublisher(Node):

    def __init__(self):
        super().__init__('get_signal')
        self.publisher_ = self.create_publisher(Float32, 'signal', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        """
        :param p0: Signal strength at reference distance (dBm)
        :param n: Path loss exponent (environment-dependent, 2-4)
        """
        # Fake params for path loss model
        self.p0 = -40
        self.n = 3

    def get_wifi_signal_strength(self):
        """
        Calculates Wi-Fi signal strength based on distance from (0,0).
        :param x: X-coordinate of the receiver
        :param y: Y-coordinate of the receiver
        :return: Signal strength in dBm
        """
        if self.x != 0 and self.y != 0:
            d = math.sqrt(self.x**2 + self.y**2)  # Compute distance from (0,0)
            if d < 1:
                d = 1  # Avoid log(0) issues; assume minimum 1m distance

            rssi = self.p0 - 10 * self.n * math.log10(d)
            # print("hello")
        return round(rssi, 1)

    def timer_callback(self):
        signal_strength = self.get_wifi_signal_strength()

        if signal_strength is not None:
            msg = Float32()
            msg.data = signal_strength
            self.publisher_.publish(msg)
            # self.get_logger().info(f"{msg.data} dBm")
        else:
            self.get_logger().info("Failed to retrieve signal strength.")

def main(args=None):
    rclpy.init(args=args)

    signal_publisher = SignalPublisher()

    rclpy.spin(signal_publisher)

    # Destroy the node explicitly
    signal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
