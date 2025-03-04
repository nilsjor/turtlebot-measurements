import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('get_pos')
        self.publisher_ = self.create_publisher(Point, 'turtlebot3_position', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timer_period = 0.1  # seconds (increase the period to slow down publishing rate)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.latest_position = None

    def odom_callback(self, msg):
        # Extract the position from the odometry message
        self.latest_position = msg.pose.pose.position
        # # Create a Point message to store the position
        # point_msg = Point()
        # point_msg.x = position.x
        # point_msg.y = position.y
        # point_msg.z = position.z
        # # Publish the position to the 'turtlebot3_position' topic
        # self.publisher_.publish(point_msg)
        # # Log the published position
        # self.get_logger().info(f"Publishing TurtleBot3 position: x={point_msg.x}, y={point_msg.y}, z={point_msg.z}")

    def timer_callback(self):
        if self.latest_position is not None:
            point_msg = Point()
            point_msg.x = self.latest_position.x
            point_msg.y = self.latest_position.y
            point_msg.z = self.latest_position.z
            self.publisher_.publish(point_msg)
            # self.get_logger().info(f"x={point_msg.x}, y={point_msg.y}, z={point_msg.z}")

def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
