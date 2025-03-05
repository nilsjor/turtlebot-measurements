import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np

class SignalVisualizer(Node):

    def __init__(self):
        super().__init__('signal_visualizer')
        self.signal_subscription = self.create_subscription(
            Float32,
            'signal',
            self.signal_callback,
            10)
        self.position_subscription = self.create_subscription(
            Point,
            'turtlebot3_position',
            self.position_callback,
            10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, 'signal_pointcloud', 10)
        self.latest_signal_strength = None
        self.latest_position = None
        self.current_position = None
        self.points = []

    def signal_callback(self, msg):
        # self.get_logger().info(f"Received signal strength: {msg.data} dBm")
        self.latest_signal_strength = msg.data
        self.add_point()

    def position_callback(self, msg):
        # self.get_logger().info(f"Received position: x={msg.x}, y={msg.y}, z={msg.z}")
        self.latest_position = msg

    def add_point(self):
        if self.latest_signal_strength is not None and self.latest_position is not None:
            if not compare_points(self.latest_position, self.current_position): #only if robot is moving
                #self.get_logger().info(str(compare_pointclouds(self.latest_position, self.current_position)))
                #print(compare_points(self.latest_position, self.current_position))
                x = self.latest_position.x
                y = self.latest_position.y
                z = self.latest_position.z
                signal_strength = self.latest_signal_strength
                # print(signal_strength)
                # Normalize signal strength to a value between 0 and 1
                normalized_signal = (signal_strength + 50) / 10.0  # Assuming signal strength is between -50 and -40 dBm
                normalized_signal = max(0.0, min(1.0, normalized_signal))
                # print(normalized_signal)
                # Convert normalized signal strength to RGB color
                r = int(255 * (1 - normalized_signal))
                g = int(255 * normalized_signal)
                b = 0

                self.points.append((x, y, z, r, g, b))
                # self.get_logger().info(f"Added point: x={x}, y={y}, z={z}, r={r}, g={g}, b={b}")
                self.current_position = self.latest_position 

                # self.get_logger().info(str(current))  
                self.publish_pointcloud()

    def publish_pointcloud(self):
        header = self.create_header()
        fields = self.create_fields()
        pointcloud_data = self.create_pointcloud_data()

        pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(self.points),
            fields=fields,
            is_bigendian=False,
            point_step=16,
            row_step=16 * len(self.points),
            data=pointcloud_data,
            is_dense=True
        )
        self.pointcloud_publisher.publish(pointcloud_msg)
        self.get_logger().info(f"Published point cloud with {len(self.points)} points")

    def create_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        return header

    def create_fields(self):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]
        return fields

    def create_pointcloud_data(self):
        pointcloud_data = []
        for point in self.points:
            x, y, z, r, g, b = point
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
            pointcloud_data.append(struct.pack('fffI', x, y, z, rgb))
        return b''.join(pointcloud_data)

def compare_points(p1: Point, p2: Point, tolerance=1e-2):

    if p1 is not None and p2 is not None:
        """Compare two geometry_msgs.msg.Point objects."""
        return (
            np.isclose(p1.x, p2.x, atol=tolerance) and
            np.isclose(p1.y, p2.y, atol=tolerance) and
            np.isclose(p1.z, p2.z, atol=tolerance)
        )
    else:
        return False

def main(args=None):
    rclpy.init(args=args)
    signal_visualizer = SignalVisualizer()
    rclpy.spin(signal_visualizer)
    signal_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
