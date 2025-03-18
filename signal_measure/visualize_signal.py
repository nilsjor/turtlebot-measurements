import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, Header

_TYPE_TO_SIZE = {
    PointField.INT8: 1,
    PointField.UINT8: 1,
    PointField.INT16: 2,
    PointField.UINT16: 2,
    PointField.INT32: 4,
    PointField.UINT32: 4,
    PointField.FLOAT32: 4,
    PointField.FLOAT64: 8,
}

_TYPE_TO_FORMAT = {
    PointField.INT8: 'b',
    PointField.UINT8: 'B',
    PointField.INT16: 'h',
    PointField.UINT16: 'H',
    PointField.INT32: 'i',
    PointField.UINT32: 'I',
    PointField.FLOAT32: 'f',
    PointField.FLOAT64: 'd',
}

def send_command(ser, command, printout=False):
    """Send an AT command to the serial device and process the response."""
    ser.flushInput()  # Clear input buffer
    ser.flushOutput()  # Clear output buffer

    ser.write(("AT"+command+"\r\n").encode())  # Send command

    raw = ser.read_until(b'OK\r\n')  # Read response until "OK"

    response = raw.decode(errors='ignore')  # Decode the response, ignoring errors

    if printout: # Print the response if requested
        print(">>>", response)

    # Process response
    for line in response.splitlines():
        line = line.strip()  # Remove leading/trailing whitespace
        if line.startswith(command.split("=")[0]):  # Check if the line starts with the command (ignoring args)
            parts = line.split(":")[1].split(",")  # Get the part after `:`, then split by `,`
            numbers = tuple(int(p) for p in parts if p.strip().lstrip('-').isdigit())  # Convert valid numbers
            return np.array(numbers)  # Return tuple with only valid integers
    return None  # Return None if no valid response is found

def add_field(fields, name, datatype, count=1):
    """Helper function to add a field dynamically."""
    offset = fields[-1].offset + _TYPE_TO_SIZE[fields[-1].datatype] * fields[-1].count if fields else 0
    fields.append(PointField(name=name, offset=offset, datatype=datatype, count=count))
    return _TYPE_TO_FORMAT[datatype] * count

def create_msg():
    
    fields = []
    format_str = ''
    
    # Define the fields
    format_str += add_field(fields, 'x', PointField.FLOAT32)
    format_str += add_field(fields, 'y', PointField.FLOAT32)
    format_str += add_field(fields, 'z', PointField.FLOAT32)
    format_str += add_field(fields, 'Signal', PointField.FLOAT32)
    format_str += add_field(fields, 'rgb', PointField.UINT32)

    msg = PointCloud2()
    msg.height = 1  # Unorganized point cloud
    msg.width = 1  # Only one measurement
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = sum(_TYPE_TO_SIZE[f.datatype] * f.count for f in fields)
    msg.row_step = msg.point_step
    msg.is_dense = True

    return msg, format_str

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('channel_info_publisher')
        qos = QoSProfile(depth=10)

        # Declare parameters
        self.declare_parameter('max_pub_hz', 1)
        self.declare_parameter('deadline_msec', 500)

        # Get parameters
        max_freq = self.get_parameter('max_pub_hz').get_parameter_value().integer_value
        self.deadline = self.get_parameter('deadline_msec').get_parameter_value().integer_value / 1000.0


        # Subscribe to fake signal generator
        self.signal_subscription = self.create_subscription(
            Float32,
            'signal',
            self.signal_callback,
            10)
        self.latest_signal_strength = None

        # Create message, re-use for each publish
        self.msg, self.format_str = create_msg()
        self.get_logger().debug(f'Creating PointCloud2 message using the format \'{self.format_str}\'')

        #self.timer = self.create_timer(1.0 / max_freq, self.timer_callback)
        self.pub = self.create_publisher(PointCloud2, 'channel_info', 10)

    def signal_callback(self, msg):
        # self.get_logger().info(f"Received signal strength: {msg.data} dBm")
        self.latest_signal_strength = msg.data
        self.add_point()

    def add_point(self):
        
        data = []
        x, y, z = np.zeros(3)

        # Convert signal to color
        normalized_signal = (self.latest_signal_strength + 50) / 10.0  # Assuming signal strength is between -50 and -40 dBm
        normalized_signal = max(0.0, min(1.0, normalized_signal))
        # print(normalized_signal)
        # Convert normalized signal strength to RGB color
        r = int(255 * (1 - normalized_signal))
        g = int(255 * normalized_signal)
        b = 0

        data.append(struct.pack(self.format_str,
            x, y, z, 
            self.latest_signal_strength,
            (r << 16) | (g << 8) | b)
        )

        # Populate message with data
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = "base_link"
        self.msg.data = b''.join(data)

        # Publish message
        self.pub.publish(self.msg)

        if rclpy.logging.get_logger_effective_level(self.get_logger().name) <= rclpy.logging.LoggingSeverity.DEBUG:
            self.get_logger().info(f'\n' +
                f'Signal:\t{self.latest_signal_strength}\n' 
            )

def main(args=None):
    rclpy.init(args=args)
    signal_visualizer = PointCloudPublisher()
    rclpy.spin(signal_visualizer)
    signal_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
