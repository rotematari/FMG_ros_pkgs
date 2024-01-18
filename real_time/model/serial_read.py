import rclpy
from rclpy.node import Node
import serial


class SerialNode(Node):
    def __init__(self, serial_port, baud_rate):
        super().__init__('serial_node')
        # Initialize serial connection
        self.ser = serial.Serial(serial_port, baud_rate)
        # Create a timer to periodically check for serial data
        self.timer = self.create_timer(0.001, self.check_serial)

    def check_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode("utf-8").rstrip(',\r\n')# Read a line from the serial port
            data = [int(num) for num in line.split(',')]
            self.process_data(data)

    def process_data(self, data):
        # Process the data received from serial
        # Example: Log the data or publish to a ROS2 topic
        self.get_logger().info(f"Received data: {data}")

def main(args=None):
    rclpy.init(args=args)

    serial_node = SerialNode('/dev/ttyACM0', 115200)  # Specify your serial port and baud rate

    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        pass
    finally:
        serial_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
