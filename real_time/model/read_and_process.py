import rclpy
from rclpy.node import Node
import serial
from data.data_processing import DataProcessor
from utils.utils import subtract_bias

class ReadAndProcess(Node):
    def __init__(self, serial_port, baud_rate):
        super().__init__('read_and_process')
        # Initialize serial connection
        self.ser = serial.Serial(serial_port, baud_rate)
        # Create a timer to periodically check for serial data
        self.timer = self.create_timer(0.1, self.read_and_process)
        # self.data_processor = DataProcessor()
        self.callibrate = True
        self.cllibration_length = 100

    def readline(self):
        # reads a line 
        line = self.ser.readline().decode("utf-8").rstrip(',\r\n')# Read a line from the serial port
        data = [int(num) for num in line.split(',')]

        return data
    
    def callibrate(self):
        data = []
        for i in range(self.cllibration_length):
            data.append(self.readline())        
        
        return calldata 
    def read_and_process(self):
        
        for i in range(100):
            self.readline()
        if self.callibrate:
            self.callibrate()

            # self.get_logger().info(f"Received data: {calldata[-1]}")
        # self.process_data(data)

    def process_data(self):
        # subtruct bais 

        
        # Process the data received from serial
        # Example: Log the data or publish to a ROS2 topic
        # self.get_logger().info(f"Received data: {}")
        pass

def main(args=None):
    rclpy.init(args=args)

    read_and_process_node = ReadAndProcess('/dev/ttyACM0', 115200)  # Specify your serial port and baud rate

    try:
        rclpy.spin(read_and_process_node)
    except KeyboardInterrupt:
        pass
    finally:
        read_and_process_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
