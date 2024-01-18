import rclpy
from rclpy.node import Node
import serial

import numpy as np 
import pandas as pd 
import sklearn
from data.data_processing import DataProcessor
from utils.utils import subtract_bias
from sklearn.preprocessing import MinMaxScaler

class ReadAndProcess(Node):
    def __init__(self, serial_port, baud_rate):
        super().__init__('read_and_process')
        # Initialize serial connection
        self.ser = serial.Serial(serial_port, baud_rate)
        # Create a timer to periodically check for serial data
        self.timer = self.create_timer(0.1, self.read_and_process)
        # self.data_processor = DataProcessor()
        self.callibrate = True
        self.normelize = True 
        self.session_bias = None
        self.min_max_scaler = None
        self.calibration_length = 1000
        # take from the model config
        self.sequence_length = 100 #same as the config and the model 
    def readline(self):
        # reads a line 
        line = self.ser.readline().decode("utf-8").rstrip(',\r\n')# Read a line from the serial port
        data = [int(num) for num in line.split(',')]

        return data
    
    def find_bias(self):
        data = []
        for i in range(self.calibration_length):
            data.append(self.readline())
            if i % 100 == 0:
                print(f'{i} points collected')

        data = np.array(data, dtype=float)
        # Calculate the mean of each column
        self.session_bias = data.mean(axis=0)
        self.calldata = data - self.session_bias
    def min_max_norm(self):
        """
        Initializes and fits the MinMaxScaler with the calibration data.
        """
        if self.session_bias is not None:
            self.min_max_scaler = MinMaxScaler(feature_range=(0, 1))
            self.min_max_scaler.fit(self.calldata)
        
    def read_and_process(self):
        
        for i in range(100):
            self.readline()
        if self.session_bias:
            self.find_bias()
            self.calibrate = False
            if self.normelize:
                self.min_max_norm()
                self.normelize = False
        
        # get sequence
        data = []
        for i in range(self.sequence_legth):
            data.append(self.readline())
        
        if self.session_bias is not None and self.min_max_scaler is not None:
            data = [self.readline() for _ in range(self.sequence_length)]

            sequence = np.array(data, dtype=float) - self.session_bias

            normalized_data = self.min_max_scaler.transform(sequence)
        
        # TODO:to tensor

        #predict 

            # self.get_logger().info(f"Received data: {calldata[-1]}")
        # self.process_data(data)

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
