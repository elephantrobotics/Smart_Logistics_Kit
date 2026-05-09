#!/usr/bin/env python                                                                                                                      
#coding=UTF-8
import rospy
import serial
import time

class SerialCANParser:
    def __init__(self, serial_port='/dev/ttyUSB0', baudrate=9600, timeout=1):
        self.serial_port = serial_port 
        self.baudrate = baudrate  
        self.timeout = timeout  
        self.ser = None  
        self.buffer = bytearray()  
        self.max_retries = 3  

        self.x_speed = 0.0
        self.z_speed = 0.0
        self.infrared_bits = []

    def open_serial(self):
        """Open the serial port"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=self.timeout)
            print(f"Serial port {self.serial_port} opened at {self.baudrate}")
        except Exception as e:
            print(f"Failed to open serial port: {e}")

    def close_serial(self):
        """Close the serial port"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port closed.")
        else:
            print("Serial port not open or already closed.")
    
    def close(self):
        """Alias for close_serial() for compatibility"""
        self.close_serial()

    def can_id_check(self,date):
        high_byte,low_byte = date[0:2]
        can_id = (high_byte << 3)
        can_id |= (low_byte >> 5)

        return can_id

    def parse_can_data(self, data):
        """Parse 8-byte CAN data frame"""
        if len(data) != 8:
            print("Invalid data frame length")
            return None

        x_speed_raw = ((data[0] << 8) | data[1])  # X speed raw data
        z_speed_raw = ((data[4] << 8) | data[5])  # Z speed raw data

        if x_speed_raw & 0x8000:  
            x_speed_raw = -((65536 - x_speed_raw) & 0xFFFF)  

        if z_speed_raw & 0x8000:  
            z_speed_raw = -((65536 - z_speed_raw) & 0xFFFF)  

        self.x_speed = x_speed_raw / 1000.0  # X speed unit: m/s
        self.y_speed = 0  # Y speed unit: 0
        self.z_speed = z_speed_raw / 1000.0  # Z speed unit: rad/s

        self.which_mode = data[2]

        self.infrared = data[6]  # Infrared data
        self.raw_current = data[7]  # Current data

        if self.raw_current > 32767:  
            self.actual_current = -(65536 - self.raw_current) * 30.0
        else:
            self.actual_current = self.raw_current * 30.0

        self.infrared_bits = [(self.infrared >> (7 - i)) & 0x01 for i in range(8)]

        print(f"X Speed: {self.x_speed:.3f}, Y Speed: {self.y_speed}, Z Speed: {self.z_speed:.3f}, "
              f"Actual Current: {self.actual_current:.3f} mA, Infrared: {self.infrared}")

        print(f"L_A: {self.infrared_bits[2]}, L_B: {self.infrared_bits[3]}, R_B: {self.infrared_bits[4]}, "
              f"R_A: {self.infrared_bits[5]}, infrared_flag : {self.infrared_bits[6]}, Charging flag: {self.infrared_bits[7]}")

    def read_serial_data(self):
        """Read serial data and parse"""
        while not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)   # Read one byte from serial port

                if len(self.buffer) < 2:
                    self.buffer.extend(byte)  
                    if len(self.buffer) == 2:

                        if self.buffer[0] != 0x41 or self.buffer[1] != 0x54:
                            # Invalid frame header, clear buffer and continue to next loop
                            self.buffer.clear()
                            continue  # Continue waiting next byte                 

                else:
                    self.buffer.extend(byte)

                    if len(self.buffer) >= 17:

                        can_frame_id = self.can_id_check(self.buffer[2:4])  

                        data_length = self.buffer[6]  
                        data = self.buffer[7:15]  


                        if can_frame_id == 0x182 and data_length == 0x08: 
                            self.parse_can_data(data)

                            self.buffer.clear()
                            return self.x_speed, self.z_speed, self.which_mode, self.infrared_bits   # 返回解析后的数据
                        else:
                            self.buffer.clear()

    def read_serial_response(self):
        """Read serial response data"""
        response = bytearray()  
        while True:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                response += byte

            if b'\r\n' in response:
                break

            if len(response) > 100:
                break

        return bytes(response)  

    def send_at_commands(self, commands):
        """Send AT commands and wait for responses"""
        for command in commands:
            retries = 0
            while retries < self.max_retries:
                self.ser.write(command.encode() + b'\r\n')
                print(f"Sent command: {command}")

                response = self.read_serial_response()

                if b"OK" in response:
                    print(f"Received OK response: {response}")
                    break  
                else:
                    retries += 1
                    print(f"Received response: {response}")

            if retries == self.max_retries:
                print(f"Failed to receive valid response after retry {self.max_retries} times")
                break

    def start(self):
        """Start reading data parser"""
        self.open_serial()
        try:
            self.send_at_commands(["AT+CG", "AT+AT"])

            self.read_serial_data()

        except KeyboardInterrupt:
            print("Program interrupted by user")
        finally:
            self.close_serial()

if __name__ == '__main__':
    parser = SerialCANParser('/dev/ttyUSB0', 9600, 1)
    parser.start()