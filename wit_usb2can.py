#!/usr/bin/env python                                                                                                                      
#coding=UTF-8
import rospy
import serial
import time


class SerialCANParser:
    def __init__(self, serial_port='/dev/ttyUSB0', baudrate=9600, timeout=1):
        self.serial_port = serial_port  # Serial Port Name
        self.baudrate = baudrate  # Baud Rate
        self.timeout = timeout  # Timeout Setting
        self.ser = None  # Serial Port Object
        self.buffer = bytearray()  # Storage for Currently Read Bytes
        self.max_retries = 3  # Maximum Retry Times

        # Real-time Data Storage
        self.x_speed = 0.0
        self.z_speed = 0.0
        self.infrared_bits = []

    def open_serial(self):
        """Open the serial port"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=self.timeout)
            print(f"Serial port {self.serial_port} has been successfully opened, baud rate: {self.baudrate}")
        except Exception as e:
            print(f"Failed to open serial port: {e}")

    def close_serial(self):
        """Close the serial port"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port has been successfully closed.")
        else:
            print("Serial port is not open or has already been closed.")

    def can_id_check(self,date):
        high_byte,low_byte = date[0:2]
        # High byte left shift 3 bits
        can_id = (high_byte << 3)
        # Low byte right shift 5 bits
        can_id |= (low_byte >> 5)

        return can_id

    def parse_can_data(self, data):
        """Parse 8-byte CAN data frame"""
        if len(data) != 8:
            print("Data frame length is not 8 bytes")
            return None

        # Parse X, Y, and Z speeds
        x_speed_raw = ((data[0] << 8) | data[1])  # X speed raw data
        z_speed_raw = ((data[4] << 8) | data[5])  # Raw data of Z velocity

        # Convert the raw data to floating-point numbers, considering both positive and negative values
        if x_speed_raw & 0x8000:  # If the highest bit is 1, it means a negative number
            x_speed_raw = -((65536 - x_speed_raw) & 0xFFFF)  # Convert to negative number using two's complement

        if z_speed_raw & 0x8000:  # If the highest bit is 1, it means a negative number
            z_speed_raw = -((65536 - z_speed_raw) & 0xFFFF)  # Convert to negative number using two's complement

        # Convert units to m/s and rad/s
        self.x_speed = x_speed_raw / 1000.0  # X speed unit is m/s
        self.y_speed = 0  # Y speed is 0
        self.z_speed = z_speed_raw / 1000.0  # Z speed unit is rad/s

        self.which_mode = data[2]

        self.infrared = data[6]  # Infrared data
        self.raw_current = data[7]  # Current data

        if self.raw_current > 32767:  # Unsigned number greater than 32767 means negative value (max value is 65535)
            # Convert to negative number using two's complement
            self.actual_current = -(65536 - self.raw_current) * 30.0
        else:
            # Positive number directly convert
            self.actual_current = self.raw_current * 30.0

        # Process infrared data
        self.infrared_bits = [(self.infrared >> (7 - i)) & 0x01 for i in range(8)]

        # Print or process data
        print(f"X Speed: {self.x_speed:.3f}, Y Speed: {self.y_speed}, Z Speed: {self.z_speed:.3f}, "
              f"Actual Current: {self.actual_current:.3f} mA, Infrared: {self.infrared}")

        # Print or process infrared bit information
        print(f"L_A: {self.infrared_bits[2]}, L_B: {self.infrared_bits[3]}, R_B: {self.infrared_bits[4]}, "
              f"R_A: {self.infrared_bits[5]}, infrared_flag : {self.infrared_bits[6]}, Charging flag: {self.infrared_bits[7]}")

    def read_serial_data(self):
        """Read serial data and parse"""
        while not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)   # Read a byte

                if len(self.buffer) < 2:
                    self.buffer.extend(byte)  
                    if len(self.buffer) == 2:
                        # If the frame header is 0x41 0x54, indicating an AT frame header, then start receiving data.
                        if self.buffer[0] != 0x41 or self.buffer[1] != 0x54:
                            # If it is not a valid frame header, clear the buffer and jump to the next loop.
                            self.buffer.clear()
                            continue  # Continue waiting for the next byte                 

                else:
                    self.buffer.extend(byte)

                    # Print or process the buffer content (for debugging)
                    # print("Buffer content:", ' '.join(f'{b:02x}' for b in self.buffer)) # debug

                    # If the buffer byte length is greater than or equal to 17 bytes (data frame length)
                    if len(self.buffer) >= 17:

                        # print("Received Frame (Hex):", ' '.join(f'{byte:02x}' for byte in self.buffer)) # debug

                        # Parse frame header, CAN frame ID, format, type, and data
                        # at_frame_header = self.buffer[0:2]  # AT frame header
                        can_frame_id = self.can_id_check(self.buffer[2:4])  # CAN standard frame ID
                        # can_frame_format = self.buffer[4]  # CAN frame format (0, standard frame; 1, extended frame)
                        # can_frame_type = self.buffer[5]  # CAN frame type (0, data frame; 1, remote frame)
                        data_length = self.buffer[6]  # Data length
                        data = self.buffer[7:15]  # Data frame

                        # print(f"Frame ID: 0x{can_frame_id:X}")    # debug

                        if can_frame_id == 0x182 and data_length == 0x08: # Determine based on the CAN frame ID and data length
                            # If the frame ID is 0x182 and the data length is 0x08, then it is a valid data frame.
                            # Assign data to variables
                            self.parse_can_data(data)

                            # Clear the buffer, ready for the next frame of data
                            self.buffer.clear()
                            return self.x_speed, self.z_speed, self.which_mode, self.infrared_bits   # 返回解析后的数据
                        else:
                            # If it is not a valid data frame, clear the buffer and jump to the next loop.
                            self.buffer.clear()

    def read_serial_response(self):
        """Read serial response data until '\r\n' is received or timeout"""
        response = bytearray()  # Use bytearray to store raw byte stream
        while True:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                response += byte

            # Check if '\r\n' is in the response, indicating the end of the frame
            if b'\r\n' in response:
                break

            # Timeout mechanism to prevent infinite loops
            if len(response) > 100:
                break

        return bytes(response)  # Return raw byte stream (bytes)

    def send_at_commands(self, commands):
        """Send AT commands and wait for responses"""
        for command in commands:
            retries = 0
            while retries < self.max_retries:
                self.ser.write(command.encode() + b'\r\n')
                print(f"Send command: {command}")

                # Wait for response and read data
                response = self.read_serial_response()

                # Check if "OK" is in the response
                if b"OK" in response:
                    print(f"Received response: {response}")
                    break  # If OK is received, exit the retry loop
                else:
                    retries += 1
                    print(f"Did not receive expected response, received: {response}")

            if retries == self.max_retries:
                print(f"After {self.max_retries} retries, no valid response was received. Please check the device.")
                break

    def start(self):
        """Start reading and processing data"""
        self.open_serial()
        try:
            # Send AT commands to enter AT command mode from transparent mode
            self.send_at_commands(["AT+CG", "AT+AT"])

            # Start reading data
            self.read_serial_data()

        except KeyboardInterrupt:
            print("Manually interrupted the program.")
        finally:
            self.close_serial()

if __name__ == '__main__':
    parser = SerialCANParser('/dev/ttyUSB0', 9600, 1)
    parser.start()