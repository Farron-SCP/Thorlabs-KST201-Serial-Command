#!/usr/bin/python
# Python wrapper to communicate with Thorlabs KST201 controllers

import serial
import time

# KST Class for KST201
class KST:
    DEBUG = True
    
    def __init__(self):
        # Initialize the serial connection
        self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, bytesize=8,
                                 parity=serial.PARITY_NONE, stopbits=1,
                                 xonxoff=0, rtscts=0, timeout=1)
        if self.DEBUG:
            print(f"Serial port opened: {self.ser.is_open}")
        
        self.ser.flushInput()
        self.ser.flushOutput()

    def __str__(self):
        return "Instance of a KST controller."

    # Commands dictionary for KST201
    cmds = {
        "identify": bytearray([0x23, 0x02, 0x00, 0x00, 0x50, 0x01]),  # MGMSG_MOD_IDENTIFY
        "set_chan_enable": bytearray([0x10, 0x02, 0x01, 0x01, 0x50, 0x01]),  # MGMSG_MOD_SET_CHANENABLESTATE
        "move_home": bytearray([0x43, 0x04, 0x01, 0x00, 0x50, 0x01]),  # MGMSG_MOT_MOVE_HOME
        "move_absolute": bytearray([0x53, 0x04, 0x06, 0x00, 0xD0, 0x01, 0x01, 0x00, 0xFB, 0x86, 0x00, 0x00]),  # MGMSG_MOT_MOVE_ABSOLUTE 1 mm
        "get_position": bytearray([0x11, 0x04, 0x01, 0x00, 0x50, 0x01]),  # Get position command
        "move_jog_forward": bytearray([0x6A, 0x04, 0x01, 0x01, 0x50, 0x01]),  # Jog forward
        "move_jog_backward": bytearray([0x6A, 0x04, 0x01, 0x02, 0x50, 0x01]),  # Jog backward
        "get_jog_params" : bytearray([0x17, 0x04, 0x01, 0x00, 0x50, 0x01]), # Request jog params
    }

    def send_command(self, cmd):
        if not self.ser.is_open:
            print("Serial port not open.")
            return
        
        if self.DEBUG:
            print(f"Sending command: {cmd.hex()}")
        
        self.ser.write(cmd)
        self.ser.flushInput()
        self.ser.flushOutput()
        
    def wait_status(self, expected_reply):
        start = time.time()
        accumulated_reply = ''
        
        while True:
            # Accumulate the response
            reply = self.recv_reply().strip()
            accumulated_reply += reply
            # Check if the expected reply is in the accumulated response
            if expected_reply in accumulated_reply:
                break

            time.sleep(0.1)  # Wait briefly before checking again
        
        time_waited = time.time() - start
        print(f'Time waited: {time_waited:.4f} seconds')
        return True  # Indicate that the expected reply was received

    def identify(self):
        self.send_command(self.cmds["identify"])
        time.sleep(1)
    
    def enable_channel(self):
        self.send_command(self.cmds["set_chan_enable"])
        time.sleep(1)

    def move_home(self):
        self.send_command(self.cmds["move_home"])
        time.sleep(1)
        self.wait_status('44 04 01 00 01 50')
        print('Homing Complete')

    def move_jog_f(self):
        self.send_command(self.cmds["move_jog_forward"])
        self.move_complete()
        time.sleep(0.1)

    def move_jog_b(self):
        self.send_command(self.cmds["move_jog_backward"])
        time.sleep(0.1)
        
        
    def move_complete(self):
        self.wait_status('64 04 0e 00 81 50')
        print('Move completed')
        
    # Function to convert distance in mm to hex format
    def convert_distance(self,distance_mm):
        movement_per_mm_microsteps = 2184533.33
        dist_microsteps = int(distance_mm * movement_per_mm_microsteps)
        dist_microsteps_bytes = dist_microsteps.to_bytes(4, byteorder='little', signed=True)
        print(f"Distance: {dist_microsteps_bytes.hex()}")
        return dist_microsteps_bytes

    def convert_acceleration(self,acc_mm_per_sec2):
        movement_per_sec2_microsteps = 24111.85
        acc_microsteps = int(acc_mm_per_sec2 * movement_per_sec2_microsteps)
        acc_microsteps_bytes = acc_microsteps.to_bytes(4, byteorder='little', signed=True)
        print(f"Acceleration: {acc_microsteps_bytes.hex()}")
        return acc_microsteps_bytes

    def convert_speed(self,speed_mm_per_sec):
        movement_per_sec_microsteps = 117265749.2
        speed_microsteps = int(speed_mm_per_sec * movement_per_sec_microsteps)
        speed_microsteps_bytes = speed_microsteps.to_bytes(4, byteorder='little', signed=True)
        print(f"Speed: {speed_microsteps_bytes.hex()}")
        return speed_microsteps_bytes
    

    def convert_dist_enccnt(self, enccnt_hex):
        # Extract the last 4 bytes
        data_bytes_hex = enccnt_hex[-8:]  # Last 4 bytes in hex (8 characters)
        
        # Reverse the bytes (little-endian)
        reversed_bytes_hex = ''.join([data_bytes_hex[i:i+2] for i in range(0, len(data_bytes_hex), 2)][::-1])
        
        # Convert the reversed hex to a decimal integer
        enccnt_int = int(reversed_bytes_hex, 16)
        
        # Conversion factor (steps/mm)
        conversion_factor = 2184533.33
        
        # Convert steps to millimeters
        distance_mm = enccnt_int / conversion_factor
        
        if self.DEBUG:
            print(f"Converted {reversed_bytes_hex} hex to {distance_mm:.6f} mm")
        
        return round(distance_mm, 6)

    def req_jogparams(self):
        fixed_cmd = bytearray([0x17, 0x04, 0x00, 0x00, 0x50, 0x01])
        
        if self.DEBUG:
            print(f"Full jog command: {fixed_cmd.hex()}")
            
        self.send_command(fixed_cmd)
        
        time.sleep(0.2)
        sys_reply = self.recv_reply()
        # Step 1: Split the string into a list of bytes
        sys_reply_bytes = sys_reply.split()

        # Step 2: Slice off the first 5 bytes (elements)
        remaining_bytes = sys_reply_bytes[6:]

        # Step 3: Join the remaining bytes back into a string without spaces
        jog_params = ''.join(remaining_bytes)
        if self.DEBUG:
            print(f"Jog parameters received: {jog_params}")
            
    # Function to set jog parameters
    def set_jog(self, size_mm, acc_mm, vel_mm):
        # Base command for setting jog parameters as a bytearray
        fixed_cmd = bytearray([0x16, 0x04, 0x16, 0x00, 0xD0, 0x01, 0x01, 0x1c])

        # Set jog mode and stop mode as fixed values
        jog_mode_hex = bytearray([0x02, 0x00]) #01 for continuous, 02 for single use
        stop_mode_hex = bytearray([0x01, 0x00])

        # Convert the jog parameters to the required bytearray format
        size_hex = self.convert_distance(size_mm)
        min_velocity_hex = bytearray([0x00, 0x00, 0x00, 0x00])  # Assuming min velocity is zero
        acc_hex = self.convert_acceleration(acc_mm)
        vel_hex = self.convert_speed(vel_mm)

        # Construct the full command by concatenating all parts
        full_cmd = fixed_cmd + jog_mode_hex + size_hex + min_velocity_hex + acc_hex + vel_hex + stop_mode_hex
        
        # Print the full command for debugging
        if self.DEBUG:
            print(f"Full jog command: {full_cmd.hex()}")
        
        # Send the command
        self.send_command(full_cmd)


    # Function to get the current position of the stage
    # Function to get the current position of the stage
    def get_position(self):
        self.send_command(self.cmds["get_position"])
        reply = self.recv_reply()

        if not reply:
            return None
        
        # Extract the 4-byte position information (bytes 8 to 11 in the response)
        position_mm = self.convert_dist_enccnt(reply)
        
        if self.DEBUG:
            print(f"Position in mm: {position_mm}")
        
        return position_mm

    def recv_reply(self):
        time.sleep(0.04)  # necessary delay
        reply = ''
        while self.ser.in_waiting > 0:
            reply += self.ser.read().hex()
            reply += ' '
        if self.DEBUG:
            print(f"Reply: {reply}")
        return reply
    
    def close(self):
        self.ser.close()
        if self.DEBUG:
            print(f"Serial port closed: {self.ser.is_open}")

# Example usage
def example_usage():
    kst = KST()  # Serial connection is established in the constructor
    
    kst.identify()
    kst.enable_channel()
    kst.move_home()
    
    # Get jog parameters from user input
    size_mm = float(input("Enter jog size in mm (e.g., 1.0): "))
    acc_mm = float(input("Enter jog acceleration in mm/s² (e.g., 2.0): "))
    vel_mm = float(input("Enter jog velocity in mm/s (e.g., 3.0): "))

    
    kst.req_jogparams()
    # Set jog parameters
    kst.set_jog(size_mm, acc_mm, vel_mm)
    kst.req_jogparams()
        
    jog_count = 2  # Number of jog operations
    command = input("Press 1 for jog forward, 2 for jog backward: ").strip().lower()

    if command not in ["1", "2"]:
        print("Invalid input. Please try again.")
        return

    # Start the timer
    start_time = time.time()

    for i in range(jog_count):
        if command == "1":
            kst.move_jog_f()
        elif command == "2":
            kst.move_jog_b()

    # Stop the timer
    end_time = time.time()
    
    total_time = end_time - start_time
    print(f"Total time for {jog_count} jogs: {total_time:.6f} seconds")
    kst.close()

if __name__ == "__main__":
    example_usage()


