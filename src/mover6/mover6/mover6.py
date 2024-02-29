from audioop import error
import time
import can
import struct
# import keyboard


"""
Initialisation Commands:
sudo modprobe peak_usb
sudo modprobe peak_pci
sudo ip link set can0 up type can bitrate 500000

"""

def _get_message(msg):
    print("Received: ", msg)
    return msg

ERROR_DICT = {
    0x01: "Brown Out or Watch Dog",  # 0b00000001
    0x02: "VELOCITY LAG", # 0b00000010
    0x04: "MOTOR NOT ENABLED", # 0b00000100
    0x08: "COMM WATCHDOG", # 0b00001000
    0x10: "POSITION LAG", # 0b00010000
    0x20: "ENCODER ERROR",   # 0b00100000
    0x40: "OVER CURRENT", # 0b01000000
    0x80: "CAN ERROR", # 0b10000000
}


class RecMsg():
    def __init__(self, msg):
        self.msg = msg
        self.id = msg.arbitration_id
        self.data = msg.data
        self.error_code = self.data[0]
        self.error = ""
        
        # For each error code, one of the 8 bits is set.
        for key in ERROR_DICT:
            if self.error_code & key:
                self.error += "\n" + ERROR_DICT[key]
        # self.velocity = self.data[1]
        self.pos0 = self.data[1]
        self.pos1 = self.data[2]
        self.pos2 = self.data[3]
        self.pos3 = self.data[4]
        # 32 bit, signed position 
        self.position = (self.pos0 << 24) + (self.pos1 << 16) + (self.pos2 << 8) + self.pos3
        # Interpret the position as a signed 32 bit integer
        if self.position > 0x7fffffff:
            self.position = -0x100000000 + self.position
        self.position_deg = int(self.position)
        # self.velocity = self.data[5]
        self.timestamp = self.data[5]
        self.shunt = self.data[6]
        # self.div_value = self.data[6]
        self.digital_output = self.data[7]

 
    def __str__(self):
        data = [hex(i) for i in self.data]
        data_str = " ".join(data)
        return f"ID: {hex(self.id)}, \033[0m, Position: \033[32m{self.position_deg} ({self.position}) ({hex(self.pos0)} {hex(self.pos1)} {hex(self.pos2)} {hex(self.pos3)}) \033[0m, Data: {data_str}"

class mover6():
    # These are the CANv1 commands (16 bit)
    SET_POS_COMMAND = 0x04
    SET_VEL_COMMAND = 0x05
    SET_POS_COMMAND_32 = 0x14
    SET_VEL_COMMAND_32 = 0x15

    MAX_LAG = 0  # Disabled

    tics_per_degree = [-65.87,
                        -65.87,
                        65.87,
                        -69.71,
                        3.2,
                        3.2]
                        

    def __init__(self):
        self.bitrate = 500000
        self.interface = "can0"
        self.bus_type = "socketcan"
        self.bus = can.interface.Bus(channel=self.interface, bustype=self.bus_type, bitrate=self.bitrate)

        self.joint_positions = [0]*6
        # self.enable_all_axes()
        self.main_loop()


    def main_loop(self):


        max_joint = 1
        # 2. Start the main loop and send position command cyclically 
        for joint in range(1, max_joint+1):
            # Set the max lag
            self.set_position(joint, self.joint_positions[joint-1])
            rec_msg = self.recv()

            # Parse the message and update the joint position
            joint_pos = rec_msg.position
            time.sleep(2/1000) # When 
            self.joint_positions[joint-1] = joint_pos

        # 3. Reset all joints: 
        for joint in range(1, max_joint+1):
            self.reset_axis(joint)
            time.sleep(2/1000)

        # 5. Set the parameters
        for joint in range(1, max_joint+1):
            # self.set_max_lag(joint, self.MAX_LAG)
            self.set_pos_pid(joint, 0.1, 0.1, 0)
            time.sleep(2/1000)
            self.set_vel_pid(joint, 0.5, 0, 0)
            time.sleep(2/1000)
            self.set_tic_scale(joint, 1)
            time.sleep(2/1000)
            # self.set_zero_position(joint)
            time.sleep(2/1000)
        time.sleep(0.5)
            
        # 4. Enable all joints

        for joint in range(1, max_joint+1):
            self.enable_axis(joint)
            time.sleep(2/1000)
        
        # Read the keyboard input using the keyboard library


        
        i = 0
        
        reference = -45 # This is in degrees
        tick_tock = -1
        # reference = int(reference / (122146/-20000))
        position_command = reference
        while True:          
            # if i % 100 == 0:
            #     if abs(reference) > 10000:
            #         tick_tock *= -1
            #     reference += 1000*tick_tock
            
            for joint in range(1, max_joint+1):
                # for all the joints, slowly move to the zero position

                self.set_position_32bit(joint, reference)
                rec_msg = self.recv()
                # # Parse the message and update the joint position
                joint_pos = rec_msg.position
                self.joint_positions[joint-1] = joint_pos
                print("Joint", joint, "is at position: ", joint_pos)
                # self.joint_positions[joint-1] = joint_pos
                # if rec_msg.error != "":
                #     # reset the axis
                #     self.reset_axis(joint)

                time.sleep(2/1000) 
            i += 1
            time.sleep(1/20) # 20Hz

    def set_position_using_velocity(self, joint_no, pos_deg):
        # A loop that does Proportional control to move to the desired position, using velocity control
        # 1. Get the current position
        # 2. Calculate the error
        # 3. Calculate the velocity command
        # 4. Send the velocity command
        vel = 0


        while self.joint_positions[joint_no-1] - pos_deg > 10:
            # 1. Get the current position and send the velocity command
            self.set_velocity(joint_no, vel)
            rec_msg = self.recv()
            current_pos = rec_msg.position
            self.joint_positions[joint_no-1] = current_pos
            error = pos_deg - current_pos # Error.
            # 2. Calculate the velocity command
            vel = 0.01*error # Kp = 0.01

    def set_position_32bit(self, joint_no, pos_deg):
        id = joint_no*16
        # break the position into four bytes - big endian
        
        tics = int(pos_deg*self.tics_per_degree[joint_no-1])
        pos0, pos1, pos2, pos3 = struct.pack('>i', tics)

        timestamp = 0x51
        digital_output = 0x00
        data = [self.SET_POS_COMMAND_32, pos0, pos1, pos2, pos3, timestamp, digital_output]
        self.send(id, data)
        print("Set Joint", joint_no, "to position: ", pos_deg, "degrees     Hex:  ", hex(pos0), hex(pos1), hex(pos2), hex(pos3))

    def set_position(self, joint_no, pos):
        id = joint_no*16  # (it's in hex)
        vel = 0x80  # 127 is zero     # TODO: I don't think this is included - see the example string. 
        # break the position into two bytes
        # pos = 32768 + int(pos_deg*int(32786/180))
        pos_low = pos & 0xff
        pos_high = (pos >> 8) & 0xff    
        timestamp = 0x51
        digital_output = 0x00 
        data = [self.SET_POS_COMMAND, pos_high, pos_low, timestamp, digital_output]
        self.send(id, data)
        # print(f"Sent: {data}")
        print("Set Joint", joint_no, "to position: ", pos, "   Hex:  ", hex(pos_high), hex(pos_low))

    def set_velocity(self, joint_no, vel): # vel is in the range -127 to 127
        id = joint_no*16  # (it's in hex)
         #velocity is 0-255 (127 is zero)
        if vel < -127: # Saturation
            vel = -127
        if vel > 127:
            vel = 127
        vel = int(vel) + 127

        timestamp = 0x52
        data = [self.SET_VEL_COMMAND, vel, timestamp]
        self.send(id, data)
        print("Set Joint", joint_no, "to velocity: ", vel)
        
    def set_max_lag(self, joint_no, max_lag):
        id = joint_no*16 # (it's in hex)
        # max_lag is split into two bytes
        max_lag_low = max_lag & 0xff
        max_lag_high = (max_lag >> 8) & 0xff
        data = [0x02, 0x31, max_lag_high, max_lag_low] 
        self.send(id, data)

    def set_pos_pid(self, joint_no, k_p, k_i, k_d):
        id = joint_no*16
        # Split the PID values into two bytes
        # Scale the PID values by 1000
        k_p = int(k_p*1000)
        k_i = int(k_i*1000)
        k_d = int(k_d*1000)

        k_p_low = k_p & 0xff
        k_p_high = (k_p >> 8) & 0xff
        k_i_low = k_i & 0xff
        k_i_high = (k_i >> 8) & 0xff
        k_d_low = k_d & 0xff
        k_d_high = (k_d >> 8) & 0xff

        # set the proportional gain
        data = [0x02, 0x40, k_p_high, k_p_low]
        self.send(id, data)
        time.sleep(2/1000)
        print("Set the proportional gain")
        # set the integral gain
        data = [0x02, 0x41, k_i_high, k_i_low]
        self.send(id, data)
        time.sleep(2/1000)
        print("Set the integral gain")
        # set the derivative gain
        data = [0x02, 0x42, k_d_high, k_d_low]
        self.send(id, data)
        time.sleep(2/1000)
        print("Set the derivative gain")

    def set_vel_pid(self, joint_no, k_p, k_i, k_d):
        id = joint_no*16
        # Split the PID values into two bytes
        # Scale the PID values by 1000
        k_p = int(k_p*1000)
        k_i = int(k_i*1000)
        k_d = int(k_d*1000)

        k_p_low = k_p & 0xff
        k_p_high = (k_p >> 8) & 0xff
        k_i_low = k_i & 0xff
        k_i_high = (k_i >> 8) & 0xff
        k_d_low = k_d & 0xff
        k_d_high = (k_d >> 8) & 0xff

        # set the proportional gain
        data = [0x02, 0x44, k_p_high, k_p_low]
        self.send(id, data)
        time.sleep(2/1000)
        print("Set the proportional gain")
        # set the integral gain
        data = [0x02, 0x45, k_i_high, k_i_low]
        self.send(id, data)
        time.sleep(2/1000)
        print("Set the integral gain")
        # set the derivative gain
        data = [0x02, 0x46, k_d_high, k_d_low]
        self.send(id, data)
        time.sleep(2/1000)
        print("Set the derivative gain")

    def set_tic_scale(self, joint_no, tics_per_count):
        id = joint_no*16
        # Tics per count is one byte
        tics_per_count = tics_per_count & 0xff

        data = [0x02, 0x69, tics_per_count]
        self.send(id, data)
        print("Set the tics per count to: ", tics_per_count)

    def set_zero_position(self, joint_no):
        id = joint_no*16
        data = [0x01, 0x08, 0x00, 0x00]
        self.send(id, data)
        time.sleep(2/1000)
        self.send(id, data) # Have to send it twice to 
        print("Set the zero position")

    def send(self, id, data):
        message = can.Message(arbitration_id=id, data=data, is_extended_id=False)
        self.bus.send(message)

    def recv(self):
        msg = self.bus.recv()
        rec_msg = RecMsg(msg)

        if rec_msg.error != "":
            print(f"\033[91mError: {rec_msg.error}\033[0m")
        print(f"Received: {rec_msg}")
        return rec_msg
    
    def reset_axis(self, axis_no):
        print("Resetting axis", axis_no, "...")
        id = axis_no*16
        data = [0x01, 0x06]
        self.send(id, data)
        time.sleep(0.1)
        # Acknowledge the reset
        # rec_msg = self.recv()
    
    def enable_axis(self, axis_no):
        id = axis_no*16 # (it's in hex)
        data = [0x01, 0x09]
        self.send(id, data)
        print("Enabled axis", axis_no)


    def enable_all_axes(self):
        for i in range(1, 7):
            self.enable_axis(i)
            time.sleep(2/1000)

    def shutdown(self):
        self.bus.shutdown()

    def __del__(self):
        self.bus.shutdown()

mover6()