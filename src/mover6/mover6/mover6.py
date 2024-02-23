import time
import can


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
        self.velocity = self.data[1]
        self.position = self.data[2] + (self.data[3] << 8) - 32768
        self.shunt = self.data[4]
        self.timestamp = self.data[5]
        self.div_value = self.data[6]
        self.digital_input = self.data[7]

 
    def __str__(self):
        data = [hex(i) for i in self.data]
        data_str = " ".join(data)
        return f"ID: {hex(self.id)}, Velocity: \033[34m{self.velocity}\033[0m, Position: \033[32m{self.position}\033[0m, Data: {data_str}"


class mover6():
    # These are the CANv1 commands (16 bit)
    SET_POS_COMMAND = 0x04
    SET_VEL_COMMAND = 0x05

    def __init__(self):
        self.bitrate = 500000
        self.interface = "can0"
        self.bus_type = "socketcan"
        self.bus = can.interface.Bus(channel=self.interface, bustype=self.bus_type, bitrate=self.bitrate)

        self.joint_positions = [32768]*6
        # self.enable_all_axes()
        self.main_loop()


    def main_loop(self):
        max_joint = 1
        # 2. Start the main loop and send position command cyclically 
        for joint in range(1, max_joint+1):
            self.joint_to_position(joint, self.joint_positions[joint-1])
            rec_msg = self.recv()
            # Parse the message and update the joint position
            joint_pos = rec_msg.position
            time.sleep(2/1000) # When 
            self.joint_positions[joint-1] = joint_pos

        # 3. Reset all joints: 
        for joint in range(1, max_joint+1):
            self.reset_axis(joint)
            time.sleep(2/1000)
        # 4. Enable all joints
        for joint in range(1, max_joint+1):
            self.enable_axis(joint)
            time.sleep(2/1000)
        
        while True:
            for joint in range(1, max_joint+1):
                # for all the joints, slowly move to the zero position
                angle_command = 32768
                if self.joint_positions[joint-1] > 32768:
                    angle_command = self.joint_positions[joint-1] - 50
                elif self.joint_positions[joint-1] < 32768:
                    angle_command = self.joint_positions[joint-1] + 50
            
                # angle_command = self.joint_positions[joint-1]
                self.joint_to_position(joint, angle_command)
                rec_msg = self.recv()
                # Parse the message and update the joint position
                joint_pos = rec_msg.position
                time.sleep(2/1000) # When 
                self.joint_positions[joint-1] = joint_pos
            time.sleep(0.05) # 20Hz


    def joint_to_position(self, joint_no, pos_deg):
        id = joint_no*16  # (it's in hex)
        vel = 130  # 127 is zero
        # break the position into two bytes
        pos = 32768 + int(pos_deg*int(32786/180))
        pos_low = pos & 0xff
        pos_high = (pos >> 8) & 0xff       
        timestamp = 0x51
        digital_output = 0x00 
        data = [self.SET_POS_COMMAND, vel, pos_high, pos_low, timestamp, digital_output]
        self.send(id, data)
        # print(f"Sent: {data}")
        print("Set Joint", joint_no, "to position: ", pos_deg)






    
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