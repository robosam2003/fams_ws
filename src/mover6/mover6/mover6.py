import time
import can


def _get_message(msg):
    print("Received: ", msg)
    return msg

ERROR_DICT = {
    0x01: "Brown Out or Watch Dog",
    0x02: "VELOCITY LAG",
    0x04: "MOTOR NOT ENABLED",
    0x08: "COMM WATCHDOG",
    0x10: "POSITION LAG",
    0x20: "ENCODER ERROR",
    0x30: "OVER CURRENT",
    0x40: "CAN ERROR",
}


class RecMsg():
    def __init__(self, msg):
        self.msg = msg
        self.id = msg.arbitration_id
        self.data = msg.data
        self.error_code = self.data[0]
        self.error = ""
        for key in ERROR_DICT:
            if self.error_code & key:
                self.error = self.error + "\n" + ERROR_DICT[key]
        self.velocity = self.data[1]
        self.position = self.data[2] + (self.data[3] << 8) - 32768
        self.shunt = self.data[4]
        self.timestamp = self.data[5]
        self.div_value = self.data[6]
        self.digital_input = self.data[7]

    def __str__(self):
        return f"ID: {self.id}, Velocity: {self.velocity}, Position: {self.position}"


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
        # 2. Start the main loop and send position command cyclically 
        while True:
            for joint in range(1, 7):
                self.joint_to_position(joint, self.joint_positions[joint-1])
                rec_msg = self.recv()
                # Parse the message and update the joint position
                joint_pos = rec_msg.position


                time.sleep(2/1000) # When 


    def joint_to_position(self, joint_no, pos_deg):
        id = joint_no*16  # (it's in hex)
        vel = 100  # 127 is zero
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

    def joint_to_velocity(self, joint_no, vel):
        id = joint_no*16
        # break the velocity into two bytes



    
    def send(self, id, data):
        message = can.Message(arbitration_id=id, data=data, is_extended_id=False)
        self.bus.send(message)

    def recv(self):
        rec_msg = RecMsg(self.bus.recv())

        if rec_msg.error != "":
            print(f"\033[91mError: {rec_msg.error}\033[0m")
        print(f"Received: {rec_msg}")
        return rec_msg
    
    def enable_axis(self, axis_no):
        id = axis_no*16 # (it's in hex)
        data = [0x01, 0x09]
        self.send(id, data)

    def enable_all_axes(self):
        for i in range(1, 7):
            self.enable_axis(i)
            print("Enabled axis", i)
            time.sleep(0.01)

    def shutdown(self):
        self.bus.shutdown()

    def __del__(self):
        self.bus.shutdown()

mover6()