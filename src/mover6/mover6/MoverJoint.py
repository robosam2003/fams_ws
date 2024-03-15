import struct
import PCanBus
import time

class MoverJoint():
    # Constants
    SET_POS_COMMAND = 0x04
    SET_VEL_COMMAND = 0x05
    SET_POS_COMMAND_32 = 0x14
    SET_VEL_COMMAND_32 = 0x15

    def __init__(self, joint_id, can_bus: PCanBus, tics_per_degree, min_pos_deg, max_pos_deg) -> None:
        self.joint_id = int(joint_id)
        self.can_bus = can_bus
        self.tics_per_degree = tics_per_degree
        self.current_position = 0
        self.current_position_deg = 0
        #self.position_deg = 0
        
        self.min_pos_deg = min_pos_deg
        self.max_pos_deg = max_pos_deg
    
    def update_position(self, rec_msg):
        self.current_position = rec_msg.position
        self.current_position_deg = float(self.current_position) / self.tics_per_degree
        return self.current_position_deg
    
    def set_position_old(self, pos_deg):  # This is the old, 16 bit implementation
        # Saturation of joint angle.
        # if pos_deg < self.min_pos_deg:
        #     pos_deg = self.min_pos_deg
        # if pos_deg > self.max_pos_deg:
        #     pos_deg = self.max_pos_deg
        # The id is the joint number times 16 (it's in hex)
        id = self.joint_id*16
        # Convert the position to tics
        tics = int(pos_deg)
        print("Tics: ", tics)
        # Split the tics into 2 - it is an unsigned short (16 bit)
        packed_tics = struct.pack('>H', tics) # the '>' means big-endian, the 'H' means unsigned short
        pos0, pos1 = struct.unpack('>BB', packed_tics)

        timestamp = 0x51
        velocity = 0x80
        digital_output = 0x00
        data = [self.SET_POS_COMMAND, velocity, pos0, pos1, timestamp, digital_output]
        self.can_bus.send(id, data)
        print("Set Joint (OLD)", self.joint_id, "to position: ", pos_deg, "degrees,  TICS: ", tics, ",  Hex:  ", hex(pos0), hex(pos1))
    
    def set_position(self, pos_deg):  # This is the new, CANv2 32 bit implementation
        # Saturation of joint angle.    
        if pos_deg < self.min_pos_deg:
            pos_deg = self.min_pos_deg
        if pos_deg > self.max_pos_deg:
            pos_deg = self.max_pos_deg
        # The id is the joint number times 16 (it's in hex)
        # if self.joint_id > 4:
        #     self.set_position_old(pos_deg)
            # return
        id = self.joint_id*16
        # Convert the position to tics
        tics = int(pos_deg*self.tics_per_degree)
        # Split the tics into 4
        packed_tics = struct.pack('>i', tics)
        pos0, pos1, pos2, pos3 = struct.unpack('>BBBB', packed_tics)
        
        timestamp = 0x51
        digital_output = 0x00
        velocity = 0x80
        data = [self.SET_POS_COMMAND_32, velocity, pos0, pos1, pos2, pos3, timestamp, digital_output]
        self.can_bus.send(id, data)
        print("Set Joint", self.joint_id, "to position: ", pos_deg, "degrees,  TICS: ", tics, ",  Hex:  ", hex(pos0), hex(pos1), hex(pos2), hex(pos3))

    def set_velocity(self, joint_no, vel): # TODO: This is the old implementation (16 bit)
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

    def set_pos_pid(self, k_p, k_i, k_d):
        id = self.joint_id*16 # (it's in hex)
        # Scale the PID values by 1000
        k_p = int(k_p*1000)
        k_i = int(k_i*1000)
        k_d = int(k_d*1000)

        # Each pid value is split into two bytes
        k_p_low = k_p & 0xff
        k_p_high = (k_p >> 8) & 0xff
        k_i_low = k_i & 0xff
        k_i_high = (k_i >> 8) & 0xff
        k_d_low = k_d & 0xff
        k_d_high = (k_d >> 8) & 0xff

        # Set the proportional gain
        data = [0x02, 0x40, k_p_high, k_p_low]
        self.can_bus.send(id, data)
        time.sleep(2/1000)
        print("Set the proportional gain")
        # Set the integral gain
        data = [0x02, 0x41, k_i_high, k_i_low]
        self.can_bus.send(id, data)
        time.sleep(2/1000)
        print("Set the integral gain")
        # Set the derivative gain
        data = [0x02, 0x42, k_d_high, k_d_low]
        self.can_bus.send(id, data)
        time.sleep(2/1000)
        print("Set the derivative gain")

    def set_vel_pid(self, k_p, k_i, k_d):
        id = self.joint_id*16
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
        self.can_bus.send(id, data)
        time.sleep(2/1000)
        print("Set the proportional gain")
        # set the integral gain
        data = [0x02, 0x45, k_i_high, k_i_low]
        self.can_bus.send(id, data)
        time.sleep(2/1000)
        print("Set the integral gain")
        # set the derivative gain
        data = [0x02, 0x46, k_d_high, k_d_low]
        self.can_bus.send(id, data)
        time.sleep(2/1000)
        print("Set the derivative gain")

    def set_tic_scale(self, tics_per_count):
        id = self.joint_id*16
        # Tics per count is one byte
        tics_per_count = tics_per_count & 0xff

        data = [0x02, 0x69, tics_per_count, 0x9E]
        self.can_bus.send(id, data)
        print("Set the tics per count to: ", tics_per_count)
    
    def set_max_lag(self, max_lag):
        id = self.joint_id*16 # (it's in hex)
        # max_lag is split into two bytes
        max_lag_low = max_lag & 0xff
        max_lag_high = (max_lag >> 8) & 0xff
        data = [0x02, 0x31, max_lag_high, max_lag_low] 
        self.can_bus.send(id, data)

    def set_zero_position(self):
        id = self.joint_id*16
        data = [0x01, 0x08, 0x00, 0x00]
        self.can_bus.send(id, data)
        time.sleep(2/1000)
        self.can_bus.send(id, data) # Have to send it twice to confirm
        print("Set the zero position")

    def set_max_current(self, max_current):
        id = self.joint_id*16
        max_current = max_current & 0xff # Max current is one byte
        data = [0x02, 0x32, max_current, 0x00]
        self.can_bus.send(id, data)
        print("Set the max current to: ", max_current)


    def reset(self):
        print("Resetting axis", self.joint_id, "...")
        id = self.joint_id*16
        data = [0x01, 0x06]
        self.can_bus.send(id, data)
        time.sleep(0.1)

    def enable(self):
        id = self.joint_id*16
        data = [0x01, 0x09]
        self.can_bus.send(id, data)
        print("Enabled axis", self.joint_id)

    