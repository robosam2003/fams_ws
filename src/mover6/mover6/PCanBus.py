import can


class PCanBus():
    def __init__(self, bitrate=500000, interface='can0', bus_type='socketcan') -> None:
        self.bitrate = bitrate
        self.interface = interface
        self.bus_type = bus_type
        self.bus = can.interface.Bus(channel=self.interface, bustype=self.bus_type, bitrate=self.bitrate)

    def send(self, id, data):
        message = can.Message(arbitration_id=id, data=data, is_extended_id=False)
        self.bus.send(message)

    def recv(self, vervose=True):
        msg = self.bus.recv()
        rec_msg = RecMsg32(msg)
        if vervose:
            if rec_msg.error != "":
                print(f"\033[91mError: {rec_msg.error}\033[0m")
            print(f"Received: {rec_msg}")
        return rec_msg
    

class RecMsg32():
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
        self.pos0 = self.data[1]
        self.pos1 = self.data[2]
        self.pos2 = self.data[3]
        self.pos3 = self.data[4]
        # 32 bit, signed position 
        self.position = (self.pos0 << 24) + (self.pos1 << 16) + (self.pos2 << 8) + self.pos3
        # Interpret the position as a signed 32 bit integer
        if self.position > 0x7fffffff:
            self.position = -0x100000000 + self.position
        # self.position_deg = int(self.position)
        # self.velocity = self.data[5]
        self.timestamp = self.data[5]
        self.shunt = self.data[6]
        # self.div_value = self.data[6]
        self.digital_output = self.data[7]

    def __str__(self):
        data = [hex(i) for i in self.data]
        data_str = " ".join(data)
        return f"ID: {hex(self.id)}, \033[0m, Position: ({self.position}) ({hex(self.pos0)} {hex(self.pos1)} {hex(self.pos2)} {hex(self.pos3)}) \033[0m, Data: {data_str}"



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
