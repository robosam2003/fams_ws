
def convert_ros_message_to_json(self, msg):
    # Convert ROS message to JSON format
    json_data = {
        "job_id": msg.job_id,
        "priority": msg.priority,
        "subprocesses": [],
        "subprocess_start_times": [],
        "subprocess_end_times": [],
        "status": msg.status
    }
    for sub_process in msg.subprocesses:
        json_data["subprocesses"].append({
            "sub_process_id": sub_process.sub_process_id,
            "operation_type": sub_process.operation_type
        })
    for start_time in msg.subprocess_start_times:
        json_data["subprocess_start_times"].append(start_time)
    for end_time in msg.subprocess_end_times:
        json_data["subprocess_end_times"].append(end_time)
    
    # Convert JSON data to string
    json_string = json.dumps(json_data, indent=4)
    
    return json_string

def convert_json_to_ros_message(self, json_string):
    # Convert JSON string to JSON data
    json_data = json.loads(json_string)
    # Convert JSON data to ROS message
    msg = JobMessage()
    msg.job_id = json_data["job_id"]
    msg.priority = json_data["priority"]
    msg.status = json_data["status"]
    for sub_process in json_data["subprocesses"]:
        sub = SubProcess()
        sub.sub_process_id = sub_process["sub_process_id"]
        sub.operation_type = sub_process["operation_type"]
        msg.subprocesses.append(sub)
    for start_time in json_data["subprocess_start_times"]:
        msg.subprocess_start_times.append(start_time)
    for end_time in json_data["subprocess_end_times"]:
        msg.subprocess_end_times.append(end_time)
    return msg        




# class Job: # TODO: Move Job class to utils package?
#     def __init__(self):
#         self.job_id = None
#         self.priority = None
#         self.part_id = None
#         self.num_subprocesses = None
#         self.subprocesses = []
#         self.subprocess_start_times = []
#         self.subprocess_end_times = []
#         self.job_start_time = None
#         self.job_end_time = None
#         self.status = None
    
#     # def from_msg(self, msg):
#         # self.job_id = msg.job_id
#         # self.priority = msg.priority
#         # self.part_id = msg.part_id
#         # for sub_process in msg.subprocesses:
#         #     s = SubProcess()
#         #     s.sub_process_id = sub_process.sub_process_id
#         #     s.operation_type = sub_process.operation_type
#         #     self.subprocesses.append(s)
#         # self.subprocess_start_times = msg.subprocess_start_times
#         # self.subprocess_end_times = msg.subprocess_end_times
#         # self.job_start_time = msg.job_start_time
#         # self.job_end_time = msg.job_end_time
#         # self.status = msg.status

#     # def from_csv(self, csv_string): # TODO: Update when message structure is finalised
#         # # Parse the CSV string and set the attributes of the Job object
#         # l = csv_string.split(',')
#         # self.job_id = int(l[0])
#         # self.priority = int(l[1])
#         # self.part_id = int(l[2])
        
#         # self.num_subprocesses = (len(l[3:]) - 3) // 2 # There are two parameters for each subprocess
#         # for i in range(self.num_subprocesses):
#         #     sub_process = SubProcess()
#         #     sub_process.sub_process_id = int(l[3+2*i])
#         #     sub_process.operation_type = l[3+2*i+1] # Order is sub_process_id, operation_type, sub_process_id, operation_type, ...
#         #     self.subprocesses.append(sub_process)
#         # self.subprocess_start_times = l[3+self.num_subprocesses*2:3+3*self.num_subprocesses]
#         # self.subprocess_end_times = l[3+3*self.num_subprocesses:3+4*self.num_subprocesses]
#         # self.job_start_time = l[-3]
#         # self.job_end_time = l[-2]
#         # self.status = l[-1]



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

    def set_position_OLD(self, pos):
        id = self.joint_id*16  # (it's in hex)
        vel = 0x80  # 127 is zero     # TODO: I don't think this is included - see the example string. 
        # break the position into two bytes
        # pos = 32768 + int(pos_deg*int(32786/180))
        pos_low = pos & 0xff
        pos_high = (pos >> 8) & 0xff    
        timestamp = 0x51
        digital_output = 0x00 
        data = [self.SET_POS_COMMAND, pos_high, pos_low, timestamp, digital_output]
        self.can_bus.send(id, data)
        # print(f"Sent: {data}")
        print("Set Joint", self.joint_id, "to position: ", pos, "   Hex:  ", hex(pos_high), hex(pos_low))
