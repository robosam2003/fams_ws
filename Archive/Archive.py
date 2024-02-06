
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