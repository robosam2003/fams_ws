import rclpy
from rclpy.node import Node
from fams_interfaces.msg import JobMessage, SubProcess, Part, SystemState
from rosidl_runtime_py import *



class Scheduler(Node):
    def __init__(self):
        super().__init__('scheduler')
        self.job_log_path = "/home/robosam/fams_ws/src/scheduler/scheduler/JobLog.csv"
        self.get_logger().info('Scheduler node has been started')

        self.job_subscriber = self.create_subscription(
            JobMessage,
            'job', # Topic name
            self.job_message_callback,
            10 # QoS profile
        )

    
    def job_message_callback(self, msg):
        self.save_job_to_log(msg)
        self.get_logger().info('Job message has been added to the job log')

    def save_job_to_log(self, msg):
        # Write to a job log csv file
        with open(self.job_log_path, 'a') as f:
            csv = message_to_csv(msg)
            f.write(csv)
            f.write('\n')  


class Job: # TODO: Move Job class to utils package?
    def __init__(self):
        self.job_id = None
        self.priority = None
        self.part_id = None
        self.num_subprocesses = None
        self.subprocesses = []
        self.subprocess_start_times = []
        self.subprocess_end_times = []
        self.job_start_time = None
        self.job_end_time = None
        self.status = None
    
    def from_msg(self, msg):
        self.job_id = msg.job_id
        self.priority = msg.priority
        self.part_id = msg.part_id
        for sub_process in msg.subprocesses:
            s = SubProcess()
            s.sub_process_id = sub_process.sub_process_id
            s.operation_type = sub_process.operation_type
            self.subprocesses.append(s)
        self.subprocess_start_times = msg.subprocess_start_times
        self.subprocess_end_times = msg.subprocess_end_times
        self.job_start_time = msg.job_start_time
        self.job_end_time = msg.job_end_time
        self.status = msg.status

    def from_csv(self, csv_string):
        # Parse the CSV string and set the attributes of the Job object
        l = csv_string.split(',')
        self.job_id = int(l[0])
        self.priority = int(l[1])
        self.part_id = int(l[2])
        
        self.num_subprocesses = (len(l[3:]) - 3) // 2 # There are two parameters for each subprocess
        for i in range(self.num_subprocesses):
            sub_process = SubProcess()
            sub_process.sub_process_id = int(l[3+2*i])
            sub_process.operation_type = l[3+2*i+1] # Order is sub_process_id, operation_type, sub_process_id, operation_type, ...
            self.subprocesses.append(sub_process)
        self.subprocess_start_times = l[3+self.num_subprocesses*2:3+3*self.num_subprocesses]
        self.subprocess_end_times = l[3+3*self.num_subprocesses:3+4*self.num_subprocesses]
        self.job_start_time = l[-3]
        self.job_end_time = l[-2]
        self.status = l[-1]
    



def main(args=None):
    rclpy.init(args=args)
    scheduler = Scheduler()
    rclpy.spin(scheduler)
    scheduler.destroy_node()
    rclpy.shutdown()