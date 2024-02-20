import rclpy
from rclpy.node import Node
from fams_interfaces.msg import Job, SubProcess, Part, SystemState
from rosidl_runtime_py import *



class Scheduler(Node):
    def __init__(self):
        super().__init__('scheduler')
        self.job_log_path = "./src/scheduler/scheduler/JobLog.csv"  # Relative so that it works on any machine
        self.get_logger().info('Scheduler node has been started')

        self.job_subscriber = self.create_subscription(
            Job,
            'job', # Topic name
            self.job_message_callback,
            10 # QoS profile
        )
        self.state_publisher = self.create_publisher(
            SystemState,
            'system_state',
            10
        )

        self.state_subscriber = self.create_subscription(
            SystemState,
            'system_state',
            self.system_state_callback,
            10
        )

        self.system_state = SystemState()

        self.active_job_list = []
        self.update_active_job_list() # Update active job list from job log

    def update_active_job_list(self):
        # For all the jobs in the job log, if they are IN PROGRESS or PENDING, add them to the active job list
        with open(self.job_log_path, "r") as f:
            for line in f:
                if "IN PROGRESS" in line or "PENDING" in line:
                    job = self.csv_to_job(line)
                    self.active_job_list.append(job)
        self.get_logger().info('Active job list has been updated, with ' + str(len(self.active_job_list)) + ' jobs')

    def csv_to_job(self, csv_string) -> Job:
        # Convert csv string to Job message
        job = Job()
        csv_list = csv_string.split(',')

        job.job_id = int(csv_list[0])
        job.priority = int(csv_list[1])

        part = Part()
        part.part_id = int(csv_list[2])
        part.location = int(csv_list[3])
        part.current_subprocess_id = int(csv_list[4])
        part.job_id = int(csv_list[5])
        job.part = part

        part_obj_size = 4 # Number of parameters in the Part object
        sub_obj_size = 4 # Number of parameters in the SubProcess object
        sub_start_id = 2 + part_obj_size
        n_sub = (len(csv_list) - part_obj_size - 5) // sub_obj_size
        for i in range(n_sub):
            sub = SubProcess()
            sub.sub_process_id = int(csv_list[sub_start_id + sub_obj_size*i])
            sub.operation_type = str(csv_list[sub_start_id + sub_obj_size*i + 1])
            sub.start_time = int(csv_list[sub_start_id + sub_obj_size*i + 2])
            sub.end_time = int(csv_list[sub_start_id + sub_obj_size*i + 3])
            job.subprocesses.append(sub)
        
        job.start_time = int(csv_list[-3])
        job.end_time = int(csv_list[-2])
        job.status = str(csv_list[-1])

        return job

    def job_message_callback(self, msg):
        self.save_job_to_log(msg) # Add Job to Job Log
        self.get_logger().info('Job message has been added to the job log')

        if msg.status == 'PENDING' or msg.status == 'IN PROGRESS':
            self.active_job_list.append(msg)
            self.get_logger().info('Job message has been added to the active job list, which now has ' + str(len(self.active_job_list)) + ' jobs')

        self.system_state.parts.append(msg.part) # Add parts to system state parts list
        self.state_publisher.publish(self.system_state) # Publish /SystemState message

    def save_job_to_log(self, msg):
        # Write to a job log csv file
        with open(self.job_log_path, 'a') as f:
            csv = message_to_csv(msg)
            f.write(csv)
            f.write('\n')  

    def system_state_callback(self, msg):
        self.system_state = msg

        # "Update Job Log with Timings" ??? 
        
        priority_list = []
        for part in self.system_state.parts: # Sort parts with no current subprocesses into a priority list
            if part.current_subprocess_id == 0:
                priority_list.append(part)
        job_priority_mapping = {job.job_id: job.priority for job in self.active_job_list} # This is a dictionary that maps job_id to priority
        job_start_time_mapping = {job.job_id: job.start_time for job in self.active_job_list}

        priority_list = sorted(priority_list, key=lambda part: (job_priority_mapping[part.job_id], job_start_time_mapping[part.job_id])) # Sort the priority list by priority first and then start time
         

        free_workstations = [workstation for workstation in self.system_state.workstations if workstation.status == 'FREE']
        # Go down the priority list, matching subprocesses with workstations that are both free and can perform the correct part operations
    
        schedule_worstations = []
        # for part in priority_list:
        #     for workstation in free_workstations:
        #         if workstation.operation_type == part.operation_type:
        #             schedule_worstations.append(workstation)
        #             free_workstations.remove(workstation)
        #             break
            

        # Create Schedule message from these matches
        # TODO

        # Publish Schedule message   
        # TODO

        


    



def main(args=None):
    rclpy.init(args=args)
    scheduler = Scheduler()
    rclpy.spin(scheduler)
    scheduler.destroy_node()
    rclpy.shutdown()