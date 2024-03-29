import rclpy
from rclpy.node import Node
from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Schedule, Workstation, JobList
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

        self.active_job_publisher = self.create_publisher(
            JobList,
            'active_jobs',
            10
        )

        self.state_subscriber = self.create_subscription(
            SystemState,
            'system_state',
            self.system_state_callback,
            10
        )
        self.state_publisher = self.create_publisher(
            SystemState,
            'system_state',
            10
        )
        self.schedule_publisher = self.create_publisher(
            Schedule,
            'schedule',
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
        joblist = JobList()
        joblist.list = self.active_job_list
        self.active_job_publisher.publish(joblist)

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

        part_obj_size = 6 # Number of parameters in the Part object
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
        self.get_logger().info('System state message received')
        self.system_state = msg

        # Create some fake workstations for now
        workstation1 = Workstation()
        workstation1.workstation_id = 1
        workstation1.state = 'FREE'
        workstation1.available_operations = ['MILLING', 'DRILLING']
        workstation2 = Workstation()
        workstation2.workstation_id = 2
        workstation2.state = 'FREE'
        workstation2.available_operations = ['MILLING']
        workstation3 = Workstation()
        workstation3.workstation_id = 3
        workstation3.state = 'FREE'
        workstation3.available_operations = ['DRILLING']
        self.system_state.workstations = [workstation1, workstation2, workstation3]

        # "Update Job Log with Timings" ??? 
        
        # Sort parts with no current subprocesses into a priority list
        priority_list = []
        for part in self.system_state.parts: 
            if part.current_subprocess_id == 0:
                priority_list.append(part)
        
        # Update the parts with their next current subprocesses, if any.
        job_job_id_mapping = {job.job_id: job for job in self.active_job_list} # This is a dictionary that maps job_id to job
        for part in priority_list:
            parts_job = job_job_id_mapping[part.job_id]
            for sub in parts_job.subprocesses:
                if part.previous_subprocess_id == 0: # If the part has no previous subprocess, then the first subprocess is the current one
                    part.next_subprocess_id = parts_job.subprocesses[0].sub_process_id
                elif sub.sub_process_id == part.previous_subprocess_id: # If the part has no previous subprocess, then the first subprocess is the current one
                    # Find the next subprocess
                    prev_sub_index = parts_job.subprocesses.index(sub)
                    # If the previous subprocess was the last one, then the part is done, the job in job log needs changing to FINISHED
                    if prev_sub_index == len(parts_job.subprocesses) - 1:
                        parts_job.status = 'FINISHED'
                        self.get_logger().info('Job ' + str(parts_job.job_id) + ' has finished')
                        # Update job log - I'm not sure if this actually replaces the line in the file, I have no way to test rn
                        with open(self.job_log_path, "r") as f:
                            lines = f.readlines()
                        with open(self.job_log_path, "w") as f:
                            for line in lines:
                                if str(parts_job.job_id) in line:
                                    line = line.replace('IN PROGRESS', 'FINISHED')
                                f.write(line)
                        # remove the job from the active job list and the priority list and the system state parts list
                        self.active_job_list.remove(parts_job)
                        priority_list.remove(part)
                        self.system_state.parts.remove(part)
                        self.get_logger().info('Job ' + str(parts_job.job_id) + ' has been removed from the active job list and the priority list')
                    else:
                        part.next_subprocess_id = parts_job.subprocesses[prev_sub_index + 1].sub_process_id
                # Every part should have a current_subprocess_id now
                                        
        job_priority_mapping = {job.job_id: job.priority for job in self.active_job_list} # This is a dictionary that maps job_id to priority
        job_start_time_mapping = {job.job_id: job.start_time for job in self.active_job_list} # This is a dictionary that maps job_id to start time

        # Sort the priority list by priority first and then start time
        priority_list = sorted(priority_list, key=lambda part: (job_priority_mapping[part.job_id], job_start_time_mapping[part.job_id])) 
        # self.get_logger().info('Priority list has been sorted: ' + str(priority_list))

        free_workstations = [workstation for workstation in self.system_state.workstations if workstation.state == 'FREE']
        # self.get_logger().info('Free workstations: ' + str(free_workstations))
        # Go down the priority list, matching subprocesses with workstations that are both free and can perform the correct part operations
        schedule_msg = Schedule()
        schedule_parts = []
        schedule_subprocesses = []
        schedule_worstations = []

        for part in priority_list:
            part_job = job_job_id_mapping[part.job_id]
            subprocess_subprocess_id_mapping = {sub.sub_process_id: sub for sub in part_job.subprocesses}
            next_subprocess = subprocess_subprocess_id_mapping[part.next_subprocess_id]
            # self.get_logger().info('Next subprocess: ' + str(next_subprocess))
            for workstation in free_workstations:
                if next_subprocess.operation_type in workstation.available_operations:
                    self.get_logger().info('Part ' + str(part.part_id) + ' has been matched with workstation ' + str(workstation.workstation_id))
                    schedule_parts.append(part)
                    schedule_subprocesses.append(next_subprocess)
                    schedule_worstations.append(workstation)
                    free_workstations.remove(workstation)
                    break
            # If no free workstations are available, break the loop
            if len(free_workstations) == 0:
                break

        # Create Schedule message from these matches
        schedule_msg.parts = schedule_parts
        schedule_msg.subprocesses = schedule_subprocesses
        schedule_msg.workstations = schedule_worstations

        # Publish Schedule message   
        self.schedule_publisher.publish(schedule_msg)
        self.get_logger().info('Schedule message has been published')


def main(args=None):
    rclpy.init(args=args)
    scheduler = Scheduler()
    rclpy.spin(scheduler)
    scheduler.destroy_node()
    rclpy.shutdown()