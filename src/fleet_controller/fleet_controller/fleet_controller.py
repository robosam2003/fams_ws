import rclpy
from rclpy import Node

from fams_interfaces.msg import Schedule, Vision, SystemState
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class FleetController(Node):
    def __init__(self):
        super().__init__('fleet_controller')
        self.get_logger().info('Fleet controller node has been started')

        self.workstation_goal_poses = {  # CHANGE TO ACTUAL VALUES
            'workstation1': [1, 1, 0],
            'workstation2': [1, 2, 3.14159]
        }

        self.schedule_subscription = self.create_subscription(
            Schedule,
            'schedule',
            self.schedule_handler,
            10
        )

        # self.vision_locations_subscription = self.create_subscription(
        #     Vision,
        #     'vision_locations',
        #     self.vision_locations_handler,
        #     10
        # )

        self.system_state_subscription = self.create_subscription(
            SystemState,
            'system_state',
            self.system_state_handler,
            10
        )

        self.nexus1_goal_pose_publisher = self.create_publisher(
            PoseStamped,
            'nexus1/goal_pose',
            10
        )

        self.assign_fleet_timer = self.create_timer(1, self.assign_fleet) # Every second, check if there are free AMRs and assign them to tasks
        

        # Attributes
        self.system_state = SystemState()
        self.schedule = Schedule()

    def system_state_handler(self, msg):
        self.system_state = msg # Update local system state
        # self.assign_fleet()
        
    def schedule_handler(self, msg):
        self.schedule = msg
        # self.assign_fleet()

    def assign_fleet(self):
        parts_schedule = self.schedule.parts
        subprocesses_schedule = self.schedule.subprocesses
        workstations_schedule = self.schedule.workstations
        n = len(parts_schedule)

        free_amrs = []
        for amr_sending in self.system_state.mobile_robots:
            if amr_sending.state == "FREE":
                free_amrs.append(amr_sending)

        if len(free_amrs) != 0: # If there are free AMRs
            for i in range(n):
                part = parts_schedule[i]
                subprocess = subprocesses_schedule[i]
                workstation = workstations_schedule[i]

                # Find the location of the part
                part_location = part.location
                if part_location.startswith("workstation"):
                    workstation_name = part_location

                    # Set the workstation goal pose
                    workstation_pose = PoseStamped()
                    workstation_pose.pose.position.x = self.workstation_goal_poses[workstation_name][0]
                    workstation_pose.pose.position.y = self.workstation_goal_poses[workstation_name][1]
                    workstation_pose.pose.position.z = 0.0
                    yaw = self.workstation_goal_poses[workstation_name][2]
                    q = quaternion_from_euler(0, 0, yaw)
                    workstation_pose.pose.orientation.x = q[0]
                    workstation_pose.pose.orientation.y = q[1]
                    workstation_pose.pose.orientation.z = q[2]
                    workstation_pose.pose.orientation.w = q[3]
                                        
                    # Find a free AMR
                    amr_sending = free_amrs.pop()

                    # Send the AMR to the workstation
                    self.nexus1_goal_pose_publisher.publish(workstation_pose)
                    self.get_logger().info(f"Sent AMR to workstation {workstation_name}")

                    # Set the AMR to busy
                    # Find the amr in the system state
                    for amr_state in self.system_state.mobile_robots:
                        if amr_state.name == amr_sending.name:
                            amr_state.state = "BUSY"
                            break
                    # Publish the new system state
                    self.system_state_publisher.publish(self.system_state)
                    break
        
        






def main(args=None):
    rclpy.init(args=args)
    fleet_controller = FleetController()
    rclpy.spin(fleet_controller)
    fleet_controller.destroy_node()
    rclpy.shutdown()


