import rclpy

from rclpy.node import Node

from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Workstation


class WorkstationController(Node):
    def __init__(self):
        super().__init__('workstation_controller')
        self.get_logger().info('Workstation controller node has been started')
        
        self.system_state_publisher = self.create_publisher(
            SystemState,
            'system_state',
            10
        )

        # Create several fake workstations and publish to state
        self.Workstation1 = Workstation()
        self.Workstation1.workstation_id = 1
        self.workstation1.available_operations = [32, 33]

        self.Workstation2 = Workstation()
        self.Workstation2.workstation_id = 2
        self.workstation2.available_operations = [22, 33]

        self.Workstation3 = Workstation()
        self.Workstation3.workstation_id = 3
        self.workstation3.available_operations = [22, 32]

        self.workstation_list = [self.Workstation1, self.Workstation2, self.Workstation3]
        for w in self.workstation_list: w.status = 'FREE'
        self.system_state = SystemState()
        self.system_state.workstations = self.workstation_list
        self.system_state_publisher.publish(self.system_state)
        

def main(args=None):
    rclpy.init(args=args)
    workstation_controller = WorkstationController()
    rclpy.spin(workstation_controller)
    workstation_controller.destroy_node()
    rclpy.shutdown()
