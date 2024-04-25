import rclpy
from rclpy import Node

from fams_interfaces.msg import Schedule, Vision



class FleetController(Node):
    def __init__(self):
        super().__init__('fleet_controller')
        self.get_logger().info('Fleet controller node has been started')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('fleet_size', 2),
            ]
        )

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

        self.vision_locations_subscription = self.create_subscription(
            Vision,
            'vision_locations',
            self.vision_locations_handler,
            10
        )


        

    


def main(args=None):
    rclpy.init(args=args)
    fleet_controller = FleetController()
    rclpy.spin(fleet_controller)
    fleet_controller.destroy_node()
    rclpy.shutdown()


