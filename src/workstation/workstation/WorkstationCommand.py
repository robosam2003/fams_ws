# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Workstation, Location

# class WorkstationCommand(Node):
#     def __init__(self):
#         super().__init__('WorkstationCommand')
#         self.get_logger().info('Workstation C node has been started')
#         self.WorkstationCommand_subscriber = self.create_subscription(
#         SystemState,
#         'system_state',
#         self.listener_callback,
#         10
#         )
#         self.workstation_publisher = self.create_publisher(String,'WorkstationCommand'
#             ,
#             10
#         )

#         command=String()
#         command.data="partInput"
#         self.workstation_publisher.publish(command)
#         self.get_logger().info('{}:{}'.format("Publishing",command))
       

#     def listener_callback(self):
#         # msg = Location()
#     #     msg.x = 1.0  # Replace with your actual X coordinate
#     #     msg.y = 2.0  # Replace with your actual Y coordinate 
        
        
# def main(args=None):
#     rclpy.init(args=args)
#     workstation_command = WorkstationCommand()
#     rclpy.spin(workstation_command)
#     workstation_command.destroy_node()
#     rclpy.shutdown()

    