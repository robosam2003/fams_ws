import rclpy
from rclpy.node import Node
from fams_interfaces import JobMessage


class Scheduler(Node):
    def __init__(self):
        super().__init__('scheduler')
        self.get_logger().info('Scheduler node has been started')

        self.job_subscriber = self.create_subscription(
            JobMessage,
            'job',
            self.job_callback,
            10
        )

    def job_callback(self, msg):
        self.get_logger().info(f'Received job:\n ID: {msg.job_id}')
        self.get_logger().info(f'Job type: {msg.job_type}')
        self.get_logger().info(f'Job priority: {msg.priority}')
        self.get_logger().info(f'Job Subprocesses: {msg.subprocesses}')

    

def main(args=None):
    rclpy.init(args=args)
    scheduler = Scheduler()
    rclpy.spin(scheduler)
    scheduler.destroy_node()
    rclpy.shutdown()