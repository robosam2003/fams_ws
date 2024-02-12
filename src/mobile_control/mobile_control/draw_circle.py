import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    
    def __init__(self): #Costructor of node
        super().__init__("DrawCircleFile")
        self.cmd_draw_circle_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.5,self.send_velocity_input)
        self.get_logger().info("Draw circle node started")
        
    def send_velocity_input(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_draw_circle_.publish(msg)

def main(args=None):
    rclpy.init(args=args) #initialises ROS
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown() # stops ROS

if __name__ == "__main__":
    main()
