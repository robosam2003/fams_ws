import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self): #Costructor of node
        super().__init__("first_node") 
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    def  timer_callback(self):
        self.get_logger().info("Hello" + str(self.counter_))
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args) #initialises ROS
    node = MyNode() #Calls Node class to a variable and starts it
    rclpy.spin(node) #Enables all call backs
    rclpy.shutdown() # stops ROS

if __name__ == '__main__':
    main()