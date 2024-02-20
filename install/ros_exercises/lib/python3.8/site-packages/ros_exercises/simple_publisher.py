import rclpy 
import random
from rclpy.node import Node 
from std_msgs.msg import Float32

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Float32, 'my_random_float', 10)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
       
    def timer_callback(self):
        msg = Float32()
        msg.data = random.randint(0, 10)
        self.publisher_.publish(msg) 
        self.get_logger().info('Publishing: "%f"' % msg.data)


    def main(args=None):
        rclpy.init(args=args)
    
        simple_publisher = SimplePublisher() 

        rclpy.spin(simple_publisher) 

        minimal_publisher.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__': 
    main() 

