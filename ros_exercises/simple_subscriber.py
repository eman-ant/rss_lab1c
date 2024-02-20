import rclpy 
from rclpy.node import Node
import math
from std_msgs.msg import Float32 

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
                Float32,
                'my_random_float',
                self.listener_callback,
                10)
        self.subscription

        self.publisher_ = self.create_publisher(
                Float32,
                'random_float_log',
                10)


    def listener_callback(self, msg):
        float_val = msg.data
        log_float_val = Float32() 
        log_float_val.data = math.log(float_val)

        self.get_logger().info('I heard: %f' % float_val)
        
        self.publisher_.publish(log_float_val) 
        self.get_logger().info('Publishing to random_float_log: %f' % log_float_val.data) 


def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber() 

    rclpy.spin(simple_subscriber)

    simple_subscriber.destroy_node()
    rclpy.shutdown() 


if __name__ == '__main__':
    main() 
