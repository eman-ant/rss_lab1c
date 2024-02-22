import rclpy
import math
import random
from rclpy.node import Node
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import Float32 

class FakeScanPublisher(Node):

    def __init__(self):
        super().__init__('fake_scan_publisher')
        
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('publish_topic', 'fake_scan'),
                    ('publish_rate', 0.05),
                    ('angle_min', (-2/3)*math.pi),
                    ('angle_max', (2/3)*math.pi),
                    ('range_min', 1.0),
                    ('range_max', 10.0),
                    ('angle_increment', (1/300)*math.pi)
                ]
        )
    
        (param_publish_topic, param_publish_rate) = self.get_parameters(['publish_topic', 'publish_rate'])

        self.publisher_ = self.create_publisher(
                LaserScan,
                param_publish_topic.value,
                10)
        
        self.range_test = self.create_publisher(
                Float32,
                'range_test',
                10)

        timer_period = param_publish_rate.value
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        msg = LaserScan()
        
        (param_angle_min, param_angle_max, param_range_min, param_range_max, param_angle_increment, param_publish_rate) = self.get_parameters(['angle_min', 'angle_max', 'range_min', 'range_max', 'angle_increment', 'publish_rate']) 

        msg.angle_min = param_angle_min.value
        msg.angle_max = param_angle_max.value
        msg.angle_increment = param_angle_increment.value
        msg.scan_time = param_publish_rate.value
        msg.range_min = param_range_min.value
        msg.range_max = param_range_max.value
        msg.ranges = [random.uniform(msg.range_min, msg.range_max) for _ in range(1 + int((msg.angle_max-msg.angle_min)/msg.angle_increment))]
        
        
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)
        self.get_logger().info(f'angle max: {msg.angle_max}, angle min: {msg.angle_min}, angle increment: {msg.angle_increment}, ranges length: {len(msg.ranges)}')
        

        tst_msg = Float32()
        tst_msg.data = float(len(msg.ranges))
        self.range_test.publish(tst_msg)


def main(args=None):
    rclpy.init(args=args)

    fake_scan_publisher = FakeScanPublisher()

    rclpy.spin(fake_scan_publisher)

    fake_scan_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
