import rclpy 
from rclpy.node import Node
import tf2_ros
import numpy as np
import geometry_msgs
import tf_transformations
import time

from geometry_msgs.msg import TransformStamped 


class DynamicTfPublisher(Node):

    def __init__(self):
        super().__init__('dynamic_tf_cam_publisher')

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)

        timer_period = 0.05 
        self.timer = self.create_timer(timer_period, self.node_callback)

        self.t = time.perf_counter() 

        self.rotation_rate = 1
        self.rotation_radius = 2.0 

    def tf_to_se3(self, transform):
        q = transform.rotation 
        q = [q.x, q.y, q.z, q.w]
        t = transform.translation 
        mat = tf_transformations.quaternion_matrix(q)
        mat[0, 3] = t.x 
        mat[1, 3] = t.y
        mat[2, 3] = t.z
        return mat 
    
    def se3_to_tf(self, mat, time, parent, child):
        obj = geometry_msgs.msg.TransformStamped()
        obj.header.stamp = time.to_msg() 
        obj.header.frame_id = parent
        obj.child_frame_id = child
        obj.transform.translation.x = mat[0, 3]
        obj.transform.translation.y = mat[1, 3]
        obj.transform.translation.z = mat[2, 3] 

        q = tf_transformations.quaternion_from_matrix(mat)
        obj.transform.rotation.x = q[0]
        obj.transform.rotation.y = q[1]
        obj.transform.rotation.z = q[2]
        obj.transform.rotation.w = q[3]

        return obj


    def node_callback(self):
        try:
            tf_world_to_robot = self.tfBuffer.lookup_transform('world', 'base_link_gt', rclpy.time.Time()) 

        except tf2_ros.TransformException:
            self.get_logger().info('no transform from world to base_link_gt found') 
            return 

        robot_to_world = self.tf_to_se3(tf_world_to_robot.transform)
        
        new_t = time.perf_counter()
        time_elapsed = new_t - self.t
        angle = 2 * np.pi * self.rotation_rate * time_elapsed 

        
        left_cam_to_robot_translation = [self.rotation_radius * np.cos(angle), -0.05, self.rotation_radius * np.sin(angle)]
        left_cam_to_robot_translation = np.array(left_cam_to_robot_translation).T
        left_cam_to_robot = np.eye(4)
        left_cam_to_robot[:3, -1] = left_cam_to_robot_translation[:3]

        now = self.get_clock().now()

        left_cam_to_world = robot_to_world @ left_cam_to_robot
        tf_left_cam_to_world = self.se3_to_tf(left_cam_to_world, now, parent='world', child='left_cam')

        #self.br.sendTransform([tf_world_to_robot, tf_left_cam_to_world])


        right_to_left_translation = [0, 1, 0]
        right_to_left_translation = np.array(right_to_left_translation).T

        right_to_left = np.eye(4)
        right_to_left[:3, -1] = right_to_left_translation[:3]

        right_to_left = left_cam_to_world @ right_to_left
        tf_right_to_left = self.se3_to_tf(right_to_left, now, parent='world', child='right_cam')
       
        self.br.sendTransform([tf_world_to_robot, tf_left_cam_to_world])
        self.br.sendTransform([tf_left_cam_to_world, tf_right_to_left])
        
        self.get_logger().info('Published')

def main(args=None):
    rclpy.init(args=args)

    node = DynamicTfPublisher() 

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown() 
    

if __name__ == '__main__':
    main() 
