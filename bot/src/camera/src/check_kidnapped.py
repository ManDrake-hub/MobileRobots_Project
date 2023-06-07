#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import random
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool
import math
from geometry_msgs.msg import PoseArray, Pose

class Kidnapped:
    def __init__(self):
        rospy.init_node('kidnapped')
        self.listener = tf.TransformListener()
        # Set initial robot position and orientation
        self.robot_x = None
        self.robot_y = None
        self.robot_z = None
        self.sub_upward = rospy.Subscriber('upward', Bool, self.upward)
        self.sub_downward = rospy.Subscriber('downward', Bool, self.downward)
        
        self.pub_pose_estimate = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.robot_position = None
        self.estimate_tresh = 1
        self.pose_estimate = PoseWithCovarianceStamped()
        self.particle_pub = rospy.Publisher('particlecloud', PoseArray, queue_size=10)

    def get_robot_position(self):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.robot_x = trans[0]
            self.robot_y = trans[1]
            self.robot_z = rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def publish_particle_cloud(self):
        particle_array = PoseArray()
        particle_array.header.frame_id='map'
        num_particles = 1000  # Numero di particle
        for _ in range(num_particles):
            particle_pose = Pose()
            particle_pose.position.x = random.uniform(self.pose_estimate.pose.pose.position.x - 0.5, self.pose_estimate.pose.pose.position.x + 0.5)
            particle_pose.position.y = random.uniform(self.pose_estimate.pose.pose.position.y - 0.5, self.pose_estimate.pose.pose.position.y + 0.5)
            particle_pose.position.z = 0.0  
            particle_array.poses.append(particle_pose)
        #print(particle_array)
        #self.particle_pub.publish(particle_array)


    def upward(self,msg):
        self.sub_upward.unregister()
        self.get_robot_position()

    def downward(self,msg):
        self.sub_downward.unregister()
        self.pose_estimate.header
        self.pose_estimate.header.frame_id = "map"
        if self.robot_x or self.robot_y is not None:
            self.pose_estimate.pose.pose.position.x = self.robot_x - 3.0
            self.pose_estimate.pose.pose.position.y = self.robot_y
            self.pose_estimate.pose.pose.orientation.x = self.robot_z[0]
            self.pose_estimate.pose.pose.orientation.y = self.robot_z[1]
            self.pose_estimate.pose.pose.orientation.z = self.robot_z[2]
            self.pose_estimate.pose.pose.orientation.w = self.robot_z[3]
            self.pose_estimate.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
            print(self.pose_estimate)
            self.pub_pose_estimate.publish(self.pose_estimate)
            rospy.sleep(0.5)
            self.publish_particle_cloud()
            rospy.sleep(0.5)
        

if __name__ == '__main__':
    try:
        node = Kidnapped()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
