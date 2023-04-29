#! /usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
import random
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class OdomNoiser:
    def __init__(self, publisher: rospy.Publisher, position_var, orientation_var, debug=False) -> None:
        self._publisher = publisher
        self.position_var = position_var
        self.orientation_var = orientation_var
        self.debug = debug
        rospy.Subscriber('odom', Odometry, self.callback_odom)

    def get_rotation(self, orientation):
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion (orientation_list)
        return yaw

    def get_rotation_noised(self, orientation):
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion (orientation_list)
        yaw += random.normalvariate(0, self.orientation_var)
        return quaternion_from_euler(roll, pitch, yaw)

    def callback_odom(self, odom):
        if self.debug:
            print("Clean: ", odom.pose.pose.position.x, odom.pose.pose.position.y, self.get_rotation(odom.pose.pose.orientation))
        odom.pose.pose.position.x += random.normalvariate(0, self.position_var)
        odom.pose.pose.position.y += random.normalvariate(0, self.position_var)
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = self.get_rotation_noised(odom.pose.pose.orientation)
        if self.debug:
            print("Noised: ", odom.pose.pose.position.x, odom.pose.pose.position.y, self.get_rotation(odom.pose.pose.orientation))
        self._publisher.publish(odom)


if __name__ == "__main__":
    POSITION_VAR = 0.05
    ORIENTATION_VAR = 0.05

    rospy.init_node("odom_noised")
    pub = rospy.Publisher("odom_noised", Odometry, queue_size=10)
    rospy.sleep(3)

    odom = OdomNoiser(pub, POSITION_VAR, ORIENTATION_VAR, debug=True)

    rospy.spin()
