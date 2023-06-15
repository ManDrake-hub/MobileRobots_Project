#! /usr/bin/python3
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty

def clear_costmaps():
    try:
        clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        response = clear_costmaps_service()
        rospy.loginfo("Costmaps cleared successfully!")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to clear costmaps: %s" % e)

if __name__ == "__main__":
    rospy.init_node("stop")
    rospy.wait_for_service('/move_base/clear_costmaps')
    pub = rospy.Publisher("stop", Bool,queue_size=1)
    rospy.sleep(0.5)
    rate = rospy.Rate(10)
    
    while True:
        counter = 0
        input("If you want to stop the robot location press ENTER; after the movement press ENTER: ")
        pub.publish(True)
        rospy.sleep(0.5)
        input()
        clear_costmaps()
        pub.publish(False)