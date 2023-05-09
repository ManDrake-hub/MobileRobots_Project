#! /usr/bin/python3
import rospy
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
import csv
import os
from std_msgs.msg import String
import random
from geometry_msgs.msg import Twist

class Move:
    def __init__(self) -> None:
        self.msg = PoseStamped()
        self.commands={"go_back":0, "left":0, "right":0,"stop":0,"straight_on":0}
        self.waiting = False
        self.pub_goal = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("qr_data_topic", String, self.callback_commands)
        rospy.sleep(3.0)

    def callback_commands(self, msg):
        command = msg.data.lower()
        if self.waiting == True:
            self.commands[command] += 1
        else:
            for key in self.commands.keys():
                self.commands[key] = 0
            self.commands[command] += 1

    def send_goal(self,x,y):
        self.msg.header.frame_id = "map"
        self.msg.pose.position.x = float(x)
        self.msg.pose.position.y = float(y)
        self.msg.pose.orientation.w = 1.0
        self.pub_goal.publish(self.msg)
        #print(f"{self.msg.__str__()}")
    
    def calibration(self):
        forward_msg = Twist()
        forward_msg.linear.x = 0.2
        backward_msg = Twist()
        backward_msg.linear.x = -0.2

        self.pub_cmd.publish(forward_msg)
        rospy.sleep(2)  
        self.pub_cmd.publish(backward_msg)
        rospy.sleep(2)  

        stop_msg = Twist()
        self.pub_cmd.publish(stop_msg)
    
def test_goal(next_goal):
    move.send_goal(next_goal[0],next_goal[1])
    rospy.loginfo("Goal send")
    move.waiting = True
    rospy.wait_for_message("move_base/result", MoveBaseActionResult)
    rospy.loginfo("Goal reached")
    #TO DO: modificare per gestire i casi limite (qr su waypoint raggiunto ecc.)
    move.waiting = False
    most_common_command = max(move.commands, key=move.commands.get)
    print(f"comando più visto: {most_common_command}")

if __name__ == "__main__":
    rospy.init_node("goal_custom")
    move = Move()
    waypoints = []
    #dir_path = os.getcwd()
    #print("La directory corrente è: ", dir_path)
    with open('/home/francesca/Scrivania/MobileRobots_Project/bot/src/navigation/src/waypoints.csv', newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            waypoints.append((float(row[0]), float(row[1])))
    move.calibration()
    print("HO CALIBRATO")
    # Primo goal
    next_goal = random.choice(waypoints)
    test_goal(next_goal)
    # Secondo goal
    next_goal = random.choice(waypoints)
    test_goal(next_goal)

    rospy.spin()