#!/usr/bin/env python

import rospy
import math
import mir_msgs
import actionlib
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal, MoveBaseFeedback

class Navigator():
    def __init__(self):
        self.pose = PoseStamped()
        # self.listener = rospy.Subscriber()  #TODO -> Add subscriber info for current position
        self.talker = rospy.Publisher('move_base', MoveBaseAction, queue_size=1)     #TODO -> Add publisher info for navigation goal
        self.goal = MoveBaseGoal()
        self.facing = 0

    def main(self):
        rospy.init_node('Mir_remote_nav', anonymous=False)
        self.goal.target_pose.pose.position.x = input('X goal: ')
        self.goal.target_pose.pose.position.y = input('Y goal: ')
        self.facing = input('Facing angle (degrees): ')
        self.goal.target_pose.pose.orientation = Quaternion(*(quaternion_from_euler(0, 0, self.facing*math.pi/180, axes='sxyz')))
        self.goal.target_pose.header.frame_id = "map"

        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.talker.publish(self.goal)


if __name__ == '__main__':
    navigator = Navigator()
    while not rospy.is_shutdown():
        print(navigator.goal)
        navigator.main()
        rospy.sleep(1)
        

