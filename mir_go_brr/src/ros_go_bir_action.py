#!/usr/bin/env python

# Major Reference: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
# https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

import rospy
import mir_msgs
import actionlib
import math
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class movebase_client():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        
    def main(self):
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = input('X position: ')
        self.goal.target_pose.pose.position.y = input('Y position: ')
        self.goal.target_pose.pose.orientation = Quaternion(*(quaternion_from_euler(0, 0, input('Facing angle (degrees): ')*math.pi/180, axes='sxyz')))
        self.client.send_goal(self.goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

if __name__ == '__main__':
    rospy.init_node('movebase_client')
    client = movebase_client()
    while not rospy.is_shutdown():
        result = client.main()
        if result:
            rospy.loginfo("Move to goal complete!")
        else:
            rospy.loginfo("Navigation failed!")
        rospy.sleep(1)
