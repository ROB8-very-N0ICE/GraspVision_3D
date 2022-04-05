#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

state = 0


def find_target():
    global state
    # if target found
    # deactivate yolact
    # activate ransac
    rospy.loginfo("find_target - state: %d", state)
    state = 1


def query_shape():
    global state
    # rospy.loginfo("state: %d", state)
    rospy.loginfo("query_shape - state: %d", state)
    state = 0


if __name__ == '__main__':
    rospy.init_node('state_machine')
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if state == 0: find_target()            # find a target
        elif state == 1: query_shape()          # run ransac
        rate.sleep()
