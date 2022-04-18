#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8

state = 0
previous_state = 0


def look():
    global state
    # can it find a target object?
    # if not, can it find a hand?
    # when something found, deactivate yolact and change state
    rospy.loginfo("state: looking")


def fit_object():
    global state
    # find the object shape using the depth image
    rospy.loginfo("state: fit object")


def hand_pose():
    global state
    # fit hand based on previous state
    rospy.loginfo("state: changing hand pose")


def manual():
    global state
    # user can grab or relise object, change function or restart
    rospy.loginfo("state: manual controll")


def state_update(data):
    # receives feedback to proceed to the next state when a state is done
    global state
    state = data
    rospy.loginfo("Next state: %d", data)


if __name__ == '__main__':
    rospy.init_node('state_machine')
    rospy.Subscriber("state_feedback", UInt8, state_update)
    pub_state = rospy.Publisher('state_feedforward', UInt8, queue_size=1)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rospy.loginfo("running")
        if state == 0: look()               # camera node
        elif state == 1: fit_object()       # ransac node
        elif state == 2: hand_pose()        # hand_pose node
        elif state == 3: manual()           # user_command node
        rate.sleep()
