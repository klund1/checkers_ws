#!/usr/bin/python

import rospy
from std_msgs.msg import String

class Data:
    pass

# store our state
# possible states: START, HUMAN_MOVE, COMPUTE, MOVE_ARM
D = Data()
D.state = "START"

def eventCallback(msg):
    global D

    if msg.data == "RESET":
        D.state = "HUMAN_MOVE"

    elif msg.data == "START":
        if D.state == "START":
            D.state = "HUMAN_MOVE"
        else:
            rospy.logerr("Bad state transition, resetting state machine")
            D.state = "START"

    elif msg.data == "VIS:FINISHED":
        if D.state == "HUMAN_MOVE":
            D.state = "COMPUTE"
        else:
            rospy.logerr("Bad state transition, resetting state machine")
            D.state = "START"

    elif msg.data == "AI:FINISHED":
        if D.state == "COMPUTE":
            D.state = "MOVE_ARM"
        else:
            rospy.logerr("Bad state transition, resetting state machine")
            D.state = "START"

    elif msg.data == "ARM:FINISHED":
        if D.state == "MOVE_ARM":
            D.state = "HUMAN_MOVE"
        else:
            rospy.logerr("Bad state transition, resetting state machine")
            D.state = "START"

    else:
        rospy.logerr("Bad state transition, resetting state machine")
        D.state = "START"



def main():
    global D

    rospy.init_node('checkers_main')

    state_pub = rospy.Publisher('state', String)
    rospy.Subscriber('event', String, eventCallback)

    r = rospy.Rate(2) #2 Hz
    while not rospy.is_shutdown():
        r.sleep()
        state_pub.publish(D.state)


if __name__ == "__main__":
    main()
