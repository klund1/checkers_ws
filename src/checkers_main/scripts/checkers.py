#!/usr/bin/python

import rospy
from std_msgs.msg import String

class Data:
    pass

# store our state
# possible states: START, HUMAN_MOVE, VIS, AI_COMPUTE, MOVE_ARM
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

    elif msg.data == "HUMAN:FINISHED":
        if D.state == "HUMAN_MOVE":
            D.state = "VIS"
        else:
            #rospy.logerr("Bad state transition, resetting state machine")
            #D.state = "START"
            # this is ok because this is just a button that might get pressed out of turn
            pass

    elif msg.data == "VIS:FINISHED":
        if D.state == "VIS":
            D.state = "AI_COMPUTE"
        else:
            rospy.logerr("Bad state transition, resetting state machine")
            D.state = "START"

    elif msg.data == "AI:FINISHED":
        if D.state == "AI_COMPUTE":
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

    state_pub = rospy.Publisher('state', String, queue_size = 10)
    rospy.Subscriber('event', String, eventCallback, queue_size = 10)

    r = rospy.Rate(5) #5 Hz
    while not rospy.is_shutdown():
        state_pub.publish(D.state)
        r.sleep()


if __name__ == "__main__":
    main()
