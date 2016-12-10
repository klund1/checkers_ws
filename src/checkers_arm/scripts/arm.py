#!/usr/bin/python

import rospy
from std_msgs.msg import String
from checkers_arm.msg import JointAngles

class Data():
  pass

D = Data()
D.last_state = ""
D.next_move = ""
D.need_to_move = False

def move(move_str):
  #TODO
  #needs to compute trajectories and publish to arm_position
  pass

def stateCallback(msg):
  global D
  state = msg.data

  # check if we are in the correct state to move
  if state == "MOVE_ARM" and state != D.last_state:
    D.need_to_move = True

  D.last_state = state


def moveCallback(msg):
  global D
  D.next_move = msg.data
  D.has_move = True


def main():
  global D

  rospy.init_node('checkers_arm')

  D.arm_pub = rospy.Publisher('arm_position', JointAngles, queue_size = 10)
  D.event_pub = rospy.Publisher('event', String, queue_size = 10)

  rospy.Subscriber('state', String, stateCallback, queue_size = 10)
  rospy.Subscriber('move_piece', String, moveCallback, queue_size = 10)

  r = rospy.Rate(5) # 5Hz
  while not rospy.is_shutdown():
    if D.need_to_move and D.has_move:
      move(D.next_move)
      D.event_pub.publish('ARM:FINISHED')
      D.has_move = False
      D.need_to_move = False
    r.sleep()


if __name__ == "__main__":
  main()
