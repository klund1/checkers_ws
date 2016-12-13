#!/usr/bin/python

import re
import numpy as np

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from checkers_arm.msg import JointAngles

from inverseKinematics import *

class Data():
  pass

D = Data()

def move(move_str):
  board_coords = D.move_pattern.findall(move_str)
  board_coords = [(int(p[0]),int(p[1])) for p in board_coords]

  move_between(D.RESET_POS, world_pos(board_coords[0]) )
  em_on()
  for i in range(len(board_coords)-1):
    pos1 = world_pos(board_coords[i])
    pos2 = world_pos(board_coords[i+1])

    move_between( pos1,        above(pos1) )
    move_between( above(pos1), above(pos2) )
    move_between( above(pos2), pos2        )

  em_off()
  move_between(world_pos(board_coords[-1]), D.RESET_POS)
  return

def move_between(pos1,pos2):
  """ move between two world positions """
  arm_msg = JointAngles()
  r = rospy.Rate(2)
  #move to every intermediate position
  for pos in intermediate_positions(pos1,pos2):
    j0,j1,j2,j3 = invKin(pos)

    arm_msg.j0 = j0
    arm_msg.j1 = j1
    arm_msg.j2 = j2
    arm_msg.j3 = j3
    D.arm_pub.publish(arm_msg)
    r.sleep()

  return

def intermediate_positions(pos1,pos2):
  """ get a list of positions between pos1 and pos2 """
  #TODO not sure how to do this... some ideas/notes:
  #  - limit the number of points based on how far the move is
  #  - finding the positions should be very easy (just a line)
  #  - how should we handle orientation?

def world_pos(board_coord):
  """ get the world position for a given board coordinate """
  P = np.identity(4)
  # TODO just need to measure the board
  return P

def above(pos):
  """ get the position 2cm above the given position """
  new_pos = pos.copy()
  new_pos[2][3] += 2 # add 2 to p_z
  return new_pos

def em_on():
  global D
  D.em_pub.publish(True)

def em_off():
  global D
  D.em_pub.publish(False)

def stateCallback(msg):
  global D
  state = msg.data

  # check if we are in the correct state to move
  if state == "MOVE_ARM" and state != D.last_state:
    D.need_to_move = True

  D.last_state = state
  return


def moveCallback(msg):
  global D
  D.next_move = msg.data
  D.has_move = True
  return

def main():
  global D

  # initialize global variables
  D.last_state = ""
  D.next_move = ""
  D.need_to_move = False
  D.move_pattern = re.compile('\(([0-9]),([0-9])\)')
  #TODO put an actual position here:
  D.RESET_POS = np.array( [[1,0,0,0],
                           [0,1,0,0],
                           [0,0,1,5],
                           [0,0,0,1]] )

  # initialize ROS
  rospy.init_node('checkers_arm')
  D.arm_pub = rospy.Publisher('arm_position', JointAngles, queue_size = 10)
  D.em_pub = rospy.Publisher('electromagnet', Bool, queue_size = 10)
  D.event_pub = rospy.Publisher('event', String, queue_size = 10)
  rospy.Subscriber('state', String, stateCallback, queue_size = 10)
  rospy.Subscriber('move_piece', String, moveCallback, queue_size = 10)

  # main loop
  r = rospy.Rate(5) # 5Hz
  while not rospy.is_shutdown():
    if D.need_to_move and D.has_move:
      move(D.next_move)
      D.event_pub.publish('ARM:FINISHED')
      D.has_move = False
      D.need_to_move = False
    r.sleep()
  return

if __name__ == "__main__":
  main()
