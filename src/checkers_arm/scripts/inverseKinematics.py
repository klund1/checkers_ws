import numpy as np

def invKin(P):
  """ get the necessary joint angles for a given position P
      P is a 4x4 matrix describing the desired position and oreintation
      return a tuple of four joint angles (j0,j1,j2,j3) in degrees """

  px = P[0][3]
  py = P[1][3]
  pz = P[2][3]

  nz = P[2][0]
  oz = P[2][1]

  #TODO

  return
