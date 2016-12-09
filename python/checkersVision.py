import cv2
import numpy

def getImage():
  """ Capture an image from the webcam """
  camera = cv2.VideoCapture(0)
  for i in range(10):
    camera.read()
  retval, im = camera.read()
  fileName = "~/testImage.png"
  cv2.imwrite(file, im)
  del(camera)


