#!/usr/bin/python
# The contents of this file are in the public domain. See LICENSE_FOR_EXAMPLE_PROGRAMS.txt
#
# This example program shows how you can use dlib to make an object
#   detector for things like faces, pedestrians, and any other semi-rigid
#   object.  In particular, we go though the steps to train the kind of sliding
#   window object detector first published by Dalal and Triggs in 2005 in the
#   paper Histograms of Oriented Gradients for Human Detection.
#
#
# COMPILING/INSTALLING THE DLIB PYTHON INTERFACE
#   You can install dlib using the command:
#       pip install dlib
#
#   Alternatively, if you want to compile dlib yourself then go into the dlib
#   root folder and run:
#       python setup.py install
#   or
#       python setup.py install --yes USE_AVX_INSTRUCTIONS
#   if you have a CPU that supports AVX instructions, since this makes some
#   things run faster.  
#
#   Compiling dlib should work on any operating system so long as you have
#   CMake and boost-python installed.  On Ubuntu, this can be done easily by
#   running the command:
#       sudo apt-get install libboost-python-dev cmake
#
#   Also note that this example requires scikit-image which can be installed
#   via the command:
#       pip install scikit-image
#   Or downloaded from http://scikit-image.org/download.html. 

import os
import sys
import tensorflow as tf
import dlib
import roslib
import rospy
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from PIL import Image as im_p
from skimage import io
import numpy as np

VERBOSE=False

# instantiate bridge
#
bridge = CvBridge();

class image_feature:

   def __init__(self):
      '''Initialize ros publisher, ros subscriber'''

      # ros topic that will be published
      self.img_pub = rospy.Publisher("buoy_data", SolidPrimitive, \
                                     queue_size = 1);
      # ros topic that we subscribe to
      self.subscriber = rospy.Subscriber("/camera/left/image_color", Image, \
                                         self.callback, queue_size = 1)
      if VERBOSE:
         print "subscribed to /camera/left/image_color"
      

      # instantiate CvBridge
      #
      # bridge = CvBridge()

   def callback(self, ros_data):
      ''' Call back function of subsribed topic.
       here the images will be process and the buoys will be detected'''

      if VERBOSE:
         print 'recieved image of type: "%s"' % ros_data.format
            
      try:
         # convert your ROS image message to OpenCV2
         #
         img = bridge.imgmsg_to_cv2(ros_data,"bgr8")
      except CvBridgeError, e:
         print(e)
         # save you opencv image as jpg
         #
         # cv2.imwrite('camera_image.jpg', img)

      # create detector object
      #
      detector = dlib.fhog_object_detector ("/home/roboboat/dlib/tools/imglab/detector2.svm");#("object_detector.svm")

      # evaluate the image
      #
      dets = detector(img)
      # win = dlib.image_window()
      count = 0;
       
      # list the number of buoys detected in the image
      #
      print("Number of buoys detected: {}".format(len(dets)));

      # loop over each buoy detected and show coordinates of the buoy
      # crop the image and send it to tensorflow
      #
      for k, d in enumerate(dets):

         # print the coordinates of the location
         #
         print('%s %d %d %d %d' % (k, d.left(), d.top(), \
                                      d.right(), d.bottom()))

         # calcualte x-y coordinates
         #
         xcoord = (d.right()-d.left())/2 + d.left();
         ycoord = (d.bottom()-d.top())/2 + d.top();

         # calculate normalized coordiantes
         #
         xcoordn = (float(xcoord)-320)/320;
         ycoordn = -1*((float(ycoord)-240)/240);
   
         # calculate radius (in pixels)
         #
         radius = d.right()-d.left();

         print "xcoord = " + str(xcoordn);
         print "ycoord = " + str(ycoordn);
         print radius;

         im_crop = img[d.top():d.top()+d.bottom(), d.left():d.left()+d.right()];

         boundaries = [
            ([38, 25, 200], [116, 99, 255], [1]),
            ([71, 243, 254], [111, 255, 255], [2]),
            ([21, 84, 4], [105, 220, 93], [3]),
         ]

         most_pixel = 0;
         buoy_color = 0;
      
         # loop over the boundaries
         for (lower, upper, code) in boundaries:

            # create NumPy arrays from the boundaries                           
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

            # find the colors within the specified boundaries and apply
            # the mask                                                          
            mask = cv2.inRange(img, lower, upper)
            color = cv2.countNonZero(mask)

            if color==0:
               continue;

            if color > most_pixel:
               most_pixel = color;
               buoy_color = code;

            print('The number of pixels with color are ' + str(color));
            
            
         print ('The color of the buoy is ' + str(buoy_color));
         cv2.imwrite('img.jpg', im_crop);

         
         # crop the image based on the bounding boxes from the detector
         #
         # im_crop = im.crop((d.left(), d.top(), d.right(), d.bottom()));
                      # im_crop.show();d
         # list_colors = im_crop.getcolors(256);
         # print list_colors;

         # represent the color with the following code:
         # red = 1, yellow = 2, green = 3, white = 4, black = 5
         # publishes information concerning the buoy, for now zcoord will
         # represent the color with the following code:
         # red = 1, yellow = 2, green = 3, white = 4, black = 5
         # lsat argument is the shape of the buoy (1 for spherical)
         #
         dimensions = [xcoordn, ycoordn, most_pixel]
         self.img_pub.publish(1, dimensions);
#         cv2.imwrite('img_' + str(count) + '.jpg', im_crop);
#         count = count + 1;

#      win.clear_overlay()
#      win.set_image(img)
#      win.add_overlay(dets)

      if len(dets)==0:
         dimensions = [0, 0, 0]
         self.img_pub.publish(1, dimensions);   



         
def main(args):
   '''Initializes and cleanup ros node'''
   try:
      rospy.init_node('image_feature', anonymous=True);
   except:
      pass;

   ic = image_feature();

   try:
      rospy.spin()
   except KeyboardInterrupt:
      print "shutting down ROS Image buoy detector module"

   cv2.destroyAllWindows()
   
   

if __name__ == "__main__":
    main(sys.argv)
