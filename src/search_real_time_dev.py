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

VERBOSE=False

def callback(ros_data):
  global img
  img = bridge.imgmsg_to_cv2(ros_data,"bgr8")

img = None
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
                                         callback, queue_size = 1, \
                                         buff_size = 52428800)
      if VERBOSE:
         print "subscribed to /camera/left/image_color"
      

      # instantiate CvBridge
      #
      # bridge = CvBridge()

   def image_process(self):
      ''' Call back function of subsribed topic.
       here the images will be process and the buoys will be detected'''
      count = 0;
            
      global img
      if img == None:
          return
      cv2.imwrite("camera_image.jpg", img)


      # create detector object
      #
      detector = dlib.fhog_object_detector("/home/roboboat/dlib/tools/imglab/detector2.svm");

      # evaluate the image
      #
      dets = detector(img)
      # win = dlib.image_window()
      
      # list the number of buoys detected in the image
      #
      print("Number of buoys detected: {}".format(len(dets)));
      cv2.imwrite('/home/roboboat/Documents/photos/outdoor_test/img'+ str(count)+ '.jpg', img)
      count = count + 1;
      
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
         ycoordn = (float(ycoord)-240)/240;
   
         # calculate radius (in pixels)
         #
         radius = d.right()-d.left();

         print xcoordn;
         print ycoordn;
         print radius;

         im = im_p.fromarray(img);

         # reopen the image using a PIL Image data type
         #
#         try:
#            im
#         except NameError:
#            pass;

         # crop the image based on the bounding boxes from the detector
         #
         im_crop = im.crop((d.left(), d.top(), d.right(), d.bottom()));

         # filepace to save cropped image to a specific file
         #
         file_path = '/home/roboboat/tf_files/buoy_eval/tens_img.jpg';

         # save the image to that file path
         #
         im_crop.save(file_path);

         # open the cropped image using Tensorflow
         #
         image_data = tf.gfile.FastGFile(file_path, 'rb').read();

         # Loads label file, strips off carriage return
         #
         label_lines = [line.rstrip() for line in tf.gfile.GFile("/home/roboboat/tf_files/retrained_labels.txt")]

         # Unpersists graph from file
         #
         with tf.gfile.FastGFile("/home/roboboat/tf_files/retrained_graph.pb", 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
            _ = tf.import_graph_def(graph_def, name='')

         with tf.Session() as sess:

            # Feed the image_data as input and get first prediction
            #
            softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')
    
            predictions = sess.run(softmax_tensor, \
                             {'DecodeJpeg/contents:0': image_data})
    
            # Sort to show labels of first prediction in order of confidence
            #
            top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]

            # grabs prediction for the color
            #
            greatest_pred = 0;
            buoy_color = '';
      
         for node_id in top_k:
            human_string = label_lines[node_id]
            score = predictions[0][node_id]
            print('%s %.5f' % (human_string, score))
            if score > greatest_pred:
               greatest_pred = score;
               buoy_color = human_string;

         # represent the color with the following code:
         # red = 1, yellow = 2, green = 3, white = 4, black = 5
         if buoy_color.startswith('r'):
            zcoord = 1;
         elif buoy_color.startswith('y'):
            zcoord = 2;
         elif buoy_color.startswith('g'):
            zcoord = 3;
         elif buoy_color.startswith('w'):
            zcoord = 4;
         elif buoy_color.startswith('b'):
            zcoord = 5;
         else:
            zcoord = 0;

         print zcoord;
         print greatest_pred;
         zcoord = zcoord + greatest_pred;
         print zcoord;
   
         # publishes information concerning the buoy, for now zcoord will
         # represent the color with the following code:
         # red = 1, yellow = 2, green = 3, white = 4, black = 5
         # lsat argument is the shape of the buoy (1 for spherical)
         #
         dimensions = [xcoordn, ycoordn, zcoord]
         self.img_pub.publish(1, dimensions);


#         win.clear_overlay()
#         win.set_image(img)
#         win.add_overlay(dets)

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
      while True:
          ic.image_process()

   except KeyboardInterrupt:
      print "shutting down ROS Image buoy detector module"

   cv2.destroyAllWindows()
   
   

if __name__ == "__main__":
    main(sys.argv)
