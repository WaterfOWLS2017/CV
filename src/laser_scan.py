#!/usr/bin/python

import sys, os
import numpy as np
import roslib, rospy
from shape_msgs.msg import SolidPrimitive
import serial
import math
import time


class laser_scan:

    global ser;
    
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''

        # ros publisher
        # self.laser_pub = rospy.Publisher
	self.ser = serial.Serial('/dev/ttyACM0', 115200); 
	time.sleep(2);
        # ros subscriber
        self.subscriber = rospy.Subscriber("buoy_data", SolidPrimitive, \
                                            self.callback, queue_size = 1);


    def lidarScan(self,panMin,panMax,panStep,tiltMin,tiltMax,tiltStep,ardSerial):
	#print(ardSerial)
	# create the string to send to the arduino
	string = str(panMin)+","+str(panMax)+","+str(panStep)+","+str(tiltMin)+","+str(tiltMax)+","+str(tiltStep)
	# write the string to the ardunio over serial
	ardSerial.write(string)
	# intitalize places to store each scan and the scans
	data = []
	scans = []

	while 1:
	    # read each line from the arduino and strip away any terminating charectors
	    line =  ardSerial.readline().strip()
	    # the start of a new scan sweep
	    if line == "Begin Scan":
	        print(line)
	    # the end of a scan sweep
	    elif line == "End Scan":
	        scans.append(data)
	        data = []
	        print(line)
	    # all the scan sweeps have been completed
	    elif line == "Done":
	        print(line)
	        break
	    # currently in the middle of a scan sweep
	    else:
	        line = line.split(",")
	        try:
	            print(line)
	            data.append([line[0],line[1],line[2]])
	        except ValueError:
	            pass

        # return the array of scan sweeps
        return scans

    def callback(self, ros_data):
        ''' call back function of subscribed topic.'''

        # acquire data from images
        img_data = ros_data.dimensions;
        
        # if the list is empty, do nothing
        if img_data[2]==0:
            return;
        
        # field of view for vertical and horizontal
        hfov = 97;
        vfov = 81;

        # angle of the horizontal 
        h_angle = (img_data[0])*(hfov/2);
        v_angle = (img_data[1])*(vfov/2);

        
        print "h_angle = " + str(h_angle)
        h_center_pot_value = 292;
        h_min_pot_value = 108;
        h_max_pot_value = 486;
        v_center_pot_value = 311;
        v_min_pot_value = 119;
        v_max_pot_value = 505;
        h_step_value = (h_max_pot_value - h_min_pot_value)/180;
        print "h_step_value = " + str(h_step_value);

        v_step_value = (v_max_pot_value - v_min_pot_value)/180;
        print "v_step_value = " + str(v_step_value);
        
        h_pot_value = h_center_pot_value - (h_angle*h_step_value);
        v_pot_value = v_center_pot_value + (v_angle*v_step_value);
        
        print "h_pot_value = " + str(h_pot_value);
        print "v_pot_value = " + str(v_pot_value);
        self.lidarScan(h_pot_value-10, h_pot_value+10, 2, v_pot_value - 10, v_pot_value + 10, 2, self.ser);



def main(args):
    ''' initializes xxand cleanup ros node '''
    try:
        rospy.init_node('laser_data', anonymous=True);
    except:
        pass;

    ls = laser_scan();
        
    try:
        rospy.spin();
    except KeyboardInterrupt:
        print "shutting down ROS laser scan module"

if __name__ == "__main__":
    main(sys.argv)
