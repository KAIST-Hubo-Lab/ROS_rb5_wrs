#!/usr/bin/env python

 ###################################################################
 #
 # This ROS node is Action Client that enables motion on RB5 platform.
 # Refer to RB5 Client at www.kirobotics.com
 # 
 #
 # Output : /
 # 
 # Input  : /
 
 # E-mail : ml634@kaist.ac.kr     (Lee Moonyoung)
 
 #
 # Versions :
 # v1.0.2019.09
 ###################################################################
 
 

#import standard ROS packages
import rospy
import numpy as np
import smach
import smach_ros# First import the library
import pyrealsense2 as rs
from smach import StateMachine
from std_msgs.msg import String
from std_msgs.msg import Int32
import time

#import rb5 action headers
import actionlib
import rb5_ros_wrapper.msg 
from rb5_ros_wrapper.msg import *

#openCV
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge  = CvBridge()

#ros msg
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

#PLY pointcloud
import pyrealsense2 as rs


#rb5 goal data 
pose_coordinate = np.zeros(6)

for i in range(len(pose_coordinate)):
	pose_coordinate[i] = 0

rb5_goal = rb5_ros_wrapper.msg.MotionGoal(type = '', d0 = 0, d1 = 0, data = 0, coordinate = pose_coordinate , spd = 0, acc = 0)


#global var
result_flag = False
cv_image = []

pos = Point(x=0,y=0,z=0)
ori = Point(x=0,y=0,z=0)
countFlag = 0
processFlag = True

# ====================== create directory  ===================================
try: 
      
    # creating a folder named data 
    if not os.path.exists('../data'): 
        os.makedirs('../data') 
  
# if not created then raise error 
except OSError: 
    print ('Error: Creating directory of data') 
    
    


    
# ================================================ result subscribe call back ================================================ #

def callback_pos(data):
	global pos, countFlag
	#rospy.loginfo("x_pos: %f", data.x)
	pos.x = data.x
	pos.y = data.y
	pos.z = data.z
	
	countFlag = countFlag + 1


def callback_ori(data):
	global ori, countFlag
	#rospy.loginfo("x_ori: %f", data.x)
	ori.x = data.x
	ori.y = data.y
	ori.z = data.z
	
	countFlag = countFlag + 1
    
def realsense_callback_depth(data):
    global countFlag
    
    cv_image_depth = bridge.imgmsg_to_cv2(data,"mono16")
    #save img name
    countFlag = countFlag + 1
    name = '../data/frame' + str(countFlag)  +'.jpg'
    	
    print ('Creating...' + name) 
    cv2.imwrite(name,cv_image)
	
def realsense_callback(data):
    global cv_image, countFlag, pos, ori, processFlag

    #convert msg to openCV Mat
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	
    #save img with filename showing pos&ori.
    if countFlag is 0 and processFlag is False:
        countFlag = 0
        processFlag = True
        rospy.loginfo("x_pos: %f, x_ori: %f", pos.x, ori.x)
		
		#save img name
        name = '../data/frame' + 'X' + str(pos.x) + 'Y'+ str(pos.y) + 'Z'+ str(pos.z) +'.jpg'
		
		#save im write
        print ('Creating...' + name) 
        cv2.imwrite(name,cv_image)
		
		#save pcloud into PCD format
    



# ================================================ result subscribe call back ================================================ #


            


def main():
    #initialize node
    rospy.init_node('image_save')
    rospy.Subscriber("chatter", Point, callback_pos)
    rospy.Subscriber("chatter_ori", Point, callback_ori)
    
    rospy.Subscriber("/camera/color/image_raw", Image, realsense_callback)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, realsense_callback_depth)
    
    # Declare pointcloud object, for calculating pointclouds and texture mappings
    pc = rs.pointcloud()
    # We want the points object to be persistent so we can display the last cloud when a frame drops
    points = rs.points()
    
    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()
    config = rs.config()
    
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    
    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    #Start streaming with default recommended configuration
    pipe.start(config);
    

    try:
    # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()
        
        # unaligned
        #depth_frame = frames.get_depth_frame() is a 640x360 depth image 
        #color_frame = frames.get_color_frame()
        
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()
        
        if not aligned_depth_frame or not color_frame:
			pass
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        

        points = pc.calculate(aligned_depth_frame)
        
        
        images = np.hstack((color_image, depth_image))
        cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Align Example', images)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            
            
        #name = "1"
        #print("Saving to 1.ply...")
        #points.export_to_ply(name + ".ply", color_frame)
        #cv2.imwrite(name + ".png",depth_image)
        #cv2.imwrite(name + ".jpg",color_image)
        #print("Done")
        
    finally:
	    pipe.stop()
	    processFlag = False
	    
    rospy.spin()
    
if __name__ == '__main__':
    main()
