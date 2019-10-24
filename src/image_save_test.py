# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Subscribe to position values
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

#======== global variables ========#
pos = Point(x=0,y=0,z=0)
ori = Point(x=0,y=0,z=0)
countFlag = 0
processFlag = True


#======== ROS callback functions ========#
def callback_pos(data):
    global pos, countFlag
    #rospy.loginfo("x: %f, y: %f, z: %f", data.x, data.y, data.z)
    
        
    pos.x = data.x
    pos.y = data.y
    pos.z = data.z
	
    countFlag = countFlag + 1


def callback_ori(data):
    global ori, countFlag
    #rospy.loginfo("xt: %f, yt: %f, zt: %f", data.x, data.y, data.z)
	
	
    ori.x = data.x
    ori.y = data.y
    ori.z = data.z
    countFlag = countFlag + 1
	
#======== realsense SDK configure ========#	
# Create a pipeline
pipeline = rs.pipeline()

# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()
    
#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)


# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)


#======== ROS initialize ========#
rospy.init_node('image_save')
rospy.Subscriber("chatter", Point, callback_pos)
rospy.Subscriber("chatter_ori", Point, callback_ori)
    
looprate = rospy.Rate(1)

# Streaming loop
try:
    while not rospy.is_shutdown():
		if countFlag == 2:
			
			#save when both topics received
			countFlag = 0
			
	        # Get frameset of color and depth
	        frames = pipeline.wait_for_frames()
	        # frames.get_depth_frame() is a 640x360 depth image
	
	        # Align the depth frame to color frame
	        aligned_frames = align.process(frames)
	
	        # Get data from aligned frames
	        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
	        color_frame = aligned_frames.get_color_frame()
	        
	        points = pc.calculate(aligned_depth_frame)
	
	        # Validate that both frames are valid
	        if not aligned_depth_frame or not color_frame:
	            continue
	
	        depth_image = np.asanyarray(aligned_depth_frame.get_data())
	        color_image = np.asanyarray(color_frame.get_data())
	        
	        #print("size depth: ",depth_image.shape)
	        #print("size depth: ",color_image.shape)
	        
	        #save name according to pos/ori
	        #name = "X" + str(pos.x) + "Y" + str(pos.y) + "Z" + str(pos.z) + "X" + str(ori.x) + "Y" + str(ori.y) + "Z" + str(ori.z)
	        name = "../data/" + "X" + "{:.1f}".format(pos.x) + "Y" + "{:.1f}".format(pos.y) + "Z" + "{:.1f}".format(pos.z) + "X" + "{:.1f}".format(ori.x) + "Y" + "{:.1f}".format(ori.y) + "Z" + "{:.1f}".format(ori.z)


	        # Save data
	        cv2.imwrite(name + ".png",depth_image)
	        cv2.imwrite(name + ".jpg",color_image)
	        points.export_to_ply(name + ".ply", aligned_depth_frame)
	        rospy.loginfo("saving file name: " + name)
        
		looprate.sleep()
        
finally:
    pipeline.stop()
