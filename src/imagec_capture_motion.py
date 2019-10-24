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
import smach_ros
from smach import StateMachine
from std_msgs.msg import String
from std_msgs.msg import Int32
import time

#import rb5 action headers
import actionlib
import rb5_ros_wrapper.msg 
from rb5_ros_wrapper.msg import *

#rb5 goal data 
pose_coordinate = np.zeros(6)

for i in range(len(pose_coordinate)):
	pose_coordinate[i] = 0

rb5_goal = rb5_ros_wrapper.msg.MotionGoal(type = '', d0 = 0, d1 = 0, data = 0, coordinate = pose_coordinate , spd = 0, acc = 0)


#global var
result_flag = False


# ================================================ result subscribe call back ================================================ #

def base_result_callback(data):
    global base_result_flag
    rospy.loginfo("Base Callback result flag: %i", data.result.result_flag)
    base_result_flag = True

def arm_result_callback(data):
    global arm_result_flag
    rospy.loginfo("Arm Callback result flag: %i", data.result.result_flag)
    arm_result_flag = True

def gripper_result_callback(data):
    global gripper_result_flag
    rospy.loginfo("Gripper Callback result flag: %i", data.result.result_flag)
    gripper_result_flag = True

# ================================================ result subscribe call back ================================================ #

def set_goal_parameters(command, d0, data, coordinate, spd, acc):
	
	#rospy.loginfo("received command: %c" , command)
	rb5_goal.type = ord(command) #char action type
	rb5_goal.d0 = int(d0)
	rb5_goal.data = int(data)
	rb5_goal.coordinate[0] = float(coordinate[0])
	rb5_goal.coordinate[1] = float(coordinate[1])
	rb5_goal.coordinate[2] = float(coordinate[2])
	rb5_goal.coordinate[3] = float(coordinate[3])
	rb5_goal.coordinate[4] = float(coordinate[4])
	rb5_goal.coordinate[5] = float(coordinate[5])
	rb5_goal.spd = int(spd)
	rb5_goal.acc = int(acc)
	
	rospy.loginfo("data: %c, %d, %d, %f, %f", rb5_goal.type, rb5_goal.d0, rb5_goal.d1, rb5_goal.spd, rb5_goal.acc)
	rospy.loginfo("pose: %f, %f, %f, %f, %f, %f", rb5_goal.coordinate[0], rb5_goal.coordinate[1], rb5_goal.coordinate[2], rb5_goal.coordinate[3], rb5_goal.coordinate[4], rb5_goal.coordinate[5])


# ================================================ state machine states ================================================ #
# define state IDLE
class IDLE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
 
    def execute(self, userdata):
        #simple time countdown 
        timeCountSec = 3

        while timeCountSec:
            mins, secs = divmod(timeCountSec, 60)
            timeformat = '{:02d}:{:02d}'.format(mins, secs)
            print("time until action start: " + str(timeformat))
            time.sleep(1)
            timeCountSec -= 1

        print(" === Starting Store Application Motion ===")

        return 'outcome1' #go to init
    
# define state INIT
class INIT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
 

    def execute(self, userdata):
		global ac_cobot, rb5_goal
		rospy.loginfo('Executing state INIT')
		set_goal_parameters('I',0,0,pose_coordinate,0,0)
		ac_cobot.send_goal_and_wait(rb5_goal, rospy.Duration(10.0))
		
		time.sleep(1)
		
		set_goal_parameters('R',0,0,pose_coordinate,0,0)
		ac_cobot.send_goal_and_wait(rb5_goal, rospy.Duration(10.0))
		
		time.sleep(1)
		
		set_goal_parameters('V',0,0.65,pose_coordinate,0,0)
		ac_cobot.send_goal_and_wait(rb5_goal, rospy.Duration(10.0))
		
		time.sleep(1)
		

		return 'outcome1' #go to display
        
                
# ================================================ motion states ================================================ #

# define state GOTO_DISPLAY
class GOTO_DISPLAY(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GOTO_DISPLAY')
        global ac_cobot, result_flag, pose_coordinate
        
        pose_coordinate[0] = 300
        pose_coordinate[1] = -100
        pose_coordinate[2] = 275
        pose_coordinate[3] = 1.51
        pose_coordinate[4] = -1.37
        pose_coordinate[5] = 90

        set_goal_parameters('T',0,0,pose_coordinate,0.8,0.3)
        ac_cobot.send_goal_and_wait(rb5_goal, rospy.Duration(10.0))
        time.sleep(3)
        
        pose_coordinate[0] = 400
        pose_coordinate[1] = -40
        pose_coordinate[2] = 550
        pose_coordinate[3] = 45
        pose_coordinate[4] = 1.16
        pose_coordinate[5] = 90

        set_goal_parameters('T',0,0,pose_coordinate,0.8,0.3)
        ac_cobot.send_goal_and_wait(rb5_goal, rospy.Duration(10.0))
        
        time.sleep(3)
        rate = rospy.Rate(10) # 10hz
		
        # wait in loop until received done flags
        while(result_flag is False):
            rate.sleep()


        if (result_flag is True):

            return 'outcome1' #next
        
        else:
            
            return 'outcome2' #abort
        
# define state GOTO_DISPLAY
#class GOTO_DISPLAY(smach.State):
    #def __init__(self):
        #smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    #def execute(self, userdata):
        #rospy.loginfo('Executing state GOTO_DISPLAY')
        #global ac_cobot, base_result_flag



        ## Creates a base goal with parameters.
        #goal_base = ros_podo_connector.msg.RosPODO_BaseGoal(wheelmove_cmd = 1, MoveX = -3.0, MoveY = 0.0, ThetaDeg = 0 )
        #ac_base.send_goal(goal_base)


        ## Creates an arm goal for WST movement.
        #arnGoal_joint_ref = []
        #for i in range(18):
            #arnGoal_joint_ref.append( ros_podo_connector.msg.JointData(OnOffControl = 0) )


        ## desired parameters 
        #arnGoal_joint_ref[14].OnOffControl = 1
        #arnGoal_jac_cobotoint_ref[14].reference = -180
        #arnGoal_joint_ref[14].GoalmsTime = 5000

        #arnGoal_joint_ref[6].OnOffControl = 1
        #arnGoal_joint_ref[6].reference = -90
        #arnGoal_joint_ref[6].GoalmsTime = 5000

        ## send arm goal with desired parameters
        #goal_arm = ros_podo_connector.msg.RosPODO_ArmGoal(jointmove_cmd = 2, joint_ref = arnGoal_joint_ref)
        #ac_arm.send_goal(goal_arm)


        ## send gripper goal with desired parameters
        #goal_gripper = ros_podo_connector.msg.RosPODO_GripperGoal(grippermove_cmd = 2, mode = 1)
        #ac_gripper.send_goal(goal_gripper)
        #gripper_result_flag = False


        ## check for done flags
        #rospy.Subscriber("/rospodo_base/result",RosPODO_BaseActionResult, base_result_callback, queue_size=1, buff_size=10)
        #rospy.Subscriber("/rospodo_arm/result",RosPODO_ArmActionResult, arm_result_callback, queue_size=1, buff_size=10)

        #rate = rospy.Rate(10) # 10hz

        ## wait in loop until received done flags
        #while(base_result_flag is False or arm_result_flag is False):
            #rate.sleep()


        #if (base_result_flag is True and arm_result_flag is True):

            #return 'outcome1' #next
        
        #else:
            
            #return 'outcome2' #abort

# ================================================ main loop ================================================ #


def main():
    #initialize node
    rospy.init_node('rb5_action_FSMclient')
    #declare global var
    global  ac_cobot

    # Creates the SimpleActionClient, passing the type of the action (Motion) to the constructor.
    ac_cobot = actionlib.SimpleActionClient('motion', rb5_ros_wrapper.msg.MotionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    # ac_cobot.wait_for_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['ABORTED', 'RETURNED_SUCCESS', 'RETURNED_WITHOUT'])

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('IDLE', IDLE(), 
                               transitions={'outcome1':'INIT'})
                               
        smach.StateMachine.add('INIT', INIT(), 
                               transitions={'outcome1':'GOTO_DISPLAY'})                       

        smach.StateMachine.add('GOTO_DISPLAY', GOTO_DISPLAY(), 
                               transitions={'outcome1':'RETURNED_WITHOUT', 'outcome2':'ABORTED'})

    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    sis.stop()
   

if __name__ == '__main__':
    main()
