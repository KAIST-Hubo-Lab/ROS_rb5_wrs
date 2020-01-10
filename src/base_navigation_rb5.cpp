
/* =============================================================
 *
 * This ROS node is Base Navigation Client that sends base motions to ROSPodo Server for Mobile-RB5 Platform.
 * Sends base command with (1) velocity profile from path planning and (2) position profile from aruco marker alignment 
 * ROS Motion Action msgs {RB5}
 * 
 *
 * Output : /Motion/feedback
 *          /Motion/result
 * 			/Motion/status
 * 			/mobile_hubo/arrived_path

 * 
 * Input  : /mobile_hubo/navigation_path
 * 			/aruco_single/pose
 * 			/tf
 * 			/move_base_simple/goal
 * 			/navi_dummy/goal
 

 * E-mail : ml634@kaist.ac.kr     (Moonyoung Lee)

 *
 * Versions :
 * v2.0.2019.11
 * =============================================================
 */
 
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
/*custom defined Action header for robot motion */

//rb5
#include <rb5_ros_wrapper/MotionAction.h>
#include <rb5_ros_wrapper/MotionActionGoal.h>
#include <rb5_ros_wrapper/MotionActionResult.h>
#include <rb5_ros_wrapper/MotionActionFeedback.h>

/* ROS package include */
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include  <tf/tf.h>

#include <sensor_msgs/JointState.h>

/* transformation headers*/
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "tf/transform_datatypes.h"

#include <unistd.h>

/* include navi action file */
#include <mobile_path_planning/naviAction.h>

const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;
const float     PI   = 3.14159265;

#define arrival_threshold 2 //arrival grid size
#define averageWindowN 5    //sample number for low-pass filter
#define grid_resolution 0.1 //meter
#define marker_offset_z 0.932

#define MAX_TRANSLATION 5.0

/* ========= global variables ========== */
ros::Publisher path_pub; //path difference

float velocityxyzw [4][averageWindowN];
float vx_tx, vy_tx, vw_tx, vz_tx;
float vx_out_robot, vy_out_robot, vw_out_robot, vz_out_robot;

float sum_of_i = 0;

//initial rotate flag
int initial_robot_rotation_flag = 0;
float yaw_goal_deg, yaw_goal_rad;
float yaw_current_deg, yaw_current_rad;
float temp_rad;

//path move flag
int received_path = 0;
int arrived_goal = 0;
int move_flag = 0;

//marker align flag
int have_aligned = 0;
int marker_detected = 0;

int orientation_align_start = 0;
int orientation_aligned_finished = 0;

//marker pose related variables
float marker_x, marker_y, marker_z;
float marker_robot_x, marker_robot_y, marker_robot_z;
float roll_marker, pitch_marker, yaw_marker;

//state flags
int robot_move_request = 0;
int use_aruco_marker = 0;
int use_navi = 0;

//state machine var
int state_variable = 0;


ros::Time currentTime, beginTime;

geometry_msgs::Pose temporary_goal_pose;
geometry_msgs::Pose current_robot_pose;


/* ======================================= */

//test output csv data
#include <fstream>
std::ofstream outputFile;


/* receive goal pose from Navi Action */
void receive_goal_pose_action(const mobile_path_planning::naviActionGoalConstPtr &goal)
{
	ROS_INFO("===== CLIENT GOAL RECEIVED. START!!! ===== \n");
    //update flag to start new motion
    robot_move_request = 1;

    
    //get goal pose
    temporary_goal_pose.position.x = goal->goal.pose_x;
    temporary_goal_pose.position.y = goal->goal.pose_y;
    temporary_goal_pose.position.z = goal->goal.pose_z;
    temporary_goal_pose.orientation.x = goal->goal.ori_x;
    temporary_goal_pose.orientation.y = goal->goal.ori_y;
    temporary_goal_pose.orientation.z = goal->goal.ori_z;
    temporary_goal_pose.orientation.w = goal->goal.ori_w;
    

    use_navi = goal->goal.use_navi;
    use_aruco_marker = goal->goal.use_marker;
    
    ROS_INFO("use navi: %d ,use marker: %d\n", use_navi, use_aruco_marker);
    
    //check nan values
    if( isnanf(temporary_goal_pose.position.x) || isnanf(temporary_goal_pose.position.y) || isnanf(temporary_goal_pose.position.z) || isnanf(temporary_goal_pose.orientation.x) || isnanf(temporary_goal_pose.orientation.y) || isnanf(temporary_goal_pose.orientation.z) || isnanf(temporary_goal_pose.orientation.w) )
    {
		ROS_ERROR("received NAN value goal\n");
		temporary_goal_pose.position.x = 0;
		temporary_goal_pose.position.y = 0;
		temporary_goal_pose.position.z = 0;
		temporary_goal_pose.orientation.x = 0;
		temporary_goal_pose.orientation.y = 0;
		temporary_goal_pose.orientation.z = 0;
		temporary_goal_pose.orientation.w = 0;
		robot_move_request = 0;
	}
    
    //check large values
	if( (fabs(temporary_goal_pose.position.x)> MAX_TRANSLATION) || (fabs(temporary_goal_pose.position.y)> MAX_TRANSLATION) || (fabs(temporary_goal_pose.position.z)> MAX_TRANSLATION) )
	{
		ROS_ERROR("received goal pos greater than 5.0 meters\n");
		robot_move_request = 0;
	}
    
    
	//convert quaternion to euler
	tf::Quaternion quat_goal;
	tf::quaternionMsgToTF(temporary_goal_pose.orientation, quat_goal);
	double roll_goal, pitch_goal, yaw_goal;
	tf::Matrix3x3(quat_goal).getRPY(roll_goal, pitch_goal, yaw_goal);
	
	yaw_goal_rad = yaw_goal;
	yaw_goal_deg = yaw_goal * R2Df;
	
	ROS_INFO("NEW GOAL from Action!! goalX: %f, goalY: %f, goalTheta: %f",temporary_goal_pose.position.x, temporary_goal_pose.position.y, yaw_goal_deg);

}

/* receive goal pose from RVIZ (duplicate from path plan) */
void receive_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	
	////update flag to start new motion
	//initial_robot_rotation_flag = 1;
    //orientation_align_start = 1;
    //orientation_aligned_finished = 0;
    
    ////get goal pose
    //temporary_goal_pose = msg->pose;
    
    ////convert quaternion to euler
	//tf::Quaternion quat_goal;
	//tf::quaternionMsgToTF(msg->pose.orientation, quat_goal);
	//double roll_goal, pitch_goal, yaw_goal;
	//tf::Matrix3x3(quat_goal).getRPY(roll_goal, pitch_goal, yaw_goal);
	
	//yaw_goal_rad = yaw_goal;
	//yaw_goal_deg = yaw_goal * R2Df;
	
	//ROS_INFO("NEW GOAL from RViz!! goalX: %f, goalY: %f, goalTheta: %f",msg->pose.position.x, msg->pose.position.y, yaw_goal_deg);

}

/* map base velocity command from path size in 2D */
void velocity_from_path(const nav_msgs::Path::ConstPtr& msg)
{
    //ROS_INFO(" ============= received path CB =============");
    
     /*check for non-empty vector*/
    if(msg->poses.empty())
    {
		ROS_ERROR(" empty path");
		return;
	}
    

    int pathSize = msg->poses.size();
    received_path = 1;
    
    if(pathSize > arrival_threshold)
    {
		arrived_goal = 0;
		move_flag = 1;
		have_aligned = 0;
	}
	
	ROS_INFO("received path: %d, path size: %d, move flag: %d\n", received_path, pathSize, move_flag); 
	
        nav_msgs::Path pathDifferece; //vectory of differences in path
    geometry_msgs::PoseStamped temporaryPose; //used to fill
    
    //create pose difference array 
    for(int i = 0; i < pathSize-1; i++)
    {
		temporaryPose.pose.position.x = msg->poses[i+1].pose.position.x - msg->poses[i].pose.position.x;
		temporaryPose.pose.position.y = msg->poses[i+1].pose.position.y - msg->poses[i].pose.position.y;
		temporaryPose.pose.orientation.z = msg->poses[i+1].pose.orientation.z -msg->poses[i].pose.orientation.z;
		temporaryPose.pose.orientation.w = msg->poses[i+1].pose.orientation.w -msg->poses[i].pose.orientation.w;
		
		pathDifferece.poses.push_back(temporaryPose);
	}
    
    int pathSize2 = pathDifferece.poses.size();
    //ROS_INFO("path2 size: %d", pathDifferece.poses.size()); 
    
    path_pub.publish(pathDifferece);
    
    
    /* deltaX range [0.0~0.1]m */
    /* velocity range [0.0~1.0] */
    /* linearly scale delta to velocity output range*/
    float vx_ref, vy_ref, vz_ref, vw_ref;
    float vx_out, vy_out, vz_out, vw_out;
    
    
    //many samples remainining in path
    if (pathSize > averageWindowN ) 
    {
		
			if(pathDifferece.poses.empty())
		{
			ROS_ERROR("Difference path  empty!!!");
			return;
		}
		
		//ROS_INFO("Difference path size: %d", pathSize2); 
		
	
		for (int i =0; i < averageWindowN; i++) //sum the next window of path differences
		{
			vx_ref = vx_ref + pathDifferece.poses[i].pose.position.x;
			vy_ref = vy_ref + pathDifferece.poses[i].pose.position.y;
			vw_ref = vw_ref + pathDifferece.poses[i].pose.orientation.z;
			vz_ref = vz_ref + pathDifferece.poses[i].pose.orientation.w;
		}
		
			//average and then scale to unitless velocity
		vx_ref = vx_ref / averageWindowN * (1.0-0.0)/grid_resolution;
		vy_ref = vy_ref / averageWindowN * (1.0-0.0)/grid_resolution;
		vz_ref = vz_ref / averageWindowN * (1.0-0.0)/grid_resolution;
		vw_ref = vw_ref / averageWindowN * (1.0-0.0)/grid_resolution;
		
		//ROS_INFO("vx_ref: %f, vy_ref: %f\n", vx_ref, vy_ref); 
		
		//new reference at [end]
		velocityxyzw [0][averageWindowN-1] = vx_ref;
		velocityxyzw [1][averageWindowN-1] = vy_ref;
		velocityxyzw [2][averageWindowN-1] = vz_ref;
		velocityxyzw [3][averageWindowN-1] = vw_ref;
		
		
		/* to weight the LPF values by recent ones*/
		sum_of_i = 0;
		
		for(int i =1; i < averageWindowN+1; i++)
		{
			sum_of_i = sum_of_i + i;
			 
		}
			//ramp velocity using low-pass filter window
		for(int i =0; i < averageWindowN; i++)
		{
			
			vx_out = vx_out + ((i+1)/float(sum_of_i))*velocityxyzw[0][i];
			vy_out = vy_out + ((i+1)/float(sum_of_i))*velocityxyzw[1][i];
			vz_out = vz_out + ((i+1)/float(sum_of_i))*velocityxyzw[2][i];
			vw_out = vw_out + ((i+1)/float(sum_of_i))*velocityxyzw[3][i];
			
		}
	
		//update window
		for (int i =0; i < averageWindowN-1; i++)
		{
			velocityxyzw [0][i] = velocityxyzw [0][i+1];
			velocityxyzw [1][i] = velocityxyzw [1][i+1];
			velocityxyzw [2][i] = velocityxyzw [2][i+1];
			velocityxyzw [3][i] = velocityxyzw [3][i+1];
		}
	
	}
	
	
	//less than averageWindowN samples remaining in path. slow down
	else 
	{
		
		//slowing down
		if (pathSize > arrival_threshold)
		{
			 
			vx_out = pathDifferece.poses[0].pose.position.x * pathSize; // [-0.1 ~ 0.1] * [1 ~ 5]
			vy_out = pathDifferece.poses[0].pose.position.y * pathSize;
			vz_out = pathDifferece.poses[0].pose.orientation.z * pathSize;
			vw_out = pathDifferece.poses[0].pose.orientation.w * pathSize;
		}
		
		//arrived at goal
		else
		{
			vx_out = 0;
			vy_out = 0;
			vz_out = 0;
			vw_out = 0;
			//ROS_INFO("FINISHED STOP");
			arrived_goal = 1;
	
		}
	}
	
	/*account for global rotation w.r.t robot*/
	vx_out_robot = vx_out*cos(yaw_goal_rad) + vy_out*sin(yaw_goal_rad);
	vy_out_robot = -vx_out*sin(yaw_goal_rad) + vy_out*cos(yaw_goal_rad);

	/* bound max values*/
	if (vx_out_robot <= 1.0 and vx_out_robot >= -1.0) vx_out_robot = vx_out_robot;
	else if(vx_out_robot > 1.0 ) 		vx_out_robot = 1.0;
	else if(vx_out_robot < -1.0) 	vx_out_robot = -1.0;
	else vx_out_robot = 0;
	
	if(vy_out_robot <= 1.0 and vy_out_robot >= -1.0) 	vy_out_robot = vy_out_robot;
	else if(vy_out_robot > 1.0) 		vy_out_robot = 1.0;
	else if(vy_out_robot < -1.0) 	vy_out_robot = -1.0;
	else vy_out_robot = 0;
	
	//ROS_INFO("Computed robot vx: %f, vy: %f\n", vx_out_robot, vy_out_robot); 
	
}





/* align robot with aruco marker */
void position_from_marker(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	
	//move only when arrived at goal and user requested to use marker alignment
	if( (use_aruco_marker == 1) )
	{
		ROS_INFO("detected marker!");
		marker_detected = 1;
		//ROS_INFO("x: %.3f, y: %.3f, z:%.3f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
		marker_x = msg->pose.position.x;
		marker_y = msg->pose.position.y;
		marker_z = msg->pose.position.z;
		
		//convert marker to robot position in 2D application
		marker_robot_y = -1 * marker_x;
		marker_robot_x = -marker_offset_z + marker_z;
		
		
		//convert quaternion to euler
		tf::Quaternion quat;
		tf::quaternionMsgToTF(msg->pose.orientation, quat);
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		
		roll_marker = roll * R2Df;
		pitch_marker = -1*pitch;
		yaw_marker = yaw * R2Df;
		
		
		//ROS_INFO("roll: %.3f, pitch: %.3f, yaw:%.3f",roll_marker, pitch_marker*R2Df, yaw_marker);
		
		//data output test
		//outputFile << marker_x << "," <<  marker_y << "," <<  marker_z << "," << msg->pose.orientation.x << "," <<  msg->pose.orientation.y << "," <<  msg->pose.orientation.z<< "," << msg->pose.orientation.w << "," <<  roll_marker << "," <<  pitch_marker* R2Df << "," << yaw_marker <<std::endl;
	}

}


/* to prevent old marker values being stored when no marker detected*/
void reset_marker_values()
{
    //ROS_INFO("resetting marker pose data");

    marker_detected = 0;
    marker_x = 0;
    marker_y = 0;
    marker_z = 0;

    marker_robot_y = 0;
    marker_robot_x = 0;


    roll_marker = 0;
    pitch_marker = 0;
    yaw_marker = 0;

}

/* update current robot pose from TF subscribe */
void set_current_pose(const tf2_msgs::TFMessage::ConstPtr& _msg)
{
    // Convert tf2_msgs to geometry_msgs::Pose
    for(int i = 0; i < _msg->transforms.size(); i++){
        if(std::string("base_link").compare(_msg->transforms.at(i).child_frame_id) == 0)
        {
            current_robot_pose.position.x = _msg->transforms.at(i).transform.translation.x;
            current_robot_pose.position.y = _msg->transforms.at(i).transform.translation.y;
            current_robot_pose.position.z = 0.0;

            current_robot_pose.orientation.w = _msg->transforms.at(i).transform.rotation.w;
            current_robot_pose.orientation.x = _msg->transforms.at(i).transform.rotation.x;
            current_robot_pose.orientation.y = _msg->transforms.at(i).transform.rotation.y;
            current_robot_pose.orientation.z = _msg->transforms.at(i).transform.rotation.z;
			
            //convert quaternion to euler
            tf::Quaternion quat_goal;
            tf::quaternionMsgToTF(current_robot_pose.orientation, quat_goal);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat_goal).getRPY(roll, pitch, yaw);

            yaw_current_rad = yaw;
            yaw_current_deg = yaw * R2Df;
	
            break;
        }
    }

    //std::cout << "Set start pose" << std::endl;
}

/*===============================  main loop ================================================*/

int main (int argc, char **argv)
{
    ros::init(argc, argv, "moving_base_test");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<rb5_ros_wrapper::MotionAction> ac_motion("motion", true);
   
    // wait for the action server to start
    //ac_motion.waitForServer();      //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    // create goal instance
    rb5_ros_wrapper::MotionGoal      goal_motion;


    /* ============== Initialize ==============  */
    ros::NodeHandle n;
    ros::Subscriber key_input_sub = n.subscribe("/mobile_hubo/navigation_path",1,velocity_from_path);
    ros::Subscriber marker_sub = n.subscribe("/aruco_single/pose",1,position_from_marker);
    ros::Subscriber current_pose_subscriber = n.subscribe("/tf", 1, set_current_pose);

    
    //goal subscriber for 1. RVIZ 2. Navi Action
    ros::Subscriber goal_pose_subscriber = n.subscribe("/move_base_simple/goal", 10, receive_goal_pose);
    ros::Subscriber goal_pose_action_subscriber = n.subscribe("/hubo_navigation/goal", 1, receive_goal_pose_action);
    
    //publish after align with marker
    ros::Publisher arrived_publisher = n.advertise<geometry_msgs::PoseStamped>("mobile_hubo/arrived_path",1);
    
    
    ros::Rate loop_rate(10);
    
    
    //marker data output test
    //outputFile.open("marker_data.csv");
	//outputFile << "X" << "," << "Y" << ","  << "Z" << "," << "x" << "," <<  "y" << "," <<  "z" << ","  <<"w" << "," << "r" << ","  << "p"  << "," << "y" << std::endl;



    while(ros::ok())
    {
		
		//ROS_INFO("rotation flag: %d, received path: %d, move flag: %d\n", initial_robot_rotation_flag, received_path, move_flag); 

        
		
		/*state_variable
		 * 00 idle
		 * 01 no navi, no marker
		 * 02 no navi, yes marker align
		 * 10 yes navi, rotation motion 
		 * 11 yes navi, path follow 
		 * 12 yes navi, yes marker align
		 * 20 done
		*/
		switch(state_variable)
		{
			case(00):
				
				ROS_INFO("===== case 00: idle =====");
				//ROS_INFO("current yaw ?: %f\n",yaw_current_deg);
				
				if(robot_move_request)
				{
					
					if(use_navi == 0)
					{
						state_variable = 01;
					}
					
					else if(use_navi == 1)
					{
						state_variable = 10;
					}
					
					else
					{
						state_variable = 00;
					}	
				}
				break;
		
			case(01):
				ROS_INFO("===== case 01: no navi! goal movement =====");
				
				//send local goal 
				goal_motion.type = 'W';
				goal_motion.d0 = 0; 
				goal_motion.d1 = 1; 
				goal_motion.wheel[0] = 0; //moveX
				goal_motion.wheel[1] = 0; //moveY
			
				goal_motion.wheel[2] = yaw_goal_rad * R2Df;
                                if(goal_motion.wheel[2] != 0.)
                                {
                                    ROS_INFO("Rotation Motion x: %.3f, y: %.3f, theta: %.3f\n", goal_motion.wheel[0]  ,goal_motion.wheel[1] , goal_motion.wheel[2] );

                                    ac_motion.sendGoalAndWait(goal_motion, ros::Duration(5));
                                }
				
				//send local goal 
				goal_motion.type = 'W';
				goal_motion.d0 = 0; 
				goal_motion.d1 = 1; 
				goal_motion.wheel[0] = temporary_goal_pose.position.x*cos(yaw_goal_rad) + temporary_goal_pose.position.y*sin(yaw_goal_rad);
				goal_motion.wheel[1] = -temporary_goal_pose.position.x*sin(yaw_goal_rad) + temporary_goal_pose.position.y*cos(yaw_goal_rad);
				goal_motion.wheel[2] = 0;
				
                                if(goal_motion.wheel[0] != 0. || goal_motion.wheel[1] != 0.)
                                {
                                    ROS_INFO("Translation Motion x: %.3f, y: %.3f, theta: %.3f\n", goal_motion.wheel[0]  ,goal_motion.wheel[1] , goal_motion.wheel[2] );

                                    ac_motion.sendGoalAndWait(goal_motion, ros::Duration(5));

                                }
				
				if(use_aruco_marker == 1)
				{
					//state_variable = 02; //intermediate step due to update
					state_variable = 100;
					
					/*
					//send goal rotation(w.r.t. robot local frame)
					goal_motion.type = 'W';
					goal_motion.d0 = 0; 
					goal_motion.d1 = 1; 
					goal_motion.wheel[0] = 0;
					goal_motion.wheel[1] = 0;
					goal_motion.wheel[2] = 90-yaw_current_deg;
					ROS_INFO("current yaw NOW?: %f\n",yaw_current_deg);
	
					ROS_INFO("Rotation Motion theta: %.3f\n", goal_motion.wheel[2]); 
						
					ac_motion.sendGoalAndWait(goal_motion, ros::Duration(5));
					*/
					
				}
				
				else 
				{
					state_variable = 20;
				}
				
				//usleep(1000* 2 * 1000); 
				ros::spinOnce(); //update marker flag
				//ensure previous marker flag is off 
				reset_marker_values();
				break;
			
			case(100):
				ROS_INFO(" dummy state for updating marker flag");
				ros::spinOnce(); //update marker flag
				state_variable = 02;
				usleep(1000* 2 * 1000); 
				break;
				
			case(02):
				ros::spinOnce(); //update marker flag
				ROS_INFO("===== case 02 no navi! Align marker =====");
				
				if(marker_detected == 1)
				{
					//send goal (w.r.t. robot local frame)
					goal_motion.type = 'W';
					goal_motion.d0 = 0; 
					goal_motion.d1 = 1; 
					goal_motion.wheel[0] = marker_robot_x;
					goal_motion.wheel[1] = marker_robot_y;
					goal_motion.wheel[2] = 0;

					ROS_INFO("Translation Motion x: %.3f, y: %.3f\n", marker_robot_x , marker_robot_y); 
						
					ac_motion.sendGoalAndWait(goal_motion, ros::Duration(5));
		
					state_variable = 20;
				}
				
				else
				{
					ROS_INFO("couldn't detect marker. no motion.");
					state_variable = 20;
				}
				break;
				
			/* ============================ End of Local Movement ==================================== */
			
			
			/* ============================ Start of Global Movement (Navi) ==================================== */
			case(10):
				ROS_INFO("====== [CASE 10] navi! initial rotatae ====== ");
				
				//send rotate command
				goal_motion.type = 'W';
				goal_motion.d0 = 0; 
				goal_motion.d1 = 1; 
				goal_motion.wheel[0] = 0; //moveX
				goal_motion.wheel[1] = 0; //moveY
				
				temp_rad = yaw_goal_rad - yaw_current_rad;
				
				// [-PI, PI] range, bound 2PI turns from subtraction
				if(temp_rad >= PI) goal_motion.wheel[2] = temp_rad - 2*PI;		
				else if(temp_rad <= -1*PI) goal_motion.wheel[2]  = temp_rad + 2*PI;			
				//within bound
				else goal_motion.wheel[2] = temp_rad*R2Df;

				ROS_INFO("rotate x: %.3f, y: %.3f, thetaGoal: %.3f, thetaCurrent: %.3f, thetaMove: %.3f\n", goal_motion.wheel[0] ,goal_motion.wheel[1], yaw_goal_rad*R2Df, yaw_current_rad*R2Df, goal_motion.wheel[2]*R2Df); 
				
				//wait until done or timeout
				ac_motion.sendGoalAndWait(goal_motion, ros::Duration(5));
				
				
				ROS_INFO("1st Motion: initial rotation motion done");
				//send navi start command
				goal_motion.type = 'W';
				goal_motion.d0 = 1; 
				goal_motion.d1 = 0; 
				goal_motion.wheel[0] = 0; //moveX
				goal_motion.wheel[1] = 0; //moveY
				goal_motion.wheel[2] = 0; //moveTheta
				ac_motion.sendGoalAndWait(goal_motion, ros::Duration(5));
				
				state_variable = 11;
				ROS_INFO("====== [CASE 11] navi! follow path ====== ");
				
				break;
				
			case(11):
				//received path
				if(received_path == 1)
				{
					if(arrived_goal == 0 )
					{
						//send navi velocity commands
						goal_motion.type = 'N';
						goal_motion.wheel[0] = vx_out_robot; //moveX
						goal_motion.wheel[1] = vy_out_robot; //moveY
						ROS_INFO(" == 2nd Motion: Follow path vx: %f, vy: %f == \n", goal_motion.wheel[0] , goal_motion.wheel[1]); 

						ac_motion.sendGoal(goal_motion);
						
						
					}
					
					//arrived goal 
					else
					{
						
						//send STOP navi velocity command
						goal_motion.type = 'W';
						goal_motion.d0 = 1; 
						goal_motion.d1 = 1; 
						goal_motion.wheel[0] = 0; //moveX
						goal_motion.wheel[1] = 0; //moveY
						goal_motion.wheel[2] = 0; //moveTheta
						ROS_INFO("2nd Motion: STOP. velocity movement completed");
						
						ac_motion.sendGoal(goal_motion);
					
						if(use_aruco_marker == 1)
						{
							state_variable = 200;
						}
						else
						{
							state_variable = 20;
						}
					}
							
				}
				
				//haven't received path yet
				else
				{
					state_variable = 11;
				}
			
				break;
				
			case(200):
			
				//ROS_INFO("dummy state for updating marker flag");
				ros::spinOnce(); //update marker flag
				state_variable = 02;
				break;
				
			case(20):
				ROS_INFO("===== DONE MOTION ===== ");
				
				//publish arrival flag to stop planning 
				geometry_msgs::PoseStamped arrived;
				arrived.pose.position.x = 1;
				arrived_publisher.publish(arrived);
				
				//reset flags
				state_variable = 0;
				robot_move_request = 0;
				use_navi = 0;
				marker_detected = 0;
				
				break;
		}

		
		

        
		
		//update flags
		received_path = 0;
        

        ros::spinOnce();
        loop_rate.sleep();
    }

    //exit
    return 0;
}
