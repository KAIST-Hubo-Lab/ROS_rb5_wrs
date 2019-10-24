#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rb5_ros_wrapper/MotionAction.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include "lanros2podo.h"
#include <math.h>
#include "std_msgs/String.h"



const int OFFSET = 70;

typedef actionlib::SimpleActionClient<rb5_ros_wrapper::MotionAction> Client;

////THIS IS NOT DONE. I STILL NEED TO IMPLEMENT COLLISION CHECKS AND SINGULARITY CHECKS
////Note ctrl-c will not kill the node
////Also, please make sure that you close the action client after you exit the action server
////The Robot will need to initialize again

char CMD_Initialize = 'I';
char CMD_RealMode = 'R';
char CMD_SimulationMode = 'S';
char CMD_MoveJoint = 'J';
char CMD_MoveTCP = 'T';
char CMD_MoveBlend = 'B';
char CMD_ChangeSpeed = 'V';
char CMD_MoveGripper = 'E';
char CMD_StopGripper = 'F';
char CMD_MotionPause = 'P';
char CMD_MotionHalt = 'H';
char CMD_MotionResume = 'Q';
char CMD_CollisionResume = 'C';

ros::Publisher chatter_pub;
ros::Publisher chatter_pub_ori;

geometry_msgs::Point pos;
geometry_msgs::Point ori;

class State
{
public:

    State() : ac_("motion", true)
    {

        ROS_INFO("Waiting for action server to start.");
        ac_.waitForServer();
        ROS_INFO("Action server started.");


        //need to initialize the robot before sending commands
        command(CMD_Initialize,0,0,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Initializing.");
        ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

        //set modeblishing Feedback

        command(CMD_RealMode,0,0,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Setting to real mode.");
        ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

        //change speed to 0.6
        command(CMD_ChangeSpeed,0,0.6,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Changing speed.");
        ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

        //set to home
        startMotion();
    }

    void command(char type, int d0, float data, float coordinate0, float coordinate1, float coordinate2, float coordinate3, float coordinate4, float coordinate5, float spd, float acc)
    {
        rb5_goal.type = type;
        rb5_goal.d0 = d0;
        rb5_goal.data = data;
        rb5_goal.coordinate[0] = coordinate0;
        rb5_goal.coordinate[1] = coordinate1;
        rb5_goal.coordinate[2] = coordinate2;
        rb5_goal.coordinate[3] = coordinate3;
        rb5_goal.coordinate[4] = coordinate4;
        rb5_goal.coordinate[5] = coordinate5;
        rb5_goal.spd = spd;
        rb5_goal.acc = acc;
        
        ROS_INFO("cmd:%c, 0: %f, 1: %f, 2:%f, 3:%f, 4: %f, 5: %f", type, coordinate0, coordinate1,coordinate2,coordinate3,coordinate4,coordinate5);

    }

private:

    rb5_ros_wrapper::MotionGoal rb5_goal;
    
  
    Client ac_;


    int objectNum = 0;
    float objectPoints[3] = {0, 0, 0};


    void openGripper();
    void closeGripper();
    void goHome();
    void startMotion();
    void captureMotion();
    void pause(float time);
    void resume();
    void requestData();
};


float pi = M_PI;
float deg2rad = 2*pi/360;
float minRad = 0*deg2rad;
float maxRad = 45*deg2rad;
float gapRad = 5*deg2rad;

float distance_obj2gripper = 550;
float x = 0;
float y = 0;
float z = 0;
float radZ = 0;
float radX = 0;

float objX = 800;
float objY = -150;  
float objZ = 220; 
float gripper2camera = 70;

float startX = objX - distance_obj2gripper;
float startY = objY;
float startZ = objZ;
float startRadz = 90;

float testvar = 0;
float testvar2 = 0;

float safeJoint_base = -13;
float safeJoint_shoulder = -16;
float safeJoint_elbow = 120;
float safeJoint_wristx = 73;
float safeJoint_wristy = -103;
float safeJoint_wristz = -180;

void State::captureMotion()
{
    command(CMD_MoveTCP,0,0,startX, startY, startZ, 0, 0, startRadz,0.5,0.3);
    ROS_INFO("Setting to initial image capture position.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));


    ROS_INFO("Moving in Spherical Motion around object.");
    
    for(int heightAngle = 0; heightAngle <= 30; heightAngle = heightAngle + 5)
    {
		z = startZ + distance_obj2gripper * sin(heightAngle*deg2rad);
		radX = heightAngle;
		
		command(CMD_MoveTCP,0,0,startX, startY, z, radX, 0, startRadz,0.5,0.3);
		ROS_INFO("angle: %d, z: %f, radX:%f ", heightAngle, z, radX);
		ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
			
		
		/*
		 * to avoid collision but happens anyways after couple movements in tcp
		if(heightAngle > 5)
		{
			command(CMD_MoveJoint,0, 0,-53.8, 31.4, 80.7, 12.1, -192.3, -235.4,0.5,0.3);
			ROS_INFO("joints to avoid collision");
			ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
		}
		*/
		

		
		for (int circleAngle = -30; circleAngle <= 30; circleAngle = circleAngle + 5)
		{
		
		
			//x = objX - distance_obj2gripper * cos(circleAngle*deg2rad);
			//y = (startY) + distance_obj2gripper * sin(circleAngle*deg2rad);
			
			x = objX - (distance_obj2gripper * cos(circleAngle*deg2rad))*cos(heightAngle*deg2rad);
			y = (startY) + (distance_obj2gripper * sin(circleAngle*deg2rad))*cos(heightAngle*deg2rad) ;
			
			radZ = startRadz - circleAngle;
			
			command(CMD_MoveTCP,0,0,x, y, z, radX, 0, radZ,0.5,0.3);
			//command(CMD_MoveTCP,0,0,x, y, startZ, 0, 0, radZ,0.5,0.3);
			
			//ROS_INFO("heightAngle:%d, circleAngle: %d, x: %f, y:%f, radZ:%f ", heightAngle, circleAngle, x, y, radZ);
			ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
			
			pos.x = ((distance_obj2gripper-gripper2camera) * cos(circleAngle*deg2rad))*cos(heightAngle*deg2rad);
			pos.y = ((distance_obj2gripper-gripper2camera) * sin(circleAngle*deg2rad))*cos(heightAngle*deg2rad);
			pos.z = z - objZ + 120;
			
			ori.x = radX;
			ori.z = radZ;
			
			chatter_pub.publish(pos);
			chatter_pub_ori.publish(ori);
			ros::Duration(1).sleep();
		

		}
		
		ROS_INFO(" ========== finished XY plane =========== ");
		goHome(); //reset joints by using joint space values 
		
		
		
		
	}
	
    
    
    


}


void State::goHome()
{
    command(CMD_MoveJoint,0,0,safeJoint_base, safeJoint_shoulder, safeJoint_elbow, safeJoint_wristx, safeJoint_wristy, safeJoint_wristz,0.5,0.3);
    ROS_INFO("Going home. Setting 0 joint");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
    
    command(CMD_MoveJoint,0,0,0, 15, 130, 25, -80, -180,0.5,0.3);
    ROS_INFO("Going home. Setting default joint");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));


}

void State::startMotion()
{
    command(CMD_MoveJoint,0,0,safeJoint_base, safeJoint_shoulder, safeJoint_elbow, safeJoint_wristx, safeJoint_wristy, safeJoint_wristz,0.5,0.3);
    ROS_INFO("Going home. Setting 0 joint");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
    
    command(CMD_MoveJoint,0,0,0, 15, 130, 25, -80, -180,0.5,0.3);
    ROS_INFO("Going home. Setting default joint");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
    
    captureMotion();


}


//void State::openGripper()
//{
    ////'E' command opens on 1, closes on 2
    //command(CMD_MoveGripper,1,0,0,0, 0, 0, 0, 0,0,0);
    //ROS_INFO("Opening gripper.");
    //ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    ////will continue to open, so need to force a stop
    //command(CMD_StopGripper,0,0,0,0, 0, 0, 0, 0,0,0);
    //ROS_INFO("Gripper opened.");
    //ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
//}

//void State::closeGripper()
//{
    ////'E' command opens on 1, closes on 2
    //command(CMD_MoveGripper,2,0,0,0, 0, 0, 0, 0,0,0);
    //ROS_INFO("Closing gripper.");
    //ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    ////will continue to close, so need to force a stop
    //command(CMD_StopGripper,0,0,0,0, 0, 0, 0, 0,0,0);
    //ROS_INFO("Gripper closed.");
    //ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
//}



//void State::pause(float time)
//{
    //command(CMD_MotionPause,0,0,0,0, 0, 0, 0, 0,0,0);
    //ROS_INFO("Waiting ", time, " seconds.");
    //ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    //ros::Time beginTime = ros::Time::now();
    //ros::Time curTime;
    //ros::Duration durTime;
    //int continueFlag = 0;

    //while(continueFlag == 0)
    //{
        //curTime = ros::Time::now();
        //durTime = curTime - beginTime;

        //if(durTime.toSec() > time)
            //continueFlag = 1;
    //}

    //resume();
//}

void State::resume()
{
    command(CMD_MotionResume,0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Starting again.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rb5_ros_client");
    ros::NodeHandle n;
    
    
     chatter_pub = n.advertise<geometry_msgs::Point>("chatter", 1000);
	 chatter_pub_ori = n.advertise<geometry_msgs::Point>("chatter_ori", 1000);

	
    State newState;

    return 0;
}
