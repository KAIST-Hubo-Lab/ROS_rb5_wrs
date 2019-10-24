#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rb5_ros_wrapper/MotionAction.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_client_goal_state.h>
#include "lanros2podo.h"
#include <geometry_msgs/Point.h>

rb5_ros_wrapper::MotionGoal rb5_goal;
float xPos, yPos, zPos;

/*
void command(char type, float data, float coordinate[], float spd, float acc)
{
    rb5_goal.type = type;
    rb5_goal.data = data;
    for(int i = 0; i < 6; i++)
        rb5_goal.coordinate[i] = coordinate[i];
    rb5_goal.spd = spd;
    rb5_goal.acc = acc;
}*/
//rb5_ros_wrapper::MotionGoal
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
}

void setCoords(const geometry_msgs::Point::ConstPtr &msg)
{
    xPos = msg->x;
    yPos = msg->y;
    zPos = msg->z;
}

int main(int argc, char *argv[])
{

    float xMov, yMov, zMov;
        

    ros::init(argc, argv, "rb5_ros_client");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/position_of_object", 1, setCoords);

    actionlib::SimpleActionClient<rb5_ros_wrapper::MotionAction> ac_("motion", true);
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
    ROS_INFO("Action server started.");

    command('I',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Initializing.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('R',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Setting to real mode.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('V',0,0.65,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Changing speed.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    //opens on 1, closes on 2
    command('E',1,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Setting gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('F',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Stopping gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('J',0,0,-31.07, -20.94, 122.16, 78.74, -30.99, -90.99,0.8,0.3);
    ROS_INFO("Sending goal.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    ros::spinOnce();
    
    xPos = -125;
    yPos = -450;
    zPos = 200;
    
    
    xMov = xPos;
    yMov = yPos;
    zMov = zPos + 200;

    ROS_INFO("xMov LMY = %f", xMov);
    ROS_INFO("yMov LMY = %f", yMov);
    ROS_INFO("zMov LMY = %f", zMov);
	//LMY
    command('T',0,0, xMov, yMov, zMov,     45,      75, 45,0.8,0.3);
    //command('T',0,0, xMov, yMov, zMov,     90,      0, 90,0.8,0.3);
    ROS_INFO("Moving towards object.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    zMov = zPos;

	//LMY
    command('T',0,0, xMov, yMov, zMov,     45,      75, 45,0.8,0.3);
    //command('T',0,0, xMov, yMov, zMov,     90,      0, 90,0.8,0.3);
    ROS_INFO("Moving towards object.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('E',2,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Grabbing object.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('F',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Stopping gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('T',0,0,-15.78, -685.14,    470.7,     -179.82,      83.41, -179.82,0.8,0.3);
    ROS_INFO("Starting transport.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('T',0,0,806.04,     -0.01,    352.00,     180,      -90,      -90,0.8,0.3);
    ROS_INFO("Moving towards box.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));


    command('T',0,0,806.04,     -0.01,    120.00,     180,      -90,      -90,0.8,0.3);
    ROS_INFO("Dropping towards box.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('E',1,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Opening gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('F',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Stopping gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('T',0,0,806.04,     -0.01,    352.00,     180,      -90,      -90,0.8,0.3);
    ROS_INFO("Moving away from box.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('J',0,0,-31.07, -20.94, 122.16, 78.74, -30.99, -90.99,0.8,0.3);
    ROS_INFO("Action server ended, returning home.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
/*
    command('E',2,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Closing gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('F',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Stopping gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
*/
    command('P',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Waiting 1 second.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    ros::Time beginTime = ros::Time::now();
    ros::Time curTime;
    ros::Duration durTime;
    int continueFlag = 0;

    while(continueFlag == 0)
    {
        curTime = ros::Time::now();
        durTime = curTime - beginTime;

        if(durTime.toSec() > .5)
            continueFlag = 1;

        //ROS_INFO("durTime = ", durTime.toSec());
    }

    command('Q',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Starting again.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
/*
    //opens on 1, closes on 2
    command('E',1,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Setting gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('F',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Stopping gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
*/
    command('J',0,0,-31.07, -20.94, 122.16, 78.74, -30.99, -90.99,0.8,0.3);
    ROS_INFO("Sending goal.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    ros::spinOnce();
    xMov = xPos;
    yMov = yPos;
    zMov = zPos + 200;

    ROS_INFO("xMov LMY= %f", xMov);
    ROS_INFO("yMov LMY= %f", yMov);
    ROS_INFO("zMov LMY= %f", zMov);

    command('T',0,0, xMov, yMov, zMov,     90,      0, 90,0.8,0.3);
    ROS_INFO("Moving towards object.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    zMov = zPos;

    command('T',0,0, xMov, yMov, zMov,     90,      0, 90,0.8,0.3);
    ROS_INFO("Moving towards object.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('E',2,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Grabbing object.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('F',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Stopping gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('T',0,0,-15.78, -685.14,    470.7,     -179.82,      83.41, -179.82,0.8,0.3);
    ROS_INFO("Starting transport.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('T',0,0,806.04,     -0.01,    352.00,     180,      -90,      -90,0.8,0.3);
    ROS_INFO("Moving towards box.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));


    command('T',0,0,806.04,     -0.01,    120.00,     180,      -90,      -90,0.8,0.3);
    ROS_INFO("Dropping towards box.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('E',1,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Opening gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('F',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Stopping gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('T',0,0,806.04,     -0.01,    352.00,     180,      -90,      -90,0.8,0.3);
    ROS_INFO("Moving away from box.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('J',0,0,-31.07, -20.94, 122.16, 78.74, -30.99, -90.99,0.8,0.3);
    ROS_INFO("Action server ended, returning home.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('E',2,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Closing gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('F',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Closing gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));



    return 0;
}
