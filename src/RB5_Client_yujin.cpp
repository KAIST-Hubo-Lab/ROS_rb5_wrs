#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rb5_ros_wrapper/MotionAction.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include "lanros2podo.h"

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
char CMD_WheelMove = 'W';



class State
{
public:

    State() : ac_("motion", true)
    {

        ROS_INFO("Waiting for action server to start.");
        ac_.waitForServer();
        ROS_INFO("Action server started.");        
    }

    void doInit()
    {
        //need to initialize the robot before sending commands
        command(CMD_Initialize,0,0,0,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Initializing.");
        SendGoalandWait();

        //set mode
        command(CMD_RealMode,0,0,0,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Setting to real mode.");
        SendGoalandWait();

        //change speed to 0.6
        command(CMD_ChangeSpeed,0,0,0.6,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Changing speed.");
        SendGoalandWait();

        doTest();
        //move add
//        gofront();
//        //set to home
//        goHome();
//        GripperTest();

//        WheelTest();
    }
    void doTest()
        {
            //RB5 move pose1
            command(CMD_MoveTCP, 0,0,0, -123,-185,414,90,0,0,0.8,0.3);
            ROS_INFO("RB5 : pose1(TCP)");
            SendGoalandWait();

            closeGripper();

            //Wheel Move +x 1.0m
            commandwheel(CMD_WheelMove,0,1,0.5,0,0);
            ROS_INFO("Wheel move(GoalMode) : +1mx");
            SendGoalandWait();

            //RB5 move pose2
            command(CMD_MoveTCP, 0,0,0, -123,-385,414,90,0,0,0.8,0.3);
            ROS_INFO("RB5 : pose2(TCP)");
            SendGoalandWait();

            openGripper();

            //RB5 move pose3
            command(CMD_MoveTCP, 0,0,0, -123,-385,200,90,0,0,0.8,0.3);
            ROS_INFO("RB5 : pose3(TCP)");
            SendGoalandWait();

            closeGripper();

            //RB5 move pose2
            command(CMD_MoveTCP, 0,0,0, -123,-385,414,90,0,0,0.8,0.3);
            ROS_INFO("RB5 : pose2(TCP)");
            SendGoalandWait();

            //RB5 move pose1
            command(CMD_MoveTCP, 0,0,0, -123,-185,414,90,0,0,0.8,0.3);
            ROS_INFO("RB5 : pose1(TCP)");
            SendGoalandWait();

            ROS_INFO("All done");
        }

    void doneCb(const actionlib::SimpleClientGoalState& state,
                const rb5_ros_wrapper::MotionResultConstPtr& result)
    {
        ROS_INFO("Done");
    }

    void activeCb()
    {
//        ROS_INFO("Active");
    }

    void feedbackCb(const rb5_ros_wrapper::MotionFeedbackConstPtr& feedback)
    {
//        ROS_INFO("Feedback");
//        ROS_INFO("%f",feedback->joint_cur[0]);
    }

    void command(char type, int d0, int d1, float data, float coordinate0, float coordinate1, float coordinate2, float coordinate3, float coordinate4, float coordinate5, float spd, float acc)
    {
        rb5_goal.type = type;
        rb5_goal.d0 = d0;
        rb5_goal.d1 = d1;
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

    void commandwheel(char type, int d0, int d1, float x_m, float y_m, float yaw_deg)
    {
        rb5_goal.type = type;
        rb5_goal.d0 = d0;
        rb5_goal.d1 = d1;
        rb5_goal.wheel[0] = x_m;
        rb5_goal.wheel[1] = y_m;
        rb5_goal.wheel[2] = yaw_deg;
    }

    void SendGoalandWait()
    {
        ac_.sendGoal(rb5_goal, boost::bind(&State::doneCb, this, _1, _2), boost::bind(&State::activeCb, this), boost::bind(&State::feedbackCb, this, _1));
        ac_.waitForResult();
    }

private:

    rb5_ros_wrapper::MotionGoal rb5_goal;
    ros::NodeHandle n;
    Client ac_;


    int objectNum = 0;
    float objectPoints[3] = {0, 0, 0};

    void openGripper(int _msec=600);
    void closeGripper(int _msec=600);
    void goHome();
    void GripperTest();
    void gofront();
    void pause(float time);
    void resume();
    void WheelTest();
};

void State::openGripper(int _msec)
{
    if(_msec < 0 || _msec > 600)
        _msec = 600;

    //'E' command opens on 0,1, closes on 1,0
    command(CMD_MoveGripper,0,1,_msec,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Opening gripper.");
    SendGoalandWait();

    //will continue to open, so need to force a stop
//    command(CMD_StopGripper,0,0,0,0, 0, 0, 0, 0,0,0);
//    ROS_INFO("Gripper opened.");
//    SendGoalandWait();
}

void State::closeGripper(int _msec)
{
    if(_msec < 0 || _msec > 600)
        _msec = 600;
    //'E' command opens on 0,1, closes on 1,0
    command(CMD_MoveGripper,1,0,_msec,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Closing gripper.");
    SendGoalandWait();

    //will continue to close, so ne;ed to force a stop
//    command(CMD_StopGripper,0,0,0,0, 0, 0, 0, 0,0,0);
//    ROS_INFO("Gripper closed.");
//    SendGoalandWait();
}

void State::goHome()
{
    command(CMD_MoveJoint,0,0,0,-31.07, -20.94, 122.16, 78.74, -30.99, -90.99,0.8,0.3);
    ROS_INFO("Going home.");
    SendGoalandWait();

    closeGripper();

}

void State::GripperTest()
{
    closeGripper();

    for(int i=0;i<5;i++)
    {
        openGripper(170);
        usleep(4000*1000);
        closeGripper(170);
    }
//    openGripper(0);
//    closeGripper(0);
//    openGripper(170);
//    usleep(2000*1000);
//    closeGripper(170);
//    usleep(2000*1000);
//    openGripper(220);
//    usleep(2000*1000);
//    closeGripper(220);
//    usleep(2000*1000);
//    openGripper(270);
//    usleep(2000*1000);
//    closeGripper(270);
}

void State::WheelTest()
{
    commandwheel(CMD_WheelMove,0,1,1,0,0);
    ROS_INFO("Wheel move(GoalMode) : +1mx");
    SendGoalandWait();
    ROS_INFO("Wheel move done");
}

void State::gofront()
{
    command(CMD_MoveTCP,0,0,0,9.70, -509.51, 480.00, 34.57, 89.43, 34.56,0.8,0.3);
    ROS_INFO("Going home.");
    SendGoalandWait();
}


void State::pause(float time)
{
    command(CMD_MotionPause,0,0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Waiting ", time, " seconds.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    ros::Time beginTime = ros::Time::now();
    ros::Time curTime;
    ros::Duration durTime;
    int continueFlag = 0;

    while(continueFlag == 0)
    {
        curTime = ros::Time::now();
        durTime = curTime - beginTime;

        if(durTime.toSec() > time)
            continueFlag = 1;
    }

    resume();
}

void State::resume()
{
    command(CMD_MotionResume,0,0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Starting again.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rb5_client_yujin");

    State newState;
    ros::Subscriber _feedback;
    ros::NodeHandle n;
    newState.doInit();
    return 0;
}
