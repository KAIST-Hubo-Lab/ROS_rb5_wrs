#include <ros/ros.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <chrono>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rb5_ros_wrapper/MotionAction.h>
#include <rb5_ros_wrapper/manipulationAction.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include "lanros2podo.h"

const int OFFSET = 70;

typedef actionlib::SimpleActionClient<rb5_ros_wrapper::MotionAction> Client;

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

#define HOME_POSE       0
#define SEARCH_POSE     1
#define GRASP_OBJECT    2
#define PUT_OBJECT      3


#define KIMBAP1         0
#define KIMBAP2         1
#define SANDWICH        2
#define HAMBUG_THIN     3
#define HAMBUG_FAT      4
#define COFFEE          5
#define LUNCHBOX        6

class GraspAction
{
public:

    GraspAction() : ac_("motion", true)
    {
        ROS_INFO("Waiting for action server to start.");
        ac_.waitForServer();
        ROS_INFO("Action server started.");        
    }

    bool MotionDone = false;
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
        command(CMD_ChangeSpeed,0,0,0.2,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Changing speed.");
        SendGoalandWait();
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

    void SendGoalandWait()
    {
        ac_.sendGoal(rb5_goal, boost::bind(&GraspAction::doneCb, this, _1, _2), boost::bind(&GraspAction::activeCb, this), boost::bind(&GraspAction::feedbackCb, this, _1));
        ac_.waitForResult();
    }
    void goHome();
    void goSearch(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    void goGrasp(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    void goPut(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);

protected:

    rb5_ros_wrapper::MotionGoal rb5_goal;
    ros::NodeHandle n;
    rb5_ros_wrapper::manipulationFeedback feedback_;
    rb5_ros_wrapper::manipulationResult result_;
    Client ac_;

    int objectNum = 0;
    float objectPoints[3] = {0, 0, 0};
    float offset_kimbap[6] = {0.,};
    float offset_sandwich[6] = {0.,};
    float offset_hambug_thin[6] = {0.,};
    float offset_hambug_fat[6] = {0.,};
    float offset_coffee[6] = {0.,};
    float offset_lunchbox[6] = {0.,};
    float offset_approach_z = 200.;

    float x,y,z,r,p,oy;

    void openGripper(int _msec=600);
    void closeGripper(int _msec=600);
    void pause(float time);
    void resume();
    void WheelTest();
};

void GraspAction::openGripper(int _msec)
{
    if(_msec < 0 || _msec > 600)
        _msec = 600;

    //'E' command opens on 0,1, closes on 1,0
    command(CMD_MoveGripper,0,1,_msec,0,0, 0, 0, 0, 0,0,0);
    SendGoalandWait();
}

void GraspAction::closeGripper(int _msec)
{
    if(_msec < 0 || _msec > 600)
        _msec = 600;
    //'E' command opens on 0,1, closes on 1,0
    command(CMD_MoveGripper,1,0,_msec,0,0, 0, 0, 0, 0,0,0);
    SendGoalandWait();
}

void GraspAction::goHome()
{
    command(CMD_MoveJoint,0,0,0,-92.05,3.76,97.56,-11.32,90.01,2.05,0.8,0.3);
    ROS_INFO("Going home.\n");
    SendGoalandWait();
    closeGripper();
    MotionDone = true;
}

void GraspAction::goSearch(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    if(_goal->motion_cmd != SEARCH_POSE)
        return;

    command(CMD_MoveTCP,0,0,0,0,-400,300, 0.,0.,0.,0.8,0.3);
    ROS_INFO("Going Search pose.\n");

    SendGoalandWait();

    MotionDone = true;
}

void GraspAction::goGrasp(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    if(_goal->motion_cmd != GRASP_OBJECT)
        return;

    x = _goal->pose_x;     //mm
    y = _goal->pose_y;
    z = _goal->pose_z;
    r = _goal->ori_r;      //deg
    p = _goal->ori_p;
    oy = _goal->ori_y;
    printf("Object Pose = %f, %f, %f, %f, %f, %f\n",x, y, z, r, p, oy);

    float offset[6];
    switch(_goal->object_id)
    {
    case KIMBAP1:
        ROS_INFO("Object = KIMBAP1\n");
        memcpy(&offset,offset_kimbap,sizeof(float)*6);
        break;
    case KIMBAP2:
        ROS_INFO("Object = KIMBAP2\n");
        memcpy(&offset,offset_kimbap,sizeof(float)*6);
        break;
    case SANDWICH:
        ROS_INFO("Object = SANDWICH\n");
        memcpy(&offset,offset_sandwich,sizeof(float)*6);
        break;
    case HAMBUG_FAT:
        ROS_INFO("Object = HAMBUG_FAT\n");
        memcpy(&offset,offset_hambug_fat,sizeof(float)*6);
        break;
    case HAMBUG_THIN:
        ROS_INFO("Object = HAMBUG_THIN\n");
        memcpy(&offset,offset_hambug_thin,sizeof(float)*6);
        break;
    case COFFEE:
        ROS_INFO("Object = COFFEE\n");
        memcpy(&offset,offset_coffee,sizeof(float)*6);
        break;
    case LUNCHBOX:
        ROS_INFO("Object = LUNCHBOX\n");
        memcpy(&offset,offset_lunchbox,sizeof(float)*6);
        break;
    default:
        break;
    }

    x -= offset[0];
    y -= offset[1];
    z -= offset[2];
    r  = 90.0;//-= offset[3]; (assume that roll, pitch of object doesn't change)
    p  = 0.;  //-= offset[4];
    oy-= offset[5];

    //-------------------Approach---------------------//
    //approach z offset + 20cm
    z += offset_approach_z;

    ROS_INFO("GRASP::Approach pose");
    printf("Approach Pose = %f, %f, %f, %f, %f, %f\n\n",x, y, z, r, p, oy);
    command(CMD_MoveTCP,0,0,0,x,y,z,r,p,oy,0.6,0.3);
    SendGoalandWait();

    ROS_INFO("GRASP::Close Gripper");
    openGripper();

    //-------------------Grasp---------------------//
    //z offset
    z -= offset_approach_z;

    ROS_INFO("GRASP::Grasp pose");
    printf("Grasp Pose = %f, %f, %f, %f, %f, %f\n\n",x, y, z, r, p, oy);
    command(CMD_MoveTCP,0,0,0,x,y,z,r,p,oy,0.6,0.3);
    SendGoalandWait();


    ROS_INFO("GRASP::Close Gripper");
    closeGripper();

    //-------------------Lift---------------------//
    //Lift z offset + 20cm
    z += offset_approach_z;

    ROS_INFO("GRASP::Lift pose");
    printf("Lift Pose = %f, %f, %f, %f, %f, %f\n\n",x, y, z, r, p, oy);
    command(CMD_MoveTCP,0,0,0,x,y,z,r,p,oy,0.6,0.3);
    SendGoalandWait();

    ROS_INFO("GRASP::Grasp Completed ;D\n\n");

    MotionDone = true;
}

void GraspAction::goPut(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    if(_goal->motion_cmd != PUT_OBJECT)
        return;

    //-------------------Put---------------------//
    x += 200.;
    z -= offset_approach_z;
    ROS_INFO("GRASP::Put pose (manually -x 15cm)");
    printf("Put Pose = %f, %f, %f, %f, %f, %f\n\n",x, y, z, r, p, oy);
    command(CMD_MoveTCP,0,0,0,x,y,z,r,p,oy,0.6,0.3);
    SendGoalandWait();


    ROS_INFO("GRASP::Open Gripper\n");
    openGripper();


    //-------------------Up---------------------//
    z += offset_approach_z;
    command(CMD_MoveTCP,0,0,0,x,y,z,r,p,oy,0.6,0.3);
    SendGoalandWait();

    ROS_INFO("GRASP::Put Completed ;D\n\n");

    MotionDone = true;
}

void GraspAction::pause(float time)
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

void GraspAction::resume()
{
    command(CMD_MotionResume,0,0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Starting again.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
}




//=========================================================================//

class manipulationAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<rb5_ros_wrapper::manipulationAction> as_;
    std::string action_name_;
    GraspAction newGrasp;

    rb5_ros_wrapper::manipulationFeedback feedback_;
    rb5_ros_wrapper::manipulationResult result_;

public:

    manipulationAction(std::string name) :
        as_(nh_, name, boost::bind(&manipulationAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~manipulationAction(void)
    {
    }

    void writeTX(const rb5_ros_wrapper::manipulationGoalConstPtr &goal)
    {
        return;
    }

//    //only called when client requests goal
    void executeCB(const rb5_ros_wrapper::manipulationGoalConstPtr &goal)
    {
        newGrasp.doInit();

        switch(goal->motion_cmd)
        {
        case HOME_POSE:
            ROS_INFO("New CMD : Home Pose");
            newGrasp.goHome();
            break;
        case SEARCH_POSE:
            ROS_INFO("New CMD : Search Pose");
            newGrasp.goSearch(goal);
            break;
        case GRASP_OBJECT:
            ROS_INFO("New CMD : Grasp Pose");
            newGrasp.goGrasp(goal);
            break;
        case PUT_OBJECT:
            ROS_INFO("New CMD : Put Pose");
            newGrasp.goPut(goal);
            break;
        default:
            ROS_INFO("trash");
            break;
        }

        while(newGrasp.MotionDone == false)
        {
            if(as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                break;
            }

            if(as_.isActive())
            {
                feedback_.dummy = 0;
                as_.publishFeedback(feedback_);
            }
        }

        ROS_INFO("Send result");
        result_.result_flag = 1;
        as_.setSucceeded(result_);
        newGrasp.MotionDone = false;
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grasp_motion");

    manipulationAction manipulation("manipulation");

    ros::Subscriber _feedback;
    ros::NodeHandle n;

    ros::spin();
    return 0;
}
