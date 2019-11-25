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
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "rb5_ros_wrapper/update.h"

#define D2R             0.0174533
#define R2D             57.2958

const int OFFSET = 70;

typedef actionlib::SimpleActionClient<rb5_ros_wrapper::MotionAction> Client;
//rb5_ros_wrapper::updatePtr robot_states;
tf::TransformListener listener;
tf::StampedTransform transform;

char CMD_Initialize = 'I';
char CMD_ChangeMode = 'M';
char CMD_Suction = 'S';
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

//motion_cmd
enum{
    HOME_POSE = 0, SEARCH_POSE, GRASP_OBJECT, PUT_OBJECT, PULL_SHELF, PUSH_SHELF, TEST_MARKER
};

//shelf_state
enum{
    SEARCH = 0, PULL
};

//object_id
enum{
    KIMBAP1 = 0, KIMBAP2, SANDWICH, HAMBUG_THIN, HAMBUG_FAT, COFFEE, LUNCHBOX
};

//tool_id
enum{
    TOOLMODE_GRIPPER = 0, TOOLMODE_SUCTION
};

//result
enum{
    SUCCESSED = 1, ROBOT_STATE_ERROR, CMD_ERROR, GOAL_ERROR
};

//object_state
#define UNKNOWN                 0

float joint_shelf_init[6] = {-73.997, -8.921, 128.944, -30.022, 90.010, 163.990};
float joint_shelf_search1[6] = {-67.245, 11.202, 141.470, -152.670, 67.234, -0.005};    //pose
float joint_shelf_search2[6] = {-67.245, 11.202, 141.470, -152.670, 67.234, -0.005};
float joint_shelf_search3[6] = {-67.245, 11.202, 141.470, -152.670, 67.234, -0.005};

/*
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
        command(CMD_ChangeMode,0,0,1,0,0, 0, 0, 0, 0,0,0);
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
        ac_.sendGoal(rb5_goal, boost::bind(&GraspAction::doneCb, this, _1, _2), boost::bind(&GraspAction::activeCb, this), boost::bind(&GraspAction::feedbackCb, this, _1));
        ac_.waitForResult();
    }
    void goHome();
    void goSearch(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    void goGraspGripper(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    void goGraspSuction(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    void goPutGripper(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    void goPutSuction(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    int pullshelf(const rb5_ros_wrapper::manipulationGoalConstPtr _goal, int _state);
    int pullshelf(int _state);
    void pushshelf(const rb5_ros_wrapper::manipulationGoalConstPtr _goal, int _state);
    void testMarker(int _mode);
    void GrippertoRB5();
    void SuctiontoRB5();

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

    float x_real, y_real, z_real, r_real, p_real, oy_real;

    tf::Matrix3x3 Rtarget, Robject;
    tf::Vector3   Ptarget, Pobject;

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

    command(CMD_MoveTCP,0,0,0,250,419,381, 90.,0.,90.,0.2,0.1);
    ROS_INFO("Going Search pose.\n");

    SendGoalandWait();

    MotionDone = true;
}

void GraspAction::goGraspGripper(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
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
    default:
        ROS_INFO("Object id error");
        return;
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
    GrippertoRB5();
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.6,0.3);
    SendGoalandWait();

    ROS_INFO("GRASP::Close Gripper");
    openGripper();

    //-------------------Grasp---------------------//
    //z offset
    z -= offset_approach_z;

    ROS_INFO("GRASP::Grasp pose");
    printf("Grasp Pose = %f, %f, %f, %f, %f, %f\n\n",x, y, z, r, p, oy);
    GrippertoRB5();
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.6,0.3);
    SendGoalandWait();

    ROS_INFO("GRASP::Close Gripper");
    closeGripper();

    //-------------------Lift---------------------//
    //Lift z offset + 20cm
    z += offset_approach_z;

    ROS_INFO("GRASP::Lift pose");
    printf("Lift Pose = %f, %f, %f, %f, %f, %f\n\n",x, y, z, r, p, oy);
    GrippertoRB5();
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.6,0.3);
    SendGoalandWait();

    ROS_INFO("GRASP::Grasp Completed ;D\n\n");

    MotionDone = true;
}

void GraspAction::goGraspSuction(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
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
        ROS_INFO("Object id error");
        return;
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
    SuctiontoRB5();
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.6,0.3);
    SendGoalandWait();

    ROS_INFO("GRASP::Close Gripper");
    openGripper();

    //-------------------Grasp---------------------//
    //z offset
    z -= offset_approach_z;

    ROS_INFO("GRASP::Grasp pose");
    printf("Grasp Pose = %f, %f, %f, %f, %f, %f\n\n",x, y, z, r, p, oy);
    SuctiontoRB5();
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.6,0.3);
    SendGoalandWait();


    ROS_INFO("GRASP::Close Gripper");
    closeGripper();

    //-------------------Lift---------------------//
    //Lift z offset + 20cm
    z += offset_approach_z;

    ROS_INFO("GRASP::Lift pose");
    printf("Lift Pose = %f, %f, %f, %f, %f, %f\n\n",x, y, z, r, p, oy);
    SuctiontoRB5();
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.6,0.3);
    SendGoalandWait();

    ROS_INFO("GRASP::Grasp Completed ;D\n\n");

    MotionDone = true;
}

void GraspAction::goPutGripper(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    if(_goal->motion_cmd != PUT_OBJECT)
        return;

    //-------------------Put---------------------//
    x += 200.;
    z -= offset_approach_z;
    y = 0.;
    ROS_INFO("GRASP::Put pose (manually -x 15cm)");
    printf("Put Pose = %f, %f, %f, %f, %f, %f\n\n",x, y, z, r, p, oy);
    GrippertoRB5();
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.6,0.3);
    SendGoalandWait();


    ROS_INFO("GRASP::Open Gripper\n");
    openGripper();


    //-------------------Up---------------------//
    z += offset_approach_z;
    GrippertoRB5();
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.6,0.3);
    SendGoalandWait();

    ROS_INFO("GRASP::Put Completed ;D\n\n");

    MotionDone = true;
}

void GraspAction::goPutSuction(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    if(_goal->motion_cmd != PUT_OBJECT)
        return;

    //-------------------Put---------------------//
    x += 200.;
    z -= offset_approach_z;
    y = 0.;
    ROS_INFO("GRASP::Put pose (manually -x 15cm)");
    printf("Put Pose = %f, %f, %f, %f, %f, %f\n\n",x, y, z, r, p, oy);
    SuctiontoRB5();
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.6,0.3);
    SendGoalandWait();


    ROS_INFO("GRASP::Open Gripper\n");
    openGripper();


    //-------------------Up---------------------//
    z += offset_approach_z;
    SuctiontoRB5();
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.6,0.3);
    SendGoalandWait();

    ROS_INFO("GRASP::Put Completed ;D\n\n");

    MotionDone = true;
}
*/
//int GraspAction::pullshelf(const rb5_ros_wrapper::manipulationGoalConstPtr _goal, int _state)
//{
//    if(_goal->motion_cmd != PULL_SHELF)
//        return -1;

//    switch(_state)
//    {
//    case SEARCH:
//    {
//        //-----------------search pose------------------------//
//        ROS_INFO("PULL SHELF::Search pose");
//        switch(_goal->shelf_id)
//        {
//        case 1:
//            command(CMD_MoveJoint,0,0,0,joint_shelf_search1[0],joint_shelf_search1[1],joint_shelf_search1[2],joint_shelf_search1[3],joint_shelf_search1[4],joint_shelf_search1[5],0.3,0.3);
//            SendGoalandWait();
//            break;
//        case 2:
//            command(CMD_MoveJoint,0,0,0,joint_shelf_search2[0],joint_shelf_search2[1],joint_shelf_search2[2],joint_shelf_search2[3],joint_shelf_search2[4],joint_shelf_search2[5],0.3,0.3);
//            SendGoalandWait();
//            break;
//        case 3:
//            command(CMD_MoveJoint,0,0,0,joint_shelf_search3[0],joint_shelf_search3[1],joint_shelf_search3[2],joint_shelf_search3[3],joint_shelf_search3[4],joint_shelf_search3[5],0.3,0.3);
//            SendGoalandWait();
//            break;
//        default:
//            break;
//        }
//        break;
//    }
//    case PULL:
//    {
//        //-----------------initial pose-----------------------//
//        ROS_INFO("PULL SHELF::Initial pose");
//        command(CMD_MoveJoint,0,0,0,joint_shelf_init[0],joint_shelf_init[1],joint_shelf_init[2],joint_shelf_init[3],joint_shelf_init[4],joint_shelf_init[5],0.4,0.3);
//        SendGoalandWait();

//        //-----------------goal pose set----------------------//
//        try
//        {
//            ros::Time now = ros::Time::now();
//            listener.waitForTransform("/rb5/base", "/marker1", now, ros::Duration(1.0));
//            listener.lookupTransform("/rb5/base", "/marker1", ros::Time(0), transform);
//        }
//        catch(tf::TransformException ex)
//        {
//            ROS_ERROR("%s",ex.what());
//            return -1;
//        }

//        Pobject = transform.getOrigin();
//        Robject = transform.getBasis();

//        x = Pobject.x();
//        y = Pobject.y();
//        z = Pobject.z();
//        r = 90.;
//        p = 0.;
//        oy= 180.;

//        Ptarget.setValue(Pobject.x(), Pobject.y(), Pobject.z());
//        Rtarget = Robject;
//        SuctiontoRB5();
//        ROS_INFO("TEST::Suction");

//        //-----------------shelf approach---------------------//
//        ROS_INFO("PULL SHELF::Approach pose");
//        command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//        SendGoalandWait();

//        //--------------------Suction-------------------------//
//        ROS_INFO("PULL SHELF::Suction");
//        command(CMD_Suction,0,0,0,0,0,0,0,0,0,0,0); //reset
//        SendGoalandWait();

//        command(CMD_Suction,0,0,1,0,0,0,0,0,0,0,0); //reset
//        SendGoalandWait();

//        //--------------------Pull shelf------------------------//
//        ROS_INFO("PULL SHELF::Pull");
//        switch(_goal->shelf_id)
//        {
//        case 1:
//            y_real += 400;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//            SendGoalandWait();
//            break;
//        case 2:
//            y_real += 400;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//            SendGoalandWait();
//            break;
//        case 3:
////            command(CMD_MoveTCP,0,0,0,robot_states->tcp_position[0], robot_states->tcp_position[1], robot_states->tcp_position[2], robot_states->tcp_position[3], robot_states->tcp_position[4], robot_states->tcp_position[5], 0.2, 0.1);
//            y_real += 400;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//            SendGoalandWait();
//            break;
//        default:
//            break;
//        }

//        //--------------------release suction -----------------------//
//        ROS_INFO("PULL SHELF::Release");
//        command(CMD_Suction,0,0,2,0,0,0,0,0,0,0,0); //reset
//        SendGoalandWait();

//        //----------------------Go to idle pose---------------------//
//        ROS_INFO("PULL_SHELF::Pose reset");
//        switch(_goal->shelf_id)
//        {
//        case 1:
////            y_real += 400;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//            SendGoalandWait();
//            break;
//        case 2:
////            y_real += 400;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//            SendGoalandWait();
//            break;
//        case 3:
//            y_real += 100;
//            z_real += 200;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//            SendGoalandWait();
//            break;
//        default:
//            break;
//        }
//        break;
//    }
//    default:
//        break;
//    }
//    MotionDone = true;
//    return 0;
//}

//int GraspAction::pullshelf(int _state)
//{
//    int shelf_id = 3;

//    switch(_state)
//    {
//    case SEARCH:
//    {
//        //-----------------search pose------------------------//
//        ROS_INFO("PULL SHELF::Search pose");
//        switch(shelf_id)
//        {
//        case 1:
//            command(CMD_MoveJoint,0,0,0,joint_shelf_search1[0],joint_shelf_search1[1],joint_shelf_search1[2],joint_shelf_search1[3],joint_shelf_search1[4],joint_shelf_search1[5],0.3,0.3);
//            SendGoalandWait();
//            break;
//        case 2:
//            command(CMD_MoveJoint,0,0,0,joint_shelf_search2[0],joint_shelf_search2[1],joint_shelf_search2[2],joint_shelf_search2[3],joint_shelf_search2[4],joint_shelf_search2[5],0.3,0.3);
//            SendGoalandWait();
//            break;
//        case 3:
//            command(CMD_MoveJoint,0,0,0,joint_shelf_search3[0],joint_shelf_search3[1],joint_shelf_search3[2],joint_shelf_search3[3],joint_shelf_search3[4],joint_shelf_search3[5],0.3,0.3);
//            SendGoalandWait();
//            break;
//        default:
//            break;
//        }
//        break;
//    }
//    case PULL:
//    {
//        //-----------------initial pose-----------------------//
//        ROS_INFO("PULL SHELF::Initial pose");
//        command(CMD_MoveJoint,0,0,0,joint_shelf_init[0],joint_shelf_init[1],joint_shelf_init[2],joint_shelf_init[3],joint_shelf_init[4],joint_shelf_init[5],0.4,0.3);
//        SendGoalandWait();

//        //-----------------goal pose set----------------------//
//        try
//        {
//            ros::Time now = ros::Time::now();
//            listener.waitForTransform("/rb5/base", "/marker1", now, ros::Duration(1.0));
//            listener.lookupTransform("/rb5/base", "/marker1", ros::Time(0), transform);
//        }
//        catch(tf::TransformException ex)
//        {
//            ROS_ERROR("%s",ex.what());
//            return -1;
//        }

//        Pobject = transform.getOrigin();
//        Robject = transform.getBasis();

//        x = Pobject.x();
//        y = Pobject.y();
//        z = Pobject.z();
//        r = 90.;
//        p = 0.;
//        oy= 180.;

//        Ptarget.setValue(Pobject.x(), Pobject.y(), Pobject.z());
//        Rtarget = Robject;
//        SuctiontoRB5();
//        ROS_INFO("TEST::Suction");

//        //-----------------shelf approach---------------------//
//        ROS_INFO("PULL SHELF::Approach pose");
//        command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//        SendGoalandWait();

//        //--------------------Suction-------------------------//
//        ROS_INFO("PULL SHELF::Suction");
//        command(CMD_Suction,0,0,0,0,0,0,0,0,0,0,0); //reset
//        SendGoalandWait();

//        command(CMD_Suction,0,0,1,0,0,0,0,0,0,0,0); //reset
//        SendGoalandWait();

//        //--------------------Pull shelf------------------------//
//        ROS_INFO("PULL SHELF::Pull");
//        switch(shelf_id)
//        {
//        case 1:
//            y_real += 400;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//            SendGoalandWait();
//            break;
//        case 2:
//            y_real += 400;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//            SendGoalandWait();
//            break;
//        case 3:
////            command(CMD_MoveTCP,0,0,0,robot_states->tcp_position[0], robot_states->tcp_position[1], robot_states->tcp_position[2], robot_states->tcp_position[3], robot_states->tcp_position[4], robot_states->tcp_position[5], 0.2, 0.1);
//            y_real += 400;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//            SendGoalandWait();
//            break;
//        default:
//            break;
//        }

//        //--------------------release suction -----------------------//
//        ROS_INFO("PULL SHELF::Release");
//        command(CMD_Suction,0,0,2,0,0,0,0,0,0,0,0); //reset
//        SendGoalandWait();

//        //----------------------Go to idle pose---------------------//
//        ROS_INFO("PULL_SHELF::Pose reset");
//        switch(shelf_id)
//        {
//        case 1:
////            y_real += 400;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//            SendGoalandWait();
//            break;
//        case 2:
////            y_real += 400;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//            SendGoalandWait();
//            break;
//        case 3:
//            y_real += 100;
//            z_real += 200;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.2,0.3);
//            SendGoalandWait();
//            break;
//        default:
//            break;
//        }
//        break;
//    }
//    default:
//        break;
//    }
//    MotionDone = true;
//    return 0;
//}
/*
void GraspAction::pushshelf(const rb5_ros_wrapper::manipulationGoalConstPtr _goal, int _state)
{

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

void GraspAction::GrippertoRB5()
{
    double rr,pp,yy;
    Rtarget.getEulerYPR(yy,pp,rr);
    printf("Target Pose : %f, %f, %f, %f, %f, %f\n",Ptarget.x(), Ptarget.y(), Ptarget.z(), rr*R2D, pp*R2D, yy*R2D);

   // tf::Matrix3x3 Rtarget, Rtool;
    tf::Matrix3x3 Rtool;

    //tf::Vector3 Ptarget, Ptool, Poffset;
    tf::Vector3 Ptool, Poffset;
    Poffset.setValue(-0.00035, 0.22122, 0.);

    Rtool = Rtarget;
    Ptool = Ptarget + Rtool*Poffset;

    Rtool.getEulerYPR(yy,pp,rr);

    x_real = Ptool.x()*1000;
    y_real = Ptool.y()*1000;
    z_real = Ptool.z()*1000;
    r_real = (float)rr*R2D;
    p_real = (float)pp*R2D;
    oy_real= (float)yy*R2D;

    printf("GRIPPERtoRB5 : %f, %f, %f, %f, %f, %f\n",x_real, y_real, z_real, r_real, p_real, oy_real);
}

void GraspAction::SuctiontoRB5()
{
    double rr,pp,yy;
    Rtarget.getEulerYPR(yy,pp,rr);
    printf("Target Pose : %f, %f, %f, %f, %f, %f\n",Ptarget.x(), Ptarget.y(), Ptarget.z(), rr*R2D, pp*R2D, yy*R2D);

    tf::Matrix3x3 Rtool, Roffset;
//    Rtarget.setEulerYPR(oy*D2R,p*D2R,r*D2R);
    Roffset.setEulerYPR(0., 0., -90.*D2R);

    tf::Vector3 Ptool, Poffset;
//    Ptarget.setValue(x,y,z);
    Poffset.setValue(0., 0.18, 0.);

    Rtool = Rtarget;
    Ptool = Ptarget + Rtool*Poffset;
    Rtool = Rtool*Roffset;

//    Poffset.setValue(0., 0.143, 0.);
    Poffset.setValue(0., 0.07345, 0.);

    Ptool = Ptool + Rtool*Poffset;

    Rtool.getEulerYPR(yy,pp,rr);

    printf("3 : %f, %f, %f, %f, %f, %f\n",Ptool.x(), Ptool.y(), Ptool.z(), rr*R2D, pp*R2D, yy*R2D);
    x_real = Ptool.x()*1000;
    y_real = Ptool.y()*1000;
    z_real = Ptool.z()*1000;
    r_real = (float)rr*R2D;
    p_real = (float)pp*R2D;
    oy_real= (float)yy*R2D;

    printf("SUCTIONtoRB5 : %f, %f, %f, %f, %f, %f\n",x_real, y_real, z_real, r_real, p_real, oy_real);
}

void GraspAction::testMarker(int _mode)
{
    try
    {
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/rb5/base", "/marker1", now, ros::Duration(1.0));
        listener.lookupTransform("/rb5/base", "/marker1", ros::Time(0), transform);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }

    Pobject = transform.getOrigin();
    Robject = transform.getBasis();

    if(_mode == TOOLMODE_GRIPPER)
    {
        Ptarget.setValue(Pobject.x(), Pobject.y(), Pobject.z()+0.2);
        Rtarget = Robject;
        GrippertoRB5();
        ROS_INFO("TEST::Gripper");
        printf("End Pose = %f, %f, %f, %f, %f, %f\n\n",Pobject.x(),Pobject.y(),Pobject.z()+0.2,r_real,p_real,oy_real);
    }else if(_mode == TOOLMODE_SUCTION)
    {
        Ptarget.setValue(Pobject.x(), Pobject.y(), Pobject.z()+0.2);
        Rtarget = Robject;
        SuctiontoRB5();
        ROS_INFO("TEST::Suction");
        printf("End Pose = %f, %f, %f, %f, %f, %f\n\n",Pobject.x(),Pobject.y(),Pobject.z()+0.2,r_real,p_real,oy_real);
    }
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.6,0.3);
    SendGoalandWait();

    ROS_INFO("TEST::Completed ;D\n\n");

    MotionDone = true;
}
*/
//=========================================================================//

class manipulationAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<rb5_ros_wrapper::manipulationAction> as_;
    std::string action_name_;

    rb5_ros_wrapper::manipulationFeedback feedback_;
    rb5_ros_wrapper::manipulationResult result_;

//        GraspAction newGrasp;
public:

    manipulationAction(std::string name) :
        as_(nh_, name, boost::bind(&manipulationAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
//        newGrasp.doInit();
//        newGrasp.pullshelf(SEARCH);
//        newGrasp.pullshelf(PULL);
    }

    ~manipulationAction(void)
    {
    }

    void writeTX(const rb5_ros_wrapper::manipulationGoalConstPtr &goal)
    {
        return;
    }

    //only called when client requests goal
    void executeCB(const rb5_ros_wrapper::manipulationGoalConstPtr &goal)
    {
//        newGrasp.doInit();

//        switch(goal->motion_cmd)
//        {
//        case HOME_POSE:
//            ROS_INFO("New CMD : Home Pose");
//            newGrasp.goHome();
//            break;
//        case SEARCH_POSE:
//            ROS_INFO("New CMD : Search Pose");
//            newGrasp.goSearch(goal);
//            break;
//        case GRASP_OBJECT:
//            if(goal->tool_id == TOOLMODE_GRIPPER)
//            {
//                ROS_INFO("New CMD : Grasp Pose");
//                newGrasp.goGraspGripper(goal);
//            }else if(goal->tool_id == TOOLMODE_SUCTION)
//            {
//                ROS_INFO("New CMD : Grasp Pose");
//                newGrasp.goGraspSuction(goal);
//            }else
//            {
//                ROS_INFO("CMD error : tool_id");
//            }
//            break;
//        case PUT_OBJECT:
//            if(goal->tool_id == TOOLMODE_GRIPPER)
//            {
//                ROS_INFO("New CMD : Put Pose");
//                newGrasp.goPutGripper(goal);
//            }else if(goal->tool_id == TOOLMODE_SUCTION)
//            {
//                ROS_INFO("New CMD : Put Pose");
//                newGrasp.goPutSuction(goal);
//            }else
//            {

//            }
//            break;
//        case PULL_SHELF:
//            ROS_INFO("New CMD : Pull Shelf");
//            break;
//        case PUSH_SHELF:
//            ROS_INFO("New CMD : Push Shelf");
//            break;
//        case TEST_MARKER:
//            ROS_INFO("New CMD : Test Marker");
//            newGrasp.testMarker(TOOLMODE_GRIPPER);
//            break;
//        default:
//            ROS_INFO("trash");

//            break;
//        }

//        while(newGrasp.MotionDone == false)
//        {
//            if(as_.isPreemptRequested() || !ros::ok())
//            {
//                ROS_INFO("%s: Preempted", action_name_.c_str());
//                as_.setPreempted();
//                break;
//            }
//            if(as_.isActive())
//            {
//                feedback_.dummy = 0;
//                as_.publishFeedback(feedback_);
//            }
//        }

//        ROS_INFO("Send result");
//        result_.result_flag = 1;
//        as_.setSucceeded(result_);
//        newGrasp.MotionDone = false;
    }

    void sendResult(int flag)
    {
        if(flag == SUCCESSED)
        {
            result_.result_flag = flag;
            as_.setSucceeded(result_);
        }else
        {
            result_.result_flag = flag;
            as_.setAborted(result_);
        }
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grasp_motion");

//    manipulationAction manipulation("manipulation");

//    ros::Subscriber _robot_state = n.subscribe("/robot_states",10,&stateCallback);
    ros::spin();
    return 0;
}
