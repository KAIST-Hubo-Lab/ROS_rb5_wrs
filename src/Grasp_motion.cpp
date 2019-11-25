#include <ros/ros.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <chrono>
#include <string.h>
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

typedef struct Motion{
    char type;
    float data;
    float coordinate[6];
    float spd;
    float acc;
} Motion;

Motion cur_motion;
Motion *motion;

const int OFFSET = 70;

typedef actionlib::SimpleActionClient<rb5_ros_wrapper::MotionAction> Client;
//rb5_ros_wrapper::updatePtr robot_states;

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
#define UNKNOWN 0

float jshelfinit[6] = {-77.185, -77.396, 134.561, -55.864, 67.807, 0.243};
Motion Shelf_Search1[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], 0.5, 0.1},
    {'J', 0, -86.5, 22.55, 128.39, -60.26, 89.99, -5.9, 0.6, 0.1},
    {'J', 0, -73.00, 86.60, 113.85, -159.19, 77.05, -11.12, 0.6, 0.1},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Search2[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], 0.8, 0.4},
    {'J', 0, -69.213, 18.208, 131.552, 51.515, -70.511, -187.407, 0.8, 0.4},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Search3[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], 0.5, 0.1},
    {'J', 0, -67.245, 11.202, 141.470, -152.670, 67.234, -0.005, 0.6, 0.1},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Push1[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], 0.4, 0.3},
    {'J', 0, -66.78, 92.31, 96.12, -278.42, -90., -23.23, 0.5, 0.3},
    {'T', 0, 0., -600, -330, 90, 0, -180, 0.5, 0.3},
    {'T', 0, 0., -400, -330, 90, 0, -180, 0.5, 0.3},
    {'T', 0, 0., -400, -100, 90, 0, -180, 0.5, 0.3},
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], 0.4, 0.3},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Push2[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], 0.8, 0.4},
    {'J', 0, -68.42, -24.99, 142.56, -27.58, 90.01, -201.59, 0.8, 0.4},
    {'J', 0, -68.42, 21.302, 156.726, -88.028, 90.000, -201.592, 0.5, 0.3},
    {'T', 0, 0, -607.0, 80.0, 90.0, 0, 180.0, 0.1, 0.1},
    {'T', 0, 0, -407.0, 80.0, 90.0, 0, 180.0, 0.4, 0.1},
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], 0.6, 0.4},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Push3[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], 0.4, 0.3},
    {'J', 0, -56.51, -47.5, 117.94, 19.57, 90, 146.51, 0.5, 0.3},
    {'T', 0, 0, -607.0, 495.0, 90.0, 0, 180.0, 0.1, 0.1},
    {'T', 0, 0, -407.0, 495.0, 90.0, 0, 180.0, 0.1, 0.1},
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], 0.4, 0.3},
    {'E',0,0,0,0,0,0,0,0,0}
};

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
    void goGrasp(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    void goPut(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    int pullshelf(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    int sendMotion(Motion _curMotion);
    void pushshelf(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    void testMarker(int _mode);
    void GrippertoRB5();
    void SuctiontoRB5();
    void PrintCoordinate(char *_name, tf::Vector3 _p, tf::Matrix3x3 _o)
    {
        double rr,pp,yy;
        _o.getEulerYPR(yy,pp,rr);
        printf("%s Pose = [%f, %f, %f], [%f, %f, %f]\n", _name, _p.x()*1000, _p.y()*1000, _p.z()*1000, rr*R2D, pp*R2D, yy*R2D);
    }
    void PrintCoordinate(char *_name, float c0, float c1, float c2, float c3, float c4, float c5)
    {
        printf("%s Pose = [%f, %f, %f], [%f, %f, %f]\n", _name, c0,c1,c2,c3,c4,c5);
    }

protected:

    rb5_ros_wrapper::MotionGoal rb5_goal;
    ros::NodeHandle n;
    rb5_ros_wrapper::manipulationFeedback feedback_;
    rb5_ros_wrapper::manipulationResult result_;
    tf::TransformListener listener;
    tf::StampedTransform transform;
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
    void suction(int _mode)
    {
        if(_mode == 0)
        {
            ROS_INFO("Suction reset1");
            command(CMD_Suction,0,0,0,0,0,0,0,0,0,0,0);
        }else if(_mode == 1)
        {
            ROS_INFO("Suction suction1");
            command(CMD_Suction,0,0,1,0,0,0,0,0,0,0,0);
        }else if(_mode == 2)
        {
            ROS_INFO("Suction release1");
            command(CMD_Suction,0,0,2,0,0,0,0,0,0,0,0);
        }
        SendGoalandWait();
    }
};

int GraspAction::sendMotion(Motion _curMotion)
{
    switch(_curMotion.type)
    {
    case 'J':
        command(CMD_MoveJoint, 0, 0, 0, _curMotion.coordinate[0],_curMotion.coordinate[1],_curMotion.coordinate[2],_curMotion.coordinate[3],_curMotion.coordinate[4],_curMotion.coordinate[5],_curMotion.spd,_curMotion.acc);
        SendGoalandWait();
        break;
    case 'T':
        command(CMD_MoveTCP, 0, 0, 0, _curMotion.coordinate[0],_curMotion.coordinate[1],_curMotion.coordinate[2],_curMotion.coordinate[3],_curMotion.coordinate[4],_curMotion.coordinate[5],_curMotion.spd,_curMotion.acc);
        SendGoalandWait();
        break;
    case 'E':
        return 0;
    default:
        return -1;
    }
    return 1;
}

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

    ROS_INFO("Going Search pose.\n");
    command(CMD_MoveJoint, 0,0,0,jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5],0.8,0.4);
    SendGoalandWait();

//    command(CMD_MoveJoint, 0, 0, 0, -78.42, 5.19, 92.11, -7.29, 90.01, -11.58, 0.5, 0.3);
//    command(CMD_MoveJoint, 0,0,0,-89.38, -8.13, 109.98, -21.29, 93.31, 19.1, 0.3, 0.2);//origin
    command(CMD_MoveJoint,0,0,0,-131.54, -24.98, 147.61, -62.23, 132.34, 49.85, 0.8, 0.4);
    //    command(CMD_MoveJoint,0,0,0,85.63, -35.05, 136.8, -37.05, 89.04, -4.14, 0.2, 0.1);

    SendGoalandWait();

    MotionDone = true;
}

void GraspAction::goGrasp(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    if(_goal->motion_cmd != GRASP_OBJECT)
        return;

    int tool = _goal->tool_id;


    //---------------------transform base to marker--------------------------//
    try
    {
        ros::Time now = ros::Time::now();
        listener.lookupTransform("/rb5/base", "/marker1", ros::Time(0), transform);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }
    Pobject = transform.getOrigin();
    Robject = transform.getBasis();

    double rr,pp,yy;
    Robject.getEulerYPR(yy,pp,rr);
    PrintCoordinate("Object",Pobject, Robject);


    //-----------------------set target pose-------------------------------//
    if(tool == TOOLMODE_GRIPPER)
    {
        Ptarget.setValue(Pobject.x(), Pobject.y(), Pobject.z()+0.05);

        printf("object id = %d\n",_goal->object_id);
        if(_goal->object_id == KIMBAP1 || _goal->object_id ==KIMBAP2)
        {
            Rtarget.setEulerYPR(yy, 0, 90*D2R);
        }else
        {
            Rtarget.setEulerYPR(yy-90*D2R, 0, 90*D2R);
        }

        PrintCoordinate("Gripper Target",Ptarget,Rtarget);
        GrippertoRB5();

        ROS_INFO("GRASP::Open Gripper");
        openGripper();
    }else if(tool == TOOLMODE_SUCTION)
    {
        Ptarget.setValue(Pobject.x(), Pobject.y(), Pobject.z()+0.05);
        Rtarget.setEulerYPR(0, 0, 90*D2R);

        PrintCoordinate("Suction Target",Ptarget,Rtarget);
        SuctiontoRB5();
    }


    //-------------------Approach---------------------//
    ROS_INFO("GRASP::Approach pose1");
    PrintCoordinate("Approach",x_real,y_real,z_real,r_real,p_real,oy_real);

    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.5,0.3);
    SendGoalandWait();

    ROS_INFO("GRASP::Approach pose2");
    z_real -= 60;
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.5,0.3);
    SendGoalandWait();


    //--------------------Suction----------------------//s
    if(tool == TOOLMODE_GRIPPER)
    {
        ROS_INFO("GRASP::Close Gripper");
        closeGripper();
    }else if(tool == TOOLMODE_SUCTION)
    {
        ROS_INFO("GRASP::Suction");
        suction(0);
        suction(1);
    }
    usleep(500*1000);

    //------------------Lift-----------------------//
    ROS_INFO("GRASP::Lift");
    z_real += 150;
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.5,0.3);
    SendGoalandWait();


    ROS_INFO("GRASP::Completed ;D\n\n");
    MotionDone = true;
}

void GraspAction::goPut(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    if(_goal->motion_cmd != PUT_OBJECT)
        return;

    int tool = _goal->tool_id;

    //-------------------go to goal pose (blend)-------------------//
    ROS_INFO("GRASP::Put pose (manually)");
    if(tool == TOOLMODE_GRIPPER)
    {
        command(CMD_MoveJoint,0,0,0,82.24, -5.61, 110.31, -14.70, 90.01, 7.76, 0.5, 0.3);
        x_real = 170;
        y_real = 430;
        r_real = 90;
        p_real = 0;
        oy_real = 180;

    }else if(tool == TOOLMODE_SUCTION)
    {
        command(CMD_MoveJoint,0,0,0,82.83, 12.93, 126.03, -138.96, 87.16, -0.01, 0.5, 0.3);
        x_real = 170;
        y_real = 430;
        r_real = 0;
        p_real = 0;
        oy_real = 180;
    }
    SendGoalandWait();


    //-------------------Put---------------------//
    ROS_INFO("GRASP::Put -z\n");
    z_real += 6;
    z_real -= 150;
    PrintCoordinate("Put",x_real,y_real,z_real,r_real,p_real,oy_real);
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.5,0.3);
    SendGoalandWait();

    if(tool == TOOLMODE_GRIPPER)
    {
        ROS_INFO("Grasp::Open Gripper");
        openGripper();
    }else if(tool == TOOLMODE_SUCTION)
    {
        ROS_INFO("GRASP::Release");
        suction(2);
    }

    //------------------- Up -----------------------//
    z_real += 150;
    ROS_INFO("GRASP::Put +z\n");
    command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.5,0.3);
    SendGoalandWait();

    if(tool == TOOLMODE_GRIPPER)
    {
        ROS_INFO("Grasp::Close Gripper");
        closeGripper();
    }

    ROS_INFO("GRASP::Put Completed ;D\n\n");
    MotionDone = true;
}

int GraspAction::pullshelf(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    if(_goal->motion_cmd != PULL_SHELF)
        return -1;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    switch(_goal->sub_cmd)
    {
    case SEARCH:
    {
        //-----------------search pose set------------------------//
        int seq = 0;
        switch(_goal->shelf_id)
        {
        case 1:
            ROS_INFO("PULL SHELF::Search pose1");
            motion = Shelf_Search1;
            break;
        case 2:
            ROS_INFO("PULL SHELF::Search pose2");
            motion = Shelf_Search2;
            break;
        case 3:
            ROS_INFO("PULL SHELF::Search pose3");
            motion = Shelf_Search3;
            break;
        default:
            break;
        }

        cur_motion = motion[seq];

        //---------------------motion send------------------------//
        while(sendMotion(cur_motion) == 1)
        {
            seq++;
            cur_motion = motion[seq];
        }
        ROS_INFO("PULL SHELF::Search pose done");
        break;
    }
    case PULL:
    {
        //----------------- target pose set ----------------------//
        usleep(1000*1000);
        try
        {
            ros::Time now = ros::Time::now();
//            listener.waitForTransform("/rb5/base", "/marker1", now, ros::Duration(1.0));
            listener.lookupTransform("/rb5/base", "/marker1", ros::Time(0), transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return -1;
        }

        printf("transform time = %f\n",transform.stamp_.toSec());
        Pobject = transform.getOrigin();
        Robject = transform.getBasis();

        switch(_goal->shelf_id)
        {
        case 1:
            Ptarget.setValue(Pobject.x()+0.005, Pobject.y()+0.04, Pobject.z()+0.01);
            break;
        case 2:
            Ptarget.setValue(Pobject.x()+0.003, Pobject.y()+0.04, Pobject.z()+0.01);
            break;
        case 3:
            Ptarget.setValue(Pobject.x()-0.015, Pobject.y()+0.06, Pobject.z()+0.058);
            break;
        default:
            break;
        }

//        Ptarget.setValue(Pobject.x(), Pobject.y(), Pobject.z());
        Rtarget.setEulerYPR(0, 180*D2R, 0);

        SuctiontoRB5();

        //-----------------shelf approach---------------------//
        if(_goal->shelf_id == 2)
        {
            ROS_INFO("PULL SHELF::J6 set 2");
            command(CMD_MoveJoint,0,0,0,-69.213, 18.208, 131.552, 51.515, -70.511, 0., 1.0, 0.7);
            SendGoalandWait();
        }else if(_goal->shelf_id == 1)
        {
//            ROS_INFO("PULL SHELF::J6 set 1");
//            command(CMD_MoveJoint,0,0,0,-72.014, 59.888, 113.569, 47.964, -76.309, -11.795, 0.8, 0.3);
//            SendGoalandWait();
        }
        ROS_INFO("PULL SHELF::Approach pose1");
        command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.5, 0.3);
        SendGoalandWait();

        y_real-=53;
        ROS_INFO("PULL SHELF::Approach pose2");
        command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.1, 0.1);
        SendGoalandWait();

        //--------------------Suction-------------------------//
        ROS_INFO("PULL SHELF::Suction");
        suction(0); //reset

        suction(1); //suction

        usleep(1000*1000);
        //--------------------Pull shelf------------------------//
        ROS_INFO("PULL SHELF::Pull");
        switch(_goal->shelf_id)
        {
        case 1:
            y_real += 300;
            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.1,0.1);
            SendGoalandWait();
            break;
        case 2:
            y_real += 300;
            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.1,0.1);
            SendGoalandWait();
            break;
        case 3:
            y_real += 300;
            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.1,0.1);
            SendGoalandWait();
            break;
        default:
            break;
        }

        //--------------------release suction -----------------------//
        ROS_INFO("PULL SHELF::Release");
        suction(2); //release
        usleep(500*1000);

        //----------------------Go to idle pose---------------------//
        switch(_goal->shelf_id)
        {
        case 1:
            ROS_INFO("PULL_SHELF::Final pose1");
            z_real += 150;
            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.3, 0.2);
            SendGoalandWait();

//            x_real = 0.;
//            y_real = -500.;
//            z_real = -100.;
//            r_real = 90.;
//            p_real = 0.;
//            oy_real = 0;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.1, 0.1);
//            command(CMD_MoveJoint,0,0,0,-77.25, 59.39, 119.1, -88.49, 89.99, -12.76, 0.2, 0.1);
            command(CMD_MoveJoint,0,0,0,jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5],0.5,0.3);
            SendGoalandWait();
            break;
        case 2:
            ROS_INFO("PULL_SHELF::Final pose2");
            z_real += 150;
            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.1, 0.1);
            SendGoalandWait();

            ROS_INFO("PULL_SHELF::Final pose2");
//            x_real = 0.;
//            y_real = -500.;
//            z_real = 450.;
//            r_real = 90.;
//            p_real = 0.;
//            oy_real = 0;
//            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.1, 0.1);
            command(CMD_MoveJoint,0,0,0,jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5],0.5,0.3);
            SendGoalandWait();
            break;
        case 3:
            ROS_INFO("PULL_SHELF::Final pose3");
            z_real += 150;
            command(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,0.1, 0.1);
            SendGoalandWait();
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
    MotionDone = true;
    return 0;
}

void GraspAction::pushshelf(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    //-----------------Init pose set------------------------//
    int seq = 0;
    switch(_goal->shelf_id)
    {
    case 1:
        ROS_INFO("PUSH SHELF::Init pose1");
        motion = Shelf_Push1;
        break;
    case 2:
        ROS_INFO("PUSH SHELF::Init pose2");
        motion = Shelf_Push2;
        break;
    case 3:
        ROS_INFO("PUSH SHELF::Init pose3");
        motion = Shelf_Push3;
        break;
    default:
        break;
    }

    cur_motion = motion[seq];

    //---------------------motion send------------------------//
    while(sendMotion(cur_motion) == 1)
    {
        seq++;
        cur_motion = motion[seq];
    }
    ROS_INFO("PUSH SHELF::Search pose done");
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
    printf("Target Pose : %f, %f, %f, %f, %f, %f\n",Ptarget.x()*1000, Ptarget.y()*1000, Ptarget.z()*1000, rr*R2D, pp*R2D, yy*R2D);

    tf::Matrix3x3 Rtool, Roffset;
    Roffset.setEulerYPR(0., 0., -90.*D2R);

    tf::Vector3 Ptool, Poffset;
    Poffset.setValue(0., 0.18, 0.);

    Rtool = Rtarget;
    Ptool = Ptarget + Rtool*Poffset;
    Rtool = Rtool*Roffset;

    Poffset.setValue(0., 0.07345, 0.);

    Ptool = Ptool + Rtool*Poffset;

    Rtool.getEulerYPR(yy,pp,rr);

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
//        listener.lookupTransform("/rb5/base", "/marker1", now, transform);
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

//=========================================================================//

class manipulationAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<rb5_ros_wrapper::manipulationAction> as_;
    std::string action_name_;

    rb5_ros_wrapper::manipulationFeedback feedback_;
    rb5_ros_wrapper::manipulationResult result_;

        GraspAction newGrasp;
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

    //only called when client requests goal
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
            newGrasp.goGrasp(goal);
            break;
        case PUT_OBJECT:
            newGrasp.goPut(goal);
            break;
        case PULL_SHELF:
            ROS_INFO("New CMD : Pull Shelf");
            newGrasp.pullshelf(goal);
            break;
        case PUSH_SHELF:
            ROS_INFO("New CMD : Push Shelf");
            newGrasp.pushshelf(goal);
            break;
        case TEST_MARKER:
            ROS_INFO("New CMD : Test Marker");
            newGrasp.testMarker(TOOLMODE_GRIPPER);
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

    manipulationAction manipulation("manipulation");

//    ros::Subscriber _robot_state = n.subscribe("/robot_states",10,&stateCallback);
    ros::spin();
    return 0;
}
