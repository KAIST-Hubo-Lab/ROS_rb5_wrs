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
#include "wrs_parameter.h"
#include "wrs_motion.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "rb5_ros_wrapper/update.h"

#define D2R             0.0174533
#define R2D             57.2958

typedef actionlib::SimpleActionClient<rb5_ros_wrapper::MotionAction> Client;

Motion cur_motion;
Motion *motion;

class GraspAction
{
public:
    GraspAction() : ac_("motion", true)
    {
        ROS_INFO("Waiting for action server to start.");
        ac_.waitForServer();
        ROS_INFO("Action server started.");
    }

    tf::StampedTransform transform_shelf;
    tf::StampedTransform transform;
    int MotionDone = IDLE;

    void doInit()
    {
        //need to initialize the robot before sending commands
        ROS_INFO("Initializing.");
        Sendcommand(CMD_Initialize,0,0,0,0,0, 0, 0, 0, 0,0,0);

        //set mode
        ROS_INFO("Setting to real mode.");
        Sendcommand(CMD_ChangeMode,0,0,1,0,0, 0, 0, 0, 0,0,0);

        //change speed 50%
        ChangeSpeed(0.6);
        goHome(IDLEPOSE);
    }

    void doTest()
    {
        ROS_INFO("Send Joint move");
        Sendcommand(CMD_MoveJoint,0,0,0,-245.103, -138.91, 89.064, -132, -20, 0.21, 0.5,0.3);
        if(MotionDone == EXT_COLLISION)
        {
            MotionDone = IDLE;
            Sendcommand(CMD_MotionHalt,0,0,0,0,0,0,0,0,0,0,0);
            ROS_ERROR("halt..");
        }
        ROS_INFO("Joint move done");
    }

    void doneCb(const actionlib::SimpleClientGoalState& state,
                const rb5_ros_wrapper::MotionResultConstPtr& result)
    {
        ROS_INFO("Done CB");
        switch(result->rb5result)
        {
        case ACCEPT:
        case DONE:
            break;
        case STATE_ERROR:
        case INPUT_ERROR:
        case ERROR_STOP:
            ROS_ERROR("Action Server send rb5 ERROR");
            MotionDone = result->rb5result;
            break;
        case EXT_COLLISION:
            ROS_ERROR("EXT COLLISION!!!!");
//            MotionDone = result->rb5result;
            break;
        }

        switch(result->wheelresult)
        {
        case ACCEPT:
        case DONE:
            break;
        case STATE_ERROR:
        case INPUT_ERROR:
        case ERROR_STOP:
            ROS_ERROR("Action Server send wheel ERROR");
            MotionDone = result->wheelresult;
            break;
        }
    }
    void activeCb()
    {
    }
    void feedbackCb(const rb5_ros_wrapper::MotionFeedbackConstPtr& feedback)
    {
    }

    void Sendcommand(char type, int d0, int d1, float data, float coordinate0, float coordinate1, float coordinate2, float coordinate3, float coordinate4, float coordinate5, float spd, float acc)
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
        SendGoalandWait();
    }
    void Sendcommand(char type, int d0, int d1, float data, float coordinate0, float coordinate1, float coordinate2, float coordinate3, float coordinate4, float coordinate5, const float spd[])
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
        rb5_goal.spd = spd[0];
        rb5_goal.acc = spd[1];
        SendGoalandWait();
    }
    void Sendcommand(char type, Pose _pose,  const float spd[])
    {
        rb5_goal.type = type;
        rb5_goal.d0   = 0;
        rb5_goal.d1   = 0;
        rb5_goal.data = 0;
        rb5_goal.coordinate[0] = _pose.cdn[0];
        rb5_goal.coordinate[1] = _pose.cdn[1];
        rb5_goal.coordinate[2] = _pose.cdn[2];
        rb5_goal.coordinate[3] = _pose.cdn[3];
        rb5_goal.coordinate[4] = _pose.cdn[4];
        rb5_goal.coordinate[5] = _pose.cdn[5];
        rb5_goal.spd = spd[0];
        rb5_goal.acc = spd[1];
        SendGoalandWait();
    }
    void Sendcommandwheel(char type, int d0, int d1, float x_m, float y_m, float yaw_deg)
    {
        rb5_goal.type = type;
        rb5_goal.d0 = d0;
        rb5_goal.d1 = d1;
        rb5_goal.wheel[0] = x_m;
        rb5_goal.wheel[1] = y_m;
        rb5_goal.wheel[2] = yaw_deg;
        SendGoalandWait();
    }
    void SendGoalandWait()
    {
        if(MotionDone == ACCEPT || MotionDone == IDLE)
        {
            ac_.sendGoal(rb5_goal, boost::bind(&GraspAction::doneCb, this, _1, _2), boost::bind(&GraspAction::activeCb, this), boost::bind(&GraspAction::feedbackCb, this, _1));
            ac_.waitForResult();
        }
    }
    int sendMotion(Motion _curMotion);

    void goHome(int _curPose);
    void goSearch(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    void goGrasp(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    void goPut(const rb5_ros_wrapper::manipulationGoalConstPtr _goal, Pose _pose);

    void pullshelf(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);
    void pushshelf(const rb5_ros_wrapper::manipulationGoalConstPtr _goal);

    void TCPtoRB5(int _mode);
    void TCPtoRB5(Pose _pose, int _mode);

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
    void PrintGoal(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
    {
        int _shelf = _goal->shelf_id;
        int _id = _goal->object_id;
        int _sub = _goal->sub_cmd;
        char *cmd, *sub, *object;
        sub = "";
        switch(_goal->motion_cmd)
        {
        case HOME_POSE:
        {
            cmd = "Home_Pose";
            break;
        }
        case OBJECT_SEARCH:
        {
            if(_shelf == BASKET)
                cmd = "Object_Search (Basket)";
            else if(_shelf == TEMP)
                cmd = "Object_Search (Lunchbox)";
            else
                cmd = "Object_Search (Shelf)";

            switch(_sub)
            {
            case 0:
                sub = "Motion 1";
                break;
            case 1:
                sub = "Motion 2";
                break;
            case 2:
                sub = "Motion 3";
                break;
            case 3:
                sub = "Motion 4";
                break;
            default:
                sub = "Wrong_Subcmd";
                break;
            }
            break;
        }
        case GRASP_OBJECT:
        {
            cmd = "Object_Grasp";
            switch(_shelf)
            {
            case SHELF1:
                sub = "at Shelf1";
                break;
            case SHELF2:
                sub = "at Shelf2";
                break;
            case SHELF3:
                sub = "at Shelf3";
                break;
            case BASKET:
                sub = "at Basket";
                break;
            case TEMP:
                sub = "at TempPlace";
                break;
            default:
                sub = "Wrong Shelf num";
                break;
            }
            break;
        }
        case PUT_OBJECT:
        {
            cmd = "Object_Put";
            switch(_sub)
            {
            case DISPOSE:
                sub = "Dispose";
                break;
            case LINE1:
                sub = "Align to Line 1";
                break;
            case LINE2:
                sub = "Align to Line 2";
                break;
            case LINE3:
                sub = "Align to Line 3";
                break;
            case LUNCHBOX_PUT:
                sub = "Put temp lunchbox";
                break;
            default:
                sub = "Wrong_Subcmd";
                break;
            }
            break;
        }
        case PULL_SHELF:
        {
            if(_goal->sub_cmd == SEARCH)
            {
                cmd = "Shelf_Pull (Search)";
            }else
            {
                cmd = "Shelf_Pull (Pull)";
            }
            if(_shelf == SHELF1)
                sub = "1";
            else if(_shelf == SHELF2)
                sub = "2";
            else if(_shelf == SHELF3)
                sub = "3";
            else
                sub = "-1";
            break;
        }
        case PUSH_SHELF:
        {
            cmd = "Shelf_Push";
            if(_shelf == SHELF1)
                sub = "1";
            else if(_shelf == SHELF2)
                sub = "2";
            else if(_shelf == SHELF3)
                sub = "3";
            else
                sub = "-1";
            break;
        }
        case DETECT_SHELF:
        {
            cmd = "Shelf_Search";
            if(_shelf == SHELF1)
                sub = "1";
            else if(_shelf == SHELF2)
                sub = "2";
            else if(_shelf == SHELF3)
                sub = "3";
            else
                sub = "-1";
            break;
        }
        default:
            cmd = "Wrong_Command";
            break;
        }

        switch(_goal->object_id)
        {
        case KIMBAP1:
            object = "Kimbap1";
            break;
        case KIMBAP2:
            object = "Kimbap2";
            break;
        case SANDWICH:
            object = "Sandwich";
            break;
        case HAMBUG:
            object = "Hambug";
            break;
        case COFFEE:
            object = "Coffee";
            break;
        case LUNCHBOX:
            object = "Lunchbox";
            break;
        default:
            object = "Wrong_object";
            break;
        }

        ROS_INFO("\n");
        ROS_INFO("-------------------------------------");
        ROS_INFO("New Goal received : %s %s",cmd,sub);
        ROS_INFO("object : %s",object);
        ROS_INFO("-------------------------------------");
        ROS_INFO("\n");
    }

    void ChangeSpeed(float _spd)
    {
        if(_spd > 1.0) _spd = 1.0;
        else if(_spd < 0.0) _spd = 0.0;

        //change speed to _spd*100%
        Sendcommand(CMD_ChangeSpeed,0,0,_spd,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Changing speed : %f",_spd);
    }

    void getShelfPose(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
    {
        //----------------- target pose set ----------------------//
        usleep(500*1000);
        int _shelf = _goal->shelf_id;
        try
        {
            ros::Time now = ros::Time::now();
            listener.lookupTransform("/rb5/base", "/shelf", ros::Time(0), transform_shelf);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }
        tf::Vector3 temp;
        temp = transform_shelf.getOrigin();
        ROS_INFO("\n");
        ROS_INFO("-----------------------------------------------");
        ROS_INFO("Back Shelf %d detected : %f, %f, %f", _shelf, temp.x(), temp.y(), temp.z());

        float new_x = temp.x();
        float new_y = temp.y();
        float new_z = temp.z();
        switch(_shelf)
        {
        case SHELF1:
            if(fabs(temp.y()-shelf_marker[BACK1].y) > limit_marker_offset_b)
            {
                ROS_ERROR("Marker y is changed %f to %f",temp.y(),shelf_marker[BACK1].y);
                new_y = shelf_marker[BACK1].y;
            }
            if(fabs(temp.z()-shelf_marker[BACK1].z) > limit_marker_offset_z)
            {
                ROS_ERROR("Marker z is changed %f to %f",temp.z(),shelf_marker[BACK1].z);
                new_z = shelf_marker[BACK1].z;
            }
            Sendcommand(CMD_MoveTCP,0,0,0,0,-350,100,90,0,0,spd_60);
            break;
        case SHELF2:
            if(fabs(temp.y()-shelf_marker[BACK2].y) > limit_marker_offset_b)
            {
                ROS_ERROR("Marker y is changed %f to %f",temp.y(),shelf_marker[BACK2].y);
                new_y = shelf_marker[BACK2].y;
            }
            if(fabs(temp.z()-shelf_marker[BACK2].z) > limit_marker_offset_z)
            {
                ROS_ERROR("Marker z is changed %f to %f",temp.z(),shelf_marker[BACK2].z);
                new_z = shelf_marker[BACK2].z;
            }
            break;
        case SHELF3:
            if(fabs(temp.y()-shelf_marker[BACK3].y) > limit_marker_offset_b)
            {
                ROS_ERROR("Marker y is changed %f to %f",temp.y(),shelf_marker[BACK3].y);
                new_y = shelf_marker[BACK3].y;
            }
            if(fabs(temp.z()-shelf_marker[BACK3].z) > limit_marker_offset_z)
            {
                ROS_ERROR("Marker z is changed %f to %f",temp.z(),shelf_marker[BACK3].z);
                new_z = shelf_marker[BACK3].z;
            }
            break;
        default:
            ROS_ERROR("Wrong Shelf number!!!! %d",_shelf);

            break;
        }

        Pshelf.setValue(new_x, new_y, new_z);
        transform_shelf.setOrigin(Pshelf);
        Rshelf = transform_shelf.getBasis();

        ROS_INFO("-----------------------------------------------");
        ROS_INFO("\n");

        Sendcommand(CMD_MoveJoint,home_pose[JDEFAULTHOME],spd_80);
        MotionDone = DONE;
    }

protected:
    rb5_ros_wrapper::MotionGoal rb5_goal;
    ros::NodeHandle n;
    rb5_ros_wrapper::manipulationFeedback feedback_;
    rb5_ros_wrapper::manipulationResult result_;
    tf::TransformListener listener;
    Client ac_;

    float offset_approach_z = 200.;
    float x_real, y_real, z_real, r_real, p_real, oy_real;

    tf::Matrix3x3 Rtarget, Robject, Rshelf;
    tf::Vector3   Ptarget, Pobject, Pshelf;

    void openGripper(int _msec=max_gripper_cnt);
    void closeGripper(int _msec=max_gripper_cnt);
    void pause(float time);
    void resume();
    void suction(int _mode)
    {
        if(_mode == 0)
        {
            ROS_INFO("Suction reset");
            Sendcommand(CMD_Suction,0,0,0,0,0,0,0,0,0,0,0);
        }else if(_mode == 1)
        {
            ROS_INFO("Suction suction");
            Sendcommand(CMD_Suction,0,0,1,0,0,0,0,0,0,0,0);
        }else if(_mode == 2)
        {
            ROS_INFO("Suction release");
            Sendcommand(CMD_Suction,0,0,2,0,0,0,0,0,0,0,0);
        }

        usleep(500*1000);
    }
};

int GraspAction::sendMotion(Motion _curMotion)
{
    switch(_curMotion.type)
    {
    case 'J':
        ROS_INFO("Move Joint : %f, %f, %f, %f, %f, %f, %f, %f",_curMotion.coordinate[0],_curMotion.coordinate[1],_curMotion.coordinate[2],_curMotion.coordinate[3],_curMotion.coordinate[4],_curMotion.coordinate[5],_curMotion.spd,_curMotion.acc);
        Sendcommand(CMD_MoveJoint, 0, 0, 0, _curMotion.coordinate[0],_curMotion.coordinate[1],_curMotion.coordinate[2],_curMotion.coordinate[3],_curMotion.coordinate[4],_curMotion.coordinate[5],_curMotion.spd, _curMotion.acc);
        break;
    case 'T':
        ROS_INFO("Move TCP : %f, %f, %f, %f, %f, %f, %f, %f",_curMotion.coordinate[0],_curMotion.coordinate[1],_curMotion.coordinate[2],_curMotion.coordinate[3],_curMotion.coordinate[4],_curMotion.coordinate[5],_curMotion.spd,_curMotion.acc);
        Sendcommand(CMD_MoveTCP, 0, 0, 0, _curMotion.coordinate[0],_curMotion.coordinate[1],_curMotion.coordinate[2],_curMotion.coordinate[3],_curMotion.coordinate[4],_curMotion.coordinate[5],_curMotion.spd, _curMotion.acc);
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
    if(_msec < 0 || _msec > max_gripper_cnt)
        _msec = max_gripper_cnt;

    //'E' command opens on 0,1, closes on 1,0
    Sendcommand(CMD_MoveGripper,0,1,_msec,0,0, 0, 0, 0, 0,0,0);

    usleep(500*1000);
}

void GraspAction::closeGripper(int _msec)
{
    if(_msec < 0 || _msec > max_gripper_cnt)
        _msec = max_gripper_cnt;

    //'E' command opens on 0,1, closes on 1,0
    Sendcommand(CMD_MoveGripper,1,0,_msec,0,0, 0, 0, 0, 0,0,0);

    usleep(500*1000);
}

void GraspAction::goHome(int _curPose)
{
    switch(_curPose)
    {
    case SHELF1:
        ROS_INFO("Going home : SHELF1");
        Sendcommand(CMD_MoveJoint,home_pose[TSHELF1HOME],spd_80);
        closeGripper();
        break;
    case BASKET:
        ROS_INFO("Going home : BASKET");
        Sendcommand(CMD_MoveJoint,home_pose[JBASKETHOME],spd_80);
        closeGripper();
        break;
    default:
        ROS_INFO("Going home : DEFAULT");
        Sendcommand(CMD_MoveJoint,home_pose[JDEFAULTHOME],spd_80);
        closeGripper();

        break;
    }
    MotionDone = DONE;
}

void GraspAction::goSearch(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    if(_goal->motion_cmd != OBJECT_SEARCH)
        return;

    int _shelf = _goal->shelf_id;
    int _id = _goal->object_id;
    Sendcommand(CMD_MoveJoint,home_pose[JDEFAULTHOME],spd_80);

    char *object_name;
    switch(_shelf)
    {
    case SHELF1:
    {
        if(_id == COFFEE)
        {
            object_name = "coffee";
            switch(_goal->sub_cmd)
            {
            case 0:
                Sendcommand(CMD_MoveJoint,0,0,0,-45.34, 43.01, 127.3, -80.31, 90., -44.67, spd_80);
                break;
            case 1:
                Sendcommand(CMD_MoveJoint,0,0,0,-47.01, 27.97, 125.65, -63.63, 90.00, -43., spd_80);
                break;
            case 2:
                Sendcommand(CMD_MoveJoint,0,0,0,-22.15, -13.82, 103.18, -12.09, 58.61, -98.12, spd_80);
                Sendcommand(CMD_MoveJoint,0,0,0,-7.45, 93.85, 36.74, 2.87, 62.24, -153.78, spd_80);
                break;
            case 3:
                Sendcommand(CMD_MoveJoint,0,0,0,-99.35, 29.61, 137.15, -87.09, 85.72, 67.73, spd_80);
                Sendcommand(CMD_MoveJoint,0,0,0,-110.21, 78.31, 113.92, -102.05, 130.0, 90.26, spd_80);
                break;
            default:
                printf("no search pose %d\n",_goal->sub_cmd);
                MotionDone = INPUT_ERROR;
                return;
            }
        }else if(_id == LUNCHBOX)
        {
            object_name = "lunchbox";
            switch(_goal->sub_cmd)
            {
            case 0:
                Sendcommand(CMD_MoveJoint,0,0,0,-99.66, 42.1, 133.78, -85.88, 90.0, 9.65, spd_80);
                break;
            case 1:
                Sendcommand(CMD_MoveJoint,0,0,0,-99.66, 23.65, 135.01, -68.66, 90., 9.65, spd_80);
                break;
            default:
                printf("no search pose %d\n",_goal->sub_cmd);
                return;
            }
        }
        break;
    }
    case SHELF2:
    {
        if(_id == SANDWICH)
        {
            object_name = "sandwich";
            switch(_goal->sub_cmd)
            {
            case 0:
                Sendcommand(CMD_MoveJoint,0,0,0,24.31, 10.22, 145.1, -14.9, 51.71, -152.88, spd_80);
                break;
            case 1:
                Sendcommand(CMD_MoveJoint,0,0,0,-108.45, 25.24, 137.3, -69.11, 158.87, -80, spd_80);
                break;
            case 2:
                Sendcommand(CMD_MoveJoint,0,0,0,-17.7, 42.29, 79.6, 25.15, 50.99, 22.21, spd_80);
                break;
            case 3:
                Sendcommand(CMD_MoveJoint,0,0,0,-20.27, 66.84, 34.09, 55.93, 55.58, 10.23, spd_80);
                break;
            default:
                printf("no search pose %d\n",_goal->sub_cmd);
                return;
            }
        }else if(_id == HAMBUG)
        {
            object_name = "hambug";
            switch(_goal->sub_cmd)
            {
            case 0:
                Sendcommand(CMD_MoveJoint,0,0,0,-160, 11.05, 135.04, -55.27, 140, 91.08, spd_80);
                break;
            case 1:
                Sendcommand(CMD_MoveJoint,0,0,0,-151.92, 29.43, 93.85, -9.3, 135.3, 122.04, spd_80);
                break;
            case 2:
                Sendcommand(CMD_MoveJoint,0,0,0,-47.37 ,15.39, 115.78, 1.6, 42.94, 38.36, spd_80);
                break;
            case 3:
                Sendcommand(CMD_MoveJoint,0,0,0,-20.27, 66.84, 34.09, 55.93, 55.58, 10.23, spd_80);
                break;
            default:
                printf("no search pose %d\n",_goal->sub_cmd);
                return;
            }
        }
        break;
    }
    case SHELF3:
    {
        if(_id == KIMBAP1)
        {
            object_name = "kimbap1";
            switch(_goal->sub_cmd)
            {
            case 0:
                Sendcommand(CMD_MoveJoint,0,0,0,-121.11, -12.38, 100.23, -0.08, 138.88, 96.48, spd_80);
                break;
            case 1:
                Sendcommand(CMD_MoveJoint,0,0,0,-18.58, 47.75, 57.42, -137.58, -58.38, 8.17, spd_80);
                break;
            case 2:
                Sendcommand(CMD_MoveJoint,0,0,0, -138.89, 28.32, 67., 36.06, 142.46, 138.52, spd_80);
                break;
            case 3:
                Sendcommand(CMD_MoveJoint,0,0,0, 34.94, -47.04, 118.19, 62.98, 55.26, -141.9, spd_80);
                break;
            default:
                printf("no search pose %d\n",_goal->sub_cmd);
                return;
            }
        }else if(_id == KIMBAP2)
        {
            object_name = "kimbap2";
            switch(_goal->sub_cmd)
            {
            case 0:
                Sendcommand(CMD_MoveJoint,0,0,0,-127.16, -8.17, 93.33, 3.92, 118.06, 95.88, spd_80);
                Sendcommand(CMD_MoveJoint,0,0,0,-145.87, 20.53, 78.59, 30.96, 143.31, 136.89, spd_80);
                break;
            case 1:
                Sendcommand(CMD_MoveJoint,0,0,0,-27.89, 18.5, 93.73, -150.46, -53.9, 24.92, spd_80);
                break;
            case 2:
                Sendcommand(CMD_MoveJoint,0,0,0,-126.48, -34.46, 98.08, 159.93, -112.1, -154.93, spd_80);
                break;
            default:
                printf("no search pose %d\n",_goal->sub_cmd);
                return;
            }
        }
        break;
    }
    case BASKET:
    {
        Sendcommand(CMD_MoveJoint,home_pose[JDEFAULTHOME],spd_80);

        switch(_goal->object_id)
        {
        case KIMBAP1:
            object_name = "kimbap1 (basket)";
            Sendcommand(CMD_MoveJoint, 0, 0, 0, -213.88, 44.03, 81.54, 19.62, 61.15, -161.44, spd_80);
            break;
        case KIMBAP2:
            object_name = "kimbap2 (basket)";
            Sendcommand(CMD_MoveJoint, 0, 0, 0, -213.88, 44.03, 81.54, 19.62, 61.15, -161.44, spd_80);
            break;
        case SANDWICH:
            object_name = "sandwich (basket)";
            Sendcommand(CMD_MoveJoint, 0, 0, 0, -211.24, 49.43, 70.97, 25.6, 63.32, -163.12, spd_80);
            break;
        case HAMBUG:
            object_name = "hambug (basket)";
            Sendcommand(CMD_MoveJoint, 0, 0, 0, -211.24, 49.43, 70.97, 25.6, 63.32, -163.12, spd_80);
            break;
        case COFFEE:
            object_name = "coffee (basket)";
            Sendcommand(CMD_MoveJoint, 0, 0, 0, 82.87, 18.91, 106.7, -35.61, 90.01, 7.12, spd_80);
            break;
        case LUNCHBOX:
            object_name = "lunchbox (basket)";
            Sendcommand(CMD_MoveJoint, 0, 0, 0, 80.34, 1.63, 128.19, -39.81, 90.01, 9.65, spd_80);
            break;
        default:
            return;
        }
        break;
    }
    case TEMP:
    {
        object_name = "old lunchbox";
        Sendcommand(CMD_MoveJoint , search_pose[TEMP_LUNCHBOX], spd_80);
        break;
    }
    }

    ROS_INFO("Going Search :: %s, %d", object_name, _goal->sub_cmd);

    usleep(500*1000);
    MotionDone = DONE;
}

void GraspAction::goGrasp(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    if(_goal->motion_cmd != GRASP_OBJECT)
        return;

    int tool = _goal->tool_id;

    usleep(500*1000);
    /*---------------------transform base to marker--------------------------*/
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

    tf::Vector3 temp;
    temp = transform.getOrigin();
    Robject = transform.getBasis();
    double rr,pp,yy;
    Robject.getEulerYPR(yy,pp,rr);

    int _id = _goal->object_id;
    int _shelf = _goal->shelf_id;
    float desired_z = shelf_marker[_shelf+5].z + Info_Object[_id].height;

    ROS_INFO("\n");
    ROS_INFO("-----------------------------------------------");
    ROS_INFO("Object %d detected : %f, %f, %f", _id, temp.x(), temp.y(), temp.z());
    ROS_INFO("RPY : %f, %f, %f", rr,pp,yy);
    ROS_INFO("desired Z : %f (coffee = -0.469)",desired_z);

    float new_x = temp.x();
    float new_y = temp.y();
    float new_z = temp.z();

    //-0.592 + 0.123 = -0.469

    if(_id == LUNCHBOX && fabs(temp.z()-desired_z) > Info_Object[LUNCHBOX].height - limit_marker_offset_z)
    {
        ROS_ERROR("Marker z is changed %f to %f",temp.z(),desired_z);
        new_z = desired_z + Info_Object[LUNCHBOX].height;
    }else if(fabs(temp.z()-desired_z) > limit_marker_z)
    {
        ROS_ERROR("Marker z is changed %f to %f",temp.z(),desired_z);
        //new_z = desired_z;
    }

    Pobject.setValue(new_x, new_y, new_z);
    transform.setOrigin(Pobject);
    ROS_INFO("-----------------------------------------------");
    ROS_INFO("\n");

    /*---------------------------- check object pose -------------------------------*/
    switch(_shelf)
    {
    case SHELF1:
    case SHELF2:
    {
        if(Pobject.y() < I_S[_goal->shelf_id].limit)
        {
            ROS_ERROR("RB5 can't grasp this object [y]");
            MotionDone = INPUT_ERROR;
            return;
        }
        break;
    }
    case SHELF3:
    {
        float length = sqrt(Pobject.x()*Pobject.x() + Pobject.y()*Pobject.y());
        if(length > I_S[_shelf].limit)
        {
            ROS_ERROR("RB5 can't grasp this object [length = %f]",length);
            MotionDone = INPUT_ERROR;
            return;
        }
        break;
    }
    default:
        break;
    }
    ROS_INFO("RB5 can grasp this object..maybe..");






    tool = Info_Object[_id].default_toolmode;
    float basket_z_lift = 0.0;
    float basket_z_approach = 0.0;
    float z_approach = z_grasp_approach;
    if(_shelf == BASKET)
    {
        basket_z_lift = 0.1;
        basket_z_approach = 0.05;
    }else if(_shelf == TEMP)
    {
        z_approach = 0.05;
    }


    //-------------------------------  Approach  -----------------------------------//
    if(tool == TOOLMODE_GRIPPER)
    {
        //approach to target + 5cm(z_approach) + 10cm(only basket)
        if(_goal->object_id == KIMBAP1 || _goal->object_id ==KIMBAP2)
        {

            Rtarget.setEulerYPR(yy, 0, 90*D2R);
            z_approach += 0.05;
        }else
        {
            Rtarget.setEulerYPR(yy-90*D2R, 0, 90*D2R);
        }

        Ptarget.setValue(Pobject.x(), Pobject.y(), Pobject.z() + z_approach + basket_z_approach);

        PrintCoordinate("Gripper Target",Ptarget,Rtarget);
        TCPtoRB5(TOOLMODE_GRIPPER);

        ROS_INFO("GRASP::Open Gripper");
        if(_goal->sub_cmd == LINE1)
        {
            openGripper(GRIPPER_SECOND);
        }else
        {
            openGripper(GRIPPER_FIRST);
        }

    }else if(tool == TOOLMODE_SUCTION)
    {
        Ptarget.setValue(Pobject.x(), Pobject.y(), Pobject.z() + z_approach + basket_z_approach);
        Rtarget.setEulerYPR(yy, 0, 90*D2R);
        PrintCoordinate("Suction Target",Ptarget,Rtarget);
        TCPtoRB5(TOOLMODE_SUCTION);
    }


    //---------------------------------Approach----------------------------------------//
    ROS_INFO("GRASP::Approach pose1");
    PrintCoordinate("Approach",x_real,y_real,z_real,r_real,p_real,oy_real);

    Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real, spd_60);


    ROS_INFO("GRASP::Approach pose2");
    z_real -= (z_approach + z_grasp_offset[tool] + basket_z_approach)*1000;
    PrintCoordinate("Approach2",x_real,y_real,z_real,r_real,p_real,oy_real);

    //--------------------------------------Suction-------------------------------------//
    if(tool == TOOLMODE_GRIPPER)
    {
        Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real, spd_approach);
        ROS_INFO("GRASP::Close Gripper");
        closeGripper();
    }else if(tool == TOOLMODE_SUCTION)
    {
        Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real, spd_10);
        ROS_INFO("GRASP::Suction");
        suction(0);
        suction(1);
    }

    //-------------------------------------------Lift-----------------------------------//
    ROS_INFO("GRASP::Lift");
    z_real += (Info_Object[_id].z_lift + basket_z_lift)*1000;
    PrintCoordinate("Lift",x_real,y_real,z_real,r_real,p_real,oy_real);
    Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,spd_lift);

    switch(_shelf)
    {
    case SHELF1:
        break;
    case SHELF2:
        break;
    case SHELF3:
        break;
    case BASKET:
        switch(_id)
        {
        case KIMBAP1:
        case KIMBAP2:
            Sendcommand(CMD_MoveJoint,via_point[JBASKETTO3],spd_80);
            break;
        case SANDWICH:
        case HAMBUG:
        case COFFEE:
        case LUNCHBOX:
            Sendcommand(CMD_MoveJoint,via_point[JBASKETTO12],spd_80);
            break;
        }
        break;
    case TEMP:
        Sendcommand(CMD_MoveJoint,via_point[JTEMPTOSHELF],spd_80);
        break;
    }

    ROS_INFO("GRASP::Completed ;D\n");
    MotionDone = DONE;
}

void GraspAction::goPut(const rb5_ros_wrapper::manipulationGoalConstPtr _goal, Pose _pose)
{
    int tool = _goal->tool_id;
    int _shelf = _goal->shelf_id;
    int _mode = _goal->sub_cmd;
    int _id = _goal->object_id;

    //need to erase this for adding suction picking mode
    tool = Info_Object[_goal->object_id].default_toolmode;

    switch(_mode)
    {
    case DISPOSE:
    {
        if(_shelf == SHELF1)
        {
            ROS_INFO("PUT::Lift");
            Sendcommand(CMD_MoveTCP,via_point[TLUNCHBOX_VIA1],spd_80);
            ROS_INFO("PUT::VIA1");
            Sendcommand(CMD_MoveJoint,via_point[JDISPOSE_VIA1],spd_80);
            ROS_INFO("PUT::Dispose");
            Sendcommand(CMD_MoveJoint,via_point[JDISPOSE],spd_80);
        }else if(_shelf == SHELF3)
        {
            ROS_INFO("PUT::Lift");
            Sendcommand(CMD_MoveTCP, via_point[TGRASP_SHELF3],spd_60);
            ROS_INFO("PUT::Dispose");
            Sendcommand(CMD_MoveJoint,via_point[JDISPOSE],spd_80);
        }else if(_shelf == SHELF2)
        {
            ROS_INFO("PUT::Lift");
            Sendcommand(CMD_MoveJoint,via_point[JDISPOSE_VIA1],spd_80);
            ROS_INFO("PUT::Dispose");
            Sendcommand(CMD_MoveJoint,via_point[JDISPOSE],spd_80);
        }
        break;
    }
    case LUNCHBOX_PUT:
    {
        if(_goal->object_id == LUNCHBOX)
        {
            Sendcommand(CMD_MoveTCP,via_point[TLUNCHBOX_VIA1],spd_80);
        }
//        Sendcommand(CMD_MoveJoint,via_point[JLUNCHBOX_VIA2],spd_80);
        Sendcommand(CMD_MoveJoint,via_point[JLUNCHBOX_PUT],spd_80);

        break;
    }
    case LINE1:
    case LINE2:
    case LINE3:
    {
        /* pos_x, pos_y - 0.03, pos_z + 0.1~0.2 */
        _pose.cdn[1] -= y_put_object*1000;
        _pose.cdn[2] += Info_Object[_goal->object_id].z_lift*1000;
        TCPtoRB5(_pose, tool);
        PrintCoordinate("Put 1",x_real,y_real,z_real,r_real,p_real,oy_real);
        Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real, spd_approach);


        /* pos_x, pos_y - 0.03, pos_z + 0.02(only suction) */
        _pose.cdn[2] -= (Info_Object[_goal->object_id].z_lift - z_put_offset[tool])*1000;


        //shelf tunning
        if(_mode == LINE1)
        {
            if(_id == HAMBUG || _id == COFFEE)
            {
                ROS_INFO("Hambug,Coffee(Line1) -20z");
                _pose.cdn[2] -= 20;
            }else
            {
                ROS_INFO("Line1 -15z");
                _pose.cdn[2] -= 15;

            }
        }else if(_mode == LINE2)
        {
            if(_id == HAMBUG)
            {
                ROS_INFO("Hambug(Line2) +20z");
                _pose.cdn[2] += 20;
            }else if(_id == KIMBAP1 || _id == KIMBAP2)
            {
                ROS_INFO("Kimbap(Line1) -10z");
                _pose.cdn[2] -= 10;
            }
        }else if(_mode == LINE3)
        {
            if(_id == KIMBAP1 || _id == KIMBAP2)
            {
                ROS_INFO("Kimbap(Line1) -5z");
                _pose.cdn[2] -= 5;
            }
        }

        TCPtoRB5(_pose, tool);
        PrintCoordinate("Put 2",x_real,y_real,z_real,r_real,p_real,oy_real);
        Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,spd_approach);


        if(_goal->object_id == LUNCHBOX || _goal->sub_cmd == LINE1)
        {//no over pull
            y_real += y_put_object*1000;
            PrintCoordinate("Put 3",x_real,y_real,z_real,r_real,p_real,oy_real);
            Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real, spd_10);
        }else
        {
            /* pull object over 1cm */
            y_real += y_pull_object*1000;
            PrintCoordinate("Put 3",x_real,y_real,z_real,r_real,p_real,oy_real);
            Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real, spd_10);
        }
        break;
    }
    }

    if(tool == TOOLMODE_GRIPPER)
    {
        openGripper(GRIPPER_PUT);
    }else
    {
        suction(2);
    }

    switch(_goal->sub_cmd)
    {
    case DISPOSE:
        break;
    case LUNCHBOX_PUT:
        Sendcommand(CMD_MoveTCP,via_point[TLUNCHBOX_PUT_LIFT], spd_80);
        break;
    default:
        z_real += Info_Object[_goal->object_id].z_lift*1000;
        Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real, spd_lift);

        if(_shelf == SHELF1)
        {
            Sendcommand(CMD_MoveJoint,via_point[JSHELF1TOIDLE],spd_80);
        }
        break;
    }

    ROS_INFO("GRASP::Put Completed ;D\n\n");
    goHome(IDLEPOSE);
}

void GraspAction::pullshelf(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    if(_goal->motion_cmd != PULL_SHELF)
        return;

    int _shelf = _goal->shelf_id;
    int _id = _goal->object_id;
    switch(_goal->sub_cmd)
    {
    case SEARCH:
    {
        //-----------------search pose set------------------------//
        int seq = 0;
        switch(_shelf)
        {
        case SHELF1:
            ROS_INFO("PULL SHELF::Search pose1");
            motion = Shelf_Search1;
            break;
        case SHELF2:
            ROS_INFO("PULL SHELF::Search pose2");
            motion = Shelf_Search2;
            break;
        case SHELF3:
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
        //----------------- listen tf shelf ----------------------//
        usleep(500*1000);
        try
        {
            ros::Time now = ros::Time::now();
            listener.lookupTransform("/rb5/base", "/shelf", ros::Time(0), transform_shelf);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }


        tf::Vector3 temp;
        temp = transform_shelf.getOrigin();
        ROS_INFO("\n");
        ROS_INFO("-----------------------------------------------");
        ROS_INFO("Front Shelf %d detected : %f, %f, %f",_goal->shelf_id, temp.x(), temp.y(), temp.z());

        float new_x = temp.x();
        float new_y = temp.y();
        float new_z = temp.z();
        switch(_shelf)
        {
        case SHELF1:
            if(fabs(temp.y()-shelf_marker[FRONT1].y) > 0.1)//limit_marker_offset_f)
            {
                ROS_ERROR("Marker y is changed %f to %f",temp.y(),shelf_marker[FRONT1].y);
                new_y = shelf_marker[FRONT1].y;
            }
            if(fabs(temp.z()-shelf_marker[FRONT1].z) > 0.1)//limit_marker_offset_z)
            {
                ROS_ERROR("Marker z is changed %f to %f",temp.z(),shelf_marker[FRONT1].z);
                new_z = shelf_marker[FRONT1].z;
            }
            break;
        case SHELF2:
            if(fabs(temp.y()-shelf_marker[FRONT2].y) > limit_marker_offset_f)
            {
//                ROS_ERROR("Marker y is changed %f to %f",temp.y(),shelf_marker[FRONT2].y);
//                new_y = shelf_marker[FRONT2].y;
            }
            if(fabs(temp.z()-shelf_marker[FRONT2].z) > limit_marker_offset_z)
            {
//                ROS_ERROR("Marker z is changed %f to %f",temp.z(),shelf_marker[FRONT2].z);
//                new_z = shelf_marker[FRONT2].z;
            }
            break;
        case SHELF3:
            if(fabs(temp.y()-shelf_marker[FRONT3].y) > limit_marker_offset_f)
            {
//                ROS_ERROR("Marker y is changed %f to %f",temp.y(),shelf_marker[FRONT3].y);
//                new_y = shelf_marker[FRONT3].y;
            }
            if(fabs(temp.z()-shelf_marker[FRONT3].z) > limit_marker_offset_z)
            {
//                ROS_ERROR("Marker z is changed %f to %f",temp.z(),shelf_marker[FRONT3].z);
//                new_z = shelf_marker[FRONT3].z;
            }
            break;
        default:
            ROS_ERROR("Wrong Shelf number!!!!");
            break;
        }

        Pshelf.setValue(new_x, new_y, new_z);
        transform_shelf.setOrigin(Pshelf);
        ROS_INFO("-----------------------------------------------");
        ROS_INFO("\n");



        //------------------------- set target pos --------------------------//
        Rshelf = transform_shelf.getBasis();
        Ptarget = transform_shelf.getOrigin();
        Rtarget.setEulerYPR(0, 0, 0);

        TCPtoRB5(TOOLMODE_SUCTION);

        switch(_shelf)
        {
        case SHELF1:
        {
            openGripper();
            TCPtoRB5(TOOLMODE_GALGORI);
            Sendcommand(CMD_MoveJoint,0,0,0,-60, 77.8, 120.82, -108.59, 89.94, -30.24, spd_80);

            Sendcommandwheel('W', 0, 1, 0.3, 0, 0);

            Sendcommand(CMD_MoveTCP,0,0,0,0,-550, -330, 90, 0, 90, spd_80);

            Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real+20,90,0,90,spd_approach);

            Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,90,0,90,spd_10);

            Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real+300,z_real,90,0,90,spd_10);

            Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real+100,z_real+100,90,0,90,spd_approach);

            closeGripper();

            Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real+100,z_real+20,90,0,90,spd_approach);

            Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real+250,z_real+20,90,0,90,spd_10);

            Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real+200,z_real+100,90,0,90,spd_approach);

            Sendcommand(CMD_MoveTCP,0,0,0,x_real,-520,-230,80,0,0,spd_60);

            closeGripper();
            break;
        }
        case SHELF2:
        case SHELF3:
        {
            //-----------------shelf approach---------------------//
            ROS_INFO("PULL SHELF::Approach pose1");
            y_real+=50;
            Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real, spd_approach);


            y_real-=50;
            ROS_INFO("PULL SHELF::Approach pose2");
            Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real, spd_10);

            //--------------------Suction-------------------------//
            ROS_INFO("PULL SHELF::Suction");
            suction(0); //reset

            suction(1); //suction

            //--------------------Pull shelf------------------------//
            ROS_INFO("PULL SHELF::Pull");
            if(_shelf == SHELF2)
            {
                printf("shelf2\n");
                y_real += 400;
                Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,spd_10);
            }else if(_shelf == SHELF3)
            {
                printf("shelf3\n");
                y_real += 400;
                Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,spd_10);
            }

            //--------------------release suction -----------------------//
            ROS_INFO("PULL SHELF::Release");
            suction(2); //release


            //----------------------Go to idle pose---------------------//
            if(_shelf == SHELF2)
            {
                ROS_INFO("PULL_SHELF::Final pose2");
                z_real += 100;
                y_real += 20;
                Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,spd_60);


                ROS_INFO("PULL_SHELF::Shelf search pose2");
                Sendcommand(CMD_MoveJoint,0,0,0,-77.34, 17.82, 122.3, -59.88, 87.82, -12.48, spd_80);


                ROS_INFO("Wheel move +0.3x");
                Sendcommandwheel('W', 0, 1, 0.3, 0, 0);
            }else if(_shelf == SHELF3)
            {
                ROS_INFO("PULL_SHELF::Final pose3");
                z_real += 100;
                y_real += 20;
                Sendcommand(CMD_MoveTCP,0,0,0,x_real,y_real,z_real,r_real,p_real,oy_real,spd_60);


                ROS_INFO("PULL_SHELF::Shelf search pose3");
                Sendcommand(CMD_MoveJoint,search_pose[JSHELF3_BACK],spd_80);
                Sendcommand(CMD_MoveJoint,0,0,0,-77.34, -0.37, 78.9, 1.71, 87.83, -12.47, spd_80);


                ROS_INFO("Wheel move +0.3x");
                Sendcommandwheel('W', 0, 1, 0.3, 0, 0);
            }
            break;
        }
        default:
        {
            MotionDone = INPUT_ERROR;
            return;
        }
        }
        break;
    }
    default:
        break;
    }
    MotionDone = DONE;
    return;
}

void GraspAction::pushshelf(const rb5_ros_wrapper::manipulationGoalConstPtr _goal)
{
    int _shelf = _goal->shelf_id;
    //-----------------Init pose set------------------------//
    int seq = 0;
    switch(_shelf)
    {
    case SHELF1:
        ROS_INFO("Wheel move -0.3x");
        Sendcommandwheel('W', 0, 1, -0.1, 0, 0);
        ROS_INFO("PUSH SHELF::Init pose1");
        motion = Shelf_Push1;
        break;
    case SHELF2:
        ROS_INFO("Wheel move -0.3x");
        Sendcommandwheel('W', 0, 1, -0.3, 0, 0);

        ROS_INFO("PUSH SHELF::Init pose2");
        motion = Shelf_Push2;
        break;
    case SHELF3:
        ROS_INFO("Wheel move -0.3x");
        Sendcommandwheel('W', 0, 1, -0.3, 0, 0);

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

    if(_shelf == SHELF1)
    {
        ROS_INFO("Wheel move -0.3x");
        Sendcommandwheel('W', 0, 1, -0.2, 0, 0);
        goHome(SHELF1);
    }else
    {
        goHome(IDLEPOSE);
    }

    ROS_INFO("PUSH SHELF::Search pose done");
    MotionDone = DONE;
}

void GraspAction::pause(float time)
{
    Sendcommand(CMD_MotionPause,0,0,0,0,0, 0, 0, 0, 0,0,0);
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
    Sendcommand(CMD_MotionResume,0,0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Starting again.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
}

void GraspAction::TCPtoRB5(int _mode)
{
    double rr,pp,yy;
    Rtarget.getEulerYPR(yy,pp,rr);
    printf("Target Pose : %f, %f, %f, %f, %f, %f\n",Ptarget.x()*1000, Ptarget.y()*1000, Ptarget.z()*1000, rr*R2D, pp*R2D, yy*R2D);

    tf::Matrix3x3 Rtool;

    tf::Vector3 Ptool, Poffset;

    if(_mode == TOOLMODE_GRIPPER)
        Poffset.setValue(-0.00035, 0.22122, 0.);
    else
        Poffset.setValue(-0.0235, 0.24, 0.);

    Rtool = Rtarget;
    Ptool = Ptarget + Rtool*Poffset;

    Rtool.getEulerYPR(yy,pp,rr);

    x_real = Ptool.x()*1000;
    y_real = Ptool.y()*1000;
    z_real = Ptool.z()*1000;
    r_real = (float)rr*R2D;
    p_real = (float)pp*R2D;
    oy_real= (float)yy*R2D;

    if(_mode == TOOLMODE_GALGORI)
    {
        printf("mode is galgori\n");
        x_real = Ptarget.x()*1000;
        y_real = Ptarget.y()*1000+ 410;
        z_real = Ptarget.z()*1000+ 210;
        r_real = 90;
        p_real = 0;
        oy_real = 90;
    }

    printf("TCPtoRB5 : %f, %f, %f, %f, %f, %f\n",x_real, y_real, z_real, r_real, p_real, oy_real);
}

void GraspAction::TCPtoRB5(Pose _pose,int _mode)
{
    tf::Matrix3x3 Rtcp;
    tf::Vector3 Ptcp;

    Ptcp.setValue(_pose.cdn[0]/1000.,_pose.cdn[1]/1000.,_pose.cdn[2]/1000.);
    Rtcp.setEulerYPR(_pose.cdn[5]*D2R, _pose.cdn[4]*D2R, _pose.cdn[3]*D2R);

    double rr,pp,yy;
    Rtcp.getEulerYPR(yy,pp,rr);
    printf("Target Pose : %f, %f, %f, %f, %f, %f\n",Ptcp.x()*1000, Ptcp.y()*1000, Ptcp.z()*1000, rr*R2D, pp*R2D, yy*R2D);

    tf::Matrix3x3 Rtool;

    tf::Vector3 Ptool, Poffset;

    if(_mode == TOOLMODE_GRIPPER)
        Poffset.setValue(-0.00035, 0.22122, 0.);
    else
        Poffset.setValue(-0.0235, 0.24, 0.);

    Rtool = Rtcp;
    Ptool = Ptcp + Rtool*Poffset;

    Rtool.getEulerYPR(yy,pp,rr);

    x_real = Ptool.x()*1000;
    y_real = Ptool.y()*1000;
    z_real = Ptool.z()*1000;
    r_real = (float)rr*R2D;
    p_real = (float)pp*R2D;
    oy_real= (float)yy*R2D;
    printf("TCPtoRB5 : %f, %f, %f, %f, %f, %f\n",x_real, y_real, z_real, r_real, p_real, oy_real);
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
    rb5_ros_wrapper::manipulationGoalConstPtr goal;
    GraspAction newGrasp;

    Pose Tput_pose;
    Pose Jput_pose;
public:

    manipulationAction(std::string name) :
        as_(nh_, name, boost::bind(&manipulationAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
        newGrasp.doInit();
    }

    ~manipulationAction(void)
    {
    }

    //only called when client requests goal
    void executeCB(const rb5_ros_wrapper::manipulationGoalConstPtr &_goal)
    {
        goal = _goal;

        newGrasp.PrintGoal(goal);
        newGrasp.MotionDone = ACCEPT;

        switch(goal->motion_cmd)
        {
        case HOME_POSE:
            newGrasp.goHome(IDLEPOSE);
            break;
        case OBJECT_SEARCH:
            newGrasp.goSearch(goal);
            break;
        case GRASP_OBJECT:
            newGrasp.goGrasp(goal);
            break;
        case PUT_OBJECT:
            goPut();
            break;
        case PULL_SHELF:
            newGrasp.pullshelf(goal);
            break;
        case PUSH_SHELF:
            newGrasp.pushshelf(goal);
            break;
        case DETECT_SHELF:
            newGrasp.getShelfPose(goal);
            break;
        default:
            break;
        }

        while(newGrasp.MotionDone == ACCEPT)
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

        result_.result_flag = newGrasp.MotionDone;
        sendResult(newGrasp.MotionDone);

        newGrasp.MotionDone = IDLE;
    }

    void sendResult(int flag)
    {
        if(flag == DONE)
        {
            result_.result_flag = flag;
            as_.setSucceeded(result_);
        }else
        {
            result_.result_flag = flag;
            as_.setAborted(result_);
        }
    }

    //motion function call
    int goPut();
    void calPutoffset();
};

int manipulationAction::goPut()
{
    if(goal->motion_cmd != PUT_OBJECT)
        return CMD_ERROR;

    switch(goal->sub_cmd)
    {
    case DISPOSE:
        newGrasp.goPut(goal, Jput_pose);
        break;
    case LINE1:
        calPutoffset();
        newGrasp.goPut(goal, Tput_pose);
        break;
    case LINE2:
        calPutoffset();
        newGrasp.goPut(goal, Tput_pose);
        break;
    case LINE3:
        calPutoffset();
        newGrasp.goPut(goal, Tput_pose);
        break;
    case LUNCHBOX_PUT:
        calPutoffset();
        newGrasp.goPut(goal, Tput_pose);
        break;
    default:
        break;
    }
}

void manipulationAction::calPutoffset()
{
    float depth =       Info_Object[goal->object_id].depth;
    float first_depth = Info_Object[goal->object_id].first_depth;
    float height =      Info_Object[goal->object_id].height;
    float line_offset = (Info_Object[goal->object_id].line1 + Info_Object[goal->object_id].line2)/2.;


    tf::Vector3 Pshelf;
    Pshelf = newGrasp.transform_shelf.getOrigin();
    printf("shelf origin = %f, %f, %f\n", Pshelf.x(), Pshelf.y(), Pshelf.z());

    float lunchbox_z_offset = 0.;
    if(goal->object_id == LUNCHBOX)
    {
        lunchbox_z_offset = height*(goal->sub_cmd - 1);
    }

    float x_offset = line_offset;
    float y_offset = shelf_y_offset - first_depth - depth*(goal->sub_cmd - 1);
    float z_offset = height + lunchbox_z_offset;

    Tput_pose.cdn[0] = (Pshelf.x() + x_offset)*1000.;  //center of line x
    Tput_pose.cdn[1] = (Pshelf.y() + y_offset)*1000.;  //shelf y + object y...
    Tput_pose.cdn[2] = (Pshelf.z() + z_offset)*1000.;  //shelf z + object z
    Tput_pose.cdn[3] = 90.;
    Tput_pose.cdn[4] = 0.;

    if(goal->object_id == SANDWICH || goal->object_id == HAMBUG)
        Tput_pose.cdn[5] = -90.;
    else
        Tput_pose.cdn[5] = 0.;

    printf("Offset = %f, %f, %f\n", x_offset, y_offset, z_offset);
    printf("Tput_pose = %f, %f, %f\n", Tput_pose.cdn[0], Tput_pose.cdn[1], Tput_pose.cdn[2]);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grasp_motion");

    manipulationAction manipulation("manipulation");

    ros::spin();
    return 0;
}
