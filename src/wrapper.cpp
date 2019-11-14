#include <ros/ros.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <chrono>
#include <actionlib/server/simple_action_server.h>
#include <rb5_ros_wrapper/MotionAction.h>
#include <math.h>
#include "rb5_ros_wrapper/update.h"
#include "lanros2podo.h"
#include "lanpodo2ros.h"
#define robot_idle      1
#define robot_paused    2
#define robot_moving    3
#define real_mode       0
#define simul_mode      1
#define PORT1 4000
#define PORT2 4001
#define IPAddr "127.0.0.1"

int sock_status = 0, valread;
int sock_result = 0;
char buffer[1024] = {0};
struct sockaddr_in ROSSocket;
struct sockaddr_in RSTSocket;
LANROS2PODO TX;
LANPODO2ROS RX;
RESULT      RXresult;

enum {
    BREAK = 0,
    ACCEPT,
    DONE,
    STATE_ERROR,
    INPUT_ERROR,
    ERROR_STOP
};

ros::Publisher robot_states_pub;
rb5_ros_wrapper::update message;

bool connectROS()
{
    if((sock_status = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Error creating socket \n");
        return false;
    }

    ROSSocket.sin_family = AF_INET;
    ROSSocket.sin_port = htons(PORT1);

    if(inet_pton(AF_INET, IPAddr, &ROSSocket.sin_addr)<=0)
    {
        printf("\n Invalid Address \n");
        return false;
    }

    if(connect(sock_status, (struct sockaddr *)&ROSSocket, sizeof(ROSSocket)) < 0)
    {
        printf("\n Connection failed \n");
        return false;
    }

    printf("\n Client connected to server!(ROS)\n");
    return true;
}

bool connectRST()
{
    if((sock_result = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Error creating socket \n");
        return false;
    }

    RSTSocket.sin_family = AF_INET;
    RSTSocket.sin_port = htons(PORT2);

    if(inet_pton(AF_INET, IPAddr, &RSTSocket.sin_addr)<=0)
    {
        printf("\n Invalid Address \n");
        return false;
    }

    if(connect(sock_result, (struct sockaddr *)&RSTSocket, sizeof(RSTSocket)) < 0)
    {
        printf("\n Connection failed \n");
        return false;
    }

    printf("\n Client connected to server!(RST)\n");
    return true;
}


class MotionAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<rb5_ros_wrapper::MotionAction> as_;
    std::string action_name_;

    rb5_ros_wrapper::MotionFeedback feedback_;
    rb5_ros_wrapper::MotionResult result_;

public:

    MotionAction(std::string name) :
        as_(nh_, name, boost::bind(&MotionAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~MotionAction(void)
    {
    }

    void writeTX(const rb5_ros_wrapper::MotionGoalConstPtr &goal)
    {
        //char *buffed = new char[TX.size];
        void *buffed;

        //send over the TX motion data
        TX.command.type = goal->type;
        TX.command.d0 = goal->d0;
        TX.command.d1 = goal->d1;

        std::cout << "TX.command.d0 = " << TX.command.d0 << std::endl;
        std::cout << "TX.command.d1 = " << TX.command.d1 << std::endl;

        TX.command.data = goal->data;
        for(int i = 0; i < 6; i++)
        {
            TX.command.coordinate[i] = goal->coordinate[i];
        }
        for(int i=0;i<3;i++)
        {
            TX.command.wheel[i] = goal->wheel[i];
        }

        TX.command.spd = goal->spd;
        TX.command.acc = goal->acc;


        buffed = (void*)malloc(TX.size);
        memcpy(buffed, &TX.command, TX.size);

        send(sock_status, buffed, TX.size, 0);

        free(buffed);
        buffed = nullptr;

        return;
    }

    //only called when client requests goal
    void executeCB(const rb5_ros_wrapper::MotionGoalConstPtr &goal)
    {
        ros::Rate r(30);
        bool success = true;

        //===write to RB5===
        int rxDoneFlag = 0;
        int activeFlag = 0;

        //write TX to RB5
        writeTX(goal);

        //loop until TX complete

        while(rxDoneFlag == 0)
        {
            //check that preempt has not been requested by client
            if(as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                break;
            }

            read(sock_result,RXresult.buffer,RXresult.size);
            memcpy(&RXresult.message, RXresult.buffer, RXresult.size);

            //check result flag
            switch(RXresult.message.rb5result)
            {
            case ACCEPT:
                if(activeFlag != true)
                {
                    ROS_INFO("PODO accept rb5 command");
                    activeFlag = true;
                }
                break;
            case STATE_ERROR:
            case INPUT_ERROR:
            case DONE:
            case ERROR_STOP:
                rxDoneFlag = 1;
                break;
            }

            switch(RXresult.message.wheelresult)
            {
            case ACCEPT:
                if(activeFlag != true)
                {
                    ROS_INFO("PODO accept wheel command");
                    activeFlag = true;
                }
                break;
            case STATE_ERROR:
            case INPUT_ERROR:
            case DONE:
            case ERROR_STOP:
                rxDoneFlag = 1;
                break;
            }

            if(returnServerStatus())
            {
                publishFeedback();
            }

            //result setting
            result_.rb5result = RXresult.message.rb5result;
            result_.wheelresult = RXresult.message.wheelresult;

            //maintain desired loop rate
            r.sleep();
        }

        if(success)
        {
            ROS_INFO("%s: Succeeded");
            as_.setSucceeded(result_);
        }
    }

    int returnServerStatus()
    {
        if(as_.isActive())
            return 1;
        else
            return 0;
    }

    void publishFeedback()
    {
        std::cout << "Publishing Feedback" << std::endl;
        if(RX.message.robot_state == robot_moving)
        {
            feedback_.state = "Moving";
            printf("RobotMoving\n");
        }else if(RX.message.robot_state == robot_paused)
            feedback_.state = "Paused or Collision";
        else
            feedback_.state = "Idle";

        if(RX.message.program_mode == real_mode)
            feedback_.mode = "Real";
        else
            feedback_.mode = "Simulation";

        feedback_.collision = RX.message.collision_detect;
        feedback_.freedrive = RX.message.freedrive_mode;
        feedback_.speed = RX.message.speed;

        for(int i = 0; i < 6; i++)
        {
            feedback_.joint_ang[i] = RX.message.joint_angles[i];
            feedback_.joint_ref[i] = RX.message.joint_references[i];
            feedback_.joint_cur[i] = RX.message.joint_current[i];
            feedback_.joint_temp[i] = RX.message.joint_temperature[i];
            feedback_.joint_info[i] = RX.message.joint_information[i];

            feedback_.tcp_ref[i] = RX.message.tcp_reference[i];
            feedback_.tcp_pos[i] = RX.message.tcp_position[i];
        }

        feedback_.tool_ref = RX.message.tool_reference;

        as_.publishFeedback(feedback_);
    }
};

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "rb5_ros_wrapper");

    ros::NodeHandle n;
    robot_states_pub = n.advertise<rb5_ros_wrapper::update>("robot_states",1);

    if(connectROS() == false)
    {
        printf("waitForResult\n\n Failed to connect. Closing...\n");
        return -1;
    }
    if(connectRST() == false)
    {
        printf("waitForResult\n\n Failed to connect. Closing...\n");
        return -1;
    }

    ROS_INFO("Starting Action Server");
    MotionAction motion("motion");

    while(1)
    {
        //read robot status from PODO
        read(sock_status, RX.buffer, RX.size);
        memcpy(&RX.message, RX.buffer, RX.size);

        //publish robot status
        message.robot_state = RX.message.robot_state;
        message.power_state = RX.message.power_state;
        message.program_mode = RX.message.program_mode;
        message.collision_detect = RX.message.collision_detect;
        message.freedrive_mode = RX.message.freedrive_mode;
        message.speed = RX.message.speed;
        message.tool_reference = RX.message.tool_reference;

        for(int i=0;i<6;i++)
        {
            message.joint_angles[i] = RX.message.joint_angles[i];
            message.joint_references[i] = RX.message.joint_references[i];
            message.joint_current[i] = RX.message.joint_current[i];
            message.joint_temperature[i] = RX.message.joint_temperature[i];
            message.joint_information[i] = RX.message.joint_information[i];
            message.tcp_reference[i] = RX.message.tcp_reference[i];
            message.tcp_position[i] = RX.message.tcp_position[i];
        }
        robot_states_pub.publish(message);


        //callback check
        ros::spinOnce();
    }

    return 0;
}
