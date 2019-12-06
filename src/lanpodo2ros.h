#ifndef LANPODO2ROS_H
#define LANPODO2ROS_H

struct Update{
    int     robot_state;            //state of robot motion [1:idle   2:paused/stopped by accident   3:moving]
    int     power_state;            //power state
    int     program_mode;           //current program mode [0:real mode   1:simulation mode]

    int     collision_detect;       //collision dectect onoff [0:off   1:on]
    int     freedrive_mode;         //current freedrive status [0:off   1:on]

    float   speed;                  //overriding speed [0~1]

    float   joint_angles[6];        //[deg]
    float   joint_references[6];    //[deg]
    float   joint_current[6];       //[mA]
    float   joint_temperature[6];   //[celcius]
    int     joint_information[6];   //look mSTAT

    float   tcp_reference[6];
    float   tcp_position[6];

    float   tool_reference;         //reference voltage of tool flalnge board [0, 12, 24]

    float   ft_sensor[6];           //mx, my, fz, fx, fy, mz

};
struct Result
{
    int     rb5_result;
    int     wheel_result;
};
#endif // LANPODO2ROS_H
