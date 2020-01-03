# lanpodo2ros.h

int32       robot_state         #1:idle 2:paused/stopped by accident 3:moving
int32       power_state
int32       program_mode        #0:realmode 1:simulationmode

int32       collision_detect    #0:off 1:on
int32       freedrive_mode      #0:off 1:on

float64     speed               #overriding speed [0~1]

float64[6]  joint_angles        #[deg]
float64[6]  joint_references    #[deg]
float64[6]  joint_current       #[mA]
float64[6]  joint_temperature   #[celcius]
int32[6]    joint_information   #look mSTAT

float64[6]  tcp_reference
float64[6]  tcp_position

float64     tool_reference      #reference voltage [0, 12, 24V]

float64[6]  ft_sensor
