//*************************** User Parameter *************************//

//********************************* Commands *************************************//
//object_state
#define UNKNOWN 0
const char CMD_Initialize = 'I';
const char CMD_ChangeMode = 'M';
const char CMD_Suction = 'S';
const char CMD_MoveJoint = 'J';
const char CMD_MoveTCP = 'T';
const char CMD_MoveBlend = 'B';
const char CMD_ChangeSpeed = 'V';
const char CMD_MoveGripper = 'E';
const char CMD_StopGripper = 'F';
const char CMD_MotionPause = 'P';
const char CMD_MotionHalt = 'H';
const char CMD_MotionResume = 'Q';
const char CMD_CollisionResume = 'C';
const char CMD_WheelMove = 'W';

//motion_cmd
enum{
    HOME_POSE = 0, OBJECT_SEARCH, GRASP_OBJECT, PUT_OBJECT, PULL_SHELF,
    PUSH_SHELF, DETECT_SHELF
};

//shelf_state
enum{
    SEARCH = 0, PULL
};

//object_id
enum{
    KIMBAP1 = 0, KIMBAP2, SANDWICH, HAMBUG, COFFEE, LUNCHBOX
};

//sub_cmd(put pose)
enum{
    DISPOSE = 0, LINE1, LINE2, LINE3, LUNCHBOX_PUT
};

//return state
enum{
    IDLE, ACCEPT, DONE, STATE_ERROR, INPUT_ERROR, ERROR_STOP, CMD_ERROR
};

//tool_id
enum{
    TOOLMODE_GRIPPER = 0, TOOLMODE_SUCTION, TOOLMODE_GALGORI
};


//self_marker_num
enum{
    FRONT1 = 0, FRONT2, FRONT3, DUMMY1, DUMMY2, BACK1, BACK2, BACK3, BASKET_Z, TEMP_Z
};

enum{
    SHELF1 = 0, SHELF2, SHELF3, BASKET, TEMP
};






const float w_line = 0.018;
const int max_gripper_cnt = 600;

const float spd_approach_suction[2] =   {0.1, 0.02};
const float spd_put[2] =                {0.5, 0.3};
const float spd_half[2] =               {0.5, 0.3};
const float spd_approach[2] =           {0.3, 0.05};
const float spd_fast[2] =               {0.8, 0.5};
const float spd_lift[2] =               {0.5, 0.2};

//grasp [m]
const float z_limit_object = 0.01;
const float z_grasp_approach = 0.1;
const float z_put = 0.015;
const float y_put_object = 0.03;
const float y_pull_object = 0.04;
const float z_grasp_offset[2] = {0.0, -0.0};
const float z_put_offset[2] = {0., 0.03};

//limit [m]
const float limit_marker_offset_f = 0.03;
const float limit_marker_offset_b = 0.02;
const float limit_marker_offset_z = 0.01;
const float limit_marker_z = 0.005;
const float shlef_distance_z = 0.2;
const float shelf_y_offset = 0.355;






//---------------------------- Object info ----------------------------//
typedef struct object_info{
    float line1;
    float line2;
    float depth;
    float first_depth;
    float height;
    float width;
    float z_lift;
    float default_toolmode;
}object_info;

typedef struct shelf_info{
    float width;
    float depth;
    float height;
    float limit;
}shelf_info;

typedef struct marker_info{
    float half_side;
    float x_offset;
    float y_offset;
    float z_offset;
}marker_info;

typedef struct robot_info{
    float robot_base_z;
    float basket_z;
    float shelf_to_robot;
    float distance_move;
    float robot_base_y;
}robot_info;

typedef struct pose{
    float x;
    float y;
    float z;
}pose;

const object_info Info_Object[] = {
    //line1,            line2,              depth,      f_dep,      height,     width,      z_lift,     toolmode
    { 0.057+w_line/2,    0.157+w_line/2,    0.035,      0.020,      0.08,        0.08,      0.1,        TOOLMODE_GRIPPER},    //kimbap1
    {-0.056-w_line/2,   -0.156-w_line/2,    0.035,      0.020,      0.08,       -0.08,      0.1,        TOOLMODE_GRIPPER},   //kimbap2
    { 0.120+w_line/2,    0.228+w_line/2,    0.090,      0.075,      0.125,       0.09,      0.2,        TOOLMODE_GRIPPER},    //sandwich
    {-0.158+w_line/2,   -0.336+w_line/2,    0.065,      0.065/2,    0.115,      -0.16,      0.2,        TOOLMODE_GRIPPER},  //hambug
    { 0.199+w_line/2,    0.288+w_line/2,    0.09,       0.08/2,     0.12,        0.09,      0.2,        TOOLMODE_SUCTION},    //coffee
    {-0.111-w_line/2,   -0.379-w_line/2,    0.0,        0.17/2,     0.038,      -0.25,      0.1,        TOOLMODE_SUCTION}   //lunchbox
};

const shelf_info I_S[] = {//Info shelf
    //width,    depth,      height,     //limit
    { 0.9,      0.4,        0.5,        -0.62},         //1floor
    { 0.9,      0.4,        0.915,        -0.65},         //2floor
    { 0.9,      0.4,        1.315,         0.7},          //3floor
    { 0.9,      0.4,        1.3,         0.7},          //basket
    { 0.9,      0.4,        1.3,         0.7}           //temp(lunchbox)
};

const robot_info I_R = {1.1,   1.11,  0.9,    0.3,    0.15}; //Info robot

const marker_info I_M[] = { //Info marker
    //side/2,   xoffset,    yoffset,    zoffset
    { 0.012,    0.,         0.,         0.008}, //front
    { 0.012,    0.,         0.369,      0.},    //back
};
                                                                                        //1.1 - 0.9 - 0.008 = 0.192
const pose shelf_marker[] = {
    {0.,    -(I_R.shelf_to_robot + I_R.robot_base_y + I_M[0].y_offset),      -(I_R.robot_base_z - I_S[0].height - I_M[0].z_offset)},    //1 floor front
    {0.,    -(I_R.shelf_to_robot + I_R.robot_base_y + I_M[0].y_offset),      -(I_R.robot_base_z - I_S[1].height - I_M[0].z_offset)},    //2 floor front
    {0.,    -(I_R.shelf_to_robot + I_R.robot_base_y + I_M[0].y_offset),      -(I_R.robot_base_z - I_S[2].height - I_M[0].z_offset)},    //3 floor front
    {0.,    -(I_R.shelf_to_robot + I_R.robot_base_y + I_M[0].y_offset),      -(I_R.robot_base_z - I_S[2].height - I_M[0].z_offset)},    //3 floor front
    {0.,    -(I_R.shelf_to_robot + I_R.robot_base_y + I_M[0].y_offset),      -(I_R.robot_base_z - I_S[2].height - I_M[0].z_offset)},    //3 floor front
            //-0.9 - 0.15 - 0.369 + 0.3 + 0.4 = -0.719                                                          -1.1 + 0.5 + 0.008 = -0.592
    {0.,    -(I_R.shelf_to_robot + I_R.robot_base_y + I_M[1].y_offset - I_R.distance_move - I_S[0].depth),      -(I_R.robot_base_z - I_S[0].height - I_M[1].z_offset)},   //1 floor back
    {0.,    -(I_R.shelf_to_robot + I_R.robot_base_y + I_M[1].y_offset - I_R.distance_move - I_S[1].depth),      -(I_R.robot_base_z - I_S[1].height - I_M[1].z_offset)},   //2 floor back
    {0.,    -(I_R.shelf_to_robot + I_R.robot_base_y + I_M[1].y_offset - I_R.distance_move - I_S[2].depth),      -(I_R.robot_base_z - I_S[2].height - I_M[1].z_offset)},   //3 floor back
    {0.,    0.,      I_R.basket_z - I_R.robot_base_z},   //basket
    {0.,    0.,      0.}    //lunchbox
};