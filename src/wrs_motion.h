//******************************* Motion ****************************************//

typedef struct Motion{
    char type;
    float data;
    float coordinate[6];
    float spd;
    float acc;
} Motion;

typedef struct Pose{
    float cdn[6];
} Pose;

float Pinit[6] = {-77.185, -77.396, 134.561, -55.864, 67.807, 0.243};

//home_pose
enum{
    JDEFAULTHOME = 0, TSHELF1HOME, JBASKETHOME
};
Pose home_pose[] = {
    //default
    {Pinit[0], Pinit[1], Pinit[2], Pinit[3], Pinit[4], Pinit[5]},

    //shelf 1(TCP)
    {150, -400, 300, 90, 0, 0},

    //basket
    {72.29, -26.4, 113.06, -6.19, 93.04, 17.45}
};

//via_point
enum{
    JDISPOSE_VIA1 = 0, JDISPOSE, TDISPOSE,
    TGRASP_SHELF3, JSHELF1TOIDLE,
    TLUNCHBOX_VIA1, JLUNCHBOX_VIA2, JLUNCHBOX_PUT, TLUNCHBOX_PUT_LIFT,
    JBASKETTO3, JBASKETTO12, JTEMPTOSHELF
};
Pose via_point[] = {
    //JDISPOSE_VIA1
    {-90., -17.91, 127.4, -19.48, 90.01, 2.37},
    //JDISPOSE
    {-182.38, -38.33, 137.1, -8.77, 90.01, 2.38},
    //TDISPOSE
    {-230, 120, 250, 90, 0, -90},

    //TGRASP_SHELF3
    {0., -250, 680, 90, 0, 0},
    //JSHELF1TOIDLE
    {-71.62, -17.26, 134.95, -27.68, 90.01, -18.38},

    //TLUNCHBOX_VIA1 - lunchbox(TCP) via1
    {150, -400, 300, 90, 0, 0},
    //JLUNCHBOX_VIA2 - lunchbox put via2
    {-24.12, -0.33, 105.33, -15.01, 90.01, -20.88},
    //JLUNCHBOX_PUT - lunchbox put
    {53.2, 14.09, 157.79, -81.88, 90.00, -143.21},
    //TLUNCHBOX_PUT_LIFT
    {250,150,250,90,0,0},

    //JBASKETTO3 - basket to shelf after grasp (3 floor)
    {-75.81, -0.13, 56.36, 33.76, 90.01, -14.18},
    //JBASKETTO12 - basket to shelf  after grasp (1,2 floor)
    {-75.81, -9.09, 98.56, 0.52, 90.01, -14.19},
    //JTEMPTOSHELF - temp(lunchbox) to shelf  after grasp
    {-67.44, -12.89, 103.72, -7.82, 88.17, -20.38}
};

enum{
    JSHELF2_BACK = 0, JSHELF3_BACK, TEMP_LUNCHBOX
};
Pose search_pose[] = {
    //JSHELF2_BACK
    {-77.34, 17.82, 122.3, -59.88, 87.82, -12.48},
    //JSHELF3_BACK
    {-77.34, -0.37, 78.9, 1.71, 87.83, -12.47},
    //TEMP_LUNCHBOX
    {45.77, -19.91, 136.56, -26.66, 90.01, -225.78}
};


Motion Shelf_Search1[] = {
    {'J', 0, Pinit[0],Pinit[1],Pinit[2],Pinit[3],Pinit[4],Pinit[5], spd_80[0], spd_80[1]},
    {'J', 0, -82.81, 44.19, 109.07, -62.58, 88.92, -8.91, spd_80[0], spd_80[1]},
    {'J', 0, -82.78, 112.23, 55.02, -126.97, 84.46, -4.66, spd_80[0], spd_80[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Search2[] = {
    {'J', 0, Pinit[0],Pinit[1],Pinit[2],Pinit[3],Pinit[4],Pinit[5], spd_80[0], spd_80[1]},
    {'J', 0, -77.39, 71.82, 95.18, 13.03, -77.37, -180, spd_80[0], spd_80[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Search3[] = {
    {'J', 0, Pinit[0],Pinit[1],Pinit[2],Pinit[3],Pinit[4],Pinit[5], spd_80[0], spd_80[1]},
    {'J', 0, -74.2, 14.46, 119.32, 46.21, -74.18, -179.99, spd_80[0], spd_80[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Push1[] = {
    {'J', 0, Pinit[0],Pinit[1],Pinit[2],Pinit[3],Pinit[4],Pinit[5], spd_80[0], spd_80[1]},
    {'J', 0, -83.97, 66.84, 122.51, -97.86, 87.76, -8.39, spd_80[0], spd_80[1]},
    {'T', 0, 0, -410, -300, 90, 0, 90, spd_80[0], spd_80[1]},
    {'T', 0, 0, -410, -350, 90, 0, 90, spd_10[0], spd_10[1]},
    {'T', 0, 0, -700, -350, 90, 0, 90, spd_approach[0], spd_approach[1]},
    {'T', 0, 0, -600, -280, 90, 0, 90, spd_approach[0], spd_approach[1]},
    {'T', 0, 0, -600, -360, 90, 0, 90, spd_80[0], spd_80[1]},
    {'T', 0, 0, -760, -360, 90, 0, 90, spd_approach[0],  spd_approach[1]},
    {'T', 0, 0, -400,    0, 90, 0, 90, spd_80[0], spd_80[1]},
    {'J', 0, Pinit[0],Pinit[1],Pinit[2],Pinit[3],Pinit[4],Pinit[5], spd_80[0], spd_80[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Push2[] = {
    {'J', 0, Pinit[0],Pinit[1],Pinit[2],Pinit[3],Pinit[4],Pinit[5], spd_80[0], spd_80[1]},
    {'J', 0, -68.81, 18.57, 139.11, 22.4, -68.8, -180, spd_80[0], spd_80[1]},
    {'T', 0, 0, -390, -170, 0, 0, 0, spd_60[0], spd_60[1]},
    {'T', 0, 0, -800, -170, 0, 0, 0, spd_10[0], spd_10[1]},
    {'T', 0, 0, -600, -170, 0, 0, 0, spd_60[0], spd_60[1]},
    {'J', 0, Pinit[0],Pinit[1],Pinit[2],Pinit[3],Pinit[4],Pinit[5], spd_80[0], spd_80[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Push3[] = {
    {'J', 0, Pinit[0],Pinit[1],Pinit[2],Pinit[3],Pinit[4],Pinit[5], spd_80[0], spd_80[1]},
    {'J',0,-83.04, 36.69, 144.89, -181.58, 83.0, 0., spd_80[0], spd_80[1]},
    {'T', 0, -80, -800, 230.0, 0.0, 0, 0.0, spd_10[0], spd_10[1]},
    {'T', 0, -80, -400, 230.0, 0.0, 0, 0.0, spd_half[0], spd_half[1]},
    {'J', 0, Pinit[0],Pinit[1],Pinit[2],Pinit[3],Pinit[4],Pinit[5], spd_80[0], spd_80[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};




