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


float jshelfinit[6] = {-77.185, -77.396, 134.561, -55.864, 67.807, 0.243};
float jdispose1[6] = {-0.88, -24.57, 94.18, 20.39, 90.01, 0.89};
float jdispose2[6] = {126.92, -2.77, 85.63, 7.14, 90.00, -36.92};
float jdispose_lunchbox[6] = {-60.04, 1.07, 119.81, -30.88, 90.01, -6.56};
float tdispose_lunchbox[6] = {150, -400, 300, 90, 0, 0};
float jlunchbox1[6] = {-24.12, -0.33, 105.33, -15.01, 90.01, -20.88};
float jlunchbox2[6] = {26.2, -40.95, 130.58, 0.37, 90.01, -26.2};
//float jdispose2[6] = {116.38, -7.09, 89.54, 7.54, 90.01, -26.38};

Motion Shelf_Search1[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], spd_fast[0], spd_fast[1]},
    {'J', 0, -82.81, 44.19, 109.07, -62.58, 88.92, -8.91, spd_fast[0], spd_fast[1]},
    {'J', 0, -70.1, 120.76, 41.72, -120.7, 74.88, -13.11, spd_fast[0], spd_fast[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Search2[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], spd_fast[0], spd_fast[1]},
    {'J', 0, -77.39, 71.82, 95.18, 13.03, -77.37, -180, spd_fast[0], spd_fast[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Search3[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], spd_fast[0], spd_fast[1]},
    {'J', 0, -74.2, 14.46, 119.32, 46.21, -74.18, -179.99, spd_fast[0], spd_fast[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Pull1[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], spd_fast[0], spd_fast[1]},
    {'J', 0, -90.66, 13.99, 128.08, -52.07, 90.01, 90.66, spd_fast[0], spd_fast[1]},
    {'T', 0, 0, -550, -330, 90, 0, 90, spd_fast[0], spd_fast[1]},
    {'T', 0, 0, -630, -330, 90, 0, 90, spd_put[0], spd_put[1]},
    {'T', 0, 0, -630, -350, 90, 0, 90, spd_put[0], spd_put[1]},
    {'T', 0, 0, -230, -350, 90, 0, 90, spd_approach[0], spd_approach[1]},
    {'T', 0, 0, -430, -320, 90, 0, 90, spd_approach[0], spd_approach[1]},
    {'T', 0, 0, -430,  200, 90, 0, 90, spd_put[0], spd_put[1]},
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], spd_fast[0], spd_fast[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Push1[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], spd_fast[0], spd_fast[1]},
    {'J', 0, -90.66, 13.99, 128.08, -52.07, 90.01, 90.66, spd_fast[0], spd_fast[1]},
    {'T', 0, 0, -400, -130, 90, 0, 90, spd_fast[0], spd_fast[1]},
    {'T', 0, 0, -245, -330, 90, 0, 90, spd_approach[0], spd_approach[1]},
    {'T', 0, 0, -245, -360, 90, 0, 90, spd_approach[0], spd_approach[1]},
    {'T', 0, 0, -655, -360, 90, 0, 90, spd_approach[0], spd_approach[1]},
    {'T', 0, 0, -630, -330, 90, 0, 90, spd_approach[0], spd_approach[1]},
    {'T', 0, 0, -410,    0, 90, 0, 90, spd_put[0], spd_put[1]},
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], spd_fast[0], spd_fast[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Push2[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], spd_fast[0], spd_fast[1]},
    {'J', 0, -68.81, 18.57, 139.11, 22.4, -68.8, -180, spd_fast[0], spd_fast[1]},
    {'T', 0, 0, -390, -170, 0, 0, 0, spd_fast[0], spd_fast[1]},
    {'T', 0, 0, -800, -170, 0, 0, 0, spd_approach_suction[0], spd_approach_suction[1]},
    {'T', 0, 0, -600, -170, 0, 0, 0, spd_half[0], spd_half[1]},
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], spd_fast[0], spd_fast[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};

Motion Shelf_Push3[] = {
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], spd_fast[0], spd_fast[1]},
    {'J',0,-83.04, 36.69, 144.89, -181.58, 83.0, 0., spd_fast[0], spd_fast[1]},
    {'T', 0, -80, -800, 230.0, 0.0, 0, 0.0, spd_approach_suction[0], spd_approach_suction[1]},
    {'T', 0, -80, -400, 230.0, 0.0, 0, 0.0, spd_half[0], spd_half[1]},
    {'J', 0, jshelfinit[0],jshelfinit[1],jshelfinit[2],jshelfinit[3],jshelfinit[4],jshelfinit[5], spd_fast[0], spd_fast[1]},
    {'E',0,0,0,0,0,0,0,0,0}
};




