#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.42; // -0.37;
float offset_Enc3_rad = 0.23; // 0.27;

// Your global varialbes.  

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

long arrayindex = 0;

float xposition = 0;
float yposition = 0;
float zposition = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float DHtheta1 = 0;
float DHtheta2 = 0;
float DHtheta3 = 0;

float calctheta1 = 0;
float calctheta2 = 0;
float calctheta3 = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;
float kp1 = 50;
float kd1 = 2;
float kp1_2 = 6000;
float kd1_2 = 200;

float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;
float kp2 = 50;
float kd2 = 2;
float kp2_2 = 6000;
float kd2_2 = 200;

float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;
float kp3 = 50;
float kd3 = 2;
float kp3_2 = 6000;
float kd3_2 = 200;

float Ik_1 = 0;
float Ik_1_1 = 0;
float Ik_2 = 0;
float Ik_2_1 = 0;
float Ik_3 = 0;
float Ik_3_1 = 0;
float ek1 = 0;
float ek1_1 = 0;
float ek2 = 0;
float ek2_1 = 0;
float ek3 = 0;
float ek3_1 = 0;
float ki1 = 1000;
float ki2 = 1000;
float ki3 = 1000;

float J1 = .0167;
float J2 = .03;
float J3 = .0128;

float a1 = -1;
float b1 = 1.5;
float c1 = 0;
float d1 = 0;
float t_ms = 0;
float thetad = 0;
float thetadot = 0;
float thetaddot = 0;

float a2 = 1;
float b2 = -4.5;
float c2 = 6;
float d2 = -2;

float t = 0;
float tst = 500;
float ttot = 0;

float xcurr = 0;
float zcurr = 0;
float sideL = 5;

float v_min1 = 0.1;
float fVpos1 = 0.22;
float fVneg1 = 0.22;
float fCpos1 = 0.44;
float fCneg1 = -0.44;
float fmid1 = 4.62;
float fric1_on = 1;

float v_min2 = 0.05;
float fVpos2 = 0.275;
float fVneg2 = 0.25;
float fCpos2 = 0.5;
float fCneg2 = -0.45;
float fmid2 = 3.6;
float fric2_on = 1;

float v_min3 = 0.05;
float fVpos3 = 0.19;
float fVneg3 = 0.17;
float fCpos3 = 0.5;
float fCneg3 = -0.4;
float fmid3 = 3.6;
float fric3_on = 1;
float a_theta_1 = 0;
float a_theta_2 = 0;
float a_theta_3 = 0;

float s32 = 0;
float c32 = 0;
float s2 = 0;
float c3 = 0;

float p1 = .0466;
float p2 = .0388;
float p3 = .0284;
float p4 = .1405;
float p5 = .1298;
float g = 9.8;

float tau1_temp = 0;
float tau2_temp = 0;
float tau3_temp = 0;

float tau1_fric = 0;
float tau2_fric = 0;
float tau3_fric = 0;

float step = 0;

float Kpx = .5;
float Kpy = .5;
float Kpz = .5;
float Kdx = .025;
float Kdy = .025;
float Kdz = .025;

float xold = 0;
float yold = 0;
float zold = 0;

float xdot = 0;
float ydot = 0;
float zdot = 0;

float xdotold = 0;
float ydotold = 0;
float zdotold = 0;

float xdotold2 = 0;
float ydotold2 = 0;
float zdotold2 = 0;

float xd = 11.9923944;
float yd = -0.3663975;
float zd = 15;

float xddot = 0;
float yddot = 0;
float zddot = 0;

float Fx = 0;
float Fy = 0;
float Fz = 0;

float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0;
float thetax = 0;
float thetay = 0;
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;

float KPxN = .5;
float KDxN = .025;
float KPyN = .5;
float KDyN = .025;
float KPzN = .5;
float KDzN = .025;

float xa = 11.9923944;
float ya = -0.3663975;
float za = 15;

float xb = 11.9923944;
float yb = -0.3663975;
float zb = 6;

float z_hole = 4.97;

#define XYZSTIFF  1
#define ZSTIFF  2
#define XZSTIFF  3
#define XYSTIFF  4

int freemove = 5;
int ctrlmove = 2;

typedef struct points
{
    float x;
    float y;
    float z;

    int stiffmode;
    float angle;
    float vel;
} points;

points pts[] = { { 10, 0.00, 20, XYZSTIFF, 0, 25.0 },                      //home
        { 1.52, 13.96, 8, XYZSTIFF, 0, 25.0 },            //above hole
        { 1.52, 13.96, 4.95, ZSTIFF, 0, 10.0 },                 //in hole
        { 1.52, 13.96, 8, ZSTIFF, 0, 10.0 },              //above hole
        { 7.63, 4.15, 8, XYZSTIFF, 0, 25.0 },             //tf point
        { 15, 4.15, 8.1, XYZSTIFF, 0, 25.0 },             //start zigzag
        {16.1, 1.91, 8.25, XZSTIFF, 0.14189526339, 12.5 }, //start curve1
        { 15.55, 1.64, 8.25, XZSTIFF, 0.78539816339, 12.5 }, //mid curve1
        { 14.9, 1.36, 8.25, XZSTIFF, 0.39269908169, 12.5 },  //end curve1
        { 13, 1.76, 8.25, XZSTIFF, -0.261799, 12.5 },        //start curve2
        { 12.48, 1.63, 8.25, XZSTIFF, 0.261799, 12.5 },      //mid curve2
        { 12.39, 0.89, 8.25, XZSTIFF, 0.52359916339, 12.5 }, //end curve2
        { 14.45, -1.99, 8.25, XZSTIFF, 0.6435029, 12.5 },    //end zigzag
        { 14.45, -1.99, 14.12, XYZSTIFF, 0, 25.0 },        //avoid killing egg
        { 14.03, -5.97, 14.12, XYZSTIFF, 0, 25.0 },          //above egg
        { 14.03, -5.97, 13.25, XYZSTIFF, 0, 1.0 },          //push egg
        { 14.03, -5.97, 13.25, XYZSTIFF, 0, 1.0 },           //delay egg
        { 14.03, -5.97, 14.12, XYZSTIFF, 0, 5.0 },          //above egg
        { 10, 0, 20, XYZSTIFF, 0, 20.0 }                     //home
};

int i = 0;

void lab(float theta1motor, float theta2motor, float theta3motor, float *tau1,
         float *tau2, float *tau3, int error)
{

    *tau1 = 0;
    *tau2 = 0;
    *tau3 = 0;

    //Motor torque limitation(Max: 5 Min: -5)

    // save past states
    if ((mycount % 50) == 0)
    {

        theta1array[arrayindex] = theta1motor;
        theta2array[arrayindex] = theta2motor;

        if (arrayindex >= 100)
        {
            arrayindex = 0;
        }
        else
        {
            arrayindex++;
        }

    }
    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;
    DHtheta1 = theta1motor; //DH equation 1
    //printtheta1motor = DHtheta1;

    DHtheta2 = theta2motor - PI / 2; //DH equation 2
    //printtheta2motor = DHtheta2;

    DHtheta3 = -theta2motor + theta3motor + PI / 2; //DH equation 3
    //printtheta3motor = DHtheta3;

    xb = pts[i].x;
    yb = pts[i].y;
    zb = pts[i].z;

    if (i != 0 && i != 16)
    {
        xa = pts[i - 1].x;
        ya = pts[i - 1].y;
        za = pts[i - 1].z;
        ttot = 1000
                * sqrt((pts[i].x - pts[i - 1].x) * (pts[i].x - pts[i - 1].x)
                + (pts[i].y - pts[i - 1].y) * (pts[i].y - pts[i - 1].y)
                + (pts[i].z - pts[i - 1].z) * (pts[i].z - pts[i - 1].z))
                / pts[i].vel;
        xd = (xb - xa) * (t) / ttot + xa;
        yd = (yb - ya) * (t) / ttot + ya;
        zd = (zb - za) * (t) / ttot + za;
    }
    else if (i == 0)
    {
        ttot = 300;
        xd = pts[i].x;
        yd = pts[i].y;
        zd = pts[i].z;
    }
    else
    {
        ttot == 1000;
        xd = pts[i].x;
        yd = pts[i].y;
        zd = pts[i].z;
    }

    if (pts[i].stiffmode == 1)
    {
        KPxN = 0.5;
        KDxN = 0.025;
        KPyN = 0.5;
        KDyN = 0.025;
        KPzN = 0.5;
        KDzN = 0.025;
    }
    else if (pts[i].stiffmode == 2)
    {
        KPxN = 0.05;
        KDxN = 0.005;
        KPyN = 0.05;
        KDyN = 0.005;
        KPzN = 0.75;
        KDzN = 0.045;
    }
    else if (pts[i].stiffmode == 3)
    {
        KPxN = 0.25;
        KDxN = 0.01;
        KPyN = 0.25;
        KDyN = 0.01;
        KPzN = 0.5;
        KDzN = 0.025;
    }
    else if (pts[i].stiffmode == 4)
    {
        KPxN = 0.5;
        KDxN = 0.025;
        KPyN = 0.5;
        KDyN = 0.025;
        KPzN = 0.25;
        KDzN = 0.0125;
    }

    thetaz = pts[i].angle;

    if (t > ttot && i < 17)
    {
        t = 0;
        i++;
    }
    else if (t > ttot)
    {
        t = 0;
        i = 0;
    }
    else
    {
        t++;
    }

    // Rotation zxy and its Transpose
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
    RT11 = R11 = cosz * cosy - sinz * sinx * siny;
    RT21 = R12 = -sinz * cosx;
    RT31 = R13 = cosz * siny + sinz * sinx * cosy;
    RT12 = R21 = sinz * cosy + cosz * sinx * siny;
    RT22 = R22 = cosz * cosx;
    RT32 = R23 = sinz * siny - cosz * sinx * cosy;
    RT13 = R31 = -cosx * siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx * cosy;

    // Jacobian Transpose
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    JT_11 = -10 * sinq1 * (cosq3 + sinq2);
    JT_12 = 10 * cosq1 * (cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 10 * cosq1 * (cosq2 - sinq3);
    JT_22 = 10 * sinq1 * (cosq2 - sinq3);
    JT_23 = -10 * (cosq3 + sinq2);
    JT_31 = -10 * cosq1 * sinq3;
    JT_32 = -10 * sinq1 * sinq3;
    JT_33 = -10 * cosq3;

    //Assign x, y, z coordinates from Jacobian of system
    xposition = 10 * cos(theta1motor) * (cos(theta3motor) + sin(theta2motor));
    yposition = 10 * sin(theta1motor) * (cos(theta3motor) + sin(theta2motor));
    zposition = 10 * (1 + cos(theta2motor) - sin(theta3motor));

    xdot = (xposition - xold) / .001;
    ydot = (yposition - yold) / .001;
    zdot = (zposition - zold) / .001;

    xdot = (xdot + xdotold + xdotold2) / 3;
    ydot = (ydot + ydotold + ydotold2) / 3;
    zdot = (zdot + zdotold + zdotold2) / 3;

    Fx = Kpx * (xd - xposition) + Kdx * (xddot - xdot);
    Fy = Kpy * (yd - yposition) + Kdy * (yddot - ydot);
    Fz = Kpz * (zd - zposition) + Kdz * (zddot - zdot);

    xold = xposition;
    yold = yposition;
    zold = zposition;

    xdotold2 = xdotold;
    ydotold2 = ydotold;
    zdotold2 = zdotold;

    xdotold = xdot;
    ydotold = ydot;
    zdotold = zdot;

    tau1_temp = (JT_11 * R11 + JT_12 * R21 + JT_13 * R31)
            * (KDxN
                    * (RT11 * (xddot - xdot) + RT12 * (yddot - ydot)
                            + RT13 * (zddot - zdot))
                    + KPxN
                            * (RT11 * (xd - xposition) + RT12 * (yd - yposition)
                                    + RT13 * (zd - zposition)))
            + (JT_11 * R12 + JT_12 * R22 + JT_13 * R32)
                    * (KDyN
                            * (RT21 * (xddot - xdot) + RT22 * (yddot - ydot)
                                    + RT23 * (zddot - zdot))
                            + KPyN
                                    * (RT21 * (xd - xposition)
                                            + RT22 * (yd - yposition)
                                            + RT23 * (zd - zposition)))
            + (JT_11 * R13 + JT_12 * R23 + JT_13 * R33)
                    * (KDzN
                            * (RT31 * (xddot - xdot) + RT32 * (yddot - ydot)
                                    + RT33 * (zddot - zdot))
                            + KPzN
                                    * (RT31 * (xd - xposition)
                                            + RT32 * (yd - yposition)
                                            + RT33 * (zd - zposition)));
    tau2_temp = (JT_21 * R11 + JT_22 * R21 + JT_23 * R31)
            * (KDxN
                    * (RT11 * (xddot - xdot) + RT12 * (yddot - ydot)
                            + RT13 * (zddot - zdot))
                    + KPxN
                            * (RT11 * (xd - xposition) + RT12 * (yd - yposition)
                                    + RT13 * (zd - zposition)))
            + (JT_21 * R12 + JT_22 * R22 + JT_23 * R32)
                    * (KDyN
                            * (RT21 * (xddot - xdot) + RT22 * (yddot - ydot)
                                    + RT23 * (zddot - zdot))
                            + KPyN
                                    * (RT21 * (xd - xposition)
                                            + RT22 * (yd - yposition)
                                            + RT23 * (zd - zposition)))
            + (JT_21 * R13 + JT_22 * R23 + JT_23 * R33)
                    * (KDzN
                            * (RT31 * (xddot - xdot) + RT32 * (yddot - ydot)
                                    + RT33 * (zddot - zdot))
                            + KPzN
                                    * (RT31 * (xd - xposition)
                                            + RT32 * (yd - yposition)
                                            + RT33 * (zd - zposition)));
    tau3_temp = (JT_31 * R11 + JT_32 * R21 + JT_33 * R31)
            * (KDxN
                    * (RT11 * (xddot - xdot) + RT12 * (yddot - ydot)
                            + RT13 * (zddot - zdot))
                    + KPxN
                            * (RT11 * (xd - xposition) + RT12 * (yd - yposition)
                                    + RT13 * (zd - zposition)))
            + (JT_31 * R12 + JT_32 * R22 + JT_33 * R32)
                    * (KDyN
                            * (RT21 * (xddot - xdot) + RT22 * (yddot - ydot)
                                    + RT23 * (zddot - zdot))
                            + KPyN
                                    * (RT21 * (xd - xposition)
                                            + RT22 * (yd - yposition)
                                            + RT23 * (zd - zposition)))
            + (JT_31 * R13 + JT_32 * R23 + JT_33 * R33)
                    * (KDzN
                            * (RT31 * (xddot - xdot) + RT32 * (yddot - ydot)
                                    + RT33 * (zddot - zdot))
                            + KPzN
                                    * (RT31 * (xd - xposition)
                                            + RT32 * (yd - yposition)
                                            + RT33 * (zd - zposition)));

    /*
     tau1_temp = -10 * sin(theta1motor) * (cos(theta3motor) + sin(theta2motor)) * Fx + 10 * cos(theta1motor) * (cos(theta3motor) + sin(theta2motor)) * Fy;
     tau2_temp = 10 * cos(theta1motor) * (cos(theta2motor) - sin(theta3motor)) * Fx + 10 * sin(theta1motor) * (cos(theta2motor) - sin(theta3motor)) * Fy + -10 * (cos(theta3motor) + sin(theta2motor)) * Fz;
     tau3_temp = -10 * cos(theta1motor) * sin(theta3motor) * Fx + -10 * sin(theta1motor) * sin(theta3motor) * Fy + -10 *cos(theta3motor) * Fz;
     */

    //Inverse kinematic equations
    calctheta1 = atan2(yposition, xposition);
    calctheta2 = -(acos(
            (sqrt(xposition * xposition + yposition * yposition
                    + (zposition - 10) * (zposition - 10)) / 20)) - PI / 2
            + atan2(zposition - 10,
                    sqrt(xposition * xposition + yposition * yposition)));
    calctheta3 = acos(
            ((xposition * xposition + yposition * yposition
                    + (zposition - 10) * (zposition - 10) - 200) / 200))
            + calctheta2 - PI / 2;

    Omega1 = (theta1motor - Theta1_old) / 0.001;
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2) / 3.0;
    ek1 = (thetad - theta1motor);
    ek1_1 = (thetad - Theta1_old);
    Theta1_old = theta1motor;
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Omega2 = (theta2motor - Theta2_old) / 0.001;
    Omega2 = (Omega2 + Omega2_old1 + Omega2_old2) / 3.0;
    ek2 = (thetad - theta2motor);
    ek2_1 = (thetad - Theta2_old);
    Theta2_old = theta2motor;
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    Omega3 = (theta3motor - Theta3_old) / 0.001;
    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2) / 3.0;
    ek3 = (thetad - theta3motor);
    ek3_1 = (thetad - Theta3_old);
    Theta3_old = theta3motor;
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;

    if (Omega1 > v_min1)
    {
        tau1_fric = *tau1 + .6 * fric1_on * (fVpos1 * Omega1 + fCpos1);
    }
    else if (Omega1 < -v_min1)
    {
        tau1_fric = *tau1 + .6 * fric1_on * (fVneg1 * Omega1 + fCneg1);
    }
    else
    {
        tau1_fric = *tau1 + .6 * fric1_on * (fmid1 * Omega1);
    }

    if (Omega2 > v_min2)
    {
        tau2_fric = *tau2 + .6 * fric2_on * (fVpos2 * Omega2 + fCpos2);
    }
    else if (Omega2 < -v_min2)
    {
        tau2_fric = *tau2 + .6 * fric2_on * (fVneg2 * Omega2 + fCneg2);
    }
    else
    {
        tau2_fric = *tau2 + .6 * fric2_on * (fmid2 * Omega2);
    }

    if (Omega3 > v_min3)
    {
        tau3_fric = *tau3 + .6 * fric3_on * (fVpos3 * Omega3 + fCpos3);
    }
    else if (Omega3 < -v_min3)
    {
        tau3_fric = *tau3 + .6 * fric3_on * (fVneg3 * Omega3 + fCneg3);
    }
    else
    {
        tau3_fric = *tau3 + .6 * fric3_on * (fmid3 * Omega3);
    }

    *tau1 = tau1_temp + tau1_fric;
    *tau2 = tau2_temp + tau2_fric;
    *tau3 = tau3_temp + tau3_fric;

    if (fabs(*tau1) > 5)
    {
        *tau1 = (*tau1 > 0) ? 5 : -5;
    }
    if (fabs(*tau2) > 5)
    {
        *tau2 = (*tau2 > 0) ? 5 : -5;
    }
    if (fabs(*tau3) > 5)
    {
        *tau3 = (*tau3 > 0) ? 5 : -5;
    }

    if ((mycount % 500) == 0)
    {
        if (whattoprint > 0.5)
        {
            serial_printf(&SerialA, "I love robotics\n\r");
        }
        else
        {

            SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        //GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }
    if ((mycount % 1000) == 380)
    {
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
    }
    if ((mycount % 1000) == 460)
    {
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
    }
    if ((mycount % 1000) == 540)
    {
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
    }
    if ((mycount % 1000) == 620)
    {
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
    }

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = thetad;

    mycount++;

}

void printing(void)
{
    serial_printf(
            &SerialA,
            "%.2f, %.2f, %.2f \n\r%.2f, %.2f, %.2f \n\r%.2f, %.2f, %.2f \n\r%.2f, %.2f, %.2f  \n\r\n\r",
            printtheta1motor * 180 / PI, printtheta2motor * 180 / PI,
            printtheta3motor * 180 / PI, xposition, yposition, zposition,
            DHtheta1 * 180 / PI, DHtheta2 * 180 / PI, DHtheta3 * 180 / PI,
            calctheta1 * 180 / PI, calctheta2 * 180 / PI,
            calctheta3 * 180 / PI);
}
