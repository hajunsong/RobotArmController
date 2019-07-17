#pragma once

#include <iostream>
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>

#include "fileio.h"
#include "numerical.h"

using namespace std;

class RobotArm
{
public:
    RobotArm(uint numbody, uint DOF);
    ~RobotArm();
    void run_kinematics();
    void run_inverse_kinematics();
    void run_inverse_kinematics(double* cur_joint, double* des_pose, double* res_joint, double* res_pose);

private:
    void tilde(double *a, double *b) {
        *(b++) = 0;	*(b++) = -a[2];	*(b++) = a[1];
        *(b++) = a[2];	*(b++) = 0;	*(b++) = -a[0];
        *(b++) = -a[1];	*(b++) = a[0];	*(b++) = 0;
    }

    class Body
    {
    public:
        Body();
        Body(double psi, double theta, double phi, double sijp_x, double sijp_y, double sijp_z);
        ~Body();
        // base body information
        double A0[9], C01[9], s01p[3], J0p[9], r0[3], m0, A0_C01[9], C01_A01pp[9];
        // body initial data
        double qi, qi_dot, mi, qi_init;
        double ri[3], ri_dot[3], wi[3], rhoip[3], sijp[3], Jip[9], Cii[9], Cij[9], Ai_Cij[9], Cij_Aijpp[9];
        // Orientation
        double Aijpp[9], Ai[9], Hi[3], u_vec[3];
        // Position
        double sij[3], rhoi[3], ric[3], rit[9];
        // End point
        double Ce[9], sep[3], se[3], re[3], Ae[9], roll, pitch, yaw;
        // Jacobian
        double Jvi[3], Jwi[3], re_qi[3], Ae_qi[9], r6_qi[3], A6_qi[9], Aijpp_qi[9];
        double Ae_qi_31, Ae_qi_32, Ae_qi_33, Ae_qi_21, Ae_qi_11, roll_qi, pitch_qi, yaw_qi;
    };

    uint num_body, dof;
    double *PH, *PH_pos, *PH_ori, *delta_q, *J, *JD;

    // system variable
    double start_time, end_time, h, t_current;
    double g;

    // file
    char file_name[256];
    FILE *fp;

    Body *body;
    Numerical *numeric;

    double lamda;

    void kinematics();
    void inverse_kinematics(double pos_d[3], double ori_d[3]=NULL);
    void jacobian();
    void save_data();
};

// Euler Angle to Transformation Matrix(body 3-1-3)
void Euler2Trans(double psi, double theta, double phi, double* Mat);

