#include "robotarm.h"

RobotArm::Body::Body(){
    u_vec[0] = 0;
    u_vec[1] = 0;
    u_vec[2] = 1;
}

RobotArm::Body::~Body(){}

RobotArm::RobotArm(uint numbody, uint DOF) {
    num_body = numbody;
    dof = DOF;

    PH = new double[dof];
    PH_pos = new double[3 * num_body];
    PH_ori = new double[3 * num_body];
    delta_q = new double[dof];
    J = new double[num_body * dof];
    JD = new double[dof * num_body];

    body = new Body[num_body+1];

    lamda = 0.0001f;

    // read data
    start_time = 0;
    end_time = 2;
    h = 0.001f;
    g = -9.80665f;

    double *Ai_ptr, *ri_ptr, *Cij_ptr, *sijp_ptr;
    // body 0 variable
    Ai_ptr = body[0].Ai;
    *(Ai_ptr++) = 1;	*(Ai_ptr++) = 0;	*(Ai_ptr++) = 0;
    *(Ai_ptr++) = 0;	*(Ai_ptr++) = 1;	*(Ai_ptr++) = 0;
    *(Ai_ptr++) = 0;	*(Ai_ptr++) = 0;	*(Ai_ptr) = 1;
    ri_ptr = body[0].ri;
    *(ri_ptr++) = 0;	*(ri_ptr++) = 0;	*(ri_ptr++) = 0;
    Cij_ptr = body[0].Cij;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = -1;	*(Cij_ptr++) = 0;
    *(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;	*(Cij_ptr) = 1;
    sijp_ptr = body[0].sijp;
    *(sijp_ptr++) = 0;	*(sijp_ptr++) = 0;	*(sijp_ptr++) = 0;

    // body 1 variable
    Cij_ptr = body[1].Cij;
    *(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 1;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = -1;	*(Cij_ptr) = 0;
    sijp_ptr = body[1].sijp;
    *(sijp_ptr++) = 0;	*(sijp_ptr++) = 0.0245f;	*(sijp_ptr++) = 0.042f;

    // body 2 variable
    Cij_ptr = body[2].Cij;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;
    *(Cij_ptr++) = -1;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;	*(Cij_ptr) = 1;
    sijp_ptr = body[2].sijp;
    *(sijp_ptr++) = 0;	*(sijp_ptr++) = -0.15175f;	*(sijp_ptr++) = 0;

    // body 3 variable
    Cij_ptr = body[3].Cij;
    *(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;	*(Cij_ptr) = 1;
    sijp_ptr = body[3].sijp;
    *(sijp_ptr++) = 0.15f;	*(sijp_ptr++) = 0;	*(sijp_ptr++) = 0;

    // body 4 variable
    Cij_ptr = body[4].Cij;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 1;
    *(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = 1;	*(Cij_ptr) = 0;
    sijp_ptr = body[4].sijp;
    *(sijp_ptr++) = 0.0245f;	*(sijp_ptr++) = 0.08675f;	*(sijp_ptr++) = -0.0245f;

    // body 5 variable
    Cij_ptr = body[5].Cij;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 1;
    *(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = 1;	*(Cij_ptr) = 0;
    sijp_ptr = body[5].sijp;
    *(sijp_ptr++) = 0.0825f;	*(sijp_ptr++) = 0;	*(sijp_ptr++) = -0.04475f;

    // body 6 variable
    Cij_ptr = body[6].Cij;
    *(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;
    *(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;	*(Cij_ptr) = 1;
    sijp_ptr = body[6].sijp;
    *(sijp_ptr++) = 0.085f;	*(sijp_ptr++) = 0;	*(sijp_ptr++) = 0.0125f;

    numeric = new Numerical();
}

RobotArm::~RobotArm() {
    delete[] PH;
    delete[] PH_pos;
    delete[] PH_ori;
    delete[] delta_q;
    delete[] J;
    delete[] JD;

    delete[] body;
    delete numeric;
}

void RobotArm::run_kinematics()
{
    sprintf(file_name, "../data/hj_kinematics_result.txt");
    fp = fopen(file_name, "w+");

//    uint data_size = 2001;
//    double *input = new double[data_size * col_size];
    vector<vector<double> >input;
    load_data("../data/kinematics_input_q.txt", &input);

    for(uint indx = 0; indx < input.size(); indx++) {
        for (uint i = 1; i <= 6; i++) {
            body[i].qi = input[indx][i];
        }

        kinematics();

        save_data();

//        printf("Time : %.3f[s]\n", static_cast<double>(t_current));
        cout << t_current << endl;

        t_current += h;
    }

    input.clear();
    fclose(fp);
}

void RobotArm::run_inverse_kinematics() {
    sprintf(file_name, "../data/hj_inverse_kinematics_result.txt");
    fp = fopen(file_name, "w+");

//    uint data_size = 4001;
//    uint col_size = 7;

//    uint sim_flag = 1;

//    double *q_data = new double[data_size * col_size];
//    char q_file_name[256];
    //sprintf(q_file_name, "../data/inverse_kinematics_output_q%d.txt", sim_flag);
//    sprintf(q_file_name, "../data/inverse_kinematics_output_q_pick.txt");
    vector<vector<double> > q_data;
    load_data("../data/inverse_kinematics_output_q_pick.txt", &q_data);
    for (uint i = 1; i <= 6; i++) {
        body[i].qi = q_data[0][i];
    }
//    delete[] q_data;
    q_data.clear();

//    double *input = new double[data_size * col_size];
//    char end_file_name[256];
    //sprintf_s(end_file_name, "../data/inverse_kinematics_input_end%d.txt", sim_flag);
//    sprintf(end_file_name, "../data/inverse_kinematics_input_end_pick.txt");
    vector<vector<double> > input;
    load_data("../data/inverse_kinematics_input_end_pick.txt", &input);

    double pos_d[3], ori_d[3];
    for (uint indx = 0; indx < input.size(); indx++) {
        pos_d[0] = input[indx][1];
        pos_d[1] = input[indx][2];
        pos_d[2] = input[indx][3];
        ori_d[0] = input[indx][4];
        ori_d[1] = input[indx][5];
        ori_d[2] = input[indx][6];

        kinematics();

        inverse_kinematics(pos_d, ori_d);

        save_data();

//        printf("Time : %.3f[s]\n", static_cast<double>(t_current));
        cout << t_current << endl;

        t_current += h;
    }

    input.clear();
    fclose(fp);
}

void RobotArm::run_inverse_kinematics(double* input_q, double* des_pose, double* cur_joint, double* cur_pose){
    for (uint i = 1; i <= 6; i++) {
        body[i].qi = input_q[i - 1];
    }

    double pos_d[3], ori_d[3];
    pos_d[0] = des_pose[0];
    pos_d[1] = des_pose[1];
    pos_d[2] = des_pose[2];
    ori_d[0] = des_pose[3];
    ori_d[1] = des_pose[4];
    ori_d[2] = des_pose[5];

    kinematics();

    inverse_kinematics(pos_d, ori_d);

    for(uint i = 1; i <= 6; i++){
        cur_joint[i - 1] = body[i].qi;
    }
    cur_pose[0] = body[6].re[0];
    cur_pose[1] = body[6].re[1];
    cur_pose[2] = body[6].re[2];
    cur_pose[3] = body[6].roll;
    cur_pose[4] = body[6].pitch;
    cur_pose[5] = body[6].yaw;
}

void RobotArm::kinematics()
{
    for (uint indx = 1; indx <= num_body; indx++) {
        // Orientation
        double *Aijpp_ptr = body[indx].Aijpp;
        *(Aijpp_ptr++) = cos(body[indx].qi);	*(Aijpp_ptr++) = -sin(body[indx].qi);	*(Aijpp_ptr++) = 0;
        *(Aijpp_ptr++) = sin(body[indx].qi);	*(Aijpp_ptr++) = cos(body[indx].qi);	*(Aijpp_ptr++) = 0;
        *(Aijpp_ptr++) = 0;						*(Aijpp_ptr++) = 0;						*(Aijpp_ptr++) = 1;
        memset(body[indx].Hi, 0, sizeof(double) * 3);
        memset(body[indx].Ai, 0, sizeof(double) * 9);
        memset(body[indx].Ai_Cij, 0, sizeof(double) * 9);
        for (uint i = 0; i < 3; i++) {
            for (uint j = 0; j < 3; j++) {
                for (uint k = 0; k < 3; k++) {
                    body[indx].Ai_Cij[i * 3 + j] += body[indx - 1].Ai[i * 3 + k] * body[indx - 1].Cij[k * 3 + j];
                }
            }
        }
        for (uint i = 0; i < 3; i++) {
            for (uint j = 0; j < 3; j++) {
                body[indx].Hi[i] += body[indx].Ai_Cij[i * 3 + j] * body[indx].u_vec[j];
                for (uint k = 0; k < 3; k++) {
                    body[indx].Ai[i * 3 + j] += body[indx].Ai_Cij[i * 3 + k] * body[indx].Aijpp[k * 3 + j];
                }
            }
        }
        // position
        memset(body[indx - 1].sij, 0, sizeof(double) * 3);
        for (uint i = 0; i < 3; i++) {
            for (uint j = 0; j < 3; j++) {
                body[indx - 1].sij[i] += body[indx - 1].Ai[i * 3 + j] * body[indx - 1].sijp[j];
            }
            body[indx].ri[i] = body[indx - 1].ri[i] + body[indx - 1].sij[i];
        }
    }
    // End point
    for (uint i = 0; i < 3; i++) {
        body[num_body].sij[i] = 0;
        for (uint j = 0; j < 3; j++) {
            body[num_body].sij[i] += body[num_body].Ai[i * 3 + j] * body[num_body].sijp[j];
        }
        body[num_body].re[i] = body[num_body].ri[i] + body[num_body].sij[i];
    }
    for (uint i = 0; i < 3; i++) {
        for (uint j = 0; j < 3; j++) {
            body[num_body].Ae[i * 3 + j] = 0;
            for (uint k = 0; k < 3; k++) {
                body[num_body].Ae[i * 3 + j] += body[num_body].Ai[i * 3 + k] * body[num_body].Cij[k * 3 + j];
            }
        }
    }
    body[num_body].roll = atan2(body[num_body].Ae[2 * 3 + 1], body[num_body].Ae[2 * 3 + 2]);
    body[num_body].pitch = atan2(-body[num_body].Ae[2 * 3 + 0], sqrtf(pow(body[num_body].Ae[2 * 3 + 1], 2.0f) + powf(body[num_body].Ae[2 * 3 + 2], 2.0f)));
    body[num_body].yaw = atan2(body[num_body].Ae[1 * 3 + 0], body[num_body].Ae[0 * 3 + 0]);
}

void RobotArm::inverse_kinematics(double des_pos[3], double des_ang[3]) {
    for (uint i = 0; i < 3; i++) {
        PH_pos[i] = des_pos[i] - body[num_body].re[i];
    }
    PH_ori[0] = des_ang[0] - body[num_body].roll;
    PH_ori[1] = des_ang[1] - body[num_body].pitch;
    PH_ori[2] = des_ang[2] - body[num_body].yaw;

    for (uint i = 0; i < 3; i++) {
        PH[i] = PH_pos[i];
        PH[i + 3] = PH_ori[i];
    }

    jacobian();

    double *U, *s, *V;
    U = new double[dof * dof];
    s = new double[MIN(dof, num_body)];
    V = new double[num_body*num_body];

    numeric->svdcmp(J, dof, num_body, U, s, V);

    memset(JD, 0, sizeof(double) * num_body*dof);
    double *temp = new double[num_body*dof];
    double lamda = 1e-5f;
    for (uint i = 0; i < dof; i++) {
        for (uint j = 0; j < num_body; j++) {
            for (uint k = 0; k < dof; k++) {
                temp[j * dof + k] = V[j * num_body + i] * U[k * num_body + i];
            }
        }
        for (uint j = 0; j < num_body; j++) {
            for (uint k = 0; k < dof; k++) {
                JD[j * dof + k] += (s[i] / (s[i]*s[i] +lamda*lamda))*(temp[j * dof + k]);
            }
        }
    }

    delete[] s;
    delete[] U;
    delete[] V;
    delete[] temp;

    memset(delta_q, 0, sizeof(double) * 6);
    for (uint i = 0; i < num_body; i++) {
        for (uint j = 0; j < num_body; j++) {
            delta_q[i] += JD[i * num_body + j] * PH[j];
        }
    }

    for (uint i = 0; i < num_body; i++) {
        body[i + 1].qi += delta_q[i];
    }
}

void RobotArm::jacobian()
{
    double *Jv = new double[3 * num_body];
    double *Jw = new double[3 * num_body];

    //for (uint indx = 1; indx <= num_body; indx++) {
    //	for (uint i = 0; i < 3; i++) {
    //		body[indx].oi[i] = des_pos[i] - body[indx].ri[i];
    //	}
    //	tilde(body[indx].Hi, body[indx].zit);
    //	for (uint i = 0; i < 3; i++) {
    //		body[indx].Jvi[i] = 0;
    //		for (uint j = 0; j < 3; j++) {
    //			body[indx].Jvi[i] += body[indx].zit[i * 3 + j] * body[indx].oi[j];
    //		}
    //	}
    //	for (uint j = 0; j < 3; j++) {
    //		Jv[j * num_body + indx - 1] = body[indx].Jvi[j];
    //		Jw[j * num_body + indx - 1] = body[indx].Hi[j];
    //	}
    //}

    for (uint indx = 1; indx <= num_body; indx++) {
        for (uint i = 0; i < 3; i++) {
            for (uint j = 0; j < 3; j++) {
                body[indx - 1].Cij_Aijpp[i * 3 + j] = 0;
                for (uint k = 0; k < 3; k++) {
                    body[indx - 1].Cij_Aijpp[i * 3 + j] += body[indx - 1].Cij[i * 3 + k] * body[indx].Aijpp[k * 3 + j];
                }
            }
        }
    }

    for (uint indx = 1; indx <= num_body; indx++) {
        double *Aijpp_qi_ptr = body[indx].Aijpp_qi;
        *(Aijpp_qi_ptr++) = -sin(body[indx].qi);	*(Aijpp_qi_ptr++) = -cos(body[indx].qi);	*(Aijpp_qi_ptr++) = 0;
        *(Aijpp_qi_ptr++) =  cos(body[indx].qi);	*(Aijpp_qi_ptr++) = -sin(body[indx].qi);	*(Aijpp_qi_ptr++) = 0;
        *(Aijpp_qi_ptr++) = 0;						*(Aijpp_qi_ptr++) = 0;						*(Aijpp_qi_ptr) = 0;
        memset(body[indx].A6_qi, 0, sizeof(double) * 9);
        memset(body[indx].Ae_qi, 0, sizeof(double) * 9);
        memset(body[indx].r6_qi, 0, sizeof(double) * 3);
        memset(body[indx].re_qi, 0, sizeof(double) * 3);
        double temp[9] = { 0, };
        for (uint indx2 = indx; indx2 <= num_body; indx2++) {
            if (indx2 == indx) {
                for (uint i = 0; i < 3; i++) {
                    for (uint j = 0; j < 3; j++) {
                        for (uint k = 0; k < 3; k++) {
                            body[indx].A6_qi[i * 3 + j] += body[indx2].Ai_Cij[i * 3 + k] * body[indx2].Aijpp_qi[k * 3 + j];
                        }
                    }
                }
            }
            else {
                for (uint i = 0; i < 3; i++) {
                    for (uint j = 0; j < 3; j++) {
                        temp[i * 3 + j] = 0;
                        for (uint k = 0; k < 3; k++) {
                            temp[i * 3 + j] += body[indx].A6_qi[i * 3 + k] * body[indx2 - 1].Cij_Aijpp[k * 3 + j];
                        }
                    }
                }
                memcpy(body[indx].A6_qi, temp, sizeof(double) * 9);
            }
            if (indx2 < num_body) {
                for (uint i = 0; i < 3; i++) {
                    for (uint j = 0; j < 3; j++) {
                        body[indx].r6_qi[i] += body[indx].A6_qi[i * 3 + j] * body[indx2].sijp[j];
                    }
                }
            }
        }
        for (uint i = 0; i < 3; i++) {
            for (uint j = 0; j < 3; j++) {
                for (uint k = 0; k < 3; k++) {
                    body[indx].Ae_qi[i * 3 + j] += body[indx].A6_qi[i * 3 + k] * body[num_body].Cij[k * 3 + j];
                }
            }
        }
        for (uint i = 0; i < 3; i++) {
            for (uint j = 0; j < 3; j++) {
                body[indx].re_qi[i] += body[indx].A6_qi[i * 3 + j] * body[num_body].sijp[j];
            }
            if (indx < num_body) {
                body[indx].re_qi[i] += body[indx].r6_qi[i];
            }
        }
    }
    for (uint i = 0; i < 3; i++) {
        for (uint j = 0; j < num_body; j++) {
            Jv[i*num_body + j] = body[j + 1].re_qi[i];
        }
    }

    double Ae_31 = body[num_body].Ae[6];
    double Ae_32 = body[num_body].Ae[7];
    double Ae_33 = body[num_body].Ae[8];
    double Ae_21 = body[num_body].Ae[3];
    double Ae_11 = body[num_body].Ae[0];
    for (uint indx = 1; indx <= num_body; indx++) {
        body[indx].Ae_qi_31 = body[indx].Ae_qi[6];
        body[indx].Ae_qi_32 = body[indx].Ae_qi[7];
        body[indx].Ae_qi_33 = body[indx].Ae_qi[8];
        body[indx].Ae_qi_21 = body[indx].Ae_qi[3];
        body[indx].Ae_qi_11 = body[indx].Ae_qi[0];
    }

    double roll_q_temp1 = Ae_32 * Ae_32 + Ae_33 * Ae_33;
    double roll_q_temp2 = sqrt(roll_q_temp1);
    double roll_q_temp3 = Ae_33 + roll_q_temp2;
    double roll_q_temp4 = roll_q_temp2 * (roll_q_temp1 + Ae_33*roll_q_temp2);

    double pitch_q_temp1 = sqrt(Ae_32*Ae_32 + Ae_33*Ae_33);
    double pitch_q_temp2 = Ae_31 * Ae_31 + pitch_q_temp1 * pitch_q_temp1;
    double pitch_q_temp3 = sqrt(pitch_q_temp2);
    double pitch_q_temp4 = pitch_q_temp3 * (pitch_q_temp2 + pitch_q_temp1 * pitch_q_temp3);

    double yaw_q_temp1 = Ae_21 * Ae_21 + Ae_11 * Ae_11;
    double yaw_q_temp2 = sqrt(yaw_q_temp1);
    double yaw_q_temp3 = Ae_11 + yaw_q_temp2;
    double yaw_q_temp4 = yaw_q_temp2 * (yaw_q_temp1 + Ae_11*yaw_q_temp2);

    for (uint indx = 1; indx <= num_body; indx++) {
        body[indx].roll_qi = (roll_q_temp3*(body[indx].Ae_qi_32*Ae_33 - Ae_32*body[indx].Ae_qi_33)) / roll_q_temp4;
        body[indx].pitch_qi = -((pitch_q_temp3 + pitch_q_temp1)*(body[indx].Ae_qi_31*pitch_q_temp1 - Ae_31 * (Ae_32*body[indx].Ae_qi_32 + Ae_33 * body[indx].Ae_qi_33)/pitch_q_temp1))/ pitch_q_temp4;
        body[indx].yaw_qi = (yaw_q_temp3*(body[indx].Ae_qi_21*Ae_11 - Ae_21*body[indx].Ae_qi_11)) / yaw_q_temp4;
    }

    for (uint i = 0; i < num_body; i++) {
        Jw[0 * num_body + i] = body[i + 1].roll_qi;
        Jw[1 * num_body + i] = body[i + 1].pitch_qi;
        Jw[2 * num_body + i] = body[i + 1].yaw_qi;
    }

    memcpy(J, Jv, sizeof(double) * 3 * num_body);
    memcpy(J + 3 * num_body, Jw, sizeof(double) * 3 * num_body);

    delete[] Jv;
    delete[] Jw;
}

void RobotArm::save_data() {
    fprintf(fp, "%.7f\t", t_current);
    for (uint i = 1; i <= num_body; i++) {
        fprintf(fp, "%.7f\t", body[i].qi);
    }
    kinematics();
    fprintf(fp, "%.7f\t%.7f\t%.7f\t", body[num_body].re[0], body[num_body].re[1], body[num_body].re[2]);
    fprintf(fp, "%.7f\t%.7f\t%.7f", body[num_body].roll, body[num_body].pitch, body[num_body].yaw);
    fprintf(fp, "\n");
}
