#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

#include <QTimer>
#include <QObject>

#include <iostream>
#include <vector>

#include "robotarm.h"
#include "tcpserver.h"
#include "dxlcontrol.h"

using namespace std;

struct ServerToClient{
    unsigned long now, previous, ik_now, ik_previous, dxl_now, dxl_previous;
    double desCart[6], curJoint[6], curCart[6], desJoint[6];
    double t;
};
struct ClientToServer{
    double desCart[6], desJoint[6];
    uint mode;
};

enum{moveStart=0, moveReady, moveJoint, moveCart, moveRect, moveStop};

class RobotControl : public QObject
{
    Q_OBJECT
public:
    explicit RobotControl(QObject *parent = 0);
    ~RobotControl();
    void run();

public slots:
    void readMessage();
    void disconnected();
    void connectedClient();

    void timeout();
    void timeoutDxl();

public:
    RT_TASK robot_task, comm_task, dxl_task;
    RTIME now, previous, ik_time1, ik_time2, dxl_time1, dxl_time2;

    RobotArm *robot;
    double h, t;
    double pose_d[6], q_c[6], pose_c[6], input_q[6], pose_d_old[6];

    QTimer *timer, *timerDxl;

    TcpServer *tcpServer;
    vector<ServerToClient> mServerToClient;
    ClientToServer mClientToServer;
    DxlControl *dxlControl;

    int32_t goal_position[6], present_position[6];

    bool readyFlag, moveCartFlag, moveRectFlag, poseCartUpdateFlag;
    int moveRectIndx;
    bool debugging;
};

const int32_t offset[6] = {2038, 500, 1672, 3200, 1142, 340};
const int32_t ready_pose[6] = {30, 30, 90, -120, 30, 0};
const double ready_pose_cart[6] = {-0.2710368, 0.2272252, 0.0781693, 1.5707963, -0.0000000, -2.0943951};
const double moveRectPath[4][6] = {
    -345.32,	224.94,	-23.95,	-44.85,	83.87,	127.01,
    -59.69, 	237.5,	-11.29,	-91.95,	66.82,	79.74,
    -348.36,	14.5,	1.04,	-49.07,	83.52,	151.54,
    -146.8, 	61.95,	-67.95,	90.19,	58.42,	-36.23
};

#endif // ROBOTCONTROL_H
