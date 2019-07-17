#pragma once

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

#include <QCoreApplication>
#include <QTimer>
#include <QObject>

#include <iostream>
#include <vector>

#include "robotarm.h"
#include "tcpserver.h"

using namespace std;

RT_TASK robot_task;
RT_TASK comm_task;
RTIME now, previous, ik_time1, ik_time2;

RobotArm robot(6,6);
double h = 0.001;
double t = 0;
double pose_d[6], q_c[6], pose_c[6], input_q[6];

QTimer timer;

struct ServerToClient{
    unsigned long now, previous, ik_now, ik_previous;
    double desCart[6], curJoint[6], curCart[6], desJoint[6];
    double t;
};
struct ClientToServer{
    double desCart[6], desJoint[6];
    uint mode;
};

enum{moveReady=0, moveJoint, moveCart, moveRect};

TcpServer *tcpServer;
vector<ServerToClient> mServerToClient;
ClientToServer mClientToServer;
