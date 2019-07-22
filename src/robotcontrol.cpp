#include "robotcontrol.h"

RobotControl::RobotControl(QObject *parent) : QObject(parent)
{
    t = 0;
    h = 0;
    ik_time1 = 0;
    ik_time2 = 0;

    memset(&mClientToServer, 0, sizeof(ClientToServer));
    mClientToServer.mode = 10;

    memset(goal_position, 0, sizeof(double)*6);
    memset(present_position, 0, sizeof(double)*6);
    memset(pose_d, 0, sizeof(double)*6);
    memset(q_c, 0, sizeof(double)*6);
    memset(pose_c, 0, sizeof(double)*6);
    memset(input_q, 0, sizeof(double)*6);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &RobotControl::timeout);
    timer->setInterval(5);

    timerDxl = new QTimer(this);
    connect(timerDxl, &QTimer::timeout, this, &RobotControl::timeoutDxl);
    timerDxl->setInterval(100);

    tcpServer = new TcpServer();
    tcpServer->setting("192.168.137.100", 9090);
    tcpServer->startServer();
    connect(tcpServer, &TcpServer::connectedClient, this, &RobotControl::connectedClient);

    robot = new RobotArm(6,6);

    dxlControl = new DxlControl();
    dxlControl->init();
    // for(int i = 1; i <= 6; i++){
    //     dxlControl->dxl_init(i);
    //     printf("ID %d Current Position : %d\n", i, dxlControl->getPresentPosition(i));
    // }
    readyFlag = false;
    moveCartFlag = false;
    moveRectFlag = false;
    moveRectIndx = 0;
    debugging = false;
    poseCartUpdateFlag = false;
}

RobotControl::~RobotControl(){
    delete tcpServer;
    delete timer;
    delete dxlControl;
    delete robot;
}

void RobotControl::readMessage(){
    QByteArray rxData = tcpServer->socket->readAll();

    char ch[3];
    for (int j = 0; j < 3; j++) {
        ch[j] = rxData.at(j);
    }

    if (ch[0] == 0x02 && ch[1] == 0x03){
        uint id = ch[2];
        int init_result = dxlControl->dxl_init(id);
        
        QByteArray txData;
        txData.append(QByteArray::fromRawData("\x0A\x03", 2));
        txData.append(id);
        txData.append(init_result);
        txData.append(QByteArray::number(dxlControl->getPresentPosition(id)));
        txData.append(QByteArray::fromRawData("\x02\x03", 2));
        tcpServer->socket->write(txData);
    }
    else if(ch[0] == 0x02 && ch[1] == 0x04){
        int id = rxData.at(2);
        dxlControl->dxl_deinit(id);

        QByteArray txData;
        txData.append(QByteArray::fromRawData("\x0A\x04", 2));
        txData.append(id);
        txData.append(QByteArray::fromRawData("\x00", 1));
        txData.append(QByteArray::fromRawData("\x02\x04", 2));
        tcpServer->socket->write(txData);
    }
    else{
        QList<QByteArray> Data = rxData.split(',');
        if (Data.length() > 10){
            for (int i = 0; i < 6; i++){
                mClientToServer.desJoint[i] = Data.at(i).toDouble();
                mClientToServer.desCart[i] = Data.at(i + 6).toDouble();
            }
            mClientToServer.mode = Data.at(12).toUInt();

            switch(mClientToServer.mode){
                case moveStart:
                    readyFlag = true;
                    moveCartFlag = false;
                    moveRectFlag = false;
                    for (int i = 0; i < 6; i++)
                    {
                        goal_position[i] = offset[i];
                    }

                    timerDxl->start();
                    run();
                    break;
                case moveReady:
                    for(int i = 0; i < 6; i++){
                        goal_position[i] = ready_pose[i]/POSITION_UNIT + offset[i];
                    }
                    moveCartFlag = false;
                    moveRectFlag = false;
                    for(int i = 0; i < 6; i++){
                        pose_d_old[i] = ready_pose_cart[i];
                    }
                    break;
                case moveJoint:
                    moveCartFlag = false;
                    moveRectFlag = false;
                    for (int i = 0; i < 6; i++){
                        goal_position[i] = present_position[i] + (int32_t)(mClientToServer.desJoint[i]/POSITION_UNIT);
                    }
                    break;
                case moveCart:
                    moveCartFlag = true;
                    poseCartUpdateFlag = true;
                    break;
                case moveRect:
                    moveRectFlag = true;
                    break;
                case moveStop:
                    timerDxl->stop();
                    rt_task_delete(&robot_task);
                    break;
                case 10:
                    debugging = false;
                    break;
                default:
                    break;
            }
        }
    }
}

void RobotControl::disconnected(){
    tcpServer->connectState = false;
    timer->stop();
    timerDxl->stop();
}

void RobotControl::connectedClient(){
    tcpServer->connectState = true;
    connect(tcpServer->socket, &QTcpSocket::readyRead, this, &RobotControl::readMessage);
    connect(tcpServer->socket, &QTcpSocket::disconnected, this, &RobotControl::disconnected);
    timer->start();
}

void RobotControl::timeout(){
    if (tcpServer->connectState){
        QByteArray txData;
        if (mServerToClient.size() > 0){
            do
            {
                ServerToClient tempServerToClient = mServerToClient.front();
                txData.append(QByteArray::number(tempServerToClient.t));
                txData.append(",");
                txData.append(QByteArray::number((unsigned long long)(tempServerToClient.now - tempServerToClient.previous)));
                txData.append(",");
                txData.append(QByteArray::number((unsigned long long)(tempServerToClient.ik_now - tempServerToClient.ik_previous)));
                txData.append(",");
                txData.append(QByteArray::number((unsigned long long)(tempServerToClient.dxl_now - tempServerToClient.dxl_previous)));
                txData.append(",");
                for(double q : tempServerToClient.curJoint){
                    txData.append(QByteArray::number(q, 'f', 17));
                    txData.append(",");
                }
                for(double pos : tempServerToClient.curCart){
                    txData.append(QByteArray::number(pos, 'f', 17));
                    txData.append(",");
                }
                for(double q : mClientToServer.desJoint){
                    txData.append(QByteArray::number(q/POSITION_UNIT, 'f', 17));
                    txData.append(",");
                }
                for(double pos : mClientToServer.desCart){
                    txData.append(QByteArray::number(pos, 'f', 17));
                    txData.append(",");
                }
                txData.append("\n");
                mServerToClient.erase(mServerToClient.begin());
            } while(mServerToClient.size() > 0);

            tcpServer->socket->write(txData);
        }
    }
    else{
        mServerToClient.clear();
    }
}

void RobotControl::timeoutDxl()
{
    // if (readyFlag){
    //     dxlControl->setGroupSyncWriteGoalPosition(goal_position);
    // }
    // dxlControl->getGroupSyncReadPresentPosition(present_position);

//    for(int i = 1; i <= 6; i++){
//        printf("CurrentPosition : %d\n", present_position[i]);
//    }

//    dxlControl->getGroupSyncReadPresentPosition(present_position);
}

void catch_signal(int sig=0)
{
    if (sig != 0){
        exit(1);
    }
}

void robot_operating(void *arg)
{
    //    unsigned long step, time = 0, ik_step;

    /*
     * Arguments: &task (NULL=self), start time, period (here: 1 ms)
     */

    RobotControl *rt = (RobotControl*)arg;

    rt_task_set_periodic(NULL, TM_NOW, 1e7);
    rt->previous = rt_timer_read();

    ServerToClient tempServerToClient;

    double input_q[6] = {0,}, pose_d[6] = {0,}, q_c[6] = {0,}, pose_c[6] = {0,};
    int flag5 = 0, flag4 = 0, flag3 = 0, flag2 = 0, flag1 = 0;

    while (1) {
        rt_task_wait_period(NULL);
        rt->now = rt_timer_read();

        rt->dxl_time1 = rt_timer_read();
        rt->dxlControl->getGroupSyncReadPresentPosition(rt->present_position);
        if (rt->readyFlag){
            rt->dxlControl->setGroupSyncWriteGoalPosition(rt->goal_position);

            // axis 6
            // rt->goal_position[5] = rt->present_position[5] + 10;

            // axis 5
            // if (flag5 == 0) rt->goal_position[4] += 10;
            // else rt->goal_position[4] -= 10;
            // if (flag5 == 0)
            //     if (rt->present_position[4] >= 2000)
            //         flag5 = 1;
            // if (flag5 == 1)
            //     if (rt->present_position[4] <= 200)
            //         flag5 = 0;

            // axis 4
            // if (flag4 == 0) rt->goal_position[3] -= 10;
            // else rt->goal_position[3] += 10;
            // if (flag4 == 0)
            //     if (rt->present_position[3] <= 1100)
            //         flag4 = 1;
            // if (flag4 == 1)
            //     if (rt->present_position[3] >= 3200)
            //         flag4 = 0;

            // axis 3
            // if (flag3 == 0) rt->goal_position[2] -= 10;
            // else rt->goal_position[2] += 10;
            // if (flag3 == 0)
            //     if (rt->present_position[2] <= 600)
            //         flag3 = 1;
            // if (flag3 == 1)
            //     if (rt->present_position[2] >= 2600)
            //         flag3 = 0;
            
            // axis 2
            // if (flag2 == 0) rt->goal_position[1] -= 10;
            // else rt->goal_position[1] += 10;
            // if (flag2 == 0)
            //     if (rt->present_position[1] <= -526)
            //         flag2 = 1;
            // if (flag2 == 1)
            //     if (rt->present_position[1] >= 1480)
            //         flag2 = 0;

            // rt_printf("%d\n", rt->present_position[1]);
        }

        rt->dxl_time2 = rt_timer_read();
        
        for(int i = 0; i < 6; i++){
            rt->q_c[i] = static_cast<double>((rt->present_position[i] - offset[i])*POSITION_UNIT*DEG2RAD);
            // rt_printf("q_c : %d : %f\n", i+1, rt->q_c[i]);
        }
        rt->robot->run_kinematics(rt->q_c, rt->pose_c);
        // for(int i = 0; i < 6; i++){
        //     rt_printf("pose_c : %d : %f\n", i+1, rt->pose_c[i]);
        // }

        // rt_printf("%d, %d\n", rt->moveCartFlag, rt->moveRectFlag);
        if(rt->moveCartFlag || rt->moveRectFlag){
            memcpy(rt->input_q, rt->q_c, sizeof(double)*6);
            // memcpy(rt->pose_d, rt->pose_d_old, sizeof(double)*6);
            
            // for(int i = 0; i < 6; i++){
            //     rt_printf("1 pose_d : %d : %f\n", i+1, rt->pose_d[i]);
            // }

            if (rt->moveCartFlag && rt->poseCartUpdateFlag){
                rt->pose_d[0] = rt->pose_d_old[0] + rt->mClientToServer.desCart[0]*0.001;
                rt->pose_d[1] = rt->pose_d_old[1] + rt->mClientToServer.desCart[1]*0.001;
                rt->pose_d[2] = rt->pose_d_old[2] + rt->mClientToServer.desCart[2]*0.001;
                rt->pose_d[3] = rt->pose_d_old[3] + rt->mClientToServer.desCart[3]*DEG2RAD;
                rt->pose_d[4] = rt->pose_d_old[4] + rt->mClientToServer.desCart[4]*DEG2RAD;
                rt->pose_d[5] = rt->pose_d_old[5] + rt->mClientToServer.desCart[5]*DEG2RAD;
                for (int i = 0; i < 6; i++)
                {
                    rt_printf("\ndesCart : %d : %f\t", i + 1, rt->mClientToServer.desCart[i]);
                    rt_printf("pose_d_old : %d : %f\t", i + 1, rt->pose_d_old[i]);
                    rt_printf("pose_d : %d : %f\n", i + 1, rt->pose_d[i]);
                }
                rt->poseCartUpdateFlag = false;
            }

            if (rt->moveRectFlag){
                rt->pose_d[0] = moveRectPath[rt->moveRectIndx][0]*0.001;
                rt->pose_d[1] = moveRectPath[rt->moveRectIndx][1]*0.001;
                rt->pose_d[2] = moveRectPath[rt->moveRectIndx][2]*0.001;
                rt->pose_d[3] = moveRectPath[rt->moveRectIndx][3]*DEG2RAD;
                rt->pose_d[4] = moveRectPath[rt->moveRectIndx][4]*DEG2RAD;
                rt->pose_d[5] = moveRectPath[rt->moveRectIndx][5]*DEG2RAD;

                if (rt->pose_c[0] - rt->pose_d[0] <= 0.01 && rt->pose_c[1] - rt->pose_d[1] <= 0.01
                    && rt->pose_c[2] - rt->pose_d[2] <= 0.01 && rt->pose_c[3] - rt->pose_d[3] <= 0.01
                    && rt->pose_c[4] - rt->pose_d[4] <= 0.01 && rt->pose_c[5] - rt->pose_d[5] <= 0.01){
                    // rt->moveRectIndx++;
                    // if (rt->moveRectIndx >= 4) rt->moveRectIndx = 0;
                }
            }

            
            // for(int i = 0; i < 6; i++){
            //     rt_printf("2 pose_d : %d : %f\n", i+1, rt->pose_d[i]);
            // }

            rt->ik_time1 = rt_timer_read();
            rt->robot->run_inverse_kinematics(rt->input_q, rt->pose_d, rt->q_c, rt->pose_c);
            memcpy(rt->pose_d_old, rt->pose_d, sizeof(double)*6);
            // rt_printf("input_q : %f, %f, %f, %f, %f, %f\n", rt->input_q[0], rt->input_q[1], rt->input_q[2], rt->input_q[3], rt->input_q[4], rt->input_q[5]);
            // rt_printf("pose_d : %f, %f, %f, %f, %f, %f\n", rt->pose_d[0], rt->pose_d[1], rt->pose_d[2], rt->pose_d[3], rt->pose_d[4], rt->pose_d[5]);
            // rt_printf("q_c : %f, %f, %f, %f, %f, %f\n", rt->q_c[0], rt->q_c[1], rt->q_c[2], rt->q_c[3], rt->q_c[4], rt->q_c[5]);
            // rt_printf("pose_c : %f, %f, %f, %f, %f, %f\n", rt->pose_c[0], rt->pose_c[1], rt->pose_c[2], rt->pose_c[3], rt->pose_c[4], rt->pose_c[5]);
            rt->ik_time2 = rt_timer_read();

            for(int i = 0; i < 6; i++){
                rt->goal_position[i] = static_cast<int32_t>(rt->q_c[i]*RAD2DEG/POSITION_UNIT) + offset[i];
            }
        }

        memcpy(tempServerToClient.curJoint, rt->q_c, sizeof(double)*6);
        memcpy(tempServerToClient.curCart, rt->pose_c, sizeof(double)*6);

        tempServerToClient.now = (unsigned long)rt->now;
        tempServerToClient.previous = (unsigned long)rt->previous;
        tempServerToClient.ik_now = (unsigned long)rt->ik_time2;
        tempServerToClient.ik_previous = (unsigned long)rt->ik_time1;
        tempServerToClient.t = rt->t;
        tempServerToClient.dxl_now = (unsigned long)rt->dxl_time2;
        tempServerToClient.dxl_previous = (unsigned long)rt->dxl_time1;

        rt->mServerToClient.push_back(tempServerToClient);

        /**
        rt_printf("Time : %ld.%06ld ms\t IK Time : %ld.%06ld ms\n",
                  (unsigned long)(rt->now - rt->previous) / 1000000,
                  (unsigned long)(rt->now - rt->previous) % 1000000,
                  (unsigned long)(rt->ik_time2 - rt->ik_time1) / 1000000,
                  (unsigned long)(rt->ik_time2 - rt->ik_time1) % 1000000);
        /**/

        rt->previous = rt->now;
        rt->t += rt->h;

        catch_signal();
    }
}

void RobotControl::run(){
    rt_print_auto_init(1);

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    /* Arguments: &task, name, stack size (0=default), priority, mode (FPU, start suspended, ...) */
    rt_task_create(&robot_task, "RobotControlTask", 0, 1, 0);

    /* Arguments: &task, task function, function argument */
    rt_task_start(&robot_task, robot_operating, this);
}
