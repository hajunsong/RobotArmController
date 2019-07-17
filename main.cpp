#include "variables.h"

void catch_signal(int sig = 0)
{
    if (sig != 0){
        printf("Catch Signal : %d\n", sig);

        rt_task_delete(&robot_task);
        rt_task_delete(&comm_task);

        printf("complete\n");
        exit(1);
    }
}

void robot_operating(void *arg)
{
    cout << arg << endl;
//    unsigned long step, time = 0, ik_step;

    /*
     * Arguments: &task (NULL=self), start time, period (here: 1 ms)
     */
    rt_task_set_periodic(NULL, TM_NOW, 1e6);
    previous = rt_timer_read();

    ServerToClient tempServerToClient;

    while (1) {
        rt_task_wait_period(NULL);
        now = rt_timer_read();

        switch(mClientToServer.mode){
            case moveReady:
                break;
            case moveJoint:
                break;
            case moveCart:
                break;
            case moveRect:
                break;
            default :
                break;
        }

        input_q[0] = 0.5235988;
        input_q[1] = 0.5235988;
        input_q[2] = 1.5707964;
        input_q[3] = -2.0943951;
        input_q[4] = 0.5235988;
        input_q[5] = 0.0000000;

        pose_d[0] = -0.2710368;
        pose_d[1] = 0.2272252;
        pose_d[2] = 0.0781693;
        pose_d[3] = 1.5707963;
        pose_d[4] = -0.0000000;
        pose_d[5] = -2.0943951;

        ik_time1 = rt_timer_read();
        robot.run_inverse_kinematics(input_q, pose_d, q_c, pose_c);
        ik_time2 = rt_timer_read();

        memcpy(tempServerToClient.curJoint, q_c, sizeof(double)*6);
        memcpy(tempServerToClient.curCart, pose_c, sizeof(double)*6);

        tempServerToClient.now = (unsigned long)now;
        tempServerToClient.previous = (unsigned long)previous;
        tempServerToClient.ik_now = (unsigned long)ik_time2;
        tempServerToClient.ik_previous = (unsigned long)ik_time1;
        tempServerToClient.t = t;

        mServerToClient.push_back(tempServerToClient);

        rt_printf("Time : %ld.%06ld ms\t IK Time : %ld.%06ld ms\n",
                  (unsigned long)(now - previous) / 1000000,
                  (unsigned long)(now - previous) % 1000000,
                  (unsigned long)(ik_time2 - ik_time1) / 1000000,
                  (unsigned long)(ik_time2 - ik_time1) % 1000000);

        previous = now;
        t += h;

        catch_signal();
    }
}

void timeout(){
    timer.stop();
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
                for(double q : tempServerToClient.curJoint){
                    txData.append(QByteArray::number(q, 'f', 17));
                    txData.append(",");
                }
                for(double pos : tempServerToClient.curCart){
                    txData.append(QByteArray::number(pos, 'f', 17));
                    txData.append(",");
                }
                for(double pos : mClientToServer.desJoint){
                    txData.append(QByteArray::number(pos, 'f', 17));
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
    timer.start();
}

void readMessage(){
    QByteArray rxData = tcpServer->socket->readAll();

    QList<QByteArray> Data = rxData.split(',');
    if (Data.length() > 10){
        for (int i = 0; i < 6; i++){
            mClientToServer.desJoint[i] = Data.at(i).toDouble();
            mClientToServer.desCart[i] = Data.at(i + 6).toDouble();
        }
        mClientToServer.mode = Data.at(12).toUInt();
    }
}

void disconnected(){
    tcpServer->connectState = false;
}

void connectedClient(){
    tcpServer->connectState = true;
    QObject::connect(tcpServer->socket, &QTcpSocket::readyRead, readMessage);
    QObject::connect(tcpServer->socket, &QTcpSocket::disconnected, disconnected);
}

int main(int argc, char* argv[])
{
    QCoreApplication a(argc, argv);

    rt_print_auto_init(1);

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    printf("Now running...\n");

    /* Arguments: &task, name, stack size (0=default), priority, mode (FPU, start suspended, ...) */
    rt_task_create(&robot_task, "RobotTask", 0, 99, 0);

    memset(&mServerToClient, 0, sizeof(ServerToClient));
    memset(&mClientToServer, 0, sizeof(ClientToServer));

    /* Arguments: &task, task function, function argument */
    rt_task_start(&robot_task, &robot_operating, NULL);

//    rt_task_create(&comm_task, "CommunicationTask", 0, 75, 0);
//    rt_task_start(&comm_task, &communicating, NULL);

    tcpServer = new TcpServer();
    tcpServer->setting("192.168.137.100", 9090);
    tcpServer->startServer();
    QObject::connect(tcpServer, &TcpServer::connectedClient, connectedClient);

    QObject::connect(&timer, &QTimer::timeout, timeout);
    timer.setInterval(1);
    timer.start();

    return a.exec();
}


