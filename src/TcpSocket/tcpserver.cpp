#include "tcpserver.h"

TcpServer::TcpServer(QObject *parent) : QTcpServer(parent) {
    connectState = false;
}

void TcpServer::startServer() {
    if (!this->listen(QHostAddress(ip), port))
    {
        qDebug() << "Could not start server";
    }
    else
    {
        qDebug() << "Listening ...";
    }
}

void TcpServer::setting(QString IP, quint16 PORT)
{
    ip = IP;
    port = PORT;
}

void TcpServer::incomingConnection(qintptr socketDescriptor) {
    // We have a new connection
    qDebug() << QString::number(socketDescriptor) + " Connecting...";

    socket = new QTcpSocket();

    // set the ID
    if (!socket->setSocketDescriptor(socketDescriptor))
    {
        // something's wrong, we just emit a signal
        emit error(socket->error());
        return;
    }

    // connect socket and signal
    // note - Qt::DirectConnection is used because it's multithreaded
    //        This makes the slot to be invoked immediately, when the signal is emitted.

//    connect(socket, SIGNAL(readyRead()), this, SLOT(readyRead()), Qt::DirectConnection);
//    connect(socket, SIGNAL(disconnected()), this, SLOT(disconnected()), Qt::DirectConnection);

//    connectState = true;
    emit connectedClient();
}	

void TcpServer::readyRead() {
//    rxData = socket->readAll();
//    qDebug() << "Receive Data : " + rxData;
}

void TcpServer::disconnected() {
    qDebug() << QString::number(socket->socketDescriptor()) + " Disconnected";
    socket->deleteLater();
    socket->close();
    connectState = false;
}
