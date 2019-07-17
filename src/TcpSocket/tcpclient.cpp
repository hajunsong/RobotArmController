#include "tcpclient.h"

TcpClient::TcpClient(QObject *parent) : QObject(parent) {
    socket = new QTcpSocket;
}

TcpClient::~TcpClient() {
    delete socket;
}

void TcpClient::connectToServer() {
    socket->connectToHost(ipAddress, port);

    connect(socket, SIGNAL(connected()), this, SLOT(onConnectServer()));
    connect(socket, SIGNAL(readyRead()), this, SLOT(readMessage()));

    QByteArray txData;
    txData.append(QByteArray::fromRawData("\x01\x02", 2));
    socket->write(txData);
    qDebug() << "Transmit Data : " + txData.toHex();
}

void TcpClient::setIpAddress(QString address) {
    ipAddress = address;
}

void TcpClient::setPort(quint16 num){
    port = num;
}

void TcpClient::onConnectServer(){
    qDebug() << "Connect complete";
}

void TcpClient::readMessage(){
    rxData = socket->readAll();
    qDebug() << "Receive Data : " + rxData.mid(0, 17);
}
