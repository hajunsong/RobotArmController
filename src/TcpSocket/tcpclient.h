#pragma once

#include <iostream>
#include <cstdio>
#include <QObject>
#include <QTcpSocket>
#include <QHostAddress>
#include <QtDebug>
#include <QDataStream>

class TcpClient : public QObject
{
    Q_OBJECT
public:
    explicit TcpClient(QObject *parent = 0);
    ~TcpClient();

	QTcpSocket *socket;
	void setIpAddress(QString address);
    void setPort(quint16 num);
    void connectToServer();
    QByteArray rxData;

public slots:
    void onConnectServer();
    void readMessage();

private:
	QString ipAddress;
    quint16 port;
};

