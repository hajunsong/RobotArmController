#pragma once

#include <QTcpServer>
#include <QDebug>
#include <QTcpSocket>
#include <QTimer>
#include <iostream>
#include <QtMath>

class TcpServer : public QTcpServer
{
	Q_OBJECT
public:
	explicit TcpServer(QObject *parent = nullptr);
	void startServer();
    void setting(QString IP, quint16 port);
    QTcpSocket *socket;
    bool connectState;


signals:
	void error(QTcpSocket::SocketError socketerror);
    void connectedClient();

public slots:
	void readyRead();
	void disconnected();

protected:
	void incomingConnection(qintptr socketDescriptor);

private:
    QString ip;
    quint16 port;
};

