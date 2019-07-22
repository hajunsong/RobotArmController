#include <QCoreApplication>
#include "robotcontrol.h"

int main(int argc, char* argv[])
{
    QCoreApplication a(argc, argv);

    RobotControl robotControl;

    return a.exec();
}


