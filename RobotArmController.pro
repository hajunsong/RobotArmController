QT += core network
QT -= gui

TARGET = RobotArmController
CONFIG += console c++11
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    src/FileIO/fileio.cpp \
    src/Numerical/numerical.cpp \
    src/RobotArm/robotarm.cpp \
    src/TcpSocket/tcpserver.cpp \
    src/DxlControl/dxlcontrol.cpp

HEADERS += include/variables.h \
    src/FileIO/fileio.h \
    src/Numerical/numerical.h \
    src/RobotArm/robotarm.h \
    src/TcpSocket/tcpserver.h \
    src/DxlControl/dxlcontrol.h

INCLUDEPATH += $$PWD/include\
                $$PWD/../../dev/lib/NRMK/xenomai/include\
                $$PWD/../../dev/lib/NRMK\
                $$PWD/../../dev/lib/NRMK/core/3rdparty/Poco/lib/i686\
                $$PWD/../../dev/lib/tp_gpio\
                $$PWD/src/Numerical\
                $$PWD/src/RobotArm\
                $$PWD/src/TcpSocket\
                $$PWD/src/FileIO\
                $$PWD/include/dynamixel_sdk\

DEPENDPATH += $$PWD/include\
                $$PWD/../../dev/lib/NRMK/xenomai/include\
                $$PWD/../../dev/lib/NRMK\
                $$PWD/../../dev/lib/NRMK/core/3rdparty/Poco/lib/i686\

unix:!macx: LIBS += -L$$PWD/../../dev/lib/NRMK/xenomai/lib/ -lnative\
                -L$$PWD/../../dev/lib/NRMK/xenomai/lib/ -lxenomai\
                -L$$PWD/../../dev/lib/NRMK/xenomai/lib/ -lrtdm\
                -L$$PWD/../../dev/lib/NRMK/lib/ -lNRMKHelperi686\
                -L$$PWD/../../dev/lib/NRMK/core/3rdparty/Poco/lib/i686/ -lPocoFoundation\
                -L$$PWD/../../dev/lib/NRMK/core/3rdparty/Poco/lib/i686/ -lPocoNet

unix:!macx: PRE_TARGETDEPS += $$PWD/../../dev/lib/NRMK/lib/libNRMKHelperi686.a

