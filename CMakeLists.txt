cmake_minimum_required(VERSION 2.8.12)
project(RobotArmController)
add_compile_options(-std=c++11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_PREFIX_PATH /home/user/Qt5.5.1/5.5/gcc)

find_package(Qt5 REQUIRED Core Network)

include_directories(
    include/
    src/DxlControl/
    src/FileIO/
    src/Numerical/
    src/RobotArm/
    src/TcpSocket/
    ~/dev/lib/NRMK/xenomai/include/
    ~/dev/lib/NRMK/
    ~/dev/lib/NRMK/core/3rdparty/Poco/lib/i686/
    ~/dev/lib/tp_gpio/
)

link_directories(
    lib/
    ~/dev/lib/NRMK/xenomai/lib/
    ~/dev/lib/NRMK/lib/
    ~/dev/lib/NRMK/core/3rdparty/Poco/lib/i686/
)

set(QT_LIBS Qt5::Core Qt5::Network)
set(DXL_LIB -ldxl_x86_cpp)
set(STEP_LIBS -lnative -lxenomai -lrtdm -lNRMKHelperi686 -lPocoFoundation -lPocoNet)

# file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
# file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp *.h)

# QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
# QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

# add_executable(${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_executable(${PROJECT_NAME} ${QT_SOURCES} ${QT_MOC_HPP})

target_link_libraries(${PROJECT_NAME} ${QT_LIBS} ${DXL_LIB} ${STEP_LIBS})
