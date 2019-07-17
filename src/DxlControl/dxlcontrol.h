#pragma once

#include <iostream>
#include <QtMath>
#include "dynamixel_sdk.h"
#include <stdio.h>
#include <stdlib.h>

using namespace std;
typedef unsigned int uint;

enum { current_mode = 0, velocity_mode, position_mode = 3, extended_position_mode, current_based_position_mode, pwm_mode = 16 };

// Control table address
#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_POSITION          116
#define ADDR_GOAL_VELOCITY          104
#define ADDR_GOAL_CURRENT           102
#define ADDR_PRESENT_POSITION       132
#define ADDR_PRESENT_VELOCITY       128
#define ADDR_PRESENT_CURRENT        126
#define ADDR_PRESENT_INPUT_VOLTAGE  144
#define ADDR_OPERATING_MODE         11
#define ADDR_LED                    65
#define ADDR_REALTIME_TICK          120
#define ADDR_BAUD_RATE              8
#define ADDR_VELOCITY_LIMIT			44
#define ADDR_CURRRENT_LIMIT			38
#define ADDR_PROFILE_VELOCITY		112
#define ADDR_PROFILE_ACCELERATION	108
#define ADDR_POSITION_P_GAIN		84

// Lenght address
#define LEN_PRESENT_POSITION	4
#define LEN_PRESENT_VELOCITY	4
#define LEN_GOAL_POSITION		4

// Protocol version
#define PROTOCOL_VERSION    2.0

#define BAUDRATE    4000000
#define DEVICENAME  "COM3"

#define ADDR_BR_4M  6
#define ADDR_BR_45M 7

#define TORQUE_ENABLE   1
#define TORQUE_DISABLE  0

class DxlControl
{
public:
	DxlControl();
	~DxlControl();

	int dxl_comm_result;// = COMM_TX_FAIL;
	uint8_t dxl_error;// = 0;	// Dynamixel error
	uint32_t moving_threshold;// = 0;
	int16_t current_limit;

	void init();
	int dxl_init(uint8_t ID);
	void dxl_deinit(uint8_t ID, int32_t home_pos);

	void setLEDon(uint8_t ID);
	void setLEDoff(uint8_t ID);
	void setPosition(int32_t goal_position, uint8_t ID);
	void setVelocity(int32_t goal_velocity, uint8_t ID);
	void setOperateMode(uint8_t mode, uint8_t ID);
	void setTorqueEnable(uint8_t enable, uint8_t ID);

	int32_t getPresentPosition(uint8_t ID);
	int32_t getPresentVelocity(uint8_t ID);
	int16_t getPresentCurrent(uint8_t ID);
	double getPresentVoltage(uint8_t ID);
	void getGroupSyncReadPresentPosition(int32_t *presentPosition);
	void getGroupSyncReadPresentVelocity(int32_t *presentPosition);
	void setGroupSyncWriteGoalPosition(int32_t *goalPosition);

	dynamixel::PortHandler *portHandler;
	dynamixel::PacketHandler *packetHandler;
	dynamixel::GroupSyncRead *groupSyncReadPos;
	dynamixel::GroupSyncRead *groupSyncReadVel;

	const double POSITION_UNIT = 0.088;	// [deg]
	const double VELOCITY_UNIT = 0.229;	// [RPM]
private:
};

