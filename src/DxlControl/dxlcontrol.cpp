#include "dxlcontrol.h"

DxlControl::DxlControl()
{
	dxl_comm_result = COMM_TX_FAIL;
	dxl_error = 0;
	moving_threshold = 0;
	current_limit = 0;
}

DxlControl::~DxlControl()
{
	// Close port
	portHandler->closePort();

	// Clear syncwrite parameter storage
	groupSyncReadPos->clearParam();
	groupSyncReadVel->clearParam();

	delete groupSyncReadPos;
	delete groupSyncReadVel;
}

void DxlControl::init() {

	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	if (portHandler->openPort()) {
		printf("Succeeded to open the port!\n");
	}
	else {
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		return;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE)) {
		printf("Succeeded to change the baudrate!\n");
	}
	else {
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		return;
	}

    // Initialize Groupsyncread instance for Present Position
    groupSyncReadPos = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    groupSyncReadVel = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);

    bool dxl_addparam_result = false;	// addParam result
    // Add parameter storage for Dynamixel#1 present position value
    for (uint8_t i = 1; i <= 6; i++) {
        dxl_addparam_result = groupSyncReadPos->addParam(i);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead add param failed", i);
            return;
        }
    }
    // Add parameter storage for Dynamixel#1 present position value
    for (uint8_t i = 1; i <= 6; i++) {
        dxl_addparam_result = groupSyncReadVel->addParam(i);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead add param failed", i);
            return;
        }
    }
}

int DxlControl::dxl_init(uint8_t ID)
{
	// Check Dynamixel Torque on or off
	uint8_t torque = 0;
	dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, &torque, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS) {
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		return 0;
	}
	else if (dxl_error != 0) {
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		return 0;
	}
	else {
        printf("Dynamixel(ID : %d) has been successfully connected\n", ID);
	}

	//    cout << "torque enable : " << torque << endl;
	if (torque == TORQUE_ENABLE) {
		// Disable Dynamixel Torque
		packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	}

	// Write Dynamixel Baud rate (1M bps)
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_BAUD_RATE, BR_4M, &dxl_error);

	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED, 1, &dxl_error);

	// Write Dynamixel Operating mode
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_OPERATING_MODE, extended_position_mode, &dxl_error);

    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_PROFILE_ACCELERATION, 200, &dxl_error); // df : 0
    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_PROFILE_VELOCITY, 50, &dxl_error); // df : 0

	packetHandler->write4ByteTxRx(portHandler, ID, ADDR_VELOCITY_LIMIT, 300, &dxl_error); // df : 300

    packetHandler->write2ByteTxRx(portHandler, ID, ADDR_POSITION_P_GAIN, 800, &dxl_error); // df : 800
    // packetHandler->write2ByteTxRx(portHandler, ID, ADDR_POSITION_D_GAIN, 0, &dxl_error); // df : 0
    // packetHandler->write2ByteTxRx(portHandler, ID, ADDR_POSITION_I_GAIN, 0, &dxl_error); // df : 0

	// Enable Dynamixel Torque
	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

	return 1;
}

void DxlControl::dxl_deinit(uint8_t ID, int32_t home_pos)
{
	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED, 0, &dxl_error);
	// Reset home position
	int32_t pos = 0;
	do {
		packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, static_cast<uint32_t>(home_pos), &dxl_error);
		packetHandler->read4ByteTxRx(portHandler, ID, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&pos), &dxl_error);
		printf("Goal pos : %d, Current pos : %d\n", home_pos, pos);
	} while (abs(home_pos - pos) > 200);
	// Disable Dynamixel Torque
	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

	printf("Dynamixel has been successfully disconnected\n");
}

void DxlControl::dxl_deinit(uint8_t ID){
	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED, 0, &dxl_error);
	// Disable Dynamixel Torque
	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

	printf("Dynamixel has been successfully disconnected\n");
}

void DxlControl::setLEDon(uint8_t ID)
{
	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED, 1, &dxl_error);
}

void DxlControl::setLEDoff(uint8_t ID)
{
	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED, 0, &dxl_error);
}

void DxlControl::setPosition(int32_t goal_position, uint8_t ID)
{
    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, goal_position, &dxl_error);
}

void DxlControl::setVelocity(int32_t goal_velocity, uint8_t ID)
{
    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_VELOCITY, (uint32_t)goal_velocity, &dxl_error);
}

void DxlControl::setOperateMode(uint8_t mode, uint8_t ID)
{
	// Disable Dynamixel Torque
	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

	// Write Dynamixel Operating mode
	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_OPERATING_MODE, mode, &dxl_error);

	// Reset home position
	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
}

void DxlControl::setTorqueEnable(uint8_t enable, uint8_t ID)
{
	// Change State Dynamixel Torque
	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, enable, &dxl_error);
}

int32_t DxlControl::getPresentPosition(uint8_t ID)
{
    int32_t present_position = 0;
    packetHandler->read4ByteTxRx(portHandler, ID, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&present_position), &dxl_error);
	return present_position/* * 360.0 / RESOLUTION * M_PI/180.0*/;
}

int32_t DxlControl::getPresentVelocity(uint8_t ID)
{
	int32_t present_velocity = 0;
    packetHandler->read4ByteTxRx(portHandler, ID, ADDR_PRESENT_VELOCITY, (uint32_t*)&present_velocity, &dxl_error);
	//    return static_cast<double>(present_velocity) * 0.01*6.0 * M_PI / 180.0;
	return present_velocity;
}

int16_t DxlControl::getPresentCurrent(uint8_t ID)
{
	int16_t present_current = 0;
    packetHandler->read2ByteTxRx(portHandler, ID, ADDR_PRESENT_CURRENT, (uint16_t*)&present_current, &dxl_error);
	return present_current;// * 0.001;
}

double DxlControl::getPresentVoltage(uint8_t ID)
{
	uint16_t present_velocity = 0;
	packetHandler->read2ByteTxRx(portHandler, ID, ADDR_PRESENT_INPUT_VOLTAGE, &present_velocity, &dxl_error);
	return present_velocity * 0.1;
}

void DxlControl::getGroupSyncReadPresentPosition(int32_t *present_position)
{
    bool dxl_getdata_result = false;	// GetParam result

    // Syncread present position
    dxl_comm_result = groupSyncReadPos->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else {
        for (uint8_t i = 1; i <= 6; i++) {
            if (groupSyncReadPos->getError(i, &dxl_error)) {
                printf("[ID:%03d] %s\n", i, packetHandler->getRxPacketError(dxl_error));
            }
        }
    }

    // Check if groupsyncread data of Dynamixel#n is available
    for (uint8_t i = 1; i <= 6; i++) {
        dxl_getdata_result = groupSyncReadPos->isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", i);
            return;
        }
    }

	// Get Dynamixel#n present position value
    for (uint8_t i = 1; i <= 6; i++) {
        present_position[i-1] = static_cast<int32_t>(groupSyncReadPos->getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));
		//printf("[ID:%03d] PresPos:%03d\n", i, present_position[i]);
	}
}

void DxlControl::getGroupSyncReadPresentVelocity(int32_t *present_velocity)
{
	bool dxl_getdata_result = false;	// GetParam result

	// Syncread present position
	dxl_comm_result = groupSyncReadVel->txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else {
        for (uint8_t i = 1; i <= 6; i++) {
			if (groupSyncReadVel->getError(i, &dxl_error)) {
				printf("[ID:%03d] %s\n", i, packetHandler->getRxPacketError(dxl_error));
			}
		}
	}

	// Check if groupsyncread data of Dynamixel#n is available
    for (uint8_t i = 1; i <= 6; i++) {
		dxl_getdata_result = groupSyncReadVel->isAvailable(i, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
		if (dxl_getdata_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncRead get data failed", i);
			return;
		}
	}

	// Get Dynamixel#n present position value
    for (uint8_t i = 0; i < 6; i++) {
        present_velocity[i] = static_cast<int32_t>(groupSyncReadVel->getData(i, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY));
		//printf("[ID:%03d] PresPos:%03d\n", i, present_position[i]);
	}
}

void DxlControl::setGroupSyncWriteGoalPosition(int32_t *goalPosition)
{
	bool dxl_addparam_result = false;	// addParam result
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
	
    for (uint8_t i = 0; i < 6; i++) {
		uint8_t param_goal_position[4];
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goalPosition[i]));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goalPosition[i]));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goalPosition[i]));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goalPosition[i]));

		// Add Dynamixel#n goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(i+1, param_goal_position);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncWrite add param failed", i);
			return;
		}
	}

	// Syncwrite goal position
	dxl_comm_result = groupSyncWrite.txPacket();
	if (dxl_comm_result != COMM_SUCCESS)
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear syncwrite parameter storage
	groupSyncWrite.clearParam();
}
