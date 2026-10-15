/***************************************************************************
 *   Copyright (C) 2018 by Andres Montano                                  *
 *   andres.felipe.montano@upc.edu                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 ***************************************************************************/

#include <allegro_hand/CAN_AH.h>

#include <iostream>
#include <cmath>

#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <ctime>

#include <thread>
#include <PCANBasic.h>
#define _T(X)   X


enum eJointName {
	eJOINTNAME_INDEX_0,
	eJOINTNAME_INDEX_1,
	eJOINTNAME_INDEX_2,
	eJOINTNAME_INDEX_3,
	eJOINTNAME_MIDDLE_0,
	eJOINTNAME_MIDDLE_1,
	eJOINTNAME_MIDDLE_2,
	eJOINTNAME_MIDDLE_3,
	eJOINTNAME_PINKY_0,
	eJOINTNAME_PINKY_1,
	eJOINTNAME_PINKY_2,
	eJOINTNAME_PINKY_3,
	eJOINTNAME_THUMB_0,
	eJOINTNAME_THUMB_1,
	eJOINTNAME_THUMB_2,
	eJOINTNAME_THUMB_3,
	DOF_JOINTS
};

//structures
typedef struct __attribute__((packed))
{
    unsigned short position;
    unsigned short imu;
    unsigned short temp;
} can_period_msg_t;

CAN_AH::CAN_AH(const std::string &channel, const Parameters &params) :
	run_commLoop(true) {

		hand_version = params.handVersion;

		if (hand_version <= 3.0){
			versionHandler = std::make_unique<Version3Handler>();
			initV3(params);
		}
		else{
			versionHandler = std::make_unique<Version4Handler>();
		}
		// Get Can channel
		// TODO use input chanel (check getCANChannelIndex and expand it)
		channelToBus(channel, bus);
		//bus = PCAN_USBBUS1;
		versionHandler->bus = bus;
		int CAN_Ch = 0;
		CAN_Ch = GetCANChannelIndex(std::to_string(bus).c_str());

		int rets = 0;
		TPCANMsg lmsg;  
		TPCANTimestamp timeStamp;
		// Open CAN channel communications
		std::cout << "CAN: Opening Channel" << std::endl;
		assert(CAN_Ch >= 0 && CAN_Ch < MAX_BUS);
		rets = initCAN(bus); // Initialize CAN
		if (rets < 0) {
			std::cout << "CAN: Error in CAN_Open() on Channel " << CAN_Ch << std::endl;
			throw std::runtime_error("Error openn CAN.");
		} else {
			std::cout << "CAN: Success Opening Channel: Ch " << CAN_Ch << std::endl;
		}
		startCAN(CAN_Ch);
		std::cout << "CAN: Clearing the CAN buffer" << std::endl;
		for (unsigned int i = 0; i < 3000; ++i) {
			CAN_Read(bus, &lmsg, &timeStamp);
		}
		// Get ID first finger.
		versionHandler->ID_MSG_FIRST_FINGER = getIDFirstFinger();
		std::cout << "Id first finger:" << versionHandler->ID_MSG_FIRST_FINGER << std::endl;

}

CAN_AH::~CAN_AH() {
    std::cout << "CAN: Stopping System" << std::endl;
	int ret;
	ret = versionHandler->command_can_stop(bus);
	if(ret < 0)
	{
		std::cout << "ERROR command_can_stop !!!" << std::endl;
	}
	usleep(10000);
	std::cout << "Closing CAN" << std::endl;
	ret = versionHandler->command_can_close(bus);
	if(ret < 0)
	{
		std::cout << "ERROR command_can_close !!!" << std::endl;
	}
}

void CAN_AH::channelToBus(std::string channel, TPCANHandle &bus){
	std::cout << "Selected channel:::::::::::::: " << channel << std::endl;
	if(channel == "32"){bus = PCAN_USBBUS1;}
	else if (channel == "33"){bus = PCAN_USBBUS2;}
	else if (channel == "34"){bus = PCAN_USBBUS3;}
	else if (channel == "35"){bus = PCAN_USBBUS4;}
	else if (channel == "36"){bus = PCAN_USBBUS5;}
	else if (channel == "37"){bus = PCAN_USBBUS6;}
	else if (channel == "38"){bus = PCAN_USBBUS7;}
	else if (channel == "39"){bus = PCAN_USBBUS8;}
	else{
		std::cout << "Unknown channel. Setting default PCAN_USBBUS1" << std::endl;
		bus = PCAN_USBBUS1;
	}
}

void CAN_AH::startCAN(int CAN_Ch) {
    versionHandler->startCAN(CAN_Ch);
	std::cout << "Starting CAN ..." << std::endl;
}

void CAN_AH::Version3Handler::startCAN(int CAN_Ch){
	// hand_version = this->getHandVersion();
	if (hand_version == 3.0) {
		tau_cov_const = 1200.0;
	} else {
		tau_cov_const = 800.0;
	}
	bool done = true;
	char id_des;
	char id_cmd;
	char id_src;
	int len;

	// v3
	//  Define CAN Command
	ID_CMD_SET_SYSTEM_ON = 0x01;
	ID_CMD_SET_SYSTEM_OFF= 0x02;
	ID_CMD_SET_PERIOD    = 0x03;
	ID_CMD_SET_TORQUE_1  = 0x06;
	ID_CMD_SET_TORQUE_2  = 0x07;
	ID_CMD_SET_TORQUE_3  = 0x08;
	ID_CMD_SET_TORQUE_4  = 0x09;
	ID_DEVICE_SUB_01     = 0x03;
	ID_DEVICE_SUB_02     = 0x04;
	ID_DEVICE_SUB_03     = 0x05;
	ID_DEVICE_SUB_04     = 0x06;

	// Hand information
	Txid = ((unsigned long)ID_CMD_QUERY_ID<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	ret = canSendMsg(bus, Txid, 0, data);
	if(ret < 0)
	{
			std::cout << "ERROR requesting hand information !!!" << std::endl;
	}
	if(hand_version == 3.0){
		while(done){
			ret = get_message(bus, &id_cmd, &id_src, &id_des, &len, data);
			if(ret && (id_cmd == ID_CMD_QUERY_ID))
			{
				printf(">CAN(%d): AllegroHand revision info: 0x%02x%02x\n", CAN_Ch, data[3], data[2]);
				printf("                      firmware info: 0x%02x%02x\n", data[5], data[4]);
				printf("                      hardware type: 0x%02x\n", data[7]);
				done = false;
			}
		}
	}
	// Init and setting periodic communication parameters(period)
	std::cout << "CAN: Setting communications period to 3ms" << std::endl;
	ret = command_set_period(bus, 3); // 3ms
	if(ret < 0)
	{
			std::cout << "ERROR setting the period !!!" << std::endl;
			command_can_close(bus);
	}
	// Start periodic communication
	ret = command_can_start_v3(bus);
	if(ret < 0)
	{
			std::cout << "ERROR starting communication !!!" << std::endl;
			command_can_stop(bus);
			command_can_close(bus);
	}
}
void CAN_AH::Version4Handler::startCAN(int CAN_Ch){
	tau_cov_const = 1200.0;
	CAN_ID = 0;
	//  Define CAN Command
	// v4
	ID_CMD_SET_SYSTEM_ON  =   0x40;
	ID_CMD_SET_SYSTEM_OFF =   0x41;
	ID_CMD_SET_PERIOD     =   0x81;
	ID_CMD_SET_TORQUE     =   0x60;
	ID_CMD_SET_TORQUE_1   =  (ID_CMD_SET_TORQUE+0);
	ID_CMD_SET_TORQUE_2   =  (ID_CMD_SET_TORQUE+1);
	ID_CMD_SET_TORQUE_3   =  (ID_CMD_SET_TORQUE+2);
	ID_CMD_SET_TORQUE_4   =  (ID_CMD_SET_TORQUE+3);
	ID_DEVICE_SUB_01      =  (ID_RTR_FINGER_POSE+0);
	ID_DEVICE_SUB_02      =  (ID_RTR_FINGER_POSE+1);
	ID_DEVICE_SUB_03      =  (ID_RTR_FINGER_POSE+2);
	ID_DEVICE_SUB_04      =  (ID_RTR_FINGER_POSE+3);

	bool done = true;
	int id;
	int len;
	// Request hand information.
	Txid = ID_RTR_HAND_INFO;
	ret = canSentRTR(bus, Txid);
	if(ret < 0)
	{
		std::cout << "ERROR requesting hand information !!!" << std::endl;
		command_can_close(bus);
	}
	while(done){
		ret = get_message(bus, &id, &len, data);
		if(ret && (id == ID_RTR_HAND_INFO))
		{
			printf(">CAN(%d): AllegroHand hardware version: 0x%02x%02x\n", CAN_Ch, data[1], data[0]);
			printf("                      firmware version: 0x%02x%02x\n", data[3], data[2]);
			printf("                      hardware type: %d(%s)\n", data[4], (data[4] == 0 ? "right" : "left"));
			printf("                      temperature: %d (celsius)\n", data[5]);
			printf("                      status: 0x%02x\n", data[6]);
			printf("                      servo status: %s\n", (data[6] & 0x01 ? "ON" : "OFF"));
			printf("                      high temperature fault: %s\n", (data[6] & 0x02 ? "ON" : "OFF"));
			printf("                      internal communication fault: %s\n", (data[6] & 0x04 ? "ON" : "OFF"));
			done = false;
		}
	}
	done = true;
	Txid = ID_RTR_SERIAL;
	ret = canSentRTR(bus, Txid);
	if(ret < 0)
	{
		std::cout << "ERROR requesting hand serial !!!" << std::endl;
		command_can_close(bus);
	}
	while(done){
		ret = get_message(bus, &id, &len, data);
		if(ret && (id == ID_RTR_SERIAL))
		{
			printf(">CAN(%d): AllegroHand serial number: %c%c%c%c%c%c%c%c\n", CAN_Ch,
				data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
			done = false;
		}
	}
	// set periodic communication parameters(period)
	std::cout << "CAN: Setting communications period to 3ms" << std::endl;
	short comm_period[3] = {3, 0, 0}; // millisecond {position, imu, temperature}
	ret = command_set_period(bus, comm_period);
	if(ret < 0)
	{
		std::cout << "ERROR setting the period !!!" << std::endl;
		command_can_close(bus);
	}
	// servo on
	std::cout << ">CAN: servo on\n" << std::endl;
	Txid = ID_CMD_SYSTEM_ON;
	ret = canSendMsg(bus, Txid, 0, data);
	if(ret < 0)
	{
		std::cout << "ERROR command_servo_on !!!" << std::endl;
		command_set_period(bus, 0);
		command_can_close(bus);
	}
}
// Function to initialize the CAN communications
int CAN_AH::initCAN(int bus){
    TPCANStatus Status = PCAN_ERROR_OK;
    char strMsg[256];
    TPCANBaudrate Baudrate = PCAN_BAUD_1M;
    TPCANType HwType = 0;
    DWORD IOPort = 0;
    WORD Interrupt = 0;

    Status = CAN_Initialize(bus, Baudrate, HwType, IOPort, Interrupt);
    if (Status != PCAN_ERROR_OK)
    {
        CAN_GetErrorText(Status, 0, strMsg);
        std::cout << "initCAN(): CAN_Initialize() failed with error " << Status << std::endl;
        std::cout << strMsg << std::endl;
        return Status;
    }

    Status = CAN_Reset(bus);
    if (Status != PCAN_ERROR_OK)
    {
        CAN_GetErrorText(Status, 0, strMsg);
				std::cout << "initCAN(): CAN_Reset() failed with error " << Status << std::endl;
        std::cout << strMsg << std::endl;
        return Status;
    }
    return 0; // PCAN_ERROR_OK
}

// V4 send RTR message
int CAN_AH::Version4Handler::canSentRTR(int bus, int id){
    TPCANMsg CANMsg;
    TPCANStatus Status = PCAN_ERROR_OK;
    char strMsg[256];
    CANMsg.ID = (id << 2) | CAN_ID;
    CANMsg.LEN = 0;
    CANMsg.MSGTYPE = PCAN_MESSAGE_RTR; // Remote Transmission Request
    Status = CAN_Write(bus, &CANMsg);
    if (Status != PCAN_ERROR_OK)
    {
        CAN_GetErrorText(Status, 0, strMsg);
				std::cout << "canSendRTR(): CAN_Write() failed with error " << Status << std::endl;
        std::cout << strMsg << std::endl;
        return Status;
    }

    return 0; //PCAN_ERROR_OK;
}

// Close Can
int CAN_AH::VersionHandler::command_can_close(int bus){
	TPCANStatus Status = PCAN_ERROR_OK;
	char strMsg[256];
	std::cout << "CAN: Close..." << std::endl;
	Status = CAN_Uninitialize(bus);
	if (Status != PCAN_ERROR_OK)
	{
		CAN_GetErrorText(Status, 0, strMsg);
		std::cout << "command_can_close(): CAN_Uninitialize() failed with error " << Status << std::endl;
		std::cout << strMsg << std::endl;
		return Status;
	}
	return 0; //PCAN_ERROR_OK;
}

// Setting period v4
int CAN_AH::Version4Handler::command_set_period(int bus, short* period){
    can_period_msg_t msg;
     Txid = ID_CMD_SET_PERIOD;
    if (period != 0)
    {
        msg.position = period[0];
        msg.imu = period[1];
        msg.temp = period[2];
    }
    else
    {
        msg.position = 0;
        msg.imu = 0;
        msg.temp = 0;
    }
    ret = canSendMsg(bus, Txid, 6, (unsigned char *)&msg);

    return ret;
}
// Period v3
int CAN_AH::Version3Handler::command_set_period(int ch, int period_msec){
	Txid = ((unsigned long)ID_CMD_SET_PERIOD<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	data[0] = (unsigned char)period_msec;
	ret = canSendMsg(ch, Txid, 1, data);

	usleep(10);

	Txid = ((unsigned long)ID_CMD_SET_MODE_TASK<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	ret = canSendMsg(ch, Txid, 0, data);

	usleep(10);

	Txid = ((unsigned long)ID_CMD_QUERY_STATE_DATA<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	ret = canSendMsg(ch, Txid, 0, data);

	return 0; //PCAN_ERROR_OK;
}

// Can initialize comunications v3
int CAN_AH::Version3Handler::command_can_start_v3(int ch){
	Txid = ((unsigned long)ID_CMD_QUERY_STATE_DATA<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	ret = canSendMsg(ch, Txid, 0, data);

	usleep(10);

	Txid = ((unsigned long)ID_CMD_SET_SYSTEM_ON<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	ret = canSendMsg(ch, Txid, 0, data);

	return 0; //PCAN_ERROR_OK;
}

// Stop communications v4 set period to 0
int CAN_AH::Version4Handler::command_can_stop(int bus){
	command_set_period(bus, 0);
	return 0; //PCAN_ERROR_OK;
}

// Stop communications v3 (not the same as command_can_close)
int CAN_AH::Version3Handler::command_can_stop(int bus){
	long Txid;
	unsigned char data[8];

	Txid = ((unsigned long)ID_CMD_SET_SYSTEM_OFF<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	canSendMsg(bus, Txid, 0, data);

	return 0; //PCAN_ERROR_OK;
}

int CAN_AH::Version4Handler::canSendMsg(int bus, int id, char len, unsigned char *data){
	TPCANMsg CANMsg;
	TPCANStatus Status = PCAN_ERROR_OK;
	char strMsg[256];
	int i;

	CANMsg.ID = (id << 2) | CAN_ID;
	CANMsg.LEN = len & 0x0F;

	for(i = 0; i < len; i++)
        CANMsg.DATA[i] = data[i];
	CANMsg.MSGTYPE = PCAN_MESSAGE_STANDARD;
	Status = CAN_Write(bus, &CANMsg);
	if (Status != PCAN_ERROR_OK)
	{
		CAN_GetErrorText(Status, 0, strMsg);
		std::cout << "canSendMsg(): CAN_Write() failed with error " << Status << std::endl;
		std::cout << strMsg << std::endl;
		return Status;
	}
	return 0; //PCAN_ERROR_OK;
}

int CAN_AH::Version3Handler::canSendMsg(int bus, int id, char len, unsigned char *data){
	TPCANMsg CANMsg;
	TPCANStatus Status = PCAN_ERROR_OK;
	char strMsg[256];
	int i;

	CANMsg.ID = id;
	CANMsg.LEN = len;

	for(i = 0; i < len; i++)
        CANMsg.DATA[i] = data[i];
	CANMsg.MSGTYPE = PCAN_MESSAGE_STANDARD;
	Status = CAN_Write(bus, &CANMsg);
	if (Status != PCAN_ERROR_OK)
	{
		CAN_GetErrorText(Status, 0, strMsg);
		std::cout << "canSendMsg(): CAN_Write() failed with error " << Status << std::endl;
		std::cout << strMsg << std::endl;
		return Status;
	}
	return 0; //PCAN_ERROR_OK;
}

// Get message v4
int CAN_AH::Version4Handler::get_message(int ch, int* id, int* len, unsigned char* data){
    int err;
    err = canReadMsg(ch, id, len, data);
    return err;
}

// Get message v3
int CAN_AH::Version3Handler::get_message(int ch, char* cmd, char* src, char* des, int* len, unsigned char* data){
	int err;
	unsigned long Rxid;

	err = canReadMsg(ch, (int*)&Rxid, len, data);
	if (!err)
	{
		*cmd = (char)( (Rxid >> 6) & 0x1f );
		*des = (char)( (Rxid >> 3) & 0x07 );
		*src = (char)( Rxid & 0x07);
	}
	else
	{
		return err;
	}
	return 0;
}

int CAN_AH::Version4Handler::canReadMsg(int bus, int *id, int *len, unsigned char *data){
    TPCANMsg CANMsg;
    TPCANTimestamp CANTimeStamp;
    TPCANStatus Status = PCAN_ERROR_OK;
    char strMsg[256];
    int i;
    // We execute the "Read" function of the PCANBasic
    Status = CAN_Read(bus, &CANMsg, &CANTimeStamp);
    if (Status != PCAN_ERROR_OK){
        if (Status != PCAN_ERROR_QRCVEMPTY)
        {
            CAN_GetErrorText(Status, 0, strMsg);
			std::cout << "canReadMsg(): CAN_Read() failed with error " << Status << std::endl;
			std::cout << strMsg << std::endl;
        }
        return Status;
    }
    *id = (CANMsg.ID & 0xfffffffc) >> 2;
    *len = CANMsg.LEN;
    for(i = 0; i < CANMsg.LEN; i++)
        data[i] = CANMsg.DATA[i];
    return 0;
}
int CAN_AH::Version3Handler::canReadMsg(int bus, int *id, int *len, unsigned char *data){
    TPCANMsg CANMsg;
    TPCANTimestamp CANTimeStamp;
    TPCANStatus Status = PCAN_ERROR_OK;
    char strMsg[256];
    int i;
    // We execute the "Read" function of the PCANBasic
    Status = CAN_Read(bus, &CANMsg, &CANTimeStamp);
    if (Status != PCAN_ERROR_OK)
    {
        if (Status != PCAN_ERROR_QRCVEMPTY)
        {
            CAN_GetErrorText(Status, 0, strMsg);
			std::cout << "canReadMsg(): CAN_Read() failed with error " << Status << std::endl;
			std::cout << strMsg << std::endl;
        }
        return Status;
    }
	*id = CANMsg.ID;
    *len = CANMsg.LEN;
    for(i = 0; i < CANMsg.LEN; i++)
        data[i] = CANMsg.DATA[i];
    return 0;
}

// New parse message Function

char CAN_AH::Version4Handler::parseCANMessage(TPCANMsg CANMsg, double *values){

	cmd = (CANMsg.ID & 0xfffffffc) >> 2;
	len = CANMsg.LEN;
	for(unsigned int i = 0; i < CANMsg.LEN; i++)
		tmpdata[i] = CANMsg.DATA[i];

    switch (cmd){
		// Version 4
		case ID_RTR_FINGER_POSE_1:
        case ID_RTR_FINGER_POSE_2:
        case ID_RTR_FINGER_POSE_3:
        case ID_RTR_FINGER_POSE_4:
        {
			tmppos[0] = (short)(tmpdata[0] | (tmpdata[1] << 8));
			tmppos[1] = (short)(tmpdata[2] | (tmpdata[3] << 8));
			tmppos[2] = (short)(tmpdata[4] | (tmpdata[5] << 8));
			tmppos[3] = (short)(tmpdata[6] | (tmpdata[7] << 8));
	        // The output position in V4 ranges from 0 to 2pi. And in V3 from -pi to pi.
			for (int i=0; i<4; i++){
				values[i] = (double)tmppos[i] * (333.3/65536.) * (M_PI/180.);
			}
			return cmd;
			break;
        }

        default:
            std::cout << "CAN: Unknown command " << (int)cmd << " " << (int)src
                      << " " << (int)len << " " << std::endl;
        break;
    }

    return 0;
}
char CAN_AH::Version3Handler::parseCANMessage(TPCANMsg CANMsg, double *values){

	cmd = (char)((CANMsg.ID >> 6) & 0x1f);
	src = (char)( CANMsg.ID       & 0x07);

	len = CANMsg.LEN;
	for(unsigned int i = 0; i < CANMsg.LEN; i++)
	tmpdata[i] = CANMsg.DATA[i];

    switch (cmd){
			// Version 3
        case ID_CMD_QUERY_CONTROL_DATA:

			// if (src >= ID_DEVICE_SUB_01 && src <= ID_DEVICE_SUB_04) {
			tmppos[0] = (int)(tmpdata[0] | (tmpdata[1] << 8));
			tmppos[1] = (int)(tmpdata[2] | (tmpdata[3] << 8));
			tmppos[2] = (int)(tmpdata[4] | (tmpdata[5] << 8));
			tmppos[3] = (int)(tmpdata[6] | (tmpdata[7] << 8));
			
			lIndexBase = 4*(src-ID_DEVICE_SUB_01);
			values[0] = ((double)encoderDirection[lIndexBase+0] *(double)tmppos[0] - 32768.0
			- (double)encoderOffset[lIndexBase+0]) * (333.3/65536.) * (M_PI/180.);
			values[1] = ((double)encoderDirection[lIndexBase+1] *(double)tmppos[1] - 32768.0
			- (double)encoderOffset[lIndexBase+1]) * (333.3/65536.) * (M_PI/180.);
			values[2] = ((double)encoderDirection[lIndexBase+2] *(double)tmppos[2] - 32768.0
			- (double)encoderOffset[lIndexBase+2]) * (333.3/65536.) * (M_PI/180.);
			values[3] = ((double)encoderDirection[lIndexBase+3] *(double)tmppos[3] - 32768.0
			- (double)encoderOffset[lIndexBase+3]) * (333.3/65536.) * (M_PI/180.);
			// std::cout << "Dit "<< lIndexBase <<". Data: " << (double)tmppos[0] << ". value: " << values[0]<<std::endl;
			return src;
		break;

        default:
            std::cout << "CAN: Unknown command " << (int)cmd << " " << (int)src
                      << " " << (int)len << " " << std::endl;
        break;
    }

    return 0;
}

void CAN_AH::setTorque(const double *torque_) {
    std::copy(torque_,torque_+DOF_JOINTS,versionHandler->torque);
}

void CAN_AH::getPosition(double *position_) {
		mutex.lock();
			std::copy(versionHandler->position,versionHandler->position+DOF_JOINTS,position_);
		mutex.unlock();
}

void CAN_AH::getTimeStamp(TPCANTimestamp &timeStamp_) {
	mutex.lock();
		timeStamp_ = versionHandler->timeStamp;
	mutex.unlock();
}

// Obtain automatically the ID of the first finger so the readings are the ones made at the same time
DWORD CAN_AH::getIDFirstFinger(){
	TPCANMsg CANMsg;
	TPCANTimestamp CANTimeStamp;
	bool idFirstFinger = true;
	TPCANStatus Status = PCAN_ERROR_OK;
	char strMsg[256];
	int ite;
	TPCANMsg lmsgs[4];  //All hand msgs with timestamp
	TPCANStatus rets[4];
  	bool notmsg = true;
	DWORD ID_MSG_FIRST_FINGER;
	while(idFirstFinger){
		for (ite=0; ite<4; ite++){
		 	// Read the message
			while(notmsg){
				Status = CAN_Read(bus, &CANMsg, &CANTimeStamp);
				if (Status == PCAN_ERROR_OK){
					notmsg = false;
					if((ite!=0))
					{
						for(int ii=0; ii<ite; ii++){
							if(lmsgs[ite].ID == lmsgs[ii].ID){
								notmsg = true;
							}
						}
					}
				}
			}
			rets[ite] = Status;
			lmsgs[ite] = CANMsg;
			versionHandler->timeStamp = CANTimeStamp;
			notmsg = true;
		}
		if ((rets[0]!= PCAN_ERROR_OK) || (rets[1]!= PCAN_ERROR_OK) || (rets[2]!= PCAN_ERROR_OK) || (rets[3]!= PCAN_ERROR_OK)) {
			std::cout << "CAN: readDevices, LINUX_CAN_Read failed" << std::endl;
			for (int i = 0; i < 4; ++i) {
				if(rets[i]!= PCAN_ERROR_OK){
					std::cout << "Finger that failed: " << i << std::endl;
					if (rets[i] != PCAN_ERROR_QRCVEMPTY)
					{
						CAN_GetErrorText(rets[i], 0, strMsg);
						std::cout << "canReadMsg(): CAN_Read() failed with error " << rets[i] << std::endl;
						std::cout << strMsg << std::endl;
					}
					else{
						std::cout << "Queue empty: no messages available to be read" << rets[i] << std::endl;
						std::cout << strMsg << std::endl;
					}
				}
			}
		}
		else{
			ID_MSG_FIRST_FINGER = lmsgs[0].ID;
			for (ite=0; ite<4; ite++){
				//std::cout << lmsgs[ite].ID << std::endl;
				if(lmsgs[ite].ID < ID_MSG_FIRST_FINGER){
				ID_MSG_FIRST_FINGER = lmsgs[ite].ID;

				}
			}
			idFirstFinger = false;
		}
	}
	return ID_MSG_FIRST_FINGER;
}

// Read the timestamp
void CAN_AH::readDataTimeStamp(TPCANTimestamp &timeStamp_A){ //, DWORD &timeStamp_B, DWORD &timeStamp_C, DWORD &timeStamp_D){
	Status = PCAN_ERROR_OK;
    notmsg = true;
	// Obtain the timestamp of the new data
	for (int ite=0; ite<4; ite++){
		// Read the message with a timestamp
		while(notmsg){
			Status = CAN_Read(bus, &CANMsg, &CANTimeStamp);
			if (Status == PCAN_ERROR_OK){
				notmsg = false;
				if((ite!=0)){
					for(int ii=0; ii<ite; ii++){
						if(CANMsgs[ite].ID == CANMsgs[ii].ID){
							notmsg = true;
						}
					}
				}
			}
		}
		rets[ite] = Status;
		CANMsgs[ite] = CANMsg;
		versionHandler->lmsgs[ite] = CANMsgs[ite];
		notmsg = true;
		// Check if the first read message corresponds to the first finger.
		if(ite == 0 && CANMsgs[ite].ID != versionHandler->ID_MSG_FIRST_FINGER){
			ite = -1; // Restart the readings
		}
	}
	 
	if ((rets[0]!= PCAN_ERROR_OK) || (rets[1]!= PCAN_ERROR_OK) || (rets[2]!= PCAN_ERROR_OK) || (rets[3]!= PCAN_ERROR_OK)) {
		for (int i = 0; i < 4; ++i) {
			if(rets[i]!= PCAN_ERROR_OK){
				std::cout << "Finger that failed: " << i << std::endl;
				if (rets[i] != PCAN_ERROR_QRCVEMPTY){
					CAN_GetErrorText(rets[i], 0, strMsg);
					std::cout << "canReadMsg(): CAN_Read() failed with error " << rets[i] << std::endl;
					std::cout << strMsg << std::endl;
				}
			}
		}
		timeStamp_A.micros = 0;
		timeStamp_A.millis = 0;
		timeStamp_A.millis_overflow = 0;
	}
	// Parse the msgs to obtain the position
	else{
		mutex.lock();
			versionHandler->timeStamp = CANTimeStamp;
		mutex.unlock();
		timeStamp_A.micros = CANTimeStamp.micros;
		timeStamp_A.millis = CANTimeStamp.millis;
		timeStamp_A.millis_overflow = CANTimeStamp.millis_overflow;
	}
}


void CAN_AH::readDataTimeStamp(){ //, DWORD &timeStamp_B, DWORD &timeStamp_C, DWORD &timeStamp_D){
	Status = PCAN_ERROR_OK;
    notmsg = true;
	// Obtain the timestamp of the new data
	for (int ite=0; ite<4; ite++){
		// Read the message with a timestamp
		while(notmsg){
			Status = CAN_Read(bus, &CANMsg, &CANTimeStamp);
			if (Status == PCAN_ERROR_OK){
				notmsg = false;
				if((ite!=0)){
					for(int ii=0; ii<ite; ii++){
						if(CANMsgs[ite].ID == CANMsgs[ii].ID){
							notmsg = true;
						}
					}
				}
			}
		}
		rets[ite] = Status;
		CANMsgs[ite] = CANMsg;
		versionHandler->lmsgs[ite] = CANMsgs[ite];
		notmsg = true;
		// Check if the first read message corresponds to the first finger.
		if(ite == 0 && CANMsgs[ite].ID != versionHandler->ID_MSG_FIRST_FINGER){
			ite = -1; // Restart the readings
		}
	}
	 
	if ((rets[0]!= PCAN_ERROR_OK) || (rets[1]!= PCAN_ERROR_OK) || (rets[2]!= PCAN_ERROR_OK) || (rets[3]!= PCAN_ERROR_OK)) {
		for (int i = 0; i < 4; ++i) {
			if(rets[i]!= PCAN_ERROR_OK){
				std::cout << "Finger that failed: " << i << std::endl;
				if (rets[i] != PCAN_ERROR_QRCVEMPTY){
					CAN_GetErrorText(rets[i], 0, strMsg);
					std::cout << "canReadMsg(): CAN_Read() failed with error " << rets[i] << std::endl;
					std::cout << strMsg << std::endl;
				}
			}
		}
	}
	// Parse the msgs to obtain the position
	else{
		mutex.lock();
			versionHandler->timeStamp = CANTimeStamp;
		mutex.unlock();
	}
}

void CAN_AH::readDevices() {
    versionHandler->readDevices();
}

// Reads all the msgs and parses them
void CAN_AH::Version4Handler:: readDevices() {
	good_msgs_counter = 0;
	// parse all the msgs and check if the result is good.
	for (ite=0; ite<4; ite++){
		lIDs[ite] = parseCANMessage(lmsgs[ite],qs[ite]);
		if ((lIDs[ite]  >= ID_DEVICE_SUB_01) && (lIDs[ite]  <= ID_DEVICE_SUB_04)) {
			good_msgs_counter ++;
		} else{
			std::cout << "CAN: readDevices, lID " << lIDs[ite]  << std::endl;
			emergencyStop = true;
			break;
		}
	}
	// Store the data of the 4 msgs in the position variable. (We have divide the parse and the store so we store all the joint positions at the same time)
	if(good_msgs_counter == 4){
		// Store in the global variables the timestamp and data at the same time.
		mutex.lock();
			// Iterate through the fingers
			for (ite=0; ite<4; ite++){
				int findex = (lIDs[ite] & 0x00000007);
				// Iterate through the finger joints
				for (unsigned int i = 0; i < 4; ++i) {
					position[i+4*findex] = qs[ite][i];
					//std::cout << "Can pos " << i+4*findex << ": " << position[i+4*findex] << ", ";
				}
				//std::cout << std::endl;
			}
			//std::cout << std::endl;
		mutex.unlock();
	}
}
void CAN_AH::Version3Handler:: readDevices() {
	good_msgs_counter = 0;
	// parse all the msgs and check if the result is good.
	for (ite=0; ite<4; ite++){
		lIDs[ite] = parseCANMessage(lmsgs[ite],qs[ite]);
		if ((lIDs[ite]  >= ID_DEVICE_SUB_01) && (lIDs[ite]  <= ID_DEVICE_SUB_04)) {
			good_msgs_counter ++;
		} else{
			std::cout << "CAN: readDevices, lID " << lIDs[ite]  << std::endl;
			emergencyStop = true;
			break;
		}
	}

	// Store the data of the 4 msgs in the position variable. (We have divide the parse and the store so we store all the joint positions at the same time)
	if(good_msgs_counter == 4){
		// Store in the global variables the timestamp and data at the same time.
		mutex.lock();
			// Iterate through the fingers
			for (ite=0; ite<4; ite++){
				// Iterate through the finger joints
				for (unsigned int i = 0; i < 4; ++i) {
					position[i+4*(lIDs[ite]-ID_DEVICE_SUB_01)] = qs[ite][i];
				}
			}
		mutex.unlock();
	}
}

void CAN_AH::writeDevices() {
    versionHandler->writeDevices();
}

void CAN_AH::Version4Handler::writeDevices(){

	double cur_des[DOF_JOINTS];
	short pwm[DOF_JOINTS];

	for (int i=0; i<DOF_JOINTS; i++)
	{
		cur_des[i] = torque[i];
		if (cur_des[i] > 1.0) 
			cur_des[i] = 1.0;
		else if (cur_des[i] < -1.0) 
			cur_des[i] = -1.0;
	}
	for (int i=0; i<4;i++)
	{
		// --- PWM_MAX per-joint clamp (added) -------------------------------
		// Enforce the per-motor PWM ceiling loaded from PWM_LIMIT_* in
		// startCAN() (PWM_MAX[]). Without this, the only limit was the
		// controller torque_max (0.7 Nm -> 840 PWM), which lets the hardest-
		// working motor (thumb roll, j12) draw sustained overcurrent and
		// overheat until the firmware high-temperature fault latches it off.
		// The guard (lim > 0) means an unset limit never zeroes a motor, so
		// behaviour is unchanged for any joint whose PWM_MAX was not set.
		for (int k=0; k<4; ++k)
		{
			int j = i*4 + k;
			short p = (short)(cur_des[j]*tau_cov_const);
			short lim = (short)PWM_MAX[j];
			if (lim > 0)
			{
				if (p >  lim) p =  lim;
				else if (p < -lim) p = -lim;
			}
			pwm[j] = p;
		}
		// --- end PWM_MAX clamp --------------------------------------------
		command_set_torque(bus, i, &pwm[4*i]);
	}

}

void CAN_AH::Version3Handler::writeDevices(){
	double cur_des[DOF_JOINTS];
	short pwm[DOF_JOINTS];

	for (int i=0; i<DOF_JOINTS; i++)
	{
		cur_des[i] = torque[i]*(double)motorDirection[i];
		if (cur_des[i] > 1.0) {
			cur_des[i] = 1.0;
		}
		else if (cur_des[i] < -1.0) {
			cur_des[i] = -1.0;
		}
	}

	for (int i=0; i<4;i++)
	{
		pwm[i*4+0] = (short)(cur_des[i*4+0]*tau_cov_const);
		pwm[i*4+1] = (short)(cur_des[i*4+1]*tau_cov_const);
		pwm[i*4+2] = (short)(cur_des[i*4+2]*tau_cov_const);
		pwm[i*4+3] = (short)(cur_des[i*4+3]*tau_cov_const);
		command_set_torque(bus, i, &pwm[4*i]);
	}
}

int CAN_AH::Version4Handler::command_set_torque(int ch, int findex, short* pwm){
    assert(findex >= 0 && findex < 4);

    long Txid;
    short duty[4];
    int ret;

    if (findex >= 0 && findex < 4)
    {
        duty[0] = pwm[0];
        duty[1] = pwm[1];
        duty[2] = pwm[2];
        duty[3] = pwm[3];
        Txid = ID_CMD_SET_TORQUE_1 + findex;
        ret = canSendMsg(ch, Txid, 8, (unsigned char *)duty);
    }
    else
        return -1;
    return ret;
}

int CAN_AH::Version3Handler::command_set_torque(int ch, int findex, short* pwm){
	if (findex >= 0 && findex < 4)
	{
		if ((hand_version == 1.0))  { // for Allegro Hand v1.0
			data[0] = (unsigned char)((pwm[0] >> 8) & 0x00ff);
			data[1] = (unsigned char)( pwm[0]       & 0x00ff);
			data[2] = (unsigned char)((pwm[1] >> 8) & 0x00ff);
			data[3] = (unsigned char)( pwm[1]       & 0x00ff);
			data[4] = (unsigned char)((pwm[2] >> 8) & 0x00ff);
			data[5] = (unsigned char)( pwm[2]       & 0x00ff);
			data[6] = (unsigned char)((pwm[3] >> 8) & 0x00ff);
			data[7] = (unsigned char)( pwm[3]       & 0x00ff);
		} else if ((hand_version == 2.0) || (hand_version == 3.0)) { // for Allegro Hand v2.0 and v3.0
			data[0] = (unsigned char)((pwm[3] >> 8) & 0x00ff);
			data[1] = (unsigned char)( pwm[3]       & 0x00ff);
			data[2] = (unsigned char)((pwm[2] >> 8) & 0x00ff);
			data[3] = (unsigned char)( pwm[2]       & 0x00ff);
			data[4] = (unsigned char)((pwm[1] >> 8) & 0x00ff);
			data[5] = (unsigned char)( pwm[1]       & 0x00ff);
			data[6] = (unsigned char)((pwm[0] >> 8) & 0x00ff);
			data[7] = (unsigned char)( pwm[0]       & 0x00ff);
		}
		else{
			std::cout << "Unrecognized hand version" << std::endl;
		}
		Txid = ((DWORD)(ID_CMD_SET_TORQUE_1 + findex)<<6) | (ID_COMMON <<3) | (ID_DEVICE_MAIN);
		ret = canSendMsg(ch, Txid, 8, data);
	}
	else
		return -1;

	return 0;
}

// Get channel index for Peak CAN interface
int CAN_AH::GetCANChannelIndex(const TCHAR* cname){
    if (!cname) return 0;
    if (!_tcsicmp(cname, _T("0")) || !_tcsicmp(cname, _T("PCAN_NONEBUS")) || !_tcsicmp(cname, _T("NONEBUS")))
        return 0;
    // else if (!_tcsicmp(cname, _T("1")) || !_tcsicmp(cname, _T("PCAN_ISABUS1")) || !_tcsicmp(cname, _T("ISABUS1")))
    //     return 1;
    // else if (!_tcsicmp(cname, _T("2")) || !_tcsicmp(cname, _T("PCAN_ISABUS2")) || !_tcsicmp(cname, _T("ISABUS2")))
    //     return 2;
    // else if (!_tcsicmp(cname, _T("3")) || !_tcsicmp(cname, _T("PCAN_ISABUS3")) || !_tcsicmp(cname, _T("ISABUS3")))
    //     return 3;
    // else if (!_tcsicmp(cname, _T("4")) || !_tcsicmp(cname, _T("PCAN_ISABUS4")) || !_tcsicmp(cname, _T("ISABUS4")))
    //     return 4;
    // else if (!_tcsicmp(cname, _T("5")) || !_tcsicmp(cname, _T("PCAN_ISABUS5")) || !_tcsicmp(cname, _T("ISABUS5")))
    //     return 5;
    // else if (!_tcsicmp(cname, _T("7")) || !_tcsicmp(cname, _T("PCAN_ISABUS6")) || !_tcsicmp(cname, _T("ISABUS6")))
    //     return 6;
    // else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS7")) || !_tcsicmp(cname, _T("ISABUS7")))
    //     return 7;
    // else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS8")) || !_tcsicmp(cname, _T("ISABUS8")))
    //     return 8;
    // else if (!_tcsicmp(cname, _T("9")) || !_tcsicmp(cname, _T("PCAN_DNGBUS1")) || !_tcsicmp(cname, _T("DNGBUS1")))
    //     return 9;
    // else if (!_tcsicmp(cname, _T("10")) || !_tcsicmp(cname, _T("PCAN_PCIBUS1")) || !_tcsicmp(cname, _T("PCIBUS1")))
    //     return 10;
    // else if (!_tcsicmp(cname, _T("11")) || !_tcsicmp(cname, _T("PCAN_PCIBUS2")) || !_tcsicmp(cname, _T("PCIBUS2")))
    //     return 11;
    // else if (!_tcsicmp(cname, _T("12")) || !_tcsicmp(cname, _T("PCAN_PCIBUS3")) || !_tcsicmp(cname, _T("PCIBUS3")))
    //     return 12;
    // else if (!_tcsicmp(cname, _T("13")) || !_tcsicmp(cname, _T("PCAN_PCIBUS4")) || !_tcsicmp(cname, _T("PCIBUS4")))
    //     return 13;
    // else if (!_tcsicmp(cname, _T("14")) || !_tcsicmp(cname, _T("PCAN_PCIBUS5")) || !_tcsicmp(cname, _T("PCIBUS5")))
    //     return 14;
    // else if (!_tcsicmp(cname, _T("15")) || !_tcsicmp(cname, _T("PCAN_PCIBUS6")) || !_tcsicmp(cname, _T("PCIBUS6")))
    //     return 15;
    // else if (!_tcsicmp(cname, _T("16")) || !_tcsicmp(cname, _T("PCAN_PCIBUS7")) || !_tcsicmp(cname, _T("PCIBUS7")))
    //     return 16;
    // else if (!_tcsicmp(cname, _T("17")) || !_tcsicmp(cname, _T("PCAN_PCIBUS8")) || !_tcsicmp(cname, _T("PCIBUS8")))
    //     return 17;
    else if (!_tcsicmp(cname, _T("18")) || !_tcsicmp(cname, _T("PCAN_USBBUS1")) || !_tcsicmp(cname, _T("USBBUS1")))
        return 18;
    else if (!_tcsicmp(cname, _T("19")) || !_tcsicmp(cname, _T("PCAN_USBBUS2")) || !_tcsicmp(cname, _T("USBBUS2")))
        return 19;
    else if (!_tcsicmp(cname, _T("20")) || !_tcsicmp(cname, _T("PCAN_USBBUS3")) || !_tcsicmp(cname, _T("USBBUS3")))
        return 20;
    else if (!_tcsicmp(cname, _T("21")) || !_tcsicmp(cname, _T("PCAN_USBBUS4")) || !_tcsicmp(cname, _T("USBBUS4")))
        return 21;
    else if (!_tcsicmp(cname, _T("22")) || !_tcsicmp(cname, _T("PCAN_USBBUS5")) || !_tcsicmp(cname, _T("USBBUS5")))
        return 22;
    else if (!_tcsicmp(cname, _T("23")) || !_tcsicmp(cname, _T("PCAN_USBBUS6")) || !_tcsicmp(cname, _T("USBBUS6")))
        return 23;
    else if (!_tcsicmp(cname, _T("24")) || !_tcsicmp(cname, _T("PCAN_USBBUS7")) || !_tcsicmp(cname, _T("USBBUS7")))
        return 24;
    else if (!_tcsicmp(cname, _T("25")) || !_tcsicmp(cname, _T("PCAN_USBBUS8")) || !_tcsicmp(cname, _T("USBBUS8")))
        return 25;
    // else if (!_tcsicmp(cname, _T("26")) || !_tcsicmp(cname, _T("PCAN_PCCBUS1")) || !_tcsicmp(cname, _T("PCCBUS1")))
    //     return 26;
    // else if (!_tcsicmp(cname, _T("27")) || !_tcsicmp(cname, _T("PCAN_PCCBUS2")) || !_tcsicmp(cname, _T("PCCBUS2")))
    //     return 27;
    else
        return 0;
}

void CAN_AH::initV3(const Parameters &params){
	versionHandler->hand_version = hand_version;

	versionHandler->PWM_MAX[eJOINTNAME_INDEX_0] = PWM_LIMIT_ROLL;
	versionHandler->PWM_MAX[eJOINTNAME_INDEX_1] = PWM_LIMIT_NEAR;
	versionHandler->PWM_MAX[eJOINTNAME_INDEX_2] = PWM_LIMIT_MIDDLE;
	versionHandler->PWM_MAX[eJOINTNAME_INDEX_3] = PWM_LIMIT_FAR;

	versionHandler->PWM_MAX[eJOINTNAME_MIDDLE_0] = PWM_LIMIT_ROLL;
	versionHandler->PWM_MAX[eJOINTNAME_MIDDLE_1] = PWM_LIMIT_NEAR;
	versionHandler->PWM_MAX[eJOINTNAME_MIDDLE_2] = PWM_LIMIT_MIDDLE;
	versionHandler->PWM_MAX[eJOINTNAME_MIDDLE_3] = PWM_LIMIT_FAR;

	versionHandler->PWM_MAX[eJOINTNAME_PINKY_0] = PWM_LIMIT_ROLL;
	versionHandler->PWM_MAX[eJOINTNAME_PINKY_1] = PWM_LIMIT_NEAR;
	versionHandler->PWM_MAX[eJOINTNAME_PINKY_2] = PWM_LIMIT_MIDDLE;
	versionHandler->PWM_MAX[eJOINTNAME_PINKY_3] = PWM_LIMIT_FAR;

	versionHandler->PWM_MAX[eJOINTNAME_THUMB_0] = PWM_LIMIT_THUMB_ROLL;
	versionHandler->PWM_MAX[eJOINTNAME_THUMB_1] = PWM_LIMIT_THUMB_NEAR;
	versionHandler->PWM_MAX[eJOINTNAME_THUMB_2] = PWM_LIMIT_THUMB_MIDDLE;
	versionHandler->PWM_MAX[eJOINTNAME_THUMB_3] = PWM_LIMIT_THUMB_FAR;

	versionHandler->encoderOffset[eJOINTNAME_INDEX_0] = params.encoderOffset.j00;
	versionHandler->encoderOffset[eJOINTNAME_INDEX_1] = params.encoderOffset.j01;
	versionHandler->encoderOffset[eJOINTNAME_INDEX_2] = params.encoderOffset.j02;
	versionHandler->encoderOffset[eJOINTNAME_INDEX_3] = params.encoderOffset.j03;
	versionHandler->encoderOffset[eJOINTNAME_MIDDLE_0] = params.encoderOffset.j10;
	versionHandler->encoderOffset[eJOINTNAME_MIDDLE_1] = params.encoderOffset.j11;
	versionHandler->encoderOffset[eJOINTNAME_MIDDLE_2] = params.encoderOffset.j12;
	versionHandler->encoderOffset[eJOINTNAME_MIDDLE_3] = params.encoderOffset.j13;
	versionHandler->encoderOffset[eJOINTNAME_PINKY_0] = params.encoderOffset.j20;
	versionHandler->encoderOffset[eJOINTNAME_PINKY_1] = params.encoderOffset.j21;
	versionHandler->encoderOffset[eJOINTNAME_PINKY_2] = params.encoderOffset.j22;
	versionHandler->encoderOffset[eJOINTNAME_PINKY_3] = params.encoderOffset.j23;
	versionHandler->encoderOffset[eJOINTNAME_THUMB_0] = params.encoderOffset.j30;
	versionHandler->encoderOffset[eJOINTNAME_THUMB_1] = params.encoderOffset.j31;
	versionHandler->encoderOffset[eJOINTNAME_THUMB_2] = params.encoderOffset.j32;
	versionHandler->encoderOffset[eJOINTNAME_THUMB_3] = params.encoderOffset.j33;

	versionHandler->encoderDirection[eJOINTNAME_INDEX_0] = params.encoderDirection.j00;
	versionHandler->encoderDirection[eJOINTNAME_INDEX_1] = params.encoderDirection.j01;
	versionHandler->encoderDirection[eJOINTNAME_INDEX_2] = params.encoderDirection.j02;
	versionHandler->encoderDirection[eJOINTNAME_INDEX_3] = params.encoderDirection.j03;
	versionHandler->encoderDirection[eJOINTNAME_MIDDLE_0] = params.encoderDirection.j10;
	versionHandler->encoderDirection[eJOINTNAME_MIDDLE_1] = params.encoderDirection.j11;
	versionHandler->encoderDirection[eJOINTNAME_MIDDLE_2] = params.encoderDirection.j12;
	versionHandler->encoderDirection[eJOINTNAME_MIDDLE_3] = params.encoderDirection.j13;
	versionHandler->encoderDirection[eJOINTNAME_PINKY_0] = params.encoderDirection.j20;
	versionHandler->encoderDirection[eJOINTNAME_PINKY_1] = params.encoderDirection.j21;
	versionHandler->encoderDirection[eJOINTNAME_PINKY_2] = params.encoderDirection.j22;
	versionHandler->encoderDirection[eJOINTNAME_PINKY_3] = params.encoderDirection.j23;
	versionHandler->encoderDirection[eJOINTNAME_THUMB_0] = params.encoderDirection.j30;
	versionHandler->encoderDirection[eJOINTNAME_THUMB_1] = params.encoderDirection.j31;
	versionHandler->encoderDirection[eJOINTNAME_THUMB_2] = params.encoderDirection.j32;
	versionHandler->encoderDirection[eJOINTNAME_THUMB_3] = params.encoderDirection.j33;

	versionHandler->motorDirection[eJOINTNAME_INDEX_0] = params.motorDirection.j00;
	versionHandler->motorDirection[eJOINTNAME_INDEX_1] = params.motorDirection.j01;
	versionHandler->motorDirection[eJOINTNAME_INDEX_2] = params.motorDirection.j02;
	versionHandler->motorDirection[eJOINTNAME_INDEX_3] = params.motorDirection.j03;
	versionHandler->motorDirection[eJOINTNAME_MIDDLE_0] = params.motorDirection.j10;
	versionHandler->motorDirection[eJOINTNAME_MIDDLE_1] = params.motorDirection.j11;
	versionHandler->motorDirection[eJOINTNAME_MIDDLE_2] = params.motorDirection.j12;
	versionHandler->motorDirection[eJOINTNAME_MIDDLE_3] = params.motorDirection.j13;
	versionHandler->motorDirection[eJOINTNAME_PINKY_0] = params.motorDirection.j20;
	versionHandler->motorDirection[eJOINTNAME_PINKY_1] = params.motorDirection.j21;
	versionHandler->motorDirection[eJOINTNAME_PINKY_2] = params.motorDirection.j22;
	versionHandler->motorDirection[eJOINTNAME_PINKY_3] = params.motorDirection.j23;
	versionHandler->motorDirection[eJOINTNAME_THUMB_0] = params.motorDirection.j30;
	versionHandler->motorDirection[eJOINTNAME_THUMB_1] = params.motorDirection.j31;
	versionHandler->motorDirection[eJOINTNAME_THUMB_2] = params.motorDirection.j32;
	versionHandler->motorDirection[eJOINTNAME_THUMB_3] = params.motorDirection.j33;
}