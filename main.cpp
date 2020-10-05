#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <cassert>

#include <WS2tcpip.h>
#pragma comment (lib, "ws2_32.lib")

#define _CRT_SECURE_NO_WARNINGS

#include <Windows.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include "hd_controller.h"
#include "hd_comm.h"
#include "hd_logger.h"

using namespace std;

HHD phantomId_1, phantomId_2; // Dual Phantom devices.
HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;
HDSchedulerHandle gSchedulerCallback2 = HD_INVALID_HANDLE;

/* Global variable declaration*/
int modC = 10;   // default: 1000 Hz -> 1000%modC HZ

hduVector3Dd posMasterHold(0, 0, 0);
HDdouble velThreshold = 300;

const int Vband[3] = { 3,3,3 };
double Vhold[3];
int Vcount[3] = { 0,0,0 };

HDErrorInfo lastError;
/* Haptic device record. */
struct DeviceDisplayStates
{
	HHD m_hHD;
	hduVector3Dd position;
	hduVector3Dd force;
};

SOCKET master_sock;
SOCKET slave_sock;
sockaddr_in master_addr;
sockaddr_in slave_addr;

HDCommunicator* HDComm;
HDCommunicator* HDComm2;

HapticDeviceController* DeviceCon;
HapticDeviceController* DeviceCon2;

// FOR INITIAL SETTINGS
char DEVICE_NAME_1[32] = "PHANToM 1";
char DEVICE_NAME_2[32] = "PHANToM 2";
char ALIAS = 'S';
char MASTER_ADDR[32] = "127.0.0.1"; //"192.168.1.158"; "192.168.42.166"
uint32_t MASTER_PORT = 25000;
char SLAVE_ADDR[32] = "127.0.0.1"; // "192.168.1.136"; "192.168.42.166"
uint32_t SLAVE_PORT = 25001;

/******************************************************************************
Makes a device specified in the pUserData current.
Queries haptic device state: position, force, etc.
******************************************************************************/
HDCallbackCode HDCALLBACK DeviceStateCallback(void *pUserData)
{
	DeviceDisplayStates *pDisplayState = static_cast<DeviceDisplayStates *>(pUserData);

	hdMakeCurrentDevice(pDisplayState->m_hHD);
	hdGetDoublev(HD_CURRENT_POSITION, pDisplayState->position);
	hdGetDoublev(HD_CURRENT_FORCE, pDisplayState->force);

	return HD_CALLBACK_DONE;
}

/*******************************************************************************
Given the position is space, calculates the (modified) coulomb force.
*******************************************************************************/


//V-band algorithm
void ifErrorHold(hduVector3Dd pos)
{
	for (int i = 0; i++; i = 3)
	{
		if (fabs(pos[i] - Vhold[i]) > Vband[i] * (Vcount[i] + 1))
		{
			pos[i] = Vhold[i];
			Vcount[i]++;
		}
		else {
			Vcount[i] = 0;
			Vhold[i] = pos[i];
		}
	}
}

//velocity-adaptive device control
void velCon(hduVector3Dd pos)
{
	// Device Hz control 
	HDdouble velMaster = ((pos - posMasterHold)*(1000 / modC)).magnitude();
	if (velMaster > velThreshold)
	{
		modC = 5;
		//cout << velMaster << " / " << 1000 / modC << endl;
	}
	else
	{
		modC = 1;
		//cout << velMaster << " / " << 1000 / modC << endl;
	}
	posMasterHold = pos;
}

/******************************************************************************
Main callback.  Retrieves position from both devices, calculates forces,
and sets forces for both devices.
******************************************************************************/
HDCallbackCode HDCALLBACK deviceCallback(void *data)
{
	DeviceCon->tick();

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Error during scheduler callback");
		if (hduIsSchedulerError(&error)) return HD_CALLBACK_DONE;
	}

	return HD_CALLBACK_CONTINUE;
}

HDCallbackCode HDCALLBACK deviceCallback2(void *data)
{
	DeviceCon2->tick();

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Error during scheduler callback");
		if (hduIsSchedulerError(&error)) return HD_CALLBACK_DONE;
	}

	return HD_CALLBACK_CONTINUE;
}

int initSocket() {
	// create a hint structure for the server
	memset(&master_addr, 0, sizeof(master_addr));
	master_addr.sin_family = AF_INET;
	master_addr.sin_port = htons(MASTER_PORT);
	inet_pton(AF_INET, MASTER_ADDR, &master_addr.sin_addr);

	memset(&slave_addr, 0, sizeof(slave_addr));
	slave_addr.sin_family = AF_INET;
	slave_addr.sin_port = htons(SLAVE_PORT);
	inet_pton(AF_INET, SLAVE_ADDR, &slave_addr.sin_addr);

	ULONG isNonBlocking = 1;

	// socket creation

	//if (ALIAS == 'S') {
	{
		slave_sock = socket(AF_INET, SOCK_DGRAM, 0);
		ioctlsocket(slave_sock, FIONBIO, &isNonBlocking);
		if (::bind(slave_sock, (sockaddr*)&slave_addr, sizeof(sockaddr_in)) == SOCKET_ERROR) {
			cout << "Can't bind slave socket! " << WSAGetLastError() << endl;
			return -1;
		}
	}
	//else if (ALIAS == 'M') {
	{
		master_sock = socket(AF_INET, SOCK_DGRAM, 0);
		ioctlsocket(master_sock, FIONBIO, &isNonBlocking);
		if (::bind(master_sock, (sockaddr*)&master_addr, sizeof(sockaddr_in)) == SOCKET_ERROR) {
			cout << "Can't bind master socket! " << WSAGetLastError() << endl;
			return -1;
		}
	}
	/*else {
	printf("Err: Alias should be either M or S\n");
	return -1;
	}*/

	return 0;
}

/******************************************************************************
This handler gets called when the process is exiting.  Ensures that HDAPI is
properly shutdown.
******************************************************************************/
void exitHandler()
{

	if (!lastError.errorCode)
	{
		hdStopScheduler();
		hdUnschedule(gSchedulerCallback);
		hdUnschedule(gSchedulerCallback2);
	}

	if (phantomId_1 != HD_INVALID_HANDLE)
	{
		hdDisableDevice(phantomId_1);
		phantomId_1 = HD_INVALID_HANDLE;
	}

	if (phantomId_2 != HD_INVALID_HANDLE)
	{
		hdDisableDevice(phantomId_2);
		phantomId_2 = HD_INVALID_HANDLE;
	}
}

/******************************************************************************
Main entry point.
******************************************************************************/
int main(int argc, char* argv[])
{
	HDErrorInfo error;

	if (argc > 1) {
		if (argc == 5) {
			ALIAS = argv[1][0];
			if (ALIAS == 'M') {
				strcpy(SLAVE_ADDR, argv[2]);
				SLAVE_PORT = atoi(argv[3]);
			}
			else {
				strcpy(MASTER_ADDR, argv[2]);
				MASTER_PORT = atoi(argv[3]);
			}
			strcpy(DEVICE_NAME_1, argv[4]);
		}
		else {
			printf("Usage: ./CouloumbForceDual.exe <alias> <target IP> <target PORT> <device name>n");
		}
	}

	printf("Starting application\n");

	atexit(exitHandler);

	// First device.
	phantomId_1 = hdInitDevice(DEVICE_NAME_1);
	if (HD_DEVICE_ERROR(lastError = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to initialize first haptic device");
		fprintf(stderr, "Make sure the configuration \"%s\" exists\n", DEVICE_NAME_1);
		fprintf(stderr, "\nPress any key to quit.\n");
		exit(-1);
	}

	printf("1. Found device %s\n", hdGetString(HD_DEVICE_MODEL_TYPE));
	hdEnable(HD_FORCE_OUTPUT);
	hdEnable(HD_FORCE_RAMPING);

	// Second device.
	phantomId_2 = hdInitDevice(DEVICE_NAME_2);
	if (HD_DEVICE_ERROR(lastError = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to initialize second haptic device");
		fprintf(stderr, "Make sure the configuration \"%s\" exists\n", DEVICE_NAME_2);
		fprintf(stderr, "\nPress any key to quit.\n");
		exit(-1);
	}

	printf("2. Found device %s\n", hdGetString(HD_DEVICE_MODEL_TYPE));
	hdEnable(HD_FORCE_OUTPUT);
	hdEnable(HD_FORCE_RAMPING);

	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to start scheduler");
		fprintf(stderr, "\nPress any key to quit.\n");
		getchar();
		exit(-1);
	}

	hdMakeCurrentDevice(phantomId_1);

	// startup winsock
	WSADATA data;
	WORD version = MAKEWORD(2, 2);
	int wsOK = WSAStartup(version, &data);
	if (wsOK != 0)
	{
		cout << "Can't start Winsock! " << wsOK;
		return -1;
	}

	if (initSocket() < 0) {
		cout << "Failed to initialize socket" << endl;
		return -1;
	}

	// haptics callback
	std::cout << "haptics callback" << std::endl;

	SNDLogger m_sndlogger("m_snd.csv");
	RCVLogger m_rcvlogger("m_rcv.csv");
	ERRLogger m_errlogger("m_err.csv");

	SNDLogger s_sndlogger("s_snd.csv");
	RCVLogger s_rcvlogger("s_rcv.csv");
	ERRLogger s_errlogger("s_err.csv");

	HDComm = new HDCommunicator(phantomId_1, master_sock, &slave_addr, sizeof(slave_addr), 'M', &m_sndlogger, &m_rcvlogger, &m_errlogger);
	DeviceCon = new HapticDeviceController(phantomId_1, 'M', HDComm, &m_sndlogger, &m_rcvlogger, &m_errlogger);

	HDComm2 = new HDCommunicator(phantomId_2, slave_sock, &master_addr, sizeof(master_addr), 'S', &s_sndlogger, &s_rcvlogger, &s_errlogger);
	DeviceCon2 = new HapticDeviceController(phantomId_2, 'S', HDComm2, &s_sndlogger, &s_rcvlogger, &s_errlogger);

	gSchedulerCallback = hdScheduleAsynchronous(
		deviceCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

	gSchedulerCallback2 = hdScheduleAsynchronous(
		deviceCallback2, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to initialize haptic device");
		fprintf(stderr, "\nPress any key to quit.\n");
		getchar();
		exit(-1);
	}

	while (true);

	// close socket
	closesocket(master_sock);
	closesocket(slave_sock);
	WSACleanup();

	return 0;
}
