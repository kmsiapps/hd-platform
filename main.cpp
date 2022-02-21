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

HHD deviceID; // Dual Phantom devices.
HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;

hduVector3Dd forceField(hduVector3Dd pos);

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

SOCKET sock;
sockaddr_in server_addr;

HDCommunicator* HDComm;
HapticDeviceController* DeviceCon;

char* DEVICE_NAME;
char* SERVER_ADDR;
uint32_t SERVER_PORT = 50000;

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

int initSocket() {
	ULONG isNonBlocking = 1;

	// create a hint structure for the server
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(SERVER_PORT);
	inet_pton(AF_INET, SERVER_ADDR, &server_addr.sin_addr);

	sock = socket(AF_INET, SOCK_DGRAM, 0);
	ioctlsocket(sock, FIONBIO, &isNonBlocking);

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
	}

	if (deviceID != HD_INVALID_HANDLE)
	{
		hdDisableDevice(deviceID);
		deviceID = HD_INVALID_HANDLE;
	}
}

/******************************************************************************
Main entry point.
******************************************************************************/
int main(int argc, char* argv[])
{
	HDErrorInfo error;

	if (argc == 4) {
		SERVER_ADDR = argv[1];
		SERVER_PORT = atoi(argv[2]);
		DEVICE_NAME = argv[3];
	}
	else {
		printf("Usage: ./CouloumbForceDual.exe <server HOST> <server PORT> <device name>n");
		return 0;
	}

	printf("Starting application\n");

	atexit(exitHandler);

	// Initialize device
	deviceID = hdInitDevice(DEVICE_NAME);
	if (HD_DEVICE_ERROR(lastError = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to initialize first haptic device");
		fprintf(stderr, "Make sure the configuration \"%s\" exists\n", DEVICE_NAME);
		fprintf(stderr, "\nPress any key to quit.\n");
		getchar();
		exit(-1);
	}

	printf("Found device %s\n", hdGetString(HD_DEVICE_MODEL_TYPE));
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

	hdMakeCurrentDevice(deviceID);

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

	SNDLogger m_sndlogger("m_snd.csv");
	RCVLogger m_rcvlogger("m_rcv.csv");
	ERRLogger m_errlogger("m_err.csv");

	// haptics callback
	std::cout << "haptics callback" << std::endl;
	HDComm = new HDCommunicator(deviceID, sock, &server_addr, sizeof(server_addr), 'S', &m_sndlogger, &m_rcvlogger, &m_errlogger);
	DeviceCon = new HapticDeviceController(deviceID, 'S', HDComm, &m_sndlogger, &m_rcvlogger, &m_errlogger);

	gSchedulerCallback = hdScheduleAsynchronous(
		deviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to initialize haptic device");
		fprintf(stderr, "\nPress any key to quit.\n");
		getchar();
		exit(-1);
	}

	while (true);

	// close socket
	closesocket(sock);
	WSACleanup();

	return 0;
}
