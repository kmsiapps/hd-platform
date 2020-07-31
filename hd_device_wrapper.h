#pragma once

#include <HD/hd.h>
#include <HDU/hduVector.h>

#include "hd_comm.h"

class HapticDevice {
private:
	HHD deviceID;
	char alias;
	HDCommunicator *HDComm;

	/* Charge (positive/negative) */
	int charge = 1;

	hduVector3Dd forceField(hduVector3Dd pos)
	{
		double dist = pos.magnitude();

		hduVector3Dd forceVec(0, 0, 0);

		// Attract the charge to the center of the sphere.
		forceVec = -0.1*pos; // default = 0.1
		forceVec *= charge;

		return forceVec;
	}

	void updateState() {
		HDComm->rcvPacket(true);
		if (HDComm->isPacketValid()) {
			hduVector3Dd pos;
			hduVector3Dd RCVpos;
			hdGetDoublev(HD_CURRENT_POSITION, pos);

			HDComm->getPacket()->getPos(&RCVpos);

			hduVector3Dd posDiff_2 = pos - RCVpos;
			hduVector3Dd forceVec_2 = forceField(posDiff_2);

			hdMakeCurrentDevice(deviceID);
			hdSetDoublev(HD_CURRENT_FORCE, forceVec_2);
		}
	}

public:
	HapticDevice(HHD deviceID, char alias, HDCommunicator* HDComm) {
		this->deviceID = deviceID;
		this->alias = alias;
		this->HDComm = HDComm;
	}
	
	void tick() {
		hdBeginFrame(deviceID);

		if (alias == 'M') {
			HDComm->preparePacket();
			HDComm->sendPacket();
			updateState();
		}
		else if (alias == 'S') {
			updateState();
			HDComm->preparePacket();
			HDComm->sendPacket();
		}
		else {
			printf("Err: Alias should be either M or S\n");
			exit(-1);
		}
		hdEndFrame(deviceID);
	}
};
