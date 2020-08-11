#pragma once

#include <list>

#include <HD/hd.h>
#include <HDU/hduVector.h>

#include "hd_comm.h"

#define PREDICTOR_QUEUE_SIZE 5 // packetQueue Size
#define PREDICTOR_CONSTANT_K 1 // K in Weber's law

class HapticDeviceController {
private:
	HHD deviceID;						// haptic device ID
	char alias;							// alias - 'M' for master, 'S' for slave
	HDCommunicator *HDComm;				// HDCommunicator for udp packet exchange
	std::list<HDPacket*> packetQueue;	// queue used for predictive coding
	float posDelta;						// last movement difference, used for perception based coding

	/* Charge (positive/negative) */
	int charge = 1;

	hduVector3Dd posToForce(hduVector3Dd pos)
	{
		double dist = pos.magnitude();

		hduVector3Dd forceVec(0, 0, 0);

		// Attract the charge to the center of the sphere.
		forceVec = -0.1*pos; // default = 0.1
		forceVec *= charge;

		return forceVec;
	}

	HapticPacket* preparePacket(bool debug = false) {
		/* create packet from current device state. */

		hdBeginFrame(deviceID);
		// get current device position
		hduVector3Dd pos;
		hdGetDoublev(HD_CURRENT_POSITION, pos);

		hdEndFrame(deviceID);

		// update packet with current position
		HapticPacket* sendingPacket = new HapticPacket();
		sendingPacket->update(&pos);

		if (debug) printf("%c preparePacket::Pos(%.1f %.1f %.1f) Sent(%u) Time(%llu)\n", alias, pos[0], pos[1], pos[2], sendingPacket->getCounter(), sendingPacket->getTimestamp());
		return sendingPacket;
	}

	hduVector3Dd* predictPos(hduVector3Dd* base_pos) {
		std::list<HDPacket*>::iterator iter;
		hduVector3Dd* prev = NULL;
		hduVector3Dd* acc = new hduVector3Dd(0, 0, 0);

		if (base_pos) {
			for (iter=packetQueue.begin; iter != packetQueue.end; iter++) {
				// calculate vector slope
				if (prev) *acc += (**iter - *prev);
				prev = *iter;
			}
			*acc /= packetQueue.size();
		}

		*acc += *base_pos;
		return acc;
	}

	void updateState() {
		// recieve packet from remote, and update current device's state with the packet
		HapticPacket* packet = HDComm->rcvPacket(true);
		hduVector3Dd target_pos();
		hduVector3Dd current_pos;
		hdGetDoublev(HD_CURRENT_POSITION, current_pos);

		if (packet == NULL)
			// No received pos
			target_pos = predictPos(current_pos);
		else if (HDComm->isPacketValid(packet)) 
			target_pos = *(packet->getPos());

		hduVector3Dd posDiff = current_pos - target_pos;
		hduVector3Dd forceVec = posToForce(posDiff);

		hdMakeCurrentDevice(deviceID);
		hdSetDoublev(HD_CURRENT_FORCE, forceVec);

		posDelta = (*packet - *(packetQueue.back())).magnitude();
		packetQueue.push_back(packet);
		if (packetQueue.size() > PREDICTOR_QUEUE_SIZE)
			delete packetQueue.pop_front();
	}

	void sendState(bool debug = false) {
		// predictive, perception-based packet sending
		HapticPacket* packet = preparePacket();
		hduVector3Dd real_pos = *(packet->getPos());

		// predictive packet sending
		hduVector3Dd pred_pos = *predictPos(packetQueue.back());

		// perception-based packet sending
		if ((pred_pos - real_pos).magnitude() / posDelta < PREDICTOR_CONSTANT_K)
			if (debug) printf("%c sendState::packet sending dismissed")
		else
			HDComm->sendPacket(packet);
		
		delete packet;
		delete &pred_pos;
		delete &real_pos;
	}

public:
	HapticDeviceController(HHD deviceID, char alias, HDCommunicator* HDComm) {
		this->deviceID = deviceID;
		this->alias = alias;
		this->HDComm = HDComm;
		posDelta = 0;
	}

	void tick() {
		hdBeginFrame(deviceID);

		if (alias == 'M') {
			sendState();
			updateState();
		}
		else if (alias == 'S') {
			updateState();
			sendState();
		}
		else {
			printf("Err: Alias should be either M or S\n");
			exit(-1);
		}
		hdEndFrame(deviceID);
	}
};
