#pragma once

#include <list>

#include <HD/hd.h>
#include <HDU/hduVector.h>

#include "hd_comm.h"

#define PREDICTOR_QUEUE_SIZE 5 // packet_queue Size
#define PREDICTOR_CONSTANT_K 1 // K in Weber's law

class HapticDeviceController {
private:
	hduVector3Dd PosToForce(const hduVector3Dd pos);
	HapticPacket PreparePacket(bool debug);
	hduVector3Dd PredictPos(const hduVector3Dd base_pos);
	void UpdateState();
	void SendState(bool debug);
	bool IsPerceptable(const hduVector3Dd pred_pos, const hduVector3Dd real_pos);

	/* Charge (positive/negative) */
	const int kCharge = 1;

	HHD device_id;						// haptic device ID
	char alias;							// alias - 'M' for master, 'S' for slave
	HDCommunicator *hdcomm;				// HDCommunicator for udp packet exchange
	std::list<HDPacket*> packet_queue;	// queue used for predictive coding
	float pos_delta;						// last movement difference, used for perception based coding

public:
	HapticDeviceController(const HHD device_id, const char alias, const HDCommunicator *hdcomm);
	void tick();
};

hduVector3Dd HapticDeviceController::PosToForce(const hduVector3Dd pos)
{
	double dist = pos.magnitude();

	hduVector3Dd force_vec(0, 0, 0);

	// Attract the kCharge to the center of the sphere.
	force_vec = -0.1*pos; // default = 0.1
	force_vec *= kCharge;

	return force_vec;
}

HapticPacket HapticDeviceController::PreparePacket(bool debug = false) {
	/* create packet from current device state. */

	hdBeginFrame(device_id);
	// get current device position
	hduVector3Dd pos;
	hdGetDoublev(HD_CURRENT_POSITION, pos);

	hdEndFrame(device_id);

	if (debug)
		printf("%c PreparePacket::Pos(%.1f %.1f %.1f) Sent(%u) Time(%llu)\n",
		alias, pos[0], pos[1], pos[2], sendingPacket->getCounter(), sendingPacket->getTimestamp());
	return sendingPacket(pos);
}

hduVector3Dd HapticDeviceController::PredictPos(const hduVector3Dd base_pos) {
	std::list<HDPacket*>::iterator iter;
	hduVector3Dd* prev = NULL;
	hduVector3Dd acc = hduVector3Dd(0, 0, 0);

	for (iter=packet_queue.begin; iter != packet_queue.end; iter++) {
		// calculate vector slope
		if (prev) acc += (**iter - *prev);
		prev = *iter;
	}
	
	acc /= packet_queue.size();
	acc += base_pos;
	return acc;
}

void HapticDeviceController::UpdateState() {
	// recieve packet from remote, and update current device's state with the packet
	HapticPacket* packet = HDComm->RecievePacket(true);
	hduVector3Dd target_pos();
	hduVector3Dd current_pos;
	hdGetDoublev(HD_CURRENT_POSITION, current_pos);

	if (packet == NULL)
		// No received pos
		target_pos = PredictPos(current_pos);
	else if (HDComm->IsPacketValid(*packet)) 
		target_pos = packet->GetPos();

	hduVector3Dd posDiff = current_pos - target_pos;
	hduVector3Dd force_vec = PosToForce(posDiff);

	hdMakeCurrentDevice(device_id);
	hdSetDoublev(HD_CURRENT_FORCE, force_vec);

	if (packet) {
		pos_delta = (*packet - *(packet_queue.back())).magnitude();
		packet_queue.push_back(packet);
		if (packet_queue.size() > PREDICTOR_QUEUE_SIZE)
			delete packet_queue.pop_front();
	}
}

void HapticDeviceController::SendState(bool debug = false) {
	// predictive, perception-based packet sending
	HapticPacket packet = PreparePacket();
	hduVector3Dd real_pos = packet->GetPos();

	// predictive packet sending
	hduVector3Dd pred_pos = PredictPos(packet_queue.back());

	// perception-based packet sending
	if !IsPerceptable(pred_pos, real_pos)
		if (debug) printf("%c SendState::packet sending dismissed")
	else
		HDComm->SendPacket(packet);
}

bool HapticDeviceController::IsPerceptable(const hduVector3Dd pred_pos, const hduVector3Dd real_pos) {
	return (pred_pos - real_pos).magnitude() < PREDICTOR_CONSTANT_K * pos_delta;
}

HapticDeviceController::HapticDeviceController(const HHD device_id, const char alias, const HDCommunicator* HDComm) : 
	device_id(device_id), alias(alias), HDComm(HDComm) {
	pos_delta = 0;
}

void HapticDeviceController::tick() {
	hdBeginFrame(device_id);

	if (alias == 'M') {
		SendState();
		UpdateState();
	}
	else if (alias == 'S') {
		UpdateState();
		SendState();
	}
	else {
		printf("Err: Alias should be either M or S\n");
		exit(-1);
	}
	hdEndFrame(device_id);
}
