#pragma once

#include <list>

#include <HD/hd.h>
#include <HDU/hduVector.h>

#include "hd_comm.h"

#define PREDICTOR_QUEUE_SIZE 2 // packet_queue Size
#define PREDICTOR_CONSTANT_K 0.1 // K in Weber's law

class HapticDeviceController {
private:
	/* Charge (positive/negative) */
	const int kCharge = 1;

	HHD device_id;						// haptic device ID
	char alias;							// alias - 'M' for master, 'S' for slave
	HDCommunicator *hdcomm;				// HDCommunicator for udp packet exchange
	std::list<HapticPacket*> packet_queue;	// queue used for predictive coding
	float pos_delta;						// last movement difference, used for perception based coding

	hduVector3Dd PosToForce(const hduVector3Dd pos)
	{
		double dist = pos.magnitude();

		hduVector3Dd force_vec(0, 0, 0);

		// Attract the kCharge to the center of the sphere.
		force_vec = -0.1*pos; // default = 0.1
		force_vec *= kCharge;

		return force_vec;
	}

	HapticPacket PreparePacket(bool debug = false) {
		/* create packet from current device state. */

		hdBeginFrame(device_id);
		// get current device position
		hduVector3Dd pos;
		hdGetDoublev(HD_CURRENT_POSITION, pos);

		hdEndFrame(device_id);

		HapticPacket sending_packet(pos);

		if (debug)
			printf("%c PreparePacket::Pos(%.1f %.1f %.1f) Sent(%u) Time(%llu)\n",
					alias, pos[0], pos[1], pos[2], sending_packet.GetPacketNum(), sending_packet.GetTimestamp());
		return sending_packet;
	}

	hduVector3Dd PredictPos(const hduVector3Dd base_pos) {
		std::list<HapticPacket*>::iterator iter;
		hduVector3Dd prev(0, 0, 0);
		hduVector3Dd acc = hduVector3Dd(0, 0, 0);

		for (iter = packet_queue.begin(); iter != packet_queue.end(); iter++) {
			// calculate vector slope
			acc += ((**iter).GetPos() - prev);
			prev = (**iter).GetPos();
		}

		acc /= packet_queue.size();
		acc += base_pos;

		return acc;
	}

	void UpdateState() {
		// recieve packet from remote, and update current device's state with the packet
		HapticPacket* packet = hdcomm->ReceivePacket(true);
		hduVector3Dd target_pos(0, 0, 0);
		hduVector3Dd current_pos;
		hdGetDoublev(HD_CURRENT_POSITION, current_pos);

		if (packet == NULL)
			// No received pos
			target_pos = PredictPos(current_pos);
		else if (hdcomm->IsLatestPacket(*packet))
			target_pos = packet->GetPos();

		hduVector3Dd posDiff = current_pos - target_pos;
		hduVector3Dd force_vec = PosToForce(posDiff);

		hdMakeCurrentDevice(device_id);
		hdSetDoublev(HD_CURRENT_FORCE, force_vec);

		if (packet) {
			pos_delta = ((*packet).GetPos() - (*packet_queue.back()).GetPos()).magnitude();
			packet_queue.push_back(packet);
			if (packet_queue.size() > PREDICTOR_QUEUE_SIZE) {
				delete packet_queue.front();
				packet_queue.pop_front();
			}
		}
	}

	void SendState(bool debug = false) {
		// predictive, perception-based packet sending
		HapticPacket packet = PreparePacket();
		hduVector3Dd real_pos = packet.GetPos();
		hduVector3Dd pred_pos;

		// predictive packet sending
		if (packet_queue.back())
			pred_pos = PredictPos(packet_queue.back()->GetPos());
		else
			pred_pos = hduVector3Dd(0, 0, 0);

		// perception-based packet sending
		if (!IsPerceptable(pred_pos, real_pos)) {
			if (debug) printf("%c SendState::packet sending dismissed");
		}
		else {
			hdcomm->SendPacket(packet);
		}
	}

	bool IsPerceptable(const hduVector3Dd pred_pos, const hduVector3Dd real_pos) {
		return (pred_pos - real_pos).magnitude() < PREDICTOR_CONSTANT_K * pos_delta;
	}

public:
	HapticDeviceController(const HHD device_id, const char alias, HDCommunicator* hdcomm) :
		device_id(device_id), alias(alias), hdcomm(hdcomm) {
		pos_delta = 0;
	}

	void tick() {
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
};
