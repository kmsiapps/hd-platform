#pragma once

#include <list>

#include <HD/hd.h>
#include <HDU/hduVector.h>

#include "hd_comm.h"

#define PREDICTOR_QUEUE_SIZE 5 // received_queue Size
#define PREDICTOR_CONSTANT_K 0.3 // K in Weber's law
#define FORCE_STRENGTH 0.3

class HapticDeviceController {
private:
	/* Charge (positive/negative) */
	const int kCharge = 1;

	HHD device_id;								// haptic device ID
	char alias;									// alias - 'M' for master, 'S' for slave
	HDCommunicator *hdcomm;						// HDCommunicator for udp packet exchange
	std::list<HapticPacket*> received_queue;	// queue used for predictive coding
	std::list<HapticPacket*> sent_queue;		// queue used for predictive coding

	float pos_delta;							// last movement difference, used for perception based coding

	hduVector3Dd PosToForce(const hduVector3Dd pos)
	{
		double dist = pos.magnitude();

		hduVector3Dd force_vec(0, 0, 0);

		// Attract the kCharge to the center of the sphere.
		force_vec = -1 * FORCE_STRENGTH * pos;
		force_vec *= kCharge;

		return force_vec;
	}

	HapticPacket* PreparePacket(bool debug = false) {
		/* create packet from current device state. */

		// get current device position
		hduVector3Dd pos;
		hdGetDoublev(HD_CURRENT_POSITION, pos);

		HapticPacket *sending_packet = new HapticPacket(pos);

		if (debug)
			printf("%c PreparePacket::Pos(%.1f %.1f %.1f) Sent(%u) Time(%llu)\n",
					alias, pos[0], pos[1], pos[2], sending_packet->GetPacketNum(), sending_packet->GetTimestamp());
		return sending_packet;
	}

	hduVector3Dd PredictPos(const hduVector3Dd base_pos, std::list<HapticPacket*>& queue) {
		std::list<HapticPacket*>::iterator iter;
		hduVector3Dd prev(0, 0, 0);
		if (queue.size() > 1)
			prev = queue.front()->GetPos();
		else
			return base_pos;

		hduVector3Dd acc(0, 0, 0);

		for (iter = ++queue.begin(); iter != queue.end(); ++iter) {
			// calculate vector slope
			acc += ((**iter).GetPos() - prev);
			prev = (**iter).GetPos();
		}

		acc /= queue.size() - 1;
		acc += base_pos;

		return acc;
	}

	void UpdateState(bool debug=false) {
		// recieve packet from remote, and update current device's state with the packet
		HapticPacket* packet = hdcomm->ReceivePacket(debug);
		hduVector3Dd target_pos(0, 0, 0);
		hduVector3Dd current_pos;
		hdGetDoublev(HD_CURRENT_POSITION, current_pos);

		if (packet == NULL) {
			// No received pos
			//printf("%c::UpdateState::Use pos prediction(No packet received)\n", alias);
			hduVector3Dd base_pos = received_queue.back() ? received_queue.back()->GetPos() : current_pos;
			target_pos = PredictPos(base_pos, received_queue);
			if (debug) printf("%c UpdateState::Using predicted position\n", alias);
		}
		else {
			target_pos = packet->GetPos();
		}

		hduVector3Dd posDiff = current_pos - target_pos;
		hduVector3Dd force_vec = PosToForce(posDiff);
		hdSetDoublev(HD_CURRENT_FORCE, force_vec);

		if (packet) {
			hduVector3Dd prevPos(0, 0, 0);
			if (received_queue.size() > 0)
				prevPos = (*received_queue.back()).GetPos();
			pos_delta = ((*packet).GetPos() - prevPos).magnitude();
			received_queue.push_back(packet);
			if (received_queue.size() > PREDICTOR_QUEUE_SIZE) {
				delete received_queue.front();
				received_queue.pop_front();
			}
		}
	}

	void SendState(bool debug = false) {
		// predictive, perception-based packet sending
		HapticPacket* packet = PreparePacket(debug);
		hduVector3Dd real_pos = packet->GetPos();
		hduVector3Dd prev_pos;

		// predictive packet sending
		if (sent_queue.back())
			prev_pos = sent_queue.back()->GetPos();
		else
			prev_pos = hduVector3Dd(0, 0, 0);

		// perception-based packet sending
		if (!IsPerceptable(PredictPos(prev_pos, sent_queue), real_pos)) {
			if (debug) printf("%c SendState::packet sending dismissed\n", alias);
		}
		else {
			hdcomm->SendPacket(packet);
			sent_queue.push_back(packet);
			if (sent_queue.size() > PREDICTOR_QUEUE_SIZE) {
				delete sent_queue.front();
				sent_queue.pop_front();
			}
		}
	}

	bool IsPerceptable(const hduVector3Dd pred_pos, const hduVector3Dd real_pos) {
		float i = (pred_pos - real_pos).magnitude();
		float delta_i = PREDICTOR_CONSTANT_K * (pos_delta + 0.0001);
		return i >= delta_i;
	}

public:
	HapticDeviceController(const HHD device_id, const char alias, HDCommunicator* hdcomm) :
		device_id(device_id), alias(alias), hdcomm(hdcomm) {
		pos_delta = 0;
	}

	HapticDeviceController() {

	}

	void tick() {
		hdBeginFrame(device_id);
		hdMakeCurrentDevice(device_id);
		
		if (alias == 'M') {
			SendState();
			UpdateState(true);
		}
		else if (alias == 'S') {
			UpdateState(true);
			SendState();
		}
		else {
			printf("Err: Alias should be either M or S\n");
			exit(-1);
		}
		hdEndFrame(device_id);
	}
};
