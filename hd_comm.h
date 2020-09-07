#pragma once

#include <WS2tcpip.h>
#include <stdio.h>
#include <string>
#include <sstream>

#include <HD/hd.h>
#include <HDU/hduVector.h>

#include "hd_packet.h"
#include "hd_types.h"
#include "hd_time.h"
#include "hd_logger.h"

class HDCommunicator {
private:
	SOCKET socket;
	sockaddr_in* sock_addr;
	int32_t sock_addr_size;

	char alias;
	char rcvbuf[PACKET_SIZE];
	uint32_t last_received_packet = 0;
	uint32_t packet_receive_counter = 0;
	HHD device_id;
	Logger *sndlogger;
	Logger *rcvlogger;
	Logger *errlogger;

public:
	HDCommunicator(const HHD device_id, const SOCKET socket,
				   sockaddr_in* sock_addr, const int32_t sock_addr_size, const char alias,
				   Logger* sndlogger, Logger* rcvlogger, Logger* errlogger) :
		device_id(device_id), socket(socket), sock_addr(sock_addr), sock_addr_size(sock_addr_size),
		alias(alias), sndlogger(sndlogger), rcvlogger(rcvlogger), errlogger(errlogger) {}

	bool SendPacket(HapticPacket* packet, bool debug) {
		// send packet to remote device. return if it succeded
		if (sendto(socket, packet->ToArray(), packet->GetSize(), 0, (sockaddr*)sock_addr, sock_addr_size) != SOCKET_ERROR) {
			if (debug) {
				std::stringstream msg_stream;
				hduVector3Dd &packet_pos = packet->GetPos();

				// Predict? , PacketTime, PacketNo, PosX, PosY, PosZ
				msg_stream << "," << 0 << "," << packet->GetTimestamp() << "," << packet->GetPacketNum() << "," <<
					packet_pos[0] << "," << packet_pos[1] << "," << packet_pos[2];

				sndlogger->log(msg_stream.str());
			}
			return true;
		}
		else {
			errlogger->log("Packet send failed!");
			return false;
		}
	}

	HapticPacket* ReceivePacket(bool debug = true) {
		// recieve packet from remote device. returns ptr of packet, or NULL if failed
		bool has_received = false;
		HapticPacket* received_packet = NULL;
		while (true) {
			uint32_t bytesIn = recvfrom(socket, rcvbuf, sizeof(rcvbuf), 0, (sockaddr*)sock_addr, &sock_addr_size);
			if (bytesIn != SOCKET_ERROR && bytesIn == sizeof(rcvbuf)) {
				received_packet = new HapticPacket(rcvbuf);
				packet_receive_counter++;

				if (received_packet->GetPacketNum() > last_received_packet) {
					last_received_packet = received_packet->GetPacketNum();
				}
				has_received = true;
			}
			else {
				if (has_received && debug) {
					uint32_t current_packet_num = received_packet->GetPacketNum();
					std::stringstream msg_stream;

					// Predict? , PacketTime, PacketNo, PosX, PosY, PosZ, Loss
					msg_stream << "," << 0 << "," << received_packet->GetTimestamp() << "," << current_packet_num << "," <<
					*((float*)(rcvbuf + POS_OFFSET)) << "," << *((float*)(rcvbuf + POS_OFFSET + 4)) << "," << *((float*)(rcvbuf + POS_OFFSET + 8)) <<
					(current_packet_num - packet_receive_counter) << "/" << current_packet_num;

					rcvlogger->log(msg_stream.str());
				}
				break;
			}
		}
		return received_packet;
	}

	bool IsLatestPacket(HapticPacket packet) {
		// returns if given packet is latest packet received by this communicator.
		return (last_received_packet == packet.GetPacketNum());
	}
};
