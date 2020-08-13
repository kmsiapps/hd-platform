#pragma once

#include <WS2tcpip.h>
#include <stdio.h>

#include <HD/hd.h>
#include <HDU/hduVector.h>

#include "hd_packet.h"
#include "hd_types.h"
#include "hd_time.h"

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

public:
	HDCommunicator(const HHD device_id, const SOCKET socket,
		sockaddr_in* sock_addr, const int32_t sock_addr_size, const char alias) :
		device_id(device_id), socket(socket), sock_addr(sock_addr), sock_addr_size(sock_addr_size), alias(alias) {}

	bool SendPacket(HapticPacket packet, bool debug = false) {
		// send packet to remote device. return if it succeded
		if (sendto(socket, packet.ToArray(), packet.GetSize(), 0, (sockaddr*)sock_addr, sock_addr_size) != SOCKET_ERROR) {
			if (debug) printf("%c sendingPacket::Success\n", alias);
			return true;
		}
		else {
			printf("%c sendingPacket::Error\n", alias);
			return false;
		}
	}

	HapticPacket* ReceivePacket(bool debug = false) {
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
				if (debug && has_received) {
					uint32_t current_packet_num = received_packet->GetPacketNum();
					printf("%c ReceivePacket::Delay(%5.2fms) PacketNo(%llu) Loss(%u/%u)\n",
						alias, (getCurrentTime() - received_packet->GetTimestamp()) / 1000.0,
						current_packet_num, (current_packet_num - packet_receive_counter), current_packet_num);
					/*
					printf("%c ReceivePacket::Pos(%.2f %.2f %.2f) Time(%llu) Packet(%u)\n", alias, *((float*)(rcvbuf + POS_OFFSET)),
					*((float*)(rcvbuf + POS_OFFSET + 4)), *((float*)(rcvbuf + POS_OFFSET + 8)), getRCVTimestamp(), getcurrent_packet_num());
					*/
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
