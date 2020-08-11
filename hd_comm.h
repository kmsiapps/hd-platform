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
	uint32_t lastRCVPacket = 0;

	char rcvbuf[PACKET_SIZE];

	uint32_t packetRCVCounter = 0;

public:
	HHD deviceID;

	HDCommunicator(HHD deviceID, SOCKET socket, sockaddr_in* sock_addr, int32_t sock_addr_size, char alias) {
		this->deviceID = deviceID;
		this->socket = socket; // socket fd
		this->sock_addr = sock_addr; // sock_addr struct ptr
		this->sock_addr_size = sock_addr_size; // sock_addr struct size
		this->alias = alias; // e.g. 'M' for master, 'S' for slave
		sendingPacket = new HapticPacket();
		receivedPacket = new HapticPacket();
	}

	bool sendPacket(HapticPacket *packet, bool debug = false) {
		// send packet to remote device. return if it succeded
		if (sendto(socket, packet->toArray(), packet->getSize(), 0, (sockaddr*)sock_addr, sock_addr_size) != SOCKET_ERROR) {
			if (debug) printf("%c sendingPacket::Success\n", alias);
			return true;
		}
		else {
			printf("%c sendingPacket::Error\n", alias);
			return false;
		}
	}

	HapticPacket* rcvPacket(bool debug = false) {
		// recieve packet from remote device. returns ptr of packet, or NULL if failed
		bool hasRCVed = false;
		HapticPacket* receivedPacket = NULL;
		while (true) {
			uint32_t bytesIn = recvfrom(socket, rcvbuf, sizeof(rcvbuf), 0, (sockaddr*)sock_addr, &sock_addr_size);
			if (bytesIn != SOCKET_ERROR && bytesIn == sizeof(rcvbuf)) {
				receivedPacket = new HapticPacket();
				receivedPacket->update(rcvbuf);
				packetRCVCounter++;

				if (receivedPacket->getCounter() > lastRCVPacket) {
					lastRCVPacket = receivedPacket->getCounter();
				}
				hasRCVed = true;
			}
			else {
				if (debug && hasRCVed) {
					uint32_t RCVpacketCount = receivedPacket->getCounter();
					printf("%c rcvPacket::Delay(%5.2fms) PacketNo(%llu) Loss(%u/%u)\n", alias, (getCurrentTime() - receivedPacket->getTimestamp()) / 1000.0,
						RCVpacketCount, (RCVpacketCount - packetRCVCounter), RCVpacketCount);
					/*
					printf("%c rcvPacket::Pos(%.2f %.2f %.2f) Time(%llu) Packet(%u)\n", alias, *((float*)(rcvbuf + POS_OFFSET)),
						*((float*)(rcvbuf + POS_OFFSET + 4)), *((float*)(rcvbuf + POS_OFFSET + 8)), getRCVTimestamp(), getRCVpacketCount());
					*/
				}
				break;
			}
		}
		return receivedPacket;
	}

	bool isLatestPacket(HapticPacket* packet) {
		// returns if given packet is latest packet received by this communicator.
		return (lastRCVPacket == packet->getCounter());
	}
};
