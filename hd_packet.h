#pragma once

#include <HDU/hduVector.h>

#include "hd_types.h"
#include "hd_time.h"

#define POS_OFFSET 0
#define COUNT_OFFSET sizeof(pos_t) * 3
#define TIMESTAMP_OFFSET (COUNT_OFFSET + sizeof(cnt_t))
#define PACKET_SIZE (TIMESTAMP_OFFSET + sizeof(ts_t))

class HapticPacket {
// 0       4       8       12          16          24
// #################################################
// # Pos_x # Pos_y # Pos_z # PacketCnt # Timestamp #
// #################################################

private:
	char buffer[PACKET_SIZE];

public:
	HapticPacket() {
		// initialize packet count data
		*((cnt_t*)(buffer + COUNT_OFFSET)) = 0;
		*((pos_t*)(buffer + POS_OFFSET)) = 0;
		*((pos_t*)(buffer + POS_OFFSET + sizeof(pos_t))) = 0;
		*((pos_t*)(buffer + POS_OFFSET + 2 * sizeof(pos_t))) = 0;
	}
;	void update(hduVector3Dd* pos) {
		/* update from position data. also automatically updates counter and timestamp. */

		// update position
		*((pos_t*)(buffer + POS_OFFSET)) = *pos[0];
		*((pos_t*)(buffer + POS_OFFSET + sizeof(pos_t))) = *pos[1];
		*((pos_t*)(buffer + POS_OFFSET + 2 * sizeof(pos_t))) = *pos[2];
		
		// update packet counter
		*((cnt_t*)(buffer + COUNT_OFFSET)) += 1;

		// update timestamp
		*((ts_t*)(buffer + TIMESTAMP_OFFSET)) = getCurrentTime();
	}

	void update(char *source) {
		/* update from array. array should be in proper format */
		memcpy(buffer, source, PACKET_SIZE);
	}

	char* toArray() {
		/* return packet in serialized form (char *array) */
		return buffer;
	}

	hduVector3Dd* getPos() {
		/* return pos vector from packet */
		return new hduVector3Dd(
			*((pos_t*)(buffer + POS_OFFSET)),
			*((pos_t*)(buffer + POS_OFFSET + sizeof(pos_t))),
			*((pos_t*)(buffer + POS_OFFSET + sizeof(pos_t) * 2))
		)
	}

	ts_t getTimestamp() {
		/* get timestamp from packet */
		return *((ts_t*)(buffer + TIMESTAMP_OFFSET));
	}

	cnt_t getCounter() {
		/* get counter from packet */
		return *((cnt_t*)(buffer + COUNT_OFFSET));
	}

	uint32_t getSize() {
		/* get Packet size (Bytes) */
		return PACKET_SIZE;
	}
};
