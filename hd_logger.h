#pragma once

#include <fstream>
#include <string>

#include "hd_time.h"

class Logger {
private:
protected:
	std::ofstream *output_file;
	virtual void writeHeader() {}
public:
	Logger(const std::string &filename) {
		output_file = new std::ofstream(filename);
		*output_file << "EventTime,"; // First column header is always event time
		writeHeader();
		*output_file << "\n";
	}

	virtual ~Logger() {
		delete output_file;
	}

	void log(const std::string &text) {
		if (output_file->is_open()) {
			*output_file << getCurrentTime();
			*output_file << ",";
			*output_file << text;
			*output_file << "\n";
		}
	}

};

class RCVLogger : public Logger {
protected:
	void writeHeader() {
		*output_file << "Predict?,PacketTime,PacketNo,PosX,PosY,PosZ,Loss";
	}
public:
	RCVLogger(const std::string &filename) : Logger(filename) {}
};

class SNDLogger : public Logger {
protected:
	void writeHeader() {
		*output_file << "Predict?,PacketTime,PacketNo,PosX,PosY,PosZ";
	}
public:
	SNDLogger(const std::string &filename) : Logger(filename) {}
};

class ERRLogger : public Logger {
protected:
	void writeHeader() {
		*output_file << "msg";
	}
public:
	ERRLogger(const std::string &filename) : Logger(filename) {}
};
