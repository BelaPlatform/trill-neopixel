#pragma once
#include <stdint.h>
#include <sys/types.h>

class GpNeoPixels
{
protected:
	struct Settings {
		unsigned int clockHz;
		unsigned int wordLength;
		float interWordTimeNs;
		float leadingZerosNs;
		bool busBigEndian;
		bool busMsbBitFirst;
	};
	GpNeoPixels() {};
	GpNeoPixels(const Settings& settings);
	int setup(const Settings& settings);
	ssize_t rgbToClk(const uint8_t* rgb, size_t numRgb, uint8_t* out, size_t numOut);
private:
	unsigned int leadingZerosClk;
	void writeBitField(uint8_t* data, uint32_t offset, bool value);
	float periodNs;
	float interWordTimeNs;
	unsigned int wordLength;
	bool busBigEndian;
	bool busMsbBitFirst;
};
