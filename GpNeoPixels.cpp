#include "GpNeoPixels.h"
#include <string.h>
#include <stdio.h>
#include <cmath>

const uint8_t kBitsPerByte = 8;
// The specs (WS2812B-2020 datasheet) say:
// T0H 0 code, high voltage time 220ns~380ns
// T1H 1 code, high voltage time 580ns~1µs
// T0L 0 code, low voltage time 580ns~1µs
// T1L 1 code, low voltage time 220ns~420ns
// RES Frame unit, low voltage time >280µs
// WS2812B v1.0 on the other hand says:
// TH+TL = 1.25µs ± 600ns
// T0H 0 code, high voltage time 0.4us ±150ns
// T1H 1 code, high voltage time 0.8us ±150ns
// T0L 0 code, low voltage time 0.85us ±150ns
// T1L 1 code, low voltage time 0.45us ±150ns
// More recent datasheets say

// We pick below some conservative values for WS2812B
typedef struct {
	float min;
	float max;
} T_t;

const T_t TH[2] = {
	{.min = 200, .max = 400}, // T0H
	{.min = 700, .max = 900}, // T1H
};

const T_t TL[2] = {
	{.min = 750, .max = 950}, // T0L
	{.min = 350, .max = 550}, // T1L
};

GpNeoPixels::GpNeoPixels(const Settings& settings) {
	setup(settings);
}

int GpNeoPixels::setup(const Settings& settings) {
	periodNs = 1000000000.0 / settings.clockHz;
	wordLength = settings.wordLength;
	interWordTimeNs = settings.interWordTimeNs;
	leadingZerosClk = std::ceil((settings.leadingZerosNs / periodNs));
	busBigEndian = settings.busBigEndian;
	busMsbBitFirst = settings.busMsbBitFirst;
	return 0;
}

// In {read,write}BitField, `offset` is the number of bits.
// This is decomposed into `i` (the byte number) and `position` (the bit
// number).

// In readBitField we want `0` to be the MSB and `7` to be the LSB, which maps
// nicely to the NeoPixel bus protocol.
static bool readBitField(const uint8_t* data, uint32_t offset) {
	uint8_t position = kBitsPerByte - (offset % kBitsPerByte);
	unsigned int i = offset / kBitsPerByte;
	return data[i] & (1 << position);
}

// This function is used to write into memory the bit sequence that will be
// shifted out over the bus.
// We interpret the offset differently according to whether the bus in use is
// MSBit or LSBit first.
// Bus endianness is handled elsewhere.
void GpNeoPixels::writeBitField(uint8_t* data, uint32_t offset, bool value) {
	uint8_t position;
	if(busMsbBitFirst)
		position = kBitsPerByte - 1 - (offset % kBitsPerByte);
	else
		position = offset % kBitsPerByte;
	unsigned int i = offset / kBitsPerByte;
	if(value)
		data[i] |= 1 << position;
	else
		data[i] &= ~(1 << position);
}

ssize_t GpNeoPixels::rgbToClk(const uint8_t* rgb, size_t numRgb, uint8_t* out, size_t numOut)
{
	uint32_t clk = 0;
	// clear the head if needed in a big chunk
	// we may end up clearing a few more bits than needed but it's OK
	size_t clearSize = leadingZerosClk / kBitsPerByte + 1;
	if(clearSize > numOut)
		return -1;
	memset(out, 0, clearSize);
	clk += leadingZerosClk;
	// ensure we have complete RGB sets
	numRgb = numRgb - (numRgb % 3);
	for(uint32_t inBit = 0; inBit < numRgb * kBitsPerByte; ++inBit) {
		// data comes in as RGB but needs to be shuffled into GRB for the WS2812B
		uint32_t actualInBit;
		uint8_t remainder = inBit % (3 * kBitsPerByte);
		if(remainder < kBitsPerByte)
			actualInBit = inBit + kBitsPerByte;
		else if(remainder < kBitsPerByte * 2)
			actualInBit = inBit - kBitsPerByte;
		else
			actualInBit = inBit;
		bool value = readBitField(rgb, actualInBit);
		const T_t* Ts[2] = {TH, TL};
		for(unsigned int t = 0; t < 2; ++t) {
			const T_t* T = Ts[t];
			bool highLow = (0 == t) ? true : false;
			float time = 0;
			while(time < T[value].min) {
				bool wordEnd = (wordLength - 1 == clk);
				writeBitField(out, clk, highLow);
				time += periodNs;
				if(wordEnd)
					time += interWordTimeNs;
				++clk;
				if(clk / kBitsPerByte > numOut)
					return -1;
			}
			if(time > T[value].max) {
				printf("Error: expected %f but got %f\n", T[value].max, time);
				return 0;
			}

		}
	}
	// uint8_t* data = out;
	// data[0] = 0x1;
	// data[1] = 0x5;
	// data[2] = 0x3;
	// data[3] = 11;
	// data[4] = data[5] = data[6] = data[7] = 0;
	ssize_t numBytes = ((clk + wordLength - 1) / wordLength) * wordLength / kBitsPerByte; // round up to the next word
	if(numBytes > numOut) // just in case numOut was not a multiple, we now no longer fit
		return -1;
	// TODO: add write zeros to the end
	// format data if needed
	// If we have more than 1 byte per word we need to shuffle them
	// around so that they are output in the correct order
	unsigned int bytesPerWord = wordLength / kBitsPerByte;
	for(unsigned int n = 0; n < numBytes; n += bytesPerWord) {
		for(unsigned int b = 0; b < bytesPerWord / 2; ++b) {
			uint8_t tmp = out[n + b];
			out[n + b] = out[n + bytesPerWord - 1 - b];
			out[n + bytesPerWord - 1 - b] = tmp;
		}
	}
	return numBytes;
}


