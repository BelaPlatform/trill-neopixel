#pragma once
#include <Bela.h>
#include <vector>
#include <array>
#include "GpNeoPixels.h"

class BelaAudioNeoPixels : GpNeoPixels {
public:
	BelaAudioNeoPixels() {};
	BelaAudioNeoPixels(const BelaContext* context, const std::vector<unsigned int>& channels);
	int setup(const BelaContext* context, const std::vector<unsigned int>& channels);
	/// RGB values in triplets
	ssize_t send(const uint8_t* rgb, size_t length);
	void process(BelaContext* context);
private:
	std::array<std::vector<uint8_t>,2> data;
	std::vector<unsigned int> channels;
	volatile int dataReady;
	volatile int processingData; // which data is owned by process()
	int transmitting;
	ssize_t readPtr;
	unsigned int trailingZeros;
};
