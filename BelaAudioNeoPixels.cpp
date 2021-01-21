#include "BelaAudioNeoPixels.h"

const unsigned int kMcaspWordLength = 16;
const float kInterTransmissionIntervalS = 0.000500; // datasheet says 280us, let's play it safe
BelaAudioNeoPixels::BelaAudioNeoPixels(const BelaContext* context, const std::vector<unsigned int>& channels) {
	setup(context, channels);
}

int BelaAudioNeoPixels::setup(const BelaContext* context, const std::vector<unsigned int>& channels)
{
	unsigned int mcaspBitClock = context->audioSampleRate * context->audioOutChannels * kMcaspWordLength;
	// some (excessively forgiving) heuristics to figure out if we have the
	// clock we need
	// TODO: be passed in some more details about the McASP
	if(mcaspBitClock < 2800000)
	{
		fprintf(stderr, "McASP bit clock is too slow.\n");
		return -1;
	}
	if(context->audioOutChannels % 16)
	{
		fprintf(stderr, "All McASP slots should be in use\n");
		return -1;
	}

	this->channels = channels;
	for(auto c : channels)
		if(c >= context->audioOutChannels)
			return -1;
	for(auto& d : data)
		d.resize(4096);
	readPtr = 0;
	int ret = GpNeoPixels::setup({
			.clockHz = mcaspBitClock,
			.wordLength = kMcaspWordLength,
			.interWordTimeNs = 0,
			.leadingZerosNs = 0,
			.busBigEndian = true,
			.busMsbBitFirst = true,
		});
	dataReady = 0;
	processingData = 0;
	transmitting = 0;
	trailingZeros = 0;
	return ret;
}

ssize_t BelaAudioNeoPixels::send(const uint8_t* rgb, size_t length)
{
	if(dataReady)
		rt_fprintf(stderr, "Previous data not sent out yet\n");
	dataReady = 0;
	std::vector<uint8_t>& data = this->data[!processingData];
	data.resize(data.capacity());
	ssize_t len;
	while(!gShouldStop && (len = rgbToClk(rgb, length, data.data(), data.size())) < 0) {
		fprintf(stderr, "Message too long, resizing\n");
		data.resize(data.size() * 2);
	}
	data.resize(len);
	if(data.size() & 1)
	{
		data.resize(data.size() + 1);
	}
	dataReady = 1;
	//rt_printf("%u data ready\n", data.size());
	return len;
}

// make a u16 a float that can then be sent out the bus as a s16
static float u16_tofloat_for_s16(uint16_t val)
{
	float n = val;
	if(n >= 32768)
	       n -= 65536;
	return n / 32768.f;
}

void BelaAudioNeoPixels::process(BelaContext* context)
{
	if(!transmitting && dataReady
		&& (trailingZeros / context->audioSampleRate > kInterTransmissionIntervalS))
	{
		dataReady = 0;
		//rt_printf("starting transmission at %llu %d\n", context->audioFramesElapsed, trailingZeros);
		processingData = !processingData;
		transmitting = 1;
		readPtr = 0;
	}
	std::vector<uint8_t>& data = this->data[processingData];
	for(unsigned int n = 0; n < context->audioFrames; ++n)
	{
		for(auto c : channels)
		{
			if(data.size() && readPtr < data.size() - 1)
			{
				uint8_t lsb = data[readPtr++];
				uint8_t msb = data[readPtr++];
				uint16_t val = lsb | (msb << 8);
				audioWrite(context, n, c, u16_tofloat_for_s16(val));
			} else {
				audioWrite(context, n, c, 0);
			}
			if(data.size() == readPtr && transmitting) {
				transmitting = 0;
				trailingZeros = 0;
			}
		}
		if(!transmitting)
			trailingZeros++;
	}
}
