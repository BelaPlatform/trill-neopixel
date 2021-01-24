#include "TrillRackInterface.h"
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include "NeoPixel.h"
#include <cmath>

static uint8_t kNumLeds = 16;
TrillRackInterface tri(0, 0, 1);
NeoPixel np(kNumLeds, 0, NEO_RGB);
Trill trill;
const unsigned int kLoopSleepTimeUs = 10000;

bool tr_setup()
{
	np.begin();
	if(trill.setup(1, Trill::FLEX, 0x50))
			return false;
	trill.setMode(Trill::DIFF);
	trill.printDetails();
	return true;
}

void resample(float* out, unsigned int nOut, float* in, unsigned int nIn)
{
#if 0 // naive: sum all input energy into outputs
	for(unsigned int no = 0; no < nOut; ++no) {
		out[no] = 0;
		unsigned int niStart = no * nIn / nOut;
		unsigned int niEnd = (no + 1) * nIn / nOut;
		for(unsigned int ni = niStart; ni < niEnd; ++ni) {
			out[no] += in[ni];
		}
	}
#endif
#if 1 // weighted sum
	// How many accompanying LEDs are on
	float r = 2;
	for(int no = 0; no < nOut; ++no) {
		out[no] = 0;
		for(int ni = 0; ni < nIn; ++ni) {
			float fracInIdx = no * nIn / (float)nOut;
			float weight = (1 - (std::abs(fracInIdx - ni) / r));
			weight = std::max(0.f, weight); // clip to 0
			out[no] += in[ni] * weight;
		}
	}
#endif
}

unsigned int padsToOrderMap[30] = {
	0,
	1,
	2,
	3,
	4,
	5,
	6,
	7,
	8,
	9,
	10,
	11,
	12,
	13,
	28,
	27,
	26,
	25,
	15,
	17,
	18,
	19,
	20,
	24,
	21,
	22,
	23,
	29,
	14,
	16,
};

template <typename T, typename U>
void sort(T* out, U* in, unsigned int* order, unsigned int size)
{
	for(unsigned int n = 0; n < size; ++n)
		out[n] = in[order[n]];
}

void tr_loop()
{
	trill.readI2C();
	unsigned int numPads = 24;//trill.rawData.size();
	float* tmp = trill.rawData.data();
	float pads[numPads];
	sort(pads, tmp, padsToOrderMap, numPads);
#if 0 // debug order
	for(unsigned int n = 0; n < numPads; ++n)
		printf("%4d ", n);
	printf("\n");
	for(unsigned int n = 0; n < numPads; ++n)
		printf("%4.0f ", pads[n] * 4096);
	printf("\n");
	for(unsigned int n = 0; n < numPads; ++n)
		printf("%4.0f ", pads[n] * 4096);
	printf("\n");
	printf("\n");
#endif
	float bright[kNumLeds];
	// bright is a scratchpad for LED values
	resample(bright, kNumLeds, pads, numPads);
#if 0 // debug resample
	for(unsigned int n = 0; n < numPads; ++n)
		printf("%.2f ", pads[n]);
	printf("\n");
	for(unsigned int n = 0; n < kNumLeds; ++n)
		printf("%.2f	", bright[n]);
	printf("\n\n");
#endif
	// Set colour depending on activation
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, bright[n] * 255, 0, 0);
	np.show(); // actually display the updated LEDs

	// find peak (i.e.: "finger position" (really poor man centroid detection)
	unsigned int pk = 0;
	for(unsigned int n = 0; n < numPads; ++n)
		if(pads[n] > pads[pk])
			pk = n;
	float fingerPos = pk / (float)numPads;
	// even poorer man's "touch size"
	float touchSize = pads[pk];
	 // read analog in.
	float anIn = tri.analogRead();
	// write outs
	tri.analogWrite(0, fingerPos);
	tri.analogWrite(1, touchSize);

	tri.scopeWrite(0, anIn);
	tri.scopeWrite(1, fingerPos);
	tri.scopeWrite(2, touchSize);
	usleep(kLoopSleepTimeUs);
}
