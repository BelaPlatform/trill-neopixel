#include "TrillRackInterface.h"
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include <libraries/Trill/CentroidDetection.h> // include this above NeoPixel or "HEX" gets screwed up
#include "LedSliders.h"
#include "NeoPixel.h"
#include <cmath>

TrillRackInterface tri(0, 0, 1);
const unsigned int kNumLeds = 16;
NeoPixel np(kNumLeds, 0, NEO_RGB);
Trill trill;
CentroidDetection cd;

const unsigned int kLoopSleepTimeUs = 10000;

const unsigned int kNumPads = 24;
unsigned int padsToOrderMap[kNumPads] = {
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
};

#define ON_BOARD_SENSOR // use on-board sensor. Otherwise, use an external Trill Bar
#define LEDSLIDERS // use ledSliders for single- or multi- slider + viz. Otherwise, maually manage LEDs and CentroidDetection

LedSliders ledSliders;

bool tr_setup()
{
	np.begin();
#ifdef ON_BOARD_SENSOR
	if(trill.setup(1, Trill::FLEX, 0x50))
		return false;
#ifndef LEDSLIDERS
	cd.setup({padsToOrderMap, padsToOrderMap + kNumPads / 2}, 4, 3200);
#endif // !LEDSLIDERS
#else // ON_BOARD_SENSOR
	if(trill.setup(1, Trill::BAR, 0x23))
		return false;
#ifndef LEDSLIDERS
	cd.setup(24, 4, 3200);
#endif // !LEDSLIDERS
#endif // ON_BOARD_SENSOR
	trill.setMode(Trill::DIFF);
	trill.printDetails();

#ifdef LEDSLIDERS
	unsigned int guardPads = 1;
	// set up ledSliders with two sub-sliders
	ledSliders.setup({
		.order = {padsToOrderMap, padsToOrderMap + kNumPads},
		.sizeScale = 3200,
		.maxNumCentroids = {1, 1},
		.boundaries = {
			//{.firstPad = 0, .lastPad = kNumPads,
			//.firstLed = 0, .lastLed = kNumLeds, },
			{.firstPad = 0, .lastPad = kNumPads / 2 - guardPads,
			.firstLed = 0, .lastLed = kNumLeds / 2, },
			{.firstPad = kNumPads / 2 + guardPads, .lastPad = kNumPads,
			.firstLed = kNumLeds / 2, .lastLed = kNumLeds, },
		},
		.np = &np,
	});
	for(unsigned int n = 0; n < ledSliders.sliders.size(); ++n)
	{
		unsigned int m = n % 3;
		// set each subslider to R, G, B etc
		rgb_t color = {(uint8_t)((0 == m) * 255), uint8_t((1 == m) * 255), uint8_t((2 == m) * 255)};
		ledSliders.sliders[n].setColor(color);
		ledSliders.sliders[n].setLedMode(LedSlider::AUTO_CENTROIDS);
	}
#endif // LEDSLIDERS
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

template <typename T, typename U>
void sort(T* out, U* in, unsigned int* order, unsigned int size)
{
	for(unsigned int n = 0; n < size; ++n)
		out[n] = in[order[n]];
}

void tr_loop()
{
	trill.readI2C();
#ifdef LEDSLIDERS
	ledSliders.process(trill.rawData.data());
	np.show(); // actually display the updated LEDs
	float fingerPos = ledSliders.sliders[0].compoundTouchLocation();
	float touchSize = ledSliders.sliders[0].compoundTouchSize();
#else // LEDSLIDERS
	float* tmp = trill.rawData.data();
	float pads[kNumPads];
	sort(pads, tmp, padsToOrderMap, kNumPads);
#if 1 // debug order
	for(unsigned int n = 0; n < kNumPads; ++n)
		printf("%4d ", n);
	printf("\n");
	for(unsigned int n = 0; n < kNumPads; ++n)
		printf("%4.0f ", tmp[n] * 4096);
	printf("\n");
	for(unsigned int n = 0; n < kNumPads; ++n)
		printf("%4.0f ", pads[n] * 4096);
	printf("\n");
	printf("\n");
#endif
	float bright[kNumLeds];
	// bright is a scratchpad for LED values
	resample(bright, kNumLeds, pads, kNumPads);
#if 0 // debug resample
	for(unsigned int n = 0; n < kNumPads; ++n)
		printf("%.2f ", pads[n]);
	printf("\n");
	for(unsigned int n = 0; n < kNumLeds; ++n)
		printf("%.2f	", bright[n]);
	printf("\n\n");
#endif
	// Set colour depending on activation
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, bright[n] * (n >= 8) * 255, bright[n] * (n < 8) * 255, 0);
	np.show(); // actually display the updated LEDs

	cd.process(trill.rawData.data());
	float fingerPos = cd.compoundTouchLocation();
	float touchSize = cd.compoundTouchSize();
#endif // LEDSLIDERS
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
