#include "TrillRackInterface.h"
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include <libraries/Trill/CentroidDetection.h> // include this above NeoPixel or "HEX" gets screwed up
#include "LedSliders.h"
#include "NeoPixel.h"
#include <cmath>
#ifdef STM32
#define rt_printf printf
#endif // STM32

// Robbie's Variables
// ------------------
#include <libraries/Oscillator/Oscillator.h>

Oscillator oscillator1;
Oscillator oscillator2;


// Mode switching
int gMode = 0;
int gDiIn1Last = 0;
int gCounter = 0;
int gDiIn2Last = 0;
int gSubMode = 0;

// Recording the gesture
int gEndOfGesture = 0; // store gesture length
int gRestartCount = 0;
#define gMaxRecordLength 10000
float gTouchPositionRecording[gMaxRecordLength];
int gPrevTouchPresent = 0; // store whether a touch was previously present

// Recording two gestures at once
unsigned int gPrevTouchPresentDualLFO[2] = {0};
int gCounterDualLFO[2] = {0};
int gEndOfGestureDualLFO[2] = {0};
float gTouchPositionRecordingDualLFO[2][gMaxRecordLength] = {0, 0};
int gRestartCountDualLFO[2] = {0};

// Master clock
int gMtrClkCounter = 0;
int gMtrClkTimePeriod = 260;
int gMtrClkTrigger = 0;
int gMtrClkTriggerLED = 0;
float gMtrClkTimePeriodScaled = 0;

// LED Flash Event
double gEndTime = 0; //for pulse length
double gPulseLength = 40;

// Div Mult clock
int gDivMultClkCounter = 0;
int gDivMultClkTimePeriod = 160;
int gDivMultClkTrigger = 0;
double gDivMultEndTime = 0;

// Dual LFOS
float gDivisionPoint = 0;
// ------------------


TrillRackInterface tri(0, 0, 1, 15, 14, 3);
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
	for(unsigned int no = 0; no < nOut; ++no) {
		out[no] = 0;
		for(unsigned int ni = 0; ni < nIn; ++ni) {
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

void master_clock(float tempoControl)
{
	gMtrClkTrigger = 0;
	
	gMtrClkTimePeriodScaled = tempoControl * gMtrClkTimePeriod;
	
	if (gMtrClkCounter >= tempoControl * gMtrClkTimePeriod) {
		
		gMtrClkTrigger = 1;
		gMtrClkTriggerLED = 1;
		gEndTime = tri.getTimeMs() + gPulseLength;
		// rt_printf("BANG: %f  %f\n",tri.getTimeMs(), gEndTime);
		gMtrClkCounter = 0;
	}
	gMtrClkCounter++;
	
	if(gEndTime < tri.getTimeMs()) {
    	gMtrClkTriggerLED = 0;
    }
}

void divmult_clock(int trigger, float tempoControl)
{
	if (gDivMultClkCounter >= tempoControl * gDivMultClkTimePeriod) {
		
		gDivMultClkTrigger = 1;
		gDivMultEndTime = tri.getTimeMs() + gPulseLength;
		// rt_printf("BANG: %f  %f\n",tri.getTimeMs(), gDivMultEndTime);
		gDivMultClkCounter = 0;
	}
	gDivMultClkCounter++;
	
	if(gDivMultEndTime < tri.getTimeMs()) {
    	gDivMultClkTrigger = 0;
    }
}

// MODE 1: DIRECT CONTROL / SINGLE SLIDER
void mode1_setup()
{
	unsigned int guardPads = 1;
	// set up ledSliders with two sub-sliders
	ledSliders.setup({
		.order = {padsToOrderMap, padsToOrderMap + kNumPads},
		.sizeScale = 3200,
		.boundaries = {
			{.firstPad = 0, .lastPad = kNumPads,
			.firstLed = 0, .lastLed = kNumLeds, },
			// {.firstPad = 0, .lastPad = kNumPads / 2 - guardPads,
			// .firstLed = 0, .lastLed = kNumLeds / 2, },
			// {.firstPad = kNumPads / 2 + guardPads, .lastPad = kNumPads,
			// .firstLed = kNumLeds / 2, .lastLed = kNumLeds, },
		},
		.maxNumCentroids = {1},
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
	
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 255, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 255, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
}

// MODE 2: DIRECT CONTROL / DOUBLE SLIDER
void mode2_setup()
{
	unsigned int guardPads = 1;
	// set up ledSliders with two sub-sliders
	ledSliders.setup({
		.order = {padsToOrderMap, padsToOrderMap + kNumPads},
		.sizeScale = 3200,
		.boundaries = {
			//{.firstPad = 0, .lastPad = kNumPads,
			//.firstLed = 0, .lastLed = kNumLeds, },
			{.firstPad = 0, .lastPad = kNumPads / 2 - guardPads,
			.firstLed = 0, .lastLed = kNumLeds / 2, },
			{.firstPad = kNumPads / 2 + guardPads, .lastPad = kNumPads,
			.firstLed = kNumLeds / 2, .lastLed = kNumLeds, },
		},
		.maxNumCentroids = {1, 1},
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
	
	for(unsigned int n = 0; n < kNumLeds/2-guardPads; ++n)
		np.setPixelColor(n, 255, 0, 0);
	for(unsigned int n = kNumLeds/2; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 255, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds/2-guardPads; ++n)
		np.setPixelColor(n, 255, 0, 0);
	for(unsigned int n = kNumLeds/2; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 255, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
}

// MODE 3: SINGLE SLIDER / LOOP GESTURE
void mode3_setup()
{
	unsigned int guardPads = 1;
	// set up ledSliders with two sub-sliders
	ledSliders.setup({
		.order = {padsToOrderMap, padsToOrderMap + kNumPads},
		.sizeScale = 3200,
		.boundaries = {
			{.firstPad = 0, .lastPad = kNumPads,
			.firstLed = 0, .lastLed = kNumLeds, },
			// {.firstPad = 0, .lastPad = kNumPads / 2 - guardPads,
			// .firstLed = 0, .lastLed = kNumLeds / 2, },
			// {.firstPad = kNumPads / 2 + guardPads, .lastPad = kNumPads,
			// .firstLed = kNumLeds / 2, .lastLed = kNumLeds, },
		},
		.maxNumCentroids = {1},
		.np = &np,
	});
	for(unsigned int n = 0; n < ledSliders.sliders.size(); ++n)
	{
		unsigned int m = n % 3;
		// set each subslider to R, G, B etc
		rgb_t color = {(uint8_t)(255), uint8_t(255), uint8_t(255)};
		ledSliders.sliders[n].setColor(color);
		ledSliders.sliders[n].setLedMode(LedSlider::MANUAL_CENTROIDS);
	}
	
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 255, 255, 255);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 255, 255, 255);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
}

// MODE 4: LFO / DOUBLE SLIDER
void mode4_setup()
{
	unsigned int guardPads = 1;
	// set up ledSliders with two sub-sliders
	ledSliders.setup({
		.order = {padsToOrderMap, padsToOrderMap + kNumPads},
		.sizeScale = 3200,
		.boundaries = {
			//{.firstPad = 0, .lastPad = kNumPads,
			//.firstLed = 0, .lastLed = kNumLeds, },
			{.firstPad = 0, .lastPad = kNumPads / 2 - guardPads,
			.firstLed = 0, .lastLed = kNumLeds / 2, },
			{.firstPad = kNumPads / 2 + guardPads, .lastPad = kNumPads,
			.firstLed = kNumLeds / 2, .lastLed = kNumLeds, },
		},
		.maxNumCentroids = {1, 1},
		.np = &np,
	});
	for(unsigned int n = 0; n < ledSliders.sliders.size(); ++n)
	{
		unsigned int m = n % 3;
		// set each subslider to R, G, B etc
		rgb_t color = {(uint8_t)((0 == m) * 255), uint8_t((0 == m) * 255), uint8_t((1 == m) * 255)};
		ledSliders.sliders[n].setColor(color);
		ledSliders.sliders[n].setLedMode(LedSlider::MANUAL_CENTROIDS);
	}
	
	for(unsigned int n = 0; n < kNumLeds/2-guardPads; ++n)
		np.setPixelColor(n, 255, 255, 0);
	for(unsigned int n = kNumLeds/2; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 255);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds/2-guardPads; ++n)
		np.setPixelColor(n, 255, 255, 0);
	for(unsigned int n = kNumLeds/2; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 255);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
}

void mode5_setup()
{
	oscillator1.setup(1000, Oscillator::triangle);
	oscillator2.setup(1000, Oscillator::triangle);
	
	unsigned int guardPads = 1;
	// set up ledSliders with two sub-sliders
	ledSliders.setup({
		.order = {padsToOrderMap, padsToOrderMap + kNumPads},
		.sizeScale = 3200,
		.boundaries = {
			{.firstPad = 0, .lastPad = kNumPads,
			.firstLed = 0, .lastLed = kNumLeds, },
			// {.firstPad = 0, .lastPad = kNumPads / 2 - guardPads,
			// .firstLed = 0, .lastLed = kNumLeds / 2, },
			// {.firstPad = kNumPads / 2 + guardPads, .lastPad = kNumPads,
			// .firstLed = kNumLeds / 2, .lastLed = kNumLeds, },
		},
		.maxNumCentroids = {1},
		.np = &np,
	});
	for(unsigned int n = 0; n < ledSliders.sliders.size(); ++n)
	{
		unsigned int m = n % 3;
		// set each subslider to R, G, B etc
		rgb_t color = {(uint8_t)(255), uint8_t(255), uint8_t(255)};
		ledSliders.sliders[n].setColor(color);
		ledSliders.sliders[n].setLedMode(LedSlider::MANUAL_CENTROIDS);
	}
	
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 255, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 255, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
}

void mode6_setup()
{
	oscillator1.setup(1000, Oscillator::square);
	oscillator2.setup(1000, Oscillator::square);
	
	unsigned int guardPads = 1;
	// set up ledSliders with two sub-sliders
	ledSliders.setup({
		.order = {padsToOrderMap, padsToOrderMap + kNumPads},
		.sizeScale = 3200,
		.boundaries = {
			{.firstPad = 0, .lastPad = kNumPads,
			.firstLed = 0, .lastLed = kNumLeds, },
			// {.firstPad = 0, .lastPad = kNumPads / 2 - guardPads,
			// .firstLed = 0, .lastLed = kNumLeds / 2, },
			// {.firstPad = kNumPads / 2 + guardPads, .lastPad = kNumPads,
			// .firstLed = kNumLeds / 2, .lastLed = kNumLeds, },
		},
		.maxNumCentroids = {1},
		.np = &np,
	});
	for(unsigned int n = 0; n < ledSliders.sliders.size(); ++n)
	{
		unsigned int m = n % 3;
		// set each subslider to R, G, B etc
		rgb_t color = {(uint8_t)(255), uint8_t(255), uint8_t(255)};
		ledSliders.sliders[n].setColor(color);
		ledSliders.sliders[n].setLedMode(LedSlider::MANUAL_CENTROIDS);
	}
	
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 255, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 255, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
}

// MODE 7: ENVELOPE GENERATOR
void mode7_setup()
{
	unsigned int guardPads = 1;
	// set up ledSliders with two sub-sliders
	ledSliders.setup({
		.order = {padsToOrderMap, padsToOrderMap + kNumPads},
		.sizeScale = 3200,
		.boundaries = {
			{.firstPad = 0, .lastPad = kNumPads,
			.firstLed = 0, .lastLed = kNumLeds, },
			// {.firstPad = 0, .lastPad = kNumPads / 2 - guardPads,
			// .firstLed = 0, .lastLed = kNumLeds / 2, },
			// {.firstPad = kNumPads / 2 + guardPads, .lastPad = kNumPads,
			// .firstLed = kNumLeds / 2, .lastLed = kNumLeds, },
		},
		.maxNumCentroids = {1},
		.np = &np,
	});
	for(unsigned int n = 0; n < ledSliders.sliders.size(); ++n)
	{
		unsigned int m = n % 3;
		// set each subslider to R, G, B etc
		rgb_t color = {(uint8_t)(0), uint8_t(0), uint8_t(255)};
		ledSliders.sliders[n].setColor(color);
		ledSliders.sliders[n].setLedMode(LedSlider::MANUAL_CENTROIDS);
	}
	
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 255);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 255);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
}

// MODE 8: DUAL ENVELOPE GENERATOR
void mode8_setup()
{
	unsigned int guardPads = 1;
	// set up ledSliders with two sub-sliders
	ledSliders.setup({
		.order = {padsToOrderMap, padsToOrderMap + kNumPads},
		.sizeScale = 3200,
		.boundaries = {
			//{.firstPad = 0, .lastPad = kNumPads,
			//.firstLed = 0, .lastLed = kNumLeds, },
			{.firstPad = 0, .lastPad = kNumPads / 2 - guardPads,
			.firstLed = 0, .lastLed = kNumLeds / 2, },
			{.firstPad = kNumPads / 2 + guardPads, .lastPad = kNumPads,
			.firstLed = kNumLeds / 2, .lastLed = kNumLeds, },
		},
		.maxNumCentroids = {1, 1},
		.np = &np,
	});
	for(unsigned int n = 0; n < ledSliders.sliders.size(); ++n)
	{
		unsigned int m = n % 3;
		// set each subslider to R, G, B etc
		rgb_t color = {(uint8_t)((0 == m) * 255), uint8_t((0 == m) * 255), uint8_t((1 == m) * 255)};
		ledSliders.sliders[n].setColor(color);
		ledSliders.sliders[n].setLedMode(LedSlider::MANUAL_CENTROIDS);
	}
	
	for(unsigned int n = 0; n < kNumLeds/2-guardPads; ++n)
		np.setPixelColor(n, 255, 255, 0);
	for(unsigned int n = kNumLeds/2; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 255);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds/2-guardPads; ++n)
		np.setPixelColor(n, 255, 255, 0);
	for(unsigned int n = kNumLeds/2; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 255);
	np.show(); // actually display the updated LEDs
	usleep(200000);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, 0);
	np.show(); // actually display the updated LEDs
	usleep(200000);
}

void mode1_loop()
{
	ledSliders.process(trill.rawData.data());
	np.show(); // actually display the updated LEDs
	
	// outputs
	
}

void mode2_loop()
{
	ledSliders.process(trill.rawData.data());
	np.show(); // actually display the updated LEDs
}

void mode3_loop()
{
	ledSliders.process(trill.rawData.data());
	float fingerPos = ledSliders.sliders[0].compoundTouchLocation();
	float touchSize = ledSliders.sliders[0].compoundTouchSize();
	int touchPresent = ledSliders.sliders[0].getNumTouches();
	
	LedSlider::centroid_t centroids[1];
	
	if (touchPresent) {
		//  First time
		if (touchPresent != gPrevTouchPresent){
			rt_printf("NEW TOUCH\n");
			gCounter = 0;
			gEndOfGesture = 0;
			for(int n = 0; n < gMaxRecordLength; n++) {
				gTouchPositionRecording[n] = 0.0;
			}
		}
			
		centroids[0].location = fingerPos;
		centroids[0].size = touchSize;
		// Record gesture
		gTouchPositionRecording[gCounter] = fingerPos;
		
		gRestartCount = 1;
		gCounter++;
	}
	
	if (!touchPresent) {
		
		// Reset counter and store the sample length
		if (gRestartCount) {
			gEndOfGesture = gCounter;
			rt_printf("END OF RECORDING: %d\n",gEndOfGesture);
			gCounter = 0;
			gRestartCount = 0;
		}
		
		if (gCounter < gEndOfGesture) {
			centroids[0].location = gTouchPositionRecording[gCounter];
			centroids[0].size = 0.5;
			gCounter++;
		} else {
			gCounter = 0;
		}
		
	}
	
	gPrevTouchPresent = touchPresent;
	
	// Show centroid on the LEDs
	ledSliders.sliders[0].setLedsCentroids(centroids, 1);
	np.show(); // actually display the updated LEDs
}

// DUAL LFOS
void mode4_loop()
{
	ledSliders.process(trill.rawData.data());
	
	float fingerPosDualLFO[2] = {ledSliders.sliders[0].compoundTouchLocation(), ledSliders.sliders[1].compoundTouchLocation()};
	unsigned int touchPresentDualLFO[2] = {ledSliders.sliders[0].getNumTouches(), ledSliders.sliders[1].getNumTouches()};
	
	LedSlider::centroid_t centroids[2];
	
	for (int m=0; m<2; m++) {
		if (touchPresentDualLFO[m]) {
			//  First time
			if (touchPresentDualLFO[m] != gPrevTouchPresentDualLFO[m]){
				rt_printf("NEW TOUCH SENSOR %d\n", m);
				gCounterDualLFO[m] = 0;
				gEndOfGestureDualLFO[m] = 0;
				for(int n = 0; n < gMaxRecordLength; n++) {
					gTouchPositionRecordingDualLFO[m][n] = 0.0;
				}
			}
				
			centroids[m].location = fingerPosDualLFO[m];
			centroids[m].size = 0.5;
			// Record gesture
			gTouchPositionRecordingDualLFO[m][gCounterDualLFO[m]] = fingerPosDualLFO[m];
			
			gRestartCountDualLFO[m] = 1;
			gCounterDualLFO[m]++;
		}
		
		if (!touchPresentDualLFO[m]) {
			
			// Reset counter and store the sample length
			if (gRestartCountDualLFO[m]) {
				gEndOfGestureDualLFO[m] = gCounterDualLFO[m];
				rt_printf("END OF RECORDING %d\n",m);
				gCounterDualLFO[m] = 0;
				gRestartCountDualLFO[m] = 0;
			}
			
			if (gCounterDualLFO[m] < gEndOfGestureDualLFO[m]) {
				centroids[m].location = gTouchPositionRecordingDualLFO[m][gCounterDualLFO[m]];
				centroids[m].size = 0.5;
				gCounterDualLFO[m]++;
			} else {
				gCounterDualLFO[m] = 0;
			}
			
		}
		
		gPrevTouchPresentDualLFO[m] = touchPresentDualLFO[m];
		
		ledSliders.sliders[m].setLedsCentroids(centroids + m, 1);
	}
	
	np.show(); // actually display the updated LEDs
}

void mode5_loop()
{
	
	
	// t = clock time period / 1000
	// f = 1/t
	
	float t = gMtrClkTimePeriodScaled * 0.001;
	float freqMult = 1/t;
	
	ledSliders.process(trill.rawData.data());
	
	unsigned int numPads = 24;
	float* tmp = trill.rawData.data();
	float pads[numPads];
	sort(pads, tmp, padsToOrderMap, numPads);
	
	float bright[kNumLeds];
	// bright is a scratchpad for LED values
	resample(bright, kNumLeds, pads, numPads);

	float touchPosition = ledSliders.sliders[0].compoundTouchLocation();
	
	if (touchPosition > 0.0) {
		gDivisionPoint = touchPosition;
		
		rt_printf("%f and %f and freqmult: %f\n", (0.92 - gDivisionPoint) * freqMult*2, gDivisionPoint * freqMult*2, freqMult);
	}
	
	oscillator1.setFrequency((0.92 - gDivisionPoint) * freqMult*2);
	oscillator2.setFrequency(gDivisionPoint * freqMult*2);
	
	// float out1 = (oscillator1.process()+1)*0.5;
	// float out2 = (oscillator2.process()+1)*0.5;
	float out1 = oscillator1.process();
	float out2 = oscillator2.process();
	
	unsigned int split = gDivisionPoint > 0 ? kNumLeds * gDivisionPoint : 0;
	for(unsigned int n = 0; n < split; ++n)
		np.setPixelColor(n, 0, out1*255, 0);
	for(unsigned int n = split; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, out2*255);
	// for(unsigned int n = 0; n < kNumLeds; ++n)
	// 	np.setPixelColor(n, bright[n]*255, bright[n]*255, out*255 );
	np.show(); // actually display the updated LEDs
	
}

void mode6_loop()
{
	
	
	// t = clock time period / 1000
	// f = 1/t
	
	float t = gMtrClkTimePeriodScaled * 0.001;
	float freqMult = 1/t;
	
	ledSliders.process(trill.rawData.data());
	
	unsigned int numPads = 24;
	float* tmp = trill.rawData.data();
	float pads[numPads];
	sort(pads, tmp, padsToOrderMap, numPads);
	
	float bright[kNumLeds];
	// bright is a scratchpad for LED values
	resample(bright, kNumLeds, pads, numPads);

	float touchPosition = ledSliders.sliders[0].compoundTouchLocation();
	
	if (touchPosition > 0.0) {
		gDivisionPoint = touchPosition;
		
		rt_printf("%f and %f and freqmult: %f\n", (0.92 - gDivisionPoint) * freqMult*2, gDivisionPoint * freqMult*2, freqMult);
	}
	
	oscillator1.setFrequency((0.92 - gDivisionPoint) * freqMult*2);
	oscillator2.setFrequency(gDivisionPoint * freqMult*2);
	
	// float out1 = (oscillator1.process()+1)*0.5;
	// float out2 = (oscillator2.process()+1)*0.5;
	float out1 = oscillator1.process();
	float out2 = oscillator2.process();
	
	unsigned int split = gDivisionPoint > 0 ? kNumLeds * gDivisionPoint : 0;
	for(unsigned int n = 0; n < split; ++n)
		np.setPixelColor(n, 0, out1*255, 0);
	for(unsigned int n = split; n < kNumLeds; ++n)
		np.setPixelColor(n, 0, 0, out2*255);
	// for(unsigned int n = 0; n < kNumLeds; ++n)
	// 	np.setPixelColor(n, bright[n]*255, bright[n]*255, out*255 );
	np.show(); // actually display the updated LEDs
	
}

// ENVELOPE GENERATOR
void mode7_loop()
{
	ledSliders.process(trill.rawData.data());
	float fingerPos = ledSliders.sliders[0].compoundTouchLocation();
	float touchSize = ledSliders.sliders[0].compoundTouchSize();
	int touchPresent = ledSliders.sliders[0].getNumTouches();
	
	LedSlider::centroid_t centroids[1];
	
	if (touchPresent) {
		//  First time this loop runs
		if (touchPresent != gPrevTouchPresent){
			rt_printf("NEW TOUCH\n");
			gCounter = 0; // reset counter
			gEndOfGesture = 0; // reset end of gesture time
			for(int n = 0; n < gMaxRecordLength; n++) {
				gTouchPositionRecording[n] = 0.0; // clear the buffer of recorded positions
			}
		}
		
		// Every other time store the gesture
		centroids[0].location = fingerPos;
		centroids[0].size = touchSize;

		gTouchPositionRecording[gCounter] = fingerPos;
		
		gRestartCount = 1;
		gCounter++;
	}
	
	if (!touchPresent) {
		
		// Reset counter and store the sample length the first time this loop runs
		if (gRestartCount) {
			gEndOfGesture = gCounter;
			rt_printf("END OF RECORDING: %d\n",gEndOfGesture);
			gCounter = 0;
			gRestartCount = 0;
		}
		
		// ever other time playback the gesture
		if (gCounter < gEndOfGesture) {
			centroids[0].location = gTouchPositionRecording[gCounter];
			centroids[0].size = 0.5;
			gCounter++;
		} else {
			centroids[0].size = 0.0;
		}
		
		if (gMtrClkTrigger) {
			gCounter = 0; //restarts the playback
		}
		
	}
	
	gPrevTouchPresent = touchPresent;
	
	// Show centroid on the LEDs
	ledSliders.sliders[0].setLedsCentroids(centroids, 1);
	np.show(); // actually display the updated LEDs
}

// MODE 8: DUAL ENVELOPE GENERATOR
void mode8_loop()
{
	ledSliders.process(trill.rawData.data());
	
	float fingerPosDualLFO[2] = {ledSliders.sliders[0].compoundTouchLocation(), ledSliders.sliders[1].compoundTouchLocation()};
	unsigned int touchPresentDualLFO[2] = {ledSliders.sliders[0].getNumTouches(), ledSliders.sliders[1].getNumTouches()};
	
	LedSlider::centroid_t centroids[2];
	
	for (int m=0; m<2; m++) {
		if (touchPresentDualLFO[m]) {
			//  First time
			if (touchPresentDualLFO[m] != gPrevTouchPresentDualLFO[m]){
				rt_printf("NEW TOUCH SENSOR %d\n", m);
				gCounterDualLFO[m] = 0;
				gEndOfGestureDualLFO[m] = 0;
				for(int n = 0; n < gMaxRecordLength; n++) {
					gTouchPositionRecordingDualLFO[m][n] = 0.0;
				}
			}
				
			centroids[m].location = fingerPosDualLFO[m];
			centroids[m].size = 0.5;
			// Record gesture
			gTouchPositionRecordingDualLFO[m][gCounterDualLFO[m]] = fingerPosDualLFO[m];
			
			gRestartCountDualLFO[m] = 1;
			gCounterDualLFO[m]++;
		}
		
		if (!touchPresentDualLFO[m]) {
			
			// Reset counter and store the sample length
			if (gRestartCountDualLFO[m]) {
				gEndOfGestureDualLFO[m] = gCounterDualLFO[m];
				rt_printf("END OF RECORDING %d\n",m);
				gCounterDualLFO[m] = 0;
				gRestartCountDualLFO[m] = 0;
			}
			
			if (gCounterDualLFO[m] < gEndOfGestureDualLFO[m]) {
				centroids[m].location = gTouchPositionRecordingDualLFO[m][gCounterDualLFO[m]];
				centroids[m].size = 0.5;
				gCounterDualLFO[m]++;
			} else {
				centroids[m].size = 0.0;
			}
			
			if (gMtrClkTrigger) {
				gCounterDualLFO[m] = 0;
			}
			
		}
		
		gPrevTouchPresentDualLFO[m] = touchPresentDualLFO[m];
		
		ledSliders.sliders[m].setLedsCentroids(centroids + m, 1);
	}
	
	np.show(); // actually display the updated LEDs
}

bool tr_setup()
{
	np.begin();

	if(trill.setup(1, Trill::FLEX, 0x50))
		return false;
	
	trill.setPrescaler(5);

	cd.setup({padsToOrderMap, padsToOrderMap + kNumPads / 2}, 4, 3200);

	trill.setMode(Trill::DIFF);
	trill.printDetails();
	
	mode1_setup();

	
	return true;
}



void tr_loop()
{
	trill.readI2C();

	float fingerPos = ledSliders.sliders[0].compoundTouchLocation();
	float touchSize = ledSliders.sliders[0].compoundTouchSize();

	// Read analog in.
	float anIn = tri.analogRead();
	
	// Run the clock
	master_clock(anIn*3.0);
	
	// Run the div mult clock
	divmult_clock(gMtrClkTrigger, anIn*3.0);


	// Read 1st digital in (mode switching)
	int diIn1 = tri.digitalRead(0);
	
	int shouldChangeMode;
	if (diIn1 == 1 && diIn1 != gDiIn1Last){
		shouldChangeMode = 1;
	} else {
		shouldChangeMode = 0;
	}
	gDiIn1Last = diIn1;

	// Read 2nd digital in (sub mode)
	int diIn2 = tri.digitalRead(1);
	
	if (diIn2 == 1 && diIn2 != gDiIn2Last){
		gSubMode = !gSubMode;
		rt_printf("LATCH STATE: %d\n", gSubMode);
	} 
	gDiIn2Last = diIn2;
	
	
	// Switch between setup modes
	if(shouldChangeMode) {
		gMode = (gMode+1)%8;
		//  Setup first
		switch (gMode) {
			case 0:
	    		mode1_setup();
	    		break;
			case 1:
	    		mode2_setup();
	    		break;
    		case 2:
	    		mode3_setup();
	    		break;
    		case 3:
	    		mode4_setup();
	    		break;
    		case 4:
	    		mode5_setup();
	    		break;
    		case 5:
	    		mode6_setup();
	    		break;
    		case 6:
	    		mode7_setup();
	    		break;
    		case 7:
	    		mode8_setup();
	    		break;
		}
	}
	
	// Loop afterwards
	switch (gMode) {
		case 0:
    		mode1_loop();
    		break;
		case 1:
    		mode2_loop();
    		break;
		case 2:
    		mode3_loop();
    		break;
		case 3:
    		mode4_loop();
    		break;
		case 4:
    		mode5_loop();
    		break;
		case 5:
    		mode6_loop();
    		break;
		case 6:
    		mode7_loop();
    		break;
		case 7:
    		mode8_loop();
    		break;
	}
	
	tri.digitalWrite(gMtrClkTriggerLED);
	
	// write analog outputs
	tri.analogWrite(0, fingerPos);
	tri.analogWrite(1, touchSize);
	
	// Send to scope
	tri.scopeWrite(0, anIn);
	tri.scopeWrite(1, fingerPos);
	tri.scopeWrite(2, touchSize);
	usleep(kLoopSleepTimeUs);
}
