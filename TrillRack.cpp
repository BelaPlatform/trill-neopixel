#include "TrillRackInterface.h"
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include <libraries/Trill/CentroidDetection.h> // include this above NeoPixel or "HEX" gets screwed up
#include "LedSliders.h"
#include "NeoPixel.h"
#include <cmath>
#ifdef STM32
#define TRILL_CALLBACK // whether the I2C transfer is done via DMA + callback
#define TRILL_BAR // whether to use an external Trill Bar
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
#ifdef TRILL_BAR
	14,
	15,
	16,
	17,
	18,
	19,
	20,
	21,
	22,
	23,
#else // TRILL_BAR
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
#endif // TRILL_BAR
};

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

static void initSubSlider(size_t n, rgb_t color, LedSlider::LedMode_t mode)
{
	if(n < ledSliders.sliders.size())
	{
		ledSliders.sliders[n].setColor(color);
		ledSliders.sliders[n].setLedMode(mode);
	}
}

static void ledSlidersSetupOneSlider(rgb_t color, LedSlider::LedMode_t mode)
{
	ledSliders.setup({
		.order = {padsToOrderMap, padsToOrderMap + kNumPads},
		.sizeScale = 3200,
		.boundaries = {
			{.firstPad = 0, .lastPad = kNumPads,
			.firstLed = 0, .lastLed = kNumLeds, },
		},
		.maxNumCentroids = {1},
		.np = &np,
	});
	initSubSlider(0, color, mode);
}

static void ledSlidersSetupTwoSliders(unsigned int guardPads, rgb_t colors[2], LedSlider::LedMode_t mode)
{
	ledSliders.setup({
		.order = {padsToOrderMap, padsToOrderMap + kNumPads},
		.sizeScale = 3200,
		.boundaries = {
			{.firstPad = 0, .lastPad = kNumPads / 2 - guardPads,
			.firstLed = 0, .lastLed = kNumLeds / 2, },
			{.firstPad = kNumPads / 2 + guardPads, .lastPad = kNumPads,
			.firstLed = kNumLeds / 2, .lastLed = kNumLeds, },
		},
		.maxNumCentroids = {1, 1},
		.np = &np,
	});
	for(unsigned int n = 0; n < 2; ++n)
		initSubSlider(n, colors[n], mode);
}

bool modeChangeBlinkSplit(double ms, rgb_t colors[2], size_t endFirst, size_t startSecond)
{
	static double oldMs = 9999999999999999;
	bool done = false;
	double period = 200;
	if(ms < oldMs) // start when time jumps back
	{
		for(unsigned int n = 0; n < endFirst; ++n)
			np.setPixelColor(n, colors[0].r, colors[0].g, colors[0].b);
		for(unsigned int n = startSecond; n < kNumLeds; ++n)
			np.setPixelColor(n, colors[1].r, colors[1].g, colors[1].b);
		np.show();
	} else if (oldMs < 1 * period && ms >= 1 * period) {
		for(unsigned int n = 0; n < kNumLeds; ++n)
			np.setPixelColor(n, 0, 0, 0);
		np.show();
	} else if (oldMs < 2 * period && ms >= 2 * period) {
		for(unsigned int n = 0; n < endFirst; ++n)
			np.setPixelColor(n, colors[0].r, colors[0].g, colors[0].b);
		for(unsigned int n = startSecond; n < kNumLeds; ++n)
			np.setPixelColor(n, colors[1].r, colors[1].g, colors[1].b);
		np.show();
	} else if (oldMs < 3 * period && ms >= 3 * period) {
		for(unsigned int n = 0; n < kNumLeds; ++n)
			np.setPixelColor(n, 0, 0, 0);
		np.show();
	} else {
		done = true;
	}
	oldMs = ms;
	return done;
}

static bool modeChangeBlink(double ms, rgb_t color)
{
	rgb_t colors[2] = {color, color};
	return modeChangeBlinkSplit(ms, colors, kNumLeds, kNumLeds);
}

// MODE 1: DIRECT CONTROL / SINGLE SLIDER
bool mode1_setup(double ms)
{
	rgb_t color = {uint8_t(255), 0, 0};
	ledSlidersSetupOneSlider(
		color,
		LedSlider::AUTO_CENTROIDS
	);
	return modeChangeBlink(ms, color);
}

// MODE 2: DIRECT CONTROL / DOUBLE SLIDER
bool mode2_setup(double ms)
{
	unsigned int guardPads = 1;
	rgb_t colors[2] = {
		{uint8_t(255), 0, 0},
		{0, uint8_t(255), 0},
	};
	ledSlidersSetupTwoSliders(guardPads, colors, LedSlider::AUTO_CENTROIDS);
	return modeChangeBlinkSplit(ms, colors, kNumLeds / 2 - guardPads, kNumLeds / 2);
}

// MODE 3: SINGLE SLIDER / LOOP GESTURE
bool mode3_setup(double ms)
{
	rgb_t color = {uint8_t(255), uint8_t(255), uint8_t(255)};
	ledSlidersSetupOneSlider(color, LedSlider::MANUAL_CENTROIDS);
	return modeChangeBlink(ms, color);
}

// MODE 4: LFO / DOUBLE SLIDER
bool mode4_setup(double ms)
{
	unsigned int guardPads = 1;
	rgb_t colors[2] = {
		{uint8_t(255), uint8_t(255), 0},
		{0, 0, uint8_t(255)},
	};
	ledSlidersSetupTwoSliders(guardPads, colors, LedSlider::MANUAL_CENTROIDS);
	return modeChangeBlinkSplit(ms, colors, kNumLeds / 2 - guardPads, kNumLeds / 2);
}

bool mode5_setup(double ms)
{
	oscillator1.setup(1000, Oscillator::triangle);
	oscillator2.setup(1000, Oscillator::triangle);
	
	rgb_t color = {uint8_t(255), uint8_t(255), uint8_t(255)};
	ledSlidersSetupOneSlider(
		color,
		LedSlider::MANUAL_CENTROIDS
	);
	rgb_t otherColor = {0, uint8_t(255), 0}; // TODO: maybe it was a typo?
	return modeChangeBlink(ms, otherColor);
}

bool mode6_setup(double ms)
{
	oscillator1.setup(1000, Oscillator::square);
	oscillator2.setup(1000, Oscillator::square);
	
	rgb_t color = {uint8_t(255), uint8_t(255), uint8_t(255)};
	ledSlidersSetupOneSlider(
		color,
		LedSlider::MANUAL_CENTROIDS
	);
	rgb_t otherColor = {0, uint8_t(255), 0}; // TODO: maybe it was a typo?
	return modeChangeBlink(ms, otherColor);
}

// MODE 7: ENVELOPE GENERATOR
bool mode7_setup(double ms)
{
	rgb_t color = {0, 0, uint8_t(255)};
	ledSlidersSetupOneSlider(
		color,
		LedSlider::MANUAL_CENTROIDS
	);
	return modeChangeBlink(ms, color);
}

// MODE 8: DUAL ENVELOPE GENERATOR
bool mode8_setup(double ms)
{
	unsigned int guardPads = 1;
	rgb_t colors[2] = {
		{uint8_t(255), uint8_t(255), 0},
		{0, 0, uint8_t(255)},
	};
	ledSlidersSetupTwoSliders(guardPads,
		colors,
		LedSlider::MANUAL_CENTROIDS
	);
	return modeChangeBlinkSplit(ms, colors, kNumLeds / 2 - guardPads, kNumLeds / 2);
}

void mode1_loop()
{
}

void mode2_loop()
{
}

void mode3_loop()
{
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
		
		centroids[0].location = gTouchPositionRecording[gCounter];
		centroids[0].size = 0.5;
		++gCounter;
		if (gCounter >= gEndOfGesture)
			gCounter = 0;
	}
	
	gPrevTouchPresent = touchPresent;
	
	// Show centroid on the LEDs
	ledSliders.sliders[0].setLedsCentroids(centroids, 1);
}

// DUAL LFOS
void mode4_loop()
{
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
}

void mode5_loop()
{
	
	
	// t = clock time period / 1000
	// f = 1/t
	
	float t = gMtrClkTimePeriodScaled * 0.001;
	float freqMult = 1/t;
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
}

void mode6_loop()
{
	
	
	// t = clock time period / 1000
	// f = 1/t
	
	float t = gMtrClkTimePeriodScaled * 0.001;
	float freqMult = 1/t;
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
}

// ENVELOPE GENERATOR
void mode7_loop()
{
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
}

// MODE 8: DUAL ENVELOPE GENERATOR
void mode8_loop()
{
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
}

enum { kNumModes = 8 };
static bool (*mode_setups[kNumModes])(double) = {
	mode1_setup,
	mode2_setup,
	mode3_setup,
	mode4_setup,
	mode5_setup,
	mode6_setup,
	mode7_setup,
	mode8_setup,
};
static void (*mode_loops[kNumModes])(void) = {
	mode1_loop,
	mode2_loop,
	mode3_loop,
	mode4_loop,
	mode5_loop,
	mode6_loop,
	mode7_loop,
	mode8_loop,
};

#ifdef STM32_NEOPIXEL
extern TIM_HandleTypeDef htim2;
static Stm32NeoPixelT<uint32_t, 28> snp(&htim2, TIM_CHANNEL_2, 66, 33);
#endif // STM32_NEOPIXEL

#ifdef STM32_NEOPIXEL
extern "C" {
void tr_snpDone(void);
};

void tr_snpDone()
{
	snp.done();
}
#endif // STM32_NEOPIXEL

bool tr_setup()
{
	np.begin();
#ifdef STM32_NEOPIXEL
	np.setSnp(&snp);
#endif // STM32_NEOPIXEL

#ifdef TRILL_BAR
	if(trill.setup(1, Trill::BAR))
#else // TRILL_BAR
	if(trill.setup(1, Trill::FLEX, 0x50))
#endif // TRILL_BAR
		return false;
	trill.printDetails();
	if(trill.setMode(Trill::DIFF))
		return false;
#ifdef TRILL_BAR
	if(trill.setPrescaler(2))
		return false;
#else // TRILL_BAR
	if(trill.setPrescaler(5))
		return false;
#endif // TRILL_BAR
	if(trill.setNoiseThreshold(0.06))
		return false;
	if(trill.updateBaseline())
		return false;
#ifdef TRILL_CALLBACK
	if(trill.prepareForDataRead())
		return false;
#endif // TRILL_CALLBACK

	cd.setup({padsToOrderMap, padsToOrderMap + kNumPads / 2}, 4, 3200);
	return true;
}

#ifdef TRILL_CALLBACK
void tr_newData(const uint8_t* newData, size_t len)
{
	trill.newData(newData, len);
}
#endif // TRILL_CALLBACK


void tr_loop()
{
#ifndef TRILL_CALLBACK
	trill.readI2C();
#endif // TRILL_CALLBACK

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
	static bool firstRun = true;
	if ((diIn1 == 1 && diIn1 != gDiIn1Last) || (firstRun)){
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
	
	static double setupMs = 0;
	// Switch between setup modes
	if(shouldChangeMode) {
		setupMs = tri.getTimeMs();
		if(!firstRun)
			gMode = (gMode + 1) % kNumModes;
	}
	firstRun = false;

	bool shouldProcess = mode_setups[gMode](tri.getTimeMs() - setupMs);
	if(shouldProcess)
	{
		ledSliders.process(trill.rawData.data());
		// if setup is done, run
		mode_loops[gMode]();
		np.show(); // actually display the updated LEDs
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
