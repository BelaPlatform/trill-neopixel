#include "TrillRackInterface.h"
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include <libraries/Trill/CentroidDetection.h> // include this above NeoPixel or "HEX" gets screwed up
#include "LedSliders.h"
#include "NeoPixel.h"
#include <cmath>
#ifdef STM32
#define TRILL_CALLBACK // whether the I2C transfer is done via DMA + callback
#define TR_LOOP_TIME_CRITICAL // whether to disallow usleep() inside tr_loop
#define TRILL_BAR // whether to use an external Trill Bar
#define rt_printf printf
#define REDUCE_RAM_USAGE
#endif // STM32

// Robbie's Variables
// ------------------
#include <libraries/Oscillator/Oscillator.h>

Oscillator oscillator1;
Oscillator oscillator2;

typedef enum {
	kOutModeFollowTouch,
	kOutModeFollowLeds,
} OutMode;

// Mode switching
int gMode = 3;
static OutMode gOutMode = kOutModeFollowTouch;
int gDiIn1Last = 0;
int gCounter = 0;
int gDiIn2Last = 0;
int gSubMode = 0;

// Recording the gesture
int gEndOfGesture = 0; // store gesture length
int gRestartCount = 0;
enum { kMaxRecordLength = 1000 };
unsigned int gPrevTouchPresent = 0; // store whether a touch was previously present

// Recording two gestures at once
/*
unsigned int gPrevTouchPresentDualLFO[2] = {0};
int gCounterDualLFO[2] = {0};
int gEndOfGestureDualLFO[2] = {0};
float gTouchPositionRecordingDualLFO[2][kMaxRecordLength] = {0, 0};
int gRestartCountDualLFO[2] = {0};
*/

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
		.maxNumCentroids = {2},
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
		.maxNumCentroids = {2, 2},
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
	} else if (ms >= 4 * period) {
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
	gOutMode = kOutModeFollowTouch;
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
	gOutMode = kOutModeFollowTouch;
	return modeChangeBlinkSplit(ms, colors, kNumLeds / 2 - guardPads, kNumLeds / 2);
}

// MODE 3: SINGLE SLIDER / LOOP GESTURE
bool mode3_setup(double ms)
{
	rgb_t color = {uint8_t(255), uint8_t(255), uint8_t(255)};
	ledSlidersSetupOneSlider(color, LedSlider::MANUAL_CENTROIDS);
	gOutMode = kOutModeFollowLeds;
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
	gOutMode = kOutModeFollowLeds;
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
	gOutMode = kOutModeFollowLeds;
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
	gOutMode = kOutModeFollowLeds;
	return modeChangeBlinkSplit(ms, colors, kNumLeds / 2 - guardPads, kNumLeds / 2);
}

void mode1_loop()
{
}

void mode2_loop()
{
}

template <typename sample_t>
class Recorder
{
public:
	void disable()
	{
		active = false;
	}
	void startRecording()
	{
		active = true;
		start = current;
		end = 0;
	}
	sample_t& record(const sample_t& in)
	{
		data[current] = in;
		sample_t& ret = data[current];
		++current;
		if(data.size() == current)
			current = 0;
		// if the circular buffer is full,
		// move its starting point
		if(current == start)
			start++;
		return ret;
	}
	void stopRecording()
	{
		end = current;
		current = start;
	}
	sample_t& play(bool loop)
	{
		static sample_t zero = 0;
		if(!active)
			return zero;
		auto& ret = data[current];
		current++;
		if(current >= end)
		{
			if(loop)
				current = start;
			else
				active = false;
		}
		return ret;
	}
protected:
	std::array<sample_t, kMaxRecordLength> data;
	size_t start = 0;
	size_t end = 0;
	size_t current = 0;
	bool active = false;
};

template <unsigned int max>
class TimestampedRecorder : public Recorder<uint32_t>
{
	enum { kRepsBits = 10, kSampleBits = 22 };
	struct timedData_t
	{
		uint32_t reps : kRepsBits;
		uint32_t sample : kSampleBits;
	};
	static_assert(sizeof(timedData_t) <= 4); // if you change field values to be larger than 4 bytes, be well aware of that
public:
	static uint32_t inToSample(const float& in)
	{
		uint32_t r = in / max * kSampleMax + 0.5f;
		return r;
	}
	static float sampleToOut(uint32_t sample)
	{
		return sample * max / float(kSampleMax);
	}
	static struct timedData_t recordToTimedData(uint32_t d)
	{
		return *(struct timedData_t*)&d;
	}
	static uint32_t timedDataToRecord(const struct timedData_t t)
	{
		return *(uint32_t*)&t;
	}
	float record(const float& in)
	{
		uint32_t sample = inToSample(in);
		if(sample > kSampleMax)
			sample = kSampleMax;
		if(oldSample == sample && kRepsMax != reps)
			++reps;
		else {
			uint32_t r = timedDataToRecord({ .reps = reps, .sample = oldSample });
			uint32_t d = Recorder<uint32_t>::record(r);
			recordToTimedData(d);
			reps = 0;
			oldSample = sample;
		}
		return sampleToOut(sample);
	}
	float play(bool loop)
	{
		if(playData.reps)
			--playData.reps;
		else {
			playData = recordToTimedData(Recorder<uint32_t>::play(loop));
		}
		return sampleToOut(playData.sample);
	}
private:
	enum { kRepsMax = (1 << kRepsBits) - 1 };
	enum { kSampleMax = (1 << kSampleBits) - 1 };
	timedData_t playData = {0};
	uint32_t oldSample;
	uint16_t reps;
};

class GestureRecorder
{
public:
	typedef float sample_t; // this must match TimestampedRecorder's type
	typedef struct {
		sample_t first;
		sample_t second;
	} Gesture_t;
	Gesture_t process(const std::vector<LedSlider>& sliders, bool loop)
	{
		if(sliders.size() < 1)
			return Gesture_t();
		bool single = (1 == sliders.size());
		unsigned int active[2];
		active[0] = sliders[0].getNumTouches();
		if(single)
			active[1] = active[0];
		else
			active[1] = sliders[1].getNumTouches();
		sample_t out[2];
		for(unsigned int n = 0; n < 2; ++n)
		{
			if(active[n] != pastActive[n]) //state change
			{
				if(2 == active[n])  // two touches: disable
					rs[n].disable();
				else if(1 == active[n] && 0 == pastActive[n]) // going from 0 to 1 touch: start recording (and enable)
					rs[n].startRecording();
				else if(0 == active[n]) // going to 0 touches: start playing back (unless disabled)
					rs[n].stopRecording();
			}
			pastActive[n] = active[n];
			if(active[n])
			{
				sample_t val;
				if(0 == n)
					val = sliders[n].compoundTouchLocation();
				else {
					if(single)
						val = sliders[0].compoundTouchSize();
					else
						val = sliders[n].compoundTouchLocation();
				}
				out[n] = rs[n].record(val);
			}
			else
				out[n] = rs[n].play(loop);
		}
		return {out[0], out[1]};
	}
private:
	std::array<TimestampedRecorder<1>, 2> rs;
	unsigned int pastActive[2];
} gGestureRecorder;

float gTouchPositionRecording[100]; // dummy, to be removed next

static void gestureRecorderSingle_loop(bool loop)
{
	GestureRecorder::Gesture_t g = gGestureRecorder.process(ledSliders.sliders, loop);
	LedSlider::centroid_t centroids[1];
	centroids[0].location = g.first;
	centroids[0].size = g.first ? g.second : 0;
	ledSliders.sliders[0].setLedsCentroids(centroids, 1);
}

static void gestureRecorderSplit_loop(bool loop)
{
	GestureRecorder::Gesture_t g = gGestureRecorder.process(ledSliders.sliders, loop);
	LedSlider::centroid_t centroids[2];
	centroids[0].location = g.first;
	centroids[0].size = g.first ? 0.5 : 0;
	centroids[1].location = g.second;
	centroids[1].size = g.first ? 0.5 : 0;
	ledSliders.sliders[0].setLedsCentroids(centroids, 1);
	ledSliders.sliders[1].setLedsCentroids(centroids + 1, 1);
}

// SINGLE LFO
void mode3_loop()
{
	gestureRecorderSingle_loop(true);
}

// DUAL LFOS
void mode4_loop()
{
	gestureRecorderSplit_loop(true);
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
	gestureRecorderSingle_loop(false);
}

// MODE 8: DUAL ENVELOPE GENERATOR
void mode8_loop()
{
	gestureRecorderSplit_loop(false);
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
#ifdef REDUCE_RAM_USAGE
static void mode_loop_dummy() {}
#endif // REDUCE_RAM_USAGE
static void (*mode_loops[kNumModes])(void) = {
	mode1_loop,
	mode2_loop,
	mode3_loop,
	mode4_loop,
	mode5_loop,
	mode6_loop,
	mode7_loop,
#ifdef REDUCE_RAM_USAGE
	mode_loop_dummy,
#else // REDUCE_RAM_USAGE
	mode8_loop,
#endif // REDUCE_RAM_USAGE
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

#ifdef TR_LOOP_TIME_CRITICAL
void tr_process(void* ptr)
{
	tri.process(ptr);
}
#endif // TR_LOOP_TIME_CRITICAL

void tr_loop()
{
#ifndef TRILL_CALLBACK
	trill.readI2C();
#endif // TRILL_CALLBACK

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
	auto& sls = ledSliders.sliders;
	switch (gOutMode)
	{
		case kOutModeFollowTouch:
			tri.analogWrite(0, sls[0].compoundTouchLocation());
			if(1 == sls.size())
				tri.analogWrite(1, sls[0].compoundTouchSize());
			else if (2 == sls.size())
				tri.analogWrite(1, sls[1].compoundTouchLocation());
			break;
		case kOutModeFollowLeds:
			tri.analogWrite(0, sls[0][0].location);
			if(1 == sls.size())
				tri.analogWrite(1, sls[0][0].size);
			else if (2 == sls.size())
				tri.analogWrite(1, sls[1][0].location);
			break;
	}
	
	// Send to scope
	tri.scopeWrite(0, anIn);
	tri.scopeWrite(1, ledSliders.sliders[0].compoundTouchLocation());
	tri.scopeWrite(2, ledSliders.sliders[0].compoundTouchSize());
#ifndef TR_LOOP_TIME_CRITICAL
	usleep(kLoopSleepTimeUs);
#endif // TR_LOOP_TIME_CRITICAL
}
