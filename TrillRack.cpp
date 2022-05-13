#include <TrillRackApplication_bsp.h>
#include "TrillRackInterface.h"
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include <libraries/Trill/CentroidDetection.h> // include this above NeoPixel or "HEX" gets screwed up
#include "LedSliders.h"
#include "NeoPixel.h"
#include <cmath>
#include <assert.h>
extern "C" {
#include "usbd_midi_if.h"
};

#ifdef STM32
#define TRILL_CALLBACK // whether the I2C transfer is done via DMA + callback
#define TR_LOOP_TIME_CRITICAL // whether to disallow usleep() inside tr_loop
//#define TRILL_BAR // whether to use an external Trill Bar
#define rt_printf printf
#endif // STM32

// Robbie's Variables
// ------------------
#include <libraries/Oscillator/Oscillator.h>

Oscillator oscillator1;
Oscillator oscillator2;

typedef enum {
	kOutModeFollowTouch,
	kOutModeFollowLeds,
	kOutModeManual,
} OutMode;

// Mode switching
int gMode = 0;
static OutMode gOutMode = kOutModeFollowTouch;
int gDiIn0Last = 0;
int gCounter = 0;
int gSubMode = 0;

// Recording the gesture
enum { kMaxRecordLength = 1000 };
static std::vector<float> gManualAnOut(2);

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


TrillRackInterface tri(0, 0, 1, 5, 4);
const unsigned int kNumLeds = 23;
NeoPixel np(kNumLeds, 0, NEO_RGB);
Trill trill;
CentroidDetection cd;

const unsigned int kLoopSleepTimeUs = 10000;

const unsigned int kNumPads = 26;
unsigned int padsToOrderMap[kNumPads] = {
#ifdef OLD
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
#else // OLD
	29,
	28,
	27,
	26,
	25,
	24,
	23,
	22,
	21,
	17,
	18,
	19,
	14,
	13,
	11,
	3,
	4,
	5,
	6,
	8,
	7,
	9,
	10,
	12,
	15,
	16,
#endif // OLD
};

static LedSliders ledSliders;
static LedSliders ledSlidersAlt;

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

static void ledSlidersSetupMultiSlider(LedSliders& ls, std::vector<rgb_t> const& colors, const LedSlider::LedMode_t& mode, bool setInitial)
{
	std::vector<LedSliders::delimiters_t> boundaries;
	size_t numSplits = colors.size();
	if(!numSplits)
		return;

	float guardPads = 2;
	float guardLeds = 2;
	if(0 == numSplits)
	{
		guardPads = 0;
		guardLeds = 0;
	}
	float activePads = (kNumPads - (guardPads * (numSplits - 1))) / float(numSplits);
	float activeLeds = (kNumLeds - (guardLeds * (numSplits - 1))) / float(numSplits);
	for(size_t n = 0; n < numSplits; ++n)
	{
		size_t firstPad = n * (activePads + guardPads);
		size_t firstLed = n * (activeLeds + guardLeds);
		size_t lastPad = firstPad + activePads;
		size_t lastLed = firstLed + activeLeds;
		boundaries.push_back({
				.firstPad = firstPad,
				.lastPad = lastPad,
				.firstLed = firstLed,
				.lastLed = lastLed,
		});
	}
	LedSliders::Settings settings = {
			.order = {padsToOrderMap, padsToOrderMap + kNumPads},
			.sizeScale = 3200,
			.boundaries = boundaries,
			.maxNumCentroids = {2},
			.np = &np,
	};
	ls.setup(settings);
	assert(numSplits == ls.sliders.size());

	for(size_t n = 0; n < numSplits; ++n)
	{
		ls.sliders[n].setColor(colors[n]);
		ls.sliders[n].setLedMode(mode);
		if(setInitial)
		{
			LedSlider::centroid_t centroid;
			centroid.location = 0.5;
			centroid.size = 0.1;
			ls.sliders[n].setLedsCentroids(&centroid, 1);
		}
	}
}

static void ledSlidersExpButtonsProcess(LedSliders& sl, std::vector<float>& outs, float scale, std::vector<float>const& offsets = {})
{
	int highest = -1;
	for(size_t n = 0; n < sl.sliders.size(); ++n)
	{
		if(sl.sliders[n].getNumTouches())
			highest = n;
	}
	bool allFollow = false;
	for(auto& o : outs)
		o = 0;
	for(size_t n = 0; n < sl.sliders.size(); ++n)
	{
		LedSlider::centroid_t centroid;
		if(highest == int(n) || allFollow)
		{
			centroid.location = sl.sliders[n].compoundTouchLocation();
			centroid.size = sl.sliders[n].compoundTouchSize();
			if(outs.size() > 0)
				outs[0] = centroid.location * scale + (offsets.size() > n ? offsets[n] : 0);
			if(outs.size() > 1)
				outs[1] = centroid.size;
		} else {
			// dimmed for "inactive"
			centroid.size = 0.1;
			centroid.location = 0.5;
		}
		sl.sliders[n].setLedsCentroids(&centroid, 1);
	}
}

static void ledSlidersFixedButtonsProcess(LedSliders& sl, std::vector<bool>& states, std::vector<size_t>& onsets, std::vector<size_t>& offsets, bool onlyUpdateStates)
{
	onsets.resize(0);
	offsets.resize(0);
	states.resize(sl.sliders.size());
	for(size_t n = 0; n < sl.sliders.size(); ++n)
	{
		bool state = sl.sliders[n].getNumTouches();
		bool pastState = states[n];
		if(!onlyUpdateStates)
		{
			bool shouldUpdateCentroids = false;
			if(state && !pastState) {
				onsets.emplace_back(n);
				shouldUpdateCentroids = true;
			} else if (!state && pastState) {
				offsets.emplace_back(n);
				shouldUpdateCentroids = true;
			}
			if(shouldUpdateCentroids)
			{
				LedSlider::centroid_t centroid;
				centroid.location = 0.5;
				// dimmed for "inactive"
				// full brightness for "active"
				centroid.size = state ? 1 : 0.1;
				sl.sliders[n].setLedsCentroids(&centroid, 1);
			}
		}
		states[n] = state;
	}
}

static void ledSlidersSetupOneSlider(rgb_t color, LedSlider::LedMode_t mode)
{
	ledSlidersSetupMultiSlider(ledSliders, {color}, mode, false);
}

static void ledSlidersSetupTwoSliders(unsigned int guardPads, rgb_t colors[2], LedSlider::LedMode_t mode)
{
	ledSlidersSetupMultiSlider(ledSliders, {colors[0], colors[1]}, mode, false);
}

bool modeChangeBlinkSplit(double ms, rgb_t colors[2], size_t endFirst, size_t startSecond)
{
	bool done = false;
	double period = 200;
	// blink on-off-on-off
	if(
			ms < 1 *period
			|| (ms >= 2 * period && ms < 3 * period)
	){
		for(unsigned int n = 0; n < endFirst; ++n)
			np.setPixelColor(n, colors[1].r, colors[1].g, colors[1].b);
		for(unsigned int n = startSecond; n < kNumLeds; ++n)
			np.setPixelColor(n, colors[0].r, colors[0].g, colors[0].b);
	} else if (
			(ms >= 1 * period && ms < 2 * period)
			|| (ms >= 3 * period && ms < 4 * period)
	){
		for(unsigned int n = 0; n < kNumLeds; ++n)
			np.setPixelColor(n, 0, 0, 0);
	} else if (ms >= 4 * period) {
		done = true;
	}
	return done;
}

static bool modeChangeBlink(double ms, rgb_t color)
{
	rgb_t colors[2] = {color, color};
	return modeChangeBlinkSplit(ms, colors, kNumLeds, kNumLeds);
}

// MODE Alt: settings UI
static bool modeAlt_setup()
{
	ledSlidersSetupMultiSlider(
		ledSlidersAlt,
		{
			{uint8_t(255), 0, 0},
			{0, uint8_t(0), uint8_t(255)},
			{0, uint8_t(0), uint8_t(255)},
			{0, uint8_t(0), uint8_t(255)},
			{0, uint8_t(255), 0},
		},
		LedSlider::MANUAL_CENTROIDS,
		true
	);
	return true;
}

// MODE 1: DIRECT CONTROL / SINGLE SLIDER
bool mode1_setup(double ms)
{
	rgb_t color = {uint8_t(255), 0, 0};
	if(!ms)
	{
		ledSlidersSetupOneSlider(
			color,
			LedSlider::AUTO_CENTROIDS
		);
		gOutMode = kOutModeFollowTouch;
	}
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
	if(!ms)
	{
		ledSlidersSetupTwoSliders(guardPads, colors, LedSlider::AUTO_CENTROIDS);
		gOutMode = kOutModeFollowTouch;
	}
	return modeChangeBlinkSplit(ms, colors, kNumLeds / 2 - guardPads, kNumLeds / 2);
}

// MODE 3: LFO / SINGLE SLIDER
bool mode3_setup(double ms)
{
	rgb_t color = {uint8_t(255), uint8_t(255), uint8_t(255)};
	if(!ms)
	{
		ledSlidersSetupOneSlider(color, LedSlider::MANUAL_CENTROIDS);
		gOutMode = kOutModeFollowLeds;
	}
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
	if(!ms)
	{
		ledSlidersSetupTwoSliders(guardPads, colors, LedSlider::MANUAL_CENTROIDS);
		gOutMode = kOutModeFollowLeds;
	}
	return modeChangeBlinkSplit(ms, colors, kNumLeds / 2 - guardPads, kNumLeds / 2);
}

bool mode5_setup(double ms)
{
	if(!ms)
	{
		oscillator1.setup(1000, Oscillator::triangle);
		oscillator2.setup(1000, Oscillator::triangle);

		rgb_t color = {uint8_t(255), uint8_t(255), uint8_t(255)};
		ledSlidersSetupOneSlider(
			color,
			LedSlider::MANUAL_CENTROIDS
		);
	}
	rgb_t otherColor = {0, uint8_t(255), 0}; // TODO: maybe this is meant to be the same as above?
	return modeChangeBlink(ms, otherColor);
}

bool mode6_setup(double ms)
{
	if(!ms)
	{
		oscillator1.setup(1000, Oscillator::square);
		oscillator2.setup(1000, Oscillator::square);

		rgb_t color = {uint8_t(255), uint8_t(255), uint8_t(255)};
		ledSlidersSetupOneSlider(
			color,
			LedSlider::MANUAL_CENTROIDS
		);
	}
	rgb_t otherColor = {0, uint8_t(255), 0}; // TODO: maybe this is meant to be the same as above?
	return modeChangeBlink(ms, otherColor);
}

// MODE 7: ENVELOPE GENERATOR
bool mode7_setup(double ms)
{
	rgb_t color = {0, 0, uint8_t(255)};
	if(!ms)
	{
		ledSlidersSetupOneSlider(
			color,
			LedSlider::MANUAL_CENTROIDS
		);
		gOutMode = kOutModeFollowLeds;
	}
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
	if(!ms)
	{
		ledSlidersSetupTwoSliders(guardPads,
			colors,
			LedSlider::MANUAL_CENTROIDS
		);
		gOutMode = kOutModeFollowLeds;
	}
	return modeChangeBlinkSplit(ms, colors, kNumLeds / 2 - guardPads, kNumLeds / 2);
}

void mode1_loop()
{
}

void mode2_loop()
{
}

// MODE 9: monochrome VUmeter / envelope follower (pretty crude, without rectification or lowpass for now.
// good for LFOs/EGs
bool mode9_setup(double ms)
{
	rgb_t color = {uint8_t(127), uint8_t(127), 0};
	if(!ms)
	{
		ledSlidersSetupOneSlider(
			color,
			LedSlider::MANUAL_CENTROIDS
		);
		gOutMode = kOutModeFollowLeds;
	}
	return modeChangeBlink(ms, color);
}

bool mode10_setup(double ms)
{
	if(!ms)
	{
		ledSlidersSetupMultiSlider(
			ledSliders,
			{
				{0, uint8_t(255), 0},
				{0, uint8_t(200), uint8_t(50)},
				{0, uint8_t(150), uint8_t(100)},
				{0, uint8_t(100), uint8_t(150)},
				{0, uint8_t(50), uint8_t(200)},
			},
			LedSlider::MANUAL_CENTROIDS,
			true
		);
		gOutMode = kOutModeManual;
	}
	return modeChangeBlink(ms, {0, 0, uint8_t(255)});
}

template <typename sample_t>
class Recorder
{
public:
	void enable(bool restart)
	{
		active = true;
		if(restart)
			current = start;
	}
	void disable()
	{
		active = false;
	}
	bool isEnabled()
	{
		return active;
	}
	virtual void startRecording()
	{
		active = true;
		start = current;
		full = false;
	}
	sample_t& record(const sample_t& in)
	{
		data[current] = in;
		sample_t& ret = data[current];
		increment(current);
		// if the circular buffer becomes full, make a note of it
		if(current == start)
			full = true;
		return ret;
	}
	virtual void stopRecording()
	{
		end = current;
		if(full)
		{
			// if the circular buffer became full, adjust start
			// so that we use all data in the buffer
			start = end;
			increment(start);
		}
		current = start;
	}

	sample_t& play(bool loop)
	{
		static sample_t zero = 0;
		if(current == end)
		{
			if(loop)
				current = start;
			else
				active = false;
		}
		if(!active)
			return zero;
		auto& ret = data[current];
		increment(current);
		return ret;
	}

	size_t size()
	{
		return (start - end + data.size()) % data.size();
	}
protected:
	template<typename T> void increment(T& idx)
	{
		idx = idx + 1;
		if(data.size() == idx)
			idx = 0;
	};

	std::array<sample_t, kMaxRecordLength> data;
	size_t start = 0;
	size_t end = 0;
	size_t current = 0;
	bool active = false;
	bool full = false; // whether during recording the buffer becomes full
};

template <typename sample_t, unsigned int max>
class TimestampedRecorder : public Recorder<uint32_t>
{
private:
	typedef Recorder<uint32_t> Base;
	enum { kRepsBits = 10, kSampleBits = 22 };
	struct timedData_t
	{
		uint32_t reps : kRepsBits;
		uint32_t sample : kSampleBits;
	};
	static_assert(sizeof(timedData_t) <= 4); // if you change field values to be larger than 4 bytes, be well aware of that
public:
	static uint32_t inToSample(const sample_t& in)
	{
		uint32_t r = in / max * kSampleMax + 0.5f;
		return r;
	}
	static sample_t sampleToOut(uint32_t sample)
	{
		return sample * max / sample_t(kSampleMax);
	}
	static struct timedData_t recordToTimedData(uint32_t d)
	{
		return *(struct timedData_t*)&d;
	}
	static uint32_t timedDataToRecord(const struct timedData_t t)
	{
		return *(uint32_t*)&t;
	}
	void startRecording() override
	{
		firstSample = true;
		Base::startRecording();
	}

	sample_t record(const sample_t& in)
	{
		uint32_t sample = inToSample(in);
		if(sample > kSampleMax)
			sample = kSampleMax;

		if(!firstSample && oldSample == sample && kRepsMax != reps)
			++reps;
		else {
			if(!firstSample)
			{
				pushSample();
			}
			reps = 0;
			oldSample = sample;
		}
		firstSample = false;
		return sampleToOut(sample);
	}

	void stopRecording() override
	{
		// flush whatever we haven't recorded yet
		pushSample();
		Base::stopRecording();
	}

	sample_t play(bool loop)
	{
		if(playData.reps)
			--playData.reps;
		else {
			playData = recordToTimedData(Base::play(loop));
		}
		return sampleToOut(playData.sample);
	}
	void printData()
	{
		for(unsigned int n = start; n < data.size() + end; ++n)
		{
			unsigned int idx = n % data.size();
			timedData_t d = recordToTimedData(data[n]);
			printf("[%u] %lu %5.2f %s\n\r", idx, d.reps, sampleToOut(d.sample), idx == start ? "start" : (idx == end ? "end" : ""));
			if(idx == end)
				break;
		}
	}
private:
	void pushSample(){
		uint32_t r = timedDataToRecord({.reps = reps, .sample = oldSample});
		Base::record(r);
	}
	enum { kRepsMax = (1 << kRepsBits) - 1 };
	enum { kSampleMax = (1 << kSampleBits) - 1 };
	timedData_t playData = {0};
	uint32_t oldSample;
	uint16_t reps;
	bool firstSample = false;
};

//#define TWO_FINGERS_TOGGLE_ENABLE // we disable it for now because it's barely usable
#ifdef TWO_FINGERS_TOGGLE_ENABLE
class DebouncedTouches
{
	typedef uint8_t touchCount_t;
public:
	touchCount_t process(const touchCount_t newTouches)
	{
		touchCount_t ret;
		size_t backMax;
		if(newTouches > lastRet)
			backMax = kUpDebounce;
		else if (newTouches < lastRet)
			backMax = kDownDebounce;
		else
			backMax = 0;
		ret = newTouches;
		if(0 != newTouches) // going to 0 touches should not be ambiguous, so we should let it through immediately
		{
			for(unsigned int n = 0; n < backMax; ++n)
			{
				// let the new value through only if all recent past values are the same
				if(old(n) != newTouches)
				{
					#if 0
					printf("old[%d] %d %di---", n, old(n), newTouches);
					for(unsigned int n = 0; n < backMax; ++n)
						printf("%d ", old(n));
					printf("\n\r");
					#endif
					ret = lastRet;
					break;
				}
			}
		}
		history[idx++] = newTouches;
		if(idx >= history.size())
			idx = 0;
		 lastRet = ret;
		 return ret;
	}
private:
	touchCount_t old(size_t back) {
		return history[(idx - back - 1 + 2 * history.size()) % history.size()];
	}
	touchCount_t oldest() {
		return history[idx];
	}
	enum { kUpDebounce = 4 };
	enum { kDownDebounce = 8 };
	enum { kMaxDebounce = size_t(kUpDebounce) > size_t(kDownDebounce) ? size_t(kUpDebounce) : size_t(kDownDebounce) };
	std::array<touchCount_t,kMaxDebounce> history;
	size_t idx = 0;
	touchCount_t lastRet;
};
#endif // TWO_FINGERS_TOGGLE_ENABLE

class GestureRecorder
{
public:
	typedef float sample_t;
	struct HalfGesture_t{
		sample_t value;
		bool valid = false;
	};
	struct Gesture_t {
		struct HalfGesture_t first;
		struct HalfGesture_t second;
	};
	Gesture_t process(const std::vector<LedSlider>& sliders, bool loop)
	{
		if(sliders.size() < 1)
			return Gesture_t();
		bool single = (1 == sliders.size());
		std::array<unsigned int, 2> active;
#ifdef TWO_FINGERS_TOGGLE_ENABLE
		active[0] = dt[0].process(sliders[0].getNumTouches());
#else // TWO_FINGERS_TOGGLE_ENABLE
		active[0] = sliders[0].getNumTouches();
#endif // TWO_FINGERS_TOGGLE_ENABLE
		if(single)
			active[1] = active[0];
		else
		{
#ifdef TWO_FINGERS_TOGGLE_ENABLE
			active[1] = dt[1].process(sliders[1].getNumTouches());
#else // TWO_FINGERS_TOGGLE_ENABLE
			active[1] = sliders[1].getNumTouches();
#endif // TWO_FINGERS_TOGGLE_ENABLE
		}
		HalfGesture_t out[2];
		static bool pastIn = false;
		bool in = tri.analogRead() > 0.5;
		if(in && !pastIn)
		{
			assert(active.size() == rs.size());
			for(size_t n = 0; n < active.size(); ++n)
				if(!active[n])
					rs[n].enable(true);
		}
		pastIn = in;

		for(unsigned int n = 0; n < active.size(); ++n)
		{
			if(active[n] != pastActive[n]) //state change
			{
				printf("[%d] newAc: %d, pastAc: %d\n\r", n, active[n], pastActive[n]);
#ifdef TWO_FINGERS_TOGGLE_ENABLE
				if(2 == active[n] && 0 == pastActive[n]) { // two touches: toggle enable
					if(rs[n].isEnabled())
						rs[n].disable();
					else
						rs[n].enable(true); // TODO: issue with enabling here is that we may continue recording before we release
				} else
#endif // TWO_FINGERS_TOGGLE_ENABLE
				if(1 == active[n] && 0 == pastActive[n]) { // going from 0 to 1 touch: start recording (and enable)
					rs[n].startRecording();
				} else if(0 == active[n]) // going to 0 touches: start playing back (unless disabled)
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
				out[n] = { rs[n].record(val), true };
			}
			else {
				if(rs[n].size())
					out[n] = { rs[n].play(loop), true };

			}
		}
		return {out[0], out[1]};
	}
	std::array<TimestampedRecorder<sample_t,1>, 2> rs;
private:
	unsigned int pastActive[2];
#ifdef TWO_FINGERS_TOGGLE_ENABLE
	DebouncedTouches dt[2];
#endif // TWO_FINGERS_TOGGLE_ENABLE
	bool lastStateChangeWasToggling = false;
} gGestureRecorder;

static void gestureRecorderSingle_loop(bool loop)
{
	GestureRecorder::Gesture_t g = gGestureRecorder.process(ledSliders.sliders, loop);
#if 0
	static int count = 0;
	bool p = count++ % 20 == 0;
	p && printf("=%.3f %.3f\n\r", g.first, g.second);
#endif
	LedSlider::centroid_t centroid;
	if(g.first.valid && g.second.valid)
	{
		centroid.location = g.first.value;
		centroid.size = g.second.value;
		ledSliders.sliders[0].setLedsCentroids(&centroid, 1);
	}
}

static void gestureRecorderSplit_loop(bool loop)
{
	GestureRecorder::Gesture_t g = gGestureRecorder.process(ledSliders.sliders, loop);
	LedSlider::centroid_t centroid;
	if(g.first.valid)
	{
		centroid.location = g.first.value;
		centroid.size = 0.9;
		ledSliders.sliders[0].setLedsCentroids(&centroid, 1);
	}
	if(g.second.valid)
	{
		centroid.location = g.second.value;
		centroid.size = 0.9;
		ledSliders.sliders[1].setLedsCentroids(&centroid, 1);
	}
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
		
//		rt_printf("%f and %f and freqmult: %f\n", (0.92 - gDivisionPoint) * freqMult*2, gDivisionPoint * freqMult*2, freqMult);
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
		
//		rt_printf("%f and %f and freqmult: %f\n", (0.92 - gDivisionPoint) * freqMult*2, gDivisionPoint * freqMult*2, freqMult);
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

void mode9_loop()
{
	float in = tri.analogRead();
	LedSlider::centroid_t centroids[1];
	centroids[0].location = in;
	centroids[0].size = 0.9;
	ledSliders.sliders[0].setLedsCentroids(centroids, 1);
}

void mode10_loop()
{
	float scale = 0.1;
	static std::vector<float> offsets = {
			0.5,
			0.6,
			0.7,
			0.8,
			0.9,
	};
	ledSlidersExpButtonsProcess(ledSliders, gManualAnOut, scale, offsets);
}

static int gAlt = 0;

#include "bootloader.h"

static uint16_t midiInputCallback(uint8_t *msg, uint16_t length)
{
	for(unsigned int n = 0; n < length; ++n)
		printf("%02x ", msg[n]);
	if(length)
		printf("\n\r");
	// loopback the input to the output
	sendMidiMessage(msg, length);
	// program change channel 1, program 2, (i.e.: 0x0 and 0x1 respectively)
	if(0x0c == msg[0] && 0xc0 == msg[1] && 0x01 == msg[2])
	{
		printf("Jumping to bootloader\n\r");
		bootloaderResetTo();
	}
	return 0;
}

static void midiCtlCallback(uint8_t ch, uint8_t num, uint8_t value){
	static rgb_t color;
	if(0 == num)
		color.r = value * 2;
	if(1 == num)
		color.g = value * 2;
	if(2 == num)
		color.b = value * 2;
	gAlt = 2;
	printf("%d %d %d\n\r", color.r, color.g, color.b);
	for(unsigned int n = 0; n < kNumLeds; ++n)
		np.setPixelColor(n, color.r, color.g, color.b);
}

enum { kNumModes = 10 };
static bool (*mode_setups[kNumModes])(double) = {
	mode1_setup,
	mode2_setup,
	mode3_setup,
	mode4_setup,
	mode5_setup,
	mode6_setup,
	mode7_setup,
	mode8_setup,
	mode9_setup,
	mode10_setup,
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
	mode9_loop,
	mode10_loop,
};

class CalibrationProcedure {
private:
typedef enum {
	kCalibrationNoInput,
	kCalibrationWaitConnect,
	kCalibrationConnected,
	kCalibrationDone,
} Calibration_t;
Calibration_t calibrationState;
size_t count;
float unconnectedAdc;
float connectedAdc;
float anOut;
float minDiff;
float minValue;
static constexpr unsigned kCalibrationNoInputCount = 50;
static constexpr unsigned kCalibrationConnectedStepCount = 20;
static constexpr unsigned kCalibrationWaitPostThreshold = 50;
static constexpr float kCalibrationAdcConnectedThreshold = 0.1;
static constexpr float kStep = 1.0 / 4096;
static constexpr float kRangeStart = 0.30;
static constexpr float kRangeStop = 0.35;

public:
void setup()
{
	calibrationState = kCalibrationNoInput;
	count = 0;
	unconnectedAdc = 0;
	calibrationState = kCalibrationNoInput;
	printf("Disconnect INPUT\n\r"); // TODO: this is printed repeatedly till you release the button
	gOutMode = kOutModeManual;
}

void process()
{
	float anIn = tri.analogRead();
	switch (calibrationState)
	{
		case kCalibrationNoInput:
			unconnectedAdc += anIn;
			count++;
			if(kCalibrationNoInputCount == count)
			{
				calibrationState = kCalibrationWaitConnect;
				unconnectedAdc /= count;
				printf("unconnectedAdc: %.5f, connect an input\n\r", unconnectedAdc);
				anOut = 0; // set this as a test value so we can detect when DAC is connected to ADC
				count = 0;
			}
			break;
		case kCalibrationWaitConnect:
			// wait for ADC to be connected, then wait some more to avoid any spurious transients
			if(anIn < kCalibrationAdcConnectedThreshold)
			{
				if(0 == count)
					printf("Jack connected");
				count++;
			} else {
				count = 0;
			}
			if(kCalibrationWaitPostThreshold == count)
			{
				printf(", started\n\r");
				calibrationState = kCalibrationConnected;
				minDiff = 1000000000;
				minValue = 1000000000;
				count = 0;
				anOut = kRangeStart;
			}
			break;
		case kCalibrationConnected:
		{
			if(anOut >= kRangeStop)
			{
				printf("Gotten a minimum at %f (diff %f)\n\r", minValue, minDiff);
				calibrationState = kCalibrationDone;
				break;
			}
			if (count == kCalibrationConnectedStepCount) {
				connectedAdc /= (count - 1);
				float diff = connectedAdc - unconnectedAdc;
				diff = diff > 0 ? diff : -diff; // abs
				if(diff < minDiff)
				{
					minDiff = diff;
					minValue = anOut;
				}
				count = 0;
				anOut += kStep;
			}
			if(0 == count)
			{
				connectedAdc = 0;
			}
			 else if (count >= 1) {
				connectedAdc += anIn;
			}
			count++;
		}
			break;
		case kCalibrationDone:
			anOut = minValue;
			break;
	}
	gManualAnOut[0] = anOut;
}
float getGnd()
{
	return minValue;
}
bool valid()
{
	return kCalibrationDone == calibrationState;
}

} gCalibrationProcedure;


#ifdef STM32_NEOPIXEL
static Stm32NeoPixelT<uint32_t, kNumLeds> snp(&neoPixelHtim, neoPixelHtim_TIM_CHANNEL_x, 0.66 * neoPixelHtim_COUNTER_PERIOD, 0.33 * neoPixelHtim_COUNTER_PERIOD);
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

int tr_setup()
{
	np.begin();
#ifdef STM32_NEOPIXEL
	np.setSnp(&snp);
#endif // STM32_NEOPIXEL
	np.show();
#ifdef TRILL_BAR
	Trill::Device device = Trill::BAR;
	uint8_t startAddr = 0x20;
#else
	Trill::Device device = Trill::FLEX;
	uint8_t startAddr = 0x48;
#endif
	uint8_t foundAddress = 0;
	for(uint8_t addr = startAddr; addr <= startAddr + 8; ++addr)
	{
		if(!trill.setup(1, device, addr))
		{
			foundAddress = addr;
			break;
		}
	}
	if(!foundAddress)
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

	cd.setup({padsToOrderMap, padsToOrderMap + kNumPads / 2}, 4, 8000);
	modeAlt_setup();
	setHdlCtlChange(midiCtlCallback);
	setHdlAll(midiInputCallback);
	return foundAddress;
}

#ifdef TRILL_CALLBACK
void tr_newData(const uint8_t* newData, size_t len)
{
	trill.newData(newData, len);
}
#endif // TRILL_CALLBACK

#ifdef TR_LOOP_TIME_CRITICAL
void tr_process(BelaContext* ptr)
{
	tri.process(ptr);
}
#endif // TR_LOOP_TIME_CRITICAL

enum {
	kOutRangeFull,
	kOutRangeBipolar,
	kOutRangePositive5,
	kOutRangePositive10,
	kOutRangeNum,
};
// C++ doesn't allow myenumvar++, so we need this as an int
static int gOutRange = kOutRangeFull;

void tr_loop()
{
#ifdef STM32
	static std::array<uint32_t, 2> pastTicks;
	static size_t pastTicksIdx = 0;
	static size_t backoff = 0;
	// ensure that we are not taking up all CPU
	// i.e.: check that systick had a chance to run recently
	if(backoff)
		backoff--;
	if(backoff)
		return;

	uint32_t tick = HAL_GetTick();
	bool different = false;
	for(auto& t : pastTicks) {
		if(tick != t) {
			different = true;
			break;
		}
	}
	pastTicks[pastTicksIdx] = tick;
	static_assert(!((pastTicks.size() - 1) & pastTicks.size())); // ensure it's a power of 2 so that the next line works
	pastTicksIdx = (pastTicksIdx + 1) & (pastTicks.size() - 1);
	if(pastTicks.size() == pastTicksIdx)
		pastTicksIdx &= pastTicks.size() - 1;

	if(!different){
		printf("C");
		backoff = 2;
		return;
	}
#endif // STM32
	processMidiMessage();
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
	int diIn0 = tri.digitalRead(0);
	tri.buttonLedWrite(!diIn0);
	
	static bool firstRun = true;
	bool justEnteredAlt = false;
	if ((diIn0 == 0 && diIn0 != gDiIn0Last) && !firstRun){
		// button onset
		if(gAlt){ // exit from alt mode
			gAlt = false;
			np.clear();
		}
	} else
	if(!diIn0)
	{
		bool touch = false;
		for(auto& s : ledSliders.sliders)
		{
			if((touch = s.getNumTouches()))
				break;
		}
		if(touch)
		{
			//button is on + one touch: enter alt mode
			gAlt = 1;
			justEnteredAlt = true;
			np.clear();
		}
	}
	gDiIn0Last = diIn0;

	static int shouldChangeMode = 1;
	if(1 == gAlt)
	{
		ledSlidersAlt.process(trill.rawData.data());
		static const size_t numButtons = ledSlidersAlt.sliders.size();
		static std::vector<bool> altStates(numButtons);
		static std::vector<size_t> onsets(numButtons);
		static std::vector<size_t> offsets(numButtons);
		static bool isCalibration;
		if(justEnteredAlt)
			isCalibration = false;
		if(isCalibration)
		{
			gCalibrationProcedure.process();
			if(gCalibrationProcedure.valid())
			{
				// once done, let's get out of calibration
				isCalibration = false;
			}
		}
		else {
			// if we have just entered, only update states
			ledSlidersFixedButtonsProcess(ledSlidersAlt, altStates, onsets, offsets, justEnteredAlt);
			// see if a button was pressed
			if(onsets.size())
			{
				// only consider one touch
				const unsigned int button = onsets[0];
				if(0 == button)
					shouldChangeMode = -1;
				else if (1 == button)
				{
					gCalibrationProcedure.setup();
					isCalibration = true;
				}
				else if(2 == button)
				{
					// cycle through out ranges
					gOutRange = gOutRange + 1;
					if(kOutRangeNum == gOutRange)
						gOutRange = kOutRangeFull;
					printf("Range: %d\n\r", gOutRange);
				}
				else if(numButtons - 1 == button)
					shouldChangeMode = 1;
			}
		}
	}

	static double setupMs = 0;
	static bool setupDone = false;
	if(shouldChangeMode) {
		setupMs = tri.getTimeMs();
		if(!firstRun)
			gMode = (gMode + shouldChangeMode + kNumModes) % kNumModes;
		printf("mode: %d\n\r", gMode);
		setupDone = false;
		shouldChangeMode = 0;
	}
	firstRun = false;

	if(!setupDone)
		setupDone = mode_setups[gMode](tri.getTimeMs() - setupMs);
	if(setupDone)
	{
		if(!gAlt) {
			ledSliders.process(trill.rawData.data());
			mode_loops[gMode](); // TODO: we should run the active mode even if we are in alt, but making sure the LEDs don't get set
		}
	}
	// actually display the updated LEDs
	// this may have been written by alt, mode_setups or mode_loops, whatever last wrote it is whatever we display
	// TODO: clear separation of concerns: at any time make it clear who can write to each pixel.
	np.show();
//	tri.buttonLedWrite(gMtrClkTriggerLED);
	
	// write analog outputs
	auto& sls = ledSliders.sliders;
	switch (gOutMode)
	{
		case kOutModeFollowTouch:
			gManualAnOut[0] = sls[0].compoundTouchLocation();
			if(1 == sls.size())
				gManualAnOut[1] = sls[0].compoundTouchSize();
			else if (2 == sls.size())
				gManualAnOut[1] = sls[1].compoundTouchLocation();
			break;
		case kOutModeFollowLeds:
			gManualAnOut[0] = sls[0][0].location;
			if(1 == sls.size())
				gManualAnOut[1] = sls[0][0].size;
			else if (2 == sls.size())
				gManualAnOut[1] = sls[1][0].location;
			break;
		case kOutModeManual:
			// everything should have been done already.
			break;
	}
	// actually write analog outs
	for(unsigned int n = 0; n < gManualAnOut.size(); ++n)
		tri.analogWrite(n, gManualAnOut[n]);
	
	// Send to scope
	tri.scopeWrite(0, anIn);
	tri.scopeWrite(1, ledSliders.sliders[0].compoundTouchLocation());
	tri.scopeWrite(2, ledSliders.sliders[0].compoundTouchSize());
#ifndef TR_LOOP_TIME_CRITICAL
	usleep(kLoopSleepTimeUs);
#endif // TR_LOOP_TIME_CRITICAL
}
