#include "TrillRackInterface.h"
#include "GlissModes.h"
#include <vector>
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include <libraries/Oscillator/Oscillator.h>
#include "LedSliders.h"
#include "preset.h"
#include "packed.h"

static_assert(kNumOutChannels >= 2); // too many things to list depend on this in this file.

//#define TRIGGER_IN_TO_CLOCK_USES_MOVING_AVERAGE

// pick one of the two for more or less debugging printf and memory usage
//#define S(a) a
#define S(a)

extern int gAlt;
extern TrillRackInterface tri;
extern const unsigned int kNumPads;
extern const unsigned int kNumLeds;
extern int gOutRange;
extern float gOutRangeBottom;
extern float gOutRangeTop;
extern int gInRange;
extern float gInRangeBottom;
extern float gInRangeTop;
extern std::vector<unsigned int> padsToOrderMap;
extern NeoPixel np;
extern Trill trill;
extern std::array<float,2> gManualAnOut;

std::array<Oscillator, 2> oscillators;
const std::array<rgb_t, 2> gBalancedLfoColorsInit = {{{0, 0, 255}, {0, 255, 0}}};
std::array<rgb_t, 2> gBalancedLfoColors; // copy so that we can set them via MIDI without changing defaults

OutMode gOutMode = kOutModeFollowTouch;
int gCounter = 0;
int gSubMode = 0;
bool gSecondTouchIsSize;
bool gJacksOnTop = false;

// Recording the gesture
enum { kMaxRecordLength = 10000 };
const float kSizeScale = 10000;
const float kFixedCentroidSize = 0.3;

LedSliders ledSliders;
LedSliders ledSlidersAlt;
ButtonView menuBtn;
ButtonView performanceBtn;
extern CentroidDetection globalSlider;

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

uint32_t gClockPeriodUpdated = 0;
float gClockPeriod = 0; // before use, make sure it is valid
void triggerInToClock(BelaContext* context)
{
	const float kTriggerInOnThreshold = 0.6;
	const float kTriggerInOffThreshold = kTriggerInOnThreshold - 0.05;
	static bool lastTrigPrimed = false;
	static size_t lastTrig = 0;
	for(size_t n = 0; n < context->analogFrames; ++n)
	{
		static bool lastIn = false;
		// hysteresis
		float threshold = lastIn ? kTriggerInOffThreshold : kTriggerInOnThreshold;
		bool in = analogRead(context, n, 0) > threshold;
		if(in && !lastIn)
		{
			size_t newTrig = context->audioFramesElapsed  + n;
			if(lastTrigPrimed)
			{
				size_t newPeriod = newTrig - lastTrig;
#ifdef TRIGGER_IN_TO_CLOCK_USES_MOVING_AVERAGE
				enum { kInferClockNumPeriods = 5 };
				static size_t lastPeriods[kInferClockNumPeriods] = {0};
				static size_t currentPeriod = 0;
				static size_t periodSum = 0;
				// moving average
				periodSum -= lastPeriods[currentPeriod];
				lastPeriods[currentPeriod] = newPeriod;
				periodSum += newPeriod;
				// TODO: it may be that when the clock is changing it's best to follow
				// it without moving average, especially if it's slow
				float averagePeriod = float(periodSum) / kInferClockNumPeriods;
				currentPeriod++;
				if(currentPeriod == kInferClockNumPeriods)
					currentPeriod = 0;
				gClockPeriod = averagePeriod;
#else // TRIGGER_IN_TO_CLOCK_USES_MOVING_AVERAGE
				gClockPeriod = newPeriod;
				gClockPeriodUpdated++;
#endif // TRIGGER_IN_TO_CLOCK_USES_MOVING_AVERAGE
			}
			lastTrig = newTrig;
			lastTrigPrimed = true;
		}

		lastIn = in;
	}
}

static void ledSlidersSetupMultiSlider(LedSliders& ls, std::vector<rgb_t> const& colors, const LedSlider::LedMode_t& mode, bool setInitial, size_t maxNumCentroids = 1)
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
			.order = padsToOrderMap,
			.sizeScale = kSizeScale,
			.boundaries = boundaries,
			.maxNumCentroids = {maxNumCentroids},
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
#if 0
template <size_t T>
static void ledSlidersExpButtonsProcess(LedSliders& sl, std::array<float,2>& outs, float scale, std::array<float,T>const& offsets = {})
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
void ledSlidersFixedButtonsProcess(LedSliders& sl, std::vector<bool>& states, std::vector<size_t>& onsets, std::vector<size_t>& offsets, bool onlyUpdateStates)
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
#endif

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
			np.setPixelColor(n, colors[0].r, colors[0].g, colors[0].b);
		for(unsigned int n = startSecond; n < kNumLeds; ++n)
			np.setPixelColor(n, colors[1].r, colors[1].g, colors[1].b);
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
bool modeAlt_setup()
{
	ledSlidersSetupMultiSlider(
		ledSlidersAlt,
		{
			{0, 0, 255},
			{0, 0, 255},
			{0, 0, 255},
			{0, 0, 255},
			{0, 255, 0},
		},
		LedSlider::MANUAL_CENTROIDS,
		true
	);
	return true;
}

struct TouchFrame {
	float pos;
	float sz;
};
// A circular buffer. When touch sz goes to zero, retrieve the oldest
// element in the buffer so we can hope to get more stable values instead
// of whatever spurious reading we got while releasing the touch
class AutoLatcher {
public:
	AutoLatcher() {
		reset();
	}
	void reset()
	{
		// TODO: count valid frames so that for short touches
		// we don't latch on to garbage
	}
	// return: sets frame and latchStarts
	void process(TouchFrame& frame, bool& latchStarts)
	{
		size_t pastIdx = (idx - 1 + pastFrames.size()) % pastFrames.size();
		// filter out duplicate frames
		// TODO: call this per each new frame instead
		if(pastFrames[pastIdx].sz == frame.sz && pastFrames[pastIdx].pos == frame.pos)
			return;
		if(pastFrames[pastIdx].sz && !frame.sz) // if size went to zero
		{
			// use the oldest frame we have
			frame = pastFrames[idx];
			latchStarts = true;
			pastFrames[idx].pos = pastFrames[idx].sz = 0;
		} else {
			// if we are still touching
			// store current value for later
			pastFrames[idx] = frame;
		}
		++idx;
		if(idx >= pastFrames.size())
			idx = 0;
	}
private:
	static constexpr size_t kHistoryLength = 5;
	std::array<TouchFrame,kHistoryLength> pastFrames;
	size_t idx = 0;
};

class LatchProcessor {
public:
	LatchProcessor() {
		reset();
	}
	void reset()
	{
		isLatched = {false, false};
		unlatchArmed = {false, false};
		latchedValues = {0, 0};
		for(auto& al : autoLatchers)
			al.reset();
	}
	void process(bool autoLatch, size_t numValues, std::array<TouchFrame,2>& values, std::array<bool,2>& isLatchedRet,
			bool shouldLatch = false, bool shouldUnlatch = false)
	{
		if(numValues > kMaxNumValues)
			numValues = kMaxNumValues;
		std::array<bool,kMaxNumValues> hasTouch = { values[0].sz > 0, values[1].sz > 0 };
		std::array<bool,kMaxNumValues> latchStarts = {false, false};
		std::array<bool,kMaxNumValues> unlatchStarts = {false, false};

		// button latches everything if there is at least one touch
		// and unlatches everything if there is no touch
		if(shouldLatch)
		{
			for(size_t n = 0; n < numValues; ++n)
			{
				if(!isLatched[n] && hasTouch[n])
					latchStarts[n] = true;
			}
		} else if (shouldUnlatch) {
			if(!(hasTouch[0] || hasTouch[numValues - 1]))
			{
				for(size_t n = 0; n < numValues; ++n)
				{
					// no touch
					if(isLatched[n])
						unlatchStarts[n] = true;
				}
			}
		}
		if(autoLatch)
		{
			// try to hold without button
			for(size_t n = 0; n < numValues; ++n)
				autoLatchers[n].process(values[n], latchStarts[n]);
		}
		for(size_t n = 0; n < numValues; ++n)
		{
			if(isLatched[n])
			{
				// if it's latched (and you have release your finger),
				// you can unlatch and go back
				// to direct control simply by touching again
				if(!hasTouch[n])
					unlatchArmed[n] = true;
				if(unlatchArmed[n] && hasTouch[n])
					unlatchStarts[n] = true;
			}
			if(latchStarts[n])
			{
				latchedValues[n] = values[n];
				isLatched[n] = true;
				unlatchArmed[n] = false;
			}
			if(unlatchStarts[n])
				isLatched[n] = false;
		}
		for(size_t n = 0; n < numValues; ++n)
		{
			if(isLatched[n])
				values[n] = latchedValues[n];
			// or leave them untouched
		}
		isLatchedRet = isLatched;
	}
private:
	static constexpr size_t kMaxNumValues = 2;
	std::array<bool,kMaxNumValues> isLatched;
	std::array<bool,kMaxNumValues> unlatchArmed;
	std::array<TouchFrame,kMaxNumValues> latchedValues;
	std::array<AutoLatcher,kMaxNumValues> autoLatchers;
};

template <typename sample_t>
class Recorder
{
public:
	void enable()
	{
		active = true;
	}
	void restart()
	{
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
		start = current = 0;
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
			if(loop) {
				current = start;
				// make sure we go back to active in case
				// loop has just become true and we are currently !active
				active = true;
			}
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
		size_t size = data.size();
		size_t i = (end - start + size) % size;
		return i;
	}

	const std::array<sample_t, kMaxRecordLength>& getData()
	{
		return data;
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
public:
	struct timedData_t
	{
		uint32_t reps : kRepsBits;
		uint32_t sample : kSampleBits;
	};
	static_assert(sizeof(timedData_t) <= 4); // if you change field values to be larger than 4 bytes, be well aware of that
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
		restart();
	}

	void restart()
	{
		playData.reps = 0;
		Recorder::restart();
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
			printf("[%u] %lu %5.2f %s\n\r", idx, uint32_t(d.reps), sampleToOut(d.sample), idx == start ? "start" : (idx == end ? "end" : ""));
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
		std::array<unsigned int, 2> hasTouch;
		hasTouch[0] = sliders[0].getNumTouches();
		if(single)
			hasTouch[1] = hasTouch[0];
		else
		{
			hasTouch[1] = sliders[1].getNumTouches();
		}
		HalfGesture_t out[2];
		static bool pastAnalogIn = false;
		// TODO: ignore when recording
		// TODO: obey trigger level
		bool analogIn = tri.analogRead() > 0.5;
		if((analogIn && !pastAnalogIn) || performanceBtn.onset)
		{
			assert(hasTouch.size() == rs.size());
			// when playing back or paused,
			// reset on rising edge on analog or button ins
			for(size_t n = 0; n < hasTouch.size(); ++n)
				if(!hasTouch[n])
				{
					rs[n].enable();
					rs[n].restart();
				}
		}
		pastAnalogIn = analogIn;

		for(unsigned int n = 0; n < hasTouch.size(); ++n)
		{
			if(hasTouch[n] != hadTouch[n]) //state change
			{
				printf("[%d] hasTouch: %d, hadTouch: %d\n\r", n, hasTouch[n], hadTouch[n]);
				if(1 == hasTouch[n] && 0 == hadTouch[n]) { // going from 0 to 1 touch: start recording (and enable)
					rs[n].startRecording();
				} else if(0 == hasTouch[n]) // going to 0 touches: start playing back (unless disabled)
					rs[n].stopRecording();
			}
			hadTouch[n] = hasTouch[n];
			if(hasTouch[n])
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
	unsigned int hadTouch[2];
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
		centroid.size = kFixedCentroidSize;
		ledSliders.sliders[0].setLedsCentroids(&centroid, 1);
	}
	if(g.second.valid)
	{
		centroid.location = g.second.value;
		centroid.size = kFixedCentroidSize;
		ledSliders.sliders[1].setLedsCentroids(&centroid, 1);
	}
}

class Parameter {
public:
	bool same(Parameter& p) const
	{
		return (this == &p);
	}
};

class ParameterUpdateCapable {
public:
	virtual void updated(Parameter&) {};
	virtual void updatePreset() = 0;
};

class PerformanceMode : public ParameterUpdateCapable {
public:
	virtual bool setup(double ms) = 0;
	virtual void render(BelaContext*) = 0;
};

class ParameterEnum : public Parameter {
public:
	virtual void set(unsigned int) = 0;
	virtual void next() = 0;
	virtual uint8_t get() const = 0;
	virtual uint8_t getMax() const = 0;
};

template <uint8_t T>
class ParameterEnumT : public ParameterEnum
{
public:
	ParameterEnumT<T>(ParameterUpdateCapable* that, uint8_t value = 0):
		that(that), value(value) {}

	void set(unsigned int newValue) override
	{
		value = newValue - 1;
		next();
	}
	void next() override
	{
		value++;
		if(value >= T)
			value = 0;
		that->updated(*this);
	}
	uint8_t get() const override
	{
		return value;
	}
	uint8_t getMax() const override
	{
		return T;
	}
	operator uint8_t() { return value; }
private:
	ParameterUpdateCapable* that;
	uint8_t value;
};

class ParameterContinuous : public Parameter {
public:
	ParameterContinuous(ParameterUpdateCapable* that, float value = 0) : that(that), value(value) {}
	void set(float newValue)
	{
		value = newValue;
		that->updated(*this);
	}
	float get() const
	{
		return value;
	}
	operator float() { return get(); }
private:
	ParameterUpdateCapable* that;
	float value;
};

static float simpleTriangle(unsigned int phase, unsigned int period)
{
	phase = phase % period;
	unsigned int hp = period / 2;
	float value;
	if(phase < hp)
		value = phase / float(hp);
	else
		value = (2 * hp - phase) / float(hp);
	return value;
}

#ifdef TEST_MODE
class TestMode: public PerformanceMode {
public:
	bool setup(double ms) override
	{
		gOutMode = kOutModeFollowLeds;
		ledSlidersSetupTwoSliders(1, colorDefs[0], LedSlider::MANUAL_CENTROIDS);
		for(unsigned int n = 0; n < kNumLeds; ++n)
			np.setPixelColor(n, 0, 0, 0);
		return true;
	}
	void render(BelaContext* context)
	{
		bool hasTouch = (globalSlider.compoundTouchSize() > 0);
		if(hasTouch  & !hadTouch)
		{
			count = 0;
			nextState();
		}
		hadTouch = hasTouch;
		const uint32_t period = 1024;
		float pos = 0.8 * simpleTriangle(count, period) + 0.1;
		count++;
		count %= period;

		LedSlider::centroid_t centroid = { location: pos, size: 1};
		for(auto& s : ledSliders.sliders)
			s.setLedsCentroids(&centroid, 1);
	}
private:
	void nextState(){
		state = !state;
		ledSliders.sliders[0].setColor(colorDefs[state][0]);
		ledSliders.sliders[1].setColor(colorDefs[state][1]);
	}
	rgb_t colorDefs[2][2] = {
			{
				{192, 128, 64},
				{145, 110, 0},
			},
			{
				{32, 64, 96},
				{0, 55, 72},
			}
	};
	uint32_t count = 0;
	bool hadTouch = false;
	bool state = 0;
} gTestMode;
#endif // TEST_MODE

#define genericDefaulter2(CLASS,A,B) \
[](PresetField_t field, PresetFieldSize_t size, void* data) \
{ \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	pfd->A = that->A; \
	pfd->B = that->B; \
}

#define genericDefaulter3(CLASS,A,B,C) \
[](PresetField_t field, PresetFieldSize_t size, void* data) \
{ \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	pfd->A = that->A; \
	pfd->B = that->B; \
	pfd->C = that->C; \
}

#define genericDefaulter2PlusArray(CLASS,A,B,C) \
[](PresetField_t field, PresetFieldSize_t size, void* data) \
{ \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	pfd->A = that->A; \
	pfd->B = that->B; \
	for(size_t n = 0; n < that->C.size(); ++n) \
		pfd->C[n] = that->C[n]; \
}

#define genericDefaulter7(CLASS,A,B,C,D,E,F,G) \
[](PresetField_t field, PresetFieldSize_t size, void* data) \
{ \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	pfd->A = that->A; \
	pfd->B = that->B; \
	pfd->C = that->C; \
	pfd->D = that->D; \
	pfd->E = that->E; \
	pfd->F = that->F; \
	pfd->G = that->G; \
}

#define genericDefaulter9(CLASS,A,B,C,D,E,F,G,H,I) \
[](PresetField_t field, PresetFieldSize_t size, void* data) \
{ \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	pfd->A = that->A; \
	pfd->B = that->B; \
	pfd->C = that->C; \
	pfd->D = that->D; \
	pfd->E = that->E; \
	pfd->F = that->F; \
	pfd->G = that->G; \
	pfd->H = that->H; \
	pfd->I = that->I; \
}

#define genericLoadCallback2(CLASS,A,B) \
[](PresetField_t field, PresetFieldSize_t size, const void* data) { \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	that->A.set(pfd->A); that->presetFieldData.A = pfd->A; \
	that->B.set(pfd->B); that->presetFieldData.B = pfd->B; \
}

#define genericLoadCallback3(CLASS,A,B,C) \
[](PresetField_t field, PresetFieldSize_t size, const void* data) { \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	that->A.set(pfd->A); that->presetFieldData.A = pfd->A; \
	that->B.set(pfd->B); that->presetFieldData.B = pfd->B; \
	that->C.set(pfd->C); that->presetFieldData.C = pfd->C; \
}

#define genericLoadCallback2PlusArray(CLASS,A,B,C) \
[](PresetField_t field, PresetFieldSize_t size, const void* data) { \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	that->A.set(pfd->A); that->presetFieldData.A = pfd->A; \
	that->B.set(pfd->B); that->presetFieldData.B = pfd->B; \
	for(size_t n = 0; n < that->C.size(); ++n) { \
		that->C[n].set(pfd->C[n]); that->presetFieldData.C[n] = pfd->C[n]; \
	} \
}

#define genericLoadCallback7(CLASS,A,B,C,D,E,F,G) \
[](PresetField_t field, PresetFieldSize_t size, const void* data) { \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	that->A.set(pfd->A); that->presetFieldData.A = pfd->A; \
	that->B.set(pfd->B); that->presetFieldData.B = pfd->B; \
	that->C.set(pfd->C); that->presetFieldData.C = pfd->C; \
	that->D.set(pfd->D); that->presetFieldData.D = pfd->D; \
	that->E.set(pfd->E); that->presetFieldData.E = pfd->E; \
	that->F.set(pfd->F); that->presetFieldData.F = pfd->F; \
	that->G.set(pfd->G); that->presetFieldData.G = pfd->G; \
}

#define genericLoadCallback9(CLASS,A,B,C,D,E,F,G,H,I) \
[](PresetField_t field, PresetFieldSize_t size, const void* data) { \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	that->A.set(pfd->A); that->presetFieldData.A = pfd->A; \
	that->B.set(pfd->B); that->presetFieldData.B = pfd->B; \
	that->C.set(pfd->C); that->presetFieldData.C = pfd->C; \
	that->D.set(pfd->D); that->presetFieldData.D = pfd->D; \
	that->E.set(pfd->E); that->presetFieldData.E = pfd->E; \
	that->F.set(pfd->F); that->presetFieldData.F = pfd->F; \
	that->G.set(pfd->G); that->presetFieldData.G = pfd->G; \
	that->H.set(pfd->H); that->presetFieldData.H = pfd->H; \
	that->I.set(pfd->I); that->presetFieldData.I = pfd->I; \
}

template <typename T>
static bool areEqual(const T& a, const T& b)
{
	return !memcmp(&a, &b, sizeof(T));
}

#define UPDATE_PRESET_FIELD2(A,B) \
{ \
	PresetFieldData_t bak = presetFieldData; \
	presetFieldData.A = A; \
	presetFieldData.B = B; \
	if(!areEqual(bak, presetFieldData)) \
		presetSetField(this, &presetFieldData); \
}

#define UPDATE_PRESET_FIELD3(A,B,C) \
{ \
	PresetFieldData_t bak = presetFieldData; \
	presetFieldData.A = A; \
	presetFieldData.B = B; \
	presetFieldData.C = C; \
	if(!areEqual(bak, presetFieldData)) \
		presetSetField(this, &presetFieldData); \
}

#define UPDATE_PRESET_FIELD2PlusArray(A,B,C) \
{ \
	bool same = true; \
	for(size_t n = 0; n < C.size(); ++n) \
	{ \
		if(C[n] != presetFieldData.C[n]) \
		{ \
			same = false; \
			break; \
		} \
	} \
	if(presetFieldData.A != A || presetFieldData.B != B || !same) { \
		presetFieldData.A = A; \
		presetFieldData.B = B; \
		for(size_t n = 0; n < C.size(); ++n) \
			presetFieldData.C[n] = C[n]; \
		presetSetField(this, &presetFieldData); \
	} \
}

#define UPDATE_PRESET_FIELD7(A,B,C,D,E,F,G) \
{ \
	PresetFieldData_t bak = presetFieldData; \
	presetFieldData.A = A; \
	presetFieldData.B = B; \
	presetFieldData.C = C; \
	presetFieldData.D = D; \
	presetFieldData.E = E; \
	presetFieldData.F = F; \
	presetFieldData.G = G; \
	if(!areEqual(bak, presetFieldData)) \
		presetSetField(this, &presetFieldData); \
}

#define UPDATE_PRESET_FIELD9(A,B,C,D,E,F,G,H,I) \
{ \
	PresetFieldData_t bak = presetFieldData; \
	presetFieldData.A = A; \
	presetFieldData.B = B; \
	presetFieldData.C = C; \
	presetFieldData.D = D; \
	presetFieldData.E = E; \
	presetFieldData.F = F; \
	presetFieldData.G = G; \
	presetFieldData.H = H; \
	presetFieldData.I = I; \
	if(!areEqual(bak, presetFieldData)) \
		presetSetField(this, &presetFieldData); \
}

class DirectControlMode : public PerformanceMode {
public:
	bool setup(double ms) override
	{
		gSecondTouchIsSize = !split;
		if(split)
		{
			unsigned int guardPads = 1;
			if(ms <= 0)
			{
				ledSlidersSetupTwoSliders(guardPads, colors, LedSlider::AUTO_CENTROIDS);
				gOutMode = kOutModeFollowTouch;
			}
			if(ms < 0)
				return true;
			return modeChangeBlinkSplit(ms, colors, kNumLeds / 2 - guardPads, kNumLeds / 2);
		} else {
			if(ms <= 0)
			{
				ledSlidersSetupOneSlider(
					colors[0],
					LedSlider::AUTO_CENTROIDS
				);
				gOutMode = kOutModeFollowTouch;
			}
			if(ms < 0)
				return true;;
			return modeChangeBlink(ms, colors[0]);
		}
	}
	void render(BelaContext*) override
	{
		std::array<TouchFrame,2> values;
		values[0].pos = ledSliders.sliders[0].compoundTouchLocation();
		values[0].sz = ledSliders.sliders[0].compoundTouchSize();
		if(split)
		{
			values[1].pos = ledSliders.sliders[1].compoundTouchLocation();
			values[1].sz = ledSliders.sliders[1].compoundTouchSize();
		}

		bool shouldLatch = false;
		bool shouldUnlatch = false;
		if(performanceBtn.onset)
		{
			// at least one VALID and non-latched
			// (isLatched[split] is same as isLatched[0] if not split)
			if(!isLatched[0] || !isLatched[split])
				shouldLatch = true;
		}
		if(performanceBtn.offset)
		{
			// if it's not the same press that triggered the latch, unlatch
			if(lastLatchCount != performanceBtn.pressCount)
			{
				shouldUnlatch = true;
				lastLatchCount = ButtonView::kPressCountInvalid;
			}
		}
		// sets values and isLatched
		latchProcessor.process(autoLatch, 1 + split, values, isLatched, shouldLatch, shouldUnlatch);
		if(shouldLatch && (isLatched[0] || isLatched[split]))
		{
			// keep note of current press
			lastLatchCount = performanceBtn.pressCount;
		}

		if(isLatched[0] || isLatched[1]) {
			gOutMode = kOutModeFollowLeds;
			LedSlider::centroid_t centroid;
			if(split)
			{
				for(ssize_t n = 0; n < 1 + split; ++n)
				{
					if(isLatched[n])
					{
						centroid.location = values[n].pos;
						centroid.size = kFixedCentroidSize;
					} else {
						// this is not actually latched, but as gOutMode
						// is not latched, we emulate direct control here.
						// TODO: be able to set gOutMode per-split
						centroid.location = values[n].pos;
						bool hasTouch = (ledSliders.sliders[n].compoundTouchSize() > 0);
						centroid.size = kFixedCentroidSize * hasTouch; // TODO: when bipolar this should send out "nothing"
					}
					ledSliders.sliders[n].setLedsCentroids(&centroid, 1);
				}
			} else {
				centroid.location = values[0].pos;
				centroid.size = values[0].sz;
				ledSliders.sliders[0].setLedsCentroids(&centroid, 1);
			}
		}
		else
			gOutMode = kOutModeFollowTouch;
	}
	void updated(Parameter& p)
	{
		if(p.same(split)) {
			printf("DirectControlMode: updated split: %d\n\r", split.get());
			setup(-1);
		}
		else if (p.same(autoLatch)) {
			printf("DirectControlMode: updated autoLatch: %d\n\r", autoLatch.get());
		}
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD2(split, autoLatch);
	}
	DirectControlMode() :
		presetFieldData{
			.split = split,
			.autoLatch = autoLatch,
		}
	{
		PresetDesc_t presetDesc = {
			.field = this,
			.size = sizeof(PresetFieldData_t),
			.defaulter = genericDefaulter2(DirectControlMode, split, autoLatch),
			.loadCallback = genericLoadCallback2(DirectControlMode, split, autoLatch),
		};
		presetDescSet(0, &presetDesc);
	}
	ParameterEnumT<2> split{this, false};
	ParameterEnumT<2> autoLatch{this, false};
	PACKED_STRUCT(PresetFieldData_t {
		uint8_t split;
		uint8_t autoLatch;
	}) presetFieldData;
private:
	rgb_t colors[2] = {
		{255, 0, 0},
		{255, 0, 127},
	};
	LatchProcessor latchProcessor;
	std::array<bool,2> isLatched = {false, false};
	uint32_t lastLatchCount = ButtonView::kPressCountInvalid;
} gDirectControlMode;

static float linearInterpolation(float frac, float pastValue, float value)
{
	return (1.f - frac) * pastValue + frac * value;
}

#include <math.h>

static inline float vToFreq(float volts, float baseFreq)
{
	float semitones = volts * 12.f; // 1V/oct
	//TODO: use an optimised version of powf() so it can run more often cheaper
	return powf(2, semitones / 12.f) * baseFreq;
}

template <typename T>
static float interpolatedRead(const T& table, float idx)
{
	float n = table.size() * idx;
	size_t prev = size_t(n);
	size_t next = size_t(n + 1);
	if(prev >= table.size()) // idx was out of range
		prev = table.size() - 1;
	if(next >= table.size())
		next = 0; // could be we are at the end of table
	float frac = n - prev;
	float value = linearInterpolation(frac, table[prev], table[next]);
	return value;
}

class RecorderMode : public PerformanceMode {
	enum {
		kInputModeTrigger,
		kInputModeCv,
		kInputModePhasor,
		kInputModeNum,
	};
public:
	bool setup(double ms) override
	{
		gOutMode = kOutModeFollowLeds;
		gSecondTouchIsSize = !split;
		if(split)
		{
			unsigned int guardPads = 1;
			if(ms <= 0)
				ledSlidersSetupTwoSliders(guardPads, colors, LedSlider::MANUAL_CENTROIDS);
			if(ms < 0)
				return true;
			return modeChangeBlinkSplit(ms, colors, kNumLeds / 2 - guardPads, kNumLeds / 2);
		}
		else
		{
			if(ms <= 0)
				ledSlidersSetupOneSlider(colors[0], LedSlider::MANUAL_CENTROIDS);
			if(ms < 0)
				return true;
			return modeChangeBlink(ms, colors[0]);
		}
	}
	void render(BelaContext* context) override
	{
		bool hasTouch = ledSliders.sliders[0].getNumTouches();
		if(kInputModeTrigger == inputMode || hasTouch || hadTouch)
		{
			if(split)
				gestureRecorderSplit_loop(retrigger);
			else
				gestureRecorderSingle_loop(retrigger);
		}
		if(kInputModeTrigger == inputMode)
		{
			gOutMode = kOutModeFollowLeds;
		} else {
			gOutMode = kOutModeManualSample;
			if((hadTouch && !hasTouch) || inputModeShouldUpdateTable){
				updateTable();
				inputModeShouldUpdateTable = false;
			}
			else
				processTable(context);
		}
		hadTouch = hasTouch;
	}

	void processTable(BelaContext* context)
	{
		float vizOuts[2];
		if(kInputModeCv == inputMode)
		{
			//TODO: disable input scaling, use it as CV offset for tuning
			float volts = analogRead(context, 0, 0) * 15.f - 5.f;
			float baseFreq = 50;
			float freq = vToFreq(volts, baseFreq);
			for(size_t n = 0; n < context->analogFrames; ++n)
			{
				for(size_t c = 0; c < context->analogOutChannels && c < tables.size(); ++c)
				{
					float idx = map(osc.process(freq / context->analogSampleRate), -1, 1, 0, 1);
					float value = interpolatedRead(tables[c], idx);
					analogWriteOnce(context, n, c, value);
					if(0 == n)
						vizOuts[c] = value;
				}
			}
		} else if (kInputModePhasor == inputMode) {
			for(size_t n = 0; n < context->analogFrames; ++n)
			{
				float idx = analogRead(context, n, 0);
				for(size_t c = 0; c < context->analogOutChannels && c < tables.size(); ++c)
				{
					float out = interpolatedRead(tables[c], idx);
					analogWriteOnce(context, n, c, out);
					if(0 == n)
						vizOuts[c] = out;
				}
			}
		}
		// do visualisation
		std::array<LedSlider::centroid_t,2> centroids;
		if(split)
		{
			centroids[0].location = vizOuts[0];
			centroids[0].size = kFixedCentroidSize;
			centroids[1].location = vizOuts[1];
			centroids[1].size = kFixedCentroidSize;
		} else {
			centroids[0].location = vizOuts[0];
			centroids[0].size = vizOuts[0];
		}
		ledSliders.sliders[0].setLedsCentroids(centroids.data(), 1);
		if(split)
			ledSliders.sliders[1].setLedsCentroids(centroids.data() + 1, 1);
	}
	void updateTable()
	{
		// std::array<TimestampedRecorder<sample_t,1>, 2>
		auto& recorders = gGestureRecorder.rs;
		for(size_t c = 0; c < recorders.size() && c < tables.size(); ++c)
		{
			auto& data = recorders[c].getData();
			size_t srcEntries = recorders[c].size();
			// go through the whole recording first to compute its length
			size_t srcSize = 0;
			for(size_t n = 0; n < srcEntries; ++n)
			{
				TimestampedRecorder<GestureRecorder::sample_t,1>::timedData_t timedData;
				timedData = TimestampedRecorder<GestureRecorder::sample_t,1>::recordToTimedData(data[n]);
				srcSize += timedData.reps;
			}
			printf("srcSize: %u frames in %u entries\n\r", srcSize, srcEntries);
			// go through the whole recording again and fit it into the fixed-size table
			size_t srcIdx = 0;
			size_t dstIdx = 0;
			size_t dstSize = tables[c].size();
			size_t pastDstIdx = 0;
			float pastValue = 0;
			for(size_t n = 0; n <= srcEntries; ++n)
			{
				float value;
				size_t srcInc;
				if(n < srcEntries)
				{
					TimestampedRecorder<GestureRecorder::sample_t,1>::timedData_t timedData;
					timedData = TimestampedRecorder<GestureRecorder::sample_t,1>::recordToTimedData(data[n]);
					value = TimestampedRecorder<GestureRecorder::sample_t,1>::sampleToOut(timedData.sample);
					dstIdx = float(srcIdx) / float(srcSize) * dstSize;
					srcInc = timedData.reps;
				} else {
					// if we are at the end, interpolate back towards the first value
					value = tables[c][0];
					dstIdx = dstSize;
					srcInc = 0;
				}
				if(0 == n)
					pastValue = value;
				for(size_t i = pastDstIdx; i < dstIdx && i < dstSize; ++i)
				{
					// TODO: smooth sharp edges to reduce nasty aliasing
					float den = dstIdx - pastDstIdx - 1;
					float interp;
					float frac = 0;
					if(den) {
						frac = (i - pastDstIdx) / den;
						interp = linearInterpolation(frac, pastValue, value);
					} else {
						interp = value;
					}
					tables[c][i] = interp;
//					if(0 == c)
//						printf("%u %.2f %.2f \n\r", i, value, interp);
				}
				pastValue = value;;
				srcIdx += srcInc;
				pastDstIdx = dstIdx;
			}
		}
	}
	void updated(Parameter& p)
	{
		if(p.same(split)) {
			printf("RecorderMode: Updated split: %d\n\r", split.get());
			setup(-1);
		}
		else if (p.same(retrigger)) {
			printf("RecorderMode: Updated retrigger %d\n\r", retrigger.get());
		} else if (p.same(inputMode)) {
			printf("RecorderMode: Updated inputMode: %d\n\r", inputMode.get());
			if(inputMode != kInputModeTrigger)
				inputModeShouldUpdateTable = true;
		}
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD3(split, retrigger, inputMode);
	}
	RecorderMode() :
		presetFieldData {
			.split = split,
			.retrigger = retrigger,
			.inputMode = inputMode,
		}
	{
		PresetDesc_t presetDesc = {
			.field = this,
			.size = sizeof(PresetFieldData_t),
			.defaulter = genericDefaulter3(RecorderMode, split, retrigger, inputMode),
			.loadCallback = genericLoadCallback3(RecorderMode, split, retrigger, inputMode),
		};
		presetDescSet(1, &presetDesc);
	}
	ParameterEnumT<2> split{this, false};
	ParameterEnumT<2> retrigger{this, true};
	ParameterEnumT<kInputModeNum> inputMode{this, kInputModeTrigger};
	PACKED_STRUCT(PresetFieldData_t {
		uint8_t split;
		uint8_t retrigger ;
		uint8_t inputMode;
	}) presetFieldData;
private:
	rgb_t colors[2] = {
			{128, 128, 0},
			{128, 128, 100},
	};
	static constexpr size_t kTableSize = 1024;
	static constexpr size_t kNumSplits = 2;
	std::array<std::array<float,kTableSize>,kNumSplits> tables;
	Oscillator osc {1, Oscillator::sawtooth};
	bool hadTouch = false;
	bool inputModeShouldUpdateTable = false;
} gRecorderMode;

static void menu_enterRangeDisplay(const rgb_t& color, bool autoExit, ParameterContinuous& bottom, ParameterContinuous& top, const float& display);
static void menu_enterDisplayRangeRaw(const rgb_t& color, float bottom, float top);
static void menu_enterDisplayScaleMeterOutputMode(const rgb_t& color, bool bottomEnv, bool topEnv);
static void menu_up();

class ScaleMeterMode : public PerformanceMode {
public:
	bool setup(double ms) override
	{
		gSecondTouchIsSize = false; // TODO: _both_ may have to be positive
		count = 0;
		x1 = 0;
		y1 = 0;
		rms = 0;
		env = 0;
		updated(cutoff);
		if(ms <= 0)
		{
			ledSlidersSetupOneSlider(
				color,
				LedSlider::MANUAL_CENTROIDS
			);
			gOutMode = kOutModeManualSample;
		}
		if(ms < 0)
			return true;
		return modeChangeBlink(ms, color);
	}

	void render(BelaContext* context) override
	{
		// we can quickly get into menu mode from here
		if(!gAlt)
		{
			static std::array<float,26> data = {0};
			if(!performanceBtn.pressed && ledSliders.sliders[0].getNumTouches())
			{
				// only touch on: set output range
				menu_enterRangeDisplay(color, true, outRangeBottom, outRangeTop, outDisplay);
				// TODO: line below is just a workaround because we don't have a clean way of
				// _entering_ menu from here while ignoring the _last_ slider readings,
				// resulting in automatically re-entering immediately after exiting
				ledSliders.sliders[0].process(data.data());
				return;
			}
			if(performanceBtn.offset)
			{
				// press button: set input range
				menu_enterRangeDisplay(color, false, inRangeBottom, inRangeTop, inDisplay);
				// TODO: line below is just a workaround because we don't have a clean way of
				// _exiting_ the menu from here while ignoring the _first_ slider readings
				ledSliders.sliders[0].process(data.data());
				return;
			}
		}
		for(size_t n = 0; n < context->analogFrames; ++n)
		{
			float input = analogReadMapped(context, n, 0);
			// let's trust compiler + branch predictor to do a good job here
			float envIn;
			if(kCouplingAc == coupling)
			{
				//high pass
				// [b, a] = butter(1, 10/44250, 'high')
				const float b0 = 0.999630537400518;
				const float b1 = -0.999630537400518;
				const float a1 = -0.999261074801036;
				float x0 = input;
				float y = x1 * b1 + x0 * b0 - y1 * a1;
				x1 = x0;
				y1 = y;
				// times 2 to compensate for abs()
				envIn = abs(y) * 2.f;
			} else {
				envIn = input;
			}
			// peak envelope detector
			if(envIn > env)
			{
				env = envIn;
			} else {
				env = env * decay;
			}

			float outs[kNumOutChannels] = {0};
			switch (outputMode)
			{
			case kOutputModeNN: // top pass-through, bottom pass-through
				outs[0] = outs[1] = input;
				break;
			case kOutputModeNE: // top pass-through, bottom envelope
				outs[0] = input;
				outs[1] = env;
				break;
			case kOutputModeEE: // top envelope, bottom envelope
				outs[0] = outs[1] = env;
				break;
			}
			for(size_t c = 0; c < kNumOutChannels; ++c)
			{
				// TODO: combine this mapping with global mapping in tr_render(),
				// or at least reduce the number of times it gets called here if the compiler is not smart enough
				// TODO: should env also be mapped?
				float value = mapAndConstrain(outs[c], 0, 1, outRangeBottom, outRangeTop);
				analogWriteOnce(context, n, c, value);
			}
		}
		// displays if in In/OutRange mode
		outDisplay = mapAndConstrain(analogReadMapped(context, 0, 0), 0, 1, outRangeBottom, outRangeTop);
		inDisplay = analogReadMapped(context, 0, 0);
		// displays if in pure performance mode
		LedSlider::centroid_t centroids[2];
		// display actual output range
		centroids[0].location = outDisplay;
		centroids[0].size = kFixedCentroidSize;
		size_t numCentroids = 1;
		if(kOutputModeNN != outputMode)
		{
			centroids[1].location = mapAndConstrain(env, 0, 1, outRangeBottom, outRangeTop);
			centroids[1].size = kFixedCentroidSize;
			numCentroids++;
		}
		ledSliders.sliders[0].setLedsCentroids(centroids, numCentroids);
	}

	void updated(Parameter& p)
	{
		if(p.same(cutoff))
		{
			// wrangling the range to make it somehow useful
			// TODO: put more method into this
			float par = cutoff;
			par *= par;
			par *= par;
			par *= par;
			par = mapAndConstrain(par, 0, 1, 0.000005, 0.08);
			decay = 1.f - par;
		}
		else if(p.same(outRangeBottom) || p.same(outRangeTop)) {
		}
		else if(p.same(inRangeBottom) || p.same(inRangeTop)) {
		}
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD3(outputMode, coupling, cutoff);
	}
	enum {
		kCouplingDc,
		kCouplingAc,
		kCouplingNum,
	};
	enum {
		kOutputModeNN,
		kOutputModeNE,
		kOutputModeEE,
		kOutputModeNum
	};
	ScaleMeterMode() :
		presetFieldData{
			.outputMode = outputMode,
			.coupling = coupling,
			.cutoff = cutoff,
		}
	{
		PresetDesc_t presetDesc = {
			.field = this,
			.size = sizeof(PresetFieldData_t),
			.defaulter = genericDefaulter3(ScaleMeterMode, outputMode, coupling, cutoff),
			.loadCallback = genericLoadCallback3(ScaleMeterMode, outputMode, coupling, cutoff),
		};
		presetDescSet(2, &presetDesc);
	}
	ParameterEnumT<kOutputModeNum> outputMode {this, 0};
	ParameterEnumT<kCouplingNum> coupling {this, kCouplingDc};
	ParameterContinuous cutoff {this, 0.5};
	ParameterContinuous outRangeBottom {this, 0};
	ParameterContinuous outRangeTop {this, 1};
	ParameterContinuous inRangeBottom {this, 0};
	ParameterContinuous inRangeTop {this, 1};
	PACKED_STRUCT(PresetFieldData_t {
		int outputMode;
		int coupling;
		float cutoff;
	}) presetFieldData;
private:
	float inDisplay;
	float outDisplay;
	float decay;
	float analogReadMapped(BelaContext* context, size_t frame, size_t channel)
	{
		float in = analogRead(context, frame, channel);
		return mapAndConstrain(in, inRangeBottom, inRangeTop, 0, 1);
	}
	const rgb_t color = {0, 160, 160};
	float x1;
	float y1;
	float env;
	size_t count;
	float rms;
} gScaleMeterMode;

class BalancedOscsMode : public PerformanceMode {
public:
	bool setup(double ms) override
	{
		gSecondTouchIsSize = false;
		gBalancedLfoColors = gBalancedLfoColorsInit; //restore default in case it got changed via MIDI
		if(ms <= 0)
		{
			for(auto& o : oscillators)
				o.setup(1, Oscillator::Type(waveform.get())); // Fs set to 1, so we need to pass normalised frequency later

			ledSlidersSetupOneSlider(
				{0, 0, 0}, // dummy
				LedSlider::MANUAL_CENTROIDS
			);
		}
		gOutMode = kOutModeManualSample;
		if(ms < 0)
			return true;
		return modeChangeBlinkSplit(ms, gBalancedLfoColors.data(), kNumLeds / 2, kNumLeds / 2);
	}
	void render(BelaContext* context) override
	{
		float clockPeriod;
		float sampleRate = context->analogSampleRate;
		switch (inputMode)
		{
		default:
		case kInputModeTrig:
			// after frequency was set by parameter,
			// we use gClockPeriod only if it gets updated at least once
			if(gClockPeriodUpdated != lastClockPeriodUpdate)
				clockPeriod = gClockPeriod;
			else
				clockPeriod = sampleRate / (centreFrequency * 10.f + 0.1f); // TODO: more useful range? exp mapping?
			lastClockPeriodUpdate = gClockPeriodUpdated;
			break;
		case kInputModeCv:
		{
			// TODO: fix CV in: disable global rescale
			float volts = analogRead(context, 0, 0) * 15.f - 5.f;
			float freq = vToFreq(volts, 3);
			clockPeriod = sampleRate / freq;
		}
			break;
		}

		float midFreq = 1.f / clockPeriod; // we process once per block; the oscillator thinks Fs = 1
		float touchPosition = ledSliders.sliders[0].compoundTouchLocation();

		if (touchPosition > 0.0) {
			divisionPoint = touchPosition;
		}
		std::array<float,oscillators.size()> freqs = {
				(1.f - divisionPoint) * midFreq * 2.f,
				divisionPoint * midFreq * 2.f,
		};

		for(size_t n = 0; n < context->analogFrames; ++n) {
			for(size_t c = 0; c < oscillators.size() && c < context->analogOutChannels; ++c) {
				float out = oscillators[c].process(freqs[c]);
				analogWriteOnce(context, n, !c, map(out, -1, 1, 0, 1)); // The ! is so that the CV outs order matches the display. TODO: tidy up
				if(0 == n && !gAlt)
				{
					// limit max brightness. On the one hand, it reduces power consumption,
					// on the other hand it attempts to avoid the upper range, where it becomes hard
					// to discern increased brightness.
					const float kMaxBrightness = 0.3f;
					// TODO: move this to LedSlider so that it obeys to the ledEnabled there an we don't need the !gAlt here
					// if we are not in menu mode, set the display
					float brightness = map(out, -1, 1, 0, kMaxBrightness);
					unsigned int split = divisionPoint > 0 ? kNumLeds * divisionPoint : 0;
					unsigned int start = (0 == c) ? 0 : split;
					unsigned int stop = (0 == c) ? split : kNumLeds;
					rgb_t color = {uint8_t(brightness * gBalancedLfoColors[c].r), uint8_t(brightness * gBalancedLfoColors[c].g), uint8_t(brightness * gBalancedLfoColors[c].b)};
					for(unsigned int p = start; p <  stop; ++p)
						np.setPixelColor(p, color.r, color.g, color.b);
				}
			}
		}
	}
	void updated(Parameter& p)
	{
		if(p.same(waveform)) {
			for(auto& o : oscillators)
				o.setType(Oscillator::Type(waveform.get()));
		} else if (p.same(centreFrequency)) {
			lastClockPeriodUpdate = gClockPeriodUpdated;
			// force us to go into ModeTrig, or the ModeCv would make this change useless
			inputMode.set(kInputModeTrig);
		} else if (p.same(inputMode)) {

		}
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD3(waveform, centreFrequency, inputMode);
	}
	BalancedOscsMode() :
		presetFieldData {
			.centreFrequency = centreFrequency,
			.waveform = waveform,
			.inputMode = inputMode,
		}
	{
		PresetDesc_t presetDesc = {
			.field = this,
			.size = sizeof(PresetFieldData_t),
			.defaulter = genericDefaulter3(BalancedOscsMode, waveform, centreFrequency, inputMode),
			.loadCallback = genericLoadCallback3(BalancedOscsMode, waveform, centreFrequency, inputMode),
		};
		presetDescSet(3, &presetDesc);
	}
	typedef enum {
		kInputModeTrig,
		kInputModeCv,
		kNumInputModes,
	} InputMode;
	ParameterEnumT<Oscillator::numOscTypes> waveform {this, Oscillator::triangle};
	ParameterContinuous centreFrequency {this};
	ParameterEnumT<kNumInputModes> inputMode {this, kInputModeTrig};
	PACKED_STRUCT(PresetFieldData_t {
		float centreFrequency = centreFrequency;
		uint8_t waveform = waveform;
		uint8_t inputMode = inputMode;
	}) presetFieldData;
private:
	float divisionPoint = 0.5;
	uint32_t lastClockPeriodUpdate = gClockPeriodUpdated;
} gBalancedOscsMode;

class ExprButtonsMode : public PerformanceMode
{
public:
	bool setup(double ms)
	{
		gSecondTouchIsSize = true;
		if(ms <= 0)
		{
			ledSlidersSetupMultiSlider(
				ledSliders,
				{
				},
				LedSlider::MANUAL_CENTROIDS,
				true
			);
			gOutMode = kOutModeManualBlock;
			changeState(kDisabled, {0, 0});
		}
		// Force initialisation of offsets. Alternatively, always copy them in render()
		for(auto& o : offsetParameters)
			updated(o);
		if(ms < 0)
			return true;
		return modeChangeBlink(ms, {0, 200, 50});
	}
	void render(BelaContext*)
	{
		if(pitchBeingAdjusted >= 0)
		{
			// if we are adjusting the pitch, output that instead
			gManualAnOut[0] = offsets[pitchBeingAdjusted];
			gManualAnOut[1] = 1;
			pitchBeingAdjustedCount++;
			// hold it for a few blocks
			if(pitchBeingAdjustedCount >= kPitchBeingAdjustedCountMax)
			{
				pitchBeingAdjustedCount = 0;
				pitchBeingAdjusted = -1;
			}
			return;
		}
		// normal processing
		LedSlider::centroid_t centroid = {
				.location = ledSliders.sliders[0].touchLocation(0),
				.size = ledSliders.sliders[0].touchSize(0),
		};
		if(!centroid.size)
		{
			// touch is removed
			if(kDisabled != touch.state)
				changeState(kDisabled, centroid);
		} else {
			// if it is a new touch, assign this touch to a key, store location as impact location,
			// mark it as unmoved
			if(kDisabled == touch.state)
				changeState(kInitial, centroid); // note that this may fail and we go back to kDisabled
		}
		// if it is not a new touch and it has moved enough, mark it as moved
		if(kInitial == touch.state)
		{
			if(fabsf(centroid.location - touch.initialLocation) > kMoveThreshold)
				changeState(kMoved, centroid);
		}
		// if it is a moved touch, apply high-pass to get modulation amount
		if(kMoved == touch.state)
		{
			// if a moved touch gets 'far enough' from initial position
			// and getting 'close' to another button, we are bending towards it.
			if(shouldBend(centroid))
				changeState(kBending, centroid);
		}
		// if we are bending, there is a "dead" spot close to the center of the target key
		if(kBending == touch.state)
		{
			// ensure we leave the dead spot that we may (probably) be in when the bending
			// starts
			// TODO: add hysteresis?
			if(fabsf(centroid.location - getMidLocationFromKey(touch.key)) > kBendDeadSpot)
				touch.bendHasLeftStartKeyDeadSpot = true;
			size_t key;
			// check each key: are we in its dead spot?
			for(key = 0; key < kNumButtons; ++key)
			{
				float range = kBendDeadSpot;
				float midPoint = getMidLocationFromKey(key);
				if(fabsf(centroid.location - midPoint) < range)
				{
					if(key == touch.key && !touch.bendHasLeftStartKeyDeadSpot) {
						continue;
					}
					break;
				}
			}
			if(key == kNumButtons) {
				// we are not in a dead spot
				touch.bendDeadTime = 0;
				if(fabsf(centroid.location - getMidLocationFromKey(touch.key)) < step - kBendDeadSpot)
				{
					// we are between the initial key and a neighbour's dead spot
					// nothing to do
				} else {
					// we went past the dead zone towards the next key.
					// Start a new bending towards that.
					changeState(kInitial, centroid);
					changeState(kBending, centroid);
				}
			} else {
				// we are in the dead spot of a key
				// make sure it's the same key as the previous frames
				if(touch.bendDeadKey == key)
					touch.bendDeadTime++;
				else
					touch.bendDeadTime = 1;
				touch.bendDeadKey = key;
				if(touch.bendDeadTime >= kBendDeadSpotMaxCount)
				{
					// if we are here long enough, start a new touch here
					changeState(kInitial, centroid);
					changeState(kMoved, centroid);
				}
			}
		}
		// mode-specific processing
		switch(touch.state)
		{
		case kInitial:
			break;
		case kMoved:
		{
			// printf("%.5f\n\r", touch.mod);
			// apply high-pass
			float x0 = centroid.location - touch.initialLocation;
			float y0 = x0 * b0 + touch.filt.x1 * b1 - touch.filt.y1 * a1;
			touch.mod = y0;
			touch.filt.x1 = x0;
			touch.filt.y1 = y0;
		}
			break;
		case kBending:
			break;
		case kDisabled:
		case kNumStates:
			break;
		}

		// now set output
		switch(touch.state)
		{
		case kInitial:
			out = offsets[touch.key];
			break;
		case kMoved:
			out = offsets[touch.key] + touch.mod * modRange * 2.f;
			break;
		case kBending:
		{
			float bendIdx;
			float bendRange;

			float diff = centroid.location - touch.initialLocation;
			float bendSign = (diff > 0) ? 1 : -1;
			size_t bendDestKey = touch.key + bendSign * 1;
			if(bendDestKey >= kNumButtons) {
				// if at edge of keyboard, no bending
				bendIdx = 0;
				bendRange = 0;
			} else {
				float destLoc = getMidLocationFromKey(bendDestKey) - bendSign * kBendDeadSpot;
				if(bendSign * centroid.location < destLoc * bendSign)
					// outside the dead spot
					bendIdx = (centroid.location - touch.initialLocation) / (destLoc - touch.initialLocation);
				else // in the dead spot
					bendIdx = 1;
				float destOut = offsets[bendDestKey];
				bendRange = destOut - touch.initialOut;
			}
#if 0
			static int n = 0;
			if(0 == (n % 10))
			{
				printf("%+1.0f %.2f %.2f %.2f|", bendSign, centroid.location, touch.initialLocation, destLoc);
				printf("%.2f\n\r", bendIdx);
			}
			++n;
#endif
			out = touch.initialOut + bendIdx * bendRange;
		}
			break;
		case kDisabled:
		case kNumStates:
			break;
		}
		gManualAnOut[0] = out;
		gManualAnOut[1] = centroid.size;
		// display
		for(size_t n = 0; n < kNumButtons; ++n)
		{
			if(!gAlt)
			{
				//TODO: have setPixelColor obey "enabled"
				size_t pixel = size_t(getMidLocationFromKey(n) * kNumLeds + 0.5f);
				float coeff = (n == touch.key) ? 1 : 0.1;
				np.setPixelColor(pixel, colors[n].r * coeff, colors[n].g * coeff, colors[n].b * coeff);
			}
		}
	}
private:
	typedef enum {
		kInitial,
		kMoved,
		kBending,
		kDisabled,
		kNumStates,
	} TouchState;
	const std::array<const char*,kNumStates> touchStateNames {
			"kInitial",
			"kMoved",
			"kBending",
			"kDisabled",
	};
//	template <typename T = size_t>
	void changeState(TouchState newState, const LedSlider::centroid_t& centroid, size_t arg0 = 0)
	{
		S(printf("%s, {%.2f} %.2f_", touchStateNames[newState], centroid.location, out));
		switch(newState)
		{
		case kInitial:
		{
			touch = Touch();
			ssize_t key = getKeyFromLocation(centroid.location);
			if(key >= 0)
				touch.key = key;
			else
				changeState(kDisabled, centroid);
		}
			break;
		case kMoved:
			touch.filt = {0};
			break;
		case kBending:
			touch.bendStartLocation = centroid.location;
			touch.bendDeadTime = 0;
			touch.bendHasLeftStartKeyDeadSpot = false;
			break;
		case kDisabled:
			touch.key = -1;
			break;
		case kNumStates:
			break;
		}
		S(printf("\n\r"));
		touch.state = newState;
		touch.initialLocation = centroid.location;
		touch.initialOut = out;
	}
	bool shouldBend(const LedSlider::centroid_t& centroid)
	{
		float diff = centroid.location - touch.initialLocation;
		// check that we are far enough from the initial position
		if(fabsf(diff)> kBendStartThreshold)
		{
			size_t key = touch.key;
			// and that we are not bending towards the outer edges of the keyboard
			if(
				(diff < 0 && key > 0)
				|| (diff > 0 && key < (kNumButtons - 1))
				)
				return true;
		}
		return false;
	}
	static float getMidLocationFromKey(size_t key)
	{
		return key * step + step * 0.5f;
	}

	static ssize_t getKeyFromLocation(float location)
	{
		size_t key;
		// identify candidate key
		for(key = 0; key < kNumButtons; ++key)
		{
			float top = (key + 1) * step;
			if(location <= top)
				break;
		}
		if(kNumButtons == key)
			return -1; // weird ...
		// validate that we are not _too far_ from the center
		float centre = getMidLocationFromKey(key);
		if(fabsf(location - centre) > kMaxDistanceFromCenter)
			return -1;
		return key;
	}
	static constexpr size_t kNumButtons = 5;
	static constexpr float step = 1.f / kNumButtons;
	static constexpr float kMaxDistanceFromCenter = step * 0.85f;
	static constexpr float kMoveThreshold = step * 0.1f;
	static constexpr float kBendStartThreshold = step * 0.4f; // could be same as kMaxDistanceFromCenter?
	static constexpr float kBendDeadSpot = step * 0.2f;
	static constexpr size_t kBendDeadSpotMaxCount = 40;
	static constexpr float b0 = float(0.9922070637080485);
	static constexpr float b1 = float(-0.9922070637080485);
	static constexpr float a1 = float(-0.9844141274160969);
	struct Touch {
		TouchState state = kDisabled;
		size_t key = 0;
		float initialLocation = 0;
		float initialOut = 0;
		float bendStartLocation = 0;
		float mod = 0;
		struct {
			float y1 = 0;
			float x1 = 0;
		} filt;
		size_t bendDeadTime = 0;
		size_t bendDeadKey = -1;
		bool bendHasLeftStartKeyDeadSpot = false;
	} touch;
	std::array<LedSlider::centroid_t,kNumButtons> buttons;
	std::array<rgb_t,kNumButtons> colors = {{
		{0, 255, 0},
		{0, 200, 50},
		{0, 150, 100},
		{0, 100, 150},
		{0, 50, 200},
	}};
public:
	void updated(Parameter& p)
	{
		if(p.same(modRange)) {

		} else if(p.same(quantised)) {

		} else {
			for(size_t n = 0; n < kNumButtons; ++n)
			{
				if(p.same(offsetParameters[n]))
				{
					offsets[n] = offsetParameters[n];
					pitchBeingAdjusted = n;
					pitchBeingAdjustedCount = 0;
					break;
				}
			}
		}
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD2PlusArray(quantised, modRange, offsetParameters);
	}
	ExprButtonsMode():
		presetFieldData {
			.modRange = modRange,
			.offsetParameters = {
				offsetParameters[0],
				offsetParameters[1],
				offsetParameters[2],
				offsetParameters[3],
				offsetParameters[4],
			},
			.quantised = quantised,
		}
	{
		PresetDesc_t presetDesc = {
			.field = this,
			.size = sizeof(PresetFieldData_t),
			.defaulter = genericDefaulter2PlusArray(ExprButtonsMode, quantised, modRange, offsetParameters),
			.loadCallback = genericLoadCallback2PlusArray(ExprButtonsMode, quantised, modRange, offsetParameters),
		};
		presetDescSet(4, &presetDesc);
	}
	ParameterEnumT<2> quantised {this, true};
	ParameterContinuous modRange {this, 0.5};
	std::array<ParameterContinuous,kNumButtons> offsetParameters {
		ParameterContinuous(this, 0.5),
		ParameterContinuous(this, 0.6),
		ParameterContinuous(this, 0.7),
		ParameterContinuous(this, 0.8),
		ParameterContinuous(this, 0.9),
	};
	PACKED_STRUCT(PresetFieldData_t {
		float modRange;
		std::array<float,kNumButtons> offsetParameters;
		uint8_t quantised;
	}) presetFieldData;
private:
	float out = 0;
	std::array<float,kNumButtons> offsets;
	int pitchBeingAdjusted = -1;
	unsigned int pitchBeingAdjustedCount = 0;
	static constexpr unsigned int kPitchBeingAdjustedCountMax = 10;
} gExprButtonsMode;

uint8_t gNewMode = 0; // if there is a preset to load (i.e.: always except on first boot), this will be overridden then.

static std::array<PerformanceMode*,kNumModes> performanceModes = {
#ifdef TEST_MODE
	&gTestMode,
#endif // TEST_MODE
	&gDirectControlMode,
	&gRecorderMode,
	&gScaleMeterMode,
	&gBalancedOscsMode,
	&gExprButtonsMode,
};

bool performanceMode_setup(double ms)
{
	return performanceModes[gNewMode]->setup(ms);
}

void performanceMode_render(BelaContext* context)
{
	performanceModes[gNewMode]->render(context);
}

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
float minValue = 0.333447; // some resonable default, for my board at least
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
	gOutMode = kOutModeManualBlock;
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

float getGnd(){
	return gCalibrationProcedure.getGnd();
}

class ButtonAnimation {
public:
	virtual void process(uint32_t ms, LedSlider& ledSlider, float value) = 0;
};

class ButtonAnimationSplit : public ButtonAnimation {
public:
	ButtonAnimationSplit(rgb_t color0, rgb_t color1) :
		color0(color0), color1(color1) {}
	void process(uint32_t ms, LedSlider& ledSlider, float value) override {
		rgb_t color;
		if(0 == value)
			color = color0;
		else {
			float coeff0 = simpleTriangle(ms, 1000);
			float coeff1 = 1.f - coeff0;
			color.r = coeff0 * color0.r + coeff1 * color1.r;
			color.g = coeff0 * color0.g + coeff1 * color1.g;
			color.b = coeff0 * color0.b + coeff1 * color1.b;
		}
		ledSlider.setColor(color);
	};
protected:
	rgb_t color0;
	rgb_t color1;
};

class ButtonAnimationPulsatingStill : public ButtonAnimation {
public:
	ButtonAnimationPulsatingStill(rgb_t color) :
		color(color) {}
	void process(uint32_t ms, LedSlider& ledSlider, float value) override {
		rgb_t c;
		if(0 == value)
		{
			// PWM with increasingly longer width
			const unsigned int period = 600;
			const unsigned int numPeriods = 5;
			ms = ms % (period * numPeriods);
			unsigned int thisPeriod = ms / period;
			unsigned int pulseWidth = thisPeriod / float(numPeriods - 1) * period;
			unsigned int idx = ms % period;
			float coeff = idx < pulseWidth ? 0.3 : 1;
			c.r = color.r * coeff;
			c.g = color.g * coeff;
			c.b = color.b * coeff;
		} else {
			c = color;
		}
		ledSlider.setColor(c);
	};
protected:
	rgb_t color;
	unsigned int width = -1;
};

class ButtonAnimationStillTriangle : public ButtonAnimation {
public:
	ButtonAnimationStillTriangle(rgb_t color) :
		color(color) {}
	void process(uint32_t ms, LedSlider& ledSlider, float value) override {
		rgb_t c;
		if(0 == value)
		{
			c = color;
		} else {
			const unsigned int period = 600;
			float coeff = simpleTriangle(ms, period);
			c.r = color.r * coeff;
			c.g = color.g * coeff;
			c.b = color.b * coeff;
		}
		ledSlider.setColor(c);
	};
protected:
	rgb_t color;
};

class ButtonAnimationTriangle : public ButtonAnimation {
public:
	ButtonAnimationTriangle(rgb_t color, uint32_t period) :
		color(color), period(period) {}
	void process(uint32_t ms, LedSlider& ledSlider, float) override {
		rgb_t c;
		float coeff = simpleTriangle(ms, period);
		c.r = color.r * coeff;
		c.g = color.g * coeff;
		c.b = color.b * coeff;
		ledSlider.setColor(c);
	};
protected:
	rgb_t color;
	uint32_t period;
};

class ButtonAnimationBrightDimmed: public ButtonAnimation {
public:
	ButtonAnimationBrightDimmed(rgb_t color) :
		color(color) {}
	void process(uint32_t ms, LedSlider& ledSlider, float rampUp) override {
		rgb_t c;
		float coeff = rampUp > 0.5 ? 1 : 0.3;
		c.r = color.r * coeff;
		c.g = color.g * coeff;
		c.b = color.b * coeff;
		ledSlider.setColor(c);
	};
protected:
	rgb_t color;
};

class ButtonAnimationSingleRepeatedEnv: public ButtonAnimation {
public:
	ButtonAnimationSingleRepeatedEnv(rgb_t color) :
		color(color) {}
	void process(uint32_t ms, LedSlider& ledSlider, float value) override {
		rgb_t c;
		const unsigned int duration = 600;
		unsigned int period;
		if(0 == value)
		{
			// largely spaced animation
			period = duration * 2.5;
		} else {
			// tightly looped animations
			period = duration;
		}
		ms %= period;
		float coeff;
		if(ms <= duration)
			coeff = mapAndConstrain((duration - ms) / float(duration), 0, 1, 0.3, 1);
		else
			coeff = 0;
		c.r = color.r * coeff;
		c.g = color.g * coeff;
		c.b = color.b * coeff;
		ledSlider.setColor(c);
	};
protected:
	rgb_t color;
};

class ButtonAnimationRecorderInputMode: public ButtonAnimation {
public:
	ButtonAnimationRecorderInputMode(rgb_t color) :
		color(color) {}
	void process(uint32_t ms, LedSlider& ledSlider, float value) override {
		float coeff;
		if(0 == value)
		{
			const unsigned int periodicDuration = 800;
			// input mode: trigger. Show evenly spaced brief pulses
			ms %= periodicDuration;
			coeff = (ms / float(periodicDuration)) < 0.1;
		} else if (1 == value){
			// input mode: CV in
			// show a few smooth transitions
			const unsigned int duration = 3000;
			ms %= duration;
			std::array<float,12> data = {
					0.1, 0.3, 0.5, 0.3, 0.4, 0.7, 1.0, 0.8, 0.6, 0.3, 0.3, 0.3
			};
			coeff = interpolatedRead(data, ms / float(duration));
		} else {
			// input mode: phasor
			// show a phasor
			const unsigned int duration = 1000;
			ms %= duration;
			coeff = ms / float(duration);
		}
		rgb_t c;
		c.r = color.r * coeff;
		c.g = color.g * coeff;
		c.b = color.b * coeff;
		ledSlider.setColor(c);
	};
protected:
	rgb_t color;
};

class ButtonAnimationWaveform: public ButtonAnimation {
public:
	ButtonAnimationWaveform(rgb_t color) :
		color(color) {}
	void process(uint32_t ms, LedSlider& ledSlider, float value) override {
		rgb_t c;
		const unsigned int period = 1200;
		ms %= period;
		unsigned int type = int(value + 0.5f);
		if(type >= Oscillator::numOscTypes)
			type = 0;
		osc.setType(Oscillator::Type(type));
		float phase = float(ms) / float(period) * 2.f * float(M_PI);
		if(phase > float(M_PI))
			phase -= 2.f * float(M_PI);
		osc.setPhase(phase);
		float coeff = osc.process();
		coeff = mapAndConstrain(coeff, -1, 1, 0, 1);
		c.r = color.r * coeff;
		c.g = color.g * coeff;
		c.b = color.b * coeff;
		ledSlider.setColor(c);
	};
protected:
	rgb_t color;
	Oscillator osc {1};
};

class ButtonAnimationSpeedUpDown: public ButtonAnimation {
public:
	ButtonAnimationSpeedUpDown(rgb_t color) :
		color(color) {}
	void process(uint32_t ms, LedSlider& ledSlider, float value) override {
		rgb_t c;
		// over duration, show pulses with ramp up-ramp down period(constant width),
		// to give the idea of frequency control
		phase += (ms - lastMs); // this has to be uint to be deterministic on overflow.
		lastMs = ms;
		ms %= duration;
		float triangle = simpleTriangle(ms, duration);
		float period = initialPeriod + (sqrt(triangle)) * (finalPeriod - initialPeriod);
		if(phase > period)
			phase -= period;
		float coeff = (phase < onTime);
		c.r = color.r * coeff;
		c.g = color.g * coeff;
		c.b = color.b * coeff;
		ledSlider.setColor(c);
	};
protected:
	static constexpr float initialPeriod = 600;
	static constexpr float finalPeriod = 80;
	static constexpr unsigned int duration = 3000;
	static constexpr float onTime = 40;
	float phase = 0;
	uint32_t lastMs;
	rgb_t color;
};

class MenuItemType
{
public:
	MenuItemType(rgb_t baseColor) : baseColor(baseColor) {}
	virtual void process(LedSlider& slider) = 0;
	rgb_t baseColor;
};

class MenuItemTypeEvent : public MenuItemType
{
public:
	MenuItemTypeEvent(const char* name, rgb_t baseColor, uint32_t holdTime = 0) :
		MenuItemType(baseColor), name(name), holdTime(holdTime) {}
	void process(LedSlider& slider) override
	{
		bool state = slider.getNumTouches();
		if(state != pastState)
		{
			bool rising = (state && !pastState);
			if(holdTime)
				lastTransition = HAL_GetTick();
			event(rising ? kTransitionRising : kTransitionFalling);
			holdNotified = false;
		}
		if(state != pastState)
		{
			LedSlider::centroid_t centroid;
			centroid.location = 0.5;
			// dimmed for "inactive"
			// full brightness for "active"
			centroid.size = state ? 0.5 : 0.1;
			slider.setLedsCentroids(&centroid, 1);
		}
		pastState = state;

		if(holdTime && !holdNotified && HAL_GetTick() - lastTransition > holdTime)
		{
			holdNotified = true;
			event(pastState ? kHoldHigh : kHoldLow);
		}
	}
protected:
	char const* name;
	typedef enum {kTransitionFalling, kTransitionRising, kHoldLow, kHoldHigh} Event;
	virtual void event(Event) = 0;
	uint32_t lastTransition;
	uint32_t holdTime;
	bool pastState = false;
	bool holdNotified = true; // avoid notifying on startup
};

class MenuItemTypeDiscrete : public MenuItemTypeEvent
{
public:
	MenuItemTypeDiscrete(const char* name, rgb_t baseColor, ParameterEnum* parameter, ButtonAnimation* animation = nullptr) :
		MenuItemTypeEvent(name, baseColor, 0), parameter(parameter), animation(animation) {}
protected:
	virtual void process(LedSlider& slider) override
	{
		MenuItemTypeEvent::process(slider);
		if(animation)
			animation->process(HAL_GetTick(), slider, parameter->get());
	}

	virtual void event(Event e)
	{
		if(kTransitionRising == e)
		{
			if(parameter)
			{
				parameter->next();
				printf("%s: set to %d\n\r", name, parameter->get());
			}
		}
	}
	ParameterEnum* parameter;
	ButtonAnimation* animation;
};

AutoLatcher gMenuAutoLatcher;
class MenuItemTypeSlider : public MenuItemType {
public:
	MenuItemTypeSlider(): MenuItemType({0, 0, 0}) {}
	MenuItemTypeSlider(const rgb_t& color, ParameterContinuous* parameter) :
		MenuItemType(color), parameter(parameter) {
		gMenuAutoLatcher.reset();
	}
	void process(LedSlider& slider) override
	{
		if(parameter)
		{
			TouchFrame frame {
				.pos = slider.compoundTouchLocation(),
				.sz = slider.compoundTouchSize(),
			};
			bool latched = false;
			gMenuAutoLatcher.process(frame, latched);
			parameter->set(frame.pos);
			if(latched)
				menu_up();
		}
	}
	ParameterContinuous* parameter;
};

class MenuItemTypeQuantised : public MenuItemType {
public:
	MenuItemTypeQuantised(): MenuItemType({0, 0, 0}) {}
	MenuItemTypeQuantised(const rgb_t& color, ParameterEnum* parameter) :
		MenuItemType(color), parameter(parameter) {
		gMenuAutoLatcher.reset();
	}
	void process(LedSlider& slider) override
	{
		if(parameter)
		{
			unsigned int quantised = parameter->getMax();
			TouchFrame frame {
				.pos = slider.compoundTouchLocation(),
				.sz = slider.compoundTouchSize(),
			};
			bool latched = false;
			gMenuAutoLatcher.process(frame, latched);
			float pos = fix(frame.pos);
			unsigned int n = (unsigned int)(pos * quantised);
			pos = n / float(quantised);
			if(latched)
			{
				parameter->set(n);
				menu_up();
			}
			LedSlider::centroid_t centroid = {
					.location = fix(int(pos * quantised) / float(quantised - 1)),
					.size = kFixedCentroidSize,
			};
			slider.setLedsCentroids(&centroid, 1);
		}
	}
	float fix(float pos)
	{
		if(orientationAgnostic)
			return gJacksOnTop ? 1.f - pos : pos;
		return pos;
	}
	ParameterEnum* parameter;
	bool orientationAgnostic = true;
};

class MenuItemTypeRange : public MenuItemType {
public:
	MenuItemTypeRange(): MenuItemType({0, 0, 0}) {}
	MenuItemTypeRange(const rgb_t& color, bool autoExit, ParameterContinuous* paramBottom, ParameterContinuous* paramTop) :
		MenuItemType(color), parameters({paramBottom, paramTop}), autoExit(autoExit)
	{
		latchProcessor.reset();
		for(size_t n = 0; n < kNumEnds; ++n)
		{
			pastFrames[n].pos = parameters[n]->get();
			pastFrames[n].sz = 1;
		}
		std::array<bool,2> isLatched;
		// "prime" the latchProcessor. Needed because we'll always start with one touch
		latchProcessor.process(true, pastFrames.size(), pastFrames, isLatched);
		hasHadTouch = false;
	}
	void process(LedSlider& slider) override
	{
		if(parameters[0] && parameters[1])
		{
			size_t numTouches = slider.getNumTouches();
			if(0 == numTouches && hasHadTouch && autoExit)
			{
				// both touches released: exit
				menu_up();
				return;
			} else if(numTouches) {
				hasHadTouch = true;
			}
			if(hasHadTouch)
			{
				bool validTouch = 0;
				std::array<TouchFrame,kNumEnds> frames;
				if(0 == numTouches) {
					frames = pastFrames;
				} else if(1 == numTouches)
				{
					// find which existing values this is closest to
					float current = slider.touchLocation(0);
					std::array<float,kNumEnds> diffs;
					for(size_t n = 0; n < kNumEnds; ++n)
						diffs[n] = std::abs(current - pastFrames[n].pos);
					// the only touch we have is controlling the one that was closest to it
					validTouch = (diffs[0] > diffs[1]);
					// put the good one where it belongs
					frames[validTouch].pos = slider.touchLocation(0);
					frames[validTouch].sz = slider.touchSize(0);
					// and a non-touch on the other one
					frames[!validTouch].pos = 0;
					frames[!validTouch].sz = 0;
				} else if (2 == numTouches)
				{
					for(size_t n = 0; n < kNumEnds; ++n)
					{
						frames[n].pos = slider.touchLocation(n);
						frames[n].sz = slider.touchSize(n);
					}
				}
				std::array<bool,2> isLatched;
				latchProcessor.process(true, frames.size(), frames, isLatched);
				for(size_t n = 0; n < frames.size(); ++n)
				{
					parameters[n]->set(frames[n].pos);
					pastFrames[n] = frames[n];
				}
			}
			updateDisplay(slider);
		}
	}
protected:
	static constexpr size_t kNumEnds = 2;
	std::array<TouchFrame,kNumEnds> pastFrames;
private:
	virtual void updateDisplay(LedSlider& slider)
	{
		std::array<LedSlider::centroid_t,2> values = {
				LedSlider::centroid_t{ pastFrames[0].pos, 0.15 },
				LedSlider::centroid_t{ pastFrames[1].pos, 0.15 },
		};
		slider.setLedsCentroids(values.data(), values.size());
	}
	std::array<ParameterContinuous*,kNumEnds> parameters;
	static LatchProcessor latchProcessor;
	bool autoExit;
	bool hasHadTouch;
};
LatchProcessor MenuItemTypeRange::latchProcessor;

class MenuItemTypeRangeDisplayCentroids : public MenuItemTypeRange {
public:
	MenuItemTypeRangeDisplayCentroids(){}
	MenuItemTypeRangeDisplayCentroids(const rgb_t& color, bool autoExit, ParameterContinuous* paramBottom, ParameterContinuous* paramTop, const float& display) :
		MenuItemTypeRange(color, autoExit, paramBottom, paramTop), display(&display) {}
	void updateDisplay(LedSlider& slider) override
	{
		std::array<LedSlider::centroid_t,3> values = {
				LedSlider::centroid_t{ pastFrames[0].pos, 0.05 },
				LedSlider::centroid_t{ pastFrames[1].pos, 0.05 },
				LedSlider::centroid_t{ *display, 0.15 },
		};
		slider.setLedsCentroids(values.data(), values.size());
	}
private:
	const float* display;
};

class MenuItemTypeDisplayRangeRaw : public MenuItemType {
public:
	MenuItemTypeDisplayRangeRaw(): MenuItemType({0, 0, 0}) {}
	MenuItemTypeDisplayRangeRaw(const rgb_t& color, float bottom, float top) :
		MenuItemType(color), bottom(bottom), top(top) {}
	void process(LedSlider& slider) override
	{
		std::array<float,kNumLeds> values;
		for(size_t n = 0; n < values.size(); ++n)
		{
			float idx = n / float(values.size());
			bool active = (idx >= bottom && idx < top);
			values[n] = active;
		}
		slider.setLedsRaw(values.data());
		if(HAL_GetTick() - startMs >= kMaxMs)
			menu_up();
	}
private:
	float bottom;
	float top;
	static constexpr uint32_t kMaxMs = 300;
	uint32_t startMs = HAL_GetTick();
};

class MenuItemTypeDisplayScaleMeterOutputMode : public MenuItemType {
public:
	MenuItemTypeDisplayScaleMeterOutputMode(): MenuItemType({0, 0, 0}) {}
	MenuItemTypeDisplayScaleMeterOutputMode(const rgb_t& color, bool bottomEnv, bool topEnv) :
		MenuItemType(color), isEnv({bottomEnv, topEnv}) {}
	void process(LedSlider& slider) override
	{
		std::array<LedSlider::centroid_t,kNumSplits> centroids;
		uint32_t ms = HAL_GetTick() - startMs;
		for(size_t n = 0; n < isEnv.size(); ++n)
		{
			float loc;
			if(isEnv[n])
			{
				loc = simpleTriangle(ms, kMaxMs);
			} else {
				// slightly smoothed pulse:
				const uint32_t first = 150;
				const uint32_t second = 400;
				const uint32_t third = 600;
				if(ms < first)
					loc = 0;
				else if (ms < second)
					loc = 1; // high
				else if (ms < third)
				{
					// ramp down
					loc = 1 - (ms - second) / float(third - second);
				} else // low
					loc = 0;
			}
			centroids[n].location = map(loc, 0, 1, n * 0.5, n * 0.5 + 0.4);
			centroids[n].size = kFixedCentroidSize;
		}
		slider.setLedsCentroids(centroids.data(), centroids.size());

		if(HAL_GetTick() - startMs >= kMaxMs)
			menu_up();
	}
private:
	static constexpr uint32_t kMaxMs = 1000;
	uint32_t startMs = HAL_GetTick();
	static constexpr size_t kNumSplits = 2;
	std::array<bool,kNumSplits> isEnv;
};

static void requestNewMode(int mode);

static void requestIncMode(int inc)
{
	while(inc < 0)
		inc += kNumModes;
	requestNewMode((gNewMode + inc) % kNumModes);
}

class MenuItemTypeNextMode : public MenuItemTypeEvent
{
public:
	MenuItemTypeNextMode(const char* name, rgb_t baseColor) :
		MenuItemTypeEvent(name, baseColor, 0) {}
private:
	void event(Event e) override
	{
		if(kTransitionRising == e)
			requestIncMode(1);
	}
};

class MenuPage { //todo: make non-copyable
public:
	const char* name;
	std::vector<MenuItemType*> items;
	MenuPage(MenuPage&&) = delete;
	enum Type {
		kMenuTypeButtons,
		kMenuTypeSlider,
		kMenuTypeQuantised,
		kMenuTypeRange,
		kMenuTypeRaw,
	};
	MenuPage(const char* name, const std::vector<MenuItemType*>& items = {}, Type type = kMenuTypeButtons): name(name), items(items), type(type) {}
	Type type;
};

int menu_setup(double);
static void menu_in(MenuPage& menu);
static MenuPage* activeMenu;

class MenuItemTypeEnterSubmenu : public MenuItemTypeEvent
{
public:
	MenuItemTypeEnterSubmenu(const char* name, rgb_t baseColor, uint32_t holdTime, MenuPage& submenu) :
		MenuItemTypeEvent(name, baseColor, holdTime), submenu(submenu) {}
private:
	void event(Event e)
	{
		if(kHoldHigh == e) {
			menu_in(submenu);
		}
	}
	MenuPage& submenu;
};

MenuPage mainMenu("main");
MenuPage globalSettingsMenu("global settings");

static MenuItemTypeSlider singleSliderMenuItem;
// this is a submenu consisting of a continuous slider(no buttons). Before entering it,
// appropriately set the properties of singleSliderMenuItem
MenuPage singleSliderMenu("single slider", {&singleSliderMenuItem}, MenuPage::kMenuTypeSlider);

static MenuItemTypeRange singleRangeMenuItem;
// this is a submenu consisting of a range slider. Before entering it,
// appropriately set the properties of singleRangeMenuItem
MenuPage singleRangeMenu("single range", {&singleRangeMenuItem}, MenuPage::kMenuTypeRange);

static MenuItemTypeRangeDisplayCentroids singleRangeDisplayMenuItem;
// this is a submenu consisting of a range+display (no buttons). Before entering it,
// appropriately set the properties of singleRangeDisplayMenuItem
MenuPage singleRangeDisplayMenu("single range", {&singleRangeDisplayMenuItem}, MenuPage::kMenuTypeRange);

static MenuItemTypeQuantised singleQuantisedMenuItem;
// this is a submenu consisting of a quantised slider(no buttons). Before entering it,
// appropriately set the properties of singleQuantisedMenuItem
MenuPage singleQuantisedMenu("single quantised", {&singleQuantisedMenuItem}, MenuPage::kMenuTypeQuantised);

// If held-press, get into singleSliderMenu to set value
class MenuItemTypeEnterContinuous : public MenuItemTypeEnterSubmenu
{
public:
	MenuItemTypeEnterContinuous(const char* name, rgb_t baseColor, ParameterContinuous& value, ButtonAnimation* animation = nullptr) :
		MenuItemTypeEnterSubmenu(name, baseColor, 500, singleSliderMenu), value(value), animation(animation) {}
	void process(LedSlider& slider)
	{
		MenuItemTypeEnterSubmenu::process(slider);
		if(animation)
		{
			animation->process(HAL_GetTick(), slider, value);
		}
	}
	void event(Event e)
	{
		if(kHoldHigh == e) {
			singleSliderMenuItem = MenuItemTypeSlider(baseColor, &value);
			menu_in(singleSliderMenu);
		}
	}
	ParameterContinuous& value;
	ButtonAnimation* animation;
};

// If held-press, get into singleQuantisedMenu to set value as if it was a big toggle
class MenuItemTypeEnterQuantised : public MenuItemTypeEnterSubmenu
{
public:
	MenuItemTypeEnterQuantised(const char* name, rgb_t baseColor, ParameterEnum& value, ButtonAnimation* animation = nullptr) :
		MenuItemTypeEnterSubmenu(name, baseColor, 500, singleQuantisedMenu), value(value), animation(animation) {}
	void process(LedSlider& slider)
	{
		MenuItemTypeEnterSubmenu::process(slider);
		if(animation)
		{
			animation->process(HAL_GetTick(), slider, value.get());
		}
	}
	void event(Event e) override
	{
		if(kHoldHigh == e) {
			singleQuantisedMenuItem = MenuItemTypeQuantised(baseColor, &value);
			menu_in(singleQuantisedMenu);
		}
	}
	ParameterEnum& value;
	ButtonAnimation* animation;
};

// If held-press, get into singleRangeMenu to set values
class MenuItemTypeEnterRange : public MenuItemTypeEnterSubmenu
{
public:
	MenuItemTypeEnterRange(const char* name, rgb_t baseColor, ParameterContinuous& bottom, ParameterContinuous& top) :
		MenuItemTypeEnterSubmenu(name, baseColor, 500, singleSliderMenu), bottom(bottom), top(top) {}
	void event(Event e)
	{
		if(kHoldHigh == e) {
			singleRangeMenuItem = MenuItemTypeRange(baseColor, true, &bottom, &top);
			menu_in(singleRangeMenu);
		}
	}
	ParameterContinuous& bottom;
	ParameterContinuous& top;
};

class MenuItemTypeDiscretePlus : public MenuItemTypeEvent
{
public:
	MenuItemTypeDiscretePlus(const char* name, rgb_t baseColor, ParameterEnum& valueEn):
		MenuItemTypeEvent(name, baseColor, 1000), valueEn(valueEn) {}
	void event(Event e) override
	{
		switch (e)
		{
		case kTransitionFalling:
			if(!ignoreNextTransition)
			{
				// this one is on release so we avoid a spurious trigger when holding
				valueEn.next();
				printf("DiscretePlus: next to %d\n\r", valueEn.get());
			}
			ignoreNextTransition = false;
			break;
		case kHoldHigh:
			// avoid transition so that when exiting Plus mode
			// we don't mistakenly increment the enum
			ignoreNextTransition = true;
			enterPlus();
			break;
		default:
			break;
		}
	}
	virtual void enterPlus() = 0;
	ParameterEnum& valueEn;
	bool ignoreNextTransition = false;
};

class MenuItemTypeDiscreteContinuous : public MenuItemTypeDiscretePlus
{
public:
	MenuItemTypeDiscreteContinuous(const char* name, rgb_t baseColor, ParameterEnum& valueEn, ParameterContinuous& valueCon, ButtonAnimation* animation = nullptr):
		MenuItemTypeDiscretePlus(name, baseColor, valueEn), valueCon(valueCon), animation(animation) {}
	void process(LedSlider& slider) override
	{
		MenuItemTypeDiscretePlus::process(slider);
		if(animation)
		{
			animation->process(HAL_GetTick(), slider, valueEn.get());
		}
	}
	void enterPlus() override
	{
		printf("DiscreteContinuous: going to slider\n\r");
		singleSliderMenuItem = MenuItemTypeSlider(baseColor, &valueCon);
		menu_in(singleSliderMenu);
	}
	ParameterContinuous& valueCon;
	ButtonAnimation* animation;
};

class MenuItemTypeDiscreteRange : public MenuItemTypeDiscretePlus
{
public:
	MenuItemTypeDiscreteRange(const char* name, rgb_t baseColor, ParameterEnum& valueEn, ParameterContinuous& valueConBottom, ParameterContinuous& valueConTop):
		MenuItemTypeDiscretePlus(name, baseColor, valueEn), valueConBottom(valueConBottom), valueConTop(valueConTop) {}
	void enterPlus() override
	{
		printf("DiscreteRange: going to range\n\r");
		singleRangeMenuItem = MenuItemTypeRange(baseColor, true, &valueConBottom, &valueConTop);
		menu_in(singleRangeMenu);
	}
	ParameterContinuous& valueConBottom;
	ParameterContinuous& valueConTop;
};

class MenuItemTypeDiscreteRangeCv : public MenuItemTypeDiscreteRange
{
public:
	MenuItemTypeDiscreteRangeCv(const char* name, rgb_t baseColor, ParameterEnum& valueEn, ParameterContinuous& valueConBottom, ParameterContinuous& valueConTop):
		MenuItemTypeDiscreteRange(name, baseColor, valueEn, valueConBottom, valueConTop) {}
	void event(Event e) override
	{
		MenuItemTypeDiscreteRange::event(e);
		if(Event::kTransitionFalling == e)
		{
			float bottom = 0;
			float top = 0;
			const float kMinus5 = 0;
			const float kGnd = 0.33;
			const float kPlus5 = 0.66;
			const float kPlus10 = 1;
			switch(valueEn.get())
			{
			case kCvRangeBipolar:
				bottom = kMinus5;
				top = kPlus5;
				break;
			case kCvRangeCustom:
				bottom = valueConBottom;
				top = valueConTop;
				break;
			case kCvRangeFull:
				bottom = kMinus5;
				top = kPlus10;
				break;
			case kCvRangePositive5:
				bottom = kGnd;
				top = kPlus5;
				break;
			case kCvRangePositive10:
				bottom = kGnd;
				top = kPlus10;
				break;
			}
			menu_enterDisplayRangeRaw(baseColor, bottom, top);
		}
	}
};

class MenuItemTypeDiscreteScaleMeterOutputMode : public MenuItemTypeDiscrete
{
public:
	MenuItemTypeDiscreteScaleMeterOutputMode(const char* name, rgb_t baseColor, ParameterEnum* parameter) :
		MenuItemTypeDiscrete(name, baseColor, parameter, nullptr) {}
	void event(Event e) override
	{
		MenuItemTypeDiscrete::event(e);
		if(Event::kTransitionFalling == e)
		{
			bool bottomEnv;
			bool topEnv;
			switch(parameter->get())
			{
			default:
			case 0:
				bottomEnv = 0;
				topEnv = 0;
				break;
			case 1:
				bottomEnv = 1;
				topEnv = 0;
				break;
			case 2:
				bottomEnv = 1;
				topEnv = 1;
				break;
			}
			menu_enterDisplayScaleMeterOutputMode(baseColor, bottomEnv, topEnv);
		}
	}
};

class MenuItemTypeExitSubmenu : public MenuItemTypeEvent
{
public:
	MenuItemTypeExitSubmenu(const char* name, rgb_t baseColor, uint32_t holdTime = 0) :
		MenuItemTypeEvent(name, baseColor, holdTime) {}
private:
	void event(Event e)
	{
		if(kTransitionRising == e) {
			menu_up();
		}
	}
};

class MenuItemTypeDisabled : public MenuItemType
{
public:
	MenuItemTypeDisabled() : MenuItemType({0, 0, 0}) {}
	void process(LedSlider& slider) override {}
};

constexpr size_t kMaxModeParameters = 3;
static const rgb_t buttonColor {0, 255, 255};
static const rgb_t buttonColorSimilar {0, 0, 255};
static MenuItemTypeDisabled disabled;

static ButtonAnimationSplit animationSplit(buttonColor, buttonColorSimilar);
static ButtonAnimationPulsatingStill animationPulsatingStill(buttonColor);
static MenuItemTypeDiscrete directControlModeSplit("directControlModeSplit", buttonColor, &gDirectControlMode.split, &animationSplit);
static MenuItemTypeDiscrete directControlModeLatch("directControlModeAutoLatch", buttonColor, &gDirectControlMode.autoLatch, &animationPulsatingStill);
static std::array<MenuItemType*,kMaxModeParameters> directControlModeMenu = {
		&disabled,
		&directControlModeLatch,
		&directControlModeSplit,
};

static ButtonAnimationSingleRepeatedEnv animationSingleRepeatedPulse{buttonColor};
static MenuItemTypeDiscrete recorderModeSplit("recorderModeSplit", buttonColor, &gRecorderMode.split, &animationSplit);
static MenuItemTypeDiscrete recorderModeRetrigger("recorderModeRetrigger", buttonColor, &gRecorderMode.retrigger, &animationSingleRepeatedPulse);
static ButtonAnimationRecorderInputMode animationRecorderInputMode{buttonColor};
static MenuItemTypeDiscrete recorderModeInputMode("recorderModeInputMode", buttonColor, &gRecorderMode.inputMode, &animationRecorderInputMode);
static std::array<MenuItemType*,kMaxModeParameters> recorderModeMenu = {
		&recorderModeInputMode,
		&recorderModeRetrigger,
		&recorderModeSplit,
};

static ButtonAnimationStillTriangle animationSingleStillTriangle{buttonColor};
static MenuItemTypeDiscreteScaleMeterOutputMode scaleMeterModeOutputMode("scaleMeterModeOutputMode", buttonColor, &gScaleMeterMode.outputMode);
static MenuItemTypeDiscrete scaleMeterModeCoupling("scaleMeterModeCoupling", buttonColor, &gScaleMeterMode.coupling, &animationSingleStillTriangle);
static MenuItemTypeEnterContinuous scaleMeterModeCutoff("scaleMeterModeCutoff", buttonColor, gScaleMeterMode.cutoff);
static std::array<MenuItemType*,kMaxModeParameters> scaleMeterModeMenu = {
		&scaleMeterModeCutoff,
		&scaleMeterModeCoupling,
		&scaleMeterModeOutputMode,
};

static ButtonAnimationWaveform animationWaveform{buttonColor};
static ButtonAnimationSpeedUpDown animationSpeedup(buttonColor);
static MenuItemTypeDiscrete balancedOscModeWaveform("balancedOscModeWaveform", buttonColor, &gBalancedOscsMode.waveform, &animationWaveform);
static MenuItemTypeEnterContinuous balancedOscModeCentreFrequency("centreFrequency", buttonColor, gBalancedOscsMode.centreFrequency, &animationSpeedup);
static ButtonAnimationRecorderInputMode animationBalancedOscsInputMode{buttonColor};
static MenuItemTypeDiscreteContinuous balancedOscModeInputModeAndFrequency("balancedOscModeInputModeAndFrequency", buttonColor,
		gBalancedOscsMode.inputMode, gBalancedOscsMode.centreFrequency, &animationBalancedOscsInputMode);
static std::array<MenuItemType*,kMaxModeParameters> balancedOscsModeMenu = {
		&disabled,
		&balancedOscModeInputModeAndFrequency,
		&balancedOscModeWaveform,
};

static MenuItemTypeDiscrete exprButtonsModeQuantised("gExprButtonsModeQuantised", buttonColor, &gExprButtonsMode.quantised);
static MenuItemTypeEnterContinuous exprButtonsModeModRange("gExprButtonsModeQuantisedModRange", buttonColor, gExprButtonsMode.modRange);

static MenuItemTypeEnterContinuous exprButtonsModeOffset0("gExprButtonsModeOffset0", buttonColor, gExprButtonsMode.offsetParameters[0]);
static MenuItemTypeEnterContinuous exprButtonsModeOffset1("gExprButtonsModeOffset1", buttonColor, gExprButtonsMode.offsetParameters[1]);
static MenuItemTypeEnterContinuous exprButtonsModeOffset2("gExprButtonsModeOffset2", buttonColor, gExprButtonsMode.offsetParameters[2]);
static MenuItemTypeEnterContinuous exprButtonsModeOffset3("gExprButtonsModeOffset3", buttonColor, gExprButtonsMode.offsetParameters[3]);
static MenuItemTypeEnterContinuous exprButtonsModeOffset4("gExprButtonsModeOffset4", buttonColor, gExprButtonsMode.offsetParameters[4]);

static MenuPage exprButtonsModeOffsets {
	"exprButtonsModeOffsets",
	{
		&exprButtonsModeOffset0,
		&exprButtonsModeOffset1,
		&exprButtonsModeOffset2,
		&exprButtonsModeOffset3,
		&exprButtonsModeOffset4,
	}
};

static MenuItemTypeEnterSubmenu exprButtonsModeEnterOffsets("", buttonColor, 20, exprButtonsModeOffsets);

static std::array<MenuItemType*,kMaxModeParameters> exprButtonsModeMenu = {
		&exprButtonsModeEnterOffsets,
		&exprButtonsModeModRange,
		&exprButtonsModeQuantised,
};

#ifdef TEST_MODE
static std::array<MenuItemType*,kMaxModeParameters> testModeMenu = {
		&disabled,
		&disabled,
		&disabled,
};
#endif // TEST_MODE

static std::array<std::array<MenuItemType*,kMaxModeParameters>*,kNumModes> modesMenuItems = {
#ifdef TEST_MODE
		&testModeMenu,
#endif // TEST_MODE
		&directControlModeMenu,
		&recorderModeMenu,
		&scaleMeterModeMenu,
		&balancedOscsModeMenu,
		&exprButtonsModeMenu,
};

MenuItemTypeNextMode nextMode("nextMode", {0, 255, 0});
MenuItemTypeExitSubmenu exitMe("exit", {127, 255, 0});

static MenuItemTypeEnterSubmenu enterGlobalSettings("GlobalSettings", {120, 120, 0}, 20, globalSettingsMenu);
class GlobalSettings : public ParameterUpdateCapable {
public:
	void updated(Parameter& p)
	{
		bool verbose = false;
		char const* str = "+++";
		if(p.same(outRangeEnum)) {
			gOutRange = outRangeEnum;
			str = "outRangeEnum";
		}
		else if(p.same(outRangeBottom) || p.same(outRangeTop)) {
			outRangeEnum.set(kCvRangeCustom);
			gOutRange = outRangeEnum;
			gOutRangeBottom = outRangeBottom;
			gOutRangeTop = outRangeTop;
			str = "outRangeTop/Bottom";
		}
		else if(p.same(inRangeEnum)) {
			str = "inRangeEnum";
			gInRange = inRangeEnum;
		}
		else if(p.same(inRangeTop) || p.same(inRangeBottom)) {
			inRangeEnum.set(kCvRangeCustom);
			gInRange = inRangeEnum;
			gInRangeBottom = inRangeBottom;
			gInRangeTop = inRangeTop;
			str = "inRangeTop/Bottom";
		} else if(p.same(jacksOnTop)) {
			gJacksOnTop = jacksOnTop;
			str = "jacksOnTop";
		}
		else if(p.same(sizeScaleCoeff))
			str = "sizeScaleCoeff";
		else if(p.same(newMode)) {
			str = "newMode";
			requestNewMode(newMode);
		}
		if(verbose)
			printf("%s\n\r", str);
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD9(outRangeBottom, outRangeTop, outRangeEnum,
				inRangeBottom, inRangeTop, inRangeEnum,
					sizeScaleCoeff, jacksOnTop, newMode);
	}
	GlobalSettings() :
		presetFieldData {
			.outRangeBottom = outRangeBottom,
			.outRangeTop = outRangeTop,
			.inRangeBottom = inRangeBottom,
			.inRangeTop = inRangeTop,
			.sizeScaleCoeff = sizeScaleCoeff,
			.outRangeEnum = outRangeEnum,
			.inRangeEnum = inRangeEnum,
			.jacksOnTop = jacksOnTop,
			.newMode = newMode,
		}
	{
		PresetDesc_t presetDesc = {
			.field = this,
			.size = sizeof(PresetFieldData_t),
			.defaulter = genericDefaulter9(GlobalSettings, outRangeBottom, outRangeTop, outRangeEnum,
					inRangeBottom, inRangeTop, inRangeEnum,
						sizeScaleCoeff, jacksOnTop, newMode),
			// currently the {out,in}RangeEnums have to go after the corresponding
			// corresponding Range{Bottom,Top}, as setting the Range last would otherwise
			// reset the enum
			// TODO: make this more future-proof
			.loadCallback = genericLoadCallback9(GlobalSettings, outRangeBottom, outRangeTop, outRangeEnum,
							inRangeBottom, inRangeTop, inRangeEnum,
								sizeScaleCoeff, jacksOnTop, newMode),
		};
		presetDescSet(5, &presetDesc);
	}
	ParameterEnumT<kCvRangeNum> outRangeEnum {this, 0};
	ParameterContinuous outRangeBottom {this, 0.2};
	ParameterContinuous outRangeTop {this, 0.8};
	ParameterEnumT<kCvRangeNum> inRangeEnum {this, 0};
	ParameterContinuous inRangeBottom {this, 0.2};
	ParameterContinuous inRangeTop {this, 0.8};
	ParameterContinuous sizeScaleCoeff {this, 0.5};
	ParameterEnumT<2> jacksOnTop {this, false};
	ParameterEnumT<kNumModes> newMode{this, gNewMode};
	PACKED_STRUCT(PresetFieldData_t {
		float outRangeBottom;
		float outRangeTop;
		float inRangeBottom;
		float inRangeTop;
		float sizeScaleCoeff;
		uint8_t outRangeEnum;
		uint8_t inRangeEnum;
		uint8_t jacksOnTop;
		uint8_t newMode;
	}) presetFieldData;
} gGlobalSettings;

static void requestNewMode(int mode)
{
	bool different = (gNewMode != mode);
	gNewMode = mode;
	// notify the setting that is stored to disk,
	// but avoid the set() to trigger a circular call to requestNewMode()
	if(different)
		gGlobalSettings.newMode.set(mode);
}

static MenuItemTypeDisplayRangeRaw displayRangeRawMenuItem;
// this is a submenu consisting of a full-screen display of a range. Before entering it,
// appropriately set the properties of displayIoRangeMenuItem
MenuPage displayRangeRawMenu("display io range", {&displayRangeRawMenuItem}, MenuPage::kMenuTypeRaw);

MenuItemTypeDisplayScaleMeterOutputMode displayScaleMeterOutputModeMenuItem;
// this is a submenu consisting of a full-screen display of I/O range enum. Before entering it,
// appropriately set the properties of displayScaleMeterOutputMode
MenuPage displayScaleMeterOutputModeMenu("display scalemeter output mode", {&displayScaleMeterOutputModeMenuItem}, MenuPage::kMenuTypeRange);

static constexpr rgb_t globalSettingsColor = {255, 127, 0};
static MenuItemTypeDiscreteRangeCv globalSettingsOutRange("globalSettingsOutRange", globalSettingsColor, gGlobalSettings.outRangeEnum, gGlobalSettings.outRangeBottom, gGlobalSettings.outRangeTop);
static MenuItemTypeDiscreteRangeCv globalSettingsInRange("globalSettingsInRange", globalSettingsColor, gGlobalSettings.inRangeEnum, gGlobalSettings.inRangeBottom, gGlobalSettings.inRangeTop);
static ButtonAnimationTriangle animationTriangle(globalSettingsColor, 3000);
static MenuItemTypeEnterContinuous globalSettingsSizeScale("globalSettingsSizeScale", globalSettingsColor, gGlobalSettings.sizeScaleCoeff, &animationTriangle);
static ButtonAnimationBrightDimmed animationBrightDimmed(globalSettingsColor);
static MenuItemTypeEnterQuantised globalSettingsJacksOnTop("globalSettingsJacksOnTop", globalSettingsColor, gGlobalSettings.jacksOnTop, &animationBrightDimmed);

static bool isCalibration;
static bool menuJustEntered;

static void menu_enterRangeDisplay(const rgb_t& color, bool autoExit, ParameterContinuous& bottom, ParameterContinuous& top, const float& display)
{
	gAlt = 1;
	singleRangeDisplayMenuItem = MenuItemTypeRangeDisplayCentroids(color, autoExit, &bottom, &top, display);
	menu_in(singleRangeDisplayMenu);
}

static void menu_enterDisplayRangeRaw(const rgb_t& color, float bottom, float top)
{
	gAlt = 1;
	displayRangeRawMenuItem = MenuItemTypeDisplayRangeRaw(color, bottom, top);
	menu_in(displayRangeRawMenu);
}

static void menu_enterDisplayScaleMeterOutputMode(const rgb_t& color, bool bottomEnv, bool topEnv)
{
	gAlt = 1;
	displayScaleMeterOutputModeMenuItem = MenuItemTypeDisplayScaleMeterOutputMode(color, bottomEnv, topEnv);
	menu_in(displayScaleMeterOutputModeMenu);
}

static std::vector<MenuPage*> menuStack;

static void menu_update()
{
	// these vectors should really be initialised at startup but they have circular dependencies
	static bool inited = false;
	if(!inited)
	{
		inited = true;
		globalSettingsMenu.items = {
			&disabled, // TODO
			&globalSettingsJacksOnTop,
			&globalSettingsSizeScale,
			&globalSettingsInRange,
			&globalSettingsOutRange,
		};
		singleSliderMenu.items = {
			&singleSliderMenuItem,
		};
		mainMenu.items = {
			&enterGlobalSettings,
			&disabled, // mode-dependent
			&disabled, // mode-dependent
			&disabled, // mode-dependent
			&nextMode,
		};
	}
	bool hasChanged = false;
	for(size_t n = 0; n < kMaxModeParameters; ++n)
	{
		// make sure we are displaying the buttons for the current mode
		// if hasChanged, this will retrigger a new drawing of the buttons below
		// TODO: when refactoring mode switching, maybe ensure the menu's content and visualisation
		// gets updated directly when updating mode
		if(mainMenu.items[1 + n] != (*modesMenuItems[gNewMode])[n])
		{
			mainMenu.items[1 + n] = (*modesMenuItems[gNewMode])[n];
			hasChanged = true;
		}
	}
	MenuPage* newMenu = menuStack.size() ? menuStack.back() : nullptr;
	if(newMenu && (activeMenu != newMenu || hasChanged))
	{
		activeMenu = newMenu;
		printf("menu_update: %s\n\r", newMenu ? newMenu->name : "___");
		// clear display
		np.clear();
		// TODO: the below is not particularly elegant: add a parameter to MenuPage
		if(MenuPage::kMenuTypeButtons == activeMenu->type)
		{
			//buttons
			ledSlidersSetupMultiSlider(
				ledSlidersAlt,
				{
					activeMenu->items[0]->baseColor,
					activeMenu->items[1]->baseColor,
					activeMenu->items[2]->baseColor,
					activeMenu->items[3]->baseColor,
					activeMenu->items[4]->baseColor,
				},
				LedSlider::MANUAL_CENTROIDS,
				true
			);
			menuJustEntered = true;
		} else {
			size_t maxNumCentroids = MenuPage::kMenuTypeRange == activeMenu->type ? 2 : 1;
			LedSlider::LedMode_t ledMode;
			switch(activeMenu->type)
			{
			default:
				case MenuPage::kMenuTypeRange:
					ledMode = LedSlider::MANUAL_CENTROIDS;
					break;
				case MenuPage::kMenuTypeSlider:
					ledMode = LedSlider::AUTO_CENTROIDS;
					break;
				case MenuPage::kMenuTypeQuantised:
					ledMode = LedSlider::MANUAL_CENTROIDS;
					break;
				case MenuPage::kMenuTypeRaw:
					ledMode = LedSlider::MANUAL_RAW;
					break;
			}
			ledSlidersSetupMultiSlider(
				ledSlidersAlt,
				{
					activeMenu->items[0]->baseColor,
				},
				ledMode,
				true,
				maxNumCentroids
			);
			menuJustEntered = false; // this is immediately interactive
		}
		isCalibration = false;
	}
}

void menu_exit()
{
	// when exiting menu, ensure any changes
	// to parameters are reflected in the presets
	// so that presetCheckSave() can write them soon thereafter
	for(auto& p : performanceModes)
		p->updatePreset();
	gGlobalSettings.updatePreset();

	menuStack.resize(0);
	activeMenu = nullptr;
	printf("menu_exit\n\r");
	np.clear();
	gAlt = 0;
}

static void menu_up()
{
	printf("menu_up from %d %s\n\r", menuStack.size(), menuStack.back()->name);
	if(menuStack.size())
		menuStack.pop_back();
	if(!menuStack.size())
		menu_exit();
}

static void menu_in(MenuPage& menu)
{
	printf("menu_in from %s to %s\n\r", menuStack.size() ? menuStack.back()->name : "___", menu.name);
	menuStack.emplace_back(&menu);
}

int menu_dosetup(MenuPage& menu)
{
	menuStack.resize(0);
	menu_in(menu);
	menu_update(); // TODO: is this needed?
	return true;
}

int menu_setup(double)
{
	return menu_dosetup(mainMenu);
}

void menu_render(BelaContext*)
{
	// if button pressed, go back up
	if (menuBtn.onset){
		// button onset
		printf("button when stack is %d\n\r", menuStack.size());
		menu_up();
	}

	menu_update();
	if(!activeMenu)
		return;
	// process these regardless to ensure the display is updated
	// (i.e.: if menuJustEntered, this will make sure the buttons are shown)
	ledSlidersAlt.process(trill.rawData.data());
	// update touches
	if(menuJustEntered)
	{
		// if we just entered the menu, ensure we have removed
		// all fingers once before enabling interaction
		if(globalSlider.getNumTouches())
			return;
		menuJustEntered = false;
	}
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
		for(size_t n = 0; n < ledSlidersAlt.sliders.size(); ++n)
			activeMenu->items[n]->process(ledSlidersAlt.sliders[n]);
	}
}
