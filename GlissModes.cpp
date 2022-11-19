#include "TrillRackInterface.h"
#include "GlissModes.h"
#include <vector>
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include <libraries/Oscillator/Oscillator.h>
#include "LedSliders.h"

static_assert(kNumOutChannels >= 2); // too many things to list depend on this in this file.

//#define TRIGGER_IN_TO_CLOCK_USES_MOVING_AVERAGE

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

// Mode switching
int gMode = 0;
OutMode gOutMode = kOutModeFollowTouch;
int gCounter = 0;
int gSubMode = 0;
bool gSecondTouchIsSize;

// Recording the gesture
enum { kMaxRecordLength = 1000 };
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
		active[0] = sliders[0].getNumTouches();
		if(single)
			active[1] = active[0];
		else
		{
			active[1] = sliders[1].getNumTouches();
		}
		HalfGesture_t out[2];
		static bool pastAnalogIn = false;
		bool analogIn = tri.analogRead() > 0.5;
		// reset on rising edge on analog or button ins
		if((analogIn && !pastAnalogIn) || performanceBtn.onset)
		{
			assert(active.size() == rs.size());
			for(size_t n = 0; n < active.size(); ++n)
				if(!active[n])
					rs[n].enable(true);
		}
		pastAnalogIn = analogIn;

		for(unsigned int n = 0; n < active.size(); ++n)
		{
			if(active[n] != pastActive[n]) //state change
			{
				printf("[%d] newAc: %d, pastAc: %d\n\r", n, active[n], pastActive[n]);
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
	ParameterEnumT<2> split{this, false};
	ParameterEnumT<2> autoLatch{this, false};
private:
	rgb_t colors[2] = {
		{255, 0, 0},
		{255, 0, 127},
	};
	LatchProcessor latchProcessor;
	std::array<bool,2> isLatched = {false, false};
	uint32_t lastLatchCount = ButtonView::kPressCountInvalid;
} gDirectControlMode;

class RecorderMode : public PerformanceMode {
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
	void render(BelaContext*) override
	{
		if(split)
			gestureRecorderSplit_loop(retrigger);
		else
			gestureRecorderSingle_loop(retrigger);
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
		}
	}
	ParameterEnumT<2> split{this, false};
	ParameterEnumT<2> retrigger{this, true};
	ParameterEnumT<3> inputMode{this, 0};
private:
	rgb_t colors[2] = {
			{128, 128, 0},
			{128, 128, 100},
	};
} gRecorderMode;

static void menu_enterRangeDisplay(const rgb_t& color, bool autoExit, ParameterContinuous& bottom, ParameterContinuous& top, const float& display);
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
	ParameterEnumT<kOutputModeNum> outputMode {this, 0};
	ParameterEnumT<kCouplingNum> coupling {this, kCouplingDc};
	ParameterContinuous cutoff {this, 0.5};
	ParameterContinuous outRangeBottom {this, 0};
	ParameterContinuous outRangeTop {this, 1};
	ParameterContinuous inRangeBottom {this, 0};
	ParameterContinuous inRangeTop {this, 1};
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
		gOutMode = kOutModeManualBlock;
		if(ms < 0)
			return true;
		return modeChangeBlinkSplit(ms, gBalancedLfoColors.data(), kNumLeds / 2, kNumLeds / 2);
	}
	void render(BelaContext* context) override
	{
		if(!inited)
		{
			inited = true;
			// deferred initialisations ...
			sampleRate = context->analogSampleRate;
			clockPeriod = context->analogSampleRate / 5.f; // 5 Hz
		}
		switch (inputMode)
		{
		case kInputModeTrig:
			// after frequency was set by parameter,
			// we use gClockPeriod only if it gets updated at least once
			if(gClockPeriodUpdated != lastClockPeriodUpdate)
				clockPeriod = gClockPeriod;
			lastClockPeriodUpdate = gClockPeriodUpdated;
			break;
		case kInputModeCv:
			// TODO:
			break;
		}

		float midFreq = context->analogFrames / clockPeriod; // we process once per block; the oscillator thinks Fs = 1
		float touchPosition = ledSliders.sliders[0].compoundTouchLocation();

		if (touchPosition > 0.0) {
			divisionPoint = touchPosition;
		}


		std::array<float,oscillators.size()> freqs = {
				(0.92f - divisionPoint) * midFreq * 2.f,
				divisionPoint * midFreq * 2,
		};

		// limit max brightness. On the one hand, it reduces power consumption,
		// on the other hand it attempts to avoid the upper range, where it becomes hard
		// to discern increased brightness.
		const float kMaxBrightness = 0.4f;
		for(size_t n = 0; n < oscillators.size(); ++n) {
			float out = oscillators[n].process(freqs[n]);
			gManualAnOut[!n] = map(out, -1, 1, 0, 1); // The ! is so that the CV outs order matches the display. TODO: tidy up
			if(!gAlt)
			{
				// if we are not in menu mode, set the display
				// TODO: move this to LedSlider so that it obeys to the ledEnabled there

				float brightness = map(out, -1, 1, 0, kMaxBrightness);
				unsigned int split = divisionPoint > 0 ? kNumLeds * divisionPoint : 0;
				unsigned int start = (0 == n) ? 0 : split;
				unsigned int stop = (0 == n) ? split : kNumLeds;
				rgb_t color = {uint8_t(brightness * gBalancedLfoColors[n].r), uint8_t(brightness * gBalancedLfoColors[n].g), uint8_t(brightness * gBalancedLfoColors[n].b)};
				for(unsigned int p = start; p <  stop; ++p)
					np.setPixelColor(p, color.r, color.g, color.b);
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
				clockPeriod = sampleRate / (centreFrequency * 10.f + 0.1f); // TODO: more useful range? exp mapping?
//				printf("centreFrequency: %.3f => %.3f samples\n\r", centreFrequency.get(), clockPeriod);
		} else if (p.same(inputMode)) {

		}
	}
	typedef enum {
		kInputModeTrig,
		kInputModeCv,
		kNumInputModes,
	} InputMode;
	ParameterEnumT<Oscillator::numOscTypes> waveform {this, Oscillator::triangle};
	ParameterContinuous centreFrequency {this};
	ParameterEnumT<kNumInputModes> inputMode {this, kInputModeTrig};
private:
	float divisionPoint = 0.5;
	float clockPeriod; // deferred initialisation
	float sampleRate; // deferred initialisation
	uint32_t lastClockPeriodUpdate = gClockPeriodUpdated;
	bool inited = false;
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
					{0, 255, 0},
					{0, 200, 50},
					{0, 150, 100},
					{0, 100, 150},
					{0, 50, 200},
				},
				LedSlider::MANUAL_CENTROIDS,
				true
			);
			gOutMode = kOutModeManualBlock;
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
		float scale = 0.1;
		if(pitchBeingAdjusted >= 0)
		{
			// if we are adjusting the pitch, output that instead
			gManualAnOut[0] = offsets[pitchBeingAdjusted];
			pitchBeingAdjustedCount++;
			// hold it for a few blocks
			if(pitchBeingAdjustedCount >= kPitchBeingAdjustedCountMax)
			{
				pitchBeingAdjustedCount = 0;
				pitchBeingAdjusted = -1;
			}
		}
		else
			ledSlidersExpButtonsProcess(ledSliders, gManualAnOut, scale, offsets);
	}
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
	ParameterContinuous modRange {this, 0.5};
	ParameterEnumT<2> quantised {this, true};
	std::array<ParameterContinuous,5> offsetParameters {
		ParameterContinuous(this, 0.5),
		ParameterContinuous(this, 0.6),
		ParameterContinuous(this, 0.7),
		ParameterContinuous(this, 0.8),
		ParameterContinuous(this, 0.9),
	};
private:
	static constexpr size_t kNumButtons = 5;
	std::array<float,kNumButtons> offsets;
	int pitchBeingAdjusted = -1;
	unsigned int pitchBeingAdjustedCount = 0;
	static constexpr unsigned int kPitchBeingAdjustedCountMax = 10;
} gExprButtonsMode;

static unsigned int gCurrentMode;
static std::array<PerformanceMode*,kNumModes> performanceModes = {
	&gDirectControlMode,
	&gRecorderMode,
	&gScaleMeterMode,
	&gBalancedOscsMode,
	&gExprButtonsMode,
};

void performanceMode_update(unsigned int newMode)
{
	if(newMode < performanceModes.size())
		gCurrentMode = newMode;
}

bool performanceMode_setup(double ms)
{
	return performanceModes[gCurrentMode]->setup(ms);
}

void performanceMode_render(BelaContext* context)
{
	performanceModes[gCurrentMode]->render(context);
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

class ButtonParameterAnimation {
public:
	virtual void process(uint32_t ms, LedSlider& ledSlider, float value) = 0;
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

class ButtonParameterAnimationSplit : public ButtonParameterAnimation {
public:
	ButtonParameterAnimationSplit(rgb_t color0, rgb_t color1) :
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

class ButtonParameterAnimationPulsatingStill : public ButtonParameterAnimation {
public:
	ButtonParameterAnimationPulsatingStill(rgb_t color) :
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

class ButtonParameterAnimationStillTriangle : public ButtonParameterAnimation {
public:
	ButtonParameterAnimationStillTriangle(rgb_t color) :
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
	unsigned int width = -1;
};

class ButtonParameterAnimationSingleRepeatedEnv: public ButtonParameterAnimation {
public:
	ButtonParameterAnimationSingleRepeatedEnv(rgb_t color) :
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

#include <math.h>

class ButtonParameterAnimationWaveform: public ButtonParameterAnimation {
public:
	ButtonParameterAnimationWaveform(rgb_t color) :
		color(color) {}
	void process(uint32_t ms, LedSlider& ledSlider, float value) override {
		rgb_t c;
		const unsigned int period = 800;
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

class ButtonParameterAnimationSpeedUpDown: public ButtonParameterAnimation {
public:
	ButtonParameterAnimationSpeedUpDown(rgb_t color) :
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
		float coeff = mapAndConstrain(phase < onTime, 0, 1, 0.3, 1);
		c.r = color.r * coeff;
		c.g = color.g * coeff;
		c.b = color.b * coeff;
		ledSlider.setColor(c);
	};
protected:
	static constexpr float initialPeriod = 300;
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
	MenuItemTypeDiscrete(const char* name, rgb_t baseColor, ParameterEnum* parameter, ButtonParameterAnimation* animation = nullptr) :
		MenuItemTypeEvent(name, baseColor, 0), parameter(parameter), animation(animation) {}
private:
	void process(LedSlider& slider) override
	{
		MenuItemTypeEvent::process(slider);
		if(animation)
			animation->process(HAL_GetTick(), slider, parameter->get());
	}

	void event(Event e)
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
	ButtonParameterAnimation* animation;
};

class MenuItemTypeSlider : public MenuItemType {
public:
	MenuItemTypeSlider(): MenuItemType({0, 0, 0}) {}
	MenuItemTypeSlider(const rgb_t& color, ParameterContinuous* parameter) :
		MenuItemType(color), parameter(parameter) {}
	void process(LedSlider& slider) override
	{
		if(parameter)
		{
			TouchFrame frame {
				.pos = slider.compoundTouchLocation(),
				.sz = slider.compoundTouchSize(),
			};
			bool latched = false;
			autoLatcher.process(frame, latched);
			parameter->set(frame.pos);
			if(latched)
				menu_up();
		}
	}
	ParameterContinuous* parameter;
	static AutoLatcher autoLatcher;
};
AutoLatcher MenuItemTypeSlider::autoLatcher;

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

class MenuItemTypeRangeDisplay : public MenuItemTypeRange {
public:
	MenuItemTypeRangeDisplay(){}
	MenuItemTypeRangeDisplay(const rgb_t& color, bool autoExit, ParameterContinuous* paramBottom, ParameterContinuous* paramTop, const float& display) :
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

static int shouldChangeMode = 1;
class MenuItemTypeNextMode : public MenuItemTypeEvent
{
public:
	MenuItemTypeNextMode(const char* name, rgb_t baseColor) :
		MenuItemTypeEvent(name, baseColor, 0) {}
private:
	void event(Event e) override
	{
		if(kTransitionRising == e)
			shouldChangeMode = 1;
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
		kMenuTypeRange,
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

static MenuItemTypeRangeDisplay singleRangeDisplayMenuItem;
// this is a submenu consisting of a range+display (no buttons). Before entering it,
// appropriately set the properties of singleRangeDisplayMenuItem
MenuPage singleRangeDisplayMenu("single range", {&singleRangeDisplayMenuItem}, MenuPage::kMenuTypeRange);

// If held-press, get into singleSliderMenu to set value
class MenuItemTypeEnterContinuous : public MenuItemTypeEnterSubmenu
{
public:
	MenuItemTypeEnterContinuous(const char* name, rgb_t baseColor, ParameterContinuous& value, ButtonParameterAnimation* animation = nullptr) :
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
	ButtonParameterAnimation* animation;
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
	MenuItemTypeDiscreteContinuous(const char* name, rgb_t baseColor, ParameterEnum& valueEn, ParameterContinuous& valueCon):
		MenuItemTypeDiscretePlus(name, baseColor, valueEn), valueCon(valueCon) {}
	void enterPlus() override
	{
		printf("DiscreteContinuous: going to slider\n\r");
		singleSliderMenuItem = MenuItemTypeSlider(baseColor, &valueCon);
		menu_in(singleSliderMenu);
	}
	ParameterContinuous& valueCon;
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

class DummyClass : public ParameterUpdateCapable {
public:
	void updated(Parameter& p) override
	{
		printf("Dummy class: %p => ", &p);
		if(p.same(par))
			printf("par set to %d\n\r", par.get());
		else if (p.same(par2))
			printf("par2 set to %f\n\r", par2.get());
	}
	ParameterEnumT<4> par{this, 2};
	ParameterContinuous par2{this, 0.4};
} gDummyClassObj;

constexpr size_t kMaxModeParameters = 3;
static const rgb_t buttonColor {0, 255, 255};
static const rgb_t buttonColorSimilar {0, 0, 255};
static MenuItemTypeDisabled disabled;

static ButtonParameterAnimationSplit animationSplit(buttonColor, buttonColorSimilar);
static ButtonParameterAnimationPulsatingStill animationPulsatingStill(buttonColor);
static MenuItemTypeDiscrete directControlModeSplit("directControlModeSplit", buttonColor, &gDirectControlMode.split, &animationSplit);
static MenuItemTypeDiscrete directControlModeLatch("directControlModeAutoLatch", buttonColor, &gDirectControlMode.autoLatch, &animationPulsatingStill);
static std::array<MenuItemType*,kMaxModeParameters> directControlModeMenu = {
		&disabled,
		&directControlModeLatch,
		&directControlModeSplit,
};

static ButtonParameterAnimationSingleRepeatedEnv animationSingleRepeatedPulse{buttonColor};
static MenuItemTypeDiscrete recorderModeSplit("recorderModeSplit", buttonColor, &gRecorderMode.split, &animationSplit);
static MenuItemTypeDiscrete recorderModeRetrigger("recorderModeRetrigger", buttonColor, &gRecorderMode.retrigger, &animationSingleRepeatedPulse);
static MenuItemTypeDiscrete recorderModeInputMode("recorderModeInputMode", buttonColor, &gRecorderMode.inputMode);
static std::array<MenuItemType*,kMaxModeParameters> recorderModeMenu = {
		&recorderModeInputMode,
		&recorderModeRetrigger,
		&recorderModeSplit,
};

static ButtonParameterAnimationStillTriangle animationSingleStillTriangle{buttonColor};
static MenuItemTypeDiscrete scaleMeterModeOutputMode("scaleMeterModeOutputMode", buttonColor, &gScaleMeterMode.outputMode);
static MenuItemTypeDiscrete scaleMeterModeCoupling("scaleMeterModeCoupling", buttonColor, &gScaleMeterMode.coupling, &animationSingleStillTriangle);
static MenuItemTypeEnterContinuous scaleMeterModeCutoff("scaleMeterModeCutoff", buttonColor, gScaleMeterMode.cutoff);
static std::array<MenuItemType*,kMaxModeParameters> scaleMeterModeMenu = {
		&scaleMeterModeCutoff,
		&scaleMeterModeCoupling,
		&scaleMeterModeOutputMode,
};

static ButtonParameterAnimationWaveform animationWaveform{buttonColor};
static ButtonParameterAnimationSpeedUpDown animationSpeedup(buttonColor);
static MenuItemTypeDiscrete balancedOscModeWaveform("balancedOscModeWaveform", buttonColor, &gBalancedOscsMode.waveform, &animationWaveform);
static MenuItemTypeEnterContinuous balancedOscModeCentreFrequency("centreFrequency", buttonColor, gBalancedOscsMode.centreFrequency, &animationSpeedup);
static MenuItemTypeDiscrete balancedOscModeInputMode("balancedOscModeInputMode", buttonColor, &gBalancedOscsMode.inputMode);
static std::array<MenuItemType*,kMaxModeParameters> balancedOscsModeMenu = {
		&balancedOscModeInputMode,
		&balancedOscModeCentreFrequency,
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

static std::array<std::array<MenuItemType*,kMaxModeParameters>*,kNumModes> modesMenuItems = {
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
//		printf("GlobalSettings updated: %p\n\r", &p);
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
		}
		else if(p.same(sizeScaleCoeff))
			str = "sizeScaleCoeff";
		printf("%s\n\r", str);
	}
	ParameterEnumT<kCvRangeNum> outRangeEnum {this, 0};
	ParameterContinuous outRangeBottom {this, 0.2};
	ParameterContinuous outRangeTop {this, 0.8};
	ParameterEnumT<kCvRangeNum> inRangeEnum {this, 0};
	ParameterContinuous inRangeBottom {this, 0.8};
	ParameterContinuous inRangeTop {this, 0.8};
	ParameterContinuous sizeScaleCoeff {this, 0.5};
} gGlobalSettings;

static MenuItemTypeEnterContinuous globalSettingsSizeScale("globalSettingsSizeScale", {255, 127, 0}, gGlobalSettings.sizeScaleCoeff);
static MenuItemTypeDiscreteRange globalSettingsOutRange("globalSettingsOutRange", {255, 127, 0}, gGlobalSettings.outRangeEnum, gGlobalSettings.outRangeBottom, gGlobalSettings.outRangeTop);
static MenuItemTypeDiscreteRange globalSettingsInRange("globalSettingsInRange", {255, 127, 0}, gGlobalSettings.inRangeEnum, gGlobalSettings.inRangeBottom, gGlobalSettings.inRangeTop);

static bool isCalibration;
static bool menuJustEntered;
int menuShouldChangeMode()
{
	int tmp = shouldChangeMode;
	shouldChangeMode = 0;
	return tmp;
}

static void menu_enterRangeDisplay(const rgb_t& color, bool autoExit, ParameterContinuous& bottom, ParameterContinuous& top, const float& display)
{
	gAlt = 1;
	singleRangeDisplayMenuItem = MenuItemTypeRangeDisplay(color, autoExit, &bottom, &top, display);
	menu_in(singleRangeDisplayMenu);
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
			&disabled, // TODO
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
		if(mainMenu.items[1 + n] != (*modesMenuItems[gCurrentMode])[n])
		{
			mainMenu.items[1 + n] = (*modesMenuItems[gCurrentMode])[n];
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
			LedSlider::LedMode_t ledMode = MenuPage::kMenuTypeRange == activeMenu->type ? LedSlider::MANUAL_CENTROIDS : LedSlider::AUTO_CENTROIDS;
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
