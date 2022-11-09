#include "TrillRackInterface.h"
#include "GlissModes.h"
#include <vector>
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include <libraries/Oscillator/Oscillator.h>
#include "LedSliders.h"

//#define TRIGGER_IN_TO_CLOCK_USES_MOVING_AVERAGE

extern int gAlt;
extern TrillRackInterface tri;
extern const unsigned int kNumPads;
extern const unsigned int kNumLeds;
extern unsigned int gOutRange;
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
const float kFixedCentroidSize = 0.9;

// LED Flash Event
double gEndTime = 0; //for pulse length
double gPulseLength = 40;

float gClockPeriod = 20000; // some initial value so we start oscillating even in the absence of external clock
// Dual LFOS
float gDivisionPoint = 0;
// ------------------


LedSliders ledSliders;
LedSliders ledSlidersAlt;
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
#endif // TRIGGER_IN_TO_CLOCK_USES_MOVING_AVERAGE
			}
			lastTrig = newTrig;
			lastTrigPrimed = true;
		}

		lastIn = in;
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
			.order = padsToOrderMap,
			.sizeScale = kSizeScale,
			.boundaries = boundaries,
			.maxNumCentroids = {1},
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

static void processLatch(bool split)
{
	static bool pastButton = false;
	bool buttonOnset = false;

	bool button = !tri.digitalRead(0);
	if(button && !pastButton)
		buttonOnset = true;

	static std::array<bool,2> isLatched = {false, false};
	static std::array<bool,2> unlatchArmed = {false, false};
	static std::array<float,2> latchedValues;

	std::array<float,2> values;

	if(split)
	{
		values[0] = ledSliders.sliders[0].compoundTouchLocation();
		values[1] = ledSliders.sliders[1].compoundTouchLocation();

	} else {
		values[0] = ledSliders.sliders[0].compoundTouchLocation();
		values[1] = ledSliders.sliders[0].compoundTouchSize();
	}

	std::array<bool,2> hasTouch = {false, false};
	for(ssize_t n = 0; n < 1 + split; ++n)
		hasTouch[n] = ledSliders.sliders[n].compoundTouchSize() > 0;

	std::array<bool,2> latchStarts = {false, false};
	std::array<bool,2> unlatchStarts = {false, false};
	if(buttonOnset)
	{
		// button latches everything if there is at least one touch
		// and unlatches everything if there is no touch
		for(size_t n = 0; n < isLatched.size(); ++n)
		{
			if(!isLatched[n] && hasTouch[n])
				latchStarts[n] = true;
			if(!(hasTouch[0] || hasTouch[1]))
			{
				// no touch
				if(isLatched[n])
					unlatchStarts[n] = true;
			}
		}
	}
#if 1
	if(!split)
	{
		// try to hold without button
		static std::array<std::array<float,latchedValues.size()>,30> pastValues;
		static size_t idx = 0;
		size_t pastIdx = (idx - 1 + pastValues.size()) % pastValues.size();
		if(!values[1] && pastValues[pastIdx][1]) // if size went to zero
		{
			values[0] = pastValues[idx][0];
			values[1] = pastValues[idx][1];
			latchStarts[0] = true;
			static bool happend = false;
			if(!happend)
				printf("L\n\r");
			happend = true;
			pastValues[idx][0] = pastValues[idx][1] = 0;
		} else {
			// store past values so we can use them later
			pastValues[idx] = values;
		}
		++idx;
		if(idx >= pastValues.size())
			idx = 0;
	}
#endif
	for(ssize_t n = 0; n < 1 + split; ++n)
	{
		if(isLatched[n])
		{
			if(!hasTouch[n])
			{
				unlatchArmed[n] = true;
			}
			if(unlatchArmed[n] && hasTouch[n])
			{
				unlatchStarts[n] = true;
			}
		}

		if(latchStarts[n])
		{
			latchedValues[n] = values[n];
			if(!split)
				latchedValues[1] = values[1];
			isLatched[n] = true;
			unlatchArmed[n] = false;
		}
		if(unlatchStarts[n])
			isLatched[n] = false;
	}

	if(isLatched[0] || isLatched[1]) {
		gOutMode = kOutModeFollowLeds;
		LedSlider::centroid_t centroid;
		if(split)
		{
			centroid.size = kFixedCentroidSize;
			for(size_t n = 0; n < isLatched.size(); ++n)
			{
				if(isLatched[n])
				{
					centroid.location = latchedValues[n];
					centroid.size = kFixedCentroidSize;
				} else {
					// this is not actually latched, but as gOutMode
					// is not split, we emulate direct control here.
					centroid.location = values[n];
					centroid.size = kFixedCentroidSize * hasTouch[n]; // TODO: when bipolar this should send out "nothing"
				}
				ledSliders.sliders[n].setLedsCentroids(&centroid, 1);
			}
		} else {
			centroid.location = latchedValues[0];
			centroid.size = latchedValues[1];
			ledSliders.sliders[0].setLedsCentroids(&centroid, 1);
		}
	}
	else
		gOutMode = kOutModeFollowTouch;
	pastButton = button;
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
		static bool pastAnalogIn = false;
		static bool pastButtonIn = false;
		bool analogIn = tri.analogRead() > 0.5;
		bool buttonIn = !tri.digitalRead(0);
		// reset on rising edge on analog or button ins
		if((analogIn && !pastAnalogIn) || (buttonIn && !pastButtonIn))
		{
			assert(active.size() == rs.size());
			for(size_t n = 0; n < active.size(); ++n)
				if(!active[n])
					rs[n].enable(true);
		}
		pastAnalogIn = analogIn;
		pastButtonIn = buttonIn;

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
class PerformanceMode {
public:
	virtual bool setup(double ms) = 0;
	virtual void render(BelaContext*) = 0;
	virtual void setParameter(size_t idx, float value) {};
};

class DirectControlMode : public PerformanceMode {
	bool setup(double ms) override
	{
		if(split)
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
		} else {
			rgb_t color = {uint8_t(255), 0, 0};
			if(!ms)
			{
				ledSlidersSetupOneSlider(
					color,
					LedSlider::AUTO_CENTROIDS
				);
				gOutMode = kOutModeFollowTouch;
			}
			gSecondTouchIsSize = true;
			return modeChangeBlink(ms, color);
		}
	}
	void render(BelaContext*) override
	{
		processLatch(split);
	}
	void setParameter(size_t idx, float value) override
	{
		switch(idx)
		{
		case 0:
			split = value;
			break;
		case 1:
			autoLatch = value;
			break;
		}
	}
private:
	bool split = false;
	bool autoLatch = false;
} gDirectControlMode;

class RecorderMode : public PerformanceMode {
	bool setup(double ms) override
	{
		gOutMode = kOutModeFollowLeds;
		if(split)
		{
			unsigned int guardPads = 1;
			if(!ms)
				ledSlidersSetupTwoSliders(guardPads, colors, LedSlider::MANUAL_CENTROIDS);
			return modeChangeBlinkSplit(ms, colors, kNumLeds / 2 - guardPads, kNumLeds / 2);
		}
		else
		{
			if(!ms)
				ledSlidersSetupOneSlider(colors[0], LedSlider::MANUAL_CENTROIDS);
			gSecondTouchIsSize = true;
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
private:
	bool split = false;
	bool retrigger = true;
	unsigned int inputMode = 0;
	rgb_t colors[2] = {
			{128, 128, 128},
			{128, 128, 64},
	};
} gRecorderMode;

class ScaleMeterMode : public PerformanceMode {
public:
	bool setup(double ms) override
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
	void render(BelaContext*) override
	{
		float in = tri.analogRead();
		LedSlider::centroid_t centroids[1];
		centroids[0].location = in;
		centroids[0].size = kFixedCentroidSize;
		ledSliders.sliders[0].setLedsCentroids(centroids, 1);
	}
private:
	unsigned int outputMode = 0;
	float envelopeDetectorCutoff = 200;
} gScaleMeterMode;

class BalancedOscsMode : public PerformanceMode {
public:
	bool setup(double ms) override
	{
		gBalancedLfoColors = gBalancedLfoColorsInit; //restore default in case it got changed via MIDI
		bool triangle = waveform; // TODO: fix waveforms
		if(!ms)
		{
			for(auto& o : oscillators)
				o.setup(1, triangle ? Oscillator::triangle : Oscillator::square); // Fs set to 1, so we need to pass normalised frequency later

			ledSlidersSetupOneSlider(
				{0, 0, 0}, // dummy
				LedSlider::MANUAL_CENTROIDS
			);
		}
		gOutMode = kOutModeManual;
		unsigned int split = triangle ? kNumLeds * 0.66 : kNumLeds * 0.33;
		return modeChangeBlinkSplit(ms, gBalancedLfoColors.data(), split, split);
	}
	void render(BelaContext* context) override
	{
		float midFreq = context->analogFrames / gClockPeriod; // we process once per block; the oscillator thinks Fs = 1
		float touchPosition = ledSliders.sliders[0].compoundTouchLocation();

		if (touchPosition > 0.0) {
			gDivisionPoint = touchPosition;
		}

		// limit max brightness. On the one hand, it reduces power consumption,
		// on the other hand it attempts to avoid the upper range, where it becomes hard
		// to discern increased brightness.
		const float kMaxBrightness = 0.4f;
		std::array<float,oscillators.size()> freqs = {
				(0.92f - gDivisionPoint) * midFreq * 2.f,
				gDivisionPoint * midFreq * 2,
		};
		for(size_t n = 0; n < oscillators.size(); ++n) {
			float out = oscillators[n].process(freqs[n]);
			gManualAnOut[!n] = map(out, -1, 1, 0, 1); // The ! is so that the CV outs order matches the display. TODO: tidy up
			float brightness = map(out, -1, 1, 0, kMaxBrightness);
			unsigned int split = gDivisionPoint > 0 ? kNumLeds * gDivisionPoint : 0;
			unsigned int start = (0 == n) ? 0 : split;
			unsigned int stop = (0 == n) ? split : kNumLeds;
			rgb_t color = {uint8_t(brightness * gBalancedLfoColors[n].r), uint8_t(brightness * gBalancedLfoColors[n].g), uint8_t(brightness * gBalancedLfoColors[n].b)};
			for(unsigned int p = start; p <  stop; ++p)
				np.setPixelColor(p, color.r, color.g, color.b);
		}
	}
private:
	unsigned int waveform = 0;
	float masterClock = 5;
	unsigned int inputMode = 0;
} gBalancedOscsMode;

class ExprButtonsMode : public PerformanceMode
{
	bool setup(double ms)
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
	void render(BelaContext*)
	{
		float scale = 0.1;
		ledSlidersExpButtonsProcess(ledSliders, gManualAnOut, scale, offsets);
	}
private:
	std::array<float,5> offsets = {
		0.5,
		0.6,
		0.7,
		0.8,
		0.9,
	};
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

float getGnd(){
	return gCalibrationProcedure.getGnd();
}

class MenuItemType
{
public:
	MenuItemType(rgb_t baseColor) : baseColor(baseColor) {}
	virtual void process(LedSlider& slider) = 0;
	rgb_t baseColor;
};

class MenuItemTypeTransition : public MenuItemType
{
public:
	MenuItemTypeTransition(const char* name, rgb_t baseColor) :
		MenuItemType(baseColor), name(name) {}
	virtual void process(LedSlider& slider) override
	{
		bool state = slider.getNumTouches();
		if(state != pastState)
			transition(state && !pastState);
		if(state != pastState)
		{
			LedSlider::centroid_t centroid;
			centroid.location = 0.5;
			// dimmed for "inactive"
			// full brightness for "active"
			centroid.size = state ? 1 : 0.1;
			slider.setLedsCentroids(&centroid, 1);
		}
		pastState = state;
	}
protected:
	virtual void transition(bool) = 0;
	char const* name;
	bool pastState = false;
};

class MenuItemTypeTransitionOrHold: public MenuItemTypeTransition
{
public:
	MenuItemTypeTransitionOrHold(const char* name, rgb_t baseColor, uint32_t holdTime) :
		MenuItemTypeTransition(name, baseColor), holdTime(holdTime) {}
	void process(LedSlider& slider) override
	{
		MenuItemTypeTransition::process(slider);
		if(!holdNotified && HAL_GetTick() - lastTransition > holdTime)
		{
			holdNotified = true;
			event(pastState ? kHoldHigh : kHoldLow);
		}
	}
private:
	void transition(bool rising) {
		lastTransition = HAL_GetTick();
		event(rising ? kTransitionRising : kTransitionFalling);
		holdNotified = false;
	}
protected:
	typedef enum {kTransitionFalling, kTransitionRising, kHoldLow, kHoldHigh} Event;
	virtual void event(Event) = 0;
	uint32_t lastTransition;
	uint32_t holdTime;
	bool holdNotified = true; // avoid notifying on startup
};

class MenuItemTypeDiscreteContinuous : public MenuItemTypeTransitionOrHold
{
public:
	MenuItemTypeDiscreteContinuous(const char* name, rgb_t baseColor, uint32_t holdTime, unsigned int& value, unsigned int numValues):
		MenuItemTypeTransitionOrHold(name, baseColor, holdTime), value(value), numValues(numValues) {}
	void event(Event e) override
	{
		switch (e)
		{
		case kTransitionRising:
			value++;
			if(value >= numValues)
				value = 0;
			break;
		case kHoldHigh:
			break;
		default:
			break;
		}
	}
	unsigned int& value;
	unsigned int numValues;
};

class MenuItemTypeDiscrete : public MenuItemTypeTransition
{
public:
	MenuItemTypeDiscrete(const char* name, rgb_t baseColor, unsigned int& value, unsigned int numValues) :
		MenuItemTypeTransition(name, baseColor), value(value), numValues(numValues) {}
private:
	void transition(bool rising)
	{
		if(rising)
		{
			++value;
			if(value >= numValues)
				value = 0;
			printf("Setting %s to %d\n\r", name, value);
		}
	}
	unsigned int& value;
	unsigned int numValues;
};

static int shouldChangeMode = 1;
class MenuItemTypeNextMode : public MenuItemTypeTransition
{
public:
	MenuItemTypeNextMode(const char* name, rgb_t baseColor) :
		MenuItemTypeTransition(name, baseColor) {}
private:
	void transition(bool rising)
	{
		if(rising)
			shouldChangeMode = 1;
	}
};

class MenuPage { //todo: make non-copyable
public:
	const char* name;
	std::vector<MenuItemType*> items;
	MenuPage(MenuPage&&) = delete;
	MenuPage(const char* name, const std::vector<MenuItemType*>& items = {}): name(name), items(items) {}
	MenuPage() {}
};

int menu_setup(double);
static void menu_in(MenuPage& menu);
static void menu_up();
static MenuPage* activeMenu;

class MenuItemTypeEnterSubmenu : public MenuItemTypeTransitionOrHold
{
public:
	MenuItemTypeEnterSubmenu(const char* name, rgb_t baseColor, uint32_t holdTime, MenuPage& submenu) :
		MenuItemTypeTransitionOrHold(name, baseColor, holdTime), submenu(submenu) {}
private:
	void event(Event e)
	{
		if(kHoldHigh == e) {
			menu_in(submenu);
		}
	}
	MenuPage& submenu;
};

class MenuItemTypeSlider : public MenuItemType {
public:
	MenuItemTypeSlider(): MenuItemType({0, 0, 0}) {}
	MenuItemTypeSlider(const rgb_t& color, float* value) :
		MenuItemType(color), value(value) {}
	void process(LedSlider& slider) override
	{
		if(value)
			*value = slider.compoundTouchLocation();
	}
	float* value;
};

MenuPage mainMenu("main");
MenuPage globalSettingsMenu("global settings");

static MenuItemTypeSlider singleSliderMenuItem;
// this is a submenu consisting of a continuous slider(no buttons). Before entering it,
// appropriately set the properties of singleSliderMenuItem
MenuPage singleSliderMenu("single slider", {&singleSliderMenuItem});

// If held-press, get into singleSliderMenu to set value
class MenuItemTypeEnterContinuous : public MenuItemTypeEnterSubmenu
{
public:
	MenuItemTypeEnterContinuous(const char* name, rgb_t baseColor, float& value) :
		MenuItemTypeEnterSubmenu(name, baseColor, 1000, singleSliderMenu), value(value) {}
	void event(Event e)
	{
		if(kHoldHigh == e) {
			singleSliderMenuItem = MenuItemTypeSlider(baseColor, &value);
			menu_in(singleSliderMenu);
		}
	}
	float& value;
};

class MenuItemTypeExitSubmenu : public MenuItemTypeTransition
{
public:
	MenuItemTypeExitSubmenu(const char* name, rgb_t baseColor) :
		MenuItemTypeTransition(name, baseColor) {}
private:
	void transition(bool rising)
	{
		if(rising) {
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

unsigned int gDummies[5];
MenuItemTypeDiscrete zero("0", {0, 0, 255}, gDummies[0], 3);
MenuItemTypeDiscrete one("1", {0, 0, 255}, gDummies[0], 3);
MenuItemTypeDiscrete two("2", {0, 0, 255}, gDummies[0], 3);
MenuItemTypeDiscrete three("3", {0, 0, 255}, gDummies[0], 3);
MenuItemTypeNextMode nextMode("4", {0, 255, 0});
MenuItemTypeDiscreteContinuous disCon("discon", {255, 0, 0}, 2000, gDummies[0], 3);
MenuItemTypeExitSubmenu exitMe("exit", {127, 255, 0});
static MenuItemTypeDisabled disabled;

static MenuItemTypeEnterSubmenu enterGlobalSettings("GlobalSettings", {120, 120, 0}, 20, globalSettingsMenu);
float myDummyFloat;
static MenuItemTypeEnterContinuous enterContinuousMode("Enter continuous mode", {255, 0, 0}, myDummyFloat);

static bool isCalibration;
static bool menuJustEntered;
int menuShouldChangeMode()
{
	int tmp = shouldChangeMode;
	shouldChangeMode = 0;
	return tmp;
}

static std::vector<MenuPage*> menuStack;

static void menu_update()
{
	// these vectors should really be initialised at startup but they have circular dependencies
	static bool inited = false;
	if(!inited)
	{
		inited = true;
		mainMenu.items = {
			&enterGlobalSettings,
			&one,
			&two,
			&disabled,
			&nextMode,
		};
		globalSettingsMenu.items = {
			&exitMe,
			&disabled,
			&one,
			&two,
			&enterContinuousMode,
		};
		singleSliderMenu.items = {
			&singleSliderMenuItem,
		};
	}
	MenuPage* newMenu = menuStack.size() ? menuStack.back() : nullptr;
	if(newMenu && activeMenu != newMenu)
	{
		activeMenu = newMenu;
		printf("menu_update: %s\n\r", newMenu ? newMenu->name : "___");
		// clear display
		np.clear();
		// TODO: the below is not particularly elegant: add a parameter to MenuPage
		if(activeMenu->items.size() == 5)
		{
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
		} else if (1 == activeMenu->items.size()){
			ledSlidersSetupMultiSlider(
				ledSlidersAlt,
				{
					activeMenu->items[0]->baseColor,
				},
				LedSlider::AUTO_CENTROIDS,
				true
			);
			menuJustEntered = false; // this is immediately interactive
		} else {
			printf("Can't handle MenuPage\n\r");
		}
		isCalibration = false;
	}
}

void menu_exit()
{
	menuStack.resize(0);
	activeMenu = nullptr;
	printf("menu_exit\n\r");
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

static bool menuDiIn0Last;
int menu_setup(double)
{
	menuStack.resize(0);
	menu_in(mainMenu);
	menu_update(); // TODO: is this needed?
	menuDiIn0Last = tri.digitalRead(0);
	return true;
}

void menu_render(BelaContext*)
{
	// if button pressed, go back up
	bool diIn0 = tri.digitalRead(0);
	if (!diIn0 && diIn0 != menuDiIn0Last){
		// button onset
		printf("button when stack is %d\n\r", menuStack.size());
		menu_up();
	}
	menuDiIn0Last = diIn0;

	menu_update();
	if(!activeMenu)
		return;
	// process these regardless to ensure the display is updated
	// (i.e.: if menuJustEntered, this will make sure the buttons are shown)
	ledSlidersAlt.process(trill.rawData.data());
	// update touches
	if(menuJustEntered)
	{
		globalSlider.process(trill.rawData.data());
		// if we just entered the menu, ensure we have removed
		// all fingers once before enabling interaction
		if(globalSlider.getNumTouches())
			return;
		menuJustEntered = false;
	}
	static const size_t numButtons = ledSlidersAlt.sliders.size();
	static std::vector<bool> altStates(numButtons);
	static std::vector<size_t> onsets(numButtons);
	static std::vector<size_t> offsets(numButtons);
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

//		ledSlidersFixedButtonsProcess(ledSlidersAlt, altStates, onsets, offsets, false);
		// see if a button was pressed
//		if(onsets.size())
//		{
//			// only consider one touch
//			const unsigned int button = onsets[0];
////				gCalibrationProcedure.setup();
////				isCalibration = true;
//			if(numButtons - 1 == button)
//				shouldChangeMode = 1;
//		}
	}
}
