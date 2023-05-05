#include "TrillRackInterface.h"
#include "GlissModes.h"
#include <vector>
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include <libraries/Oscillator/Oscillator.h>
#include "LedSliders.h"
#include "preset.h"
#include "packed.h"
typedef LedSlider::centroid_t centroid_t;
static constexpr size_t kNumSplits = 2;

#include <cmath>
size_t msToNumBlocks(BelaContext* context, float ms)
{
	if(ms < 0)
		ms = 0;
	return std::round(context->analogSampleRate * ms / 1000.f / context->analogFrames);
}

static ButtonView ButtonViewSimplify(const ButtonView& in)
{
	ButtonView btn = in;
	if(btn.tripleClick)
	{
		btn.doubleClick = false;
		btn.onset = false;
	}
	if(btn.doubleClick)
		btn.onset = false;
	return btn;
}

class TouchTracker
{
public:
	typedef uint32_t Id;
	static constexpr Id kIdInvalid = -1;
	typedef CentroidDetection::DATA_T Position;
	struct TouchWithId
	{
		centroid_t touch;
		Position startLocation;
		Id id;
	};
private:
	static_assert(std::is_signed<Position>::value); // if not signed, distance computation below may get fuzzy
	static_assert(std::is_same<decltype(centroid_t::location),Position>::value);
	static constexpr size_t kMaxTouches = 5;
	size_t numTouches = 0;
	size_t newId = 0;
	Position maxTrackingDistance = 0.2;
	std::array<unsigned int,kMaxTouches> sortedTouchIndices {};
	std::array<unsigned int,kMaxTouches> sortedTouchIds {};
	std::array<TouchWithId,kMaxTouches> sortedTouches {};
	static constexpr TouchWithId kInvalidTouch = {
		.id = kIdInvalid,
	};
	size_t getTouchOrderById(const Id id)
	{
		for(size_t n = 0; n < sortedTouches.size() && n < numTouches; ++n)
			if(id == sortedTouches[n].id)
				return n;
		return numTouches;
	}
	// these are stored only so that we can detect frame changes.
	// TODO: if there is a guarantee process() is called only on new frames,
	// you can save memory by making these local variables there.
	std::array<centroid_t,kMaxTouches> touches;
public:
	void setMaxTrackingDistance(Position d)
	{
		maxTrackingDistance = d;
	}
	void process(CentroidDetection& slider) {
		// cache previous readings
		std::array<TouchWithId,kMaxTouches> prevSortedTouches = sortedTouches;
		size_t prevNumTouches = numTouches;
		numTouches = slider.getNumTouches();
		bool changed = (numTouches != prevNumTouches);
		for(size_t n = 0; n < numTouches; ++n)
		{
			centroid_t newTouch = centroid_t{ .location = slider.touchLocation(n), .size = slider.touchSize(n) };
			changed |= memcmp(&newTouch, &touches[n], sizeof(newTouch));
			touches[n] = newTouch;
		}
		if(!changed)
		{
			// if we are a repetition of the previous frame, no need to process anything
			// NOTE: right now we are already avoiding calling on duplicate frames,
			// so we will return early only if two successive, distinct frames happen
			// to be _exactly_ the same
			return;
		}

		Id firstNewId = newId;
		constexpr size_t kMaxPermutations = kMaxTouches * (kMaxTouches - 1);
		Position distances[kMaxPermutations];
		size_t permCodes[kMaxPermutations];
		// calculate all distance permutations between previous and current touches
		for(size_t i = 0; i < numTouches; ++i)
		{
			for(size_t p = 0; p < prevNumTouches; ++p)
			{
				size_t index = i * prevNumTouches + p;	// permutation code [says between which touches we are calculating distance]
				distances[index] = std::abs(touches[i].location - prevSortedTouches[p].touch.location);
				permCodes[index] = index;
				// sort permCodes and distances by distances from min to max
				while(index && (distances[index] < distances[index - 1]))
				{
					std::swap(permCodes[index], permCodes[index - 1]);
					std::swap(distances[index], distances[index - 1]);
					index--;
				}
			}
		}

		size_t sorted = 0;
		bool currAssigned[kMaxTouches] = {false};
		bool prevAssigned[kMaxTouches] = {false};

		// track touches assigning index according to shortest distance
		for(size_t i = 0; i < numTouches * prevNumTouches; ++i)
		{
			size_t currentIndex = permCodes[i] / prevNumTouches;
			size_t prevIndex = permCodes[i] % prevNumTouches;
			if(distances[i] > maxTrackingDistance)
			{
				// if distance is too large, it must be a new touch
				// TODO: this heuristic could be improved, e.g.: by tracking
				// and considering past velocity
				continue;
			}
			// avoid double assignment
			if(!currAssigned[currentIndex] && !prevAssigned[prevIndex])
			{
				currAssigned[currentIndex] = true;
				prevAssigned[prevIndex] = true;
				sortedTouchIndices[currentIndex] = prevIndex;
				sortedTouchIds[currentIndex] = prevSortedTouches[prevIndex].id;
				sorted++;
			}
		}
		// assign a free index to new touches
		for(size_t i = 0; i < numTouches; i++)
		{
			if(!currAssigned[i])
			{
				sortedTouchIndices[i] = sorted++; // assign next free index
				sortedTouchIds[i] = newId++;
			}
		}
		// if some touches have disappeared...
		// ...we have to shift all indices...
		for(size_t i = prevNumTouches - 1; i != (size_t)-1; --i) // things you do to avoid warnings ...
		{
			if(!prevAssigned[i])
			{
				for(size_t j = 0; j < numTouches; ++j)
				{
					// ...only if touches that disappeared were before the current one
					if(sortedTouchIndices[j] > i)
						sortedTouchIndices[j]--;
				}
			}
		}

		// done! now update
		for(size_t i = 0; i < numTouches; ++i)
		{
			// update tracked value
			size_t idx = sortedTouchIndices[i];

			const Id id = sortedTouchIds[i];
			Position startLocation = -1;
			if(id >= firstNewId)
				startLocation = touches[i].location;
			else {
				// find it in the prev arrays
				// TODO: cache this value earlier so we can be faster here
				for(size_t n = 0; n < prevNumTouches; ++n)
				{
					if(id == prevSortedTouches[n].id)
					{
						startLocation = prevSortedTouches[n].startLocation;
						break;
					}
				}
			}
			assert(-1 != startLocation);
			sortedTouches[idx] = TouchWithId {
				.touch = touches[i],
				.startLocation = startLocation,
				.id = id,
			};
		}
		// empty remaining touches. Not that they should ever be accessed...
		for(size_t i = numTouches; i < sortedTouches.size(); ++i)
			sortedTouches[i].id = kIdInvalid;
#if 0
		for(size_t n = 0; n < numTouches; ++n)
		{
			auto& t = getTouchOrdered(n);
			printf("[%u]%lu %.2f %.1f ", n, t.id, t.touch.location, t.startLocation);
		}
		if(numTouches)
			printf("\n\r");
#endif
	}
	size_t getNumTouches()
	{
		return numTouches;
	}
	const TouchWithId& getTouchById(const Id id)
	{
		size_t n = getTouchOrderById(id);
		if(n >= numTouches)
			return kInvalidTouch;
		else
			return sortedTouches[n];
	}
	void setStartLocationById(const Id id, Position newLocation)
	{
		size_t n = getTouchOrderById(id);
		if(n < numTouches)
			sortedTouches[n].startLocation = newLocation;
	}
	// the last is the most recent
	const TouchWithId& getTouchOrdered(size_t n)
	{
		return sortedTouches[n];
	}
	const TouchWithId& getTouchMostRecent()
	{
		return sortedTouches[numTouches - 1];
	}
	const TouchWithId& getTouchOldest()
	{
		return sortedTouches[0];
	}
} gTouchTracker;
static_assert(kNumOutChannels >= 2); // too many things to list depend on this in this file.

//#define TRIGGER_IN_TO_CLOCK_USES_MOVING_AVERAGE

// pick one of the two for more or less debugging printf and memory usage
//#define S(a) a
#define S(a)

extern int gAlt;
extern TrillRackInterface tri;
extern const unsigned int kNumLeds;
extern std::vector<unsigned int> padsToOrderMap;
extern NeoPixelT<kNumLeds> np;
extern Trill trill;
extern std::array<float,kNumOutChannels> gManualAnOut;

std::array<Oscillator, 2> oscillators;
const std::array<rgb_t, 2> gBalancedLfoColorsInit = {{{0, 0, 255}, {0, 255, 0}}};
std::array<rgb_t, 2> gBalancedLfoColors; // copy so that we can set them via MIDI without changing defaults

std::array<OutMode,kNumOutChannels> gOutMode { kOutModeManualBlock, kOutModeManualBlock };
int gCounter = 0;
int gSubMode = 0;
std::array<bool,2> gOutIsSize;
bool gJacksOnTop = false;
Override gOverride;
static bool gInUsesCalibration;
static bool gOutUsesCalibration;
static bool gInUsesRange;
static std::array<bool,kNumOutChannels> gOutUsesRange;

// Recording the gesture
enum { kMaxRecordLength = 5000 };
const float kSizeScale = 10000; // value used internally for rescaling the slider
static float gSizeScale = kSizeScale; // current, active value. Gets overriden upon loading from preset
const float kFixedCentroidSize = 0.3;

LedSliders ledSliders;
LedSliders ledSlidersAlt;
ButtonView menuBtn;
ButtonView performanceBtn;

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

uint32_t gClockPeriodUpdateCounter = 0;
float gClockPeriod = 10000; // arbitrary init to avoid divisions by zero. TODO: instead check before using it
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
				gClockPeriodUpdateCounter++;
#endif // TRIGGER_IN_TO_CLOCK_USES_MOVING_AVERAGE
			}
			lastTrig = newTrig;
			lastTrigPrimed = true;
		}

		lastIn = in;
	}
}

typedef enum {
	kBottomUp,
	kTopBottom,
} LedSlidersOrder;
static void ledSlidersSetupMultiSlider(LedSliders& ls, std::vector<rgb_t> const& colors, const LedSlider::LedMode_t& mode, bool setInitial, size_t maxNumCentroids, LedSlidersOrder order = kBottomUp)
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
		size_t i;
		switch (order)
		{
		default:
		case kBottomUp:
			i = n;
			break;
		case kTopBottom:
			i = numSplits - 1 - n;
			break;
		}
		size_t firstPad = i * (activePads + guardPads);
		size_t firstLed = i * (activeLeds + guardLeds);
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
			.sizeScale = gSizeScale,
			.boundaries = boundaries,
			.maxNumCentroids = {maxNumCentroids},
			.np = &np,
			.min = kSliderBottomMargin,
			.max = 1.f - kSliderTopMargin,
	};
	ls.setup(settings);
	assert(numSplits == ls.sliders.size());

	for(size_t n = 0; n < numSplits; ++n)
	{
		ls.sliders[n].setColor(colors[n]);
		ls.sliders[n].setLedMode(mode);
		if(setInitial)
		{
			centroid_t centroid;
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
		centroid_t centroid;
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
				centroid_t centroid;
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
	ledSlidersSetupMultiSlider(ledSliders, {color}, mode, false, 1);
}

static void ledSlidersSetupTwoSliders(unsigned int guardPads, rgb_t colors[2], LedSlider::LedMode_t mode)
{
	ledSlidersSetupMultiSlider(ledSliders, {colors[0], colors[1]}, mode, false, 1, kTopBottom);
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
			np.setPixelColor(kNumLeds - 1 - n, colors[0].r, colors[0].g, colors[0].b);
		for(unsigned int n = startSecond; n < kNumLeds; ++n)
			np.setPixelColor(kNumLeds - 1 - n, colors[1].r, colors[1].g, colors[1].b);
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
		true,
		1
	);
	return true;
}

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
		idx = 0;
		validFrames = 0;
		pastInputFrame = {0, 0};
		lastOutSize = 0;
	}
	// return: may modify frame and latchStarts
	void process(centroid_t& frame, bool& latchStarts)
	{
		// filter out duplicate frames
		// TODO: call this per each new frame instead
		if(validFrames && pastInputFrame == frame)
		{
			frame.size = lastOutSize;
			return;
		}
		// cache for duplicate detection
		pastInputFrame = frame;
		if(validFrames && pastFrames[getPastFrame(0)].size && !frame.size) // if size went to zero
		{
			// use the oldest frame we have for location, keeping whatever size we have been using
			if(validFrames) {
				frame = centroid_t {
						.location = pastFrames[getOldestFrame()].location,
						.size = lastOutSize,
				};
			} else {
				// nothing: we have nothing to latch onto, so we do not alter frame
			}
			latchStarts = true;
			pastFrames[idx].location = pastFrames[idx].size = 0;
			validFrames = 0;
		} else {
			// if we are still touching
			// apply a variable delay to the output size
			if(validFrames < kMaxDelay)
			{
				// we are just at the beginning of a touch: get the most recent size
				delay = 0;
			} else if (delay < kMaxDelay)
			{
				// the touch has been going on for a while, we need to progressively
				// reach the max delay
				delay++;
			}
			float newSize = frame.size; // no delay!
			if(delay > 0)
			{
				// validFrames is always larger than delay, so we always
				// get a valid inde
				ssize_t n = getPastFrame(delay - 1);
				assert(n >= 0);
				newSize = pastFrames[n].size;
			}

			// store current input value for later
			pastFrames[idx] = frame;
			++idx;
			if(idx >= pastFrames.size())
				idx = 0;
			validFrames++;

			// output the delayed size
			frame.size = newSize;
			lastOutSize = frame.size;
		}
	}
private:
	ssize_t getOldestFrame()
	{
		return getPastFrame(kHistoryLength - 1);
	}
	// back = 0 : newest frame
	// back = kHistoryLength - 1 : oldest frame
	ssize_t getPastFrame(size_t back)
	{
		if(!validFrames)
			return -1;
		size_t lastGood;
		if(back > validFrames)
			back = validFrames;
		// all values in the circular buffer are valid. Get the oldest
		if(back >= kHistoryLength)
			back = kHistoryLength - 1;
		// go back in the circular buffer to the oldest valid value.
		lastGood = (idx - 1 - back + kHistoryLength) % kHistoryLength;
		return lastGood;
	}
	static constexpr size_t kHistoryLength = 5;
	static constexpr size_t kMaxDelay = kHistoryLength - 1; // could be even less than this, if desired
	std::array<centroid_t,kHistoryLength> pastFrames;
	size_t idx;
	size_t validFrames;
	size_t delay;
	centroid_t pastInputFrame;
	float lastOutSize;
};

class LatchProcessor {
	static constexpr size_t kMaxNumValues = 2;
public:
	enum Reason {
		kLatchNone,
		kLatchAuto,
		kLatchManual,
	};
	LatchProcessor() {
		reset();
	}
	void reset()
	{
		isLatched = {kLatchNone, kLatchNone};
		unlatchArmed = {false, false};
		latchedValues = {0, 0};
		for(auto& al : autoLatchers)
			al.reset();
	}
	void process(bool autoLatch, size_t numValues, std::array<centroid_t,kMaxNumValues>& values, std::array<Reason,kMaxNumValues>& isLatchedRet,
			bool shouldLatch = false, bool shouldUnlatch = false)
	{
		if(numValues > kMaxNumValues)
			numValues = kMaxNumValues;
		std::array<bool,kMaxNumValues> hasTouch = { values[0].size > 0, values[1].size > 0 };
		std::array<Reason,kMaxNumValues> latchStarts = {kLatchNone, kLatchNone};
		std::array<bool,kMaxNumValues> unlatchStarts = {false, false};

		// button latches everything if there is at least one touch
		// and unlatches everything if there is no touch
		if(shouldLatch)
		{
			for(size_t n = 0; n < numValues; ++n)
			{
				if(!isLatched[n] && hasTouch[n])
					latchStarts[n] = kLatchManual;
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
			{
				if(!isLatched[n])
				{
					bool autoLatchStarts = false;
					autoLatchers[n].process(values[n], autoLatchStarts);
					if(autoLatchStarts && kLatchNone == latchStarts[n])
						latchStarts[n] = kLatchAuto;
				}
			}
		}
		for(size_t n = 0; n < numValues; ++n)
		{
			if(isLatched[n])
			{
				// if it's latched (and you have released your finger),
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
				isLatched[n] = latchStarts[n];
				unlatchArmed[n] = false;
			}
			if(unlatchStarts[n])
			{
				isLatched[n] = kLatchNone;
				autoLatchers[n].reset(); // ensure we don't get a spurious auto latch next frame
			}
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
	std::array<Reason,kMaxNumValues> isLatched;
	std::array<bool,kMaxNumValues> unlatchArmed;
	std::array<centroid_t,kMaxNumValues> latchedValues;
	std::array<AutoLatcher,kMaxNumValues> autoLatchers;
};

template <typename sample_t>
class Recorder
{
public:
	struct ValidSample {
		sample_t sample;
		bool valid;
	};
#if 0
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
#endif
	virtual void startRecording()
	{
		active = true;
		resize(0);
		full = false;
	}
	ValidSample record(const sample_t& in)
	{
		data[current] = in;
		sample_t& ret = data[current];
		increment(current);
		// if the circular buffer becomes full, make a note of it
		if(current == start)
			full = true;
		end = current;
		return {ret, true};
	}
	virtual void stopRecording()
	{
		if(full)
		{
			// if the circular buffer became full, adjust start
			// so that we use all data in the buffer
			start = end;
			increment(start);
		}
	}
	void resize(size_t newEnd)
	{
		newEnd = std::min(newEnd, data.size());
		current = end = newEnd;
		full = (newEnd == data.size());
	}
#if 0
	ValidSample play(bool loop)
	{
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
			return {0, false};
		auto& ret = data[current];
		increment(current);
		return {ret, true};
	}
#endif
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

	template<typename T> void decrement(T& idx)
	{
		if(0 == idx)
			idx = data.size() - 1;
		else
			--idx;
	}

	std::array<sample_t, kMaxRecordLength> data;
	size_t start = 0;
	size_t end = 0;
	size_t current = 0;
	bool active = false;
	bool full = false; // whether during recording the buffer becomes full
};

#if 0
template <typename sample_t>
class TimestampedRecorder : public Recorder<uint32_t>{
	// this is just a placeholder for the specialised implementation below
};

template<>
class TimestampedRecorder<float> : public Recorder<uint32_t>
{
private:
	using sample_t = float; // if using the actual template, remove this
	typedef Recorder<uint32_t> Base;
	enum { kRepsBits = 10, kSampleBits = 22 };
	enum { max = 1 }; // TODO: not sure what this was introduced for
public:
	struct timedData_t
	{
		uint32_t reps : kRepsBits;
		uint32_t sample : kSampleBits;
		bool valid = false;
	};
	struct sampleData_t{
		sample_t value;
		bool valid;
	};
	// sanity checks to ensure the uint32_t is enough
	static_assert(kRepsBits + kSampleBits <= sizeof(ValidSample::sample) * 8); // if you change field values to be larger than 4 bytes, be well aware that it can't fit in a uint32_t
	static_assert(std::is_same<decltype(timedData_t::reps), decltype(ValidSample::sample)>::value);
	static_assert(std::is_same<decltype(timedData_t::sample), decltype(ValidSample::sample)>::value);
	static uint32_t inToSample(const sample_t& in)
	{
		uint32_t r = in / max * kSampleMax + 0.5f;
		return r;
	}
	static sample_t sampleToOut(uint32_t sample)
	{
		return sample * max / sample_t(kSampleMax);
	}
	static struct timedData_t recordToTimedData(const ValidSample& d)
	{
		timedData_t ret;
		ret.reps = (d.sample & ((1 << kRepsBits) - 1));
		ret.sample = (d.sample >> kRepsBits);
		ret.valid = d.valid;
		return ret;
	}
	static struct timedData_t recordToTimedData(uint32_t sample)
	{
		return recordToTimedData({
			.sample = sample,
			.valid = true,
		});
	}
	static uint32_t timedDataToRecord(const struct timedData_t t)
	{
		return *(uint32_t*)&t;
	}
	void startRecording() override
	{
		firstSample = true;
		reps = 0;
		oldSample = -1;
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
		if(!firstSample)
		{
			// flush whatever we haven't recorded yet
			pushSample();
		}
		Base::stopRecording();
		playData.reps = 0;
	}

	void replaceLastFrames(unsigned int howMany)
	{
		// find frame containing the last valid value
		size_t idx = end;
		size_t reps = 0;
		sample_t refSample;
		while(1)
		{
			timedData_t d = recordToTimedData(data[idx]);
			reps += d.reps;
			if(reps >= howMany || idx == start)
			{
				// last good value:
				refSample = d.sample;
				break;
			}
			decrement(idx);
		}
		// fill in later frames with this value
		while(1)
		{
			// get the timedData, leave the same reps
			timedData_t d = recordToTimedData(data[idx]);
			// replace value
			d.sample = refSample;
			// store back
			uint32_t r = timedDataToRecord(d);
			data[idx] = r;
			if(idx == end)
				break;
			increment(idx);
		}
	}

	void restart()
	{
		playData.reps = 0;
		Recorder::restart();
	}

	sampleData_t play(bool loop)
	{
		if(playData.valid && playData.reps)
			--playData.reps;
		else {
			playData = recordToTimedData(Base::play(loop));
		}
		return { sampleToOut(playData.sample), playData.valid };
	}
	void printData()
	{
		for(unsigned int n = start; n < data.size() + end; ++n)
		{
			unsigned int idx = n % data.size();
			timedData_t d = recordToTimedData({data[n], true});
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
#endif

class GestureRecorder
{
public:
	typedef uint32_t FrameId;
	typedef float sample_t;
	typedef Recorder<sample_t>::ValidSample HalfGesture_t;
	struct Gesture_t {
		HalfGesture_t first;
		HalfGesture_t second;
		HalfGesture_t& operator[](size_t n)
		{
			return (0 == n) ? first : second;
		}
		size_t size() const {
			return 2;
		}
	};
	void startRecording(size_t n)
	{
		if(n < kNumRecs)
		{
			rs[n].r.startRecording();
			rs[n].state = kRecJustStarted;
			rs[n].activity = 0;
			rs[n].frozen = false;
		}
	}
	void stopRecording(size_t n, bool optimizeForLoop)
	{
		if(n < kNumRecs)
		{
			rs[n].r.stopRecording();
			rs[n].state = kPlayJustStarted;
			// compute how many unique frames on average per call to process.
			// Use that as an increment to playback
			rs[n].playbackInc = rs[n].r.size() / double(rs[n].recCounter);
			// printf("unique frames: %lu %lu, recCount: %lu average inc: %.5f\n\r",
			//		1 + rs[n].lastFrameId - rs[n].firstFrameId, (uint32_t)rs[n].r.size(), rs[n].recCounter, rs[n].playbackInc);
			// set playhead at the end so we do not immediately start unless loop / triggering
			rs[n].playHead = rs[n].r.size();
//			assert((1 + lastFrameIds[n] - firstFrameIds[n]) == rs[n].size());
#if 0
			if(optimizeForLoop)
				rs[n].r.replaceLastFrames(40);
#endif
		}
	}
	HalfGesture_t process(size_t n, float touch, const FrameId frameId, bool loop, bool retriggerNow, ssize_t autoFreezeAt)
	{
		if(n >= kNumRecs)
			return {0, false};
		HalfGesture_t out;
		switch(rs[n].state)
		{
		case kRecJustStarted:
			rs[n].firstFrameId = frameId;
			rs[n].recCounter = 0;
			rs[n].state = kRec;
			// NOBREAK
		case kRec:
			rs[n].recCounter++;
			if(frameId == rs[n].lastFrameId)
				return rs[n].lastOut;
			out = { rs[n].r.record(touch).sample, true };
			rs[n].activity |= touch > 0;
			break;
		case kPlayJustStarted:
			if(loop)
				rs[n].playHead = 0;
			rs[n].state = kPlay;
			// NOBREAK
		case kPlay:
			if(retriggerNow)
				resumePlaybackFrom(n, 0);
			if(rs[n].r.size()) {
				size_t idx = size_t(rs[n].playHead);
				if(idx < rs[n].r.size()) {
					out = {rs[n].r.getData()[idx], true};
					if(autoFreezeAt >= 0)
					{
						if(rs[n].playHead < autoFreezeAt && rs[n].playHead + rs[n].playbackInc >= autoFreezeAt)
							freeze(n); // unfrozen on resumePlaybackFrom()
					}
					if(!rs[n].frozen)
						rs[n].playHead += rs[n].playbackInc;
					if(rs[n].playHead >= rs[n].r.size() && loop)
						rs[n].playHead -= rs[n].r.size(); // loop back keeping phase offset
				}
				else
					out = {0, false};
			} else
				out = {0, false};
			break;
		}
		rs[n].lastFrameId = frameId;
		return rs[n].lastOut = out;
	}
	void freeze(size_t n)
	{
		rs[n].frozen = true;
	}
	void resumePlaybackFrom(size_t n, ssize_t from)
	{
		rs[n].frozen = false;
		if(from < 0)
			from = 0;
		rs[n].playHead = from;
	}
	void empty()
	{
		for(size_t n = 0; n < kNumRecs; ++n)
		{
			rs[n].r.startRecording();
			rs[n].r.stopRecording();
		}
	}
	bool isRecording(size_t n)
	{
		if(n < kNumRecs)
			return kRec == rs[n].state || kRecJustStarted == rs[n].state;
		return false;
	}
	void trimTo(size_t n, size_t recCount, size_t recSize)
	{
		if(n < kNumRecs)
		{
			rs[n].recCounter = recCount;
			rs[n].r.resize(recSize);
		}
	}
	static constexpr size_t kNumRecs = 4;
	enum State {
		kPlay = 0,
		kRec = 2,
		kPlayJustStarted = -1,
		kRecJustStarted = -2,
	};
	class SwappableRecorders {
		struct Rcr {
			Recorder<sample_t> r;
			State state {};
			FrameId firstFrameId {};
			FrameId lastFrameId {};
			HalfGesture_t lastOut {};
			uint32_t recCounter {};
			double playHead {};
			double playbackInc {1};
			bool activity {};
			bool frozen {};
		};
		std::array<Rcr,kNumRecs> rs;
		std::array<Rcr*,kNumRecs> ptrs;
	public:
		SwappableRecorders() {
			for(size_t n = 0; n < kNumRecs; ++n)
				ptrs[n] = &rs[n];
		}
		void swap(size_t a, size_t b) {
			std::swap(ptrs[a], ptrs[b]);
		}
		Rcr& operator[] (size_t n) {
			return *ptrs[n];
		}
		// do not implement size() to avoid confusion when
		// the user wants to call rs[n].r.size()
	};
	SwappableRecorders rs;
private:
	std::array<bool,kNumSplits> hadTouch {};
	bool lastStateChangeWasToggling = false;
} gGestureRecorder;

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
	virtual void render(BelaContext*, FrameData* frameData) = 0;
};

class ParameterEnum : public Parameter {
public:
	virtual void set(unsigned int) = 0;
	virtual void next() = 0;
	virtual uint8_t get() const = 0;
	virtual uint8_t getMax() const = 0;
};

template <uint8_t N, typename type = uint8_t>
class ParameterEnumT : public ParameterEnum
{
public:
	ParameterEnumT<N,type>(ParameterUpdateCapable* that, uint8_t value = 0):
		that(that), value(value) {}

	void set(unsigned int newValue) override
	{
		value = newValue - 1;
		next();
	}
	void next() override
	{
		value++;
		if(value >= N)
			value = 0;
		that->updated(*this);
	}
	uint8_t get() const override
	{
		return value;
	}
	uint8_t getMax() const override
	{
		return N;
	}
	operator type() { return type(value); }
private:
	ParameterUpdateCapable* that;
	uint8_t value;
};

class ParameterContinuous : public Parameter {
public:
	ParameterContinuous(ParameterUpdateCapable* that, float value = 0) : that(that), value(value) {}
	void set(float newValue)
	{
		setNoUpdate(newValue);
		that->updated(*this);
	}
	void setNoUpdate(float newValue) // only call this if you really know what you are doing
	{
		value = newValue;
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
		gOutMode.fill(kOutModeFollowLeds);
		ledSlidersSetupTwoSliders(1, colorDefs[0], LedSlider::MANUAL_CENTROIDS);
		for(unsigned int n = 0; n < kNumLeds; ++n)
			np.setPixelColor(n, 0, 0, 0);
		state = kNumStates - 1;
		nextState();
		return true;
	}
	void render(BelaContext* context, FrameData* frameData) override
	{
		bool hasTouch = (globalSlider.compoundTouchSize() > 0);
		if(performanceBtn.onset)
			nextState();

		if(hasTouch && !hadTouch)
		{
			if(shouldLeds)
			{
				// toggle leds colors
				count = 0;
				setColor();
			} else {

			}
		}
		if(hasTouch)
		{
			tri.buttonLedWrite(kGreenBtnIdx, 0);
			tri.buttonLedWrite(kRedBtnIdx, 0);
		} else {
			float gn = 0;
			float rd = 0;
			switch(buttonMode)
			{
			case kButtonCycle:
			{
				constexpr size_t kPeriod = 1000;
				bool which = ((HAL_GetTick() - startTime) % kPeriod) > kPeriod / 2;
				gn = which;
				rd = !which;
			}
				break;
			case kButtonManual:
				gn = greenButton;
				rd = redButton;
				break;
			}
			tri.buttonLedWrite(kGreenBtnIdx, gn);
			tri.buttonLedWrite(kRedBtnIdx, rd);
		}
		hadTouch = hasTouch;
		centroid_t centroids[2];
		switch(ledMode)
		{
		case kLedsOff:
			for(unsigned int n = 0; n < 2; ++n)
			{
				centroids[n].location = 0;
				centroids[n].size = 0;
			}
			break;
		case kLedsMoveOpposite:
		case kLedsMoveSame:
		{
			const uint32_t period = 1024;
			float location = 0.8 * simpleTriangle(count, period) + 0.1;
			centroids[0].location = location;
			centroids[0].size = 1;
			centroids[1].size = 1;
			if(kLedsMoveSame == ledMode)
			{
				// both move in the same direction
				centroids[1].location = location;
			} else {
				// they move in opposite directions
				centroids[1].location = 1.f - location;
			}
			count++;
			count %= period;
		}
			break;
		case kLedsStillTop:
		case kLedsStillBottom:
			for(unsigned int n = 0; n < 2; ++n)
			{
				centroids[n].location = (kLedsStillTop == ledMode ? 0.8 : 0.2);
				centroids[n].size = 1;
			}
			break;
		}
		for(unsigned int n = 0; n < 2; ++n)
			ledSliders.sliders[n].setLedsCentroids(centroids + n, 1);

		bool justStarted = (HAL_GetTick() - startTime < 500);
		if(justStarted)
		{
			// make sure when we start we are actually updating the leds
			// so they can be set to the values the mode will need them in later on
			tr_requestUpdateLeds(true);
		} else {
			tr_requestUpdateLeds(shouldPwm);
		}
		tr_requestScan(shouldTouch);
	}

	void updatePreset() override
	{}
private:
	void nextState() {
		state++;
		if(kNumStates == state)
			state = 0;
		count = 0;
		initState();
	}
	void initState()
	{
		startTime = HAL_GetTick();
		// update global states
		shouldLeds = false;
		shouldTouch = false;
		shouldPwm = false;
		ledMode = kLedsOff;
		buttonMode = kButtonManual;
		greenButton = 0;
		redButton = 0;
		switch(State(state))
		{
		case kState_Leds_Pwm_Touch: // slider LEDs moving together
			shouldLeds = true;
			shouldPwm = true;
			shouldTouch = true;
			ledMode = kLedsMoveSame;
			break;
		case kState_Leds_Pwm_x:  // slider LEDs moving opposite
			shouldLeds = true;
			shouldPwm = true;
			ledMode = kLedsMoveOpposite;
			break;
		case kState_Leds_x_Touch: // slider LEDs still at top
			shouldLeds = true;
			shouldTouch = true;
			ledMode = kLedsStillTop;
			break;
		case kState_Leds_x_x_: // slider LEDs still at bottom
			shouldLeds = true;
			ledMode = kLedsStillBottom;
			break;
		case kState_x_Pwm_Touch: // sliderLEDs off, LED button is red, when touching slider, button goes dark
			shouldPwm = true;
			shouldTouch = true;
			redButton = 1;
			ledMode = kLedsOff;
			break;
		case kState_x_Pwm_x_: // slider LEDs off, LED button cycles automatically
			shouldPwm = true;
			ledMode = kLedsOff;
			buttonMode = kButtonCycle;
			break;
		case kState_x_x_Touch: // slider LEDs off, LED button is green, when touching slider button goes dark
			shouldTouch = true;
			greenButton = 1;
			ledMode = kLedsOff;
			break;
		case kState_x_x_x: // slider LEDs off, no LED button
		case kNumStates:
			break;
		}
	}
	void setColor(int n = -1){
		if(n == -1)
			colorState++;
		else
			colorState = n;
		if(colorState >= kNumColors)
			colorState = 0;
		ledSliders.sliders[0].setColor(colorDefs[colorState][0]);
		ledSliders.sliders[1].setColor(colorDefs[colorState][1]);
		// re-init state so that if shouldPwm is off at least it gets
		// retriggered briefly and displayed color is updated
		initState();
	}
	static constexpr size_t kNumColors = 3;
	rgb_t colorDefs[kNumColors][2] = {
			{
				{192, 58, 40},
				{160, 70, 50},
			},
			{
				{192, 58, 40},
				{0, 0, 0},
			},
			{
				{255, 255, 255},
				{255, 255, 255},
			},
	};
	uint32_t count = 0;
	enum State {
		kState_Leds_Pwm_Touch,
		kState_Leds_Pwm_x,
		kState_Leds_x_Touch,
		kState_Leds_x_x_,
		kState_x_Pwm_Touch,
		kState_x_Pwm_x_,
		kState_x_x_Touch,
		kState_x_x_x,
		kNumStates,
	};
	enum LedMode {
		kLedsOff,
		kLedsMoveSame,
		kLedsMoveOpposite,
		kLedsStillTop,
		kLedsStillBottom,
	} ledMode;
	enum ButtonMode {
		kButtonManual,
		kButtonCycle,
	} buttonMode;
	unsigned int state = kState_Leds_Pwm_Touch;
	unsigned int colorState = 0;
	uint32_t startTime = 0;
	float greenButton = 0;
	float redButton = 0;
	static constexpr size_t kGreenBtnIdx = 1;
	static constexpr size_t kRedBtnIdx = 0;
	bool hadTouch = false;
	bool shouldLeds = true;
	bool shouldTouch = true;
	bool shouldPwm = true;
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

#define genericDefaulter12(CLASS,A,B,C,D,E,F,G,H,I,J,K,L) \
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
	pfd->J = that->J; \
	pfd->K = that->K; \
	pfd->L = that->L; \
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

#define genericLoadCallback12(CLASS,A,B,C,D,E,F,G,H,I,J,K,L) \
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
	that->J.set(pfd->J); that->presetFieldData.J = pfd->J; \
	that->K.set(pfd->K); that->presetFieldData.K = pfd->K; \
	that->L.set(pfd->L); that->presetFieldData.L = pfd->L; \
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

#define UPDATE_PRESET_FIELD12(A,B,C,D,E,F,G,H,I,J,K,L) \
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
	presetFieldData.J = J; \
	presetFieldData.K = K; \
	presetFieldData.L = L; \
	if(!areEqual(bak, presetFieldData)) \
		presetSetField(this, &presetFieldData); \
}

std::array<centroid_t,kNumSplits> touchTrackerSplit(CentroidDetection& slider, bool shouldProcess, bool split)
{
	std::array<centroid_t,kNumSplits> values;
	if(shouldProcess)
		gTouchTracker.process(globalSlider);
	size_t numTouches = gTouchTracker.getNumTouches();
	// per each split
	static_assert(kNumSplits == 2); // or the loop below won't work
	const float midMin = 0.45;
	const float midMax = 0.55;
	for(ssize_t s = 0; s < 1 + split; ++s)
	{
		bool found = false;
		const float min = split ? (kNumSplits - 1 - s) * midMax : 0;
		const float max = split ? min + midMin : 1;
		TouchTracker::TouchWithId twi;
		for(ssize_t i = numTouches - 1; i >= 0; --i)
		{
			// get the most recent touch which started on this split
			const TouchTracker::TouchWithId& t = gTouchTracker.getTouchOrdered(i);
			if(t.startLocation >= min && t.startLocation <= max)
			{
				found = true;
				twi = t;
				break;
			} else if(t.startLocation > midMin && t.startLocation < midMax) {
				// touch originated in the region between the splits.
				if(t.touch.location >= min && t.touch.location <= max)
				{
					// if it enters one of the splits,
					// we assign it to it for future reference
					// and immediately start using it
					gTouchTracker.setStartLocationById(t.id, t.touch.location);
					found = true;
					twi = t;
					break;
				}
			}
		}
		if(!found)
			twi.id = TouchTracker::kIdInvalid;
		if(TouchTracker::kIdInvalid == twi.id)
		{
			values[s].location = 0;
			values[s].size = 0;
		} else {
			// TODO: this used to be compoundTouch)
			values[s].location = mapAndConstrain(twi.touch.location, min, max, 0, 1);
			values[s].size = twi.touch.size;
		}
	}
	return values;
}

class SplitPerformanceMode : public PerformanceMode {
protected:
	static constexpr size_t kNumSplits = ::kNumSplits;
	size_t currentSplits()
	{
		return 1 + isSplit();
	}
	bool isSplit()
	{
		return splitMode != kModeNoSplit;
	}
	void renderOut(std::array<float,kNumSplits>& out, const std::array<centroid_t,kNumSplits>& values, const std::array<centroid_t,kNumSplits>& displayValues)
	{
		for(ssize_t n = 0; n < isSplit() + 1; ++n)
		{
			bool hasTouch = (values[n].size > 0);
			switch(splitMode)
			{
			case kModeNoSplit:
				ledSliders.sliders[n].setLedsCentroids(displayValues.data(), 1);
				out[0] = hasTouch ? values[0].location : kNoOutput;
				out[1] = hasTouch ? values[0].size : kNoOutput;
				break;
			case kModeSplitLocation:
			{
				centroid_t centroid;
				centroid.location = displayValues[n].location;
				centroid.size = hasTouch * kFixedCentroidSize;
				ledSliders.sliders[n].setLedsCentroids(&centroid, 1);
				out[n] = hasTouch ? values[n].location : kNoOutput;
			}
				break;
			case kModeSplitSize:
			{
				// Use multiple centroids to make a bigger dot.
				// Their spacing increases with the size
				std::array<centroid_t,kNumSplits> centroids;
				float value = displayValues[n].size;
				float spread = 0.15f * std::min(1.f, displayValues[n].size);
				for(size_t c = 0; c < centroids.size(); ++c)
				{
					centroids[c].location = 0.5f + (0 == c ? -spread : +spread);
					centroids[c].size = value;
				}
				ledSliders.sliders[n].setLedsCentroids(centroids.data(), centroids.size());
				out[n] = values[n].size;
			}
				break;
			}
		}
	}
	void setOutIsSize()
	{
		switch(splitMode)
		{
		case kModeNoSplit:
			gOutIsSize = {false, true};
			break;
		case kModeSplitSize:
			gOutIsSize = {true, true};
			break;
		case kModeSplitLocation:
			gOutIsSize = {false, false};
			break;
		}
	}
public:
	enum SplitMode {
		kModeNoSplit,
		kModeSplitLocation,
		kModeSplitSize,
	};
	ParameterEnumT<3> splitMode{this, false};
};

class DirectControlMode : public SplitPerformanceMode {
	enum AutoLatchMode {
		kAutoLatchOff,
		kAutoLatchBoth,
		kAutoLatchLocationOnly,
	};
public:
	bool setup(double ms) override
	{
		gOutMode.fill(kOutModeManualBlock);
		if(isSplit())
		{
			unsigned int guardPads = 1;
			if(ms <= 0)
			{
				ledSlidersSetupTwoSliders(guardPads, colors, LedSlider::MANUAL_CENTROIDS);
			}
			if(ms < 0)
				return true;
			return modeChangeBlinkSplit(ms, colors, kNumLeds / 2 - guardPads, kNumLeds / 2);
		} else {
			if(ms <= 0)
			{
				ledSlidersSetupOneSlider(
					colors[0],
					LedSlider::MANUAL_CENTROIDS
				);
			}
			if(ms < 0)
				return true;;
			return modeChangeBlink(ms, colors[0]);
		}
	}
	void render(BelaContext*, FrameData* frameData) override
	{
		setOutIsSize();
		std::array<centroid_t,kNumSplits> values = touchTrackerSplit(globalSlider, ledSliders.isTouchEnabled() && frameData->isNew, isSplit());
		bool shouldLatch = false;
		bool shouldUnlatch = false;
		if(performanceBtn.onset)
		{
			// at least one VALID and non-latched
			for(ssize_t n = 0; n < 1 + isSplit(); ++n)
			{
				bool hasTouch = values[n].size > 0;
				shouldLatch |= (!isLatched[n] && hasTouch);
			}
		}
		if(performanceBtn.offset)
		{
			// if it's not the same press that triggered the latch, unlatch
			if(lastLatchCount != performanceBtn.pressId)
			{
				shouldUnlatch = true;
				lastLatchCount = ButtonView::kPressIdInvalid;
			}
		}
		// sets values and isLatched
		latchProcessor.process(shouldAutoLatch(), 1 + isSplit(), values, isLatched, shouldLatch, shouldUnlatch);
		if(shouldLatch && (isLatched[0] || isLatched[isSplit()]))
		{
			// keep note of current press
			lastLatchCount = performanceBtn.pressId;
		}
		// make a copy before possibly removing size
		std::array<centroid_t,kNumSplits> displayValues = values;

		bool shouldOverrideOut0 = false;
		if(hasSizeOutput() && !shouldAutoLatchSize())
		{
			for(size_t n = 0; n < isLatched.size() && n < size_t(1 + isSplit()); ++n)
			{
				if(LatchProcessor::kLatchAuto == isLatched[n])
				{
					// if we were autolatched, we ned to ignore size
					// CV outs:
					// set size to 0 for output
					values[n].size = 0;
					// TODO: this also sets location to kNoOutput, so we have to "fixup" it below.
					// so we make a not of it here
					if(kModeNoSplit == splitMode)
						shouldOverrideOut0 = true;
					// display:
					if(kModeSplitSize == splitMode)
						// display is dark
						displayValues[n].size = 0;
					else
						// leave a faint dot for display while location is latched
						displayValues[n].size = 0.15;
				}
			}
		}
		renderOut(gManualAnOut, values, displayValues);
		if(shouldOverrideOut0) {
			// fixup: yet another hack to get something displayed, kNoOutput size output,
			// but valid location output. See TODO above
			gManualAnOut[0] = values[0].location;
		}
	}
	void updated(Parameter& p)
	{
		if(p.same(splitMode)) {
			printf("DirectControlMode: updated splitMode: %d\n\r", splitMode.get());
			setup(-1);
		}
		else if (p.same(autoLatch)) {
			printf("DirectControlMode: updated autoLatch: %d\n\r", autoLatch.get());
		}
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD2(splitMode, autoLatch);
	}
	DirectControlMode() :
		presetFieldData{
			.splitMode = splitMode,
			.autoLatch = autoLatch,
		}
	{
		PresetDesc_t presetDesc = {
			.field = this,
			.size = sizeof(PresetFieldData_t),
			.defaulter = genericDefaulter2(DirectControlMode, splitMode, autoLatch),
			.loadCallback = genericLoadCallback2(DirectControlMode, splitMode, autoLatch),
		};
		presetDescSet(0, &presetDesc);
	}
	// splitMode from base class
	ParameterEnumT<3> autoLatch{this, kAutoLatchOff};
	PACKED_STRUCT(PresetFieldData_t {
		uint8_t splitMode;
		uint8_t autoLatch;
	}) presetFieldData;
private:
	bool hasSizeOutput()
	{
		return kModeSplitLocation != splitMode;
	}
	bool shouldAutoLatchSize()
	{
		return kAutoLatchBoth == autoLatch;
	}
	bool shouldAutoLatch()
	{
		return kAutoLatchOff != autoLatch;
	}
	rgb_t colors[2] = {
		{255, 0, 0},
		{255, 0, 127},
	};
	LatchProcessor latchProcessor;
	std::array<LatchProcessor::Reason,2> isLatched = {LatchProcessor::kLatchNone, LatchProcessor::kLatchNone};
	uint32_t lastLatchCount = ButtonView::kPressIdInvalid;
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

static inline float normToVfs(float in)
{
	return in * 15.f - 5.f;
}

static inline float inToV(float in)
{
	// checks to ensure we make sense
	assert(true == gInUsesCalibration);
	assert(false == gInUsesRange); // would be meaningless otherwise
	return normToVfs(in);
}

static inline float vToOut(float v)
{
	// checks to ensure we make sense
	assert(true == gOutUsesCalibration);
	assert(false == gOutUsesRange[0] || false == gOutUsesRange[1]); // TODO: should check the specific channel
	return (v + 5.f) / 15.f;
}

static inline float normToSemi(float in)
{
	return in * 15.f * 12.f;
}

static inline float semiToNorm(float in)
{
	return in / 15.f / 12.f;
}

static inline float quantiseToSemitones(float norm)
{
	return semiToNorm(int(normToSemi(norm) + 0.5f));
}

static float interpolatedRead(const float* table, size_t size, float idx)
{
	float n = size * idx;
	size_t prev = size_t(n);
	size_t next = size_t(n + 1);
	if(prev >= size) // idx was out of range
		prev = size - 1;
	if(next >= size)
		next = 0; // could be we are at the end of table
	float frac = n - prev;
	float value = linearInterpolation(frac, table[prev], table[next]);
	return value;
}

template <typename T>
static float interpolatedRead(const T& table, float idx)
{
	return interpolatedRead(table.data(), table.size(), idx);
}

class RecorderMode : public SplitPerformanceMode {
	enum {
		kInputModeTrigger,
		kInputModeClock,
		kInputModeCv,
		kInputModePhasor,
		kInputModeNum,
	};
public:
	bool setup(double ms) override
	{
		gOutMode.fill(kOutModeManualBlock);
		pastAnalogInHigh = false;
		if(isSplit())
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
	void render(BelaContext* context, FrameData* frameData) override
	{
		// set global states
		setOutIsSize();
		gInUsesRange = true; // may be overridden below depending on mode
		uint64_t currentSamples = context->audioFramesElapsed;

		// handle button
		if(!(!autoRetrigger && kInputModeTrigger == inputMode) && // when in envelope mode, no reason to erase recordings. We use the button for other stuff here
				(performanceBtn.pressDuration == msToNumBlocks(context, 3000)
				|| (kInputModeTrigger != inputMode && performanceBtn.tripleClick)))
		{
			emptyRecordings();
			// clear possible side effects of previous press:
			for(auto& qrec : qrecs)
			{
				qrec.armedFor = kArmedForNone;
				qrec.recording = kRecNone;
			}
			lastIgnoredPressId = performanceBtn.pressId;
		}
		bool triggerNow = false;
		if(performanceBtn.doubleClick)
		{
			if(kInputModeClock == inputMode)
			{
				for(auto& qrec : qrecs)
				{
					if(kArmedForStart == qrec.armedFor || qrec.recording)
					{
						if(kRecActual == qrec.recording){
							// probably just started recording because between
							// the first click and the double click a new clock edge
							// came in
							printf("sync when %d\n\r", qrec.periodsInRecording);
						}
						qrec.armedFor = kArmedForStartSynced;
						qrec.recording = kRecNone;
					}
				}
				lastIgnoredPressId = performanceBtn.pressId;
			}
		}
		if(performanceBtn.onset)
		{
			switch(autoRetrigger)
			{
			case 0:
			{
				for(size_t n = 0; n < kNumSplits; ++n)
				{
					// in envelope mode,
					if(gGestureRecorder.isRecording(n)) {
						// if a button is pressed while recording
						// we use this intermediate point to act as
						// a separator between attack and release when playing back
						envelopeReleaseStarts[n] = gGestureRecorder.rs[n].r.size();
						lastIgnoredPressId = performanceBtn.pressId;
					} else {
						// if not recording, on button press we
						// start the attack section of the envelope
						// NOTE: all other modes use the button offset
						// instead to minimise interaction with menu entering.
						// TODO: decide if this is really the best
						triggerNow = true;
						// we DO NOT ignore this press as we are interested
						// in getting its offset, too for triggering the release phase
					}
				}
			}
			break;
			}
		}
		enum StopMode {
			kStopNone = 0,
			kStopNowOnEdge, // stopping now and we are on an edge
			kStopNowLate, // stopping now and the edge has passed
		};
		std::array<StopMode,kNumSplits> qrecStopNow {kStopNone, kStopNone};
		std::array<uint32_t,kNumSplits> stopLateSamples {};
		std::array<bool,kNumSplits> releaseStarts {false, false};
		if(performanceBtn.offset && performanceBtn.pressId != lastIgnoredPressId)
		{
			switch(inputMode.get())
			{
			case kInputModeTrigger:
				if(!autoRetrigger)
				{
					for(size_t n = 0; n < kNumSplits; ++n)
						if(envelopeReleaseStarts[n] >= 0)
							releaseStarts[n] = true;
				} else {
					triggerNow = true;
				}
				break;
			case kInputModeClock:
			{
				// how late can a button press be to be considered as
				// belonging to the previous edge
				// this is clock-dependent with an upper limit
				uint64_t maxDelaySamples = std::min(gClockPeriod * 0.25f, 0.2f * context->analogSampleRate);
				uint64_t lateSamples = currentSamples - lastAnalogRisingEdgeSamples;
				bool closeEnough = (lateSamples < maxDelaySamples);
				for(size_t n = 0; n < kNumSplits; ++n)
				{
					auto& qrec = qrecs[n];
					if(closeEnough && 0 == n)
						printf("C %.3fs\n\r", lateSamples / context->analogSampleRate);
					switch(qrec.recording)
					{
					case kRecTentative:
						// a button press came in slightly late, but we let it
						// pass through as if it had arrived on time,
						// given how we are already tentatively recording
						if(closeEnough)
						{
							qrec.recording = kRecActual;
							break;
						}
						// otherwise arm for recording on next edge
						qrec.recording = kRecNone;
						// NOBREAK
					case kRecNone:
						qrec.armedFor = kArmedForStart;
						break;
					case kRecActual:
						if(closeEnough && !qrec.isSynced)
						{
							qrecStopNow[n] = kStopNowLate;
							stopLateSamples[n] = lateSamples;
						} else {
							// wait for next edge
							qrec.armedFor = kArmedForStop;
						}
						break;
					} // switch qrec.recording
				}
			} // case kInputModeClock:
				break;
			} // switch inputMode
		}
		tri.buttonLedWrite(0,
				qrecs[0].armedFor
				|| kRecActual == qrecs[0].recording
				|| qrecs[1].armedFor
				|| kRecActual == qrecs[1].recording
				);
		// detect edges on analog in
		// TODO: obey trigger level
		bool analogInHigh = tri.analogRead() > 0.5;
		bool analogRisingEdge = (analogInHigh && !pastAnalogInHigh);
		bool analogFallingEdge = (!analogInHigh && pastAnalogInHigh);
		pastAnalogInHigh = analogInHigh;
		std::array<RecordingMode,kNumSplits> qrecStartNow {kRecNone , kRecNone};
		std::array<bool,kNumSplits> qrecResetPhase { false, false };
		size_t recordOffset = 0;
		if(kInputModeClock == inputMode)
			recordOffset += GestureRecorder::kNumRecs / 2;
		if(analogRisingEdge)
		{
			lastAnalogRisingEdgeSamples = currentSamples;
			switch(inputMode.get())
			{
				case kInputModeTrigger:
					triggerNow = true;
					break;
				case kInputModeClock:
				{
					for(size_t n = 0; n < qrecs.size(); ++n)
					{
						auto& qrec = qrecs[n];
						// process periods
						if(kRecNone != qrec.recording)
							qrec.periodsInRecording++;
						qrec.periodsInPlayback++;
						if(qrec.periodsInPlayback >= periodsInTables[n])
							qrecResetPhase[n] = true;
					}
					// now process armed, which may use the periods processed above
					for(size_t n = 0; n < qrecs.size(); ++n)
					{
						auto& qrec = qrecs[n];
						bool isSynced = false;
						size_t ref;
						if(isSplit())
							ref = !n; // sync to the other split
						else
							ref = n; // sync to itself
						// TODO: this if should be part of the switch below?
						if(kArmedForStartSynced == qrec.armedFor)
						{
							if(qrecResetPhase[ref]) {
								qrec.armedFor = kArmedForStart;
								isSynced = true;
							}
						}
						switch(qrec.armedFor)
						{
						case kArmedForStart:
							qrec.armedFor = kArmedForNone;
							qrec.isSynced = isSynced;
							qrecStartNow[n] = kRecActual;
							break;
						case kArmedForStop:
						{
							bool thisShouldStop = kStopNowOnEdge;
							bool refShouldStop = kStopNone;
							if(qrec.isSynced)
							{
								// only stop recording if on a multiple
								if(0 == qrec.periodsInRecording % periodsInTables[ref])
								{
									if(isSplit()) // both stop
										refShouldStop = true;
								} else {
									thisShouldStop = false;
								}
							}
							if(thisShouldStop)
							{
								qrec.armedFor = kArmedForNone;
								qrecStopNow[n] = kStopNowOnEdge;
							}
							if(refShouldStop)
							{
								qrecs[ref].armedFor = kArmedForNone;
								qrecStopNow[ref] = kStopNowOnEdge;
							}
						}
							break;
						default:
							if(kRecNone == qrec.recording || kRecTentative == qrec.recording)
							{
								// start a tentative recording that will only be validated
								// if a button press comes in in the next few milliseconds
								qrecStartNow[n] = kRecTentative;
								qrec.periodsInRecording = 0;
							}
							break;
						}
						if(kRecActual == qrec.recording|| kRecTentative == qrec.recording)
						{
							qrec.recCounterAtLastEdge = gGestureRecorder.rs[n + recordOffset].recCounter;
							qrec.recSizeAtLastEdge = gGestureRecorder.rs[n + recordOffset].r.size();
						}
					}
					break;
				}
			}
		}
		if(analogFallingEdge)
		{
			if(0 == autoRetrigger) {
				for(size_t n = 0; n < kNumSplits; ++n)
				{
					if(envelopeReleaseStarts[n] >= 0)
						releaseStarts[n] = true;
				}
			}
		}

		std::array<centroid_t,kNumSplits> touches = touchTrackerSplit(globalSlider, ledSliders.isTouchEnabled() && frameData->isNew, isSplit());
		std::array<bool,kNumSplits> hasTouch;
		for(size_t n = 0; n < currentSplits(); ++n)
			hasTouch[n] = touches[n].size > 0;

		// control start/stop recording
		if(kInputModeClock == inputMode)
		{
			// start/stop recording based on qrec and input edges
			for(size_t n = 0; n < kNumSplits; ++n)
			{
				auto& qrec = qrecs[n];
				if(qrecStartNow[n])
				{
					gGestureRecorder.startRecording(n + recordOffset);
					envelopeReleaseStarts[n] = -1;
					qrec.recording = qrecStartNow[n];
					qrec.periodsInRecording = 0;
				}
				float phaseOffset = 0;
				if(kStopNowOnEdge == qrecStopNow[n] || kStopNowLate == qrecStopNow[n])
				{
					if(kStopNowLate == qrecStopNow[n])
					{
						// the edge has passed already. First we need to
						// shorten the recording discarding the last few frames
						gGestureRecorder.trimTo(n + recordOffset, qrec.recCounterAtLastEdge, qrec.recSizeAtLastEdge);
					}
					bool valid = true;
					if(kRecTentative == qrec.recording)
						valid = false;
					if(isSplit())
					{
						// if non split, always keep the new one. If split, only keep if something was recorded onto it
						if(!gGestureRecorder.rs[n + recordOffset].activity)
							valid = false;
					}
					qrec.recording = kRecNone;
					gGestureRecorder.stopRecording(n + recordOffset, false);
					if(valid)
					{
						gGestureRecorder.rs.swap(n, n + recordOffset);
						periodsInTables[n] = qrec.periodsInRecording;
						qrecResetPhase[n] = true;
						qrec.periodsInPlayback = 0;
						printf("%u periods\n\r", periodsInTables[n]);
						if(kStopNowLate == qrecStopNow[n])
						{
							// adjust the phase to make up for the lost time
							float normFreq = getOscillatorFreq(context, n) / context->analogSampleRate;
							phaseOffset = stopLateSamples[n] * normFreq * 2.f * float(M_PI);
						}
					}
				}
				// keep oscillators in phase with external clock pulses
				if(qrecResetPhase[n]) {
					oscs[n].setPhase(-M_PI + phaseOffset);
					qrec.periodsInPlayback = 0;
				}
			}

		} else {
			// start/stop recording based on touch

			// We have two recording tracks available, one per each analog output.
			// We are always using both tracks and the loop below controls automatic
			// recording start/stop per each track, based on the presence/absence of touch.
			// If we are split, the start/stop logic is separate for each track/split, each
			// following its own touch.
			// If we are not-split, then they both follow the same touch.
			if(!isSplit())
				hasTouch[1] = hasTouch[0]; // the second track follows the same touch as the first one
			for(size_t n = 0; n < kNumSplits; ++n)
			{
				if(hasTouch[n] != hadTouch[n]) //state change
				{
					if(1 == hasTouch[n] && 0 == hadTouch[n]) { // going from 0 to 1 touch: start recording
						gGestureRecorder.startRecording(n);
						envelopeReleaseStarts[n] = -1;
					} else if(0 == hasTouch[n]) {
						// going to 0 touches

						// if this is size and we are looping:
						// overwrite last few values in buffer to avoid
						// discontinuity on release
						bool optimizeForLoop = isSize(n) && autoRetrigger;
						gGestureRecorder.stopRecording(n, optimizeForLoop);
						periodsInTables[n] = 1;
					}
				}
			}
			hadTouch = hasTouch;
		}

		GestureRecorder::Gesture_t gesture; // used for visualization
		std::array<float,kNumSplits> recIns;
		switch(splitMode)
		{
		case kModeNoSplit:
			recIns[0] = touches[0].location;
			recIns[1] = touches[0].size;
			break;
		case kModeSplitLocation:
			recIns[0] = touches[0].location;
			recIns[1] = touches[1].location;
			break;
		case kModeSplitSize:
			recIns[0] = touches[0].size;
			recIns[1] = touches[1].size;
			break;
		}
		// gesture may be overwritten below before it is visualised
		for(size_t n = 0; n < recIns.size(); ++n)
		{
			size_t idx = n;
			if(gGestureRecorder.isRecording(n + recordOffset))
				idx += recordOffset;
			if(inputMode == kInputModeTrigger || gGestureRecorder.isRecording(idx))
			{
				if(releaseStarts[n])
					gGestureRecorder.resumePlaybackFrom(n, envelopeReleaseStarts[n]);
				gesture[n] = gGestureRecorder.process(idx, recIns[n], frameData->id, autoRetrigger, triggerNow, envelopeReleaseStarts[n]);
			}
		}

		std::array<bool,kNumSplits> directControl = { false, false };
		switch(inputMode)
		{
		case kInputModeTrigger:
			gOutMode.fill(kOutModeManualBlock);
			break;
		case kInputModeClock:
			for(size_t n = 0; n < currentSplits(); ++n)
			{
				// if actually doing something while recording, pass through current touch
				if(kRecActual == qrecs[n].recording && gGestureRecorder.rs[n + recordOffset].activity)
					directControl[n] = true;
				// if a finger is on the sensor and we are not recording, pass through current touch
				if(hasTouch[n])
					directControl[n] = true;
				if(isSplit()) {
					if(directControl[n])
					{
						gOutMode[n] = kOutModeManualBlock;
						gesture[n] = GestureRecorder::HalfGesture_t {
							.sample = kModeSplitLocation == splitMode ? touches[n].location : touches[n].size,
							.valid = true,
						};
					} else // otherwise, keep playing back from table
						gOutMode[n] = kOutModeManualSample;
				} else {
					// non split
					directControl[1] = directControl[n];
					if(directControl[n])
					{
						gOutMode.fill(kOutModeManualBlock);
						gesture[0] = { touches[0].location, true };
						gesture[1] = { touches[0].size, true };
					} else
						gOutMode.fill(kOutModeManualSample);
				}
			}
			break;
		default:
			gOutMode.fill(kOutModeManualSample);
		}
		for(unsigned int n = 0; n < kNumSplits; ++n)
		{
			if(kOutModeManualSample == gOutMode[n])
			{
				float value = processTable(context, n);
				gesture[n] = GestureRecorder::HalfGesture_t {.sample = value, .valid = true};
			} else {
				if(kInputModeClock == inputMode && directControl[n])
				{
					// we still have to processTable() just to ensure
					// the oscillator stay in phase.
					// outputs will actually be written here but they will
					// be overwritten in tr_render() because of gOutMode
					processTable(context, n);
				}
				// gesture is already set elsewhere
			}
		}
		static constexpr centroid_t kInvalid = {0, 0};
		// visualise
		std::array<centroid_t,kNumSplits> vizValues;
		bool isSizeOnly = (kModeSplitSize == splitMode);
		if(isSplit()){
			for(size_t n = 0; n < gesture.size(); ++n)
			{
				if(gesture[n].valid)
				{
					vizValues[n].location = gesture[n].sample;
					vizValues[n].size = isSizeOnly ? gesture[n].sample : kFixedCentroidSize;
				} else {
					vizValues[n] = kInvalid;
				}
			}
		} else {
			if(gesture.first.valid && gesture.second.valid)
			{
				vizValues[0].location = gesture.first.sample;
				vizValues[0].size = gesture.second.sample;
			} else
				vizValues[0] = kInvalid;
		}
		// this may set gManualAnOut even if they are ignored
		renderOut(gManualAnOut, vizValues, vizValues);
	}

	float processTable(BelaContext* context, unsigned int c)
	{
		assert(c < context->analogOutChannels && c < gGestureRecorder.kNumRecs && c < kNumSplits);
		float vizOut = 0;
		const float* table = gGestureRecorder.rs[c].r.getData().data();
		size_t tableSize = gGestureRecorder.rs[c].r.size();
		if(!tableSize)
		{
			for(size_t n = 0; n < context->analogFrames; ++n)
				analogWriteOnce(context, n, c, kNoOutput);
			vizOut = kNoOutput;

		} else if(kInputModeCv == inputMode || kInputModeClock == inputMode)
		{

			// wavetable oscillator
			float freq;
			if(kInputModeCv == inputMode)
			{
				// input is CV
				gInUsesRange = false; // we need actual voltage here
				float volts = inToV(analogRead(context, 0, 0));
				float baseFreq = 50;
				freq = vToFreq(volts, baseFreq);
			} else {
				// input is clock
				assert(periodsInTables[c] > 0);
				if(!periodsInTables[c])
					periodsInTables[c] = 1;
				freq = getOscillatorFreq(context, c);
			}
			float normFreq = freq / context->analogSampleRate;
			for(size_t n = 0; n < context->analogFrames; ++n)
			{
					float idx = map(oscs[c].process(normFreq), -1, 1, 0, 1);
					float value = interpolatedRead(table, tableSize, idx);
					analogWriteOnce(context, n, c, value);
					if(0 == n)
						vizOut = value;
			}
		} else if (kInputModePhasor == inputMode) {
			for(size_t n = 0; n < context->analogFrames; ++n)
			{
				float idx = analogRead(context, n, 0);
				{
					float out = interpolatedRead(table, tableSize, idx);
					analogWriteOnce(context, n, c, out);
					if(0 == n)
						vizOut = out;
				}
			}
		} else {
			assert(0);
			vizOut = 0;
		}
		// return snapshot for visualisation
		return vizOut;
	}

	void updated(Parameter& p)
	{
		if(p.same(splitMode)) {
			printf("RecorderMode: Updated splitMode: %d\n\r", splitMode.get());
			setup(-1);
		}
		else if (p.same(autoRetrigger)) {
			printf("RecorderMode: Updated retrigger %d\n\r", autoRetrigger.get());
		} else if (p.same(inputMode)) {
			printf("RecorderMode: Updated inputMode: %d\n\r", inputMode.get());
			if(kInputModeClock == inputMode)
				for(auto& qrec : qrecs)
					qrec = QuantisedRecorder();
		}
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD3(splitMode, autoRetrigger, inputMode);
	}
	RecorderMode() :
		presetFieldData {
			.splitMode = splitMode,
			.autoRetrigger = autoRetrigger,
			.inputMode = inputMode,
		}
	{
		PresetDesc_t presetDesc = {
			.field = this,
			.size = sizeof(PresetFieldData_t),
			.defaulter = genericDefaulter3(RecorderMode, splitMode, autoRetrigger, inputMode),
			.loadCallback = genericLoadCallback3(RecorderMode, splitMode, autoRetrigger, inputMode),
		};
		presetDescSet(1, &presetDesc);
	}
	//splitMode from the base class
	ParameterEnumT<2> autoRetrigger{this, true};
	ParameterEnumT<kInputModeNum> inputMode{this, kInputModeTrigger};
	PACKED_STRUCT(PresetFieldData_t {
		uint8_t splitMode;
		uint8_t autoRetrigger ;
		uint8_t inputMode;
	}) presetFieldData;
private:
	float getOscillatorFreq(BelaContext* context, size_t c)
	{
		return context->analogSampleRate / gClockPeriod / periodsInTables[c];
	}
	bool isSize(size_t n)
	{
		switch(splitMode)
		{
		default:
		case kModeSplitLocation:
			return false;
		case kModeNoSplit:
			return n == 1;
		case kModeSplitSize:
			return true;
		}
	}
	void emptyRecordings()
	{
		gGestureRecorder.empty();
		periodsInTables.fill(1);
	}
	rgb_t colors[2] = {
			{128, 128, 0},
			{128, 128, 100},
	};
	enum ArmedFor {
		kArmedForNone,
		kArmedForStart,
		kArmedForStop,
		kArmedForStartSynced,
	};
	enum RecordingMode {
		kRecNone = 0,
		kRecTentative,
		kRecActual,
	};
	struct QuantisedRecorder {
		size_t periodsInRecording;
		size_t periodsInPlayback;
		size_t recCounterAtLastEdge;
		size_t recSizeAtLastEdge;
		ArmedFor armedFor;
		RecordingMode recording;
		bool isSynced;
	};
	std::array<QuantisedRecorder,kNumSplits> qrecs {};
	static constexpr size_t kNumSplits = ::kNumSplits;
	std::array<Oscillator,kNumSplits> oscs {{{1, Oscillator::sawtooth}, {1, Oscillator::sawtooth}}};
	std::array<size_t,kNumSplits> periodsInTables {1, 1};
	std::array<bool,kNumSplits> hadTouch {};
	std::array<ssize_t,kNumSplits> envelopeReleaseStarts { -1, -1 };
	size_t lastIgnoredPressId = ButtonView::kPressIdInvalid;
	uint64_t lastAnalogRisingEdgeSamples = 0;
	bool pastAnalogInHigh = false;
} gRecorderMode;

static void menu_enterRangeDisplay(const rgb_t& signalColor, const std::array<rgb_t,2>& endpointsColors, bool autoExit, ParameterContinuous& bottom, ParameterContinuous& top, const float& display);
static void menu_enterDisplayRangeRaw(const rgb_t& color, const rgb_t& otherColor, float bottom, float top);
static void menu_enterDisplayScaleMeterOutputMode(const rgb_t& color, bool bottomEnv, bool topEnv);
static void menu_up();

#define FILL_ARRAY(name, value) [](){decltype(name) a; a.fill(value); return a;}();

class ScaleMeterMode : public PerformanceMode {
public:
	bool setup(double ms) override
	{
		gOutIsSize = {false, false};
		count = 0;
		x1 = 0;
		y1 = 0;
		rms = 0;
		env = 0;
		updated(cutoff);
		if(ms <= 0)
		{
			ledSlidersSetupOneSlider(
				signalColor,
				LedSlider::MANUAL_CENTROIDS
			);
			gOutMode.fill(kOutModeManualSample);
		}
		if(ms < 0)
			return true;
		return modeChangeBlink(ms, signalColor);
	}

	void render(BelaContext* context, FrameData* frameData) override
	{
		// we can quickly get into menu mode from here
		if(!gAlt)
		{
			static std::array<float,kNumPads> data = {0};
			if(!performanceBtn.pressed && ledSliders.sliders[0].getNumTouches())
			{
				// only touch on: set output range
				menu_enterRangeDisplay(signalColor, {endpointsColorOut, endpointsColorOut}, true, outRangeMax, outRangeMin, outDisplay);
				// TODO: line below is just a workaround because we don't have a clean way of
				// _entering_ menu from here while ignoring the _last_ slider readings,
				// resulting in automatically re-entering immediately after exiting
				ledSliders.sliders[0].process(data.data());
				return;
			}
			if(performanceBtn.offset)
			{
				// press button: set input range
				menu_enterRangeDisplay(signalColor, {endpointsColorIn, endpointsColorIn}, false, inRangeBottom, inRangeTop, inDisplay);
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
				float value = mapAndConstrain(outs[c], 0, 1, outRangeMax, outRangeMin);
				analogWriteOnce(context, n, c, value);
			}
		}
		// displays if in In/OutRange mode
		outDisplay = mapAndConstrain(analogReadMapped(context, 0, 0), 0, 1, outRangeMax, outRangeMin);
		inDisplay = analogReadMapped(context, 0, 0);
		// displays if in pure performance mode
		centroid_t centroids[2];
		// display actual output range
		centroids[0].location = outDisplay;
		centroids[0].size = kFixedCentroidSize;
		size_t numCentroids = 1;
		if(kOutputModeNN != outputMode)
		{
			centroids[1].location = mapAndConstrain(env, 0, 1, outRangeMax, outRangeMin);
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
		else if(p.same(outRangeMax) || p.same(outRangeMin)) {
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
	ParameterContinuous outRangeMax {this, 0};
	ParameterContinuous outRangeMin {this, 1};
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
	const rgb_t signalColor = {0, 160, 160};
	const rgb_t endpointsColorIn = {255, 0, 0};
	const rgb_t endpointsColorOut = {0, 255, 0};
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
		gOutIsSize = {false, false};
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
		gOutMode.fill(kOutModeManualSample);
		if(ms < 0)
			return true;
		return modeChangeBlinkSplit(ms, gBalancedLfoColors.data(), kNumLeds / 2, kNumLeds / 2);
	}
	void render(BelaContext* context, FrameData* frameData) override
	{
		gInUsesRange = true; // may be overridden below depending on mode
		float clockPeriod;
		float sampleRate = context->analogSampleRate;
		switch (inputMode)
		{
		default:
		case kInputModeTrig:
			// after frequency was set by parameter,
			// we use gClockPeriod only if it gets updated at least once
			if(gClockPeriodUpdateCounter == lastClockPeriodUpdateCounter)
			{
				clockPeriod = sampleRate / (centreFrequency * 10.f + 0.1f); // TODO: more useful range? exp mapping?
			} else {
				clockPeriod = gClockPeriod;
				lastClockPeriodUpdateCounter = 0; // show that we are locked to gClockPeriod
			}
			break;
		case kInputModeCv:
		{
			gInUsesRange = false; // we want actual volts here
			float volts = inToV(analogRead(context, 0, 0));
			float freq = vToFreq(volts, 3);
			clockPeriod = sampleRate / freq;
		}
			break;
		}

		float midFreq = 1.f / clockPeriod; // we process once per block; the oscillator thinks Fs = 1

		if (ledSliders.isTouchEnabled() && globalSlider.getNumTouches()) {
			divisionPoint = globalSlider.compoundTouchLocation();
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
			lastClockPeriodUpdateCounter = gClockPeriodUpdateCounter;
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
	uint32_t lastClockPeriodUpdateCounter = gClockPeriodUpdateCounter;
} gBalancedOscsMode;

#define clickprintf(...) // use to enable debug printing in case of need
class ExprButtonsMode : public PerformanceMode
{
public:
	bool setup(double ms)
	{
		gOutIsSize = {false, true};
		if(ms <= 0)
		{
			ledSlidersSetupMultiSlider(
				ledSliders,
				{
				},
				LedSlider::MANUAL_CENTROIDS,
				true,
				1
			);
			gOutMode.fill(kOutModeManualBlock);
			changeState(kDisabled, {0, 0});
		}
		page = kPagePerf;
		// Force initialisation of offsets. Alternatively, always copy them in render()
		for(auto& o : offsetParameters)
			updated(o);
		if(ms < 0)
			return true;
		return modeChangeBlink(ms, {0, 200, 50});
	}
	void render(BelaContext* context, FrameData* frameData)
	{
		ButtonView btn = ButtonViewSimplify(performanceBtn);
		gInUsesRange = false;
		gOutUsesRange[0] = false;
		gOutUsesRange[1] = true;
		if(pitchBeingAdjusted >= 0 && !seqMode)
		{
			// if we are adjusting the pitch, output that instead
			gManualAnOut[0] = getOutForKey(pitchBeingAdjusted);
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
		if(ledSliders.isTouchEnabled())
			gTouchTracker.process(globalSlider);
		centroid_t centroid = gTouchTracker.getNumTouches() ? gTouchTracker.getTouchMostRecent().touch : centroid_t{0, 0};
		if(btn.tripleClick)
		{
			clickprintf("t%d%d->", onClickGroupStartWas, page);
			if(kPageSampling == page || kPageSampling == onClickGroupStartWas)
				page = kPagePerf;
			else
				page = kPageSampling;
			clickprintf("%d\n\r", page);
		}
		if(globalSlider.getNumTouches() >= 4 && pastNumTouches < 4)
			seqMode = !seqMode;
		pastNumTouches = globalSlider.getNumTouches();
		bool newTouch = false;
		if(!centroid.size)
		{
			// touch is removed
			if(kDisabled != touch.state && (kHold != touch.state || seqMode))
				changeState(kDisabled, centroid);
		} else {
			// if it is a new touch, assign this touch to a key, store location as impact location,
			// mark it as unmoved
			if(kDisabled == touch.state || (kHold == touch.state && touch.holdHasReleased))
			{
				changeState(kInitial, centroid); // note that this may fail and we go back to kDisabled
				newTouch = true;
			}
		}
		if(btn.offset && !seqMode)
		{
			// if holding, release
			if(kHold == touch.state)
				changeState(kDisabled, centroid);
			// if there is a press, hold
			if(kDisabled != touch.state)
			{
				changeState(kHold, centroid);
				touch.holdHasReleased = false;
			}
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
		if(kHold == touch.state)
		{
			if(!centroid.size)
				touch.holdHasReleased = true;
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
		case kHold:
			break;
		case kDisabled:
		case kNumStates:
			break;
		}

		// now set output
		switch(touch.state)
		{
		case kInitial:
			out = getOutForKey(touch.key);
			break;
		case kMoved:
			out = getOutForKey(touch.key) + touch.mod * modRange * 4.f;
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
				float destOut = getOutForKey(bendDestKey);
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
		case kHold:
			out = pastOuts[0];
			centroid.size = pastOuts[1];
			break;
		case kDisabled:
		case kNumStates:
			break;
		}
		gManualAnOut[0] = out;
		gManualAnOut[1] = centroid.size;
		pastOuts = gManualAnOut;
		if(kPageSampling == page)
		{
			// override any output that may have happened so far.
			// We leverage the state machine above even if it's
			// more complicated than what we need here, as it's
			// useful to get touch.key and the display below and
			// keep consistency overall.

			// this works as a little sample and hold
			if(kKeyInvalid == touch.key)
			{
				// TODO: pass-through at audio rate unless key is pressed
				gManualAnOut[0] = quantise(analogRead(context, 0, 0));
				gManualAnOut[1] = 1;
			} else {
				if(newTouch)
				{
					// sample
					float sum = 0;
					for(size_t n = 0; n < context->analogFrames; ++n)
						sum += analogRead(context, n, 0);
					float mean = sum / context->analogFrames;
					// we cannot simply call update() and update both,
					// or we will trigger the override at the top of the function.
					// So we have to set them individually.
					offsetParameters[touch.key].setNoUpdate(mean);
					offsets[touch.key] = mean;
				}
				// hold
				gManualAnOut[0] = getOutForKey(touch.key);
				gManualAnOut[1] = centroid.size;
			}
		}
		size_t vizKey;
		bool analogInHigh = tri.analogRead() > 0.5;
		bool analogRisingEdge = (analogInHigh && !pastAnalogInHigh);
		pastAnalogInHigh = analogInHigh;
		if(seqMode && kPageSampling != page)
		{
			if(analogRisingEdge)
			{
				size_t attempts = 0;
				do
				{
					attempts++;
					seqCurrentStep++;
					if(seqCurrentStep >= kNumButtons)
						seqCurrentStep = 0;
				} while (!stepsEnabled[seqCurrentStep] && attempts < kNumButtons);
#if 0 // print step state
				for(size_t n = 0; n < kNumButtons; ++n)
				{
					char sym0 = stepsEnabled[n] ? '_' : 'X';
					if(stepsEnabled[n] && seqCurrentStep == n)
						sym0 = '*';
					char sym1 = 'O';
					switch(stepsMode[n])
					{
					case kStepNormal:
						sym1 = 'N';
						break;
					case kStepMuted:
						sym1 = 'M';
						break;
					case kStepHold:
						sym1 = 'H';
						break;
					case kStepModesNum:
						sym1 = 'o';
						break;
					}
					printf("%c%c,", sym0, sym1);
				}
				printf("\n\r");
#endif
			}
			// single click enters (or exits) set mode page
			if(btn.onset)
			{
				Page stash = page;
				if(kPageSetMode == page)
					page = kPagePerf;
				else
					page = kPageSetMode;
				onClickGroupStartWas = stash;
				clickprintf("s%d%d\n\r", onClickGroupStartWas, page);
			}
			// double click enters (or exits) set enable page
			if(btn.doubleClick)
			{
				clickprintf("d%d%d->", onClickGroupStartWas, page);
				if(kPageSetEnable == page || kPageSetEnable == onClickGroupStartWas)
					page = kPagePerf;
				else
					page = kPageSetEnable;
				clickprintf("%d\n\r", page);
			}
			if(kKeyInvalid != touch.key)
			{
				// if we have a touch
				switch(page)
				{
				case kPagePerf:
					seqCurrentStep = touch.key; // reset to key
					break;
				case kPageSetMode:
					if(newTouch)
					{
						// each new key press cycles through step states
						StepMode& mode = stepsMode[touch.key];
						mode = StepMode(mode + 1);
						if(kStepModesNum == mode)
							mode = kStepNormal;
					}
					break;
				case kPageSetEnable:
					if(newTouch)
						stepsEnabled[touch.key] = !stepsEnabled[touch.key];
					break;
				case kPageSampling:
					break;
				}
			}
			vizKey = seqCurrentStep;
			size_t outKey = kKeyInvalid;
			switch(stepsMode[seqCurrentStep])
			{
			case kStepNormal:
				outKey = seqCurrentStep;
				break;
			case kStepMuted:
				outKey = kKeyInvalid;
				break;
			case kStepHold:
			{
				size_t step = seqCurrentStep;
				// go back looking for the step to hold
				do
				{
					if(0 == step)
						step = kNumButtons;
					step--;
					if(kStepNormal == stepsMode[step])
						break;
				} while(step != seqCurrentStep);
				if(step == seqCurrentStep) // no step to hold
					outKey = kKeyInvalid;
				else
					outKey = step;
			}
			default:
				break;
			}
			gManualAnOut[0] = getOutForKey(outKey);
			size_t lowestEnabled = 0;
			while(lowestEnabled < stepsEnabled.size() && !stepsEnabled[lowestEnabled])
				lowestEnabled++;
			gManualAnOut[1] = (seqCurrentStep == lowestEnabled); // send out a reset signal
		} else {
			vizKey = touch.key;
		}
		// display
		for(size_t n = 0; n < kNumButtons; ++n)
		{
			if(!gAlt)
			{
				float coeff = (n == vizKey) ? 1 : 0.1;
				size_t pixel = size_t(getMidLocationFromKey(n) * kNumLeds + 0.5f);
				if(seqMode && kPageSampling != page)
				{
					rgb_t color;
					coeff *= stepsEnabled[n];
					switch(stepsMode[n])
					{
					case kStepModesNum:
					case kStepNormal:
						color = {0, 0, 255};
						break;
					case kStepHold:
						color = {180, 180, 0};
						break;
					case kStepMuted:
						color = {200, 0 , 0};
						break;
					}
					float period = 0.3f * context->analogSampleRate;
					float triangle = simpleTriangle(context->audioFramesElapsed, period);
					// TODO: animating buttons while they are traversed by the sequencer
					// gives a messy result. Try syncing it to the clock input, or use a
					// different display strategy (e.g.: button?)
					switch(page)
					{
					case kPagePerf:
						coeff *= 1.f;
						break;
					case kPageSetMode:
						coeff *=  0.1f + 0.9f * triangle;
						break;
					case kPageSetEnable:
						coeff *=  triangle > 0.7f;
						break;
					case kPageSampling: // shouldn't be here anyhow
						break;
					}
					np.setPixelColor(pixel, color.r * coeff, color.g * coeff, color.b * coeff);
				} else {
					//TODO: have setPixelColor obey "enabled"
					if(kPageSampling == page && n != vizKey)
					{
						// inactive keys while sampling have a triangle pattern
						float period = 0.5f * context->analogSampleRate;
						coeff *=  0.1f + 0.9f * simpleTriangle(context->audioFramesElapsed + (n * period / kNumButtons), period);
					}
					np.setPixelColor(pixel, colors[n].r * coeff, colors[n].g * coeff, colors[n].b * coeff);
				}
			}
		}
	}
private:
	typedef enum {
		kInitial,
		kMoved,
		kBending,
		kHold,
		kDisabled,
		kNumStates,
	} TouchState;
	const std::array<const char*,kNumStates> touchStateNames {
			"kInitial",
			"kMoved",
			"kBending",
			"kDisabled",
	};
	float getOutForKey(size_t key)
	{
		if(key >=  offsets.size())
			return 0;
		return quantise(offsets[key]);
	}
	float quantise(float in, bool force = false)
	{
		if(quantised || force)
			return quantiseToSemitones(in);
		else
			return in;
	}
	void changeState(TouchState newState, const centroid_t& centroid)
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
		case kHold:
			touch.holdHasReleased = false;
			break;
		case kDisabled:
			touch.key = kKeyInvalid;
			break;
		case kNumStates:
			break;
		}
		S(printf("\n\r"));
		touch.state = newState;
		touch.initialLocation = centroid.location;
		touch.initialOut = out;
	}
	bool shouldBend(const centroid_t& centroid)
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
	static constexpr size_t kKeyInvalid = -1;
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
		bool holdHasReleased = false;
	} touch;
	enum StepMode {
		kStepNormal,
		kStepHold,
		kStepMuted,
		kStepModesNum,
	};
	std::array<bool,kNumButtons> stepsEnabled = FILL_ARRAY(stepsEnabled, true);
	std::array<StepMode,kNumButtons> stepsMode = FILL_ARRAY(stepsMode, kStepNormal);
	std::array<float,kNumOutChannels> pastOuts;
	size_t pastNumTouches = 0;
	enum Page {
		kPagePerf,
		kPageSetMode,
		kPageSetEnable,
		kPageSampling,
	};
	Page page = kPagePerf;
	// this is used in order to ignore effect of single and {single,double} click when
	// processing a double or triple click, respectively.
	Page onClickGroupStartWas = kPagePerf;
	size_t seqCurrentStep = 0;
	bool seqMode = false;
	bool pastAnalogInHigh = false;
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
	// do not retrieve offsets directly, use getOutForKey() instead
	std::array<float,kNumButtons> offsets;
	int pitchBeingAdjusted = -1;
	unsigned int pitchBeingAdjustedCount = 0;
	static constexpr unsigned int kPitchBeingAdjustedCountMax = 10;
} gExprButtonsMode;


static constexpr float fromCode(uint16_t code)
{
	return code / 4096.f;
}

static constexpr uint16_t toCode(float out)
{
	if(out >= 4095.f)
		return 4095;
	if(out <= 0)
		return 0;
	return 4096.f * out;
}

class CalibrationProcedure : public ParameterUpdateCapable {
public:
typedef enum {
	kWaitToStart,
	kNoInput,
	kWaitConnect,
	kConnected,
	kDone,
} Calibration_t;
Calibration_t getState() { return calibrationState; }

private:
class CalibrationDataParameter : public CalibrationData, public Parameter {
public:
	CalibrationDataParameter(ParameterUpdateCapable* that) : that(that) {}
	void set(const CalibrationData& cal) {
		values = cal.values;
		that->updated(*this);
	}
private:
	ParameterUpdateCapable* that;
};
Calibration_t calibrationState;
CalibrationDataParameter calibrationOut {this};
CalibrationDataParameter calibrationIn {this};

typedef enum {
	kFindingDacGnd,
	kFindingAdcVals,
	kFindingDone,
} Connected_t;
Connected_t connectedState;
unsigned int findingAdcIdx;
size_t count;
float adcAccu;
float minDiff;
uint16_t minCode;
uint16_t outCode;
// some reasonable defaults
float outGnd = 0.333447;
float outBottom = 0;
float outTop = 1;
float inGnd = 0.365818;
float inBottom = 0;
float inTop = 1;
static constexpr unsigned kAverageCount = 2000;
static constexpr unsigned kConnectedStepCount = 20;
static constexpr unsigned kWaitAfterSetting = 5;
static constexpr unsigned kDoneCount = 3000;
static constexpr unsigned kWaitPostThreshold = 100;
static constexpr float kAdcConnectedThreshold = 0.1;
static constexpr float kStep = 1;
static constexpr float kRangeStart = toCode(0.30);
static constexpr float kRangeStop = toCode(0.35);
static constexpr float kIoTopV = 10;
static constexpr float kIoGndV = 0;
static constexpr float kIoBottomV = -5;

void publishCalibrationData()
{
	calibrationOut.values = {outBottom, outGnd, outTop};
	calibrationIn.values = {inBottom, inGnd, inTop};
}
public:
CalibrationProcedure() :
	presetFieldData({
		.calibrationOut = calibrationOut,
		.calibrationIn = calibrationIn,
	})
{
	publishCalibrationData(); // load factory settings
	PresetDesc_t presetDesc = {
		.field = this,
		.size = sizeof(PresetFieldData_t),
		.defaulter = genericDefaulter2(CalibrationProcedure, calibrationOut, calibrationIn),
		.loadCallback = genericLoadCallback2(CalibrationProcedure, calibrationOut, calibrationIn),
	};
	presetDescSet(5, &presetDesc);
}
void updated(Parameter& p)
{
	if(p.same(calibrationOut)) {
		printf("CalibrationProcedure: updated calibration out %f %f %f\n\r", calibrationOut.values[0], calibrationOut.values[1], calibrationOut.values[2]);
	}
	else if (p.same(calibrationIn)) {
		printf("CalibrationProcedure: updated calibration in %f %f %f\n\r", calibrationIn.values[0], calibrationIn.values[1], calibrationIn.values[2]);
	}
}
void updatePreset() override
{
	UPDATE_PRESET_FIELD2(calibrationOut, calibrationIn);
}
struct PresetFieldData_t { // no "PACKED" because of misleading warning from gcc
	CalibrationDataParameter calibrationOut;
	CalibrationDataParameter calibrationIn;
} presetFieldData;
void setup()
{
	count = 0;
	calibrationState = kWaitToStart;
	printf("Disconnect INPUT\n\r"); // TODO: this is printed repeatedly till you release the button
	gOutMode.fill(kOutModeManualBlock);
}

void process()
{
	float anIn = tri.analogRead();
	gOverride.started = HAL_GetTick();
	gOverride.isSize = false;
	switch (calibrationState)
	{
		case kWaitToStart:
			gOverride.started = 0;
			count = 0;
			break;
		case kNoInput:
			adcAccu += anIn;
			count++;
			if(kAverageCount == count)
			{
				calibrationState = kWaitConnect;
				inGnd = adcAccu / count;
				printf("inGnd: %.5f, connect an input\n\r", inGnd);
				outCode = 0; // set this as a test value so we can detect when DAC is connected to ADC
				count = 0;
			}
			break;
		case kWaitConnect:
			// wait for ADC to be connected, then wait some more to avoid any spurious transients
			if(anIn < kAdcConnectedThreshold)
			{
				if(0 == count)
					printf("Jack connected");
				count++;
			} else {
				count = 0;
			}
			if(kWaitPostThreshold == count)
			{
				printf(", started\n\r");
				calibrationState = kConnected;
				minDiff = 1000000000;
				minCode = 4096;
				count = 0;
				outCode = kRangeStart;
				connectedState = kFindingDacGnd;
			}
			break;
		case kConnected:
		{
			switch(connectedState)
			{
				case kFindingDacGnd:
				{
					if(outCode >= kRangeStop)
					{
						printf("Gotten a minimum at code %u (%f), diff: %f)\n\r", minCode, fromCode(minCode), minDiff);
						outGnd = fromCode(minCode);
						// now that outGnd is set, we can use fromVolt()
						outTop = fromVolt(kIoTopV);
						outBottom = fromVolt(kIoBottomV);
						printf("DAC calibration: %.2fV: %f(%d), %.2fV: %f(%d), %.2fV: %f(%d)\n\r",
								kIoBottomV, outBottom, toCode(outBottom),
								kIoGndV, outGnd, toCode(outGnd),
								kIoTopV, outTop, toCode(outTop));
						count = 0;
						connectedState = kFindingAdcVals;
						findingAdcIdx = 0;
						break;
					}
					if (kConnectedStepCount == count) {
						float average = adcAccu / (count - kWaitAfterSetting);
						float diff = average - inGnd;
						diff = std::abs(diff);
						if(diff < minDiff)
						{
							minDiff = diff;
							minCode = outCode;
						}
						count = 0;
						outCode += kStep;
					}
					if(0 == count)
						adcAccu = 0;
					 else if (count >= kWaitAfterSetting)
						adcAccu += anIn;
					count++;
				}
					break;
				case kFindingAdcVals:
				{
					if(0 == findingAdcIdx)
						outCode = toCode(outBottom);
					else if (1 == findingAdcIdx)
						outCode = toCode(outTop);
					else {
						connectedState = kFindingDone;
						break;
					}
					if(0 == count)
						adcAccu = 0;
					else if(count >= kWaitAfterSetting)
						adcAccu += anIn;
					++count;
					if(kAverageCount == count) {
						float value = adcAccu / (count - kWaitAfterSetting);
						switch(findingAdcIdx)
						{
						case 0:
							inBottom = value;
							break;
						case 1:
							inTop = value;
							break;
						}
						findingAdcIdx++;
						count = 0;
					}
				}
					break;
				case kFindingDone:
					calibrationState = kDone;
					count = 0;
					printf("ADC calibration: %.2fV: %f, %.2fV: %f, %.2fV: %f\n\r",
							kIoBottomV, inBottom,
							kIoGndV, inGnd,
							kIoTopV, inTop);
					publishCalibrationData();
					break;
			}
		}
			break;
		case kDone:
		{
#if 0
			// loop through three states showing the -5V, 0V, 10V output range of the module
			uint16_t bottomCode = toCode(outBottom);
			uint16_t topCode = toCode(outTop);
			uint16_t gndCode = toCode(outGnd);
			if(outCode != gndCode && outCode != bottomCode && outCode != topCode)
				outCode = gndCode;
			if(count >= kDoneCount)
			{
				count = 0;
				if(gndCode == outCode)
					outCode = topCode;
				else if(topCode == outCode)
					outCode = bottomCode;
				else if(bottomCode == outCode)
					outCode = gndCode;
			}
			count++;
#else
			gOverride.started = 0;
#endif
		}
			break;
	}
	gOverride.out = fromCode(outCode);
	gOverride.ch = 0;
	gOverride.bypassOutRange = true;
}
void start(){
	calibrationState = kNoInput;
	adcAccu = 0;
}
void stop(){
	calibrationState = kWaitToStart;
}
void toggle()
{
	if(running())
		stop();
	else
		start();
}
bool running()
{
	return !(kWaitToStart == calibrationState|| kDone == calibrationState);
}
bool done()
{
	return kDone == calibrationState;
}
const CalibrationData& getIn() const
{
	return calibrationIn;
}
const CalibrationData& getOut() const
{
	return calibrationOut;
}
bool valid()
{
	return calibrationState >= kDone;
}
uint16_t getCode()
{
	return outCode;
}
float fromVolt(float Vo)
{
	// full range DAC is Vdfs = 3.3V
	// output gain after the DAC can be assumed fixed at Ro/Ri
	// as we are using .1% resistors  (it's an inverting amp,
	// but there is an inversion in sw elsewhere). This gives a nominal
	// analog out of 3.3 * 127/27 = 15.52V
	// The offset is such that
	// when gndCode is sent out, the output voltage is 0V.
	//
	// So, at the net of an offset (which we compensate for at
	// the end), we have:
	//   Vo = out * Vdfs * Ga
	// where out is the dac out relative to full scale and
	//   Ga = Ro/Ri // analog gain
	// so:
	//   out = Vo / (Vdfs * Ga)
	constexpr float Vdfs = 3.3;
	constexpr float Ro = 127;
	constexpr float Ri = 27;
	constexpr float Ga = Ro / Ri;
	float out = Vo / (Vdfs * Ga);
	// compensate for offset:
	out += outGnd;
	return out;
}
} gCalibrationProcedure;

static constexpr CalibrationData kNoCalibration = {
	.values = {CalibrationData::points}, // passthrough
};

CalibrationData const& getCalibrationInput()
{
	if(gInUsesCalibration)
		return gCalibrationProcedure.getIn();
	else
		return kNoCalibration;
}

CalibrationData const& getCalibrationOutput()
{
	if(gOutUsesCalibration)
		return gCalibrationProcedure.getOut();
	else
		return kNoCalibration;
}

// crossfadeRgbChannel
static uint8_t crg(uint8_t a, uint8_t b, float idx)
{
	return a * (1.f - idx) + b * idx;
}
static rgb_t crossfade(const rgb_t& a, const rgb_t& b, float idx)
{
	return rgb_t{
		.r = crg(a.r, b.r, idx),
		.g = crg(a.g, b.g, idx),
		.b = crg(a.b, b.b, idx),
	};
}

static constexpr rgb_t kCalibrationColor = {255, 255, 255};
class CalibrationMode : public PerformanceMode {
	enum Animation {
		kBlink,
		kStatic,
		kGlow,
	};
	rgb_t baseColor;
	uint32_t startTime;
	size_t demoModeCount;
	size_t demoModeState;
	std::array<uint32_t,3> clicks {};
	size_t clickIdx = 0;
	void resetDemoMode()
	{
		demoModeCount = 0;
		demoModeState = 0;
	}
public:
	CalibrationMode(const rgb_t& color) :
		baseColor(color),
		startTime(HAL_GetTick())
	{}
	bool setup(double ms){
		gCalibrationProcedure.setup();
		gOutMode.fill(kOutModeManualBlock);
		resetDemoMode();
		ledSlidersSetupOneSlider(
			baseColor,
			LedSlider::MANUAL_CENTROIDS
		);
		return true;
	}
	void render(BelaContext* context, FrameData* frameData) override
	{
		// these may be overridden below if calibration is done
		gOutUsesCalibration = false;
		gInUsesCalibration = false;
		gInUsesRange = false;
		gOutUsesRange = {false, false};
		uint32_t tick = HAL_GetTick();
		// wait for button press to start or stop calibration.
		if(performanceBtn.offset)
		{
			clicks[clickIdx++] = tick;
			if(clicks.size() == clickIdx)
				clickIdx = 0;
			bool shouldToggle = true;
			if(gCalibrationProcedure.done())
			{
				// if calibration is done, triple press in a short period
				// of time is needed to restart
				uint32_t recentTime = tick - 600; // 600 ms
				for(auto c : clicks)
				{
					if(c < recentTime)
					{
						shouldToggle = false;
						break;
					}
				}
			}
			if(shouldToggle)
			{
				gCalibrationProcedure.toggle();
				resetDemoMode();
			}
		}
		gCalibrationProcedure.process();

		Animation animation;
		CalibrationProcedure::Calibration_t calibrationState = gCalibrationProcedure.getState();
		size_t begin = 8;
		size_t end = 16;
		switch(calibrationState)
		{
		default:
		case CalibrationProcedure::kNoInput:
			animation = kGlow;
			break;
		case CalibrationProcedure::kWaitConnect:
			animation = kBlink;
			break;
		case CalibrationProcedure::kConnected:
			animation = kGlow;
			break;
		case CalibrationProcedure::kDone:
		case CalibrationProcedure::kWaitToStart:
		{
			// override is disabled here (CalibrationProcedure is not sending out anything),
			// so we enable calibration and send out fixed test voltages
			gOutUsesCalibration = true;
			gInUsesCalibration = true;
			gInUsesRange = false; // still disabled: want to get actual volts
			gOutUsesRange = {false, false}; // still disabled: want to get actual volts
			static constexpr std::array<float,4> kTestVoltages = {0, 5, 10, -5};
			demoModeCount++;
			if((demoModeCount % 300) == 0)
				printf("%.4f->%.3fV\n\r", analogRead(context, 0, 0), inToV(analogRead(context, 0, 0)));
			if(3000 == demoModeCount)
			{
				demoModeCount = 0;
				demoModeState++;
				if(demoModeState == kTestVoltages.size())
					demoModeState = 0;
			}
			LedSlider& sl = ledSliders.sliders[0];
			float v = kTestVoltages[demoModeState];
			if(sl.getNumTouches()) // draw voltage quantised to 1V
				v = round(normToVfs(sl.compoundTouchLocation()));
			float out = vToOut(v);
			gManualAnOut[0] = out;
			gManualAnOut[1] = out;
			if(CalibrationProcedure::kDone == calibrationState)
				animation = kGlow;
			else
				animation = kStatic;
			begin = out * (np.getNumPixels() - 1);
			end = begin + 1;
		}
			break;
		}
		float gain;
		constexpr size_t kPeriod = 500;
		rgb_t color = {60, 0, 60};
		rgb_t otherColor = {0, 0, 0};
		switch(animation)
		{
		default:
		case kBlink:
			gain = ((tick - startTime) % kPeriod) > kPeriod / 2;
			break;
		case kGlow:
			gain = simpleTriangle(tick, kPeriod);
			otherColor = baseColor;
			break;
		case kStatic:
			gain = 1;
			break;
		}
		color = crossfade(otherColor, color, gain);
		if(ledSliders.areLedsEnabled())
		{
			np.clear();
			for(size_t n = begin; n < np.getNumPixels() && n < end; ++n)
				np.setPixelColor(n, color.r, color.g, color.b);
		}
	}
	void updatePreset() override
	{
		gCalibrationProcedure.updatePreset();
	}
} gCalibrationMode(kCalibrationColor);

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
	&gCalibrationMode,
};
static const ssize_t kCalibrationModeIdx = []{
		auto it = std::find(performanceModes.begin(), performanceModes.end(), &gCalibrationMode);
		int idx = -1;
		if(it != performanceModes.end())
			idx = it - performanceModes.begin();
		assert(idx >= 0);
		// while we are at it, a sanity check that all modes have been init'ed
		for(auto& m : performanceModes)
			assert(m);
		return idx;
}();

bool performanceMode_setup(double ms)
{
	if(gNewMode < kNumModes && performanceModes[gNewMode])
		return performanceModes[gNewMode]->setup(ms);
	else
		return true;
}

void performanceMode_render(BelaContext* context, FrameData* frameData)
{
	// these are set here but may be overridden by render() below
	gOutUsesCalibration = true;
	gInUsesCalibration = true;
	gInUsesRange = true;
	gOutUsesRange = {true, true};
	// call the processing callback
	if(gNewMode < kNumModes && performanceModes[gNewMode])
		performanceModes[gNewMode]->render(context, frameData);
	// make the final states visible to the wrapper
	gOutRangeTop.enabled = gOutUsesRange[0];
	gOutRangeBottom.enabled = gOutUsesRange[1];
	// note: this will only take effect from the next time this function is called,
	// because obviously input range processing has already been done  by time this
	// function is called. This is not an issue normally, as long as the processing
	// callbacks know about that.
	gInRange.enabled = gInUsesRange;
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
		if(SplitPerformanceMode::kModeNoSplit == value)
			color = color0;
		else {
			float coeff0 = simpleTriangle(ms, 1000);
			float coeff1 = 1.f - coeff0;
			rgb_t otherColor = (SplitPerformanceMode::kModeSplitLocation == value) ? color1 : rgb_t{0, 0, 0};
			color.r = coeff0 * color0.r + coeff1 * otherColor.r;
			color.g = coeff0 * color0.g + coeff1 * otherColor.g;
			color.b = coeff0 * color0.b + coeff1 * otherColor.b;
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
		rgb_t otherColor;
		if(0 == value) // kAutoLatchOff,
		{
			otherColor = {0, 0, 0};
		} else if (1 == value) { // kAutoLatchBoth
			otherColor = color;
		} else { // kAutoLatchLocationOnly
			for(size_t n = 0; n < otherColor.size(); ++n)
				otherColor[n] = color[n] * 0.6;
		}
		// PWM with increasingly longer width
		const unsigned int period = 600;
		const unsigned int numPeriods = 5;
		ms = ms % (period * numPeriods);
		unsigned int thisPeriod = ms / period;
		unsigned int pulseWidth = thisPeriod / float(numPeriods - 1) * period;
		unsigned int idx = ms % period;
		bool coeff = idx < pulseWidth;
		rgb_t c;
		for(size_t n = 0; n < c.size(); ++n)
			c[n] = color[n] * coeff+ otherColor[n] * !coeff;
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
	void process(uint32_t ms, LedSlider& ledSlider, float) override {
		ledSlider.setColor(color);
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
			// in case we haven't been here in a while, we fix it quickly
			// TODO: a proper setup() call to set lastMs
			if(phase > finalPeriod * 2)
				phase = 0;

			// input mode: clock.
			// over duration, show pulses with ramp up-ramp down period(constant width),
			// to give the idea of frequency control
			phase += (ms - lastMs); // this has to be uint to be deterministic on overflow.
			lastMs = ms;
			ms %= duration;
			float triangle = simpleTriangle(ms, duration);
			float period = initialPeriod + (sqrt(triangle)) * (finalPeriod - initialPeriod);
			while(phase > period) // wrap around
				phase -= period;
			coeff = (phase < onTime);
		} else if (2 == value){
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
	static constexpr float initialPeriod = 600;
	static constexpr float finalPeriod = 80;
	static constexpr unsigned int duration = 3000;
	static constexpr float onTime = 40;
	float phase = 0;
	uint32_t lastMs = 0;
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

class ButtonAnimationSmoothQuantised : public ButtonAnimation {
public:
	ButtonAnimationSmoothQuantised(rgb_t color) :
		color(color) {}
	void process(uint32_t ms, LedSlider& ledSlider, float value) override {
		const unsigned int period = 1500;
		float coeff = simpleTriangle(ms, period);
		if(0 == value) // smooth
		{
			// nothing to do. Just smooth.
		} else { // quantised
			float steps = 4;
			coeff = int(coeff * steps + 0.5f) / steps;
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
			centroid_t centroid;
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
			centroid_t frame {
				.location = slider.compoundTouchLocation(),
				.size = slider.compoundTouchSize(),
			};
			// for some reason we are called with location and size = 0 at least once
			// When we start.
			// Work around that by checking for frame.size before starting
			// TODO: figure out the root cause and fix it
			if(!hasDoneSetup && frame.size)
			{
				// this can't be moved to constructor because
				// we don't have the slider there.
				tracking = false;
				initialPos = frame.location;
				hasDoneSetup = true;
				initialTime = HAL_GetTick();
			}
			if(!tracking)
			{
				// check if we crossed the initial point
				float refPos = parameter->get();
				float current = frame.location;
				if(
						(initialPos <= refPos && current >= refPos) ||
						(initialPos >= refPos && current <= refPos)
					)
					tracking = true;
			}
			bool latched = false;
			gMenuAutoLatcher.process(frame, latched);
			// set the centroid position to whatever the current parameter value is
			centroid_t centroid = {
					.location = parameter->get(),
					.size = kFixedCentroidSize / 2,
			};
			if(tracking) {
				// only track the slider if we have at some point crossed the initial point
				parameter->set(frame.location);
			} else {
				// or show a pulsating centroid
				centroid.size *= simpleTriangle(HAL_GetTick() - initialTime, 130);
			}
			ledSlidersAlt.sliders[0].setLedsCentroids(&centroid, 1);

			if(latched)
				menu_up();
		}
	}
	ParameterContinuous* parameter;
	//below are init'd to avoid warning
	float initialPos = 0;
	uint32_t initialTime = 0;
	bool tracking = false;
	bool hasDoneSetup = false;
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
			centroid_t frame {
				.location = slider.compoundTouchLocation(),
				.size = slider.compoundTouchSize(),
			};
			bool latched = false;
			gMenuAutoLatcher.process(frame, latched);
			float pos = fix(frame.location);
			unsigned int n = (unsigned int)(pos * quantised);
			pos = n / float(quantised);
			if(latched)
			{
				parameter->set(n);
				menu_up();
			}
			centroid_t centroid = {
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

#include <functional>


class MenuItemTypeRange : public MenuItemType {
public:
	static constexpr size_t kNumEnds = 2;
	typedef std::function<std::array<float,kNumEnds>(const std::array<float,kNumEnds>&)> PreprocessFn;
	MenuItemTypeRange(): MenuItemType({0, 0, 0}) {}
	MenuItemTypeRange(const rgb_t& color, const rgb_t& otherColor, bool autoExit, ParameterContinuous* paramBottom, ParameterContinuous* paramTop, PreprocessFn preprocess) :
		MenuItemType(color), otherColor(otherColor), preprocessFn(preprocess), parameters({paramBottom, paramTop}), autoExit(autoExit)
	{
		latchProcessor.reset();
		for(size_t n = 0; n < kNumEnds; ++n)
		{
			pastFrames[n].location = parameters[n]->get();
			pastFrames[n].size = 1;
		}
		std::array<LatchProcessor::Reason,2> isLatched;
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
				std::array<centroid_t,kNumEnds> frames;
				if(0 == numTouches) {
					frames = pastFrames;
				} else if(1 == numTouches)
				{
					// find which existing values this is closest to
					float current = slider.touchLocation(0);
					std::array<float,kNumEnds> diffs;
					for(size_t n = 0; n < kNumEnds; ++n)
						diffs[n] = std::abs(current - pastFrames[n].location);
					// TODO: if preprocessing (e.g.: quantisation) is applied
					// before this stage, then (diffs[0] > diffs[1]) may
					// return the opposite of the expected value
					// which makes it so that the min endpoint becomes higher
					// than the max endpoint.

					// the only touch we have is controlling the one that was closest to it
					validTouch = (diffs[0] > diffs[1]);
					// put the good one where it belongs
					frames[validTouch].location = slider.touchLocation(0);
					frames[validTouch].size = slider.touchSize(0);
					// and a non-touch on the other one
					frames[!validTouch].location = 0;
					frames[!validTouch].size = 0;
				} else if (2 == numTouches)
				{
					for(size_t n = 0; n < kNumEnds; ++n)
					{
						frames[n].location = slider.touchLocation(n);
						frames[n].size = slider.touchSize(n);
					}
				}
				std::array<LatchProcessor::Reason,2> isLatched;
				latchProcessor.process(true, frames.size(), frames, isLatched);
				auto preprocessedValues = preprocess({frames[0].location, frames[1].location});
				for(size_t n = 0; n < frames.size(); ++n)
				{
					float preprocessedValue = preprocessedValues[n];
					parameters[n]->set(preprocessedValue);
					// TODO: is it really the best decision to store a preprocessed (e.g.: quantised)
					// value and then use a non-preprocessed value at the next frame to check which
					// touch is closest?
					displayLocations[n] = preprocessedValue;
					pastFrames[n] = frames[n];
				}
			}
			updateDisplay(slider);
		}
	}
protected:
	std::array<centroid_t,kNumEnds> pastFrames;
	std::array<float,kNumEnds> displayLocations;
	rgb_t otherColor;
	PreprocessFn preprocessFn;
private:
	std::array<float,kNumEnds> preprocess(const std::array<float,kNumEnds>& in)
	{
		if(preprocessFn)
			return preprocessFn(in);
		else
			return in;
	}
	virtual void updateDisplay(LedSlider& slider)
	{
		slider.directBegin();
		slider.directWriteCentroid({ displayLocations[0], 0.15 }, otherColor);
		slider.directWriteCentroid({ displayLocations[1], 0.15 }, baseColor);
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
	MenuItemTypeRangeDisplayCentroids(const rgb_t& displayColor, const std::array<rgb_t,kNumEnds>& endpointsColor, bool autoExit,
				ParameterContinuous* paramBottom,ParameterContinuous* paramTop, PreprocessFn preprocess, const float& display) :
		MenuItemTypeRange(endpointsColor[0], endpointsColor[1], autoExit, paramBottom, paramTop, preprocess), displayColor(displayColor), endpointsColor(endpointsColor), display(&display) {}
	void updateDisplay(LedSlider& slider) override
	{
		std::array<centroid_t,2> endpointsCentroids {
			centroid_t{ pastFrames[0].location, 0.1 },
			centroid_t{ pastFrames[1].location, 0.1 },
		};
		slider.directBegin();
		for(size_t n = 0; n < endpointsColor.size(); ++n)
			slider.directWriteCentroid(endpointsCentroids[n], endpointsColor[n]);
		slider.directWriteCentroid({ *display, 0.15 }, displayColor);
	}
private:
	rgb_t displayColor;
	std::array<rgb_t,kNumEnds> endpointsColor; // TODO: delegate drawing endpoints to MenuItemTypeRange::updateDisplay() and avoid caching these colors here
	const float* display;
};

class MenuItemTypeDisplayRangeRaw : public MenuItemType {
public:
	MenuItemTypeDisplayRangeRaw(): MenuItemType({0, 0, 0}) {}
	MenuItemTypeDisplayRangeRaw(const rgb_t& color, const rgb_t& otherColor, float bottom, float top) :
		MenuItemType(color), bottom(bottom), top(top), otherColor(otherColor) {}
	void process(LedSlider& slider) override
	{
		np.clear();
		size_t start = std::round((kNumLeds - 1) * bottom);
		size_t stop = std::round((kNumLeds - 1) * top);
		for(size_t n = start; n <= stop; ++n)
		{
			rgb_t pixel;
			float rel = (n - start) / float(stop - start);
			if(stop == start)
				rel = 0.5; // ensure some mixing happens
			for(size_t c = 0; c < pixel.size(); ++c)
				pixel[c] = baseColor[c] * rel + otherColor[c] * (1.f - rel);
			np.setPixelColor(n, pixel.r, pixel.g, pixel.b);
		}
		if(HAL_GetTick() - startMs >= kMaxMs)
			menu_up();
	}
private:
	float bottom;
	float top;
	rgb_t otherColor;
	static constexpr uint32_t kMaxMs = 800;
	uint32_t startMs = HAL_GetTick();
};

class MenuItemTypeDisplayScaleMeterOutputMode : public MenuItemType {
public:
	MenuItemTypeDisplayScaleMeterOutputMode(): MenuItemType({0, 0, 0}) {}
	MenuItemTypeDisplayScaleMeterOutputMode(const rgb_t& color, bool bottomEnv, bool topEnv) :
		MenuItemType(color), isEnv({bottomEnv, topEnv}) {}
	void process(LedSlider& slider) override
	{
		std::array<centroid_t,kNumSplits> centroids;
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

static void requestIncMode()
{
	int newMode = (gNewMode + 1) % kNumModes;
	if(kCalibrationModeIdx == newMode)
		 // calibration mode is skipped when incrementing
		newMode = (newMode + 1) % kNumModes;
	requestNewMode(newMode);
}

class MenuItemTypeNextMode : public MenuItemTypeEvent
{
public:
	MenuItemTypeNextMode(const char* name, rgb_t baseColor) :
		MenuItemTypeEvent(name, baseColor, 3000) {}
private:
	void event(Event e) override
	{
		if(kTransitionFalling == e)
		{
			if(!ignoreNextRelease)
				requestIncMode();
			ignoreNextRelease = false;
		}
		if(kHoldHigh == e) {
			ignoreNextRelease = true;
			requestNewMode(kCalibrationModeIdx);
			// as a special case, exit menu when entering calibration
			menu_up();
		}
	}
	bool ignoreNextRelease = false;
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
	MenuItemTypeEnterRange(const char* name, rgb_t baseColor, rgb_t otherColor, ParameterContinuous& bottom, ParameterContinuous& top, MenuItemTypeRange::PreprocessFn preprocess) :
		MenuItemTypeEnterSubmenu(name, baseColor, 500, singleSliderMenu), bottom(bottom), top(top), preprocess(preprocess), otherColor(otherColor) {}
	void event(Event e)
	{
		if(kHoldHigh == e) {
			singleRangeMenuItem = MenuItemTypeRange(baseColor, otherColor, true, &bottom, &top, preprocess);
			menu_in(singleRangeMenu);
		}
	}
	ParameterContinuous& bottom;
	ParameterContinuous& top;
	MenuItemTypeRange::PreprocessFn preprocess;
	rgb_t otherColor;
};

class MenuItemTypeDiscretePlus : public MenuItemTypeEvent
{
public:
	MenuItemTypeDiscretePlus(const char* name, rgb_t baseColor, ParameterEnum& valueEn, uint32_t displayOldValueTimeout = 0):
		MenuItemTypeEvent(name, baseColor, 1000), valueEn(valueEn), displayOldValueTimeout(displayOldValueTimeout) {}
	void event(Event e) override
	{
		switch (e)
		{
		case kTransitionFalling:
			if(!ignoreNextTransition)
			{
				// this one is on release so we avoid a spurious trigger when holding
				bool shouldUpdate = true;
				if(displayOldValueTimeout)
				{
					// if we haven't been tapped in a while, do nothing.
					// An inheriting class can leverage this to display
					// the current value
					uint32_t tick = HAL_GetTick();
					if(tick - lastTick > displayOldValueTimeout)
						shouldUpdate = false;
					lastTick = tick;
				}
				if(shouldUpdate)
				{
					valueEn.next();
					printf("DiscretePlus: next to %d\n\r", valueEn.get());
				}
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
	uint32_t lastTick = 0;
	uint32_t displayOldValueTimeout;
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
	MenuItemTypeDiscreteRange(const char* name, rgb_t baseColor, rgb_t otherColor, ParameterEnum& valueEn, ParameterContinuous& valueConBottom, ParameterContinuous& valueConTop, MenuItemTypeRange::PreprocessFn preprocess, uint32_t doNotUpdateTimeout = 0):
		MenuItemTypeDiscretePlus(name, baseColor, valueEn, doNotUpdateTimeout), valueConBottom(valueConBottom), valueConTop(valueConTop), preprocess(preprocess), otherColor(otherColor) {}
	void enterPlus() override
	{
		printf("DiscreteRange: going to range\n\r");
		singleRangeMenuItem = MenuItemTypeRange(baseColor, otherColor, true, &valueConBottom, &valueConTop, preprocess);
		menu_in(singleRangeMenu);
	}
	ParameterContinuous& valueConBottom;
	ParameterContinuous& valueConTop;
	MenuItemTypeRange::PreprocessFn preprocess;
	rgb_t otherColor;
};

class MenuItemTypeDiscreteRangeCv : public MenuItemTypeDiscreteRange
{
public:
	MenuItemTypeDiscreteRangeCv(const char* name, rgb_t baseColor, rgb_t otherColor, ParameterEnum& valueEn, ParameterContinuous& valueConBottom, ParameterContinuous& valueConTop, MenuItemTypeRange::PreprocessFn preprocess):
		MenuItemTypeDiscreteRange(name, baseColor, otherColor, valueEn, valueConBottom, valueConTop, preprocess, 2000), otherColor(otherColor) {}
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
			rgb_t secondaryColor = baseColor;
			switch(valueEn.get())
			{
			case kCvRangeBipolar:
				bottom = kMinus5;
				top = kPlus5;
				break;
			case kCvRangeCustom:
				bottom = valueConBottom;
				top = valueConTop;
				secondaryColor = otherColor;
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
			const float kMargin = 0.05; // the LEDs spill up and down a bit, so we limit them a bit to get a more "accurate" visualsation
			menu_enterDisplayRangeRaw(baseColor, secondaryColor, bottom + kMargin, top - kMargin);
		}
	}
private:
	rgb_t otherColor;
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
static MenuItemTypeDiscrete directControlModeSplit("directControlModeSplit", buttonColor, &gDirectControlMode.splitMode, &animationSplit);
static MenuItemTypeDiscrete directControlModeLatch("directControlModeAutoLatch", buttonColor, &gDirectControlMode.autoLatch, &animationPulsatingStill);
static std::array<MenuItemType*,kMaxModeParameters> directControlModeMenu = {
		&disabled,
		&directControlModeLatch,
		&directControlModeSplit,
};

static ButtonAnimationSingleRepeatedEnv animationSingleRepeatedPulse{buttonColor};
static MenuItemTypeDiscrete recorderModeSplit("recorderModeSplit", buttonColor, &gRecorderMode.splitMode, &animationSplit);
static MenuItemTypeDiscrete recorderModeRetrigger("recorderModeRetrigger", buttonColor, &gRecorderMode.autoRetrigger, &animationSingleRepeatedPulse);
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
static MenuItemTypeDiscrete balancedOscModeWaveform("balancedOscModeWaveform", buttonColor, &gBalancedOscsMode.waveform, &animationWaveform);
static ButtonAnimationRecorderInputMode animationBalancedOscsInputMode{buttonColor};
static MenuItemTypeDiscreteContinuous balancedOscModeInputModeAndFrequency("balancedOscModeInputModeAndFrequency", buttonColor,
		gBalancedOscsMode.inputMode, gBalancedOscsMode.centreFrequency, &animationBalancedOscsInputMode);
static std::array<MenuItemType*,kMaxModeParameters> balancedOscsModeMenu = {
		&disabled,
		&balancedOscModeInputModeAndFrequency,
		&balancedOscModeWaveform,
};

static ButtonAnimationSmoothQuantised animationSmoothQuantised {buttonColor};
static ButtonAnimationTriangle animationTriangleExprButtonsModRange(buttonColor, 3000);
static MenuItemTypeDiscrete exprButtonsModeQuantised("gExprButtonsModeQuantised", buttonColor, &gExprButtonsMode.quantised, &animationSmoothQuantised);
static MenuItemTypeEnterContinuous exprButtonsModeModRange("gExprButtonsModeQuantisedModRange", buttonColor, gExprButtonsMode.modRange, &animationTriangleExprButtonsModRange);

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

static std::array<MenuItemType*,kMaxModeParameters> emptyModeMenu = {
		&disabled,
		&disabled,
		&disabled,
};

static std::array<std::array<MenuItemType*,kMaxModeParameters>*,kNumModes> modesMenuItems = {
#ifdef TEST_MODE
		&emptyModeMenu, // test mode
#endif // TEST_MODE
		&directControlModeMenu,
		&recorderModeMenu,
		&scaleMeterModeMenu,
		&balancedOscsModeMenu,
		&exprButtonsModeMenu,
		&emptyModeMenu, // calibration mode
};

MenuItemTypeNextMode nextMode("nextMode", {0, 255, 0});
MenuItemTypeExitSubmenu exitMe("exit", {127, 255, 0});

static void setAllSizeScales(float coeff)
{
	// adjust size on current sliders
	globalSlider.setSizeScale(coeff);
	for(auto& ls : ledSliders)
		ls.setSizeScale(coeff);
	for(auto& ls : ledSlidersAlt)
		ls.setSizeScale(coeff);
	// adjust size on future slider
	gSizeScale = coeff;
}

static void doOverride(size_t c, float value, bool bypassRange, bool isSize)
{
	gOverride.started = HAL_GetTick();
	gOverride.out = value;
	gOverride.ch = c;
	gOverride.bypassOutRange = bypassRange;
	gOverride.isSize = isSize;
}

static void doOutRangeOverride(size_t c)
{
	// generate a cheap square of period 2.048 seconds and reset phase on change
	static size_t pastC = -1;
	static uint32_t startTick = 0;
	static std::array<IoRange,2> pastRanges {
		gOutRangeTop,
		gOutRangeBottom,
	};
	constexpr size_t kFullPeriod = 2048; // has to be a power of 2

	uint32_t tick = HAL_GetTick();
	const std::array<const IoRange*,2> ranges {
		&gOutRangeTop,
		&gOutRangeBottom,
	};
	// detect a change so we can play back first the one that has changed
	ssize_t changedEndpoint = -1;
	if(ranges[c]->min != pastRanges[c].min)
		changedEndpoint = 0;
	else if(ranges[c]->max != pastRanges[c].max)
		changedEndpoint = 1;

	if(changedEndpoint != -1 || pastC != c)
	{
		// reset phase on change
		startTick = tick;
		if(1 == changedEndpoint)
		{
			// if the second endpoint changed,
			// increment the phase by half period so
			// that we start playing back the high part
			startTick -= kFullPeriod / 2;
		}
	}
	bool square = ((tick - startTick) & (kFullPeriod -1)) >= kFullPeriod / 2;
	float value = square ? ranges[c]->max : ranges[c]->min;
	doOverride(c, value, true, false);
	for(size_t n = 0; n < pastRanges.size(); ++n)
		pastRanges[n] = *ranges[n];
	pastC = c;
}

static MenuItemTypeEnterSubmenu enterGlobalSettings("GlobalSettings", {120, 120, 0}, 20, globalSettingsMenu);
class GlobalSettings : public ParameterUpdateCapable {
public:
	void updated(Parameter& p)
	{
		bool verbose = false;
		char const* str = "+++";
		if(p.same(outRangeTopEnum)) {
			gOutRangeTop.range = CvRange(outRangeTopEnum.get());
			str = "outRangeTopEnum";
		}
		else if(p.same(outRangeTopMin) || p.same(outRangeTopMax)) {
			outRangeTopEnum.set(kCvRangeCustom);
			gOutRangeTop.range = CvRange(outRangeTopEnum.get());
			gOutRangeTop.min = outRangeTopMin;
			gOutRangeTop.max = outRangeTopMax;
			str = "outRangeTopMin/Max";
			doOutRangeOverride(0);
		}
		else if(p.same(outRangeBottomEnum)) {
			gOutRangeBottom.range = CvRange(outRangeBottomEnum.get());
			str = "outRangeBottomEnum";
		}
		else if(p.same(outRangeBottomMin) || p.same(outRangeBottomMax)) {
			outRangeBottomEnum.set(kCvRangeCustom);
			gOutRangeBottom.range = CvRange(outRangeBottomEnum.get());
			gOutRangeBottom.min = outRangeBottomMin;
			gOutRangeBottom.max = outRangeBottomMax;
			str = "outRangeBottomMin/Max";
			doOutRangeOverride(1);
		}
		else if(p.same(inRangeEnum)) {
			str = "inRangeEnum";
			gInRange.range = inRangeEnum;
		}
		else if(p.same(inRangeMax) || p.same(inRangeMin)) {
			inRangeEnum.set(kCvRangeCustom);
			gInRange.range = inRangeEnum;
			gInRange.min = inRangeMin;
			gInRange.max = inRangeMax;
			str = "inRangeTop/Bottom";
		} else if(p.same(jacksOnTop)) {
			gJacksOnTop = jacksOnTop;
			str = "jacksOnTop";
		}
		else if(p.same(sizeScaleCoeff)) {
			str = "sizeScaleCoeff";
			float tmp = (powf(2, 0.5 + sizeScaleCoeff) - 1);
			float coeff = kSizeScale / (tmp * tmp * tmp);
			setAllSizeScales(coeff);
			doOverride(1, globalSlider.compoundTouchSize(), false, true);
		}
		else if(p.same(newMode)) {
			str = "newMode";
			requestNewMode(newMode);
		}
		if(verbose)
			printf("%s\n\r", str);
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD12(outRangeTopMin, outRangeTopMax, outRangeTopEnum,
				outRangeBottomMin, outRangeBottomMax, outRangeBottomEnum,
				inRangeMin, inRangeMax, inRangeEnum,
					sizeScaleCoeff, jacksOnTop, newMode);
	}
	GlobalSettings() :
		presetFieldData {
			.outRangeTopMin = outRangeTopMin,
			.outRangeTopMax = outRangeTopMax,
			.outRangeBottomMin = outRangeBottomMin,
			.outRangeBottomMax = outRangeBottomMax,
			.inRangeMin = inRangeMin,
			.inRangeMax = inRangeMax,
			.sizeScaleCoeff = sizeScaleCoeff,
			.outRangeTopEnum = outRangeTopEnum,
			.inRangeEnum = inRangeEnum,
			.jacksOnTop = jacksOnTop,
			.newMode = newMode,
		}
	{
		PresetDesc_t presetDesc = {
			.field = this,
			.size = sizeof(PresetFieldData_t),
			.defaulter = genericDefaulter12(GlobalSettings, outRangeTopMin, outRangeTopMax, outRangeTopEnum,
					outRangeBottomMin, outRangeBottomMax, outRangeBottomEnum,
					inRangeMin, inRangeMax, inRangeEnum,
						sizeScaleCoeff, jacksOnTop, newMode),
			// currently the {out,in}RangeEnums have to go after the corresponding
			// corresponding Range{Bottom,Top}, as setting the Range last would otherwise
			// reset the enum
			// TODO: make this more future-proof
			.loadCallback = genericLoadCallback12(GlobalSettings, outRangeTopMin, outRangeTopMax, outRangeTopEnum,
							outRangeBottomMin, outRangeBottomMax, outRangeBottomEnum,
							inRangeMin, inRangeMax, inRangeEnum,
								sizeScaleCoeff, jacksOnTop, newMode),
		};
		presetDescSet(6, &presetDesc);
	}
	ParameterEnumT<kCvRangeNum,CvRange> outRangeTopEnum {this, kCvRangePositive10};
	ParameterContinuous outRangeTopMin {this, 0.2};
	ParameterContinuous outRangeTopMax {this, 0.8};
	ParameterEnumT<kCvRangeNum,CvRange> outRangeBottomEnum {this, kCvRangePositive10};
	ParameterContinuous outRangeBottomMin {this, 0.2};
	ParameterContinuous outRangeBottomMax {this, 0.8};
	ParameterEnumT<kCvRangeNum,CvRange> inRangeEnum {this, kCvRangePositive10};
	ParameterContinuous inRangeMin {this, 0.2};
	ParameterContinuous inRangeMax {this, 0.8};
	ParameterContinuous sizeScaleCoeff {this, 0.5};
	ParameterEnumT<2> jacksOnTop {this, false};
	ParameterEnumT<kNumModes> newMode{this, gNewMode};
	PACKED_STRUCT(PresetFieldData_t {
		float outRangeTopMin;
		float outRangeTopMax;
		float outRangeBottomMin;
		float outRangeBottomMax;
		float inRangeMin;
		float inRangeMax;
		float sizeScaleCoeff;
		uint8_t outRangeTopEnum;
		uint8_t outRangeBottomEnum;
		uint8_t inRangeEnum;
		uint8_t jacksOnTop;
		uint8_t newMode;
	}) presetFieldData;
} gGlobalSettings;

static void requestNewMode(int mode)
{
	bool different = (gNewMode != mode);
	gNewMode = mode;
	// notify the setting that is stored to disk (unless calibration),
	// but avoid the set() to trigger a circular call to requestNewMode()
	if(different && mode != kCalibrationModeIdx)
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
static std::array<float,MenuItemTypeRange::kNumEnds> quantiseNormalisedForIntegerVolts(const std::array<float,MenuItemTypeRange::kNumEnds>& in)
{
	static constexpr float kVoltsFs = 15;
	std::array<float,MenuItemTypeRange::kNumEnds> out;
	// first we make them integers, one per each voltage step
	for(size_t n = 0; n < in.size(); ++n)
		out[n] = std::round((in[n] * kVoltsFs));
	// ensure they are in the correct order
	std::sort(out.begin(), out.end());
	if(out[0] == out[1])
	{
		// if the are the same, we
		// ensure they are at least 1 apart.
		int val = out[0];
		if(0 == val) //edge case
			out[1] = 1;
		else if (kVoltsFs == val) //edge case
			out[0] = kVoltsFs - 1;
		else //general case
			out[1] = out[0] + 1;
	}
	// convert back to normalised
	for(auto& o : out)
		o /= kVoltsFs;
	return out;
}
static constexpr rgb_t globalSettingsRangeOtherColor = {255, 0, 0};
static MenuItemTypeDiscreteRangeCv globalSettingsOutTopRange("globalSettingsOutTopRange", globalSettingsColor, globalSettingsRangeOtherColor, gGlobalSettings.outRangeTopEnum, gGlobalSettings.outRangeTopMin, gGlobalSettings.outRangeTopMax, quantiseNormalisedForIntegerVolts);
static MenuItemTypeDiscreteRangeCv globalSettingsOutBottomRange("globalSettingsOutBottomRange", globalSettingsColor, globalSettingsRangeOtherColor, gGlobalSettings.outRangeBottomEnum, gGlobalSettings.outRangeBottomMin, gGlobalSettings.outRangeBottomMax, quantiseNormalisedForIntegerVolts);
static MenuItemTypeDiscreteRangeCv globalSettingsInRange("globalSettingsInRange", globalSettingsColor, globalSettingsRangeOtherColor, gGlobalSettings.inRangeEnum, gGlobalSettings.inRangeMin, gGlobalSettings.inRangeMax, quantiseNormalisedForIntegerVolts);
static ButtonAnimationTriangle animationTriangleGlobal(globalSettingsColor, 3000);
static MenuItemTypeEnterContinuous globalSettingsSizeScale("globalSettingsSizeScale", globalSettingsColor, gGlobalSettings.sizeScaleCoeff, &animationTriangleGlobal);
static constexpr rgb_t jacksOnTopButtonColor {0, 0, 180};
static ButtonAnimationBrightDimmed animationBrightDimmed(jacksOnTopButtonColor);
static MenuItemTypeEnterQuantised globalSettingsJacksOnTop("globalSettingsJacksOnTop", jacksOnTopButtonColor, gGlobalSettings.jacksOnTop, &animationBrightDimmed);

static bool menuJustEntered;

static void menu_enterRangeDisplay(const rgb_t& signalColor, const std::array<rgb_t,2>& endpointsColors, bool autoExit, ParameterContinuous& bottom, ParameterContinuous& top, const float& display)
{
	gAlt = 1;
	singleRangeDisplayMenuItem = MenuItemTypeRangeDisplayCentroids(signalColor, endpointsColors, autoExit, &bottom, &top, nullptr, display);
	menu_in(singleRangeDisplayMenu);
}

static void menu_enterDisplayRangeRaw(const rgb_t& color, const rgb_t& otherColor, float bottom, float top)
{
	gAlt = 1;
	displayRangeRawMenuItem = MenuItemTypeDisplayRangeRaw(color, otherColor, bottom, top);
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
			&globalSettingsJacksOnTop,
			&globalSettingsSizeScale,
			&globalSettingsOutBottomRange,
			&globalSettingsOutTopRange,
			&globalSettingsInRange,
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
	std::array<MenuItemType*,kMaxModeParameters>* menuItems;
	if(gNewMode <= modesMenuItems.size() && modesMenuItems[gNewMode])
		menuItems = modesMenuItems[gNewMode];
	else
		menuItems = &emptyModeMenu;
	for(size_t n = 0; n < kMaxModeParameters; ++n)
	{
		// make sure we are displaying the buttons for the current mode
		// if hasChanged, this will retrigger a new drawing of the buttons below
		// TODO: when refactoring mode switching, maybe ensure the menu's content and visualisation
		// gets updated directly when updating mode

		if(mainMenu.items[1 + n] != (*menuItems)[n])
		{
			MenuItemType* newItem = (*menuItems)[n];
			// validate all items before adding them
			if(!newItem)
				newItem = &disabled;
			mainMenu.items[1 + n] = newItem;
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
				true,
				1
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
					ledMode = LedSlider::MANUAL_CENTROIDS;
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

void menu_render(BelaContext*, FrameData* frameData)
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
	static uint32_t pastFrameId = 0;
	if(pastFrameId != frameData->id)
		ledSlidersAlt.process(trill.rawData.data());
	pastFrameId = frameData->id;
	// update touches
	if(menuJustEntered)
	{
		// if we just entered the menu, ensure we have removed
		// all fingers once before enabling interaction
		if(globalSlider.getNumTouches())
			return;
		menuJustEntered = false;
	}
	for(size_t n = 0; n < ledSlidersAlt.sliders.size(); ++n)
		activeMenu->items[n]->process(ledSlidersAlt.sliders[n]);
}
