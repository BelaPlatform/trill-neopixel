#include "TrillRackInterface.h"
#include "GlissModes.h"
#include <vector>
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include <libraries/Oscillator/Oscillator.h>
#include "LedSliders.h"
#include "preset.h"
#include "packed.h"
#include "bootloader.h"

static constexpr size_t kNumSplits = 2;
float gBrightness = 1;
bool gModeWantsInteractionPreMenu = false;
bool gInPreMenu = false;

static constexpr rgb_t kRgbRed {255, 0, 0};
static constexpr rgb_t kRgbGreen {0, 255, 0};
static constexpr rgb_t kRgbOrange {255, 127, 0};
static constexpr rgb_t kRgbYellow {255, 255, 0};
static constexpr rgb_t kRgbWhite {0, 50, 255};
static constexpr rgb_t kRgbBlack {0, 0, 0};
static constexpr float kMenuButtonDefaultBrightness = 0.2;
static constexpr float kMenuButtonActiveBrightness = 0.7;
static constexpr float kDefaultThreshold = 0.03;

// When a WithFs kAnimationMode is enabled, there is a kPostAnimationTimeoutMs during
// which if you tap the selector, it is going to advance to the next value.
// If in kAnimationModeSolidGlowingWithFs, during this period (or actually slightly less than that),
// the button  will glow to indicate it is "active". The "slightly less" is a perhaps unnecessary
// subtlety so that the glowing is slightly shorter than the actual active time of the selector,
// so if you see it glowing, by the time you tap it it's probably still active.
static constexpr uint32_t kPostAnimationGlowingPeriod = 700;
// ensure full period so that the transition is smooth
static constexpr uint32_t kPostAnimationGlowingMs = kPostAnimationGlowingPeriod * 5;
// + 300 is the "slightly less"
static constexpr uint32_t kPostAnimationTimeoutMs = kPostAnimationGlowingMs + 300;

#define ENABLE_DIRECT_CONTROL_MODE
#define ENABLE_RECORDER_MODE
#define ENABLE_SCALE_METER_MODE
//#define ENABLE_BALANCED_OSCS_MODE
#define ENABLE_EXPR_BUTTONS_MODE
constexpr size_t kNumModes = 3 // calibration, factorytest and erasesettings are always enabled
#ifdef ENABLE_DIRECT_CONTROL_MODE
		+ 1
#endif
#ifdef ENABLE_RECORDER_MODE
		+ 1
#endif
#ifdef ENABLE_SCALE_METER_MODE
		+ 1
#endif
#ifdef ENABLE_BALANCED_OSCS_MODE
		+ 1
#endif
#ifdef ENABLE_EXPR_BUTTONS_MODE
		+ 1
#endif
#ifdef TEST_MODE
		+ 1
#endif
		; // kNumModes

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
	if(btn.tripleClickOffset)
	{
		btn.doubleClickOffset = false;
		btn.offset = false;
	}
	if(btn.doubleClickOffset)
		btn.offset = false;
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
		Id id = kIdInvalid;
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
public:
	static constexpr TouchWithId kInvalidTouch = {
		.id = kIdInvalid,
	};
private:
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
//#define S(...) __VA_ARGS__
#define S(a)
//#define M(...) __VA_ARGS__
#define M(a)
//#define printf(...) // disable printf altogether

extern int gAlt;
typedef TrillRackInterface TRI; // shorthand
extern TRI tri;
extern const unsigned int kNumLeds;
extern std::vector<unsigned int> padsToOrderMap;
extern NeoPixelT<kNumLeds> np;
extern Trill trill;
extern std::array<float,kNumOutChannels> gManualAnOut;

std::array<Oscillator, 2> oscillators;
const std::array<rgb_t, 2> gBalancedLfoColorsInit = {{kRgbYellow, kRgbGreen}};
std::array<rgb_t, 2> gBalancedLfoColors; // copy so that we can set them via MIDI without changing defaults

std::array<OutMode,kNumOutChannels> gOutMode { kOutModeManualBlock, kOutModeManualBlock };
int gCounter = 0;
int gSubMode = 0;
std::array<bool,2> gOutIsSize;
bool gJacksOnTop = true;
enum AnimationMode
{
	kAnimationModeSolid,
	kAnimationModeSolidWithFs,
	kNumAnimationMode,
	kAnimationModeSolidGlowingWithFs,
	kAnimationModeConsistent,
	kAnimationModeConsistentWithFs,
	kAnimationModeSolidDefaultWithFs,
	kAnimationModeCustom,
};
static AnimationMode gAnimationMode = kAnimationModeSolid;
static bool hasFsAnimation()
{
	switch(gAnimationMode)
	{
		case kAnimationModeConsistentWithFs:
		case kAnimationModeSolidWithFs:
		case kAnimationModeSolidDefaultWithFs:
		case kAnimationModeSolidGlowingWithFs:
			return true;
		default:
			return false;
	}
}

Override gOverride;
static bool gInUsesCalibration;
static bool gOutUsesCalibration;
static bool gInUsesRange;
static std::array<bool,kNumOutChannels> gOutUsesRange;
bool gOutAddsIn;

// Recording the gesture
static constexpr size_t kMaxRecordLength = 5000;
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

static uint32_t gClockPeriodUpdateCounter = 0;
static float gClockPeriod = 10000; // arbitrary init to avoid divisions by zero. TODO: instead check before using it
static uint64_t gClockPeriodLastUpdate = -1;

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
				gClockPeriodLastUpdate = context->audioFramesElapsed + n;
#endif // TRIGGER_IN_TO_CLOCK_USES_MOVING_AVERAGE
			}
			lastTrig = newTrig;
			lastTrigPrimed = true;
		}

		lastIn = in;
	}
}

static bool clockInIsActive(BelaContext* context)
{
	uint64_t now = context->audioFramesElapsed + context->analogFrames - 1; // rounded up to the end of the frame
	if(now < gClockPeriodLastUpdate)
		return false;
	else
		return now - gClockPeriodLastUpdate < 10.f * context->analogSampleRate;
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
			centroid.size = kMenuButtonDefaultBrightness;
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

static void ledSlidersSetupTwoSliders(unsigned int guardPads, rgb_t color, LedSlider::LedMode_t mode)
{
	ledSlidersSetupMultiSlider(ledSliders, {color, color}, mode, false, 1, kTopBottom);
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

// MODE Alt: settings UI
bool modeAlt_setup()
{
	ledSlidersSetupMultiSlider(
		ledSlidersAlt,
		{
			kRgbRed,
			kRgbRed,
			kRgbRed,
			kRgbRed,
			kRgbGreen,
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
		pastFrames.fill({0, 0});
		delay = 0;
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
		if(back >= validFrames)
			back = validFrames - 1;
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
				if(hasTouch[n])
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
class ArrayView {
public:
	ArrayView() = default;
	ArrayView(sample_t* ptr, size_t sz) : ptr(ptr), sz(sz) {}
	constexpr size_t size() const
	{
		return sz;
	}
	const sample_t* data() const
	{
		return ptr;
	}
	sample_t* data()
	{
		return ptr;
	}
	sample_t operator[](size_t n) const
	{
		return ptr[n];
	}
	sample_t& operator[](size_t n)
	{
		return ptr[n];
	}
private:
	sample_t* ptr = nullptr;
	size_t sz = 0;
};

template <typename sample_t>
class Recorder
{
public:
	struct ValidSample {
		sample_t sample;
		bool valid;
	};
	Recorder() { setup({}); }
	void setup(const ArrayView<sample_t>& newData)
	{
		start = 0;
		end = 0;
		current = 0;
		active = false;
		full = false;
		if(newData.size())
			setData(newData);
		else
			data = newData; // avoid needless recursion
	}
	void setData(const ArrayView<sample_t>& newData)
	{
		data = newData;
		// try to keep existing recording, unless it exceeds boundaries, in which case just wipe it
		if(start > data.size() || end > data.size() || current > data.size())
			setup(newData);
	}
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
		if(full){
			return {0, false};
		} else {
			data[current] = in;
			increment(current);
			// if the circular buffer becomes full, make a note of it
			if(current == start)
				full = true;
			end = current;
			return {in, true};
		}
	}
	virtual void stopRecording()
	{
		// nothing to do here at the moment. Just use start and end as set in record().
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
		size_t i = full ? size : (end - start + size) % size;
		return i;
	}
	const auto& getData()
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
	ArrayView<sample_t> data;
	size_t start;
	size_t end;
	size_t current;
	bool active;
public:
	bool full; // whether during recording the buffer becomes full
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

#ifdef ENABLE_RECORDER_MODE
static constexpr size_t kNumRecs = 4;
static std::array<float,kMaxRecordLength * kNumRecs> recorderData;
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
			rs[n].activity |= touch > 0; // TODO: if a gesture was all at exactly 0 location (unlikely), this would break
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
				else {
					if(loop) {
						// stopped but (due to a menu change) we are in looping mode now, so restart
						rs[n].playHead = 0;
					}
					out = {0, false};
				}
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
	static constexpr size_t kNumRecs = ::kNumRecs;
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
		enum WhichRecorders {
			kWhichRecordersDouble,
			kWhichRecordersDoubleWithBackup,
		};
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
		void setEnabled(WhichRecorders which)
		{
			size_t numEnabled = 0;
			switch(which)
			{
			case kWhichRecordersDouble:
				numEnabled = kNumRecs / 2;
				break;
			case kWhichRecordersDoubleWithBackup:
				numEnabled = kNumRecs;
				break;
			}
			size_t size = recorderData.size() / numEnabled;
			for(size_t n = 0; n < ptrs.size(); ++n)
			{
				size_t start = 0;
				switch(which)
				{
				case kWhichRecordersDouble:
					start = (n % numEnabled) * size;
					break;
				case kWhichRecordersDoubleWithBackup:
					if(0 == n)
						start = 0;
					else if (1 == n)
						start = size * 2;
					else if(2 == n)
						start = size;
					else if (3 == n)
						start = size * 3;
					break;
				}
				auto& r = ptrs[n]->r;
				ArrayView<sample_t> newA = { recorderData.data() + start, size};
				ArrayView<sample_t> oldA = r.getData();
				size_t count = std::min(newA.size(), oldA.size());
				if(n < kNumRecs / 2 && newA.data() != oldA.data() && newA.data() && oldA.data())
				{
					// copy data where appropriate, so that it keeps being available for the next mode.
					// TODO: could copy only the data that is actually in use (i.e.: recorder's size)
					// TODO: improve r.setData() so that it clips to new size instead of discarding
					// if past recording was larger than current
					printf("Copying %u for %d from %p to %p\n\r", count, n, oldA.data(), newA.data());
					std::copy(oldA.data(), oldA.data() + count, newA.data());
				}
				r.setData(newA);
			}
		}
		// do not implement size() to avoid confusion when
		// the user wants to call rs[n].r.size()
	};
	SwappableRecorders rs;
private:
	std::array<bool,kNumSplits> hadTouch {};
	bool lastStateChangeWasToggling = false;
} gGestureRecorder;
#endif // ENABLE_RECORDER_MODE

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
	virtual void animate(Parameter& p, LedSlider& l, rgb_t color, uint32_t ms) {};
};

template <typename T>
class ParameterGeneric : public Parameter {
public:
	ParameterGeneric<T>() {}
	ParameterGeneric<T>(ParameterUpdateCapable* that, T value):
		that(that), value(value) {}
	void set(T value)
	{
		this->value = value;
		if(that)
			that->updated(*this);
	}
	T get() const {
		return value;
	}
	operator const T&() const {
		return value;
	}
private:
	ParameterUpdateCapable* that = nullptr;
	T value;
};

class ParameterEnum : public Parameter {
public:
	virtual void set(unsigned int) = 0;
	virtual void next() = 0;
	virtual uint8_t get() const = 0;
	virtual uint8_t getMax() const = 0;
	virtual void animate(LedSlider& l, rgb_t color, uint32_t ms) {};
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
	virtual void animate(LedSlider& l, rgb_t color, uint32_t ms) override
	{
		if(hasFsAnimation())
			that->animate(*this, l, color, ms);
	}
	operator type() { return type(value); }
private:
	ParameterUpdateCapable* that;
	uint8_t value;
};

class ParameterContinuous : public Parameter {
public:
	ParameterContinuous(ParameterUpdateCapable* that, float value = 0) : that(that), value(value), defaultValue(value) {}
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
	bool isDefault(float threshold) const
	{
		return std::abs(value - defaultValue) < threshold;
	}
	void resetToDefault()
	{
		set(defaultValue);
	}
	float getDefault() const {
		return defaultValue;
	}
private:
	ParameterUpdateCapable* that;
	float value;
	const float defaultValue;
};

class AnimateFs
{
public:
static constexpr size_t kLedStart = 4;
static constexpr size_t kLedStop = kNumLeds - kLedStart - 1;
bool writeInit(Parameter& p, LedSlider& l)
{
	if(isEnabled(p))
	{
		hasWrittenInit++;
		// absolutely avoid animation from overlapping the fixed centroids at top and bottom
		// as that may unexpectedly trigger the compressor.
		// TODO: better design so we don't need this. It's probably about
		// the numWeights of directWriteCentroi.
		constexpr size_t kGuardLed = 1;
		size_t start = all ? 0 : kLedStart - kGuardLed;
		size_t stop = all ? kNumLeds : kLedStop + kGuardLed;
		for(size_t n = start; n < stop; ++n)
			np.setPixelColor(n, kRgbBlack);
	}
	return isEnabled(p);
}
void directWriteCentroid(Parameter& p, LedSlider& l, centroid_t centroid, rgb_t color, size_t numWeights = LedSlider::kDefaultNumWeights)
{
	if(!isEnabled(p))
		return;
	centroid.location = mapAndConstrain(centroid.location, 0, 1, 0.25, 0.75);
	l.directWriteCentroid(centroid, color, numWeights);
}
void setActive(Parameter& p, bool all)
{
	this->all = all;
	enabledP = &p;
}
uint32_t hasWrittenCount()
{
	return hasWrittenInit;
}

private:
bool isEnabled(Parameter& p) const
{
	return &p == enabledP;
}
Parameter* enabledP = nullptr;
uint32_t hasWrittenInit = 0;
bool all = false;
} gAnimateFs;

static float simpleRamp(unsigned int phase, unsigned int period)
{
	if(!period)
		return 0;
	return float(phase % period) / period;
}

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

class IoRangeParameters : public ParameterEnumT<kCvRangeNum,CvRange>
{
public:
	ParameterEnumT<kCvRangeNum,CvRange>& cvRange = *this; // for backwards compatibility
	ParameterContinuous min;
	ParameterContinuous max;
	operator IoRange() {
		return IoRange{
			.min = min,
			.max = max,
			.range = cvRange,
			.enabled = true,
		};
	}
	IoRangeParameters(ParameterUpdateCapable* that) :
		ParameterEnumT<kCvRangeNum,CvRange>({that, kCvRangePositive10}),
		min(that, 0),
		max(that, 1)
	{}
	virtual void animate(LedSlider& l, rgb_t baseColor, uint32_t ms) override
	{
		if(ms < 800)
		{
			if(!gAnimateFs.writeInit(*this, l))
				return;
			const rgb_t otherColor = kRgbRed;
			float bottom = 0;
			float top = 0;
			const float kMinus5 = 0;
			const float kGnd = 0.33;
			const float kPlus5 = 0.66;
			const float kPlus10 = 1;
			rgb_t secondaryColor = baseColor;

			switch(get())
			{
			case kCvRangeBipolar:
				bottom = kMinus5;
				top = kPlus5;
				break;
			case kCvRangeCustom:
				bottom = min;
				top = max;
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

			if(bottom != 0)
				bottom += kMargin;
			if(top != 1)
				top -= kMargin;
			size_t start = std::round((kNumLeds - 1) * bottom);
			size_t stop = std::round((kNumLeds - 1) * top);
			for(size_t n = start; n <= stop; ++n)
			{
				rgb_t pixel;
				float rel = (n - start) / float(stop - start);
				if(stop == start)
					rel = 0.5; // ensure some mixing happens
				for(size_t c = 0; c < pixel.size(); ++c)
					pixel[c] = baseColor[c] * rel + secondaryColor[c] * (1.f - rel);
				np.setPixelColor(n, pixel.scaledBy(0.2));
			}
		}
	}
};

class IoRangesParameters
{
public:
	IoRangeParameters in;
	IoRangeParameters outTop;
	IoRangeParameters outBottom;
	operator IoRanges()
	{
		return IoRanges{
			.in = in,
			.outTop = outTop,
			.outBottom = outBottom,
		};
	}
	IoRangesParameters(ParameterUpdateCapable* that) :
		in(that),
		outTop(that),
		outBottom(that)
	{}
	IoRangeParameters& operator[] (size_t n) {
		switch(n)
		{
		default:
		case 0:
			return in;
		case 1:
			return outTop;
		case 2:
			return outBottom;
		}
	}
	static constexpr size_t size() {
		return 3;
	}
};

static void doOutRangeOverride(size_t c);
class PerformanceMode : public ParameterUpdateCapable {
public:
	virtual bool setup(double ms) = 0;
	virtual void render(BelaContext*, FrameData* frameData) = 0;
	IoRangesParameters ioRangesParameters {this};
	virtual void updated(Parameter& p) override {
		for(size_t n = 0; n < ioRangesParameters.size(); ++n)
		{
			auto& ioRange = ioRangesParameters[n];
			if(p.same(ioRange.cvRange))
				break;
			else if(p.same(ioRange.min) || p.same(ioRange.max)) {
				ioRange.cvRange.set(kCvRangeCustom);
				if(n >= 1)
					doOutRangeOverride(n - 1);
				break;
			}
		}
	};
	PerformanceMode() : buttonColor(kRgbGreen) {}
	PerformanceMode(rgb_t color)
	: buttonColor(kRgbGreen) // TODO: change kRgbGreen to color if you want to obey the color
	{}
	rgb_t buttonColor {kRgbGreen};
};

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
			tri.buttonLedSet(TRI::kOff, TRI::kAll);
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
			tri.buttonLedSet(TRI::kSolid, TRI::kG, gn);
			tri.buttonLedSet(TRI::kSolid, TRI::kR, rd);
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
				kRgbBlack,
			},
			{
				kRgbWhite,
				kRgbWhite,
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
	bool hadTouch = false;
	bool shouldLeds = true;
	bool shouldTouch = true;
	bool shouldPwm = true;
} gTestMode;
#endif // TEST_MODE

#define type_unref(A) std::remove_reference<decltype(A)>::type
// unaligned target assign with type conversion
#define UN_T_ASS(dst, src) { \
	type_unref(dst) cpy; \
	cpy = src; /* apply any conversion */ \
	memcpy(&dst, &cpy, sizeof(dst)); \
}

// unaligned source assign
#define UN_S_ASS(dst, src) { \
	static_assert(std::is_same<type_unref(dst), type_unref(src)>::value); \
	memcpy(&dst, &src, sizeof(dst)); \
}

#define DEFAULTER_PROCESS(A) UN_T_ASS(pfd->A, that->A)
#define DEFAULTER_PROCESS_IORANGES() { \
	IoRanges ioRanges = that->ioRangesParameters; \
	UN_T_ASS(pfd->ioRanges, ioRanges); \
}

#define genericDefaulter2(CLASS,A,B) \
[](PresetField_t field, PresetFieldSize_t size, void* data) \
{ \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	DEFAULTER_PROCESS_IORANGES(); \
	DEFAULTER_PROCESS(A); \
	DEFAULTER_PROCESS(B); \
}

#define genericDefaulter3(CLASS,A,B,C) \
[](PresetField_t field, PresetFieldSize_t size, void* data) \
{ \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	DEFAULTER_PROCESS_IORANGES(); \
	DEFAULTER_PROCESS(A); \
	DEFAULTER_PROCESS(B); \
	DEFAULTER_PROCESS(C); \
}

#define genericDefaulterNoioranges3(CLASS,A,B,C) \
[](PresetField_t field, PresetFieldSize_t size, void* data) \
{ \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	DEFAULTER_PROCESS(A); \
	DEFAULTER_PROCESS(B); \
	DEFAULTER_PROCESS(C); \
}

#define genericDefaulter3PlusArrays(CLASS,A,B,C,A0,A1) \
[](PresetField_t field, PresetFieldSize_t size, void* data) \
{ \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	DEFAULTER_PROCESS_IORANGES(); \
	DEFAULTER_PROCESS(A); \
	DEFAULTER_PROCESS(B); \
	DEFAULTER_PROCESS(C); \
	for(size_t n = 0; n < that->A0.size(); ++n) \
		UN_T_ASS(pfd->A0[n], that->A0[n]); \
	for(size_t n = 0; n < that->A1.size(); ++n) \
		UN_T_ASS(pfd->A1[n], that->A1[n]); \
}

#define genericDefaulterNoioranges4(CLASS,A,B,C,D) \
[](PresetField_t field, PresetFieldSize_t size, void* data) \
{ \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	DEFAULTER_PROCESS(A); \
	DEFAULTER_PROCESS(B); \
	DEFAULTER_PROCESS(C); \
	DEFAULTER_PROCESS(D); \
}

#define genericDefaulter12(CLASS,A,B,C,D,E,F,G,H,I,J,K,L) \
[](PresetField_t field, PresetFieldSize_t size, void* data) \
{ \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	DEFAULTER_PROCESS(A); \
	DEFAULTER_PROCESS(B); \
	DEFAULTER_PROCESS(C); \
	DEFAULTER_PROCESS(D); \
	DEFAULTER_PROCESS(E); \
	DEFAULTER_PROCESS(F); \
	DEFAULTER_PROCESS(G); \
	DEFAULTER_PROCESS(H); \
	DEFAULTER_PROCESS(I); \
	DEFAULTER_PROCESS(J); \
	DEFAULTER_PROCESS(K); \
	DEFAULTER_PROCESS(L); \
}

#define LOADER_PROCESS(A) { \
		type_unref(pfd->A) a; \
		UN_S_ASS(a, pfd->A); \
		that->A.set(a); \
		that->presetFieldData.A = a; \
}

#define LOADER_PROCESS_IORANGES() { \
	IoRanges a; \
	UN_S_ASS(a, pfd->ioRanges); \
	for(size_t n = 0; n < IoRangesParameters::size(); ++n) { \
		that->ioRangesParameters[n].min.set(a[n].min); \
		that->ioRangesParameters[n].max.set(a[n].max); \
		 /* this last, so that setting min/max won't override cvRange with kCvRangeCustom */ \
		that->ioRangesParameters[n].cvRange.set(a[n].range); \
	} \
	that->presetFieldData.ioRanges = a; \
}

#define genericLoadCallback2(CLASS,A,B) \
[](PresetField_t field, PresetFieldSize_t size, const void* data) { \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	LOADER_PROCESS_IORANGES(); \
	LOADER_PROCESS(A); \
	LOADER_PROCESS(B); \
}

#define genericLoadCallback3(CLASS,A,B,C) \
[](PresetField_t field, PresetFieldSize_t size, const void* data) { \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	LOADER_PROCESS_IORANGES(); \
	LOADER_PROCESS(A); \
	LOADER_PROCESS(B); \
	LOADER_PROCESS(C); \
}

#define genericLoadCallbackNoioranges3(CLASS,A,B,C) \
[](PresetField_t field, PresetFieldSize_t size, const void* data) { \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	LOADER_PROCESS(A); \
	LOADER_PROCESS(B); \
	LOADER_PROCESS(C); \
}

#define genericLoadCallback3PlusArrays(CLASS,A,B,C,A0,A1) \
[](PresetField_t field, PresetFieldSize_t size, const void* data) { \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	LOADER_PROCESS_IORANGES(); \
	LOADER_PROCESS(A); \
	LOADER_PROCESS(B); \
	LOADER_PROCESS(C); \
	for(size_t n = 0; n < that->A0.size(); ++n) \
		LOADER_PROCESS(A0[n]); \
	for(size_t n = 0; n < that->A1.size(); ++n) \
		LOADER_PROCESS(A1[n]); \
}

#define genericLoadCallbackNoioranges4(CLASS,A,B,C,D) \
[](PresetField_t field, PresetFieldSize_t size, const void* data) { \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	LOADER_PROCESS(A); \
	LOADER_PROCESS(B); \
	LOADER_PROCESS(C); \
	LOADER_PROCESS(D); \
}

#define genericLoadCallback12(CLASS,A,B,C,D,E,F,G,H,I,J,K,L) \
[](PresetField_t field, PresetFieldSize_t size, const void* data) { \
	PresetFieldData_t* pfd = (PresetFieldData_t*)data; \
	CLASS* that = (CLASS*)field; \
	LOADER_PROCESS(A); \
	LOADER_PROCESS(B); \
	LOADER_PROCESS(C); \
	LOADER_PROCESS(D); \
	LOADER_PROCESS(E); \
	LOADER_PROCESS(F); \
	LOADER_PROCESS(G); \
	LOADER_PROCESS(H); \
	LOADER_PROCESS(I); \
	LOADER_PROCESS(J); \
	LOADER_PROCESS(K); \
	LOADER_PROCESS(L); \
}

template <typename T>
static bool areEqual(const T& a, const T& b)
{
	return !memcmp(&a, &b, sizeof(T));
}

#define UPDATE_PRESET_IORANGES() { \
	presetFieldData.ioRanges = ioRangesParameters; \
}

#define UPDATE_PRESET_IORANGES_WITH_SAME() { \
	auto ioRanges = presetFieldData.ioRanges; \
	UPDATE_PRESET_IORANGES(); \
	if(memcmp(&ioRanges, &presetFieldData.ioRanges, sizeof(ioRanges))) \
		same = false; \
}

#define UPDATE_PRESET_FIELD2(A,B) \
{ \
	PresetFieldData_t bak = presetFieldData; \
	UPDATE_PRESET_IORANGES(); \
	presetFieldData.A = A; \
	presetFieldData.B = B; \
	if(!areEqual(bak, presetFieldData)) \
		presetSetField(this, &presetFieldData); \
}

#define UPDATE_PRESET_FIELD3(A,B,C) \
{ \
	PresetFieldData_t bak = presetFieldData; \
	UPDATE_PRESET_IORANGES(); \
	presetFieldData.A = A; \
	presetFieldData.B = B; \
	presetFieldData.C = C; \
	if(!areEqual(bak, presetFieldData)) \
		presetSetField(this, &presetFieldData); \
}

#define UPDATE_PRESET_NOIORANGES_FIELD3(A,B,C) { \
	PresetFieldData_t bak = presetFieldData; \
	presetFieldData.A = A; \
	presetFieldData.B = B; \
	presetFieldData.C = C; \
	if(!areEqual(bak, presetFieldData)) \
		presetSetField(this, &presetFieldData); \
}

#define UPDATE_PRESET_FIELD3PlusArrays(A,B,C,A0,A1) \
{ \
	bool same = true; \
	UPDATE_PRESET_IORANGES_WITH_SAME(); \
	for(size_t n = 0; n < A0.size(); ++n) \
	{ \
		auto a0 = A0[n].get(); \
		if(memcmp(&a0, &presetFieldData.A0[n], sizeof(a0))) \
		{ \
			same = false; \
			break; \
		} \
	} \
	for(size_t n = 0; n < A1.size(); ++n) \
	{ \
		auto a1 = A1[n].get(); \
		if(memcmp(&a1, &presetFieldData.A1[n], sizeof(a1))) \
		{ \
			same = false; \
			break; \
		} \
	} \
	if(presetFieldData.A != A || presetFieldData.B != B || presetFieldData.C != C || !same) { \
		presetFieldData.A = A; \
		presetFieldData.B = B; \
		presetFieldData.C = C; \
		for(size_t n = 0; n < A0.size(); ++n) \
			presetFieldData.A0[n] = A0[n]; \
		for(size_t n = 0; n < A1.size(); ++n) \
			presetFieldData.A1[n] = A1[n]; \
		presetSetField(this, &presetFieldData); \
	} \
}

#define UPDATE_PRESET_NOIORANGES_FIELD4(A,B,C,D) { \
	PresetFieldData_t bak = presetFieldData; \
	presetFieldData.A = A; \
	presetFieldData.B = B; \
	presetFieldData.C = C; \
	presetFieldData.D = D; \
	if(!areEqual(bak, presetFieldData)) \
		presetSetField(this, &presetFieldData); \
}

#define UPDATE_PRESET_FIELD12(A,B,C,D,E,F,G,H,I,J,K,L) \
{ \
	PresetFieldData_t bak = presetFieldData; \
	UPDATE_PRESET_IORANGES(); \
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
std::array<TouchTracker::TouchWithId,kNumSplits> touchTrackerSplit(CentroidDetection& slider, bool shouldProcess, bool split)
{
	std::array<TouchTracker::TouchWithId,kNumSplits> values {};
	if(shouldProcess)
		gTouchTracker.process(globalSlider);
	size_t numTouches = gTouchTracker.getNumTouches();
	// per each split
	static_assert(kNumSplits == 2); // or the loop below won't work
	const float midMin = 0.45;
	const float midMax = 0.55;
	for(ssize_t s = 0; s < 1 + split; ++s)
	{
		const float min = split ? (kNumSplits - 1 - s) * midMax : 0;
		const float max = split ? min + midMin : 1;
		TouchTracker::TouchWithId twi = TouchTracker::kInvalidTouch;
		for(ssize_t i = numTouches - 1; i >= 0; --i)
		{
			// get the most recent touch which started on this split
			const TouchTracker::TouchWithId& t = gTouchTracker.getTouchOrdered(i);
			if(t.startLocation >= min && t.startLocation <= max)
			{
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
					twi = t;
					break;
				}
			}
		}
		values[s] = twi;
		if(TouchTracker::kIdInvalid != twi.id) {
			// TODO: this used to be compoundTouch)
			values[s].touch.location = mapAndConstrain(twi.touch.location, min, max, 0, 1);
		}
		if(split)
		{
			// start location relative to split, like location
			values[s].startLocation = mapAndConstrain(values[s].startLocation, min, max, 0, 1);
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
	void renderOut(std::array<float,kNumSplits>& out, const std::array<centroid_t,kNumSplits>& values, const std::array<centroid_t,kNumSplits>& displayValues, std::array<bool,kNumSplits> preserveSplitLocationSize = {})
	{
		for(ssize_t n = 0; n < isSplit() + 1; ++n)
		{
			switch(splitMode)
			{
			case kModeNoSplit:
				ledSliders.sliders[n].setLedsCentroids(displayValues.data(), 1);
				out[0] = touchOrNot(values[0]).location;
				out[1] = touchOrNot(values[0]).size;
				break;
			case kModeSplitLocation:
			{
				bool hasTouch = (values[n].size > 0);
				centroid_t centroid;
				centroid.location = displayValues[n].location;
				centroid.size = preserveSplitLocationSize[n] ? values[n].size : hasTouch * kFixedCentroidSize;
				ledSliders.sliders[n].setLedsCentroids(&centroid, 1);
				out[n] = touchOrNot(values[n]).location;
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
				out[n] = touchOrNot(values[n]).size;
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
	void animate(Parameter& p, LedSlider& l, rgb_t color, uint32_t ms) override
	{
		if(p.same(splitMode))
		{
			constexpr uint32_t kDuration = 1200;
			if(ms < kDuration)
			{
				float loc = simpleTriangle(ms, kDuration);
				gAnimateFs.writeInit(p, l);
				switch(splitMode.get())
				{
				case kModeNoSplit:
					gAnimateFs.directWriteCentroid(p, l, { .location = loc, .size = loc }, color);
					break;
				case kModeSplitLocation:
					gAnimateFs.directWriteCentroid(p, l, { .location = map(loc, 0, 1, 0, 0.5), .size = kFixedCentroidSize }, color);
					gAnimateFs.directWriteCentroid(p, l, { .location = map(loc, 0, 1, 0.5, 1), .size = kFixedCentroidSize }, color);
					break;
				case kModeSplitSize:
					gAnimateFs.directWriteCentroid(p, l, { .location = 0.1, .size = loc }, color, LedSlider::kDefaultNumWeights * 2);
					gAnimateFs.directWriteCentroid(p, l, { .location = 0.9, .size = loc }, color, LedSlider::kDefaultNumWeights * 2);
					break;
				}
			}
		}
	}
public:
	enum SplitMode {
		kModeNoSplit,
		kModeSplitLocation,
		kModeSplitSize,
	};
	ParameterEnumT<3> splitMode{this, kModeNoSplit};
	SplitPerformanceMode(rgb_t color) : PerformanceMode(color) {}
	SplitPerformanceMode() {}
};

static centroid_t processSize(centroid_t c, size_t split)
{
	static std::array<float,2> pastSizes {};
	static std::array<float,2> pastPastSizes {};
	float decay = 0.998;
	float envIn = c.size;
	// special peak envelope detector
	// whereas if the input is zero, we just accept it
	// this way we have fast attacks and fast releases
	// and what's in between is smoothed
	float env = pastSizes[split];
	if(0 == envIn || 0 == pastSizes[split] || 0 == pastPastSizes[split])
	{
		// during, or immediately after a zero (i.e.: end or beginning of touch)
		// pass through current value. This gives immediate attacks and releases.
		env = envIn;
	} else if(envIn > env)
	{
		// minimal smoothing
		env = env * 0.9 + envIn * 0.1;
	} else {
		float diff = constrain(env - envIn, 0, 1);
		if(diff < 0.1) {
			// use decay as is
		} else {
			// reduce decay for faster transition
			decay = map(diff - 0.1, 0, 0.9, decay, 0.5);
			decay = constrain(decay, 0, 0.999); // in case we get anything wrong ..
		}
		env = envIn * (1.f - decay) + env * decay;
	}
	pastPastSizes[split] = pastSizes[split];
	pastSizes[split] = env;
	return centroid_t{
		.location = c.location,
		.size = env,
	};
}

#ifdef ENABLE_DIRECT_CONTROL_MODE
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
				ledSlidersSetupTwoSliders(guardPads, color, LedSlider::MANUAL_CENTROIDS);
		} else {
			if(ms <= 0)
			{
				ledSlidersSetupOneSlider(
					color,
					LedSlider::MANUAL_CENTROIDS
				);
			}
		}
		if(ms < 0)
			return true;
		// opening animation
		// single point starts in middle, zips off in two directions to the top and bottom
		constexpr float kAnimationDuration = 1200;
		float loc = ms / (kAnimationDuration * 2) + 0.5f;
		float size = kFixedCentroidSize;
		for(auto l : { &ledSliders, &ledSlidersAlt})
		{
			l->sliders[0].directBegin();
			l->sliders[0].directWriteCentroid({ .location = loc, .size = size }, color);
			l->sliders[0].directWriteCentroid({ .location = 1.0f - loc, .size = size }, color);
		}
		return ms > kAnimationDuration;
	}
	void render(BelaContext*, FrameData* frameData) override
	{
		setOutIsSize();
		bool analogInHigh = tri.analogRead() > 0.5;
		bool analogRisingEdge = (analogInHigh && !pastAnalogInHigh);
		pastAnalogInHigh = analogInHigh;
		std::array<TouchTracker::TouchWithId,kNumSplits> twis = touchTrackerSplit(globalSlider, ledSliders.isTouchEnabled() && frameData->isNew, isSplit());
		std::array<centroid_t,kNumSplits> values {};
		for(size_t n = 0; n < currentSplits(); ++n)
			values[n] = processSize(twis[n].touch, n);
		bool shouldLatch = false;
		bool shouldUnlatch = false;

		bool hasTouch = false;
		for(size_t n = 0; n < currentSplits(); ++n)
			hasTouch |= values[n].size > 0;

		if(performanceBtn.onset)
		{
			// if you have at least one touch, we latch
			shouldLatch |= hasTouch;
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
		if(analogRisingEdge && !performanceBtn.pressed)
		{
			// on analog edge, if there is a touch, latch. Otherwise, unlatch.
			shouldLatch |= hasTouch;
			shouldUnlatch = !shouldLatch;
		}
		if(shouldLatch)
			tri.buttonLedSet(TRI::kSolid, TRI::kR, 1, 100);
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
	void animate(Parameter& p, LedSlider& l, rgb_t color, uint32_t ms) override
	{
		SplitPerformanceMode::animate(p, l, color, ms);
		if(p.same(autoLatch))
		{
			constexpr uint32_t kInitialDuration = 300;
			constexpr uint32_t kHoldDuration = 600;
			constexpr uint32_t kCombinedDuration = kInitialDuration + kHoldDuration;
			if(ms < 3 * kCombinedDuration)
			{
				size_t n = ms / kCombinedDuration;
				ms %= kCombinedDuration;
				float offset = 0.4f * n;
				float loc = offset + 0.2f * simpleRamp(constrain(ms, 0, kInitialDuration - 1), kInitialDuration);
				float size = kFixedCentroidSize;
				// all viz start with one centroid moving from mid-bottom to mid-top.
				// what follows during "Hold" is the differentiator:
				switch(autoLatch.get())
				{
				case kAutoLatchOff:
					// ... followed by a blank screen
					size *= ms < kInitialDuration;
					break;
				case kAutoLatchBoth:
					// ... followed by a hold
					size *= 1;
					break;
				case kAutoLatchLocationOnly:
					// ... followed by a hold that progressively fades out
					if(ms >= kInitialDuration)
						size *= 1.f - (ms - kInitialDuration) / float(kHoldDuration);
				}
				gAnimateFs.writeInit(p, l);
				gAnimateFs.directWriteCentroid(p, l, { .location = loc, .size = size }, color);
			}
		}
	}
	void updated(Parameter& p)
	{
		PerformanceMode::updated(p);
		if(p.same(splitMode)) {
			M(printf("DirectControlMode: updated splitMode: %d\n\r", splitMode.get()));
			setup(-1);
		}
		else if (p.same(autoLatch)) {
			M(printf("DirectControlMode: updated autoLatch: %d\n\r", autoLatch.get()));
		}
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD2(splitMode, autoLatch);
	}
	DirectControlMode() :
		SplitPerformanceMode(kRgbRed),
		presetFieldData{
			.ioRanges = ioRangesParameters,
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
		IoRanges ioRanges;
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
	rgb_t color = kRgbRed;
	bool pastAnalogInHigh = false;
	LatchProcessor latchProcessor;
	std::array<LatchProcessor::Reason,2> isLatched = {LatchProcessor::kLatchNone, LatchProcessor::kLatchNone};
	uint32_t lastLatchCount = ButtonView::kPressIdInvalid;
} gDirectControlMode;
#endif // ENABLE_DIRECT_CONTROL_MODE

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
enum TreatNoOutput {
	kTreatAssumeNot,
	kTreatPassThrough,
	kTreatAsZero,
};
static float interpolatedRead(const float* table, size_t size, float idx, TreatNoOutput treat = kTreatAssumeNot)
{
	float n = size * idx;
	size_t prev = size_t(n);
	size_t next = size_t(n + 1);
	if(prev >= size) // idx was out of range
		prev = size - 1;
	if(next >= size)
		next = 0; // could be we are at the end of table
	float frac = n - prev;
	float pr = table[prev];
	float ne = table[next];
	if(kNoOutput == pr || kNoOutput == ne)
	{
		switch(treat)
		{
		case kTreatAssumeNot:
			// nothing to do
			break;
		case kTreatPassThrough:
			return frac < 0.5 ? pr : ne;
			break;
		case kTreatAsZero:
			if(kNoOutput == pr)
				pr = 0;
			else if(kNoOutput == ne)
				ne = 0;
			break;
		}
	}
	float value = linearInterpolation(frac, pr, ne);
	return value;
}

template <typename T>
static float interpolatedRead(const T& table, float idx)
{
	return interpolatedRead(table.data(), table.size(), idx);
}

static inline float getBlinkPeriod(BelaContext* context, bool lessIntrusive)
{
	float ceiling = lessIntrusive ? 60 : 80;
	float periodMs = gClockPeriod * 1000.f / context->analogSampleRate;
	float ms = std::min(ceiling, periodMs / 4.f);
	if(lessIntrusive && periodMs < 75)
		ms = 0; // suppress when too short/fast
	return ms;
}

#ifdef ENABLE_RECORDER_MODE
class RecorderMode : public SplitPerformanceMode {
public:
	enum InputMode {
		kInputModeLfo,
		kInputModeEnvelope,
		kInputModeClock,
		kInputModeCv,
		kInputModePhasor,
		kInputModeNum,
	};
	bool setup(double ms) override
	{
		twis = {};
		reinitInputModeClock();
		gOutMode.fill(kOutModeManualBlock);
		hadTouch.fill(false);
		idxFrac = 0;
		ignoredTouch.fill(TouchTracker::kIdInvalid);
		buttonBlinksIgnored = 0;
		if(isSplit())
		{
			unsigned int guardPads = 1;
			if(ms <= 0)
				ledSlidersSetupTwoSliders(guardPads, color, LedSlider::MANUAL_CENTROIDS);
		}
		else
		{
			if(ms <= 0)
				ledSlidersSetupOneSlider(color, LedSlider::MANUAL_CENTROIDS);
		}
		if(ms < 0)
			return true;
		// opening animation
		// single point whips from the bottom, to the top, and back to the bottom
		constexpr float kAnimationDuration = 1600;
		float phase = 2.f * ms / kAnimationDuration;
		float loc = phase < 1 ? phase : 2.f - phase;
		float size = kFixedCentroidSize;
		for(auto l : { &ledSliders, &ledSlidersAlt})
		{
			l->sliders[0].directBegin();
			l->sliders[0].directWriteCentroid({ .location = loc, .size = size }, color);
		}
		return ms > kAnimationDuration;
	}
	bool buttonTriggers(){
		return kInputModeLfo == inputMode || kInputModeEnvelope == inputMode;
	}
	void render(BelaContext* context, FrameData* frameData) override
	{
		if(kInputModeClock == inputMode)
			gModeWantsInteractionPreMenu = true;
		if(!buttonTriggers()) // we need to allow for fast repeated presses when the button triggers
			performanceBtn = ButtonViewSimplify(performanceBtn);
		if(!areRecording())
		{
			bool newinputModeClockIsButton = !clockInIsActive(context);
			if(newinputModeClockIsButton != inputModeClockIsButton)
			{
				reinitInputModeClock();
				if(newinputModeClockIsButton)
					tri.buttonLedSet(TRI::kSolid, TRI::kG, 1, 60);
				inputModeClockIsButton = newinputModeClockIsButton;
			}
		}
		// set global states
		setOutIsSize();
		switch(inputMode.get())
		{
		case kInputModeCv:
		case kInputModeClock:
			gInUsesRange = false; // we need actual voltage here
			break;
		default:
			gInUsesRange = true;
		}
		uint64_t currentSamples = context->audioFramesElapsed;

		enum StopMode {
			kStopNone = 0,
			kStopNowOnEdge, // stopping now and we are on an edge
			kStopNowLate, // stopping now and the edge has passed
		};
		std::array<StopMode,kNumSplits> qrecStopNow {kStopNone, kStopNone};
		std::array<RecordingMode,kNumSplits> qrecStartNow {kRecNone , kRecNone};

		// handle button
		if(!(kInputModeEnvelope == inputMode) && // when in envelope mode, there is never a good reason to erase recordings
				(performanceBtn.pressDuration == msToNumBlocks(context, 3000)
				|| (!buttonTriggers() && performanceBtn.tripleClick))) // if button triggers, it may be pressed repeatedly for performance reasons
		{
			emptyRecordings();
			// clear possible side effects of previous press:
			reinitInputModeClock();
			lastIgnoredPressId = performanceBtn.pressId;
			tri.buttonLedSet(TRI::kSolid, TRI::kG, 1, 300);
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
		uint64_t lateSamples = currentSamples - lastAnalogRisingEdgeSamples;
		if(performanceBtn.onset)
		{
			if(kInputModeEnvelope == inputMode || kInputModeLfo == inputMode)
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
						tri.buttonLedSet(TRI::kSolid, TRI::kG, 1, 150);
					} else {
						// if not recording, on button press we
						// start the attack section of the envelope
						// NOTE: all other modes use the button offset
						// instead to minimise interaction with menu entering.
						// TODO: decide if this is really the best
						triggerNow = true;
						// we DO NOT ignore this press as we are interested
						// in getting its offset, too for triggering the release phase

						// blink button
						tri.buttonLedSet(TRI::kSolid, TRI::kR, 1, 150);
					}
				}
			} else
			if(kInputModeClock == inputMode)
			{
				if(inputModeClockIsButton)
				{
					// if clock in is disabled, button onset triggers immediate start or stop of recording
					lastIgnoredPressId = performanceBtn.pressId;
					if(areRecording()){
						for(size_t n = 0; n < qrecs.size(); ++n)
						{
							if(isRecording(n))
								qrecStopNow[n] = kStopNowOnEdge;
						}
					} else {
						qrecStartNow.fill(kRecOnButton);
					}
				} else {
					// how late can a button press be to be considered as
					// belonging to the previous edge
					// this is clock-dependent with an upper limit
					uint64_t maxDelaySamples = std::min(gClockPeriod * 0.25f, 0.2f * context->analogSampleRate);
					bool closeEnough = (lateSamples < maxDelaySamples);
					for(size_t n = 0; n < kNumSplits; ++n)
					{
						auto& qrec = qrecs[n];
						if(closeEnough && 0 == n)
							printf("C %.3fs\n\r", lateSamples / context->analogSampleRate);
						switch(qrec.recording)
						{
						case kRecOnButton:
							// normally lastIgnoredPressId will make sure we don't get here
							// however, we may in case the clock started again since the recording started.
							// In that case, ignore
							break;
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
							} else {
								// wait for next edge
								qrec.armedFor = kArmedForStop;
							}
							break;
						} // switch qrec.recording
					} // for kNumSplts
				}
			}
		}
		if(gAlt && kInputModeClock == inputMode && areRecording())
		{
			// We got into menu while recording.
			// This means that the keypress that triggered the recording onset has been used to
			// enter the menu. Clear its side effects
			reinitInputModeClock();
			for(size_t n = 0; n < twis.size(); ++n)
				ignoredTouch[n] = getId(twis, n);
		}
		// EG + Gate mode
		if(kInputModeEnvelope == inputMode && (envelopeReleaseStarts[0] > 0 || envelopeReleaseStarts[1] > 0) && performanceBtn.pressed)
		{
			// hold LED as long as button is down
			tri.buttonLedSet(TRI::kSolid, TRI::kR, 1);
		}
		std::array<bool,kNumSplits> releaseStarts {false, false};
		if(performanceBtn.offset && performanceBtn.pressId != lastIgnoredPressId)
		{
			switch(inputMode.get())
			{
			case kInputModeEnvelope:
					tri.buttonLedSet(TRI::kOff, TRI::kR);
					for(size_t n = 0; n < kNumSplits; ++n)
						if(envelopeReleaseStarts[n] >= 0)
							releaseStarts[n] = true;
				break;
			} // switch inputMode
		}
		bool redButtonIsOn = false;
		if(kInputModeClock == inputMode)
			redButtonIsOn = qrecs[0].armedFor || qrecs[1].armedFor || areRecording();
		else
			redButtonIsOn = gGestureRecorder.isRecording(0) || gGestureRecorder.isRecording(1);
		tri.buttonLedSet(TRI::kSolid, TRI::kR, redButtonIsOn * 0.6f);
		// TODO: obey trigger level
		bool analogInHigh = tri.analogRead() > 0.5;

		bool gate = false;
		if(inputMode == kInputModeEnvelope)
		{
			gate = analogInHigh || performanceBtn.pressed;
		}
		bool analogRisingEdge = (analogInHigh && !pastAnalogInHigh);
		bool analogFallingEdge = (!analogInHigh && pastAnalogInHigh);
		pastAnalogInHigh = analogInHigh;
		std::array<bool,kNumSplits> qrecResetPhase { false, false };
		size_t recordOffset = 0;
		if(kInputModeClock == inputMode)
			recordOffset += GestureRecorder::kNumRecs / 2;
		bool inputIsTrigger = false;
		if(analogRisingEdge)
		{
			inputIsTrigger = true;
			lastAnalogRisingEdgeSamples = currentSamples;
			switch(inputMode.get())
			{
				default:
					inputIsTrigger = false;
					break;
				case kInputModeLfo:
				case kInputModeEnvelope:
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
		if(inputIsTrigger || gate || triggerNow)
			tri.buttonLedSet(TRI::kSolid, TRI::kY, 1, gate ? 10 : getBlinkPeriod(context, redButtonIsOn));
		if(analogFallingEdge)
		{
			if(kInputModeEnvelope == inputMode) {
				for(size_t n = 0; n < kNumSplits; ++n)
				{
					if(envelopeReleaseStarts[n] >= 0)
						releaseStarts[n] = true;
				}
			}
		}

		twis = touchTrackerSplit(globalSlider, ledSliders.isTouchEnabled() && frameData->isNew, isSplit());
		for(size_t n = 0; n < currentSplits(); ++n)
			twis[n].touch = processSize(twis[n].touch, n);
		std::array<bool,kNumSplits> hasTouch;
		for(size_t n = 0; n < currentSplits(); ++n)
		{
			auto id = getId(twis, n);
			bool touchInvalid = (TouchTracker::kIdInvalid == id) || (id == ignoredTouch[n]);
			hasTouch[n] = !touchInvalid;
		}

		// control start/stop recording
		if(kInputModeClock == inputMode)
		{
			// start/stop recording based on qrec and input edges
			for(size_t n = 0; n < kNumSplits; ++n)
			{
				auto& qrec = qrecs[n];
				if(kRecNone != qrec.recording && gGestureRecorder.rs[n + recordOffset].r.full)
				{
					// if the recorder's buffer is full,
					// stop recording aligned to past edge if appropriate, or right now
					if(kRecOnButton == qrec.recording)
						qrecStopNow[n] = kStopNowOnEdge;
					else {
						qrecStopNow[n] = qrec.periodsInRecording ? kStopNowLate : kStopNowOnEdge;
					}
					qrec.armedFor = kArmedForNone;
				}

				if(qrecStartNow[n])
				{
					gGestureRecorder.startRecording(n + recordOffset);
					envelopeReleaseStarts[n] = -1;
					qrec.recording = qrecStartNow[n];
					qrec.periodsInRecording = 0;
					qrec.framesAtStart = context->audioFramesElapsed;
				}
				float phaseOffset = 0;
				if(kStopNone != qrecStopNow[n])
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
					gGestureRecorder.stopRecording(n + recordOffset, false);
					if(valid)
					{
						qrec.recordedAs = qrec.recording;
						qrec.framesInRecording = context->audioFramesElapsed - qrec.framesAtStart;
						gGestureRecorder.rs.swap(n, n + recordOffset);
						if(kRecOnButton == qrec.recordedAs)
						{
							// this will also be played back at fixed speed
							periodsInTables[n] = 1;
						} else {
							periodsInTables[n] = std::max(1u, qrec.periodsInRecording);
						}
						qrecResetPhase[n] = true;
						qrec.periodsInPlayback = 0;
						printf("%u periods\n\r", periodsInTables[n]);
						if(kStopNowLate == qrecStopNow[n])
						{
							// adjust the phase to make up for the lost time
							float normFreq = getOscillatorFreq(context, n) / context->analogSampleRate;
							phaseOffset = lateSamples * normFreq * 2.f * float(M_PI);
						}
					}
					qrec.recording = kRecNone;
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
						bool optimizeForLoop = isSize(n) && (kInputModeLfo == inputMode);
						gGestureRecorder.stopRecording(n, optimizeForLoop);
						periodsInTables[n] = 1;
					}
				}
			}
			hadTouch = hasTouch;
		}

		GestureRecorder::Gesture_t gesture; // used for visualization
		std::array<float,kNumSplits> recIns;

		centroid_t t0 = touchOrNot(twis[0].touch);
		centroid_t t1 = touchOrNot(twis[1].touch);
		switch(splitMode)
		{
		case kModeNoSplit:
			recIns[0] = t0.location;
			recIns[1] = t0.size;
			break;
		case kModeSplitLocation:
			recIns[0] = t0.location;
			recIns[1] = t1.location;
			break;
		case kModeSplitSize:
			recIns[0] = t0.size;
			recIns[1] = t1.size;
			break;
		}
		// gesture may be overwritten below before it is visualised
		for(size_t n = 0; n < recIns.size(); ++n)
		{
			size_t idx = n;
			if(gGestureRecorder.isRecording(n + recordOffset))
				idx += recordOffset;
			if(kInputModeLfo == inputMode || kInputModeEnvelope == inputMode || gGestureRecorder.isRecording(idx))
			{
				bool autoRetrigger = (kInputModeLfo == inputMode);
				ssize_t freezeAt = -1;
				if(kInputModeEnvelope == inputMode || kInputModeLfo == inputMode)
				{
					bool hangAtEndOfAttackOnShortGate = (kInputModeLfo == inputMode);
					// how to deal with a gate that is shorter than the attack
					if(hangAtEndOfAttackOnShortGate)
					{
						// hang at the end of attack
						freezeAt = envelopeReleaseStarts[n];
					} else {
						// play through attack and release
						freezeAt = envelopeReleaseStarts[n];
						if(!gate) // if gate is off, allow release
							releaseStarts[n] = true;
					}
					// never jump to release: only get there if already frozen
					if(releaseStarts[n] && gGestureRecorder.rs[idx].frozen)
						gGestureRecorder.resumePlaybackFrom(n, envelopeReleaseStarts[n]);
				}
				gesture[n] = gGestureRecorder.process(idx, recIns[n], frameData->id, autoRetrigger, triggerNow, freezeAt);
				TouchTracker::Id id = getId(twis, n);
				if(gGestureRecorder.isRecording(idx) && gGestureRecorder.rs[idx].r.full)
				{
					if((isRecording(n) && kInputModeClock == inputMode) || (ignoredTouch[n] != id && TouchTracker::kIdInvalid != id))
					{
						ignoredTouch[n] = id;
						tri.buttonLedSet(TRI::kSolid, TRI::kG, 1, 15);
					}
				}
			}
		}

		std::array<bool,kNumSplits> directControl = { false, false };
		switch(inputMode)
		{
		case kInputModeLfo:
		case kInputModeEnvelope:
			gOutMode.fill(kOutModeManualBlock);
			break;
		case kInputModeClock:
			for(size_t n = 0; n < currentSplits(); ++n)
			{
				// if actually doing something while recording, pass through current touch
				if(isRecording(n) && gGestureRecorder.rs[n + recordOffset].activity)
					directControl[n] = true;
				// if a finger is on the sensor and we are not recording, pass through current touch
				if(hasTouch[n])
					directControl[n] = true;
				centroid_t touch = touchOrNot(twis[n].touch);
				if(isSplit()) {
					if(directControl[n])
					{
						gOutMode[n] = kOutModeManualBlock;
						gesture[n] = GestureRecorder::HalfGesture_t {
							.sample = kModeSplitLocation == splitMode ? touch.location : touch.size,
							.valid = true,
						};
					} else // otherwise, keep playing back from table
						gOutMode[n] = kOutModeManualSampleSmoothed;
				} else {
					// non split
					directControl[1] = directControl[n];
					if(directControl[n])
					{
						gOutMode.fill(kOutModeManualBlock);
						gesture[0] = { touch.location, true };
						gesture[1] = { touch.size, true };
					} else
						gOutMode.fill(kOutModeManualSampleSmoothed);
				}
			}
			break;
		default:
			gOutMode.fill(kOutModeManualSample);
		}
		for(unsigned int n = 0; n < kNumSplits; ++n)
		{
			if(kOutModeManualBlock != gOutMode[n])
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
		std::array<bool,kNumSplits> preserveSplitLocationSize {};
		if(kInputModeCv == inputMode)
		{
			// in order to visualise something meaningful, we show a fixed-period
			// visualisation of the content of the table
			static constexpr float kDisplayPeriod = 2; // seconds
			for(size_t c = 0; c < currentSplits(); ++c)
			{
				if(TouchTracker::kIdInvalid == getId(twis, c))
				{
					preserveSplitLocationSize[c] = true;
					const float* table = gGestureRecorder.rs[c].r.getData().data();
					size_t tableSize = gGestureRecorder.rs[c].r.size();
					vizValues[c] = {};
					if(tableSize)
					{
						// visualise with fix period
						vizValues[c].location = interpolatedRead(table, tableSize, idxFrac, kTreatPassThrough);
						if(isSplit())
						{
							vizValues[c].size = isSizeOnly ? vizValues[c].location : kFixedCentroidSize;
						} else {
							const float* table = gGestureRecorder.rs[c].r.getData().data();
							size_t tableSize = gGestureRecorder.rs[c].r.size();
							if(tableSize)
								vizValues[0].size = interpolatedRead(table, tableSize, idxFrac, kTreatPassThrough);
						}
						// animate the centroid so you can tell it's not "real" but fixed period
						vizValues[c].size *= 0.1f + 0.9f * simpleTriangle(context->audioFramesElapsed, unsigned(context->analogSampleRate * 0.1f));
					}
				} else {
					// visualise current touch
					vizValues[c] = twis[c].touch;
					idxFrac = 0; // prepare phase for next time we get to draw it
				}
			}
			idxFrac += (context->analogFrames) / (context->analogSampleRate) / kDisplayPeriod;
			while(idxFrac >= 1)
				idxFrac -= 1;
		}
		// this may set gManualAnOut even if they are ignored
		renderOut(gManualAnOut, vizValues, vizValues, preserveSplitLocationSize);
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
				float volts = inToV(analogRead(context, 0, 0));
				float baseFreq = 65.40639f; // a C
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
					float idx = mapAndConstrain(oscs[c].process(normFreq), -1, 1, 0, 1);
					float value = interpolatedRead(table, tableSize, idx, kTreatPassThrough);
					analogWriteOnce(context, n, c, value);
					if(0 == n)
					{
						// for visualisation purposes, avoid
						// interpolation when reading the table
						vizOut = table[size_t(idx * tableSize)];
					}
			}
		} else if (kInputModePhasor == inputMode) {
			for(size_t n = 0; n < context->analogFrames; ++n)
			{
				float idx = analogRead(context, n, 0);
				{
					float out = interpolatedRead(table, tableSize, idx, kTreatPassThrough);
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
		PerformanceMode::updated(p);
		if(p.same(splitMode)) {
			M(printf("RecorderMode: Updated splitMode: %d\n\r", splitMode.get()));
			setup(-1);
		} else if (p.same(inputMode)) {
			M(printf("RecorderMode: Updated inputMode: %d\n\r", inputMode.get()));
			if(kInputModeClock == inputMode)
			{
				reinitInputModeClock();
				gGestureRecorder.rs.setEnabled(GestureRecorder::SwappableRecorders::kWhichRecordersDoubleWithBackup);
			} else {
				gGestureRecorder.rs.setEnabled(GestureRecorder::SwappableRecorders::kWhichRecordersDouble);
			}
		}
	}
	void animate(Parameter& p, LedSlider& l, rgb_t color, uint32_t ms) override
	{
		SplitPerformanceMode::animate(p, l, color, ms);
		if(p.same(inputMode))
		{
			static constexpr std::array<float,7> gesture = {
				0.0, 0.3, 0.0, 0.2, 0.5, 0.8, 0.99,
			};
			constexpr uint32_t kSingleGestDuration = 800;
			float size = kFixedCentroidSize;

			float gestIdx = 0;
			bool useGest = false;
			uint32_t duration = 0;
			float loc = 0;
			switch(inputMode.get())
			{
			// each should either set {loc and/or size} OR {gestIdx and useGest}
			case kInputModeLfo:
			{
				// show a gesture, repeated three times
				duration = kSingleGestDuration * 3;
				gestIdx = (ms % kSingleGestDuration) / float(kSingleGestDuration);
				useGest = true;
				break;
			}
			case kInputModeEnvelope:
			{
				// show the same gesture as above only once, followed by a blank screen
				duration = kSingleGestDuration * 2;
				gestIdx = std::min(ms / float(kSingleGestDuration), 1.f);
				useGest = true;
				break;
			}
			case kInputModeClock:
			{
				// 4 repetitions of a "clock"
				constexpr uint32_t kClockPulseDuration = kSingleGestDuration * 0.75f;
				duration = kClockPulseDuration * 4;
				float idx = (ms % kClockPulseDuration ) / float(kClockPulseDuration);
				if(idx < 0.2)
					loc = idx * 5;
				else
					loc = 0;
				break;
			}
			case kInputModeCv:
			{
				 // a centroid moves in the pattern of a sine
				duration = kSingleGestDuration * 3.f;
				uint32_t period = duration / 2;
				loc = map(sinf(2 * M_PI * (ms % period) / float(period)), -1, 1, 0, 1);
				break;
			}
			case kInputModePhasor:
			{
				//  a centroid moves from bottom to top
				duration = kSingleGestDuration * 1.5f;
				uint32_t period = duration / 2;
				loc = simpleRamp(ms, period);
				break;
			}
			} // switch
			if(ms < duration)
			{
				// actually do animation
				if(useGest)
				{
					if(gestIdx >= 1)
						size = 0;
					else
						loc = interpolatedRead(gesture, gestIdx);
				}
				gAnimateFs.writeInit(p, l);
				gAnimateFs.directWriteCentroid(p, l, { .location = loc, .size = size }, color);
			}
		}
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD2(splitMode, inputMode);
	}
	RecorderMode() :
		SplitPerformanceMode(kRgbYellow),
		presetFieldData {
			.ioRanges = ioRangesParameters,
			.splitMode = splitMode,
			.inputMode = inputMode,
		}
	{
		PresetDesc_t presetDesc = {
			.field = this,
			.size = sizeof(PresetFieldData_t),
			.defaulter = genericDefaulter2(RecorderMode, splitMode,  inputMode),
			.loadCallback = genericLoadCallback2(RecorderMode, splitMode, inputMode),
		};
		presetDescSet(1, &presetDesc);
	}
	//splitMode from the base class
	ParameterEnumT<kInputModeNum> inputMode{this, kInputModeLfo};
	PACKED_STRUCT(PresetFieldData_t {
		IoRanges ioRanges;
		uint8_t splitMode;
		uint8_t inputMode;
	}) presetFieldData;
private:
	void reinitInputModeClock()
	{
		for(auto& qrec : qrecs)
		{
			qrec.armedFor = kArmedForNone;
			qrec.recording = kRecNone;
		}
	}
	float getOscillatorFreq(BelaContext* context, size_t c)
	{
		float period;
		if(kRecOnButton == qrecs[c].recordedAs)
			period = qrecs[c].framesInRecording;
		else
			period = gClockPeriod;
		if(period < 1)
			period = 1;
		return context->analogSampleRate / period / periodsInTables[c];
	}
	bool isRecording(size_t c)
	{
		RecordingMode r = qrecs[c].recording;
		return kRecActual == r || kRecOnButton == r;
	}
	bool areRecording()
	{
		bool areRec = false;
		for(size_t n = 0; n < qrecs.size(); ++n)
		{
			if(isRecording(n))
				areRec = true;
		}
		return areRec;
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
	TouchTracker::Id getId(std::array<TouchTracker::TouchWithId,kNumSplits>& twis, size_t c)
	{
		assert(c < twis.size());
		return twis[isSplit() ? c : 0].id;
	}
	rgb_t color = kRgbYellow.scaledBy(0.5);
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
		kRecOnButton,
	};
	struct QuantisedRecorder {
		size_t periodsInRecording;
		size_t periodsInPlayback;
		size_t recCounterAtLastEdge;
		size_t recSizeAtLastEdge;
		ArmedFor armedFor;
		RecordingMode recording;
		RecordingMode recordedAs;
		uint64_t framesAtStart;
		uint64_t framesInRecording;
		bool isSynced;
	};
	std::array<QuantisedRecorder,kNumSplits> qrecs {};
	static constexpr size_t kNumSplits = ::kNumSplits;
	std::array<Oscillator,kNumSplits> oscs {{{1, Oscillator::sawtooth}, {1, Oscillator::sawtooth}}};
	std::array<size_t,kNumSplits> periodsInTables {1, 1};
	std::array<bool,kNumSplits>  hadTouch;
	std::array<TouchTracker::Id,kNumSplits> ignoredTouch;
	std::array<ssize_t,kNumSplits> envelopeReleaseStarts { -1, -1 };
	size_t lastIgnoredPressId = ButtonView::kPressIdInvalid;
	uint64_t lastAnalogRisingEdgeSamples = 0;
	float idxFrac = 0;
	size_t buttonBlinksIgnored;
	std::array<TouchTracker::TouchWithId,kNumSplits> twis;
	bool pastAnalogInHigh = false;
	bool inputModeClockIsButton = true;
} gRecorderMode;
#endif // ENABLE_RECORDER_MODE

static void menu_up();

#define FILL_ARRAY(name, ...) [this](){decltype(name) a; a.fill( __VA_ARGS__); return a;}()
static rgb_t crossfade(const rgb_t& a, const rgb_t& b, float idx);

#ifdef ENABLE_SCALE_METER_MODE
#define MENU_ENTER_RANGE_DISPLAY
static void menu_enterRangeDisplay(const rgb_t& signalColor, const std::array<rgb_t,2>& endpointsColors, bool autoExit, ParameterContinuous& bottom, ParameterContinuous& top, const float& display);

class ScaleMeterMode : public PerformanceMode {
public:
	static constexpr size_t kCentroidSize = 2;
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
		// animation
		// VU meter colour appears from bottom to top.
		// As soon as it is full it starts to disappear from the bottom upwards
		np.clear();
		constexpr float kAnimationDuration = 1200;
		float phase = ms / kAnimationDuration;
		size_t start = constrain(phase * 2.f - 1.f, 0, 1) * np.getNumPixels();
		size_t stop = constrain(phase * 2.f, 0, 1) * np.getNumPixels();
		if(stop < start)
			std::swap(start, stop);
		for(size_t n = start; n < stop && n < np.getNumPixels(); ++n)
		{
			rgb_t color = crossfade(kRgbGreen, kRgbRed, map(n, start, stop, 0, 1));
			color.scale(0.14); // dim to avoid using too much current
			np.setPixelColor(n, color.r, color.g, color.b);
		}
		return ms >= kAnimationDuration;
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
				menu_enterRangeDisplay(signalColor, {endpointsColorOut, endpointsColorOut}, true, outRangeMin, outRangeMax, outDisplay);
				// TODO: line below is just a workaround because we don't have a clean way of
				// _entering_ menu from here while ignoring the _last_ slider readings,
				// resulting in automatically re-entering immediately after exiting
				ledSliders.sliders[0].process(data.data());
			}
			if(performanceBtn.offset)
			{
				// press button: set input range
				menu_enterRangeDisplay(signalColor, {endpointsColorIn, endpointsColorIn}, false, inRangeBottom, inRangeTop, inDisplay);
				// TODO: line below is just a workaround because we don't have a clean way of
				// _exiting_ the menu from here while ignoring the _first_ slider readings
				ledSliders.sliders[0].process(data.data());
			}
		}
		// ugly workaround to turn on the red LED when in the "clipping" page
		if(inDisplayUpdated)
		{
			inDisplayUpdated--;
			tri.buttonLedSet(TRI::kSolid, TRI::kR, 1);
		}
		float outVizThrough = 0;
		float outVizEnv = 0;
		for(size_t n = 0; n < context->analogFrames; ++n)
		{
			float input = analogReadMapped(context, n, 0);
			// let's trust compiler + branch predictor to do a good job here
			float envIn;
			if(kCouplingAcRms == coupling)
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
				// compute RMS
				uint16_t newRmsVal = envIn * envIn * 65536.f;
				uint16_t oldRmsVal = rmsBuffer[rmsIdx];
				rmsAcc = rmsAcc + newRmsVal - oldRmsVal;
				rmsBuffer[rmsIdx] = newRmsVal;
				rmsIdx++;
				if(rmsIdx >= rmsBuffer.size())
					rmsIdx = 0;
				envIn = std::min(1.f, rmsAcc / 65536.f / float(rmsBuffer.size()));
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

			outVizThrough = envIn;
			outVizEnv = env;
			float outs[kNumOutChannels] = {0};
			switch (outputMode)
			{
			case kOutputModeNN: // top pass-through, bottom inverted pass-through
				outs[0] = input;
				outs[1] = 1.f - input;
				break;
			case kOutputModeNE: // top pass-through, bottom envelope
				outs[0] = input;
				outs[1] = env;
				break;
			case kOutputModeEE: // top envelope, bottom inverted envelope
				outs[0] = env;
				outs[1] = 1.f - env;
				break;
			}
			for(size_t c = 0; c < kNumOutChannels; ++c)
			{
				// TODO: combine this mapping with global mapping in tr_render(),
				// or at least reduce the number of times it gets called here if the compiler is not smart enough
				// TODO: should env also be mapped?
				float value = mapAndConstrain(outs[c], 0, 1, outRangeMin, outRangeMax);
				analogWriteOnce(context, n, c, value);
			}
		}
		// displays if in In/OutRange mode
		outDisplay = mapAndConstrain(outVizThrough, 0, 1, outRangeMin, outRangeMax);
		inDisplay = analogRead(context, 0, 0); // we always display the input range full-scale.
		// displays if in pure performance mode
		std::array<centroid_t,kNumOutChannels> centroids {};
		bool hasEnvelope = kOutputModeNN != outputMode;
		bool hasNormal = kOutputModeEE != outputMode;
		if(hasNormal)
		{
			centroids[0].location = outDisplay;
			centroids[0].size = kFixedCentroidSize;
		}
		const float kMagicLogCorrection = 2.f;
		if(hasEnvelope)
		{
			if(kCouplingAcRms == coupling)
				outVizEnv = log10f(1.f + kMagicLogCorrection * outVizEnv * 9.f);
			centroids[1].location = mapAndConstrain(outVizEnv, 0, 1, outRangeMin, outRangeMax);
			centroids[1].size = kFixedCentroidSize;
		}
		ledSliders.sliders[0].directBegin(); // clears display
		if(kCouplingAcRms == coupling)
		{
			//display color bar
			if(ledSliders.areLedsEnabled())
			{
				float throughLog = log10f(1.f + kMagicLogCorrection * outVizThrough * 9.f);
				colorBar(throughLog, outRangeMin * kNumLeds, outRangeMax * kNumLeds, kRgbGreen, kRgbRed);
			}
		}
		for(size_t n = 0; n < centroids.size(); ++n)
		{
			rgb_t color;
			if(kCouplingDc == coupling)
				color = signalColor;
			else
				color = crossfade(kRgbGreen, kRgbRed, map(centroids[n].location, outRangeMin, outRangeMax, 0, 1));
			ledSliders.sliders[0].directWriteCentroid(centroids[n], color, kCentroidSize);
		}
	}

	void updated(Parameter& p)
	{
		PerformanceMode::updated(p);
		if(p.same(cutoff))
		{
			// wrangling the range to make it somehow useful
			// TODO: put more method into this
			float par = cutoff;
			float bottom = 0.000005;
			float top = 0.0005;
			// we want par to be between about bottom and top
			// we want values to be closer to the bottom when the slider is higher
			par = 1.f - par;
			// We want changes to the input to have smaller effect closer to 0.000005
			par = powf(par, 3) * (top - bottom) + bottom;
			decay = 1.f - par;
		}
		else if(p.same(coupling)) {
			if(kCouplingAcRms == coupling)
			{
				rmsIdx = 0;
				rmsBuffer.fill(0);
				rmsAcc = 0;
			}
		}
		else if(p.same(outRangeMin) || p.same(outRangeMax)) {
		}
		else if(p.same(inRangeBottom) || p.same(inRangeTop)) {
		}
	}
	void animate(Parameter& p, LedSlider& l, rgb_t color, uint32_t ms) override
	{
		if(p.same(outputMode))
		{
			constexpr size_t kDuration = 1500;
			if(ms < kDuration)
			{
				gAnimateFs.writeInit(p, l);
				bool bottomEnv;
				bool topEnv;
				switch(outputMode.get())
				{
				default:
				case kOutputModeNN:
					bottomEnv = 0;
					topEnv = 0;
					break;
				case kOutputModeNE:
					bottomEnv = 1;
					topEnv = 0;
					break;
				case kOutputModeEE:
					bottomEnv = 1;
					topEnv = 1;
					break;
				}
				std::array<bool,2> isEnv {bottomEnv, topEnv};
				float in;
				// slightly smoothed pulse:
				const uint32_t first = 300;
				const uint32_t second = 970;
				const uint32_t third = 1000;
				if(ms < first)
				{
					// start low
					in = 0;
				} else if (ms < second)
				{
					// high
					in = 1;
				} else if (ms < third)
				{
					// ramp down
					in = 1 - (ms - second) / float(third - second);
				} else {
					// low
					in = 0;
				}
				float alpha = 0.998;
				static float pastEnv = 0;
				if(0 == ms)
					pastEnv = 0;
				env = in * (1.f - alpha) + pastEnv * alpha;
				pastEnv = env;
				for(size_t n = 0; n < isEnv.size(); ++n)
				{
					float start = 0.05 + n * 0.5;
					centroid_t centroid {
						.location = map(isEnv[n] ? env : in, 0, 1, start, start + 0.45),
						.size = kFixedCentroidSize,
					};
					gAnimateFs.directWriteCentroid(p, l, centroid, color);
				}
			}
		}
		else if(p.same(coupling))
		{
			// DC: show a single centroid move with a certain pattern
			// AC: show a colorBar doing the same
			static constexpr std::array<float,10> gesture = {
				0.3, 0.1, 0.1, 0.4, 0.1, 0.1, 0.8, 0.99, 0.8, 0.1,
			};
			constexpr uint32_t kDuration = 1200;
			if(ms < kDuration)
			{
				float idx = ms / float(kDuration);
				float loc = interpolatedRead(gesture, idx);
				gAnimateFs.writeInit(p, l);
				if(kCouplingDc == coupling)
					gAnimateFs.directWriteCentroid(p, l, { .location = loc, .size = kFixedCentroidSize }, color);
				else {
					colorBar(loc, AnimateFs::kLedStart, AnimateFs::kLedStop, color, color);
				}
			}
		}
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD3(outputMode, coupling, cutoff);
	}
	enum {
		kCouplingDc,
		kCouplingAcRms,
		kCouplingNum,
	};
	enum {
		kOutputModeNN,
		kOutputModeNE,
		kOutputModeEE,
		kOutputModeNum
	};
	ScaleMeterMode() :
		PerformanceMode(kRgbGreen),
		presetFieldData{
			.ioRanges = ioRangesParameters,
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
	ParameterContinuous outRangeMin {this, 0};
	ParameterContinuous outRangeMax {this, 1};
	ParameterContinuous inRangeBottom {this, 0};
	ParameterContinuous inRangeTop {this, 1};
	PACKED_STRUCT(PresetFieldData_t {
		IoRanges ioRanges;
		int outputMode;
		int coupling;
		float cutoff;
	}) presetFieldData;
	size_t inDisplayUpdated;
	float inDisplay;
private:
	float outDisplay;
	float decay;
	float analogReadMapped(BelaContext* context, size_t frame, size_t channel)
	{
		float in = analogRead(context, frame, channel);
		return mapAndConstrain(in, inRangeBottom, inRangeTop, 0, 1);
	}
	void colorBar(float value, size_t startLed, size_t stopLed, rgb_t colorStart, rgb_t colorStop)
	{
		float top = map(value, 0, 1, startLed, stopLed);
		for(size_t n = startLed; n < stopLed && n < kNumLeds; ++n)
		{
			const rgb_t color = crossfade(colorStart, colorStop, map(n, startLed, stopLed, 0, 1));
			float scale = 0.15; // dimmed to avoid using too much current
			if(n > top)
				scale = 0;
			else if(top == n)
				scale *= top - int(top); // smooth the top LED
			np.setPixelColor(n, color.scaledBy(scale));
		}
	}
	const rgb_t signalColor = kRgbGreen;
	const rgb_t endpointsColorIn = kRgbRed;
	const rgb_t endpointsColorOut = kRgbYellow;
	float x1;
	float y1;
	float env;
	size_t count;
	float rms;
	std::array<uint16_t,512> rmsBuffer;
	size_t rmsIdx = 0;
	float rmsAcc = 0;
} gScaleMeterMode;
#endif // ENABLE_SCALE_METER_MODE

#ifdef ENABLE_BALANCED_OSCS_MODE
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
		PerformanceMode::updated(p);
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
			.ioRanges = ioRangesParameters,
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
		presetDescSet(6, &presetDesc);
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
		IoRanges ioRanges;
		float centreFrequency = centreFrequency;
		uint8_t waveform = waveform;
		uint8_t inputMode = inputMode;
	}) presetFieldData;
private:
	float divisionPoint = 0.5;
	uint32_t lastClockPeriodUpdateCounter = gClockPeriodUpdateCounter;
} gBalancedOscsMode;
#endif // ENABLE_BALANCED_OSCS_MODE

#define clickprintf(...) // use to enable debug printing in case of need

#ifdef ENABLE_EXPR_BUTTONS_MODE
#define MENU_ENTER_SINGLE_SLIDER
static void menu_enterSingleSlider(const rgb_t& color, const rgb_t& otherColor, ParameterContinuous& parameter);

class ExprButtonsMode : public PerformanceMode
{
public:
	bool setup(double ms)
	{
		updateNumButtons();
		gOutIsSize = {false, true};
		if(ms <= 0)
		{
			ledSlidersSetupOneSlider(colors[0], LedSlider::MANUAL_CENTROIDS);
			gOutMode.fill(kOutModeManualBlock);
			changeState(kDisabled, {0, 0});
		}
		changePage(kPagePerf);
		// Force initialisation of offsets. Alternatively, always copy them in render()
		for(auto& o : offsetParameters)
			updated(o);
		if(ms < 0)
			return true;
		// animation
		// buttons appear and disappear one by one, fading in and out quickly
		constexpr size_t kAnimationDuration = 1400;
		constexpr size_t kPerButton = kAnimationDuration / kMaxNumButtons;
		size_t button = std::min(kMaxNumButtons - 1, size_t(ms) / kPerButton);
		size_t phase = size_t(ms) % kPerButton;
		float tri = simpleTriangle(phase, kPerButton);
		float step = 1.f / kMaxNumButtons;
		for(auto l : { &ledSliders, &ledSlidersAlt})
		{
			l->sliders[0].directBegin();
			l->sliders[0].directWriteCentroid({ .location = step * 0.5f + step * button, .size = tri * 0.2f}, colors[button]);
		}
		return ms >= kAnimationDuration;

	}
	void render(BelaContext* context, FrameData* frameData)
	{
		constexpr float kDefaultSize = 0.5;
		ButtonView btn = ButtonViewSimplify(performanceBtn);
		gInUsesRange = false;
		gOutUsesRange[0] = false;
		gOutUsesRange[1] = true;
		if(gAlt && kKeyInvalid != keyBeingAdjusted && !seqMode)
		{
			// if we are adjusting the pitch, output that instead
			gManualAnOut[0] = getOutForKey(keyBeingAdjusted);
			gManualAnOut[1] = kDefaultSize;
			return;
		}
		if(!gAlt)
			keyBeingAdjusted = kKeyInvalid;
		frameId = frameData->id;
		TouchTracker::TouchWithId twi {centroid_t{0, 0}, 0, TouchTracker::kIdInvalid };
		// normal processing
		if(ledSliders.isTouchEnabled())
		{
			gTouchTracker.process(globalSlider);
			if(gTouchTracker.getNumTouches())
				twi = gTouchTracker.getTouchMostRecent();
		}
		if(btn.offset)
		{
			// single click goes back to kPagePerf
			clickprintf("o%d%d\n\r", onClickGroupStartWas, page);
			onClickGroupStartWas = page;
			changePage(kPagePerf);
		}
		// double click enters (or exits) set enable page (both keys an sequencer)
		if(btn.doubleClickOffset)
		{
			clickprintf("d%d%d->", onClickGroupStartWas, page);
			if(kPageSetMode == page || kPageSetMode == onClickGroupStartWas)
				changePage(kPagePerf);
			else
				changePage(kPageSetMode);
			clickprintf("%d\n\r", page);
		}
		// triple click enters (or exits) sampling page (both keys an sequencer)
		if(btn.tripleClickOffset)
		{
			clickprintf("t%d%d->", onClickGroupStartWas, page);
			if(kPageTuning == page || kPageTuning == onClickGroupStartWas)
				changePage(kPagePerf);
			else
				changePage(kPageTuning);
			clickprintf("%d\n\r", page);
		}
		pastNumTouches = globalSlider.getNumTouches();

		centroid_t centroid = processSize(twi.touch, 0);
		// NOTE: newTouch is true only on the _first_ kInitial frame (i.e.: new twi.id)
		bool newTouch = false;
		if(TouchTracker::kIdInvalid == twi.id)
		{
			// touch is removed
			if(kDisabled != touch.state && (kHold != touch.state || seqMode))
				changeState(kDisabled, centroid);
		} else {
			// if it is a new touch, assign this touch to a key, store location as impact location,
			// mark it as unmoved
			if(kDisabled == touch.state || (kHold == touch.state && touch.holdHasReleased) || pastTouchId != twi.id)
			{
				newTouch = true;
				changeState(kInitial, centroid); // note that this may fail and we go back to kDisabled
			}
		}
		// TODO: maybe pastTouchId should be remembered as part of changeState()?
		pastTouchId = twi.id;
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
		// the first touch have a spurious location (it's an artifact of the sensor)
		// therefore the touch is only kGood after a new frameId is received
		if(kInitial == touch.state && touch.initialFrameId != frameId)
			changeState(kGood, centroid);
		// if it is not a new touch and it has moved enough, mark it as moved
		if(kGood == touch.state)
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
			for(key = 0; key < numButtons; ++key)
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
			if(key == numButtons) {
				// we are not in a dead spot
				touch.bendDeadTime = 0;
				if(fabsf(centroid.location - getMidLocationFromKey(touch.key)) < step - kBendDeadSpot)
				{
					// we are between the initial key and a neighbour's dead spot
					// nothing to do
				} else {
					changeState(kInitial, centroid);
					if(centroid.location <= getMidLocationFromKey(0) || centroid.location >= getMidLocationFromKey(numButtons - 1))
					{
						// first or last button: we start a new touch there without bending
					} else {
						// we went past the dead zone towards the next key.
						// Start a new bending towards that.
						changeState(kBending, centroid);
					}
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
					// TODO: should we get to kMoved already?
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
		case kGood:
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
		case kGood:
			out = getOutForKey(touch.key);
			break;
		case kMoved:
			out = getOutForKey(touch.key) + touch.mod * modRange * 1.f;
			break;
		case kBending:
		{
			float bendIdx;
			float bendRange;

			float diff = centroid.location - touch.initialLocation;
			float bendSign = (diff > 0) ? 1 : -1;
			size_t bendDestKey = touch.key + bendSign * 1;
			if(bendDestKey >= numButtons) {
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
		{
			// on release we may have a different out from the nominal pitch
			// e.g.: if we released from kMoved or kBending
			// here we smoothly get back to nominal pitch
			float alpha = 0.95;
			out = alpha * out + (1.f - alpha) * getOutForKey(touch.key);
		}
			break;
		case kNumStates:
			break;
		}
		gManualAnOut[0] = out; // we always hold pitch here
		gManualAnOut[1] = touchOrNot(centroid).size;
		pastOuts = gManualAnOut;
		if(kPageTuning == page)
		{
			// override any output that may have happened so far.
			// We leverage the state machine above even if it's
			// more complicated than / slightly different from
			// what we need here
			switch(touch.state)
			{
			case kBending:
				// if bending, we enter the slider menu that allows manually setting the pitch
				if(!gAlt)
				{
					keyBeingAdjusted = touch.key;
					// TODO: clear up this mess of actualKey vs touch.key.
					size_t actualKey = seqMode ? touch.key : keysIdx[touch.key];
					const IoRange& outTop = ioRangesParameters.outTop;
					float min;
					float max;
					outTop.getMinMax(min, max);
					// reverse map the range so that we start with an initial centroid that represents
					// the current voltage
					float initialValue = mapAndConstrain(offsetParameters[actualKey], min, max, 0, 1);
					offsetParameterRaw.set(initialValue);
					menu_enterSingleSlider(colors[actualKey], colors[actualKey], offsetParameterRaw);
					sampledKey = kKeyInvalid; // avoid assigning the sampled value to the key on release
					// once we bend once, don't allow sampling
					// till the next time we enter the page
					samplingEnabled = false;
				}
				break;
			case kDisabled:
				// TODO: pass-through at audio rate unless key is pressed
				if(samplingEnabled)
				{
					gManualAnOut[0] = quantise(analogRead(context, 0, 0));
					gManualAnOut[1] = kDefaultSize;
				}
				break;
			case kInitial:
				if(samplingEnabled)
				{
					if(newTouch && touch.key < keysIdx.size())
					{
						// sample
						float sum = 0;
						for(size_t n = 0; n < context->analogFrames; ++n)
							sum += analogRead(context, n, 0);
						sampled = sum / context->analogFrames;
						sampledKey = keysIdx[touch.key];
					}
					// we postpone assigning to offsets so that if we get
					// into bending to set the voltage via slider, we do not
					// accidentally assign it the sampled input on press
				}
				// no break
			case kMoved:
			case kGood:
				if(samplingEnabled)
				{
					// hold
					gManualAnOut[0] = quantise(sampled);
					gManualAnOut[1] = centroid.size;
				}
				break;
			case kHold:
				// won't be here
			case kNumStates:
				break;
			}
			if(samplingEnabled)
			{
				if(kDisabled == touch.state && stateIsNormal(samplingPastTouchState))
				{
					// upon release, we finally assign
					if(kKeyInvalid !=  sampledKey)
						offsetParameters[sampledKey].set(sampled);
				}
				samplingPastTouchState = touch.state;
			}
		}

		size_t vizKey;
		bool analogInHigh = tri.analogRead() > 0.5;
		bool analogRisingEdge = (analogInHigh && !pastAnalogInHigh);
		pastAnalogInHigh = analogInHigh;
		if(seqMode)
		{
			if(analogRisingEdge)
			{
				pastAnalogRisingEdgeSamples = context->audioFramesElapsed;
				if(kPagePerf == page)
				{
					// if a finger is down, do not move from that step
					if(kDisabled != touch.state)
						seqNextStep = touch.key;
				}
				size_t attempts = 0;
				do
				{
					attempts++;
					if(seqNextStep >= numButtons)
						seqNextStep = 0;
					seqCurrentStep = seqNextStep;
					seqNextStep++;
				} while (!stepIsEnabled(seqCurrentStep) && attempts < numButtons);
#if 0 // print step state
				for(size_t n = 0; n < numButtons; ++n)
				{
					char sym0 = stepIsEnabled(n) ? '_' : 'X';
					if(stepIsEnabled(n) && seqCurrentStep == n)
						sym0 = '*';
					char sym1 = 'O';
					switch(seqStepsMode[n])
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
					case kStepDisabled:
						sym1 = 'x';
						break;
					case kStepModesNum:
						sym1 = 'o';
						break;
					}
					printf("%c%c,", sym0, sym1);
				}
				printf("\n\r");
#endif
				tri.buttonLedSet(TRI::kSolid, TRI::kY, 1, getBlinkPeriod(context, kPagePerf != page));
			}
			bool newTriggerableStep = false;
			if(kDisabled != touch.state)
			{
				// if we have a touch
				switch(page)
				{
				case kPagePerf:
				{
					// on a new touch
					if(twi.id != seqPastTouchIdUpdated)
					{
						seqPastTouchIdUpdated = twi.id;
						// reset to a next or just passed edge (reset immediately if clock is absent)
						uint64_t maxDelaySamples = std::min(gClockPeriod * 0.25f, 0.1f * context->analogSampleRate);
						if(context->audioFramesElapsed - pastAnalogRisingEdgeSamples < maxDelaySamples)
						{
							// close enough to edge
							setStepNow(touch.key);
						}
						else {
							// late enough, schedule pressed key for next
							setNextStep(touch.key);
						}
						if(!clockInIsActive(context))
						{
							newTriggerableStep = true;
							setStepNow(touch.key);
						}
					}
				}
					break;
				case kPageSetMode:
					if(newTouch)
					{
						// each new key press cycles through step states
						KeyStepMode mode = keyStepModes[touch.key];
						mode.s = StepMode(mode.s + 1);
						if(kStepModesNum == mode.s)
							mode.s = kStepNormal;
						// do not allow to remove all keys
						size_t numEnabledKeys = countEnabledKeys();
						if(numEnabledKeys < 2 && kStepDisabled == mode.s)
							mode.s = kStepNormal;
						keyStepModes[touch.key].set(mode);
					}
					break;
				case kPageTuning:
					// if no clock, jump to current key so it can be previewed while tuning
					if(!clockInIsActive(context))
						setStepNow(touch.key);
					break;
				}
			}
			vizKey = seqCurrentStep;
			size_t outKey = kKeyInvalid;
			if(kPageTuning == page && !clockInIsActive(context))
			{
				// no clock and we are tuning a step: we want to hear it right now
				// ignore the step's settings
				outKey = seqCurrentStep;
			} else {
				switch(keyStepModes[seqCurrentStep].get().s)
				{
				case kStepNormal:
					outKey = seqCurrentStep;
					newTriggerableStep |= analogRisingEdge;
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
							step = numButtons;
						step--;
						if(kStepNormal == keyStepModes[step].get().s)
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
			}
			gManualAnOut[0] = kKeyInvalid == outKey ? kNoOutput : seqSmooth(getOutForKey(outKey));
			size_t lowestEnabled = 0;
			while(lowestEnabled < keyStepModes.size() && !stepIsEnabled(lowestEnabled))
				lowestEnabled++;
			gOutUsesRange[1] = false; // we output fix voltagese
			if(kPageTuning == page && !clockInIsActive(context))
			{
				// if pressing a key in tuning page, or tuning it, send out a 5V gate
				if(kDisabled == touch.state && kKeyInvalid == keyBeingAdjusted)
					triggerOut = vToOut(0);
				else
					triggerOut = vToOut(5);
			} else
			{
				// send out a +5V trigger on each new step
				// and send out a +10V reset signal when on the first step (if it is triggerable, anyhow)
				if(newTriggerableStep)
				{
					float outV = newTriggerableStep * 5;
					if(outV)
					{
						if(seqCurrentStep == lowestEnabled)
							outV = 10;
					}
					triggerOut = vToOut(outV);
					lastTriggerOutSet = context->audioFramesElapsed;
				}
				float ms = (context->audioFramesElapsed - lastTriggerOutSet) / context->analogSampleRate * 1000;
				if(ms > getBlinkPeriod(context, false))
				{
					triggerOut = vToOut(0);
				}
			}
			gManualAnOut[1] = triggerOut;
			seqPastStep = seqCurrentStep;
		} else {
			// if not seqMode
			if(kPageSetMode == page)
			{
				if(newTouch)
				{
					size_t numEnabledKeys = countEnabledKeys();
					// only remove a key if you're not left with 0
					if(numEnabledKeys >= 2 || !keyIsEnabled(touch.key))
					{
						KeyStepMode mode = keyStepModes[touch.key];
						mode.k = !mode.k;
						keyStepModes[touch.key].set(mode);
					}
					updateNumButtons();
				}
				if(touch.key < kMaxNumButtons && keyIsEnabled(touch.key) && stateIsNormal(touch.state))
				{
					gManualAnOut[0] = getOutForKey(touch.key);
					gManualAnOut[1] = centroid.size;
				} else {
					gManualAnOut[0] = kNoOutput;
					gManualAnOut[1] = kNoOutput;
				}
			}
			vizKey = kDisabled == touch.state ? kKeyInvalid : touch.key;
			// turn on green LED if we are in a stable position
			if(stateIsNormal(touch.state))
				tri.buttonLedSet(TRI::kSolid, TRI::kG, 0.1);
		}
		// display
		if(!gAlt)
		{
			// button
			switch(page)
			{
			case kPageTuning:
				tri.buttonLedSet(TRI::kSolid, TRI::kR);
				break;
			case kPageSetMode:
				tri.buttonLedSet(TRI::kGlow, TRI::kR);
				break;
			default:
				break;
			}
			// slider leds
			ledSliders.sliders[0].directBegin();
			for(size_t n = 0; n < numButtons; ++n)
			{
				float coeff = (n == vizKey) ? 1 : 0.1; // may be overridden
				rgb_t color;
				float period = 0.3f * context->analogSampleRate;
				float triangle = simpleTriangle(context->audioFramesElapsed, period);
				if(kPageTuning == page)
				{
					// same behaviour for seqMode and non-seqMode
					// though in seqMode it always has kmaxNumButtons buttons,
					// while in key mode it only has numButtons
					color = colors[seqMode ? n : keysIdx[n]];
					if(n == vizKey)
						coeff = 1;
					else
					{
						// inactive keys while sampling have a triangle pattern
						float period = 0.5f * context->analogSampleRate;
						coeff *=  0.1f + 0.9f * simpleTriangle(context->audioFramesElapsed + (n * period / numButtons), period);
					}
				} else if (seqMode)
				{
					coeff *= stepIsEnabled(n);
					switch(keyStepModes[n].get().s)
					{
					case kStepModesNum:
					case kStepNormal:
						color = kRgbGreen;
						break;
					case kStepHold:
						color = kRgbYellow.scaledBy(0.7);
						break;
					case kStepMuted:
						color = kRgbRed.scaledBy(0.7);
						break;
					case kStepDisabled:
						color = kRgbBlack;
						break;
					}
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
					case kPageTuning: // shouldn't be here anyhow
						break;
					}
				} else {
					// we are in keys mode
					size_t idx;
					if(kPageSetMode == page)
					{
						idx = n;
						//always has kNumMaxButtons buttons
						coeff *=  (touch.key == idx && touch.state != kDisabled) ? 1 : triangle > 0.7f;
						coeff *= keyIsEnabled(idx);
					} else {
						idx = keysIdx[n];
					}
					if(idx < kMaxNumButtons)
						color = colors[idx];
					else // shouldn't get here
						color = {0, 0, 0};
				}
				ledSliders.sliders[0].directWriteCentroid(centroid_t { getMidLocationFromKey(n), coeff }, color,
						LedSlider::kDefaultNumWeights + (kMaxNumButtons - numButtons) / (0.125f * kMaxNumButtons)
				);
			}
		}
	}
private:
	typedef enum {
		kPagePerf,
		kPageSetMode,
		kPageTuning,
	} Page;
	typedef enum {
		kInitial,
		kGood,
		kMoved,
		kBending,
		kHold,
		kDisabled,
		kNumStates,
	} TouchState;
	static bool stateIsNormal(TouchState state)
	{
		return
				kInitial == state
				|| kGood == state
				|| kMoved == state
			;
	}
	size_t countEnabledKeys()
	{
		// we compute the number of enabled keys from scratch
		// because numButtons is always kNumMaxButtons when in kPageSetMode
		size_t numEnabledKeys = 0;
		for(auto& k : keyStepModes)
			numEnabledKeys += seqMode ? (k.get().s != kStepDisabled) :  k.get().k;
		return numEnabledKeys;
	}
	float seqSmooth(float value)
	{
		seqOut = seqOut * seqAlpha + value * (1.f - seqAlpha);
		return seqOut;
	}
	bool keyIsEnabled(size_t n) {
		if(n >= keyStepModes.size())
			return false;
		return keyStepModes[n].get().k;
	}
	bool stepIsEnabled(size_t n) {
		if(n >= keyStepModes.size())
			return false;
		return (kStepDisabled != keyStepModes[n].get().s);
	}
	const std::array<const char*,kNumStates> touchStateNames {
			"kInitial",
			"kGood",
			"kMoved",
			"kBending",
			"kHold",
			"kDisabled",
	};
	float getOutForKey(size_t key)
	{
		if(key >= numButtons)
			return 0;
		size_t actualKey = seqMode ? key : keysIdx[key];
		if(actualKey >= offsetParameters.size())
			return 0;
		return quantise(offsetParameters[actualKey]);
	}
	float quantise(float in, bool force = false)
	{
		if(quantised || force)
			return quantiseToSemitones(in);
		else
			return in;
	}
	void changePage(Page newPage)
	{
		if(page != newPage)
		{
			page = newPage;
			updateNumButtons();
			if(kPagePerf == page)
			{
				// we may have updated keyStepModes and/or offsetParameters
				updatePreset();
			}
			if(kPageTuning == page)
			{
				// if in keys mode start by allowing sampling (may be later disabled)
				samplingEnabled = seqMode ? false : true;
			}
		}
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
		case kGood:
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
			break;
		case kNumStates:
			break;
		}
		S(printf("\n\r"));
		touch.state = newState;
		touch.initialLocation = centroid.location;
		touch.initialOut = out;
		touch.initialFrameId = frameId;
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
				|| (diff > 0 && key < (numButtons - 1))
				)
				return true;
		}
		return false;
	}
	float getMidLocationFromKey(size_t key)
	{
		return key * step + step * 0.5f;
	}

	ssize_t getKeyFromLocation(float location)
	{
		size_t key;
		// identify candidate key
		for(key = 0; key < numButtons; ++key)
		{
			float top = (key + 1) * step;
			if(location <= top)
				break;
		}
		if(numButtons == key)
			return -1; // weird ...
		// validate that we are not _too far_ from the center
		float centre = getMidLocationFromKey(key);
		if(fabsf(location - centre) > kMaxDistanceFromCenter)
			return -1;
		return key;
	}
	static constexpr size_t kMaxNumButtons = 5;
	void updateNumButtons()
	{
		// update keysIdx
		size_t count = 0;
		for(size_t n = 0; n < kMaxNumButtons; ++n)
		{
			if(keyIsEnabled(n))
			{
				keysIdx[count] = n;
				count++;
			}
		}
		std::fill(keysIdx.begin() + count, keysIdx.end(), kMaxNumButtons);
#if 0
		// log the resulting idxs
		for(size_t n = 0; n < keyIsEnabled(n); ++n)
			printf("%d ", keysIdx[n]);
		printf("<<\n\r");
#endif

		if(seqMode || kPageSetMode == page)
			numButtons = kMaxNumButtons;
		else
			numButtons = std::min(count, static_cast<size_t>(kMaxNumButtons));

		step = 1.f / numButtons;
		kMaxDistanceFromCenter = step * 0.85f;
		kMoveThreshold = step * 0.1f;
		kBendStartThreshold = step * 0.4f; // could be same as kMaxDistanceFromCenter?
		kBendDeadSpot = step * 0.2f;
	}
	void setStepNow(size_t key)
	{
		seqCurrentStep = key;
		setNextStep((seqCurrentStep + 1) % kMaxNumButtons);
	}
	void setNextStep(size_t key)
	{
		seqNextStep = key;
	}
	size_t numButtons;
	float step;
	float kMaxDistanceFromCenter;
	float kMoveThreshold;
	float kBendStartThreshold;
	float kBendDeadSpot;
	uint64_t lastTriggerOutSet = 0;
	float triggerOut = 0;
	float seqOut = 0;
	float seqAlpha = 0;
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
		uint32_t initialFrameId;
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
		kStepDisabled,
		kStepModesNum,
	};
	struct KeyStepMode {
		bool k : 4;
		StepMode s : 4;
		static KeyStepMode getDefault() {
			return KeyStepMode{.k = true, .s = kStepNormal};
		}
	};
	std::array<uint8_t,kMaxNumButtons> keysIdx = FILL_ARRAY(keysIdx, kMaxNumButtons);
	std::array<float,kNumOutChannels> pastOuts;
	TouchTracker::Id pastTouchId;
	size_t pastNumTouches = 0;
	float sampled = 0;
	size_t sampledKey = kKeyInvalid;
	TouchState samplingPastTouchState = kDisabled;
	Page page = kPagePerf;
	// this is used in order to ignore effect of single and {single,double} click when
	// processing a double or triple click, respectively.
	Page onClickGroupStartWas = kPagePerf;
	size_t seqCurrentStep = 0;
	size_t seqPastStep = -1;
	size_t seqNextStep = 1;
	TouchTracker::Id seqPastTouchIdUpdated = TouchTracker::kIdInvalid;
	uint64_t pastAnalogRisingEdgeSamples = 0;
	bool pastAnalogInHigh = false;
	bool samplingEnabled;
	std::array<rgb_t,kMaxNumButtons> colors = {{
		crossfade(kRgbGreen, kRgbOrange, 0),
		crossfade(kRgbGreen, kRgbOrange, 0.25),
		crossfade(kRgbGreen, kRgbOrange, 0.5),
		crossfade(kRgbGreen, kRgbOrange, 0.75),
		crossfade(kRgbGreen, kRgbOrange, 1),
	}};
public:
	void updated(Parameter& p)
	{
		PerformanceMode::updated(p);
		if(p.same(modRange)) {
			seqAlpha = modRange < 0.05 ? 0 : 0.9 + modRange * 0.09996; // stay clear of 1.0. TODO: more usable mapping
		} else if(p.same(seqMode)) {
			updateNumButtons();
		} else if(p.same(quantised)) {
		} else if(p.same(offsetParameterRaw))
		{
			IoRange outRange = ioRangesParameters.outTop;
			float min;
			float max;
			outRange.getMinMax(min, max);
			float value = map(offsetParameterRaw, 0, 1, min, max);
			offsetParameters[seqMode ? keyBeingAdjusted : keysIdx[keyBeingAdjusted]].set(value);
		} else {
			for(size_t n = 0; n < kMaxNumButtons; ++n)
			{
				if(p.same(keyStepModes[n]))
				{
					updateNumButtons();
					break;
				}
			}
		}
	}
	void animate(Parameter& p, LedSlider& l, rgb_t color, uint32_t ms) override
	{
		constexpr uint32_t kDuration = 1000;
		if(p.same(quantised))
		{
			// continuous: show a centroid move smoothly from bottom to top
			// quantised: same as above but with visible steps
			if(ms < kDuration)
			{
				gAnimateFs.writeInit(p, l);
				float loc = simpleRamp(ms, kDuration);
				if(quantised)
					loc = std::floor(loc * 5) / 5.f + 0.1f;
				gAnimateFs.directWriteCentroid(p, l, { .location = loc, .size = kFixedCentroidSize}, color);
			}
		} else if(p.same(seqMode)) {
			if(ms < kDuration)
			{
				// keys: display three keys, static
				// sequencer: display three keys with each setting highlighted in turn (as if it was going through the sequencer)
				gAnimateFs.writeInit(p, l);
				constexpr size_t kNumAnimationKeys = 2;
				std::array<float,kNumAnimationKeys> locs = {0.2, 0.7};
				size_t h = size_t(std::floor(ms / float(kDuration) * kNumAnimationKeys * 2)) % locs.size(); // highlighted key
				for(size_t n = 0; n < locs.size(); ++n)
				{
					float size = kFixedCentroidSize;
					if(seqMode)
					{
						if(h == n)
							size *= 2;
						else
							size *= 0.5f;
					}
					gAnimateFs.directWriteCentroid(p, l, { .location = locs[n], .size = size }, color, 2);
				}
			}
		}
	}
	void updatePreset()
	{
		UPDATE_PRESET_FIELD3PlusArrays(quantised, seqMode, modRange, offsetParameters, keyStepModes);
	}
	ExprButtonsMode() :
		PerformanceMode(kRgbOrange),
		presetFieldData {
			.ioRanges = ioRangesParameters,
			.quantised = quantised,
			.seqMode = seqMode,
			.modRange = modRange,
			.offsetParameters = {
				offsetParameters[0],
				offsetParameters[1],
				offsetParameters[2],
				offsetParameters[3],
				offsetParameters[4],
			},
			.keyStepModes = {
				keyStepModes[0],
				keyStepModes[1],
				keyStepModes[2],
				keyStepModes[3],
				keyStepModes[4],
			},
		}
	{
		PresetDesc_t presetDesc = {
			.field = this,
			.size = sizeof(PresetFieldData_t),
			.defaulter = genericDefaulter3PlusArrays(ExprButtonsMode, quantised, seqMode, modRange, offsetParameters, keyStepModes),
			.loadCallback = genericLoadCallback3PlusArrays(ExprButtonsMode, quantised, seqMode, modRange, offsetParameters, keyStepModes),
		};
		presetDescSet(3, &presetDesc);
	}
	ParameterEnumT<2,bool> quantised {this, false};
	ParameterEnumT<2,bool> seqMode{this, false};
	ParameterContinuous modRange {this, 0.1};
	ParameterContinuous offsetParameterRaw {this, 0};
	// do not retrieve offsetParameters directly, use getOutForKey() instead
	std::array<ParameterContinuous,kMaxNumButtons> offsetParameters {
		ParameterContinuous(this, semiToNorm(66)), // F#
		ParameterContinuous(this, semiToNorm(67)), // G
		ParameterContinuous(this, semiToNorm(69)), // A
		ParameterContinuous(this, semiToNorm(71)), // B
		ParameterContinuous(this, semiToNorm(74)), // D
	};
	std::array<ParameterGeneric<KeyStepMode>,kMaxNumButtons> keyStepModes = FILL_ARRAY(keyStepModes, {this, KeyStepMode::getDefault()});
	PACKED_STRUCT(PresetFieldData_t {
		IoRanges ioRanges;
		uint8_t quantised;
		uint8_t seqMode;
		float modRange;
		std::array<float,kMaxNumButtons> offsetParameters;
		std::array<KeyStepMode,kMaxNumButtons> keyStepModes;
	}) presetFieldData;
private:
	float out = 0;
	size_t keyBeingAdjusted = kKeyInvalid;
	uint32_t frameId = -1;
} gExprButtonsMode;
#endif // ENABLE_EXPR_BUTTONS_MODE

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
ParameterGeneric<uint8_t> dummy {this, 0}; // so that we don't need too many noioranges macros

typedef enum {
	kFindingDacGnd,
	kFindingAdcVals,
	kFindingDone,
} Connected_t;
Connected_t connectedState;
unsigned int findingAdcIdx;
size_t count;
float adcAccu;
size_t startCountBlocks;
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
static constexpr unsigned kConnectedStepCount = 60;
static constexpr unsigned kWaitAfterSetting = 3;
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
		.dummy = dummy,
	})
{
	publishCalibrationData(); // load factory settings
	PresetDesc_t presetDesc = {
		.field = this,
		.size = sizeof(PresetFieldData_t),
		.defaulter = genericDefaulterNoioranges3(CalibrationProcedure, calibrationOut, calibrationIn, dummy),
		.loadCallback = genericLoadCallbackNoioranges3(CalibrationProcedure, calibrationOut, calibrationIn, dummy),
	};
	presetDescSet(4, &presetDesc);
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
	UPDATE_PRESET_NOIORANGES_FIELD3(calibrationOut, calibrationIn, dummy);
}
PACKED_STRUCT(PresetFieldData_t {
	CalibrationData calibrationOut;
	CalibrationData calibrationIn;
	uint8_t dummy;
}) presetFieldData;
void setup()
{
	count = 0;
	calibrationState = kWaitToStart;
	printf("Disconnect INPUT\n\r"); // TODO: this is printed repeatedly till you release the button
	gOutMode.fill(kOutModeManualBlock);
}

void process(BelaContext* context)
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
			// when this first starts, we may still be using calibration
			// on the inputs, so wait a couple of blocks to ensure we get
			// non-calibrated inputs
			if(startCountBlocks < kWaitAfterSetting)
			{
				startCountBlocks++;
				break;
			}
			for(size_t n = 0; n < context->analogFrames; ++n)
			{
				adcAccu += analogRead(context, n, 0) * (1.f / float(kAverageCount));
				count++;
				if(kAverageCount == count)
				{
					calibrationState = kWaitConnect;
					inGnd = adcAccu;
					printf("inGnd: %.5f, connect an input\n\r", inGnd);
					outCode = 0; // set this as a test value so we can detect when DAC is connected to ADC
					count = 0;
					break;
				}
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
				outCode = kRangeStart;
				connectedState = kFindingDacGnd;
				startCountBlocks = 0;
			}
			break;
		case kConnected:
		{
			switch(connectedState)
			{
				case kFindingDacGnd:
				{
					if(startCountBlocks < kWaitAfterSetting)
					{
						startCountBlocks++;
						count = 0;
						adcAccu = 0;
					} else {
						for(size_t n = 0; n < context->analogFrames; ++n)
						{
							adcAccu += analogRead(context, n, 0) * (1.f / float(kConnectedStepCount));
							count++;
							if (kConnectedStepCount == count)
							{
								float average = adcAccu;
								float diff = average - inGnd;
								diff = std::abs(diff);
								if(diff < minDiff)
								{
									minDiff = diff;
									minCode = outCode;
								}
								startCountBlocks = 0;
								outCode += kStep;
								if(outCode >= kRangeStop)
								{
									printf("Gotten a minimum at code %u (%f), diff: %f)\n\r", minCode, fromCode(minCode), minDiff);
									outGnd = fromCode(minCode);
									// now that outGnd is set, we can use fromVolt()
									outTop = fromVolt(kIoTopV);
									outBottom = fromVolt(kIoBottomV);
									printf("DAC: %.2fV: %f(%d), %.2fV: %f(%d), %.2fV: %f(%d)\n\r",
											kIoBottomV, outBottom, toCode(outBottom),
											kIoGndV, outGnd, toCode(outGnd),
											kIoTopV, outTop, toCode(outTop));
									startCountBlocks = 0;
									connectedState = kFindingAdcVals;
									findingAdcIdx = 0;
								}
								break;
							}
						}
					}
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
					if(startCountBlocks < kWaitAfterSetting)
					{
						startCountBlocks++;
						adcAccu = 0;
						count = 0;
					} else {
						for(size_t n = 0; n < context->analogFrames; ++n)
						{
							adcAccu += analogRead(context, n, 0) * 1.f / float(kAverageCount);
							++count;
							if(kAverageCount == count) {
								// store the resulting value
								float value = adcAccu;
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
								startCountBlocks = 0;
							}
						}
					}
				}
					break;
				case kFindingDone:
					calibrationState = kDone;
					count = 0;
					printf("ADC: %.2fV: %f(%d), %.2fV: %f(%d), %.2fV: %f(%d)\n\r",
							kIoBottomV, inBottom, toCode(inBottom),
							kIoGndV, inGnd, toCode(inGnd),
							kIoTopV, inTop, toCode(inTop));
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
	startCountBlocks = 0;
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

static constexpr rgb_t kCalibrationColor = kRgbRed;
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
		resetDemoMode();
		ledSlidersSetupOneSlider(
			baseColor,
			LedSlider::MANUAL_CENTROIDS
		);
		return true;
	}
	void render(BelaContext* context, FrameData* frameData) override
	{
		gOutMode.fill(kOutModeManualBlock);
		// these may be overridden below if calibration is done
		gOutUsesCalibration = false;
		gInUsesCalibration = false;
		gInUsesRange = false;
		gOutUsesRange = {false, false};
		uint32_t tick = HAL_GetTick();
		// wait for button press to start or stop calibration.
		ButtonView btn = ButtonViewSimplify(performanceBtn);
		if(btn.offset ||
				(btn.tripleClick && gCalibrationProcedure.done()))
		{
			gCalibrationProcedure.toggle();
			resetDemoMode();
		}
		gCalibrationProcedure.process(context);

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
		rgb_t color = kRgbOrange;
		rgb_t otherColor = kRgbBlack;
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
				np.setPixelColor(n, color.scaledBy(0.2));
		}
	}
	void updatePreset() override
	{
		gCalibrationProcedure.updatePreset();
	}
} gCalibrationMode(kCalibrationColor);

class FactoryTestMode: public PerformanceMode {
public:
	bool setup(double ms) override
	{
		ledSlidersSetupOneSlider(kRgbBlack, LedSlider::MANUAL_DIRECT); // dummy so that ledSliders are initialised
		gOutMode.fill(kOutModeManualSample);
		stateSuccess = false;
		analogFailed = false;
		state = kNumStates;
		nextState();
		return true;
	}
	void render(BelaContext* context, FrameData* frameData) override
	{
		if(!gAlt)
			np.clear();
		if(performanceBtn.offset)
		{
			// by checking this early on, we make sure
			// gPerformanceMode doesn't grab our button presses
			performanceBtn.offset = false;
			performanceBtn.tripleClick = false;
			if(kNumStates == state)
			{
				if(++finalButtonCount >= 2)
					nextState();
			} else
				nextState();
		}
		if(analogFailed)
			tri.buttonLedSet(TRI::kSolid, TRI::kR, 1);
		if(stateSuccess)
		{
			// display a green LED and wait for button press to start next test
			tri.buttonLedSet(TRI::kSolid, TRI::kG, 1);
		} else {
			switch(state)
			{
			case kStateButton:
			{
				bool value = countMs < 300;
				if(countMs > 600)
					countMs = 0;
				tri.buttonLedSet(TRI::kSolid, value ? TRI::kG : TRI::kR);
				if(!gAlt)
				{
					rgb_t color = value ? kRgbRed.scaledBy(0.1) : kRgbOrange.scaledBy(0.1);
					for(size_t n = 0; n < np.getNumPixels(); ++n)
						np.setPixelColor(n, color.r, color.g, color.b);
				}
				// this requires visual testing, so here we assume it is successful
				testSuccessful[state] = true;
			}
				break;
			case kStateSliderLeds:
			{
				size_t n = stateSliderLedCount % kNumLeds;
				if(!gAlt)
					np.setPixelColor(n, 190, 190, 190);
				if(countMs > 300)
				{
					stateSliderLedCount++;
					countMs = 0;
				}
				if(stateSliderLedCount >= kNumLeds)
					stateSliderLedCount = 0;
				// this requires visual testing, so here we assume it is successful
				testSuccessful[state] = true;
			}
				break;
			case kStatePads:
			{
				extern std::vector<unsigned int> padsToOrderMap;
				bool success = true; // tentative
				std::array<bool,kNumLeds> allGood;
				allGood.fill(true);
				const std::vector<float>& rawData = trill.rawData;
				assert(padsToOrderMap.size() == kNumPads);
				assert(padStates.size() == kNumPads);
				for(size_t n = 0; n < kNumPads; ++n)
				{
					PadState& t = padStates[n];
					size_t orderIdx = gJacksOnTop ? padsToOrderMap.size() - 1 - n : n;
					size_t pad = padsToOrderMap[orderIdx];
					if(pad >= rawData.size())
						continue;
					const float v = rawData[pad];
					size_t led = kNumLeds * n / float(kNumPads);
					bool good = false;
					switch(t)
					{
					case kPadStateInitial:
						if(0 == v)
							t = kPadStateHasHadZero;
						break;
					case kPadStateHasHadZero:
						if(v > 0.1)
							t = kPadStateHasHadValue;
						break;
					case kPadStateHasHadValue:
						// nothing to do
						good = true;
						break;
					}
					if(!good)
					{
						success = false;
						allGood[led] = false;
					}
				}
				// after we set all allGood, we write them to the leds
				if(!gAlt)
				{
					for(size_t n = 0; n < np.getNumPixels(); ++n)
						np.setPixelColor(n, (allGood[n] ? kRgbBlack : kRgbYellow).scaledBy(0.2));
				}
				if(success)
					stateSuccess = true;
			}
				break;
			case kStateAnalog:
				if(gCalibrationProcedure.done())
				{
					auto& in = getCalibrationInput().values;
					auto& out = getCalibrationInput().values;
					// calibration is done, check if its values make sense
					if(in[0] > 0 && in[0] < 0.1
						&& in[2] > 0.9 && in[2] < 4095.f/4096.f
						&& out[0] > 0 && out[0] < 0.1
						&& out[2] > 0.9 && out[2] < 4095.f/4096.f
					)
					{
						stateSuccess = true;
						analogFailed = false;
					} else
						analogFailed = true;
				} else // process until calibration is done
					gCalibrationMode.render(context, frameData);
				break;
			case kNumStates:
				bool success = true;
				for(auto& t : testSuccessful)
				{
					if(!t)
						success = false;
				}
				if(!gAlt)
				{
					rgb_t color;
					if(success)
						color = kRgbGreen;
					else
						color = kRgbRed;
					for(size_t n = 0; n < np.getNumPixels(); ++n)
						np.setPixelColor(n, color.scaledBy(0.2));
				}
			}
		}
		if(kStateAnalog == state && !gCalibrationProcedure.done())
		{
			// global settings set by gCalibrationMode.
			// TODO: verify this conditional is not actually needed (i.e.: same results with or without)
		} else {
			gOutMode.fill(kOutModeManualSample);
			gInUsesCalibration = false;
			gInUsesRange = false;
			gOutUsesCalibration = false;
			gOutUsesRange = { false, false };
		}
		countMs += context->analogFrames / context->analogSampleRate * 1000;
	}

	void updatePreset() override
	{}
private:
	void nextState() {
		if(stateSuccess)
			testSuccessful[state] = true;
		stateSuccess = false;
		int newState = (int)this->state + 1;
		if(newState > kNumStates)
			newState = 0;
		countMs = 0;
		this->state = State(newState);
		initState();
	}
	void initState()
	{
		// update global states
		switch(state)
		{
		case kStateButton:
			testSuccessful.fill(false);
			break;
		case kStateSliderLeds:
			stateSliderLedCount = 0;
			break;
		case kStatePads:
			padStates.fill(kPadStateInitial);
			break;
		case kStateAnalog:
			gCalibrationMode.setup(-1);
			gCalibrationProcedure.start();
			break;
		case kNumStates:
			finalButtonCount = 0;
			break;
		}
		countMs = 0;
	}
	double countMs = 0;
	enum State {
		kStateButton,
		kStateSliderLeds,
		kStatePads,
		kStateAnalog,
		kNumStates,
	};
	State state = kStateButton;
	size_t stateSliderLedCount;
	enum PadState {
		kPadStateInitial,
		kPadStateHasHadZero,
		kPadStateHasHadValue,
	};
	std::array<PadState,kNumPads> padStates;
	std::array<bool,kNumStates> testSuccessful;
	size_t finalButtonCount;
	bool stateSuccess = false;
	bool analogFailed;
} gFactoryTestMode;

static void requestNewMode(int mode);
extern const ssize_t kFactoryTestModeIdx;

class EraseSettingsMode: public PerformanceMode {
public:
	bool setup(double ms) override
	{
		ledSlidersSetupOneSlider(kRgbBlack, LedSlider::MANUAL_DIRECT); // dummy so that ledSliders are initialised
		changeState(kStateWaitingForConfirmation);
		return true;
	}
	void render(BelaContext* context, FrameData* frameData) override
	{
		if(gAlt)
			return;
		np.clear();
		uint32_t ms = HAL_GetTick() - startMs;
		auto halves = getHalves();
		std::array<rgb_t,2> colors { kRgbRed, kRgbYellow };
		size_t halfSize = np.getNumPixels() / 2;
		switch(state)
		{
		case kStateWaitingForConfirmation:
		{
			constexpr size_t kPeriod = 1000;
			bool half = ms % kPeriod > kPeriod / 2;
			for(size_t h = 0; h < halves.size(); ++h)
			{
				size_t start = 0 + h * halfSize;
				size_t stop = halfSize * (1 + h);
				for(size_t n = start; n < stop; ++n)
				{
					rgb_t color = colors[h];
					if(halves[h] && (n == ((start + stop) / 2) || n == ((start + stop) / 2 + 1) ||  n == ((start + stop) / 2 + 2)))
						color = kRgbGreen;
					np.setPixelColor(n, color.scaledBy(0.3 * (half == h || halves[h])));
				}
			}

			// TODO: show current centroid in green
			if(halves[0] && halves[1])
				changeState(kStateConfirmationInProgress);
			if(ms > 10000)
				changeState(kStateAbort);
		}
			break;
		case kStateConfirmationInProgress:
		{
			constexpr size_t kDuration = 3000;
			float idx = ms / float(kDuration);
			size_t start = halfSize - idx * halfSize;
			size_t stop =  halfSize + idx * halfSize;
			for(size_t n = start; n < stop && n < np.getNumPixels(); ++n)
				np.setPixelColor(n, kRgbGreen.scaledBy(0.2));
			if(!halves[0] || !halves[1])
				changeState(kStateWaitingForConfirmation);
			if(ms > kDuration)
				changeState(kStateDisplayConfirmation);
		}
			break;
		case kStateDisplayConfirmation:
			for(size_t n = 0; n < np.getNumPixels(); ++n)
				np.setPixelColor(n, kRgbGreen.scaledBy(0.2));
			// ensure we run a couple of times before we erase flash, so display is shown
			if(ms > 100)
			{
				printf("ERASE SETTINGS\n\r");
				presetEraseAll();
				bootloaderResetTo(kBootloaderMagicUserBootloader);
			}
			break;
		case kStateAbort:
			for(size_t n = 0; n < np.getNumPixels(); ++n)
				np.setPixelColor(n, kRgbRed.scaledBy(0.2));
			if(ms > 3000)
				requestNewMode(0);
			break;
		}
	}
	std::array<uint8_t,2> getHalves()
	{
		std::array<uint8_t,2> halves {};
		for(size_t n = 0; n < globalSlider.getNumTouches(); ++n)
		{
			float loc = globalSlider.touchLocation(n);
			halves[loc > 0.5]++;
		}
		for(size_t h = 0; h < halves.size(); ++h)
		{
			if(halves[h] != 1) // more than one finger: bad
				halves[h] = 0;
		}
		return halves;
	}
	void updatePreset() override
	{};
	enum State {
		kStateWaitingForConfirmation,
		kStateConfirmationInProgress,
		kStateDisplayConfirmation,
		kStateAbort,
	};
	void changeState(State state)
	{
		this->state = state;
		startMs = HAL_GetTick();
	}
	State state;
	uint32_t startMs;
} gEraseSettingsMode;

static std::array<PerformanceMode*,kNumModes> performanceModes = {
#ifdef TEST_MODE
	&gTestMode,
#endif // TEST_MODE
#ifdef ENABLE_DIRECT_CONTROL_MODE
	&gDirectControlMode,
#endif // ENABLE_DIRECT_CONTROL_MODE
#ifdef ENABLE_RECORDER_MODE
	&gRecorderMode,
#endif // RECORDER_MODE
#ifdef ENABLE_SCALE_METER_MODE
	&gScaleMeterMode,
#endif // ENABLE_SCALE_METER_MODE
#ifdef ENABLE_BALANCED_OSCS_MODE
	&gBalancedOscsMode,
#endif // ENABLE_BALANCED_OSCS_MODE
#ifdef ENABLE_EXPR_BUTTONS_MODE
	&gExprButtonsMode,
#endif // ENABLE_EXPR_BUTTONS_MODE
	&gCalibrationMode,
	&gFactoryTestMode,
	&gEraseSettingsMode,
};

static size_t findModeIdx(const PerformanceMode& mode)
{
	auto it = std::find(performanceModes.begin(), performanceModes.end(), &mode);
	int idx = -1;
	if(it != performanceModes.end())
		idx = it - performanceModes.begin();
	assert(idx >= 0);
	// while we are at it, a sanity check that all modes have been init'ed
	for(auto& m : performanceModes)
		assert(m);
	return idx;
}
static const ssize_t kCalibrationModeIdx = findModeIdx(gCalibrationMode);
const ssize_t kFactoryTestModeIdx = findModeIdx(gFactoryTestMode);
static const ssize_t kEraseSettingsModeIdx = findModeIdx(gEraseSettingsMode);
uint8_t gNewMode = kFactoryTestModeIdx; // if there is a preset to load (i.e.: always except on first boot), this will be overridden then.

bool performanceMode_setup(double ms)
{
	tri.buttonLedSet(TRI::kOff, TRI::kAll); // for good measure. TODO: this assumes the mode's setup() doesn't need the button leds
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
	gOutAddsIn = false;
	gModeWantsInteractionPreMenu = false;
	// call the processing callback
	if(gNewMode < kNumModes && performanceModes[gNewMode])
	{
		performanceModes[gNewMode]->render(context, frameData);
		IoRanges ioRanges = performanceModes[gNewMode]->ioRangesParameters;
		gOutRangeTop = ioRanges.outTop;
		gOutRangeBottom = ioRanges.outBottom;
		gInRange = ioRanges.in;
	}
	// make the final states visible to the wrapper
	gOutRangeTop.enabled = gOutUsesRange[0];
	gOutRangeBottom.enabled = gOutUsesRange[1];
	// note: this will only take effect from the next time this function is called,
	// because obviously input range processing has already been done  by time this
	// function is called. This is not an issue normally, as long as the processing
	// callbacks know about that.
	gInRange.enabled = gInUsesRange;
}

constexpr size_t kMaxBtnStates = 6;
typedef const std::array<rgb_t,kMaxBtnStates> AnimationColors;
static AnimationColors buttonColors = {
		kRgbRed,
		kRgbOrange,
		kRgbYellow,
		kRgbGreen,
		kRgbWhite,
		kRgbBlack, // dummy
};

static bool pulses(uint32_t ms, uint32_t period, size_t numPulses)
{
	size_t blankPeriods = 2;
	size_t periods = blankPeriods + numPulses;
	size_t totalMs = periods * period;
	float ramp = simpleRamp(ms, totalMs);
	float step = 1.f / periods;
	float on = 0.5f * step;
	for(size_t n = 0; n < numPulses; ++n)
	{
		float start = n * step;
		float stop = start + on;
		if(ramp >= start && ramp < stop)
			return true;
	}
	return false;
}

class ButtonAnimation {
public:
	void process(uint32_t ms, LedSlider& ledSlider, float value)
	{
		constexpr unsigned int kPeriod = 1000;
		constexpr unsigned int kPulsesPeriod = 400;
		rgb_t color = buttonColors[getIdx(value)];
		switch(gAnimationMode)
		{
		case kNumAnimationMode:
		case kAnimationModeConsistent:
		case kAnimationModeConsistentWithFs:
		{
			float coeff;
			float tri = simpleTriangle(ms, kPeriod);
			switch(int(value))
			{
			default:
			case 0: // solid
				coeff = 1;
				break;
			case 1: // "sine"
				coeff = tri;
				break;
			case 2: // on/off blink
				coeff = tri < 0.5;
				break;
			case 3: // three pulses
				coeff = pulses(ms, kPulsesPeriod, 3);
				break;
			case 4: // ramp
				coeff = simpleRamp(ms, kPeriod);
				break;
			}
			color.scale(coeff);
			ledSlider.setColor(color);
		}
			break;
		case kAnimationModeSolid:
		case kAnimationModeSolidWithFs:
			ledSlider.setColor(color);
			break;
		case kAnimationModeSolidDefaultWithFs:
			if(0 != value)
				color.scale(0.1f + 0.9f * simpleTriangle(ms, 700));
			ledSlider.setColor(color);
			break;
		case kAnimationModeCustom:
			processCustom(ms, ledSlider, value);
			break;
		case kAnimationModeSolidGlowingWithFs:
			if(ms < kPostAnimationGlowingMs)
				color.scale(0.1f + 0.9f * simpleTriangle(ms + kPostAnimationGlowingPeriod / 2, kPostAnimationGlowingPeriod));
			ledSlider.setColor(color);
		}
	};
	virtual void processCustom(uint32_t ms, LedSlider& ledSlider, float value) {};
protected:
	size_t getIdx(size_t value)
	{
		return std::min(value, kMaxBtnStates - 1);
	}
};

class ButtonAnimationSplit : public ButtonAnimation {
public:
	ButtonAnimationSplit(AnimationColors& colors) :
		colors(colors) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float value) override {
		float tri = simpleTriangle(ms, 1000);
		float coeff;
		switch(int(value))
		{
		default:
		case SplitPerformanceMode::kModeNoSplit:
			coeff = 1.f;
		break;
		case SplitPerformanceMode::kModeSplitLocation:
			coeff = tri > 0.5 ? 1 : 0;
		break;
		case SplitPerformanceMode::kModeSplitSize:
			coeff = tri;
		break;
		}
		rgb_t color = colors[getIdx(value)];
		color.scale(coeff);
		ledSlider.setColor(color);
	};
protected:
	AnimationColors& colors;
};

class ButtonAnimationPulsatingStill : public ButtonAnimation {
public:
	ButtonAnimationPulsatingStill(AnimationColors& colors) :
		colors(colors) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float value) override {
		const rgb_t& color = colors[getIdx(value)];
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
	AnimationColors& colors;
	unsigned int width = -1;
};

class ButtonAnimationStillTriangle : public ButtonAnimation {
public:
	ButtonAnimationStillTriangle(AnimationColors& colors) :
		colors(colors) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float value) override {
		const rgb_t& color = colors[getIdx(value)];
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
	AnimationColors& colors;
};

class ButtonAnimationTriangle : public ButtonAnimation {
public:
	ButtonAnimationTriangle(rgb_t color, uint32_t period) :
		color(color), period(period) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float) override {
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

class ButtonAnimationCounter : public ButtonAnimation {
public:
	ButtonAnimationCounter(AnimationColors& colors, uint32_t sshort, uint32_t llong) :
		colors(colors), sshort(sshort), llong(llong), counter(0), lastTime(0) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float value) override {
		rgb_t color = colors[getIdx(value)];
		size_t blinks = 5 - value;
		uint32_t time = HAL_GetTick();
		uint32_t period;
		float coeff = 0;
		if(counter < blinks)
		{
			period = sshort;
			if(time - lastTime < period / 3)
				coeff = 1;
		}
		else
			period = llong;
		if(time - lastTime > period)
		{
			counter++;
			if(counter > blinks)
				counter = 0;
			lastTime = time;
		}
		for(size_t n = 0; n < color.size(); ++n)
			color[n] *= coeff;
		ledSlider.setColor(color);
	};
protected:
	AnimationColors& colors;
	uint32_t sshort;
	uint32_t llong;
	ssize_t offset;
	size_t counter;
	uint32_t lastTime;
};

class ButtonAnimationSolid: public ButtonAnimation {
public:
	ButtonAnimationSolid(AnimationColors& colors) :
		colors(colors) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float value) override {
		const rgb_t& color = colors[getIdx(value)];
		ledSlider.setColor(color);
	};
protected:
	AnimationColors& colors;
};

class ButtonAnimationBrightDimmed: public ButtonAnimation {
public:
	ButtonAnimationBrightDimmed(rgb_t color) :
		color(color) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float) override {
		ledSlider.setColor(color);
	};
protected:
	rgb_t color;
};

class ButtonAnimationSingleRepeatedEnv: public ButtonAnimation {
public:
	ButtonAnimationSingleRepeatedEnv(AnimationColors& colors) :
		colors(colors) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float value) override {
		const rgb_t& color = colors[getIdx(value)];
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
	AnimationColors& colors;
};

#ifdef ENABLE_RECORDER_MODE
class ButtonAnimationRecorderInputMode: public ButtonAnimation {
public:
	ButtonAnimationRecorderInputMode(AnimationColors& colors) :
		colors(colors) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float value) override {
		rgb_t color = colors[getIdx(value)];
		float coeff = 0;
		switch(RecorderMode::InputMode(value)) {
		case RecorderMode::kInputModeNum: // default
		case RecorderMode::kInputModeLfo:
		case RecorderMode::kInputModeEnvelope:
		{
			const unsigned int duration = 600;
			unsigned int period;
			if(RecorderMode::kInputModeLfo ==  value)
			{
				// largely spaced animation
				period = duration * 2.5;
			} else {
				// tightly looped animations
				period = duration;
			}
			ms %= period;
			if(ms <= duration)
				coeff = mapAndConstrain((duration - ms) / float(duration), 0, 1, 0.3, 1);
			else
				coeff = 0;
		}
		break;
		case RecorderMode::kInputModeClock:
		{
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
		}
		break;
		case RecorderMode::kInputModeCv:
		{
			// input mode: CV in
			// show a few smooth transitions
			const unsigned int duration = 3000;
			ms %= duration;
			std::array<float,12> data = {
					0.1, 0.3, 0.5, 0.3, 0.4, 0.7, 1.0, 0.8, 0.6, 0.3, 0.3, 0.3
			};
			coeff = interpolatedRead(data, ms / float(duration));
		}
		break;
		case RecorderMode::kInputModePhasor:
		{
			// input mode: phasor
			// show a phasor
			const unsigned int duration = 1000;
			ms %= duration;
			coeff = ms / float(duration);
		}
		break;
		}
		rgb_t c;
		c.r = color.r * coeff;
		c.g = color.g * coeff;
		c.b = color.b * coeff;
		ledSlider.setColor(c);
	};
protected:
	AnimationColors& colors;
	static constexpr float initialPeriod = 600;
	static constexpr float finalPeriod = 80;
	static constexpr unsigned int duration = 3000;
	static constexpr float onTime = 40;
	float phase = 0;
	uint32_t lastMs = 0;
};
#endif // ENABLE_RECORDER_MODE

class ButtonAnimationWaveform: public ButtonAnimation {
public:
	ButtonAnimationWaveform(AnimationColors& colors) :
		colors(colors) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float value) override {
		const rgb_t& color = colors[getIdx(value)];
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
	AnimationColors& colors;
	Oscillator osc {1};
};

class ButtonAnimationSpeedUpDown: public ButtonAnimation {
public:
	ButtonAnimationSpeedUpDown(AnimationColors& colors) :
		colors(colors) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float value) override {
		const rgb_t& color = colors[getIdx(value)];
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
	AnimationColors& colors;
};

class ButtonAnimationSmoothQuantised : public ButtonAnimation {
public:
	ButtonAnimationSmoothQuantised(AnimationColors& colors) :
		colors(colors) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float value) override {
		const rgb_t& color = colors[getIdx(value)];
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
	AnimationColors& colors;
};

class ButtonAnimationKeysSeq : public ButtonAnimation {
public:
	ButtonAnimationKeysSeq(AnimationColors& colors) :
		colors(colors) {}
	void processCustom(uint32_t ms, LedSlider& ledSlider, float value) override {
		const rgb_t& color = colors[getIdx(value)];
		const unsigned int period = 1500;
		ms %= period / 2;
		float saw = simpleTriangle(ms + period / 2, period);
		float coeff;
		if(0 == value) // keys
		{
			// blink at irregular intervals
			coeff = saw > 0.9 || (saw < 0.8 && saw > 0.7) || (saw < 0.6 && saw > 0.5);
		} else { // seq
			// blink at constant intervals (like a sequencer would)
			coeff = saw > 0.6;
		}
		rgb_t c;
		c.r = color.r * coeff;
		c.g = color.g * coeff;
		c.b = color.b * coeff;
		ledSlider.setColor(c);
	};
protected:
	AnimationColors& colors;
};

class MenuItemType
{
public:
	MenuItemType(rgb_t baseColor) : baseColor(baseColor) {}
	virtual void process(LedSlider& slider) = 0;
	virtual void resetState() {};
	rgb_t baseColor;
};

class MenuItemTypeEvent : public MenuItemType
{
public:
	MenuItemTypeEvent(const char* name, rgb_t baseColor, uint32_t holdTime = 0) :
		MenuItemType(baseColor), name(name), holdTime(holdTime) {}
	virtual void process(LedSlider& slider) override
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
			centroid.size = state ? kMenuButtonActiveBrightness : kMenuButtonDefaultBrightness;
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

class MenuItemTypeDiscreteHold : public MenuItemTypeEvent
{
public:
	MenuItemTypeDiscreteHold(const char* name, rgb_t baseColor, uint32_t holdTime, ParameterEnum& valueEn) :
		MenuItemTypeEvent(name, baseColor, holdTime), valueEn(valueEn)
	{}
	void process(LedSlider& slider) override
	{
		MenuItemTypeEvent::process(slider);
		rgb_t color = baseColor;
		// if not 0, glow
		if(valueEn.get())
			color.scale(simpleTriangle(HAL_GetTick(), 1000) * 0.9f + 0.1f);
		slider.setColor(color);
	}
	void event(Event e) override
	{
		if(kHoldHigh == e)
		{
			valueEn.next();
		}
	}
	ParameterEnum& valueEn;
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
				M(printf("%s: set to %d\n\r", name, parameter->get()));
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
	MenuItemTypeSlider(const rgb_t& color, const rgb_t& otherColor, ParameterContinuous* parameter) :
		MenuItemType(color), otherColor(otherColor), parameter(parameter) {
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
			if(!tracking && frame.size)
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
			rgb_t color;
			if(parameter->isDefault(kDefaultThreshold)) {
				color = baseColor;
				if(tracking) // make the centroid bigger as we approach the default
					centroid.size = map(std::abs(parameter->getDefault() - parameter->get()), kDefaultThreshold, 0, centroid.size, 0.9);
			} else {
				color = otherColor;
			}
			if(tracking) {
				// only track the slider if we have at some point crossed the initial point
				parameter->set(frame.location);
			} else {
				// or show a pulsating centroid
				centroid.size *= simpleTriangle(HAL_GetTick() - initialTime, 130);
			}
			ledSlidersAlt.sliders[0].setColor(color);
			ledSlidersAlt.sliders[0].setLedsCentroids(&centroid, 1);

			if(latched)
				menu_up();
		}
	}
	rgb_t otherColor;
	ParameterContinuous* parameter;
	//below are init'd to avoid warning
	float initialPos = 0;
	uint32_t initialTime = 0;
	bool tracking = false;
	bool hasDoneSetup = false;
	bool hasHadTouch = false;
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
			centroid_t frame = {
				.location = slider.compoundTouchLocation(),
				.size = slider.compoundTouchSize(),
			};
			bool latched = false;
			gMenuAutoLatcher.process(frame, latched);
			float pos = fix(frame.location);
			unsigned int n = (unsigned int)(pos * quantised);
			if(n >= quantised) // you may reach the top when pos is exactly 1 (or more...)
				n--;
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
		latchProcessor.reset();
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
		else {
			// at least ensure they are sorted
			std::array<float,kNumEnds> ret = in;
			std::sort(ret.begin(), ret.end());
			return ret;
		}
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
#ifdef ENABLE_SCALE_METER_MODE
		std::array<centroid_t,2> endpointsCentroids {
			centroid_t{ pastFrames[0].location, 0.1 },
			centroid_t{ pastFrames[1].location, 0.1 },
		};
		slider.directBegin();
		for(size_t n = 0; n < endpointsColor.size(); ++n)
			slider.directWriteCentroid(endpointsCentroids[n], endpointsColor[n], ScaleMeterMode::kCentroidSize);
		slider.directWriteCentroid({ *display, 0.15 }, displayColor, ScaleMeterMode::kCentroidSize);
		if(display == &gScaleMeterMode.inDisplay)
			gScaleMeterMode.inDisplayUpdated = 10;
#endif // ENABLE_SCALE_METER_MODE
	}
private:
	rgb_t displayColor;
	std::array<rgb_t,kNumEnds> endpointsColor; // TODO: delegate drawing endpoints to MenuItemTypeRange::updateDisplay() and avoid caching these colors here
	const float* display;
};

static void requestNewMode(int mode);

static void requestIncMode()
{
	int newMode = gNewMode;
	 // special modes are skipped when incrementing
	do
		newMode = (newMode + 1) % kNumModes;
	while(kCalibrationModeIdx == newMode || kFactoryTestModeIdx == newMode || kEraseSettingsModeIdx == newMode);
	requestNewMode(newMode);
}

class MenuItemTypeNextMode : public MenuItemTypeEvent
{
public:
	MenuItemTypeNextMode(const char* name, rgb_t baseColor) :
		MenuItemTypeEvent(name, baseColor, 3000) {}
private:
	void process(LedSlider& ledSlider) override
	{
		MenuItemTypeEvent::process(ledSlider);
		PerformanceMode* mode = nullptr;
		if(gNewMode < performanceModes.size() && (mode = performanceModes[gNewMode]))
			ledSlider.setColor(mode->buttonColor);
	}
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

static void menu_in(MenuPage& menu);
static MenuPage* activeMenu;

class MenuItemTypeEnterSubmenu : public MenuItemTypeEvent
{
public:
	MenuItemTypeEnterSubmenu(const char* name, rgb_t baseColor, uint32_t holdTime, MenuPage& submenu) :
		MenuItemTypeEvent(name, baseColor, holdTime), submenu(&submenu) {}
	void process(LedSlider& slider) override
	{
		MenuItemTypeEvent::process(slider);
		if(!submenu) {
			// dim inactive button, counteracting what may
			// have occurred in MenuItemTypeEvent::process() above
			centroid_t centroid = {
					.location = 0.5,
					.size = kMenuButtonDefaultBrightness * 0.3,
			};
			slider.setLedsCentroids(&centroid, 1);
		}
	}
private:
	void event(Event e)
	{
		if(kHoldHigh == e && submenu) {
			menu_in(*submenu);
		}
	}
public:
	MenuPage* submenu;
};

MenuPage mainMenu("main");
MenuPage globalSettingsMenu0("global settings 0");

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

static void buttonBlinkResetToDefault()
{
	tri.buttonLedSet(TRI::kSolid, TRI::kG, 300);
}

static constexpr size_t kHoldResetToDefault = 2000;
// On tap, get into singleSliderMenu to set value; on hold-press reset to default value
class MenuItemTypeEnterContinuous : public MenuItemTypeEvent
{
public:
	MenuItemTypeEnterContinuous(const char* name, rgb_t baseColor, rgb_t otherColor, ParameterContinuous& value, ButtonAnimation* animation = nullptr) :
		MenuItemTypeEvent(name, baseColor, kHoldResetToDefault), otherColor(otherColor), value(value), animation(animation), ignoreNextFalling(false) {}
	void process(LedSlider& slider)
	{
		MenuItemTypeEvent::process(slider);
		if(animation)
			animation->process(HAL_GetTick(), slider, value.isDefault(kDefaultThreshold) ? 0 : 1);
	}
	void event(Event e)
	{
		if(kHoldHigh == e) {
			value.resetToDefault();
			ignoreNextFalling = true;
			buttonBlinkResetToDefault();
		}
		if(kTransitionFalling == e) {
			if(ignoreNextFalling)
				ignoreNextFalling = false;
			else {
				singleSliderMenuItem = MenuItemTypeSlider(baseColor, otherColor, &value);
				menu_in(singleSliderMenu);
			}
		}
	}
	rgb_t otherColor;
	ParameterContinuous& value;
	ButtonAnimation* animation;
	bool ignoreNextFalling;
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
	MenuItemTypeDiscretePlus(const char* name, rgb_t baseColor, ParameterEnum& valueEn, bool alwaysDisplayOnFirstTap, uint32_t displayOldValueTimeout = 0):
		MenuItemTypeEvent(name, baseColor, kHoldResetToDefault), valueEn(valueEn), displayOldValueTimeout(displayOldValueTimeout), alwaysDisplayOnFirstTap(alwaysDisplayOnFirstTap) {}
	void event(Event e) override
	{
		switch (e)
		{
		case kTransitionFalling:
			if(!ignoreNextTransition)
			{
				// this one is on release so we avoid a spurious trigger when holding
				bool shouldUpdate = true;
				if(displayOldValueTimeout && alwaysDisplayOnFirstTap)
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
					M(printf("DiscretePlus: next to %d\n\r", valueEn.get()));
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
	virtual void resetState() override
	{
		lastTick = 0;
	}
	virtual void enterPlus() {
		valueEn.set(0); // reset to default
		buttonBlinkResetToDefault();
	};
protected:
	ParameterEnum& valueEn;
	uint32_t lastTick = 0;
	uint32_t displayOldValueTimeout;
	bool ignoreNextTransition = false;
	bool alwaysDisplayOnFirstTap;
};

static void menu_resetStates(const MenuItemType* src);

static MenuItemTypeDiscretePlus* gAnimationIsPlaying = nullptr;
static uint32_t gAnimationPlayedLast = 0;
// displays an animation every kTransitionFalling event. It leverages
// the fact that MenuItemTypeDiscretePlus displays current value upon first tap.
class MenuItemTypeDiscreteFullScreenAnimation : public MenuItemTypeDiscretePlus
{
public:
	MenuItemTypeDiscreteFullScreenAnimation(const char* name, const AnimationColors& colors, ParameterEnum& valueEn, bool alwaysDisplayOnFirstTap, ButtonAnimation* animation = nullptr) :
		MenuItemTypeDiscretePlus(name, colors[getIdx(valueEn.get())], valueEn, alwaysDisplayOnFirstTap, kPostAnimationTimeoutMs),
		colors(colors), lastTap(0), buttonAnimation(animation)
	{}
	virtual void process(LedSlider& ledSlider) override
	{
		MenuItemTypeDiscretePlus::process(ledSlider);
		uint32_t currentMs = HAL_GetTick();
		if(buttonAnimation)
			buttonAnimation->process(currentMs - lastTick, ledSlider, valueEn.get());
		uint32_t ms = currentMs - lastTap;
		rgb_t color = colors[getIdx(valueEn.get())];
		// TODO: it's a bit awkward to be calling animate() unconditionally,
		// assuming ms will be enough to tell them whether to draw something or not
		// and that a menu-wide reset will reset ms ...
		uint32_t lastCount = gAnimateFs.hasWrittenCount();
		valueEn.animate(ledSlidersAlt.sliders[0], color, ms);
		bool hasAnimated = (gAnimateFs.hasWrittenCount() != lastCount);
		if(hasAnimated)
		{
			gAnimationIsPlaying = this;
			gAnimationPlayedLast = HAL_GetTick();
			// displayOldValueTimeout  starts counting
			// from the end of the animation
			lastTick = currentMs;
		}
	}
	void event(Event e) override
	{
		MenuItemTypeDiscretePlus::event(e);
		if(kTransitionFalling == e)
		{
			// just tapped, make a note
			lastTap = HAL_GetTick();
			// exclusively enable this animation
			gAnimateFs.setActive(valueEn, animateFsAllLeds);
			// reset all other timeouts
			menu_resetStates(this);
		}
	}
	void resetState() override
	{
		MenuItemTypeDiscretePlus::resetState();
		lastTap = 0;
	}
protected:
	static size_t getIdx(size_t value)
	{
		return std::min(value, std::tuple_size<AnimationColors>::value - 1);
	}
	const AnimationColors& colors;
	uint32_t lastTap;
	ButtonAnimation* buttonAnimation;
	bool animateFsAllLeds = false;
};

class MenuItemTypeDiscreteContinuous : public MenuItemTypeDiscretePlus
{
public:
	MenuItemTypeDiscreteContinuous(const char* name, rgb_t baseColor, ParameterEnum& valueEn, ParameterContinuous& valueCon, ButtonAnimation* animation = nullptr):
		MenuItemTypeDiscretePlus(name, baseColor, valueEn, true), valueCon(valueCon), animation(animation) {}
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
		M(printf("DiscreteContinuous: going to slider\n\r"));
		// TODO: if using this again, pass a different color as otherColor
		singleSliderMenuItem = MenuItemTypeSlider(baseColor, baseColor, &valueCon);
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
		M(printf("DiscreteRange: going to range\n\r"));
		singleRangeMenuItem = MenuItemTypeRange(baseColor, otherColor, true, &valueConBottom, &valueConTop, preprocess);
		menu_in(singleRangeMenu);
	}
	ParameterContinuous& valueConBottom;
	ParameterContinuous& valueConTop;
	MenuItemTypeRange::PreprocessFn preprocess;
	rgb_t otherColor;
};

static std::array<float,MenuItemTypeRange::kNumEnds> quantiseNormalisedForIntegerVolts(const std::array<float,MenuItemTypeRange::kNumEnds>& in);
class MenuItemTypeDiscreteRangeCv : public MenuItemTypeDiscreteFullScreenAnimation
{
public:
	MenuItemTypeDiscreteRangeCv(const char* name, const AnimationColors& colors, IoRangeParameters& valueEn):
		MenuItemTypeDiscreteFullScreenAnimation(name, colors, valueEn, true), ioRangeParameters(valueEn)
	{
		animateFsAllLeds = true;
	}
	void enterPlus() override
	{
		M(printf("RangeCv: going to range\n\r"));
		singleRangeMenuItem = MenuItemTypeRange(colors[getIdx(valueEn.get())], colors.back(), true, &ioRangeParameters.min, &ioRangeParameters.max, quantiseNormalisedForIntegerVolts);
		menu_in(singleRangeMenu);
	}

private:
	IoRangeParameters& ioRangeParameters;
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
	MenuItemTypeDisabled(const rgb_t& color) : MenuItemType(color) {}
	void process(LedSlider& slider) override {}
};

constexpr size_t kMaxModeParameters = 3;
static const rgb_t buttonColor = kRgbRed;
static MenuItemTypeDisabled disabled(kRgbBlack);

static ButtonAnimationSplit animationSplit(buttonColors);
static constexpr rgb_t kSettingsSubmenuButtonColor = kRgbWhite;
#ifdef ENABLE_DIRECT_CONTROL_MODE
static ButtonAnimationPulsatingStill animationPulsatingStill(buttonColors);
static MenuItemTypeDiscreteFullScreenAnimation directControlModeSplit("directControlModeSplit", buttonColors, gDirectControlMode.splitMode, false, &animationSplit);
static MenuItemTypeDiscreteFullScreenAnimation directControlModeLatch("directControlModeAutoLatch", buttonColors, gDirectControlMode.autoLatch, false, &animationPulsatingStill);
static std::array<MenuItemType*,kMaxModeParameters> directControlModeMenu = {
		&disabled,
		&directControlModeLatch,
		&directControlModeSplit,
};
#endif // ENABLE_DIRECT_CONTROL_MODE

#ifdef ENABLE_RECORDER_MODE
//static ButtonAnimationSingleRepeatedEnv animationSingleRepeatedPulse{buttonColors};
static MenuItemTypeDiscreteFullScreenAnimation recorderModeSplit("recorderModeSplit", buttonColors, gRecorderMode.splitMode, false, &animationSplit);
//static MenuItemTypeDiscrete recorderModeRetrigger("recorderModeRetrigger", buttonColor, &gRecorderMode.autoRetrigger, &animationSingleRepeatedPulse);
static ButtonAnimationRecorderInputMode animationRecorderInputMode{buttonColors};
static MenuItemTypeDiscreteFullScreenAnimation recorderModeInputMode("recorderModeInputMode", buttonColors, gRecorderMode.inputMode, false, &animationRecorderInputMode);
static std::array<MenuItemType*,kMaxModeParameters> recorderModeMenu = {
		&disabled,
		&recorderModeInputMode,
		&recorderModeSplit,
};
#endif // ENABLE_RECORDER_MODE

#ifdef ENABLE_SCALE_METER_MODE
static ButtonAnimationStillTriangle animationSingleStillTriangle{buttonColors};
static ButtonAnimationSolid animationSolid{buttonColors};
static MenuItemTypeDiscreteFullScreenAnimation scaleMeterModeOutputMode("scaleMeterModeOutputMode", buttonColors, gScaleMeterMode.outputMode, false, &animationSolid);
static MenuItemTypeDiscreteFullScreenAnimation scaleMeterModeCoupling("scaleMeterModeCoupling", buttonColors, gScaleMeterMode.coupling, false, &animationSingleStillTriangle);
static MenuItemTypeEnterContinuous scaleMeterModeCutoff("scaleMeterModeCutoff", buttonColors[0], buttonColors[2], gScaleMeterMode.cutoff);
static std::array<MenuItemType*,kMaxModeParameters> scaleMeterModeMenu = {
		&scaleMeterModeCutoff,
		&scaleMeterModeOutputMode,
		&scaleMeterModeCoupling,
};
#endif // ENABLE_SCALE_METER_MODE

#ifdef ENABLE_BALANCED_OSCS_MODE
static ButtonAnimationWaveform animationWaveform{buttonColors};
static MenuItemTypeDiscrete balancedOscModeWaveform("balancedOscModeWaveform", buttonColor, &gBalancedOscsMode.waveform, &animationWaveform);
static ButtonAnimationRecorderInputMode animationBalancedOscsInputMode{buttonColors};
static MenuItemTypeDiscreteContinuous balancedOscModeInputModeAndFrequency("balancedOscModeInputModeAndFrequency", buttonColor,
		gBalancedOscsMode.inputMode, gBalancedOscsMode.centreFrequency, &animationBalancedOscsInputMode);
static std::array<MenuItemType*,kMaxModeParameters> balancedOscsModeMenu = {
		&disabled,
		&balancedOscModeInputModeAndFrequency,
		&balancedOscModeWaveform,
};
#endif // ENABLE_BALANCED_OSCS_MODE

#ifdef ENABLE_EXPR_BUTTONS_MODE
static ButtonAnimationSmoothQuantised animationSmoothQuantised {buttonColors};
static ButtonAnimationKeysSeq animationKeysSeq {buttonColors};
static ButtonAnimationTriangle animationTriangleExprButtonsModRange(buttonColor, 3000);
//static ButtonAnimationCounter animationCounterNumKeys {buttonColors, 300, 800};
static MenuItemTypeDiscreteFullScreenAnimation exprButtonsModeQuantised("gExprButtonsModeQuantised", buttonColors, gExprButtonsMode.quantised, false, &animationSmoothQuantised);
static MenuItemTypeDiscreteFullScreenAnimation exprButtonsModeSeqMode("gExprButtonsModeSeqMode", buttonColors, gExprButtonsMode.seqMode, false, &animationKeysSeq);
static MenuItemTypeEnterContinuous exprButtonsModeModRange("gExprButtonsModeModRange", buttonColors[0], buttonColors[2], gExprButtonsMode.modRange);
#endif // ENABLE_EXPR_BUTTONS_MODE

#ifdef ENABLE_EXPR_BUTTONS_MODE
static std::array<MenuItemType*,kMaxModeParameters> exprButtonsModeMenu = {
		&exprButtonsModeModRange,
		&exprButtonsModeQuantised,
		&exprButtonsModeSeqMode,
};
#endif // ENABLE_EXPR_BUTTONS_MODE

static std::array<MenuItemType*,kMaxModeParameters> emptyModeMenu = {
		&disabled,
		&disabled,
		&disabled,
};

static std::array<std::array<MenuItemType*,kMaxModeParameters>*,kNumModes> modesMenuItems = {
#ifdef TEST_MODE
		&emptyModeMenu, // test mode
#endif // TEST_MODE
#ifdef ENABLE_DIRECT_CONTROL_MODE
		&directControlModeMenu,
#endif // ENABLE_DIRECT_CONTROL_MODE
#ifdef ENABLE_RECORDER_MODE
		&recorderModeMenu,
#endif // ENABLE_RECORDER_MODE
#ifdef ENABLE_SCALE_METER_MODE
		&scaleMeterModeMenu,
#endif // ENABLE_SCALE_METER_MODE
#ifdef ENABLE_BALANCED_OSCS_MODE
		&balancedOscsModeMenu,
#endif // ENABLE_BALANCED_OSCS_MODE
#ifdef ENABLE_EXPR_BUTTONS_MODE
		&exprButtonsModeMenu,
#endif // ENABLE_EXPR_BUTTONS_MODE
		&emptyModeMenu, // calibration mode
};

MenuItemTypeNextMode nextMode("nextMode", kRgbGreen);

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

class GlobalSettings : public ParameterUpdateCapable {
	enum Flag {
		kFlagJacksOnTop = 1 << 0,
		kFlagAnimationsWithFs = 1 << 1,
		kFlagMax = 1 << 2,
	};
	void flagSet(Flag flag, int value)
	{
		uint8_t f = flags.get();
		if(value)
			f |= int(flag);
		else
			f &= ~int(flag);
		flags.set(f);
	}
public:
	void updated(Parameter& p)
	{
		S(bool verbose = true);
		S(char const* str = "+++");
		if(p.same(jacksOnTop)) {
			S(str = "jacksOnTop");
			flagSet(kFlagJacksOnTop, jacksOnTop);
		}
		else if(p.same(animationMode)) {
			S(str = "animationMode");
			flagSet(kFlagAnimationsWithFs, animationMode);
		}
		else if(p.same(flags)) {
			S(str = "flags");
			printf("FLAGS: 0x%x\n\r", flags.get());
			// we arrive here either because  set() has been called on the individual parameters
			// or because it has been called on flags() directly (typically by the preset loader)
			// so call set() from here only if different to avoid infinite recursion
			bool anim = (flags & kFlagAnimationsWithFs);
			if(animationMode.get() !=  anim)
				animationMode.set(anim);
			gAnimationMode = AnimationMode(anim);
			bool jacks = (flags & kFlagJacksOnTop);
			if(jacksOnTop.get() != jacks)
				jacksOnTop.set(jacks);
			gJacksOnTop = jacks;
		}
		else if(p.same(sizeScaleCoeff)) {
			S(str = "sizeScaleCoeff");
			float tmp = 0.1f + 1.5f * (powf(2, 0.5 + sizeScaleCoeff) - 1);
			if(sizeScaleCoeff > 0.98) {
				// make it very big at the top, so it works like a gate,
				// no matter how small the touch
				tmp = 10;
			}
			float coeff = kSizeScale / (tmp * tmp * tmp);
			setAllSizeScales(coeff);
			doOverride(1, globalSlider.compoundTouchSize(), false, true);
		}
		else if(p.same(brightness)) {
			S(str = "brightness");
			assert(brightness.getDefault());
			gBrightness = 0.15f + (brightness * 0.85f / brightness.getDefault());
		}
		else if(p.same(newMode)) {
			S(str = "newMode");
			requestNewMode(newMode);
		}
		S(
			if(verbose)
				printf("%s\n\r", str);
		);
	}
	void updatePreset()
	{
		UPDATE_PRESET_NOIORANGES_FIELD4(sizeScaleCoeff, brightness, flags, newMode);
	}
	GlobalSettings() :
		presetFieldData {
			.sizeScaleCoeff = sizeScaleCoeff,
			.brightness = brightness,
			.flags = flags,
			.newMode = newMode,
		}
	{
		PresetDesc_t presetDesc = {
			.field = this,
			.size = sizeof(PresetFieldData_t),
			.defaulter = genericDefaulterNoioranges4(GlobalSettings, sizeScaleCoeff, brightness, flags, newMode),
			// currently the {out,in}RangeEnums have to go after the corresponding
			// corresponding Range{Bottom,Top}, as setting the Range last would otherwise
			// reset the enum
			// TODO: make this more future-proof
			.loadCallback = genericLoadCallbackNoioranges4(GlobalSettings, sizeScaleCoeff, brightness, flags, newMode),
		};
		presetDescSet(5, &presetDesc);
	}
	ParameterContinuous sizeScaleCoeff {this, 0.5};
	ParameterEnumT<2> jacksOnTop {this, true};
	ParameterEnumT<kNumAnimationMode> animationMode {this, gAnimationMode};
	ParameterContinuous brightness {this, 0.35};
	ParameterEnumT<kNumModes> newMode{this, gNewMode};
	ParameterEnumT<kFlagMax> flags {this, kFlagJacksOnTop};
	PACKED_STRUCT(PresetFieldData_t {
		float sizeScaleCoeff;
		float brightness;
		uint8_t flags;
		uint8_t newMode;
	}) presetFieldData;
} gGlobalSettings;

static void menu_updateSubmenu();
static void requestNewMode(int mode)
{
	bool different = (gNewMode != mode);
	gNewMode = mode;
	// notify the setting that is stored to disk (unless calibration),
	// but avoid the set() to trigger a circular call to requestNewMode()
	if(different && mode != kCalibrationModeIdx)
		gGlobalSettings.newMode.set(mode);
	menu_updateSubmenu();
}

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

static constexpr rgb_t globalSettingsColor = kRgbOrange;
static constexpr rgb_t globalSettingsContinuousOtherColor = kRgbYellow;
static ButtonAnimationTriangle animationTriangleGlobal(globalSettingsColor, 3000);
static MenuItemTypeEnterContinuous globalSettingsSizeScale("globalSettingsSizeScale", globalSettingsColor, globalSettingsContinuousOtherColor, gGlobalSettings.sizeScaleCoeff);
static constexpr rgb_t jacksOnTopButtonColor = kRgbRed;
static ButtonAnimationBrightDimmed animationBrightDimmed(jacksOnTopButtonColor);
static MenuItemTypeEnterQuantised globalSettingsJacksOnTop("globalSettingsJacksOnTop", jacksOnTopButtonColor, gGlobalSettings.jacksOnTop);
static MenuItemTypeDiscreteHold globalSettingsAnimationMode("globalSettingsAnimationMode", globalSettingsColor, 3000, gGlobalSettings.animationMode);

static MenuItemTypeEnterContinuous globalSettingsBrightness("globalSettingsBrightness", globalSettingsColor, globalSettingsContinuousOtherColor, gGlobalSettings.brightness);

static constexpr rgb_t kIoRangeButtonColor = kRgbYellow;
static constexpr rgb_t kIoRangeOtherColor = kRgbRed;
static MenuItemTypeDisabled disabledIoRange(kIoRangeButtonColor.scaledBy(0.2));
static constexpr AnimationColors ioRangeColors {
	kIoRangeButtonColor, kIoRangeButtonColor, kIoRangeButtonColor, kIoRangeButtonColor, kIoRangeButtonColor, kIoRangeOtherColor
};
class PerformanceModeIoRangesMenuPage : public MenuPage {
public:
	PerformanceModeIoRangesMenuPage(const char* name, PerformanceMode& perf, bool inputEnabled) :
		MenuPage(name, {
				&disabled,
				&disabled,
				&outBottom,
				&outTop,
				inputEnabled ? (MenuItemType*)&in : &disabledIoRange,
		}),
		in(
				(std::string(name) + " in").c_str(),
				ioRangeColors,
				perf.ioRangesParameters.in),
		outTop(
				(std::string(name) + " outTop").c_str(),
				ioRangeColors,
				perf.ioRangesParameters.outTop),
		outBottom(
				(std::string(name) + " outBottom").c_str(),
				ioRangeColors,
				perf.ioRangesParameters.outBottom)

	{}
private:
	MenuItemTypeDiscreteRangeCv in;
	MenuItemTypeDiscreteRangeCv outTop;
	MenuItemTypeDiscreteRangeCv outBottom;
};

// _why_ does it have to be so verbose? For some (good???) reason we deleted the copy constructor of MenuPage,
// so we cannot put MenuPage objects in an array and we need this hack
#ifdef ENABLE_DIRECT_CONTROL_MODE
static PerformanceModeIoRangesMenuPage menuPageDirectControl {"direct control", gDirectControlMode, false};
#endif // ENABLE_DIRECT_CONTROL_MODE
#ifdef ENABLE_RECORDER_MODE
static PerformanceModeIoRangesMenuPage menuPageRecorder {"recorder", gRecorderMode, true};
#endif // ENABLE_RECORDER_MODE
#ifdef ENABLE_SCALE_METER_MODE
static PerformanceModeIoRangesMenuPage menuPageScaleMeter {"scale/meter", gScaleMeterMode, true};
#endif // ENABLE_SCALE_METER_MODE
#ifdef ENABLE_BALANCED_OSCS_MODE
static PerformanceModeIoRangesMenuPage menuPageBalancedOscs {"balanced oscs", gBalancedOscsMode, false};
#endif //ENABLE_BALANCED_OSCS_MODE
#ifdef ENABLE_EXPR_BUTTONS_MODE
static PerformanceModeIoRangesMenuPage menuPageExprButtons {"expr buttons", gExprButtonsMode, false};
#endif // ENABLE_EXPR_BUTTONS_MODE

std::array<MenuPage*,kNumModes> menuPagesIoRanges {{
#ifdef TEST_MODE
	nullptr,
#endif // TEST_MODE
#ifdef ENABLE_DIRECT_CONTROL_MODE
	&menuPageDirectControl,
#endif // ENABLE_DIRECT_CONTROL_MODE
#ifdef ENABLE_RECORDER_MODE
	&menuPageRecorder,
#endif // ENABLE_RECORDER_MODE
#ifdef ENABLE_SCALE_METER_MODE
	&menuPageScaleMeter,
#endif // ENABLE_SCALE_METER_MODE
#ifdef ENABLE_BALANCED_OSCS_MODE
	&menuPageBalancedOscs,
#endif // ENABLE_BALANCED_OSCS_MODE
#ifdef ENABLE_EXPR_BUTTONS_MODE
	&menuPageExprButtons,
#endif // ENABLE_EXPR_BUTTONS_MODE
	nullptr, // calibration
	nullptr, // factory test
}};
static MenuItemTypeEnterSubmenu enterModeSettingsPage1("ModeSettingsPage1", kSettingsSubmenuButtonColor, 20, globalSettingsMenu0); // dummy menu, replaced before use
static void menu_updateSubmenu()
{
	enterModeSettingsPage1.submenu = menuPagesIoRanges[gNewMode];
}

static bool menuJustEntered;

#ifdef MENU_ENTER_RANGE_DISPLAY
static void menu_enterRangeDisplay(const rgb_t& signalColor, const std::array<rgb_t,2>& endpointsColors, bool autoExit, ParameterContinuous& bottom, ParameterContinuous& top, const float& display)
{
	gAlt = 1;
	singleRangeDisplayMenuItem = MenuItemTypeRangeDisplayCentroids(signalColor, endpointsColors, autoExit, &bottom, &top, nullptr, display);
	menu_in(singleRangeDisplayMenu);
}
#endif // MENU_ENTER_RANGE_DISPLAY

#ifdef MENU_ENTER_SINGLE_SLIDER
static void menu_enterSingleSlider(const rgb_t& color, const rgb_t& otherColor, ParameterContinuous& parameter)
{
	gAlt = 1;
	singleSliderMenuItem = MenuItemTypeSlider(color, otherColor, &parameter);
	menu_in(singleSliderMenu);
}
#endif // MENU_ENTER_SINGLE_SLIDER

static std::vector<MenuPage*> menuStack;

static MenuPage* menu_getCurrent()
{
	return menuStack.size() ? menuStack.back() : nullptr;
}

static void menu_resetStates(const MenuItemType* src)
{
	MenuPage* page = menu_getCurrent();
	if(!page)
		return;
	for(auto& item : page->items)
	{
		if(item != src)
			item->resetState();
	}
}
static void menu_update()
{
	// these vectors should really be initialised at startup but they have circular dependencies
	static bool inited = false;
	if(!inited)
	{
		inited = true;
		globalSettingsMenu0.items = {
			&globalSettingsJacksOnTop,
			&disabled,
			&globalSettingsAnimationMode,
			&globalSettingsBrightness,
			&globalSettingsSizeScale,
		};
		singleSliderMenu.items = {
			&singleSliderMenuItem,
		};
		mainMenu.items = {
			&enterModeSettingsPage1,
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
	constexpr size_t kFirstModeSettingIdx = 1; // the bottom is to enter submenu
	for(size_t n = 0; n < kMaxModeParameters; ++n)
	{
		// make sure we are displaying the buttons for the current mode
		// if hasChanged, this will retrigger a new drawing of the buttons below
		// TODO: when refactoring mode switching, maybe ensure the menu's content and visualisation
		// gets updated directly when updating mode

		if(mainMenu.items[n + kFirstModeSettingIdx] != (*menuItems)[n])
		{
			MenuItemType* newItem = (*menuItems)[n];
			// validate all items before adding them
			if(!newItem)
				newItem = &disabled;
			mainMenu.items[n + kFirstModeSettingIdx] = newItem;
			hasChanged = true;
		}
	}
	MenuPage* newMenu = menu_getCurrent();
	if(newMenu && (activeMenu != newMenu || hasChanged))
	{
		activeMenu = newMenu;
		M(printf("menu_update: %s\n\r", newMenu ? newMenu->name : "___"));
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
		menu_resetStates(nullptr);
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
	M(printf("menu_exit\n\r"));
	np.clear();
	gAlt = 0;
	tri.buttonLedSet(TRI::kSolid, TRI::kG, 1, 100);
}

static void menu_up()
{
	M(printf("menu_up from %d %s\n\r", menuStack.size(), menuStack.back()->name));
	if(menuStack.size())
		menuStack.pop_back();
	if(!menuStack.size())
		menu_exit();
}

static void menu_in(MenuPage& menu)
{
	M(printf("menu_in from %s to %s\n\r", menuStack.size() ? menuStack.back()->name : "___", menu.name));
	menuStack.emplace_back(&menu);
}

int menu_dosetup(MenuPage& menu)
{
	menuStack.resize(0);
	menu_in(menu);
	menu_update(); // TODO: is this needed?
	return true;
}

int menu_setup(size_t page)
{
	if(3 == page)
	{
		requestNewMode(kEraseSettingsModeIdx);
		return true;
	}
	if(2 == page)
	{
		requestNewMode(kFactoryTestModeIdx);
		return true;
	}
	MenuPage* menu;
	switch(page){
	default:
	case 0:
		menu = &mainMenu;
		break;
	case 1:
		menu = &globalSettingsMenu0;
		break;
	}
	return menu_dosetup(*menu);
}

void menu_render(BelaContext*, FrameData* frameData)
{
	// if button pressed, go back up
	if (menuBtn.onset){
		// button onset
		M(printf("button when stack is %d\n\r", menuStack.size()));
		// if one of the ioranges pages, exit
		for(auto& p : menuPagesIoRanges)
		{
			if(p == activeMenu)
			{
				menu_exit();
				return;
			}
		}
		// otherwise just go up by one
		menu_up();
	}

	menu_update();
	if(!activeMenu)
		return;
	// update touches
	if(menuJustEntered)
	{
		// if we just entered the menu, ensure we have removed
		// all fingers once before enabling interaction
		if(!globalSlider.getNumTouches())
			menuJustEntered = false;
	}
	if(menuJustEntered) {
		// provide empty blank data so that process doesn't know about any touches
		// that may still be present after entering menu. The reason we cannot simply use
		// ledSlidersAlt.enableTouch(false) is because also tr_render() sets/unsets
		// that and we may end up in an inconsistent state if we are not careful.
		// conversely, tr_render() doesn't know if we have to ignore touches again at some point
		// e.g.: because we entered a submenu from here.
		// TODO: simplify
		constexpr size_t kNumChannelsMax = 30; // from trill.cpp
		static std::array<float,kNumChannelsMax> dummyData {};
		ledSlidersAlt.process(dummyData.data());
	} else
		ledSlidersAlt.process(trill.rawData.data()); // TODO: calling this only on frameData.isNew causes troubles when gJacksOnTop. Investigate why

	// set the color (i.e.: animation for the _next_ iteration)
	// (it has already been rendered to np from the ledSliders.process() call above)
	uint32_t ms = HAL_GetTick();
	for(size_t n = 0; n < ledSlidersAlt.sliders.size(); ++n)
	{
		MenuItemType* item = activeMenu->items[n];
		// TODO: using timeout because it's easier, but it should be refactored
		// so that it's more elegant
		if(ms - gAnimationPlayedLast < 10)
		{
			// While FS animation is playing, only let the item that triggered it be tapped again
			if(gAnimationIsPlaying && item != gAnimationIsPlaying)
				continue;
		}
		item->process(ledSlidersAlt.sliders[n]);
	}
}
