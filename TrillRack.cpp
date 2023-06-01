#include <TrillRackApplication_bsp.h>
#include "TrillRackInterface.h"
#include "GlissModes.h"
#include "preset.h"
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include <libraries/Trill/CentroidDetection.h> // include this above NeoPixel or "HEX" gets screwed up
#include "LedSliders.h"
#include <cmath>
#include <assert.h>
#include "usbd_midi_if.h"
#include <atomic>

constexpr std::array<float,CalibrationData::kNumPoints> CalibrationData::points;

extern std::array<rgb_t, 2> gBalancedLfoColors;
extern bool performanceMode_setup(double);
extern void performanceMode_render(BelaContext*, FrameData*);
extern void menu_render(BelaContext*, FrameData*);
extern bool menuShouldChangeMode();
extern float getGnd();
extern bool modeAlt_setup();
extern void triggerInToClock(BelaContext* context);
extern int gMtrClkTrigger;
extern LedSliders ledSliders;
extern LedSliders ledSlidersAlt;
extern ButtonView menuBtn;
extern ButtonView performanceBtn;
extern void ledSlidersFixedButtonsProcess(LedSliders& sl, std::vector<bool>& states, std::vector<size_t>& onsets, std::vector<size_t>& offsets, bool onlyUpdateStates);
std::array<float,kNumOutChannels> gManualAnOut;

#define STM32_NEOPIXEL

// Gliss revs:
// 1: no logo, exposed copper, non-inverting I/O, only used internally
// 2: logo, exposed copper, inverting I/O, first beta testing
// 3: logo, transparent solder mask, inverting I/O, rc1
#define GLISS_HW_REV 3
//#define TRILL_BAR // whether to use an external Trill Bar

#if GLISS_HW_REV >= 2
TrillRackInterface tri(0, 0, 1, __builtin_ctz(SW0_Pin), __builtin_ctz(SW_LED_A_Pin), __builtin_ctz(SW_LED_B_Pin));
#else
TrillRackInterface tri(0, 0, 1, __builtin_ctz(SW0_Pin), __builtin_ctz(SW_LED_Pin), 6 /* dummy */);
#endif

NeoPixelT<kNumLeds> np;
Trill trill;

std::vector<unsigned int> padsToOrderMap = {
#if GLISS_HW_REV >= 2
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
	20,
	12,
	10,
	9,
	3,
	4,
	5,
	6,
	7,
	8,
	11,
	13,
	14,
	15,
#else
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
#endif
};
CentroidDetectionScaled globalSlider;
int gAlt = 0;

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
		np.clear();
		np.setPixelColor(kNumLeds - 1, 0, 255, 0);
		// show() may fail if another buffer is being sent right now.
		// TODO: wait for it but ensure the timer thread has a higher preemption priority than this one
		np.show();
		printf("Jumping to bootloader\n\r");
		bootloaderResetTo();
	}
	return 0;
}

static uint8_t midiInToPixel(uint8_t value)
{
	value = value * 2;
	if(254 == value)
		value = 255;
	return value;
}

#include "LedSliders.h" // rgb_t
static void midiCtlCallback(uint8_t ch, uint8_t num, uint8_t value){
	bool shouldOverrideDisplay = false;
	if (num < 100){
		static rgb_t color;
		if(num < 3) {
			value = midiInToPixel(value);
			if(0 == num)
				color.r = value;
			if(1 == num)
				color.g = value;
			if(2 == num)
				color.b = value;
		}
		if (4 == num) {
			// we ignore the controller's value: just use this as a trigger
			for(unsigned int n = 0; n < kNumLeds; ++n)
				np.setPixelColor(n, color.r, color.g, color.b);
			printf("all leds to %d %d %d\n\r", color.r, color.g, color.b);
			shouldOverrideDisplay = true;
		}
		else if(5 == num)
		{
			unsigned int idx = 0;
			idx = value;
			if(idx < kNumLeds) {
				np.setPixelColor(idx, color.r, color.g, color.b);
			}
			printf("color at pixel %d: %d %d %d\n\r", idx, color.r, color.g, color.b);
			shouldOverrideDisplay = true;
		} else if(6 == num) {
			unsigned int split = value;
			LedSliders& sliders = (1 == gAlt) ? ledSlidersAlt : ledSliders;
			// set color currently used by active mode
			if(split < sliders.sliders.size()) {
				// for most modes
				sliders.sliders[split].setColor(color);
				printf("mode color at split %d: %d %d %d\n\r", split, color.r, color.g, color.b);
			}
			if(split < gBalancedLfoColors.size()) // for balanced lfo modes
				gBalancedLfoColors[split] = color;

		}
	} else {
		// set DACs
		static int msb;
		if(100 == num)
			msb  = value;
		else if (101 == num) {
			int lsb = value;
			gOutMode.fill(kOutModeManualBlock);
			float f = ((msb << 7) | lsb) / 4096.f;
			gManualAnOut[0] = f;
			gManualAnOut[1] = f;
			shouldOverrideDisplay = true; // override display so we know something's off
		}
	}
	if(shouldOverrideDisplay)
		gAlt = 2;
}

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
#ifdef TRILL_BAR
	padsToOrderMap.resize(kNumPads);
	for(size_t n = 0; n < padsToOrderMap.size(); ++n)
		padsToOrderMap[n] = n;
#endif // TRILL_BAR
	// find unused pads and mark them as such,
	// then shift channel numbers accordingly
	uint32_t channelMask = 0;
	for(auto p : padsToOrderMap)
		channelMask |= 1 << p;
	assert(kNumPads == padsToOrderMap.size());
	assert(kNumPads == __builtin_popcount(channelMask));
	for(auto& p : padsToOrderMap)
	{
		assert(channelMask & (1 << p));
		unsigned int end = p;
		for(unsigned int n = 0; n < end; ++n)
			if(!(channelMask & (1 << n))) // decrement once for every skipped channel
				p--;
	}
	globalSlider.setup(padsToOrderMap, 5, 1);
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
	if(trill.firmwareVersion() < 3)
	{
		printf("Incompatible Trill firwmware version: %d, should be >= %d\n\r", trill.firmwareVersion(), 3);
		return false;
	}
	trill.printDetails();
	if(trill.setMode(Trill::DIFF))
		return false;
	int prescaler;
#ifdef TRILL_BAR
	prescaler = 2;
#elif GLISS_HW_REV >= 3
	prescaler = 3;
#else
	prescaler = 5;
#endif

	if(trill.setChannelMask(channelMask))
		return false;;
	if(trill.setScanSettings(0, 12))
		return false;
	if(trill.setPrescaler(prescaler))
		return false;
	if(trill.setNoiseThreshold(0.06))
		return false;
	if(trill.updateBaseline())
		return false;
	if(trill.setScanTrigger(Trill::kScanTriggerI2c))
		return false;
	if(trill.setEventMode(Trill::kEventModeAlways)) // ... and set PSOC_EVENT pin when ready
		return false;
	if(trill.readStatusByte() < 0) // ensure that future reads via DMA have the correct offset
		return false;
	modeAlt_setup();
	setHdlCtlChange(midiCtlCallback);
	setHdlAll(midiInputCallback);
	PresetInitOptions_t presetType;
#ifdef TEST_MODE
	presetType = kPresetInit_LoadDefault;
#else // TEST_MODE
	presetType = kPresetInit_LoadLatest;
#endif // TEST_MODE
	int ret = presetInit(presetType, 2000, HAL_GetTick);
	printf("presetInit() loaded %d\n\r", ret);
	return foundAddress;
}

void tr_mainLoop()
{
#ifndef TEST_MODE
	if(!gAlt)
	{
		int ret = presetCheckSave();
		if(ret >= 0)
			printf("presetCheckSave: %d\n\r", ret);
	}
#endif // TEST_MODE
}

void tr_clearLeds()
{
#ifdef STM32_NEOPIXEL
	np.setSnp(&snp);
#endif // STM32_NEOPIXEL
	np.clear();
	np.show();
}

static int gShouldScan = 1;

void tr_requestScan(int val)
{
	gShouldScan = val;
}

int tr_scanRequested()
{
	return gShouldScan;
}

static int gShouldUpdateLeds = 1;

void tr_requestUpdateLeds(int val)
{
	gShouldUpdateLeds = val;
}

int tr_ledsUpdateRequested()
{
	return gShouldUpdateLeds;
}

constexpr uint32_t kInvalidFrameId = -1;
static std::atomic_uint32_t trillFrameId { kInvalidFrameId };

void tr_newData(const uint8_t* newData, size_t len)
{
	static uint32_t pastStatusByte = newData[0];
	// only process new data if they belong to a new frame
	if(pastStatusByte != newData[0])
	{
		pastStatusByte = newData[0];
		// mark data as invalid
		trillFrameId.store(kInvalidFrameId);
		// operate on it
		trill.newData(newData, len, true);
		// mark data as valid again
		trillFrameId.store(trill.getFrameIdUnwrapped());
	}
}

void tr_process(BelaContext* ptr)
{
	tri.process(ptr);
}

IoRange gInRange = {
	.range = kCvRangePositive10,
	.min = 0,
	.max = 1,
	.enabled = true,
};
IoRange gOutRangeTop = {
	.range = kCvRangePositive10,
	.min = 0,
	.max = 1,
	.enabled = true,
};
IoRange gOutRangeBottom = {
	.range = kCvRangePositive10,
	.min = 0,
	.max = 1,
	.enabled = true,
};

static inline void getRangeMinMax(bool input, size_t channel, float& min, float& max)
{
	float gnd = CalibrationData::kGnd;
	const IoRange& ioRange = input ? gInRange : (0 == channel) ? gOutRangeTop : gOutRangeBottom;
	CvRange range = ioRange.enabled ? ioRange.range : kCvRangeFull;
	switch (range)
	{
		case kCvRangeFull:
			min = 0;
			max = 1;
			break;
		case kCvRangeBipolar:
			min = 0;
			max = gnd * 2.f;
			break;
		case kCvRangePositive5:
			min = gnd;
			max = gnd * 2.f;
			break;
		case kCvRangePositive10:
			min = gnd;
			max = gnd * 3.f;
			break;
		default:
		case kCvRangeCustom:
			min = ioRange.min;
			max = ioRange.max;
			break;
	}
}

static float processRawThroughCalibration(const CalibrationData& cal, bool input, float raw)
{
	float value = 1;
	for(size_t n = 1; n < cal.points.size(); ++n)
	{
		// linear interpolation between the two nearest calibration points
		if(raw <= cal.points[n])
		{
			if(input)
				value = map(raw, cal.values[n - 1], cal.values[n], cal.points[n - 1], cal.points[n]);
			else
				value = map(raw, cal.points[n - 1], cal.points[n], cal.values[n - 1], cal.values[n]);
			break;
		}
	}
	return value;
}

static float rescaleInput(const CalibrationData& inCal, float value)
{
	float min;
	float max;
	value = processRawThroughCalibration(inCal, true, value);
	getRangeMinMax(true, 0, min, max);
	return mapAndConstrain(value, min, max, 0, 1);
}

static float finalise(float value)
{
#if GLISS_HW_REV >= 2
	// inverting output
	return 1.f - value;
#else
	return value;
#endif
}

static float rescaleOutput(bool ignoreRange, size_t channel, const CalibrationData& cal, float value)
{
	float gnd = cal.values[1];
	if(kNoOutput == value)
		return finalise(gnd);
	float min = 0;
	float top = 1;
	if(!ignoreRange)
		getRangeMinMax(false, channel, min, top);
	if(gOutIsSize[channel]) // if this is a size
		min = gnd; // make it always positive

	value = mapAndConstrain(value, 0, 1, min, top);
	value = processRawThroughCalibration(cal, false, value);
	return finalise(value);
}

static float touchOrNot(float val, bool hasTouch)
{
	return hasTouch ? val : kNoOutput;
}

static void analogWriteJacks(BelaContext* context, unsigned int frame, unsigned int channel, float value)
{
	// swap out channels if gJacksOnTop
	unsigned int c = gJacksOnTop ? !channel : channel;
	analogWriteOnce(context, frame, c, value);
}

void tr_render(BelaContext* context)
{
	static uint32_t pastFrameId = kInvalidFrameId;
	uint32_t frameId = trillFrameId.load();
	bool newFrame = false;
	if(frameId != kInvalidFrameId && frameId != pastFrameId)
	{
		newFrame = true;
		pastFrameId = frameId;
	}
#if 0 //  count average frames of capacitive data
	static uint32_t firstFrameId = frameId;
	static uint32_t count = 0;
	if(newFrame)
	{
		static uint32_t lastCount = 0;
		if(lastCount && count - lastCount != 5)
			printf("WRONG COUNT: %lu\n", count - lastCount);
		lastCount = count;
		if(count > 1000) {
			float mean = (frameId - firstFrameId) / double(count);
			printf(" %.4f\n\r", mean);
			count = 0;
			lastCount = 0;
			firstFrameId = frameId;
		}
	}
	count++;
#endif
	FrameData frameData = {
			.id = pastFrameId,
			.isNew = newFrame,
	};
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
	np.clear(); // clear display before we start writing to it
	processMidiMessage();
	triggerInToClock(context);

	float min = kSliderBottomMargin;
	float max = 1.f - kSliderTopMargin;
	if(gJacksOnTop)
		std::swap(min, max);
	globalSlider.setUsableRange(min, max);

	const CalibrationData& inCal = getCalibrationInput();
	// rescale analog inputs according to range
	// TODO: don't do it if we are using this input for trig instead.
	// TODO: don't do it if we are only using one or 0 frames
	for(size_t idx = 0; idx < context->analogFrames * context->analogInChannels; ++idx)
		context->analogIn[idx] = rescaleInput(inCal, context->analogIn[idx]);

#ifndef TEST_MODE
	// Button LEDs:
	// First LED follows button
//	tri.buttonLedWrite(0, !tri.digitalRead(0));
	// Second LED displays a clipped version of the input.
	// The clipping ensures that a small offset (e.g.: due to calibration or lack thereof)
	// won't cause the LED to be dim the whole time.
	// We don't use the calibrated input here, as we want the display to be independent
	// of the input range
	const float kButtonLedThreshold = 0.04;
	static_assert(inCal.points[1] == CalibrationData::kGnd); // we assume points[1] represents gnd
//	float clippedIn = tri.analogRead() - inCal.values[1]; // positive voltages
//	if(clippedIn < kButtonLedThreshold)
//		clippedIn = 0;
//	tri.buttonLedWrite(1, clippedIn);
#endif // TEST_MODE
	extern size_t msToNumBlocks(BelaContext* context, float ms);
	static const uint32_t kDoubleClickTime = msToNumBlocks(context, 300);
	static const uint32_t kTripleClickTime = msToNumBlocks(context, 500);
	static uint32_t timeNow = 0;
	static uint32_t lastOnsetTime = 0;
	static uint32_t lastLastOnsetTime = 0;
	static ButtonView btn; // reflects reality
	static bool wasPressed = !tri.digitalRead(0);
	static uint32_t doubleClickPressId = -1;
	static uint32_t tripleClickPressId = -1;
	btn = {false, false, false, false, false, false, false, false, btn.pressId, btn.pressDuration};
	bool isPressed = !tri.digitalRead(0);
	btn.enabled = true;
	btn.offset = wasPressed && !isPressed;
	btn.onset = isPressed && !wasPressed;
	if(btn.onset)
	{
		btn.pressId++;
		if(ButtonView::kPressIdInvalid == btn.pressId) // reserved value
			btn.pressId = 0;
		btn.pressDuration = 0;
		if(timeNow - lastLastOnsetTime < kTripleClickTime)
		{
			btn.tripleClick = true;
			tripleClickPressId = btn.pressId;
		}
		else if(timeNow - lastOnsetTime < kDoubleClickTime)
		{
			btn.doubleClick = true;
			doubleClickPressId = btn.pressId;
		}
		lastLastOnsetTime = lastOnsetTime;
		lastOnsetTime = timeNow;
	}
	if(btn.offset)
	{
		 if(btn.pressId == tripleClickPressId)
			 btn.tripleClickOffset = true;
		 else if(btn.pressId == doubleClickPressId)
			btn.doubleClickOffset = true;
	}
	btn.pressed = isPressed;
	if(btn.pressed)
		btn.pressDuration += isPressed;
	else
		btn.pressDuration = 0;
	wasPressed = isPressed;
	timeNow++;

	static bool hadTouch = false;
	if(newFrame)
		globalSlider.process(trill.rawData.data());
	size_t numTouches = globalSlider.getNumTouches();
	static enum {
		kMenuChangeDisabled = 0,
		kMenuPre,
	} menuState = kMenuChangeDisabled;
	if(btn.pressed)
	{
		static constexpr std::array<size_t, 4> touchesForMenu = {
			2, // kTouchesForLocalSettings
			3, // kTouchesForGlobalSettings0
			4, // kTouchesForGlobalSettings1
			5, // kTouchesForFactoryTest
		};

		// keep tracking of max touches to avoid entering the wrong mode when releasing
		static size_t maxTouchesThisMenuPre = 0;
		if(numTouches && !hadTouch) {// we start touching
			menuState = kMenuPre;
			maxTouchesThisMenuPre = 0;
		}
		if(!numTouches) // we no longer touch
			menuState = kMenuChangeDisabled;
		if(kMenuPre == menuState)
		{
			if(numTouches > maxTouchesThisMenuPre)
			{
				// we have a new touch
				for(ssize_t n = touchesForMenu.size() - 1; n >= 0; --n)
				{
					if(touchesForMenu[n] == numTouches)
					{
						menu_setup(n);
						gAlt = 1;
						tri.buttonLedSet(TrillRackInterface::kSolid, TrillRackInterface::kG, 1, 100);
						break;
					}
				}
			}
			maxTouchesThisMenuPre = std::max(maxTouchesThisMenuPre, numTouches);
		}
	} else
		menuState = kMenuChangeDisabled; // shouldn't be needed, but, you know ...
	hadTouch = numTouches > 0;
	bool menuActive = (1 == gAlt);

	// multiplexer part 1
	const static ButtonView disBtn = {0};
	menuBtn = menuActive ? btn : disBtn;
	ledSlidersAlt.enableTouch(menuActive);
	ledSlidersAlt.enableLeds(menuActive);

	if(menuActive)
	{
		// has to run before multiplexer part 2 so
		// that any slider that gets recreated because of menu action
		// doesn't lose its enables. TODO: fix this better
		menu_render(context, &frameData); // this will set gAlt back to 0 when exiting menu
	}

	static bool menuExitWaitingButtonRelease = false;
	static bool menuExitWaitingTouchRelease = false;
	static int oldAlt = gAlt;
	if(1 == oldAlt && 0 == gAlt)
	{
		// we just got out of menu mode. We may have done so by pressing the button (with or without a touch)
		// or by releasing a touch (in which case we won't have any touches here)
		// If button was pressed and/or touch was active when exiting from menu,
		// we should wait for each to be released before they get re-enabled for performance
		if(btn.pressed)
			menuExitWaitingButtonRelease = true;
		if(globalSlider.getNumTouches())
			menuExitWaitingTouchRelease = true;
	} else { // else ensures we don't run this uselessly in the same block where they were set
		if(menuExitWaitingButtonRelease) {
			if(!btn.pressed && !btn.offset)
				menuExitWaitingButtonRelease = false;
		}
		if(menuExitWaitingTouchRelease) {
			if(!globalSlider.getNumTouches())
				menuExitWaitingTouchRelease = false;
		}
	}
	oldAlt = gAlt;

	// multiplexer part 2
	bool performanceActive = (0 == gAlt) && !menuActive && kMenuChangeDisabled == menuState;
	performanceBtn = (performanceActive && !menuExitWaitingButtonRelease) ? btn : disBtn;
	ledSliders.enableTouch(performanceActive && !menuExitWaitingTouchRelease);
	ledSliders.enableLeds(performanceActive);

	static double setupMs;
	static bool setupDone = false;
	static int lastMode = -1;
	if(lastMode != gNewMode) {
		setupMs = tri.getTimeMs();
		printf("new mode: %d\n\r", lastMode);
		lastMode = gNewMode;
		setupDone = false;
	}

	if(!setupDone) {
		setupDone = performanceMode_setup(tri.getTimeMs() - setupMs);
	}
	if(setupDone)
	{
		if(newFrame)
			ledSliders.process(trill.rawData.data());
		performanceMode_render(context, &frameData);
	} else {
		// zero the outputs
		gManualAnOut[0] = gManualAnOut[1] = 0;
		for(size_t n = 0; n < context->analogFrames; ++n)
		{
			for(size_t c = 0; c < context->analogOutChannels; ++c)
				analogWriteJacks(context, n, c, 0);
		}
	}
	if(gJacksOnTop)
		np.reverse();
	// actually display the updated LEDs
	// this may have been written by alt, mode_setups or mode_renders, whatever last wrote it is whatever we display
	// TODO: clear separation of concerns: at any time make it clear who can write to each pixel.
	if(tr_ledsUpdateRequested())
	{
		np.scaleBy(gBrightness);
		np.show();
	}
//	tri.buttonLedWrite(gMtrClkTriggerLED);
	
	// write analog outputs
	const CalibrationData& outCal = getCalibrationOutput();

	assert(kNumOutChannels == context->analogOutChannels);

	// analogOut has already been written. Rescale in-place
	for(unsigned int n = 0; n < context->analogFrames; ++n)
	{
		for(unsigned int channel = 0; channel < kNumOutChannels; ++channel)
		{
			if(kOutModeManualSample == gOutMode[channel])
			{
				size_t idx = n * kNumOutChannels + channel;
				context->analogOut[idx] = rescaleOutput(false, channel, outCal, context->analogOut[idx]);
			}
		}
	}
	// we do the loop again to swap channels if needed
	// this can be incorporated in the above loop
	// but it needs a lot of care not to overwrite the other channel
	// note: if the two channels have a different gOutMode, this may swap some stale data
	// that will be overwritten below
	// TODO: consolidate these three loops. Try to incorporate this above, or just use it on
	// all channels after the next loop.
	if(gJacksOnTop && (kOutModeManualSample == gOutMode[0] || kOutModeManualSample == gOutMode[1]))
	{
		for(unsigned int n = 0; n < context->analogFrames; ++n)
		{
			size_t idx0 = n * kNumOutChannels;
			std::swap(context->analogOut[idx0], context->analogOut[idx0 + 1]);
		}
	}
	std::array<float, gManualAnOut.size()> anOutBuffer;
	for(unsigned int channel = 0; channel < anOutBuffer.size(); ++channel)
	{
		if(kOutModeManualBlock == gOutMode[channel])
		{
			anOutBuffer[channel] = rescaleOutput(false, channel, outCal, gManualAnOut[channel]);
			for(unsigned int n = 0; n < context->analogFrames; ++n)
			{
				static float pastOut[anOutBuffer.size()];
				float tmp = pastOut[channel];
				float alpha = 0.993;
				float out = tmp * alpha + anOutBuffer[channel] * (1.f - alpha);
				analogWriteJacks(context, n, channel, out);
				pastOut[channel] = out;
			}
		// TODO: could check for nan and clean filter in that case
		// (((uint32_t*)(&floatValue))[0] == 0x7fc00000) // is nan
		}
	}
	bool overrideOutput = tick - gOverride.started < 10;
	if(overrideOutput)
	{
		auto outIsSizeStash = gOutIsSize;
		gOutIsSize[gOverride.ch] = gOverride.isSize;
		unsigned int c = gOverride.ch;
		float value = rescaleOutput(gOverride.bypassOutRange, c, outCal, gOverride.out);
		for(unsigned int n = 0; n < context->analogFrames; ++n)
			analogWriteJacks(context, n, c, value);
		gOutIsSize = outIsSizeStash;
	}
#if 0 // send out a quiet tone on one channel and loop back the input on the other
	for(unsigned int n = 0; n < context->analogFrames; ++n)
	{
		analogWriteJacks(context, n, 0, analogRead(context, n, 0));
		float osc = (n % 32 > 16);
		analogWriteOnce(context, n , 1, (osc + 2048) / 4096.f);
	}
#endif
}
