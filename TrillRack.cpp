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

extern bool gBottomOutIsSize;
extern std::array<rgb_t, 2> gBalancedLfoColors;
extern bool performanceMode_setup(double);
extern void performanceMode_render(BelaContext*);
extern bool menu_setup(double);
extern void menu_render(BelaContext*);
extern bool menuShouldChangeMode();
extern float getGnd();
extern OutMode gOutMode;
extern bool modeAlt_setup();
extern void triggerInToClock(BelaContext* context);
extern int gMtrClkTrigger;
extern LedSliders ledSliders;
extern LedSliders ledSlidersAlt;
extern ButtonView menuBtn;
extern ButtonView performanceBtn;
extern void ledSlidersFixedButtonsProcess(LedSliders& sl, std::vector<bool>& states, std::vector<size_t>& onsets, std::vector<size_t>& offsets, bool onlyUpdateStates);
std::array<float,2> gManualAnOut;

#define STM32_NEOPIXEL
#define REV2
//#define TRILL_BAR // whether to use an external Trill Bar

#ifdef REV2
TrillRackInterface tri(0, 0, 1, __builtin_ctz(SW0_Pin), __builtin_ctz(SW_LED_A_Pin), __builtin_ctz(SW_LED_B_Pin));
#else // REV2
TrillRackInterface tri(0, 0, 1, __builtin_ctz(SW0_Pin), __builtin_ctz(SW_LED_Pin), 6 /* dummy */);
#endif // REV2

NeoPixelT<kNumLeds> np;
Trill trill;
CentroidDetection cd;

extern const unsigned int kNumPads = 26;
std::vector<unsigned int> padsToOrderMap = {
#ifdef REV2
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
#else // REV2
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
#endif // REV2
};
CentroidDetection globalSlider;
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
			gOutMode = kOutModeManualBlock;
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
	assert(kNumPads == padsToOrderMap.size());
	globalSlider.setup(padsToOrderMap, 4, 1);
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
	if(trill.prepareForDataRead())
		return false;

	modeAlt_setup();
	setHdlCtlChange(midiCtlCallback);
	setHdlAll(midiInputCallback);
	int ret = presetInit(kPresetInit_LoadLatest, 2000, HAL_GetTick);
	printf("presetInit() loaded %d\n\r", ret);
	return foundAddress;
}

void tr_mainLoop()
{
	if(!gAlt)
	{
		int ret = presetCheckSave();
		if(ret >= 0)
			printf("presetCheckSave: %d\n\r", ret);
	}
}

void tr_newData(const uint8_t* newData, size_t len)
{
	trill.newData(newData, len);
}

void tr_process(BelaContext* ptr)
{
	tri.process(ptr);
}

// C++ doesn't allow myenumvar++, so we need this as an int
int gOutRange = kCvRangePositive10;
float gOutRangeTop = 1;
float gOutRangeBottom = 0;
int gInRange = kCvRangePositive10;
float gInRangeTop = 1;
float gInRangeBottom = 0;

static inline void getBottomTopRange(int range, bool input, float gnd, float& bottom, float& top)
{
	switch (range)
	{
		case kCvRangeFull:
			bottom = 0;
			top = 1;
			break;
		case kCvRangeBipolar:
			bottom = 0; // -5V TODO: fix
			top = gnd * 2.f;
			break;
		case kCvRangePositive5:
			bottom = gnd;
			top = gnd * 2.f;
			break;
		case kCvRangePositive10:
			bottom = gnd;
			top = gnd * 3.f;
			break;
		default:
		case kCvRangeCustom:
			bottom = input ? gInRangeBottom : gOutRangeBottom;
			top = input ? gInRangeTop : gOutRangeTop;
			break;
	}
}

static float rescaleInput(float gnd, float value)
{
	float bottom;
	float top;
	getBottomTopRange(gInRange, true, gnd, bottom, top);
	return mapAndConstrain(value, bottom, top, 0, 1);
}

static float finalise(float value)
{
#ifdef REV2
	//
	return 1.f - value;
#else // REV2
	return value;
#endif // REV2
}

static float rescaleOutput(size_t channel, float gnd, float value)
{
	if(kNoOutput == value)
		return finalise(gnd);
	// rescale analog outputs
	float bottom;
	float top;
	getBottomTopRange(gOutRange, false, gnd, bottom, top);
	if(gBottomOutIsSize && 1 == channel) // if this is a size
		bottom = gnd; // make it always positive

	// hard-limit the ranges to avoid the nasty edges
	// (note: this does _not_ clip the output, actually it does just the opposite)
	// TODO: validate these values across several units, or possibly via
	// the calibration procedure. Better play it safe and end up slightly short of the
	// nominal range, but keep an unclipped waveform.
	if(bottom < 0.001)
		bottom = 0.001;
	if(top > 0.99)
		top = 0.99;
	value = mapAndConstrain(value, 0, 1, bottom, top);
	return finalise(value);
}

static float touchOrNot(float val, bool hasTouch)
{
	return hasTouch ? val : kNoOutput;
}

void tr_render(BelaContext* context)
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
	triggerInToClock(context);

	const float gnd = getGnd(); // TODO: probably the `gnd` value is not the same for input and output?
	// rescale analog inputs according to range
	// TODO: don't do it if we are using this input for trig instead.
	// TODO: don't do it if we are only using one or 0 frames
	for(size_t idx = 0; idx < context->analogFrames * context->analogInChannels; ++idx)
		context->analogIn[idx] = rescaleInput(gnd, context->analogIn[idx]);

	// Button LEDs:
	// First LED follows button
	tri.buttonLedWrite(0, !tri.digitalRead(0));
	// Second LED displays a clipped version of the input.
	// The clipping ensures that a small offset (e.g.: due to calibration or lack thereof)
	// won't cause the LED to be dim the whole time.
	const float kButtonLedThreshold = 0.04;
	float clippedIn = tri.analogRead() - gnd;
	if(clippedIn < kButtonLedThreshold)
		clippedIn = 0;
	tri.buttonLedWrite(1, clippedIn);
	
	static ButtonView btn; // reflects reality
	static bool wasPressed = !tri.digitalRead(0);
	btn = {false, false, false, false, btn.pressCount};
	bool isPressed = !tri.digitalRead(0);
	btn.enabled = true;
	btn.offset = wasPressed && !isPressed;
	btn.onset = isPressed && !wasPressed;
	if(btn.onset)
	{
		btn.pressCount++;
		if(ButtonView::kPressCountInvalid == btn.pressCount) // reserved value
			btn.pressCount = 0;
	}
	btn.pressed = isPressed;
	wasPressed = isPressed;

	static bool hadTouch = false;
	globalSlider.process(trill.rawData.data());
	size_t numTouches = globalSlider.getNumTouches();
	static bool preMenuActive = false;
	if(btn.pressed)
	{
		const size_t kTouchesForMenu = 2;
		if(numTouches && !hadTouch) // we start touching
			preMenuActive = true;
		if(!numTouches) // we no longer touch
			preMenuActive = false;
		if(numTouches >= kTouchesForMenu && 1 != gAlt)
		{
			//button is on + touches: enter alt mode
			gAlt = 1;
			menu_setup(0);
			preMenuActive = false;
		}
	} else
		preMenuActive = false; // shouldn't be needed, but, you know ...
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
		menu_render(context); // this will set gAlt back to 0 when exiting menu
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
	bool performanceActive = (0 == gAlt) && !menuActive && !preMenuActive;
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
		ledSliders.process(trill.rawData.data());
		performanceMode_render(context); // TODO: we should run the active mode even if we are in alt, but making sure the LEDs don't get set
	} else {
		// zero the outputs
		gManualAnOut[0] = gManualAnOut[1] = 0;
		for(size_t n = 0; n < context->analogFrames; ++n)
		{
			for(size_t c = 0; c < context->analogOutChannels; ++c)
				analogWriteOnce(context, n, c, 0);
		}
	}
	if(gJacksOnTop)
		np.reverse();
	// actually display the updated LEDs
	// this may have been written by alt, mode_setups or mode_renders, whatever last wrote it is whatever we display
	// TODO: clear separation of concerns: at any time make it clear who can write to each pixel.
	np.show();
//	tri.buttonLedWrite(gMtrClkTriggerLED);
	
	// write analog outputs
	auto& sls = ledSliders.sliders;
	if(kOutModeFollowTouch == gOutMode || kOutModeFollowLeds == gOutMode)
	{
		std::array<LedSlider::centroid_t,kNumOutChannels> centroids;
		size_t numCentroids = 1;
		switch (gOutMode)
		{
			case kOutModeFollowTouch:
				centroids[0].location = sls[0].compoundTouchLocation();
				centroids[0].size = sls[0].compoundTouchSize();
				if(2 == sls.size()) {
					numCentroids = 2;
					centroids[1].location = sls[1].compoundTouchLocation();
					centroids[1].size = sls[1].compoundTouchSize();
				}
				break;
			case kOutModeFollowLeds:
				centroids[0] = sls[0][0];
				if (2 == sls.size())
				{
					numCentroids = 2;
					centroids[1] = sls[1][0];
				}
				break;
			default:
				assert(false);
				break;
		}
		gManualAnOut[0] = touchOrNot(centroids[0].location, centroids[0].size);
		if(1 == numCentroids)
			gManualAnOut[1] = touchOrNot(centroids[0].size, centroids[0].size);
		else if (2 == numCentroids)
			gManualAnOut[1] = touchOrNot(centroids[1].location, centroids[1].size);
		else
			assert(false);
	}
	if(kOutModeManualSample == gOutMode)
	{
		constexpr size_t kNumOutChannels = 2; // hardcode to give the compiler room for optimisations
		assert(kNumOutChannels == context->analogOutChannels);

		// analogOut has already been written. Rescale in-place
		for(unsigned int n = 0; n < context->analogFrames; ++n)
		{
			for(unsigned int channel = 0; channel < kNumOutChannels; ++channel)
			{
				size_t idx = n * kNumOutChannels + channel;
				context->analogOut[idx] = rescaleOutput(channel, gnd, context->analogOut[idx]);
			}
		}
	} else {
		std::array<float, gManualAnOut.size()> anOutBuffer;
		for(unsigned int c = 0; c < gManualAnOut.size(); ++c)
			anOutBuffer[c] = rescaleOutput(c, gnd, gManualAnOut[c]);
		for(unsigned int n = 0; n < context->analogFrames; ++n)
		{
			for(unsigned int channel = 0; channel < anOutBuffer.size(); ++channel)
			{
				static float pastOut[anOutBuffer.size()];
				float tmp = pastOut[channel];
				float alpha = 0.993;
				float out = tmp * alpha + anOutBuffer[channel] * (1.f - alpha);
				analogWriteOnce(context, n, channel, out);
				pastOut[channel] = out;
			}
		// TODO: could check for nan and clean filter in that case
		// (((uint32_t*)(&floatValue))[0] == 0x7fc00000) // is nan
		}
	}
	bool overrideBottomOutput = tick - gOverride.started < 10;
	if(overrideBottomOutput)
	{
		bool bottomOutIsSizeStash = gBottomOutIsSize;
		gBottomOutIsSize = true;
		unsigned int c = 1;
		float value = rescaleOutput(c, gnd, gOverride.out);
		for(unsigned int n = 0; n < context->analogFrames; ++n)
		{
			analogWriteOnce(context, n, c, value);
		}
		gBottomOutIsSize = bottomOutIsSizeStash;
	}
}
