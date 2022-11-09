#include <TrillRackApplication_bsp.h>
#include "TrillRackInterface.h"
#include "GlissModes.h"
#include <libraries/Trill/Trill.h> // include this above NeoPixel or "HEX" gets screwed up
#include <libraries/Trill/CentroidDetection.h> // include this above NeoPixel or "HEX" gets screwed up
#include "NeoPixel.h"
#include "LedSliders.h"
#include <cmath>
#include <assert.h>
extern "C" {
#include "usbd_midi_if.h"
};
extern const unsigned int kNumModes;
extern bool gSecondTouchIsSize;
extern std::array<rgb_t, 2> gBalancedLfoColors;
extern bool (*mode_setups[])(double);
extern void (*mode_renders[])(BelaContext*);
extern bool menu_setup(double);
extern void menu_render(BelaContext*);
extern bool menuShouldChangeMode();
extern float getGnd();
extern OutMode gOutMode;
extern int gMode;
extern bool modeAlt_setup();
extern void triggerInToClock(BelaContext* context);
extern int gMtrClkTrigger;
extern LedSliders ledSliders;
extern LedSliders ledSlidersAlt;
extern void ledSlidersFixedButtonsProcess(LedSliders& sl, std::vector<bool>& states, std::vector<size_t>& onsets, std::vector<size_t>& offsets, bool onlyUpdateStates);
std::array<float,2> gManualAnOut;

#define REV2
//#define TRILL_BAR // whether to use an external Trill Bar

#ifdef REV2
TrillRackInterface tri(0, 0, 1, __builtin_ctz(SW0_Pin), __builtin_ctz(SW_LED_A_Pin), __builtin_ctz(SW_LED_B_Pin));
#else // REV2
TrillRackInterface tri(0, 0, 1, __builtin_ctz(SW0_Pin), __builtin_ctz(SW_LED_Pin), 6 /* dummy */);
#endif // REV2

extern const unsigned int kNumLeds = 23;
NeoPixel np(kNumLeds, 0, NEO_RGB);
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
			// set color currently used by active mode
			if(split < ledSliders.sliders.size()) {
				// for most modes
				ledSliders.sliders[split].setColor(color);
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
			gOutMode = kOutModeManual;
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
	if(trill.prepareForDataRead())
		return false;

	modeAlt_setup();
	setHdlCtlChange(midiCtlCallback);
	setHdlAll(midiInputCallback);
	return foundAddress;
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
int gOutRange = kOutRangeFull;

static float mapAndConstrain(float x, float in_min, float in_max, float out_min, float out_max)
{
	float value = map(x, in_min, in_max, out_min, out_max);
	value = constrain(value, out_min, out_max);
	return value;
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
	// Read 1st digital in (mode switching)
	int diIn0 = tri.digitalRead(0);
	// Button LEDs:
	// First LED follows button
	tri.buttonLedWrite(0, !diIn0);
	// Second LED displays a clipped version of the input.
	// The clipping ensures that a small offset (e.g.: due to calibration or lack thereof)
	// won't cause the LED to be dim the whole time.
	const float kButtonLedThreshold = 0.04;
	float clippedIn = tri.analogRead() - gnd;
	if(clippedIn < kButtonLedThreshold)
		clippedIn = 0;
	tri.buttonLedWrite(1, clippedIn);
	
	static bool firstRun = true;
	static bool hadTouch = false;
	bool hasTouch = false;
	// TODO: it would be nicer to use globalSlider instead, but that would require calling process()
	// on it every time, which may be expensive
	for(auto& s : ledSliders.sliders)
	{
		if((hasTouch = s.getNumTouches()))
			break;
	}
	if(!diIn0)
	{
		if(hasTouch && !hadTouch)
		{
			//button is on + one touch: enter alt mode
			gAlt = 1;
			menu_setup(0);
		}
	}
	hadTouch = hasTouch;
	if(1 == gAlt)
	{
		menu_render(context); // this will set gAlt back to 0 when exiting menu
	}

	static double setupMs = 0;
	static bool setupDone = false;
	int shouldChangeMode = menuShouldChangeMode();
	if(shouldChangeMode) {
		setupMs = tri.getTimeMs();
		if(!firstRun)
			gMode = (gMode + shouldChangeMode + kNumModes) % kNumModes;
		printf("mode: %d\n\r", gMode);
		setupDone = false;
	}
	firstRun = false;

	if(!setupDone) {
		gSecondTouchIsSize = false; // will be set as needed by the call in the next line
		setupDone = mode_setups[gMode](tri.getTimeMs() - setupMs);
	}
	if(setupDone)
	{
		if(!gAlt) {
			ledSliders.process(trill.rawData.data());
			mode_renders[gMode](context); // TODO: we should run the active mode even if we are in alt, but making sure the LEDs don't get set
		}
	}
	// actually display the updated LEDs
	// this may have been written by alt, mode_setups or mode_renders, whatever last wrote it is whatever we display
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
	bool shouldUseAnOutBuffer = true; // TODO: add new modes which set it to false
	std::array<float, gManualAnOut.size()> anOutBuffer;
	for(unsigned int n = 0; n < gManualAnOut.size(); ++n)
	{
		float value = gManualAnOut[n];
		// rescale analog outputs
		switch (gOutRange)
		{
			case kOutRangeFull:
				// nothing to do
				break;
			case kOutRangeBipolar:
			{
				float base = 0; // -5V
				if(gSecondTouchIsSize && 1 == n) // if this is a size
					base = gnd; // make it always positive
				value = mapAndConstrain(value, 0, 1, base, gnd * 2.f);
			}
				break;
			case kOutRangePositive5:
				value = mapAndConstrain(value, 0, 1, gnd, gnd * 2.f);
				break;
			case kOutRangePositive10:
				value = mapAndConstrain(value, 0, 1, gnd, gnd * 3.f);
				break;
		}
#ifdef REV2
		value = 1.f - value; // inverting outs
#endif // REV2
		// actually write analog outs
		anOutBuffer[n] = value;
	}
	if(shouldUseAnOutBuffer)
	{
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
		}
	}
}
