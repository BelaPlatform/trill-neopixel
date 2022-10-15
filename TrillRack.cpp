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
extern bool (*mode_setups[])(double);
extern void (*mode_loops[])(void);
extern OutMode gOutMode;
extern int gMode;
extern bool modeAlt_setup();
extern void master_clock(float);
extern void divmult_clock(int trigger, float tempoControl);
extern int gMtrClkTrigger;
extern LedSliders ledSliders;
extern LedSliders ledSlidersAlt;
extern void ledSlidersFixedButtonsProcess(LedSliders& sl, std::vector<bool>& states, std::vector<size_t>& onsets, std::vector<size_t>& offsets, bool onlyUpdateStates);
std::vector<float> gManualAnOut(2);

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
unsigned int padsToOrderMap[kNumPads] = {
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

static uint8_t midiInToPixel(uint8_t value)
{
	value = value * 2;
	if(254 == value)
		value = 255;
	return value;
}

#include "LedSliders.h" // rgb_t
static void midiCtlCallback(uint8_t ch, uint8_t num, uint8_t value){
	if(num < 3) {
		value = midiInToPixel(value);
		static rgb_t color;
		if(0 == num)
			color.r = value;
		if(1 == num)
			color.g = value;
		if(2 == num)
			color.b = value;
		for(unsigned int n = 0; n < kNumLeds; ++n)
			np.setPixelColor(n, color.r, color.g, color.b);
		printf("colors %d %d %d\n\r", color.r, color.g, color.b);
	} else {
		static rgb_t color;
		if(3 == num)
		{
			static unsigned int idx = 0;
			idx = value;
			if(idx < kNumLeds) {
				np.setPixelColor(idx, color.r, color.g, color.b);
			}
			printf("color at %d: %d %d %d\n\r", idx, color.r, color.g, color.b);
		} else {
			value = midiInToPixel(value);
			if(4 == num)
				color.r = value;
			if(5 == num)
				color.g = value;
			if(6 == num)
				color.b = value;
			printf("pixel color: %d %d %d\n\r", color.r, color.g, color.b);
		}
	}
	gAlt = 2;
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
	if(trill.prepareForDataRead())
		return false;

	cd.setup({padsToOrderMap, padsToOrderMap + kNumPads / 2}, 4, 1); //dummy sizeScale
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

enum {
	kOutRangeFull,
	kOutRangeBipolar,
	kOutRangePositive5,
	kOutRangePositive10,
	kOutRangeNum,
};
// C++ doesn't allow myenumvar++, so we need this as an int
static int gOutRange = kOutRangeFull;

static float mapAndConstrain(float x, float in_min, float in_max, float out_min, float out_max)
{
	float value = map(x, in_min, in_max, out_min, out_max);
	value = constrain(value, out_min, out_max);
	return value;
}

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

	// Read analog in.
	float anIn = tri.analogRead();
	// Run the clock
	master_clock(anIn*3.0);
	
	// Run the div mult clock
	divmult_clock(gMtrClkTrigger, anIn*3.0);


	// Read 1st digital in (mode switching)
	int diIn0 = tri.digitalRead(0);
	tri.buttonLedWrite(0, !diIn0);
	
	static bool firstRun = true;
	bool justEnteredAlt = false;
	static int gDiIn0Last = 0;
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
	const float gnd = gCalibrationProcedure.getGnd();
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
				if(1 == sls.size() && 1 == n) // if this is a size
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
		tri.analogWrite(n, value);
	}
	
	// Send to scope
	tri.scopeWrite(0, anIn);
	tri.scopeWrite(1, ledSliders.sliders[0].compoundTouchLocation());
	tri.scopeWrite(2, ledSliders.sliders[0].compoundTouchSize());
}
