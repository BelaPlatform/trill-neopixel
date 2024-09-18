#include <array>
#include <string.h>
#include <Utilities.h>

constexpr size_t kNumPads = 26;
constexpr size_t kNumLeds = 23;
constexpr float kSliderBottomMargin = 0.05;
constexpr float kSliderTopMargin = 0.05;
typedef enum {
	kOutModeManualBlock,
	kOutModeManualSample,
	kOutModeManualSampleSmoothed,
	kOutModeManualBlockCustomSmoothed,
} OutMode;
constexpr size_t kNumOutChannels = 2; // TODO: assert it's the same as context->analogOutChannels
extern std::array<float,kNumOutChannels> gCustomSmoothedAlpha;
extern std::array<OutMode,kNumOutChannels> gOutMode;
constexpr float kAlphaDefault = 0.993;
float getOutputSmoothDiff(size_t idx);
float getOutputReverseMap(size_t idx);
float getOutputSmoothDiffNormalised(size_t idx);

struct CalibrationData {
	static constexpr float kGnd = 0.3333333333f;
	static constexpr size_t kNumPoints = 3;
	static constexpr std::array<float,kNumPoints> points = {{0, kGnd, 1}};
	std::array<float,kNumPoints> values;
	operator std::array<float,kNumPoints>() const { return values; }
	bool operator== (const CalibrationData& other) { return !memcmp(this, &other, sizeof(other)); }
	bool operator!= (const CalibrationData& other) { return !(*this == other); }
};
CalibrationData const& getCalibrationInput();
CalibrationData const& getCalibrationOutput();

typedef enum {
	kCvRangePositive10,
	kCvRangeBipolar,
	kCvRangePositive5,
	kCvRangeBipolar1,
	kCvRangeCustom,
	kCvRangeNum,
} CvRange;

struct IoRange {
	// these are changed explicitly via global settings and are stored as preset
	float min;
	float max;
	CvRange range;
	// this can be changed dynamically by a mode and is not stored
	uint8_t enabled;
	// this struct is saved to storage and we want all
	// of its bytes to be initialised to a known value
	// We do not want to use PACKED_STRUCT() because it may mess with the alignment
	// of the floats. Instead, we add initialised padding bytes.
	// static_assert below should remind us about this if we change the elements of this struct
	uint16_t padding {0};
	static constexpr float gnd = CalibrationData::kGnd;
public:
	static IoRange init() {
		return IoRange {
			.min = 0, // -5 V
			.max = 1, // +10 V
			.range = kCvRangePositive10,
			.enabled = true,
		};
	}
	void getMinMax(float& min, float& max) const {
		static int constexpr kCvRangeFull = kCvRangeNum + 1;
		int range = enabled ? this->range : kCvRangeFull;
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
			case kCvRangeBipolar1:
				min = gnd - gnd / 5.f;
				max = gnd + gnd / 5.f;
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
				min = this->min;
				max = this->max;
				break;
		}
	}
	// What value should we set to obtain 0V
	// May return a value outside [0, 1]
	float getGnd() const
	{
		float min;
		float max;
		getMinMax(min, max);
		return map(gnd, min, max, 0, 1);
	}
};

struct IoRanges {
	IoRange in;
	IoRange outTop;
	IoRange outBottom;
	static IoRanges init() {
		return {
			IoRange::init(),
			IoRange::init(),
			IoRange::init(),
		};
	}
	IoRange& operator[] (size_t n){
		switch(n) {
		default:
		case 0:
			return in;
		case 1:
			return outTop;
		case 2:
			return outBottom;
		}
	};
	static constexpr size_t size() {
		return 3;
	}
};
// as mentioned above, we want these structs's bytes to be fully initialised
// so that they can be binary-compared reliably.
// so here we check for no unexpected padding
static_assert(sizeof(IoRange) == 12);
static_assert(sizeof(IoRanges) == 3 * sizeof(IoRange));

extern IoRange gInRange;
extern IoRange gOutRangeTop;
extern IoRange gOutRangeBottom;

const float kNoOutput = -12345.6;

struct ButtonView {
	bool onset;
	bool offset;
	bool pressed;
	bool enabled;
	bool doubleClick;
	bool tripleClick;
	bool doubleClickOffset;
	bool tripleClickOffset;
	uint32_t pressId;
	uint32_t pressDuration;
	static constexpr uint32_t kPressIdInvalid = -1;
};

struct FrameData {
	uint32_t id;
	bool isNew;
};
//#define TEST_MODE

static inline float mapAndConstrain(float x, float in_min, float in_max, float out_min, float out_max)
{
	float value = map(x, in_min, in_max, out_min, out_max);
	value = constrain(value, out_min, out_max);
	return value;
}

extern float gBrightness;
extern uint8_t gNewMode;
extern uint16_t gDebugFlags;
extern struct Override {
	uint32_t started;
	size_t ch;
	float out;
	bool bypassOutRange;
	bool isSize;
} gOverride;

#include <libraries/Trill/CentroidDetection.h>
extern CentroidDetectionScaled globalSlider;

extern int menu_setup(size_t page);
extern void menu_exit();
extern bool gModeWantsInteractionPreMenu;
extern bool gModeWantsMenuDelay;
extern bool gInPreMenu;

bool menu_isLocked();
void menu_setLocked(bool val);

#include "centroid.h"
static inline centroid_t touchOrNot(const centroid_t& touch)
{
	return touch.size ? touch : centroid_t{kNoOutput, kNoOutput};
}

class UiOrientation
{
public:
	void setMenuSwapped(bool v);
	void setTouchStripSwapped(bool v)
	{
		touchStrip = v;
	}

	bool menuSwapped() { return menu; }
	bool touchStripSwapped() { return touchStrip; }
	bool outputsSwapped() {
		return menu ^ touchStrip; // OMG using XOR
	}
private:
	bool touchStrip = false;
	bool menu = false;
};
extern UiOrientation uio;
