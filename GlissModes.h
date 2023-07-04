#include <array>
constexpr size_t kNumPads = 26;
constexpr size_t kNumLeds = 23;
constexpr float kSliderBottomMargin = 0.05;
constexpr float kSliderTopMargin = 0.05;
typedef enum {
	kOutModeManualBlock,
	kOutModeManualSample,
	kOutModeManualSampleSmoothed,
} OutMode;
constexpr size_t kNumOutChannels = 2; // TODO: assert it's the same as context->analogOutChannels
extern std::array<OutMode,kNumOutChannels> gOutMode;

struct CalibrationData {
	static constexpr float kGnd = 0.3333333333f;
	static constexpr size_t kNumPoints = 3;
	static constexpr std::array<float,kNumPoints> points = {{0, kGnd, 1}};
	std::array<float,kNumPoints> values;
};
CalibrationData const& getCalibrationInput();
CalibrationData const& getCalibrationOutput();

typedef enum {
	kCvRangePositive10,
	kCvRangeBipolar,
	kCvRangePositive5,
	kCvRangeFull,
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
public:
	static IoRange init() {
		return IoRange {
			.min = 0,
			.max = 1,
			.range = kCvRangePositive10,
			.enabled = true,
		};
	}
	void getMinMax(float& min, float& max) const {
		constexpr float gnd = CalibrationData::kGnd;
		CvRange range = enabled ? this->range : kCvRangeFull;
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
				min = this->min;
				max = this->max;
				break;
		}
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

#include <Utilities.h>
static inline float mapAndConstrain(float x, float in_min, float in_max, float out_min, float out_max)
{
	float value = map(x, in_min, in_max, out_min, out_max);
	value = constrain(value, out_min, out_max);
	return value;
}

extern float gBrightness;
extern bool gJacksOnTop;
extern uint8_t gNewMode;
extern bool gOutAddsIn;
extern struct Override {
	uint32_t started;
	size_t ch;
	float out;
	bool bypassOutRange;
	bool isSize;
} gOverride;
extern std::array<bool,2> gOutIsSize;

#include <libraries/Trill/CentroidDetection.h>
extern CentroidDetectionScaled globalSlider;

extern int menu_setup(size_t page);
extern void menu_exit();
extern bool gModeWantsInteractionPreMenu;
extern bool gInPreMenu;

#include "centroid.h"
static inline centroid_t touchOrNot(const centroid_t& touch)
{
	return touch.size ? touch : centroid_t{kNoOutput, kNoOutput};
}
