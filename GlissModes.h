#include <array>
constexpr size_t kNumPads = 26;
constexpr size_t kNumLeds = 23;
constexpr float kSliderBottomMargin = 0.05;
constexpr float kSliderTopMargin = 0.05;
typedef enum {
	kOutModeFollowTouch,
	kOutModeFollowLeds,
	kOutModeManualBlock,
	kOutModeManualSample,
} OutMode;

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
	CvRange range;
	float min;
	float max;
	// this can be changed dynamically by a mode and is not stored
	bool enabled;
};
extern IoRange gInRange;
extern IoRange gOutRangeTop;
extern IoRange gOutRangeBottom;

const float kNoOutput = -12345.6;

struct ButtonView {
	bool onset;
	bool offset;
	bool pressed;
	bool enabled;
	uint32_t pressCount;
	static constexpr uint32_t kPressCountInvalid = -1;
};

struct CalibrationData {
	static constexpr float kGnd = 0.3333333333f;
	static constexpr size_t kNumPoints = 3;
	static constexpr std::array<float,kNumPoints> points = {{0, kGnd, 1}};
	std::array<float,kNumPoints> values;
};
CalibrationData const& getCalibrationInput();
CalibrationData const& getCalibrationOutput();

//#define TEST_MODE
#ifdef TEST_MODE
constexpr size_t kNumModes = 7;
#else // TEST_MODE
constexpr size_t kNumModes = 6;
#endif // TEST_MODE

constexpr size_t kNumOutChannels = 2; // TODO: assert it's the same as context->analogOutChannels

#include <Utilities.h>
static inline float mapAndConstrain(float x, float in_min, float in_max, float out_min, float out_max)
{
	float value = map(x, in_min, in_max, out_min, out_max);
	value = constrain(value, out_min, out_max);
	return value;
}

extern bool gJacksOnTop;
extern uint8_t gNewMode;
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
