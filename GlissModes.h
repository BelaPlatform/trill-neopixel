constexpr size_t kNumLeds = 23;
typedef enum {
	kOutModeFollowTouch,
	kOutModeFollowLeds,
	kOutModeManualBlock,
	kOutModeManualSample,
} OutMode;

typedef enum {
	kCvRangeFull,
	kCvRangeBipolar,
	kCvRangePositive5,
	kCvRangePositive10,
	kCvRangeCustom,
	kCvRangeNum,
} CvRange;
const float kNoOutput = -12345.6;

struct ButtonView {
	 bool onset;
	 bool offset;
	 bool pressed;
	 bool enabled;
	 uint32_t pressCount;
	 static constexpr uint32_t kPressCountInvalid = -1;
};
//#define TEST_MODE
#ifdef TEST_MODE
constexpr size_t kNumModes = 6;
#else // TEST_MODE
constexpr size_t kNumModes = 5;
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
