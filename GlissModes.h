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

struct ButtonView {
	 bool onset;
	 bool offset;
	 bool pressed;
	 bool enabled;
	 uint32_t pressCount;
	 static constexpr uint32_t kPressCountInvalid = -1;
};
enum { kNumModes = 5 };
constexpr size_t kNumOutChannels = 2; // TODO: assert it's the same as context->analogOutChannels

#include <Utilities.h>
static inline float mapAndConstrain(float x, float in_min, float in_max, float out_min, float out_max)
{
	float value = map(x, in_min, in_max, out_min, out_max);
	value = constrain(value, out_min, out_max);
	return value;
}
