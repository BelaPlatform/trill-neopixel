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
