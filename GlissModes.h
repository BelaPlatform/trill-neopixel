typedef enum {
	kOutModeFollowTouch,
	kOutModeFollowLeds,
	kOutModeManualBlock,
	kOutModeManualSample,
} OutMode;

typedef enum {
	kOutRangeFull,
	kOutRangeBipolar,
	kOutRangePositive5,
	kOutRangePositive10,
	kOutRangeNum,
} OutRange;

enum { kNumModes = 5 };
constexpr size_t kNumOutChannels = 2; // TODO: assert it's the same as context->analogOutChannels
