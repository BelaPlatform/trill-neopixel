#include <stdint.h>
#include <stddef.h>
#include "rgb.h"

enum ProtocolPeripheral {
	kGpMidi,
	kGpI2c,
	kGpNumPp,
};
enum ProtocolCmd {
	kGpMode = 0,
	kGpParameter = 1,
	kGpIoRange = 2,
	kGpModeColor = 3,
	kGpRecorderModeGesture = 4,
	kGpDebugFlags = 6,
	kGpStore = 7,
	kGpGet = 8,
	kGpGetResponse = 9,
};
// I/O and processing
int gp_incoming(ProtocolPeripheral src, const void* data, size_t len);
int gp_outgoing(ProtocolPeripheral dst, int (*callback)(const uint8_t* data, size_t maxLen));
void gp_processIncoming(); // call from the audio thread to process incoming messages
struct GpIoRange {
	uint8_t cvRange;
	uint16_t min;
	uint16_t max;
};

// global methods
void gp_store(); // store to disk
// gp_getCalibration()
// gp_setCalibration();
// gp_setTouch(); // override touch data
// gp_setRgb(); // set per-LED colors
// gp_setCvOut();
// gp_getCvIn();

// per-mode/pseudo-mode generic universal controls. Allow to set menu or non-menu `Parameter`s.
void gp_setMode(uint8_t mode);
void gp_setModeParameter(uint8_t mode, uint8_t parameter, uint16_t value);
void gp_setModeIoRange(uint8_t mode, uint8_t rangeIdx, const GpIoRange& ioRange);
void gp_setModeColor(uint8_t mode, uint8_t colorIdx, const rgb_t& color);
void gp_setDebugFlags(uint16_t flags);

uint8_t gp_getMode();
uint16_t gp_getModeParameter(uint8_t mode, uint8_t parameter);
GpIoRange gp_getModeIoRange(uint8_t mode, uint8_t rangeIdx);
rgb_t gp_getModeColor(uint8_t mode, uint8_t colorIdx);
uint16_t gp_getDebugFlags();

// mode specialties
void gp_RecorderMode_setGestureLength(uint8_t recorder, uint32_t length);
void gp_RecorderMode_setGestureContent(uint8_t recorder, size_t length, size_t offset, const uint8_t* data);

uint32_t gp_RecorderMode_getGestureLength(uint8_t recorder);
size_t gp_RecorderMode_getGestureContent(uint8_t recorder, size_t length, size_t offset, uint16_t* data);
