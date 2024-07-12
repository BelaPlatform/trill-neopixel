#include <stdint.h>
#include <stddef.h>
enum ProtocolPeripheral {
	kGpMidi,
	kGpI2c,
};
enum ProtocolCmd {
	kGpMode = 0,
	kGpParameter = 1,
	kGpIoRange = 2,
	kGpModeColor = 3,
	kGpMenuColor = 4,
//	kGpRecorderModeGesture = 5,
	kGpDebugFlags = 6,
};
// I/O and processing
int gp_incoming(ProtocolPeripheral src, const void* data, size_t len);
int gp_outgoing(ProtocolPeripheral dst, void* data, size_t maxLen);
void gp_processIncoming(); // call from the audio thread to process incoming messages
void gp_store(); // store to disk

// global methods
void gp_setMenuColor(uint8_t idx, uint8_t r, uint8_t g, uint8_t b);
// gp_getCalibration()
// gp_setCalibration();
// gp_setTouch(); // override touch data
// gp_setRgb(); // set per-LED colors
// gp_setCvOut();
// gp_getCvIn();

// per-mode generic universal controls. Allow to set menu or non-menu `Parameter`s.
void gp_setMode(uint8_t mode);
void gp_setModeParameter(uint8_t mode, uint8_t parameter, uint16_t value);
void gp_setModeIoRange(uint8_t mode, uint8_t rangeIdx, uint8_t cvRange, uint16_t min, uint16_t max);
void gp_setModeColor(uint8_t mode, uint8_t colorIdx, uint8_t r, uint8_t g, uint8_t b); // TODO: store colors per-mode so they can be overwritten at runtime, also store them to disk

// mode specialties
void gp_RecorderMode_setGesture(uint8_t, ...); // TODO: set as array?
