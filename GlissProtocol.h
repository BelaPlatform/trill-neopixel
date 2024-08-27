#pragma once

#include <stdint.h>
#include <stddef.h>
#include "rgb.h"
#include "../../common_stuff/sysex.h"

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
enum ProtocolCmdRecorderModeGesture {
	kGpRmgEndpoints = 0,
	kGpRmgContent = 1,
	kGpRmgPlayHead = 2,
	kGpRmgPlayRate = 3,
};

// I/O and processing
int gp_incoming(SysexPeripheral src, const void* data, size_t len);
int gp_outgoing(SysexPeripheral dst, int (*callback)(SysexPeripheral sp, const uint8_t* data, size_t maxLen));
void gp_processIncoming(); // call from the audio thread to process incoming messages
struct GpIoRange {
	uint8_t cvRange;
	uint16_t min;
	uint16_t max;
};
struct GpRmgEndpoints {
	size_t offset;
	size_t length;
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
void gp_RecorderMode_setGestureEndpoints(uint8_t recorder, const GpRmgEndpoints& endpoints);
void gp_RecorderMode_setGestureContent(uint8_t recorder, size_t offset, size_t length, const uint8_t* data);
void gp_recorderMode_setGesturePlayHead(uint8_t recorder, size_t offset);
void gp_recorderMode_setGesturePlayRate(uint8_t recorder, uint32_t rate);

GpRmgEndpoints gp_RecorderMode_getGestureEndpoints(uint8_t recorder);
int gp_RecorderMode_getGestureContent(uint8_t recorder, size_t offset, size_t length, uint8_t* data);
size_t gp_recorderMode_getGesturePlayHead(uint8_t recorder);
uint32_t gp_recorderMode_getGesturePlayRate(uint8_t recorder);
