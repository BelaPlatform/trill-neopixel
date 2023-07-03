#pragma once
#include <Bela.h>
#ifdef STM32
#include <stdint.h>
#include <stddef.h>
#include <array>
#else // STM32
#define USE_SCOPE
#include <libraries/Scope/Scope.h>
#endif // STM32

extern "C" {
int tr_setup();
void tr_mainLoop();
void tr_clearLeds();
void tr_render(BelaContext* context);
void tr_newData(const uint8_t* newData, size_t len);
void tr_process(BelaContext* ptr);
void tr_requestScan(int);
int tr_scanRequested(void);
void tr_requestUpdateLeds(int);
int tr_ledsUpdateRequested(void);
void tr_snpDone(void);
}
class TrillRackInterface
{
public:
	TrillRackInterface() {};
	TrillRackInterface(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1, unsigned int diInCh, unsigned int diOutCh0, unsigned int diOutCh1);
	int setup(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1, unsigned int diInCh, unsigned int diOutCh0, unsigned int diOutCh1);
	float analogRead();
	float digitalRead(unsigned int channel);
	void buttonLedWrite(unsigned int ch, float val);
	void analogWrite(unsigned int channel, float val);
	void process(BelaContext* context);
	void scopeWrite(unsigned int channel, float val);
	void scopeWrite(float* val);
	enum {kScopeChannels = 4};
	double getTimeMs();
	enum ButtonLedStyle {
		kSolid,
		kGlow,
		kOff,
	};
	enum ButtonLedColor {
		kR,
		kG,
		kY,
		kNumButtonColors,
		kAll,
	};
	void buttonLedSet(ButtonLedStyle style, ButtonLedColor color, float intensity = 1, float durationMs = 10); // 10 allows it to be refreshed  every so often but not forgotten
private:
	enum { nAnOut = 2 };
	enum { nDigOut = 2 };
	float anIn;
	float diIn;
	float ledOut[nDigOut];
	struct LedColorsTimeout {
		ButtonLedStyle style;
		float intensity;
		float ms;
		float phase;
	};
	std::array<LedColorsTimeout,kNumButtonColors> buttonLedColorTimeouts {};
	uint8_t ledPwmIdx = 0;
	float anOut[nAnOut];
	double lastTimeMs;
	unsigned int anInCh;
	unsigned int diInCh;
	unsigned int anOutCh[nAnOut];
	unsigned int diOutCh[nDigOut];
	size_t debounceCounter {};
	bool firstRun;
#ifdef USE_SCOPE
	float scopeData[kScopeChannels];
	Scope scope;
	bool scopeInited;
#endif // USE_SCOPE
};
