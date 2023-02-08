#pragma once
#include <Bela.h>
#ifdef STM32
#include <stdint.h>
#include <stddef.h>
#else // STM32
#define USE_SCOPE
#include <libraries/Scope/Scope.h>
#endif // STM32

extern "C" {
int tr_setup();
void tr_mainLoop();
void tr_render(BelaContext* context);
void tr_newData(const uint8_t* newData, size_t len);
void tr_process(BelaContext* ptr);
void tr_requestScan(int);
int tr_scanRequested(void);
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
private:
	enum { nAnOut = 2 };
	enum { nDigOut = 2 };
	float anIn;
	float diIn;
	float ledOut[nDigOut];
	unsigned int ledPwmIdx = 0;
	float anOut[nAnOut];
	double lastTimeMs;
	unsigned int anInCh;
	unsigned int diInCh;
	unsigned int anOutCh[nAnOut];
	unsigned int diOutCh[nDigOut];
	bool firstRun;
#ifdef USE_SCOPE
	float scopeData[kScopeChannels];
	Scope scope;
	bool scopeInited;
#endif // USE_SCOPE
};
