#pragma once
#ifdef STM32
#define BelaContext void
#include <stdint.h>
#include <stddef.h>
#else // STM32
#define USE_SCOPE
#include <libraries/Scope/Scope.h>
#endif // STM32

extern "C" {
int tr_setup();
void tr_loop();
void tr_newData(const uint8_t* newData, size_t len);
void tr_process(void* ptr);
void tr_snpDone(void);
}
class TrillRackInterface
{
public:
	TrillRackInterface() {};
	TrillRackInterface(unsigned int anInCh, unsigned int anOut0Ch, unsigned int anOut1Ch, unsigned int diInCh1, unsigned int diInCh2, unsigned int diOutCh);
	int setup(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1, unsigned int diInCh1, unsigned int diInCh2, unsigned int diOutCh);
	float analogRead();
	float digitalRead(unsigned int channel);
	void digitalWrite(int val);
	void analogWrite(unsigned int channel, float val);
	void process(BelaContext* context);
	void scopeWrite(unsigned int channel, float val);
	void scopeWrite(float* val);
	enum {kScopeChannels = 4};
	double getTimeMs();
private:
	enum { nAnOut = 2 };
	enum { nDiIn = 2 };
	float anIn;
	float diIn[2];
	int diOut;
	float anOut[nAnOut];
	double lastTimeMs;
	unsigned int anInCh;
	unsigned int diInCh[2];
	unsigned int diOutCh;
	unsigned int anOutCh[nAnOut];
#ifdef USE_SCOPE
	float scopeData[kScopeChannels];
	Scope scope;
	bool scopeInited;
#endif // USE_SCOPE
};
