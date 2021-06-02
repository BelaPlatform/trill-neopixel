#pragma once
#ifdef STM32
#define BelaContext void
#else // STM32
#define USE_SCOPE
#include <libraries/Scope/Scope.h>
#endif // STM32

bool tr_setup();
void tr_loop();
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
