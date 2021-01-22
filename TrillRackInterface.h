#pragma once
#include <Bela.h>
#include <libraries/Scope/Scope.h>

bool tr_setup();
void tr_loop();
class TrillRackInterface
{
public:
	TrillRackInterface() {};
	TrillRackInterface(unsigned int anInCh, unsigned int anOut0Ch, unsigned int anOut1Ch);
	int setup(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1);
	float analogRead();
	void analogWrite(unsigned int channel, float val);
	void process(BelaContext* context);
	void scopeWrite(unsigned int channel, float val);
	void scopeWrite(float* val);
	enum {kScopeChannels = 4};
private:
	enum { nAnOut = 2 };
	float anIn;
	float anOut[nAnOut];
	unsigned int anInCh;
	unsigned int anOutCh[nAnOut];
	float scopeData[kScopeChannels];
	Scope scope;
	bool scopeInited;
};
