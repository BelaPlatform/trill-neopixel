#pragma once
#include <Bela.h>

bool tr_setup();
void tr_loop();
class TrillRackInterface
{
public:
	TrillRackInterface(unsigned int anInCh, unsigned int anOut0Ch, unsigned int anOut1Ch);
	float analogRead();
	void analogWrite(unsigned int channel, float val);
	void process(BelaContext* context);
private:
	enum { nAnOut = 2 };
	float anIn;
	float anOut[nAnOut];
	unsigned int anInCh;
	unsigned int anOutCh[nAnOut];
};
