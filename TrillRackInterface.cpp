#include "TrillRackInterface.h"

TrillRackInterface::TrillRackInterface(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1)
{
	this->anInCh = anInCh;
	this->anOutCh[0] = anOutCh0;
	this->anOutCh[1] = anOutCh1;
	anIn = 0;
	for(unsigned int n = 0; n < nAnOut; ++n)
		anOut[n] = 0;
}

float TrillRackInterface::analogRead()
{
	return anIn;
}

void TrillRackInterface::analogWrite(unsigned int channel, float val)
{
	if(channel < nAnOut)
		anOut[channel] = val;
}

void TrillRackInterface::process(BelaContext* context)
{
        if(anInCh < context->analogInChannels)
                anIn = ::analogRead(context, 0, anInCh);
	for(unsigned int i = 0; i < nAnOut; ++i)
                if(anOutCh[i] < context->analogOutChannels)
                        ::analogWrite(context, 0, anOutCh[i], anOut[i]);
}

