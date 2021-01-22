#include "TrillRackInterface.h"
#include <string.h>

TrillRackInterface::TrillRackInterface(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1)
{
	setup(anInCh, anOutCh0, anOutCh1);
}

int TrillRackInterface::setup(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1)
{
	scopeInited = false;
	this->anInCh = anInCh;
	this->anOutCh[0] = anOutCh0;
	this->anOutCh[1] = anOutCh1;
	anIn = 0;
	memset(scopeData, 0, sizeof(scopeData));
	memset(anOut, 0, sizeof(anOut));
	return 0;
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

void TrillRackInterface::scopeWrite(float* data)
{
	memcpy(scopeData, data, sizeof(scopeData));
}

void TrillRackInterface::scopeWrite(unsigned int channel, float val)
{
	if(channel < kScopeChannels)
		scopeData[channel] = val;
}
void TrillRackInterface::process(BelaContext* context)
{
	unsigned int scopeEvery = 4;
	if(!scopeInited) {
		// we initialise here so setup and constructor don't need context
		scope.setup(kScopeChannels, context->audioSampleRate / scopeEvery);
		scopeInited = true;
	}
	for(unsigned int n = 0; n < context->audioFrames / scopeEvery; ++n)
		scope.log(scopeData);
        if(anInCh < context->analogInChannels)
                anIn = ::analogRead(context, 0, anInCh);
	for(unsigned int i = 0; i < nAnOut; ++i)
                if(anOutCh[i] < context->analogOutChannels)
                        ::analogWrite(context, 0, anOutCh[i], anOut[i]);
}

