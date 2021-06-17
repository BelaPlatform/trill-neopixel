#include "TrillRackInterface.h"
#include <string.h>
#ifndef STM32
#include <Bela.h>
#endif // STM32

TrillRackInterface::TrillRackInterface(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1, unsigned int diInCh1, unsigned int diInCh2, unsigned int diOutCh)
{
	setup(anInCh, anOutCh0, anOutCh1, diInCh1, diInCh2, diOutCh);
}

int TrillRackInterface::setup(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1, unsigned int diInCh1, unsigned int diInCh2, unsigned int diOutCh)
{
	this->anInCh = anInCh;
	this->diInCh[0] = diInCh1;
	this->diInCh[1] = diInCh2;
	this->diOutCh = diOutCh;
	this->anOutCh[0] = anOutCh0;
	this->anOutCh[1] = anOutCh1;
	anIn = 0;
	diOut = 0;
	lastTimeMs =  0;
#ifdef USE_SCOPE
	scopeInited = false;
	memset(scopeData, 0, sizeof(scopeData));
#endif //USE_SCOPE
	memset(anOut, 0, sizeof(anOut));
	memset(diIn, 0, sizeof(diIn));
	return 0;
}

float TrillRackInterface::analogRead()
{
	return anIn;
}

float TrillRackInterface::digitalRead(unsigned int channel)
{
	if(channel < nDiIn)
		return diIn[channel];
	else
	  return 0;
}

void TrillRackInterface::digitalWrite(int val)
{	
	diOut = val;
}

void TrillRackInterface::analogWrite(unsigned int channel, float val)
{
	if(channel < nAnOut)
		anOut[channel] = val;
}

void TrillRackInterface::scopeWrite(float* data)
{
#ifdef USE_SCOPE
	memcpy(scopeData, data, sizeof(scopeData));
#endif // USE_SCOPE
}

void TrillRackInterface::scopeWrite(unsigned int channel, float val)
{
#ifdef USE_SCOPE
	if(channel < kScopeChannels)
		scopeData[channel] = val;
#endif // USE_SCOPE
}

double TrillRackInterface::getTimeMs() {
	return lastTimeMs;
}

void TrillRackInterface::process(BelaContext* context)
{
#ifdef USE_SCOPE
	lastTimeMs = context->audioFramesElapsed / context->audioSampleRate * 1000;
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
        for(unsigned int m = 0; m < nDiIn; ++m)
	        if(diInCh[m] < context->digitalChannels)
                diIn[m] = ::digitalRead(context, 0, diInCh[m]);
        if(diOutCh < context->digitalChannels)
    		    ::pinMode(context, 0, diOutCh, OUTPUT); // Set diOutCh as output
        		::digitalWrite(context, 0, diOutCh, diOut);
	for(unsigned int i = 0; i < nAnOut; ++i)
                if(anOutCh[i] < context->analogOutChannels)
                        ::analogWrite(context, 0, anOutCh[i], anOut[i]);
#endif // USE_SCOPE
}

