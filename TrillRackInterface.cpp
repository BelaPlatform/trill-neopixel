#include "TrillRackInterface.h"
#include <string.h>

TrillRackInterface::TrillRackInterface(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1, unsigned int diInCh, unsigned int diOutCh)
{
	setup(anInCh, anOutCh0, anOutCh1, diInCh, diOutCh);
}

int TrillRackInterface::setup(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1, unsigned int diInCh, unsigned int diOutCh)
{
	scopeInited = false;
	this->anInCh = anInCh;
	this->diInCh = diInCh;
	this->diOutCh = diOutCh;
	this->anOutCh[0] = anOutCh0;
	this->anOutCh[1] = anOutCh1;
	anIn = 0;
	diIn = 0;
	diOut = 0;
	lastTimeMs =  0;
	memset(scopeData, 0, sizeof(scopeData));
	memset(anOut, 0, sizeof(anOut));
	return 0;
}

float TrillRackInterface::analogRead()
{
	return anIn;
}

float TrillRackInterface::digitalRead()
{
	return diIn;
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
	memcpy(scopeData, data, sizeof(scopeData));
}

void TrillRackInterface::scopeWrite(unsigned int channel, float val)
{
	if(channel < kScopeChannels)
		scopeData[channel] = val;
}

double TrillRackInterface::getTimeMs() {
	return lastTimeMs;
}

void TrillRackInterface::process(BelaContext* context)
{
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
        if(diInCh < context->digitalChannels)
                diIn = ::digitalRead(context, 0, diInCh);
        if(diOutCh < context->digitalChannels)
    		    ::pinMode(context, 0, diOutCh, OUTPUT); // Set diOutCh as output
        		::digitalWrite(context, 0, diOutCh, diOut);
	for(unsigned int i = 0; i < nAnOut; ++i)
                if(anOutCh[i] < context->analogOutChannels)
                        ::analogWrite(context, 0, anOutCh[i], anOut[i]);
}

