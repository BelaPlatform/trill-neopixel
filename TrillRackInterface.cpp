#include "TrillRackInterface.h"
#include <string.h>
#ifdef STM32
#include <stdint.h>
extern "C" { int32_t HAL_GetTick(void); };
#else // STM32
#include <Bela.h>
#endif // STM32

TrillRackInterface::TrillRackInterface(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1, unsigned int diInCh0, unsigned int diOutCh0)
{
	setup(anInCh, anOutCh0, anOutCh1, diInCh0, diOutCh0);
}

int TrillRackInterface::setup(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1, unsigned int diInCh0, unsigned int diOutCh0)
{
	this->anInCh = anInCh;
	this->diInCh = diInCh0;
	this->anOutCh[0] = anOutCh0;
	this->anOutCh[1] = anOutCh1;
	this->diOutCh = diOutCh0;
	firstRun = true;
	anIn = 0;
	ledOut = 0;
	lastTimeMs =  0;
#ifdef USE_SCOPE
	scopeInited = false;
	memset(scopeData, 0, sizeof(scopeData));
#endif //USE_SCOPE
	memset(anOut, 0, sizeof(anOut));
	return 0;
}

float TrillRackInterface::analogRead()
{
	return anIn;
}

float TrillRackInterface::digitalRead(unsigned int channel)
{
	if(channel < 1)
		return diIn;
	else
	  return 0;
}

void TrillRackInterface::buttonLedWrite(float val)
{	
	ledOut = val;
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

#include <stdio.h>
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
#ifdef STM32
	lastTimeMs = HAL_GetTick();
	if(firstRun)
	{
		//emulation of Bela's setup()
		firstRun = false;
		if(diInCh >= context->digitalChannels
				|| diOutCh >= context->digitalChannels
				|| anInCh >= context->analogInChannels
				|| anOutCh[0] >= context->analogOutChannels
				|| anOutCh[1] >= context->analogOutChannels
			)
		{
			printf("Invalid channels\n\r");
			return; //false
		}
		pinMode(context, 0, diInCh, INPUT);
		pinMode(context, 0, diOutCh, OUTPUT);
		return; //true
	}

	diIn = ::digitalRead(context, 0, diInCh);
	anIn = ::analogRead(context, 0, anInCh);
	tr_loop();
	enum { kLedPwmPeriod = 512 };
	for(size_t n = 0; n < context->digitalFrames; ++n)
	{
		::digitalWriteOnce(context, n, diOutCh, ledPwmIdx < ledOut * float(kLedPwmPeriod));
		ledPwmIdx++;
		if(kLedPwmPeriod == ledPwmIdx)
			ledPwmIdx = 0;
	}
	extern float gDacNext[2];
#if 1
	for(unsigned int n = 0; n < 2; ++n)
		gDacNext[n] = anOut[n];
#else
	static unsigned int count = 0;
	gDacNext[0] = count++ / 4096.f;
	gDacNext[1] = count / 4096.f;
	if(count >= 4096)
		count = 0;
#endif
#endif // STM32
}

