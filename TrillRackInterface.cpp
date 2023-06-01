#include "TrillRackInterface.h"
#include <string.h>
#ifdef STM32
#include <stdint.h>
extern "C" { int32_t HAL_GetTick(void); };
#else // STM32
#include <Bela.h>
#endif // STM32

TrillRackInterface::TrillRackInterface(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1, unsigned int diInCh0, unsigned int diOutCh0, unsigned int diOutCh1)
{
	setup(anInCh, anOutCh0, anOutCh1, diInCh0, diOutCh0, diOutCh1);
}

int TrillRackInterface::setup(unsigned int anInCh, unsigned int anOutCh0, unsigned int anOutCh1, unsigned int diInCh0, unsigned int diOutCh0, unsigned int diOutCh1)
{
	this->anInCh = anInCh;
	this->diInCh = diInCh0;
	this->anOutCh[0] = anOutCh0;
	this->anOutCh[1] = anOutCh1;
	this->diOutCh[0] = diOutCh0;
	this->diOutCh[1] = diOutCh1;
	debounceCounter = 0;
	firstRun = true;
	anIn = 0;
	for(unsigned int c = 0; c < nDigOut; ++c)
		ledOut[c] = 0;
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

void TrillRackInterface::buttonLedWrite(unsigned int ch, float val)
{	
	if(ch < nDigOut)
		ledOut[ch] = val;
}

void TrillRackInterface::buttonLedSet(ButtonLedStyle style, ButtonLedColor color, float intensity, float durationMs)
{
	for(size_t n = 0; n < kNumButtonColors; ++n)
	{
		if(int(color) == n || color == kAll)
			buttonLedColorTimeouts[n] = { style, intensity, durationMs };
	}
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
				|| diOutCh[0] >= context->digitalChannels
				|| diOutCh[1] >= context->digitalChannels
				|| anInCh >= context->analogInChannels
				|| anOutCh[0] >= context->analogOutChannels
				|| anOutCh[1] >= context->analogOutChannels
			)
		{
			printf("Invalid channels\n\r");
			return; //false
		}
		pinMode(context, 0, diInCh, INPUT);
		for(unsigned int c = 0; c < nDigOut; ++c)
			pinMode(context, 0, c, OUTPUT);
		return; //true
	}
	for(size_t n = 0; n < context->digitalFrames; ++n)
	{
		bool currentIn = ::digitalRead(context, 0, diInCh);
		if(debounceCounter)
		{
			debounceCounter--;
		} else {
			if(currentIn != diIn)
				debounceCounter = 0.01f * context->digitalSampleRate; // ms debounce
			diIn = currentIn;
		}
	}
	anIn = ::analogRead(context, 0, anInCh);
	tr_render(context);
	// process color events
	float blockMs = context->analogFrames / context->analogSampleRate * 1000.f;
	ledOut[0] = 0;
	ledOut[1] = 0;
	for(size_t n = 0; n < kNumButtonColors; ++n)
	{
		LedColorsTimeout& lct = buttonLedColorTimeouts[n];
		float intensity = lct.intensity * (lct.ms > 0) * (lct.style != kOff);
		// map colors to out channels with appropriate intensity adjustments
		// the latter would override the others
		if(intensity)
		{
			switch(n)
			{
			case kR:
				ledOut[1] = lct.intensity;
				break;
			case kG:
				ledOut[0] = lct.intensity;
				break;
			case kY:
				ledOut[0] = lct.intensity * 0.15f;
				ledOut[1] = lct.intensity;
			}
		}
		if(lct.ms)
		{
			lct.ms -= blockMs;
			if(lct.ms < 0)
				lct.ms = 0;
		}
	}
	enum { kLedPwmPeriod = 512 };
	for(size_t n = 0; n < context->digitalFrames; ++n)
	{
		size_t activeC = ledPwmIdx & 1;
		for(size_t c = 0; c < nDigOut; ++c)
		{
			bool isActive;
			if(c == activeC)
				isActive = ledPwmIdx < ledOut[activeC] *  float(kLedPwmPeriod);
			else
				isActive = false;
			::digitalWriteOnce(context, n, diOutCh[c], isActive);
		}
		ledPwmIdx++;
		if(kLedPwmPeriod == ledPwmIdx)
			ledPwmIdx = 0;
	}
#if 0
	static unsigned int count = 0;
	gDacNext[0] = count++ / 4096.f;
	gDacNext[1] = count / 4096.f;
	if(count >= 4096)
		count = 0;
#endif
#endif // STM32
}

