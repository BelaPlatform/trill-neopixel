#include "LedSliders.h"
#include <string.h>

LedSlider::LedSlider(const Settings& settings)
{
	setup(settings);
}

int LedSlider::setup(const Settings& settings)
{
	if(CentroidDetection::setup(settings.numPads, settings.maxNumCentroids, settings.sizeScale))
		return -1;
	np = settings.np;
	ledOffset = settings.ledOffset;
	setLedMode(AUTO_CENTROIDS);
	maxNumPadCentroids = settings.maxNumCentroids;
	ledValues.resize(settings.numLeds);
	ledCentroids.resize(maxNumPadCentroids);
	scratch.resize(settings.numPads);
	return 0;
}

void LedSlider::setColor(const rgb_t& color)
{
	this->color = color;
}

void LedSlider::setLedMode(LedMode_t mode)
{
	this->mode = mode;
}

void LedSlider::setLedsRaw(const float* values)
{
	memcpy(ledValues.data(), values, ledValues.size() * ledValues[0]);
}

void LedSlider::setLedsCentroids(const centroid_t* values, unsigned int length)
{
	for(unsigned int n = 0; n < length && n < ledCentroids.size(); ++n)
		ledCentroids[n] = values[n];
	updateLeds();
}

void LedSlider::process(const float* rawData)
{
	CentroidDetection::process(rawData);
	if(AUTO_RAW == mode)
		memcpy(scratch.data(), rawData, sizeof(rawData[0]) * scratch.size());
	else if(AUTO_CENTROIDS == mode)
	{
		ledCentroids.resize(maxNumPadCentroids);
		unsigned int n;
		for(n = 0; n < getNumTouches(); ++n)
		{
			if(1 == maxNumPadCentroids)
			{
				ledCentroids[n].size = compoundTouchSize();
				ledCentroids[n].location = compoundTouchLocation();
			}
			else
			{
				ledCentroids[n].size = touchSize(n);
				ledCentroids[n].location = touchLocation(n);
			}
		}
		for( ; n < ledCentroids.size(); ++n)
		{
			ledCentroids[n].size = 0;
			ledCentroids[n].location = 0;
		}
	}
	updateLeds();
}

extern void resample(float* out, unsigned int nOut, float* in, unsigned int nIn);
void LedSlider::updateLeds()
{
	if(AUTO_CENTROIDS == mode || MANUAL_CENTROIDS == mode)
	{
		//memset(scratch.data(), 0, sizeof(scratch[0]) * scratch.size());
		memset(ledValues.data(), 0, sizeof(ledValues[0]) * ledValues.size());
		for(unsigned int n = 0; n < ledCentroids.size(); ++n)
		{
			float idx = ledCentroids[n].location;
			if(idx > 1 || idx < 0)
				continue;
			idx *= ledValues.size();
			float size = ledCentroids[n].size;
			if(size > 1)
				size = 1;
			//printf("idx: %f, siz; %f\n", idx, size);
			unsigned int idx0 = (unsigned int)(idx);
			float frac = idx - idx0;
			unsigned int idx1 = idx0 + 1;
			if(idx1 >= ledValues.size())
				idx1 = idx0;
			// attempt at kindof interpolated write
			ledValues[idx0] += (1.f - frac) * size;
			ledValues[idx1] += frac * size;
		}
		//for(unsigned int n = 0; n < ledValues.size(); ++n)
			//printf("[%d]: %f ", n, ledValues[n]);
		//printf("\n\n");
	}
	if(AUTO_RAW == mode)
		resample(ledValues.data(), ledValues.size(), scratch.data(), scratch.size());
	// MANUAL_RAW will have set ledValues elsewhere
	// so at this point ledValues is all set and we use it to set the LEDs
	for(unsigned int n = 0; n < ledValues.size(); ++n)
		np->setPixelColor(ledOffset + n, color.r * ledValues[n],
				color.g * ledValues[n], color.b * ledValues[n]);
}

LedSliders::LedSliders(const Settings& settings)
{
	setup(settings);
}

int LedSliders::setup(const Settings& settings)
{
	s = settings;
	pads.resize(s.order.size());
	sliders.clear();
	sliders.resize(s.boundaries.size());
	for(unsigned int n = 0; n < s.boundaries.size(); ++n)
	{
		auto& b = s.boundaries[n];
		unsigned int maxNumCentroids = 1;
		if(settings.maxNumCentroids.size() > n)
			maxNumCentroids = settings.maxNumCentroids[n];
		sliders[n].setup({
			.numPads = b.lastPad - b.firstPad,
			.maxNumCentroids = maxNumCentroids,
			.sizeScale = s.sizeScale,
			.np = s.np,
			.numLeds = b.lastLed - b.firstLed,
			.ledOffset = b.firstLed,
		});
	}
	return 0;
}

template <typename T, typename U>
static void sort(T* out, U* in, unsigned int* order, unsigned int size)
{
	for(unsigned int n = 0; n < size; ++n)
		out[n] = in[order[n]];
}

void LedSliders::process(const float* rawData)
{
	sort(pads.data(), rawData, s.order.data(), s.order.size());
	for(unsigned int n = 0; n < sliders.size(); ++n)
		sliders[n].process(pads.data() + s.boundaries[n].firstPad);
}
