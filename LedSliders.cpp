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
	memcpy(ledValues.data(), values, ledValues.size() * sizeof(ledValues[0]));
}

void LedSlider::setLedsCentroids(const centroid_t* values, unsigned int length)
{
	ledCentroids.resize(length);
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

// numWeights should be even.
static void idxToWeights(float idx, float* weights, unsigned int numWeights)
{
	// triangular peak
	// P0{x0, y0} is the peak
	float x0 = idx;
	float span = (numWeights / 2) * 2;
	float y0 = 1;
	// P1{x1, y1} is the left zero
	float x1 = x0 - span * 0.5f;
	float y1 = 0;
	// P2{x2, y2} is the right zero
	float x2 = x0 + span * 0.5f;
	float y2 = 0;
	// line through P0, P1:
	float m1 = (y0 - y1) / (x0 - x1);
	float q1 = y0 - m1 * x0;
	// line through P0, P2:
	float m2 = (y0 - y2) / (x0 - x2);
	float q2 = y0 - m2 * x0;
	for(unsigned int n = 0; n < numWeights; ++n)
	{
		float y;
		float x = n;
		if(x < x0)
			y = m1 * x + q1;
		else
			y = m2 * x + q2;
		weights[n] = y >= 0 ? y : 0;
	}
}

void LedSlider::updateLeds()
{
	if(!ledValues.size())
		return;
	if(AUTO_CENTROIDS == mode || MANUAL_CENTROIDS == mode)
	{
		//memset(scratch.data(), 0, sizeof(scratch[0]) * scratch.size());
		memset(ledValues.data(), 0, sizeof(ledValues[0]) * ledValues.size());
		for(unsigned int n = 0; n < ledCentroids.size(); ++n)
		{
			float size = ledCentroids[n].size;
			float idx = ledCentroids[n].location;
			if(idx > 1 || idx < 0)
				continue;
			idx *= ledValues.size();
			if(size > 1)
				size = 1;
			if(size <= 0)
				continue;
			unsigned int numWeights = 4;
			float weights[numWeights];
			int idx0 = int(idx - numWeights / 2) + 1;
			idxToWeights(idx - idx0, weights, numWeights);
			for(unsigned int i = 0; i < numWeights; ++i)
			{
				int ii = i + idx0 - 1;
				ii = ii >= 0 ? ii : 0;
				ii = ii < int(ledValues.size()) ? ii : int(ledValues.size()) - 1;
				ledValues[ii] += weights[i] * size;
			}
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
