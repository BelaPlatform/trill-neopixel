#include "LedSliders.h"
#include <string.h>

LedSlider::LedSlider(const Settings& settings)
{
	setup(settings);
}

int LedSlider::setup(const Settings& settings)
{
	if(CentroidDetection::setup(0 /* unused */, 5, settings.sizeScale))
		return -1;
	np = settings.np;
	ledOffset = settings.ledOffset;
	setLedMode(MANUAL_CENTROIDS);
	maxNumPadCentroids = settings.maxNumCentroids;
	ledValues.resize(settings.numLeds);
	ledCentroids.resize(maxNumPadCentroids);
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

void LedSlider::setLedsCentroids(const centroid_t* values, unsigned int length, bool replace)
{
	ledCentroids.resize(length);
	for(unsigned int n = 0; n < length && n < ledCentroids.size(); ++n)
		ledCentroids[n] = values[n];
	updateLeds(replace);
}

void LedSlider::process(const float* newLocations, const float* newSizes, unsigned int length, bool replace)
{
	if(touchEnabled)
	{
		CentroidSettableScaled::set(newLocations, newSizes, length);
	}
	updateLeds(replace);
}

void LedSlider::enableTouch(bool enable)
{
	touchEnabled = enable;
}

void LedSlider::enableLeds(bool enable)
{
	ledsEnabled = enable;
}

void LedSlider::directBegin(bool clear)
{
	if(!ledsEnabled)
		return;
	if(clear)
		np->clear();
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

static uint8_t clipLed(float val)
{
	if(val > 255)
		val = 255;
	return val + 0.5f;
}

void LedSlider::directWriteCentroid(const centroid_t& centroid, rgb_t color, size_t numWeights)
{
	if(!ledsEnabled)
		return;
	size_t numLeds = np->getNumPixels();
	float leds[numLeds];
	memset(leds, 0, sizeof(leds[0]) * numLeds);
	writeCentroidToArray(centroid, numWeights, leds, numLeds);
	for(size_t n = 0; n < np->getNumPixels(); ++n)
	{
		std::array<float,Stm32NeoPixel::kNumBytesPerPixel> pixel;
		for(size_t c = 0; c < pixel.size(); ++c)
		{
			pixel[c] = np->getPixelChannel(n, c);
			pixel[c] += color[c] * leds[n];
		}
		np->setPixelColor(n, clipLed(pixel[0]), clipLed(pixel[1]), clipLed(pixel[2]));
	}
}

int LedSlider::writeCentroidToArray(const centroid_t& centroid, size_t numWeights, float* dest, size_t destSize)
{
	float size = centroid.size;
	float idx = centroid.location;
	if(idx > 1 || idx < 0)
		return -1;
	idx *= destSize;
	if(size > 1)
		size = 1;
	if(size <= 0)
		return -1;
	float weights[numWeights];
	int idx0 = int(idx - numWeights / 2) + 1;
	idxToWeights(std::max(idx - idx0, 0.f), weights, numWeights);
	for(unsigned int i = 0; i < numWeights; ++i)
	{
		int ii = i + idx0 - 1;
		ii = ii >= 0 ? ii : 0;
		ii = ii < int(destSize) ? ii : int(destSize) - 1;
		dest[ii] += weights[i] * size;
	}
	return 0;
}

void LedSlider::updateLeds(bool replace)
{
	if(MANUAL_DIRECT == mode)
		return;
	if(!ledsEnabled)
		return;
	if(!ledValues.size())
		return;
	if(MANUAL_CENTROIDS == mode)
	{
		memset(ledValues.data(), 0, sizeof(ledValues[0]) * ledValues.size());
		for(unsigned int n = 0; n < ledCentroids.size(); ++n)
		{
			writeCentroidToArray(ledCentroids[n], kDefaultNumWeights, ledValues.data(), ledValues.size());
		}
		//for(unsigned int n = 0; n < ledValues.size(); ++n)
			//printf("[%d]: %f ", n, ledValues[n]);
		//printf("\n\n");
	}
	// MANUAL_RAW will have set ledValues elsewhere
	// so at this point ledValues is all set and we use it to set the LEDs
	for(unsigned int n = 0; n < ledValues.size(); ++n)
	{
		size_t idx = ledOffset + n;
		rgb_t base { 0, 0, 0};
		if(!replace)
		{
			for(size_t c = 0; c < base.size(); ++c)
				base[c] = np->getPixelChannel(idx, c);
		}
		np->setPixelColor(idx, clipLed(base.r + color.r * ledValues[n]),
				clipLed(base.g + color.g * ledValues[n]), clipLed(base.b + color.b * ledValues[n]));
	}
}

LedSliders::LedSliders(const Settings& settings)
{
	setup(settings);
}

int LedSliders::setup(const Settings& settings)
{
	s = settings;
	sliders.clear();
	sliders.resize(s.boundaries.size());
	for(unsigned int n = 0; n < s.boundaries.size(); ++n)
	{
		auto& b = s.boundaries[n];
		unsigned int maxNumCentroids = 1;
		if(settings.maxNumCentroids.size() > n)
			maxNumCentroids = settings.maxNumCentroids[n];
		sliders[n].setup({
			.maxNumCentroids = maxNumCentroids,
			.sizeScale = s.sizeScale,
			.np = s.np,
			.numLeds = b.lastLed - b.firstLed,
			.ledOffset = b.firstLed,
		});
		sliders[n].setUsableRange(b.sliderMin, b.sliderMax);
	}
	return 0;
}

template <typename T, typename U>
static void sort(T* out, U* in, unsigned int* order, unsigned int size)
{
	for(unsigned int n = 0; n < size; ++n)
		out[n] = in[order[n]];
}

// takes a nullptr-separated list of centroids
void LedSliders::process(const centroid_t** centroids)
{
	if(ledsEnabled)
	{
		//TODO: only clear unused LEDs
		s.np->clear();
	}
	centroid_t const** cen = centroids;
	for(unsigned int n = 0; n < sliders.size(); ++n)
	{
		size_t start = cen - centroids;
		size_t count = 0;
		while(*cen++) // look for next empty one
			count++;
		float locations[count];
		float sizes[count];
		for(size_t c = 0; c < count; ++c)
		{
			locations[c] = centroids[start + c]->location;
			sizes[c] = centroids[start + c]->size;
		}
		sliders[n].process(locations, sizes, count);
	}
}

void LedSliders::process(const CentroidDetectionScaled& globalSlider)
{
	if(ledsEnabled)
	{
		//TODO: only clear unused LEDs
		s.np->clear();
	}
	static constexpr size_t kMaxTouches = 5;
	for(unsigned int n = 0; n < sliders.size(); ++n)
	{
		float locations[kMaxTouches];
		float sizes[kMaxTouches];
		size_t count = 0;
		float min = s.boundaries[n].sliderMin;
		float max = s.boundaries[n].sliderMax;
		// find all relevant touches
		for(unsigned int c = 0; c < globalSlider.getNumTouches() && c < kMaxTouches; ++c)
		{
			float location = globalSlider.touchLocation(c);
			float size = globalSlider.touchSize(c);
			if(location >= min && location < max && size)
			{
				locations[count] = location;
				sizes[count] = size;
				count++;
			}
		}
		sliders[n].process(locations, sizes, count);
	}
}

void LedSliders::enableTouch(bool enable)
{
	touchEnabled = enable;
	for(unsigned int n = 0; n < sliders.size(); ++n)
		sliders[n].enableTouch(enable);
}

void LedSliders::enableLeds(bool enable)
{
	ledsEnabled = enable;
	for(unsigned int n = 0; n < sliders.size(); ++n)
		sliders[n].enableLeds(enable);
}

bool LedSliders::areLedsEnabled()
{
	return ledsEnabled;
}

bool LedSliders::isTouchEnabled()
{
	return touchEnabled;
}
