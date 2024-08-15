#pragma once
#include <vector>
#include <libraries/Trill/CentroidDetection.h>
#include "Stm32NeoPixel.h"
#include "rgb.h"
#include "centroid.h"

class LedSlider : public CentroidSettableScaled
{
public:
	static constexpr size_t kDefaultNumWeights = 2;
	typedef enum {
		MANUAL_CENTROIDS,
		MANUAL_RAW,
		MANUAL_DIRECT, // uses directBegin(), directWriteCentroid()
	} LedMode_t;
	struct Settings
	{
		unsigned int maxNumCentroids;
		float sizeScale;
		NeoPixel* np;
		unsigned int numLeds;
		unsigned int ledOffset;
	};
	LedSlider() {};
	LedSlider(const Settings& settings);
	int setup(const Settings& settings);
	void setColor(const rgb_t& color);
	void setLedMode(LedMode_t mode);
	void setLedsRaw(const float* values);
	void setLedsCentroids(const centroid_t* values, unsigned int length);
	void process(const float* locations, const float* sizes, unsigned int length);
	void enableTouch(bool enable);
	void enableLeds(bool enable);
	void directBegin();
	void directWriteCentroid(const centroid_t& centroid, rgb_t color, size_t numWeigths = kDefaultNumWeights);
	centroid_t& operator[](size_t i) {return ledCentroids[i];}
	static int writeCentroidToArray(const centroid_t& centroid, size_t numWeights, float* dest, size_t destSize);
private:
	void updateLeds();
	NeoPixel* np;
	unsigned int ledOffset;
	rgb_t color;
	LedMode_t mode;
	unsigned int maxNumPadCentroids;
	std::vector<float> ledValues;
	std::vector<centroid_t> ledCentroids;
	bool touchEnabled = true;
	bool ledsEnabled = true;
};

class LedSliders
{
public:
	struct delimiters_t {
		float sliderMin;
		float sliderMax;
		unsigned int firstLed;
		unsigned int lastLed;
	};
	struct Settings
	{
		float sizeScale;
		std::vector<delimiters_t> boundaries;
		std::vector<unsigned int> maxNumCentroids;
		NeoPixel* np;
	};

	LedSliders() {};
	LedSliders(const Settings& settings);
	int setup(const Settings& settings);
	void process(const centroid_t** centroids);
	void process(const CentroidDetectionScaled& globalSlider);
	std::vector<LedSlider> sliders;
	void enableTouch(bool enable);
	void enableLeds(bool enable);
	auto begin()
	{
		return sliders.begin();
	}
	auto end()
	{
		return sliders.end();
	}
	bool areLedsEnabled();
	bool isTouchEnabled();
	Settings s;
private:
	bool ledsEnabled = true;
	bool touchEnabled = true;
};
