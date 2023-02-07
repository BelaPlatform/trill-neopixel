#pragma once
#include <vector>
#include <libraries/Trill/CentroidDetection.h>
#include "Stm32NeoPixel.h"

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} rgb_t;

class LedSlider : public CentroidDetection
{
public:
	struct centroid_t {
		float location;
		float size;
	};
	typedef enum {
		AUTO_CENTROIDS,
		AUTO_RAW,
		MANUAL_CENTROIDS,
		MANUAL_RAW,
	} LedMode_t;
	struct Settings
	{
		unsigned int numPads;
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
	void process(const float* rawData);
	void enableTouch(bool enable);
	void enableLeds(bool enable);
	centroid_t& operator[](size_t i) {return ledCentroids[i];}
private:
	void updateLeds();
	NeoPixel* np;
	unsigned int ledOffset;
	rgb_t color;
	LedMode_t mode;
	unsigned int maxNumPadCentroids;
	std::vector<float> ledValues;
	std::vector<centroid_t> ledCentroids;
	std::vector<float> scratch;
	bool touchEnabled = true;
	bool ledsEnabled = true;
};

class LedSliders
{
public:
	struct delimiters_t {
		unsigned int firstPad;
		unsigned int lastPad;
		unsigned int firstLed;
		unsigned int lastLed;
	};
	struct Settings
	{
		std::vector<unsigned int> order;
		float sizeScale;
		std::vector<delimiters_t> boundaries;
		std::vector<unsigned int> maxNumCentroids;
		NeoPixel* np;
	};

	LedSliders() {};
	LedSliders(const Settings& settings);
	int setup(const Settings& settings);
	void process(const float* rawData);
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
private:
	std::vector<float> pads;
	Settings s;
	bool ledsEnabled = true;
};
