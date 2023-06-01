#pragma once
#include <vector>
#include <libraries/Trill/CentroidDetection.h>
#include "Stm32NeoPixel.h"

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t& operator[](size_t n) {
		switch(n)
		{
		default:
		case 0:
			return r;
		case 1:
			return g;
		case 2:
			return b;
		}
	}
	// TODO: how do you refactor this and the above to share code?
	uint8_t operator[](size_t n) const {
		switch(n)
		{
		default:
		case 0:
			return r;
		case 1:
			return g;
		case 2:
			return b;
		}
	}
	size_t size() const {
		return 3;
	}
	void scale(float gain) {
		for(size_t n = 0; n < size(); ++n)
		{
			float val = (*this)[n] * gain;
			if(val > 255)
				val = 255;
			(*this)[n] = val + 0.5f; // round
		}
	}
} rgb_t;

class LedSlider : public CentroidDetectionScaled
{
public:
	static constexpr size_t kDefaultNumWeights = 4;
	struct centroid_t {
		float location;
		float size;
		bool operator== (const centroid_t& other) { return !memcmp(this, &other, sizeof(other)); }
	};
	typedef enum {
		AUTO_CENTROIDS,
		AUTO_RAW,
		MANUAL_CENTROIDS,
		MANUAL_RAW,
		MANUAL_DIRECT, // uses directBegin(), directWriteCentroid()
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
		float min;
		float max;
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
	bool areLedsEnabled();
	bool isTouchEnabled();
private:
	std::vector<float> pads;
	Settings s;
	bool ledsEnabled = true;
	bool touchEnabled = true;
};
