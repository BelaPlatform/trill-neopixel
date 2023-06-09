#pragma once
#include <array>
#include <stdint.h>
#include <unistd.h> // ssize_t
#include "main.h"
#include <string.h> // memset
#include <algorithm>
#include "rgb.h"

class Stm32NeoPixel
{
public:
  virtual ssize_t send(const uint8_t* rgb, size_t length) = 0;
  enum { kNumBytesPerPixel = 3 }; // GRB
};

template <typename clkPwmDurType_t, size_t maxPixels>
class Stm32NeoPixelT : public Stm32NeoPixel {
public:
  Stm32NeoPixelT() {};
  Stm32NeoPixelT(TIM_HandleTypeDef* htim, uint32_t TIM_CHANNEL_x, clkPwmDurType_t setDuration, clkPwmDurType_t resetDuration) {
    setup(htim, TIM_CHANNEL_x, setDuration, resetDuration);
  }
  // htim should be initialised before being passed in here to PWM generation,
  // it should have a CCR2 clkPwmDurType_t wide and a memory to peripheral DMA
  // configured to the same size so that it can generate a variable PWM signal
  int setup(TIM_HandleTypeDef* htim, uint32_t TIM_CHANNEL_x, clkPwmDurType_t setDuration, clkPwmDurType_t resetDuration) {
    npBusyFlag = false;
    this->htim = htim;
    this->TIM_CHANNEL_x = TIM_CHANNEL_x;
    this->setDuration = setDuration;
    this->resetDuration = resetDuration;
    memset(pwmData.data(), 0, sizeof(pwmData));
    lastDataEnd = 0;
    return 0;
  }
  bool ready()
  {
    return !npBusyFlag;
  }
  void done()
  {
    HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_x);
    npBusyFlag = 0;
  }
  // RGB values in triplets
  ssize_t send(const uint8_t* rgb, size_t length) override
  {
    if(npBusyFlag)
      return -1;
    // check boundaries and ensure it's multiples of kNumBytesPerPixel.
    size_t numBytes = std::min(maxPixels, length / kNumBytesPerPixel) * kNumBytesPerPixel;

    // see https://controllerstech.com/pwm-with-dma-in-stm32/, but adapted for TIM2 (32 bit CCR).
    npBusyFlag = true;
    unsigned int n = 0;
    for(unsigned int i = 0; i < kLeadingZeros; ++i)
      pwmData[n++] = 0;
    for(unsigned int p = 0; p < numBytes; ++p)
    {
      size_t firstIdxOfPixel = p - (p % kNumBytesPerPixel);
      size_t color = p % kNumBytesPerPixel;
      size_t colorIdx;
      // data comes in as RGB, but has to go out as GRB.
      // this switch() swaps R and G
      switch(color)
      {
        default:
        case 0:
          colorIdx = 1;
          break;
        case 1:
          colorIdx = 0;
          break;
        case 2:
          colorIdx = 2;
          break;
      }
      uint8_t byte = rgb[firstIdxOfPixel + colorIdx];
      for (int i = kNumBitsPerByte - 1; i >= 0; --i)
      {
        clkPwmDurType_t dur;
        if(byte & (1 << i))
          dur = setDuration;
        else
          dur = resetDuration;
        pwmData[n++] = dur;
      }
    }
    // Ensure all pwmData after n is set to zero.
    // Clean up any old data at the end of the array that may have been set before
    // note that the last kTrailingZeros values are set to 0 in setup() and never overwritten,
    // so at each iteration we only have to clean anything dirty that may be left over from
    // the previous one.
    // This way we save CPU by always memset()'ing the minimum amount possible. In particular,
    // if we always send the same amount of data, there is never anything dirty to clean.
    ssize_t newZeros = lastDataEnd - n;
    if(newZeros > 0)
      memset(pwmData.data() + n, 0, newZeros * sizeof(pwmData[0]));
    lastDataEnd = n; // remember the last dirty index
    n += kTrailingZeros;
    htim->Instance->CCR2 = 0;
    if(HAL_OK == HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_x, (uint32_t *)pwmData.data(), n))
      return n;
    else
      return -1;
  }
  volatile bool npBusyFlag;
private:
  clkPwmDurType_t setDuration;
  clkPwmDurType_t resetDuration;
  // ensure the first PWM value is 0 because sometimes the first value is
  // shorter than expected. This way the first high period is guaranteed to be
  // correct.
  enum { kLeadingZeros = 1 };
  // ensure the last PWM value is 0, so if the DMA callback arrives late, we
  // have stopped sending out stuff already.
  // the remaining values are for end of frame ( >280us according to WS2812B-2020 datasheet)
  // alternatively, one could try to throttle requests based on timestamps
  enum { kTrailingZeros = 1 + 400 };
  enum { kNumBitsPerByte = 8 };
  TIM_HandleTypeDef* htim;
  uint32_t TIM_CHANNEL_x;
  size_t lastDataEnd;
  std::array<clkPwmDurType_t, (maxPixels) * kNumBitsPerByte * kNumBytesPerPixel + kTrailingZeros + kLeadingZeros> pwmData;
};

class NeoPixel{
public:
	void setSnp(Stm32NeoPixel* snp)
	{
		this->snp = snp;
	}
	virtual size_t getNumPixels() = 0;
	virtual int show() = 0;
	virtual uint8_t getPixelChannel(size_t n, size_t c) = 0;
	virtual void setPixelColor(size_t n, uint8_t r, uint8_t g, uint8_t b) = 0;
	virtual void clear() = 0;
	virtual void scaleBy(float gain) = 0;
protected:
	Stm32NeoPixel* snp = nullptr;
};

#include<stdio.h>
template <size_t kNumLeds>
class NeoPixelT : public NeoPixel
{
	enum {kNumBytesPerPixel = Stm32NeoPixel::kNumBytesPerPixel};
public:
	size_t getNumPixels() override { return kNumLeds; }
	void reverse()
	{
		for(size_t n = 0; n < kNumLeds / 2; ++n)
			for(size_t c = 0; c < kNumBytesPerPixel; ++c)
				std::swap(buffer[kNumBytesPerPixel * n + c], buffer[kNumBytesPerPixel * ( kNumLeds - n - 1) + c]);
	}
	int show() override
	{
		if(snp)
			return snp->send(buffer.data(), buffer.size());
		else
			return -1;
	}
	uint8_t getPixelChannel(size_t n, size_t c) override
	{
		size_t idx = kNumBytesPerPixel * n + c;
		if(idx >= buffer.size())
			return 0;
		else
			return buffer[idx];
	}
	void setPixelColor(size_t n, uint8_t r, uint8_t g, uint8_t b) override
	{
		if(n < kNumLeds)
		{
			buffer[kNumBytesPerPixel * n + 0] = r;
			buffer[kNumBytesPerPixel * n + 1] = g;
			buffer[kNumBytesPerPixel * n + 2] = b;
		}
	}
	void clear() override
	{
		memset(buffer.data(), 0, buffer.size() * sizeof(buffer[0]));
	}
	void scaleBy(float gain) override
	{
		uint8_t maxVal = 0;
		for(auto& b : buffer)
			maxVal = std::max(maxVal, b);
		float maxGain = 255.f / maxVal;
		gain = std::min(maxGain, gain);
		for(auto& b : buffer)
			b = std::min(255.f, b * gain); // this clipping shouldn't be needed
	}
private:
	std::array<uint8_t,kNumLeds * 3> buffer;
};
