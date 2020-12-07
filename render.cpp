#include <Bela.h>
#include <stdio.h>
#include <libraries/SPI/SPI.h>
#include <cmath>
#include <string.h>
SPI spi;

const unsigned int kSpiClock = 12000000;
const unsigned int kSpiMinTransferSize = 160; // min number of bytes to trigger DMA transfer
const unsigned int kSpiMaxTransferSize = 4096; // max number of bytes to be sent at once
const unsigned int kSpiWordLength = 32;
const uint8_t kSpiMsbFirst = 1;
const float kSpiInterWordTime = 200;
const float kSpiPeriodNs = 1000000000.0 / kSpiClock;
const uint8_t kBitsPerByte = 8;
// The specs (WS2812B-2020 datasheet) say:
// T0H 0 code, high voltage time 220ns~380ns
// T1H 1 code, high voltage time 580ns~1µs
// T0L 0 code, low voltage time 580ns~1µs
// T1L 1 code, low voltage time 220ns~420ns
// RES Frame unit, low voltage time >280µs
// WS2812B v1.0 on the other hand says:
// TH+TL = 1.25µs ± 600ns
// T0H 0 code, high voltage time 0.4us ±150ns
// T1H 1 code, high voltage time 0.8us ±150ns
// T0L 0 code, low voltage time 0.85us ±150ns
// T1L 1 code, low voltage time 0.45us ±150ns
// More recent datasheets say

// We pick below some conservative values for WS2812B
typedef struct {
	float min;
	float max;
} T_t;

const T_t TH[2] = {
	{.min = 200, .max = 400}, // T0H
	{.min = 700, .max = 900}, // T1H
};

const T_t TL[2] = {
	{.min = 750, .max = 950}, // T0L
	{.min = 350, .max = 550}, // T1L
};

// Additionally, there is need for adding a long enough "0" output before we start clocking out data.
// The SPI peripheral in use seems to always set the MOSI high when not clocking out data, so we have
// to start clocking out some zeros first.
// Skipping this will result in 1-bit offset in the bit shifting. If you see random flashes
// in your LEDs strip and/or the first LED remaning on when set to off
// and/or the others have the colors "sligtly" wrong, this could be the reason.
// This could be avoided if the MOSI was to be LOW when idle, but this would require
// an external transistor to invert it.
const double kSpiLeadingZerosNs = 10000; // determined empirically
const unsigned int kSpiLeadingZeros = std::ceil((kSpiLeadingZerosNs / kSpiPeriodNs));

bool readBitField(uint8_t* data, uint32_t offset) {
	uint8_t position = offset % kBitsPerByte;
	unsigned int i = offset / kBitsPerByte;
	return data[i] & (1 << position);
}

void writeBitField(uint8_t* data, uint32_t offset, bool value) {
	uint8_t position = offset % kBitsPerByte;
	unsigned int i = offset / kBitsPerByte;
	if(value)
		data[i] |= 1 << position;
	else
		data[i] &= ~(1 << position);
}

ssize_t rgbToClk(uint8_t* rgb, size_t numRgb, uint8_t* out, size_t numOut)
{
	memset(out, 0, numOut * sizeof(out[0]));
	// writeBitField(out, 0, 1);
	uint32_t clk = 0;
	// emsure we have complete RGB sets
	numRgb = numRgb - (numRgb % 3);
	for(uint32_t inBit = 0; inBit < numRgb * kBitsPerByte; ++inBit) {
		// data comes in as RGB but needs to be shuffled into GRB for the WS2812B
		uint32_t actualInBit;
		uint8_t remainder = inBit % (3 * kBitsPerByte);
		if(remainder < kBitsPerByte)
			actualInBit = inBit + kBitsPerByte;
		else if(remainder < kBitsPerByte * 2)
			actualInBit = inBit - kBitsPerByte;
		else
			actualInBit = inBit;
		bool value = readBitField(rgb, actualInBit);
		const T_t* Ts[2] = {TH, TL};
		for(unsigned int t = 0; t < 2; ++t) {
			const T_t* T = Ts[t];
			bool highLow = (0 == t) ? true : false;
			float time = 0;
			while(time < T[value].min) {
				bool wordEnd = (kSpiWordLength - 1 == clk);
				writeBitField(out, clk, highLow);
				time += kSpiPeriodNs;
				if(wordEnd)
					time += kSpiInterWordTime;
				++clk;
				if(clk / kBitsPerByte > numOut)
					return -1;
			}
			if(time > T[value].max) {
				printf("Error: expected %f but got %f\n", T[value].max, time);
				return 0;
			}

		}
	}
	// uint8_t* data = out;
	// data[0] = 0x1;
	// data[1] = 0x5;
	// data[2] = 0x3;
	// data[3] = 11;
	// data[4] = data[5] = data[6] = data[7] = 0;
	ssize_t numBytes = ((clk + kSpiWordLength - 1) / kSpiWordLength) * kSpiWordLength / kBitsPerByte; // round up to the next word
	if(numBytes > numOut) // just in case numOut was not a multiple, we now no longer fit
		return -1;
	return numBytes;
}

void sendSpi(uint8_t* data, size_t length)
{
	if(length < kSpiMinTransferSize)
		length = kSpiMinTransferSize;
    if (spi.transfer(data, NULL, length) == 0)
        printf("SPI: Transaction Complete. Sent %d bytes\n", length);
    else
        printf("SPI: Transaction Failed\r\n");
}

#include <vector>

const uint8_t kBytesPerRgb = 3;
void task(void*) {
	std::vector<uint8_t> data(kSpiMaxTransferSize);
	std::vector<uint8_t> colors = {{
		// }};
// /*
		0x00, 0xC0, 0x00,
		0xC0, 0x00, 0x00,
		0x00, 0x00, 0xC0,
		0x00, 0x00, 0x00,
		// 0x00, 0xff, 0x00,
		// 0x00, 0x00, 0xff,
		// 0x80, 0x80, 0x00,
	}};
	//*/
	// uint8_t kNumColors = colors.size() / kBytesPerRgb;
	uint8_t kNumLeds = 16;
	std::vector<uint8_t> rgb(kNumLeds * kBytesPerRgb);
	for (unsigned int n = 0; n < 100 && !Bela_stopRequested(); ++n) {
		for(unsigned int l = 0; l < kNumLeds; ++l) {
			printf("RGB[%d]: ", l);
			for(unsigned int b = 0;  b < kBytesPerRgb; ++b) {
				uint8_t val = colors[(( n + l) * kBytesPerRgb + b) % colors.size()];;
				rgb[l * kBytesPerRgb + b] = val;
				printf("%#02x ", val);
			}
			printf("\n");
		}
		printf("data: ");
		for(unsigned int n = 0; n < kNumLeds * kBytesPerRgb; ++n) {
			if(0 == (n % 3))
				printf("\n");
			printf("%#02x ", rgb[n]);
		}
		printf("\n");
		ssize_t len = kSpiLeadingZeros + rgbToClk(rgb.data(), rgb.size(), data.data() + kSpiLeadingZeros, data.size() - kSpiLeadingZeros);
		if(len < 0) {
			fprintf(stderr, "Error: message too long\n");
			break;
		} else {
			printf("Data is %d\n", len);
		}
		// print data as it will be sent out
		unsigned int k = 0;
		for(unsigned int n = 0; n < len; ++n) {
			for(unsigned int c = 0; c < kBitsPerByte; ++c) {
				printf("%d", (bool)(data[n] & (1 << c)));
				++k;
				if(0 == (k % kBitsPerByte))
					printf(" ");
				if(0 == (k % kSpiWordLength))
					printf("| ");
			}
		}
		printf("\n");

		// format data
		// SPI transmits the most significant byte first ("right justified in each word")
		// so if we have more than 1 byte per word we need to shuffle them around so that
		// they are output in the correct order
		unsigned int bytesPerWord = kSpiWordLength / kBitsPerByte;
		for(unsigned int n = 0; n < len; n += bytesPerWord) {
			for(unsigned int b = 0; b < bytesPerWord / 2; ++b) {
				uint8_t tmp = data[n + b];
				data[n + b] = data[n + bytesPerWord - 1 - b];
				data[n + bytesPerWord - 1 - b] = tmp;
			}
		}
		if(kSpiMsbFirst) {
			for(unsigned int n = 0; n < len; ++n)
				data[n] = __builtin_bitreverse8(data[n]);
		}

		sendSpi(data.data(), data.size());
		usleep(100000);
	}
}
bool setup(BelaContext *context, void *userData)
{
	spi.setup("/dev/spidev2.1", // Device to open
		kSpiClock, // Clock speed in Hz
		SPI::SS_LOW, // Chip select, won't matt	er
		0, // Delay after last transfer before deselecting the device, won't matter
		kSpiWordLength, // No. of bits per transaction word,
		SPI::MODE3 // SPI mode
	);
	task(0); _exit (0);
	Bela_runAuxiliaryTask(task, 90);
	return true;
}

void render(BelaContext *context, void *userData){}


void cleanup(BelaContext *context, void *userData)
{

}
