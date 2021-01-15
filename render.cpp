#include <Bela.h>
#include <stdio.h>
#include <libraries/Trill/Trill.h>
#include <vector>
#include "SpidevNeoPixels.h"
#include <cmath>

SpidevNeoPixels snp;
Trill trill;
uint8_t kNumLeds = 16;

const uint8_t kBytesPerRgb = 3;
// routine that slowly crossfades between neighbouring LEDs
void crossfade(void*) {
	std::vector<uint8_t> colors = {{
		0x00, 0xC0, 0x00,
		0xC0, 0x00, 0x00,
		0x00, 0x00, 0xC0,
		0x00, 0x00, 0x00,
		// 0x00, 0xff, 0x00,
		// 0x00, 0x00, 0xff,
		// 0x80, 0x80, 0x00,
	}};
	std::vector<uint8_t> rgb(kNumLeds * kBytesPerRgb);
	unsigned int maxVal = 100;
	for (unsigned int n = 0; n < 2 * maxVal && !Bela_stopRequested(); ++n) {
		for(unsigned int l = 0; l < kNumLeds; ++l) {
			for(unsigned int b = 0;  b < kBytesPerRgb; ++b) {
				//uint8_t val = colors[(( n + l) * kBytesPerRgb + b) % colors.size()];;
				//rgb[l * kBytesPerRgb + b] = val;
				uint8_t val = 0;
				if(5 == l && 0 == b) {
				if(n <  maxVal)
						val = n;
					else
						val = 2 * maxVal - 1 - n;
					val += 2;
					printf("%d, ", val);
				}
				if(6 == l && 0 == b) {
					if(n < maxVal)
						val = maxVal - 1 - n;
					else
						val = n - maxVal;
					val += 2;
					printf("%d, ", val);
				}
				rgb[l * kBytesPerRgb + b] = val;
			}
		}
		printf("\n");
#if 0
		printf("RGB:\n");
		for(unsigned int l = 0; l < kNumLeds; ++l) {
			printf("[%2d]: ", l);
			for(unsigned int b = 0;  b < kBytesPerRgb; ++b) {
				printf("0x%02x ", rgb[l * kBytesPerRgb + b ]);
			}
			printf("\n");
		}
		printf("\n");
#endif
		snp.send(rgb.data(), rgb.size());
		usleep(100000 / maxVal);
	}
}

#include "NeoPixel.h"
NeoPixel np(kNumLeds, 0, NEO_GRB);
AuxiliaryTask sendNpTask;
static void sendNp(void*) {
	np.show();
}
void npShow() {
	Bela_scheduleAuxiliaryTask(sendNpTask); //
}

void resample(float* out, unsigned int nOut, float* in, unsigned int nIn)
{
#if 0 // naive: sum all input energy into outputs
	for(unsigned int no = 0; no < nOut; ++no) {
		out[no] = 0;
		unsigned int niStart = no * nIn / nOut;
		unsigned int niEnd = (no + 1) * nIn / nOut;
		for(unsigned int ni = niStart; ni < niEnd; ++ni) {
			out[no] += in[ni];
		}
	}
#endif
#if 1 // weighted sum
	float r = 2;
	for(int no = 0; no < nOut; ++no) {
		out[no] = 0;
		for(int ni = 0; ni < nIn; ++ni) {
			float fracInIdx = no * nIn / (float)nOut;
			float weight = (1 - (std::abs(fracInIdx - ni) / r));
			weight = std::max(0.f, weight); // clip to 0
			out[no] += in[ni] * weight;
		}
	}
#endif
}

unsigned int padsToOrderMap[30] = {
	0,
	1,
	2,
	3,
	4,
	5,
	6,
	7,
	8,
	9,
	10,
	11,
	12,
	13,
	27,
	26,
	25,
	14,
	15,
	16,
	17,
	18,
	19,
	20,
	24,
	21,
	22,
	23,
	28,
	29,
};

template <typename T, typename U>
void sort(T* out, U* in, unsigned int* order, unsigned int size)
{
	for(unsigned int n = 0; n < size; ++n)
		out[n] = in[order[n]];
}

void trillNp(void*)
{
	while(!Bela_stopRequested()) {
		trill.readI2C();
		unsigned int numPads = 24;//trill.rawData.size();
		float* pads = trill.rawData.data();
		float i[numPads];
		sort(i, pads, padsToOrderMap, numPads);
#if 0 // debug order
		for(unsigned int n = 0; n < numPads; ++n)
			printf("%4d ", n);
		printf("\n");
		for(unsigned int n = 0; n < numPads; ++n)
			printf("%4.0f ", pads[n] * 4096);
		printf("\n");
		for(unsigned int n = 0; n < numPads; ++n)
			printf("%4.0f ", i[n] * 4096);
		printf("\n");
		printf("\n");
#endif
		float d[kNumLeds];
		resample(d, kNumLeds, i, numPads);
#if 0 // debug resample
		for(unsigned int n = 0; n < numPads; ++n)
			printf("%.2f ", i[n]);
		printf("\n");
		for(unsigned int n = 0; n < kNumLeds; ++n)
			printf("%.2f    ", d[n]);
		printf("\n\n");
#endif
		for(unsigned int n = 0; n < kNumLeds; ++n)
			np.setPixelColor(n, d[n] * 255, 0, 0);
		npShow(); // do the showing in a different thread, so it doesn't affect timing of this loop
		usleep(10000);
	}
}

bool setup(BelaContext *context, void *userData)
{
	snp.setup("/dev/spidev2.1");
	np.begin();
	if(trill.setup(1, Trill::FLEX, 0x50))
		return false;
	trill.setMode(Trill::DIFF);
	trill.printDetails();

	sendNpTask = Bela_runAuxiliaryTask(sendNp, 90);
	Bela_runAuxiliaryTask(trillNp, 90);
	return true;
}

void render(BelaContext *context, void *userData){}


void cleanup(BelaContext *context, void *userData)
{

}
