#include <Bela.h>
#include <stdio.h>
#include <vector>
#include "TrillRackInterface.h"
extern TrillRackInterface tri; // owned by TrillRack.cpp
static uint8_t kNumLeds = 16;

//#define SPIDEV 1
#define MCASP 1
#ifdef SPIDEV
#include "SpidevNeoPixels.h"
SpidevNeoPixels snp;
#endif // SPIDEV
#ifdef MCASP
#include "BelaAudioNeoPixels.h"
BelaAudioNeoPixels* bnp; // required by NeoPixel.h
#endif // MCASP

volatile int gRunning = 0;
#define CROSSFADE // demo: slowly crossfade between neighbouring LEDs
//#define SINGLE_LED // demo: change the color of a single LED
const uint8_t kBytesPerRgb = 3;
void lowleveldemo(void*) {
#if (!defined(CROSSFADE) && !defined(SINGLE_LED))
	return;
#endif
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
	while(!gRunning)
		usleep(100000);
	while(!Bela_stopRequested())
	for (unsigned int n = 0; n < 2 * maxVal && !Bela_stopRequested(); ++n) {
		for(unsigned int l = 0; l < kNumLeds; ++l) {
			for(unsigned int b = 0;  b < kBytesPerRgb; ++b) {
#ifdef SINGLE_LED
				if(l != 5)
					break;
				uint8_t val = colors[(( n + l) * kBytesPerRgb + b) % colors.size()];;
				rgb[l * kBytesPerRgb + b] = val;
				printf("%3d  ", val);
#endif // SINGLE_LED
#ifdef CROSSFADE
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
#endif // CROSSFADE
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
#ifdef SPIDEV
		snp.send(rgb.data(), rgb.size());
#endif // SPIDEV
#ifdef MCASP
		bnp->send(rgb.data(), rgb.size());
#endif // MCASP
		usleep(100000 / maxVal);
	}
}

void tr_loop_launcher(void*)
{
	while(!gRunning)
		usleep(100000);
	while(!Bela_stopRequested()) {
		tr_loop();
		usleep(500); // failsafe sleep
	}
}
bool setup(BelaContext *context, void *userData)
{
#ifdef SPIDEV
	if(snp.setup("/dev/spidev2.1"))
		return false;
#endif // SPIDEV
#ifdef MCASP
	bnp = new BelaAudioNeoPixels;
	if(bnp->setup(context, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}))
		return false;
#endif // MCASP
	//Bela_runAuxiliaryTask(lowleveldemo, 90);
	//return true;

	//sendNpTask = Bela_runAuxiliaryTask(sendNp, 90);
	//Bela_runAuxiliaryTask(trillNp, 90);
	Bela_runAuxiliaryTask(tr_loop_launcher, 1);
	return tr_setup();
}

void render(BelaContext *context, void *userData)
{
	gRunning = 1;
#ifdef MCASP
	bnp->process(context);
#endif // MCASP
	tri.process(context);
}


void cleanup(BelaContext *context, void *userData)
{

}
