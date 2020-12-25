#include <Bela.h>
#include <stdio.h>
#include <libraries/SPI/SPI.h>
#include <vector>
#include "SpidevNeoPixels.h"
SpidevNeoPixels snp;

const uint8_t kBytesPerRgb = 3;
void task(void*) {
	std::vector<uint8_t> colors = {{
		0x00, 0xC0, 0x00,
		0xC0, 0x00, 0x00,
		0x00, 0x00, 0xC0,
		0x00, 0x00, 0x00,
		// 0x00, 0xff, 0x00,
		// 0x00, 0x00, 0xff,
		// 0x80, 0x80, 0x00,
	}};
	uint8_t kNumLeds = 16;
	std::vector<uint8_t> rgb(kNumLeds * kBytesPerRgb);
	for (unsigned int n = 0; n < 100 && !Bela_stopRequested(); ++n) {
		for(unsigned int l = 0; l < kNumLeds; ++l) {
			for(unsigned int b = 0;  b < kBytesPerRgb; ++b) {
				uint8_t val = colors[(( n + l) * kBytesPerRgb + b) % colors.size()];;
				rgb[l * kBytesPerRgb + b] = val;
			}
		}
		printf("RGB:\n");
		for(unsigned int l = 0; l < kNumLeds; ++l) {
			printf("[%2d]: ", l);
			for(unsigned int b = 0;  b < kBytesPerRgb; ++b) {
				printf("0x%02x ", rgb[n]);
			}
			printf("\n");
		}
		printf("\n");
		snp.send(rgb.data(), rgb.size());
		usleep(100000);
	}
}

bool setup(BelaContext *context, void *userData)
{
	snp.setup("/dev/spidev2.1");
	task(0); _exit (0);
	Bela_runAuxiliaryTask(task, 90);
	return true;
}

void render(BelaContext *context, void *userData){}


void cleanup(BelaContext *context, void *userData)
{

}
