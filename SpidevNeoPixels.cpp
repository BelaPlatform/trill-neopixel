#include "SpidevNeoPixels.h"
#include <string.h>
#include <stdio.h>


#undef SNP_DEBUG
const unsigned int kSpiClock = 12000000;
const unsigned int kSpiMinTransferSize = 160; // min number of bytes to trigger DMA transfer
const unsigned int kSpiMaxTransferSize = 4096; // max number of bytes to be sent at once
const unsigned int kSpiWordLength = 32;
const float kSpiInterWordTimeNs = 200;

// Additionally, there is need for adding a long enough "0" output before we start clocking out data.
// The SPI peripheral in use seems to always set the MOSI high when not clocking out data, so we have
// to start clocking out some zeros first.
// Skipping this will result in 1-bit offset in the bit shifting. If you see random flashes
// in your LEDs strip and/or the first LED remaning on when set to off
// and/or the others have the colors "slightly" wrong, this could be the reason.
// This could be avoided if the MOSI was to be LOW when idle, but this would require
// an external transistor to invert it.
const float kSpiLeadingZerosNs = 10000; // determined empirically

int SpidevNeoPixels::setup(const char* spidev)
{
	data.resize(kSpiMaxTransferSize);
	static_assert(kSpiMaxTransferSize > kSpiMinTransferSize, "");
	int ret = spi.setup({
			.device = spidev, // Device to open
			.speed = kSpiClock, // Clock speed in Hz
			.numBits = kSpiWordLength, // No. of bits per transaction word,
			.mode = Spi::MODE3 // SPI mode
			});
	ret |= GpNeoPixels::setup({
			.clockHz = kSpiClock,
			.wordLength = kSpiWordLength,
			.interWordTimeNs = kSpiInterWordTimeNs,
			.leadingZerosNs = kSpiLeadingZerosNs,
			.busBigEndian = true,
			.busMsbBitFirst = true,
			});
	return ret;
}

ssize_t SpidevNeoPixels::send(const uint8_t* rgb, size_t length) {
	ssize_t len = rgbToClk(rgb, length, data.data(), data.size());
	if(len < 0) {
		fprintf(stderr, "Error: message too long\n");
		return -1;
	} else {
#ifdef SNP_DEBUG
		printf("Data is %d\n", len);
#endif // SNP_DEBUG
	}
#ifdef SNP_DEBUG
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
#endif // SNP_DEBUG

	size_t transmissionLength = len;
	if(transmissionLength < kSpiMinTransferSize) {
		memset(data.data() + transmissionLength, 0, transmissionLength - kSpiMinTransferSize);
		transmissionLength = kSpiMinTransferSize;
	}
	if (spi.transfer(data.data(), NULL, transmissionLength) == 0) {
#ifdef SNP_DEBUG
		printf("SPI: Transaction Complete. Sent %d bytes\n", transmissionLength);
#endif // SNP_DEBUG
	} else
		printf("SPI: Transaction Failed\r\n");
	return transmissionLength;
}
