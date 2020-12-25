#include <libraries/SPI/SPI.h>
#include <vector>
#include <stdint.h>
#include <unistd.h> // ssize_t

class SpidevNeoPixels {
public:
	SpidevNeoPixels(const char* spidev) { setup(spidev); }
	SpidevNeoPixels() {}
	int setup(const char* spidev);
	/// RGB values in triplets
	ssize_t send(const uint8_t* rgb, size_t length);
private:
	SPI spi;
	std::vector<uint8_t> data;
};
