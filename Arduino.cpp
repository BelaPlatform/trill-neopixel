#include "Arduino.h"
#include <stdio.h>
#include <inttypes.h>

Print Serial;

void Print::print() {};
void Print::print(const char* p) {
	printf("%s", p);
}
void Print::print(int16_t a) {
	printf("%d", a);
}
void Print::print(uint32_t a, const char* f) {
	printf(f, a);
}
void Print::println() {
	printf("\n");
}
void Print::println(const char* s, ...) {
	printf("%s\n", s); // TODO
}
void Print::println(uint32_t a) {
	printf(PRIu32 "\n", a);
}
void Print::println(uint32_t a, const char* f) {
	printf(PRIu32 "\n", a);
}

uint32_t random(uint32_t max)
{
	return random(0, max);
}

void randomSeed(uint32_t s)
{
	srand(s);
}

uint32_t random(uint32_t min, uint32_t max) {
	uint32_t ran = rand();
	return map(ran, 0, RAND_MAX, min, max);
}

#ifdef __linux__
uint32_t millis() {
	return micros() / 1000.f;
}

unsigned long micros() {
	static timespec startp;
	static bool inited;
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC_RAW, &tp);
	if(!inited) {
		inited = true;
		startp = tp;
		return 0;
	}
	return ((tp.tv_sec - startp.tv_sec) * 1000000 + (tp.tv_nsec - startp.tv_nsec) / 1000.0);
}
#elif defined(STM32)
extern "C" uint32_t HAL_GetTick(void);
uint32_t millis() {
  return HAL_GetTick();
}
unsigned long micros() {
  // TODO: improve accuracy
  return 1000 * millis();
}
#else
# error define your own millis() and micro()
#endif

void pinMode(uint32_t, uint32_t) {
}
void digitalWrite(uint32_t, bool) { }
bool digitalRead(uint32_t) {
	return 0;
}

#include <unistd.h>
void delay(uint32_t t) {
	usleep(t * 1000);
}
void utoa(uint32_t num, char* dest, size_t len) {
	snprintf(dest, len, "%lu", num);
}
void noInterrupts() {}
void interrupts() {}
