#pragma once

#include <stdint.h>
#define pgm_read_byte(ADDR) (*(ADDR))
#define F(...)
#ifndef NULL
#define NULL nullptr
#endif // NULL
#define PROGMEM
#include <string.h>
#include <stdlib.h>

struct Print {
	void begin(uint32_t rate) {};
	void print();
	void print(const char* p);
	void print(int16_t);
	void print(uint32_t, const char*);
	void println();
	void println(const char*, ...);
	void println(uint32_t);
	void println(uint32_t, const char*);
};

static Print Serial;

unsigned long micros();
uint32_t millis();
uint32_t random(uint32_t max);
uint32_t random(uint32_t min, uint32_t max);
void randomSeed(uint32_t s);
void delay(uint32_t);
#define LED_BUILTIN 0
#define INPUT 0
#define OUTPUT 0
#define INPUT_PULLUP 0
#define OUTPUT_PULLUP 0
#define LOW 0
#define HIGH 1
void pinMode(uint32_t, uint32_t);
bool digitalRead(uint32_t);
void digitalWrite(uint32_t, bool);
typedef char __FlashStringHelper;
#define strlen_P strlen
#include <Utilities.h>
void utoa(uint32_t, char*, size_t);
void noInterrupts();
void interrupts();

#define HEX "#x"
#define PSTR(a) (a)
#include <pthread.h>
#define yield() pthread_yield()
