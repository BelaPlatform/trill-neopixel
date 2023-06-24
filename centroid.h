#pragma once
#include <string.h>
struct centroid_t {
	float location;
	float size;
	bool operator== (const centroid_t& other) { return !memcmp(this, &other, sizeof(other)); }
};
