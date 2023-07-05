#pragma once
struct rgb_t {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t& operator[](size_t n) {
		switch(n)
		{
		default:
		case 0:
			return r;
		case 1:
			return g;
		case 2:
			return b;
		}
	}
	// TODO: how do you refactor this and the above to share code?
	uint8_t operator[](size_t n) const {
		switch(n)
		{
		default:
		case 0:
			return r;
		case 1:
			return g;
		case 2:
			return b;
		}
	}
	static constexpr size_t size() {
		return 3;
	}
	bool operator==(const rgb_t& other) const {
		for(size_t n = 0; n < size(); ++n)
			if(this->operator[](n) != other[n])
				return false;
		return true;
	}
	bool operator!=(const rgb_t& other) const {
		return !this->operator==(other);
	}
	void scale(float gain) {
		for(size_t n = 0; n < size(); ++n)
		{
			float val = (*this)[n] * gain;
			if(val > 255)
				val = 255;
			(*this)[n] = val + 0.5f; // round
		}
	}
	rgb_t scaledBy(float gain) const {
		rgb_t other(*this);
		other.scale(gain);
		return other;
	}
};

