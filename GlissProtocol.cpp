#include "GlissProtocol.h"
#include <array>
#include <sys/types.h>
#include <stdio.h>


class Queue
{
public:
	int push(const uint8_t* data, size_t len)
	{
		size_t w = writePtr;
		while(len--)
		{
			if(((readPtr - 1 + raw.size()) % raw.size()) == w) // full!
				return 1;
			raw[w++] = *data++;
			if(w >= raw.size())
				w = 0;
		}
		writePtr = w;
		return 0;
	}
	ssize_t pop(uint8_t* data, size_t maxLen)
	{
		size_t len = 0;
		size_t r = readPtr;
		while(maxLen--)
		{
			if(r == writePtr) // empty!
				break;
			data[len++] = raw[r++];
			if(r >= raw.size())
				r = 0;
		}
		readPtr = r;
		return len;
	}
	size_t size() const
	{
		return (writePtr - readPtr - 1 + raw.size()) % raw.size();
	}
private:
	volatile size_t writePtr = 0;
	volatile size_t readPtr = 0;
	std::array<uint8_t,100> raw;
};

class GlissProtocolProcessor
{
public:
	int push(const uint8_t* data, size_t len)
	{
		int ret = inQ.push(data, len);
		uint8_t c = kReserved;
		ret |= inQ.push(&c, sizeof(c)); // end of message
		return ret;
	}
	void process()
	{
		uint8_t c;
		while(1 == inQ.pop(&c, sizeof(c)))
		{
			if(kReserved == c)
			{
				// message ended, discard anything that might have been
				state = kMsgEmpty;
				continue;
			}
			if(kMsgEmpty == state)
			{
				cmd = ProtocolCmd(c);
				state = kMsgInProgress;
				msgLen = 0;
			} else
			if(kMsgInProgress == state)
			{
				m[msgLen++] = c;
			}
			switch(cmd)
			{
			case kGpMode:
			{
				if(1 == msgLen)
				{
					state = kMsgEmpty;
					gp_setMode(m[0]);
				}
				break;
			}
			case kGpParameter:
			{
				if(4 == msgLen)
				{
					state = kMsgEmpty;
					gp_setModeParameter(m[0], m[1], getUint14(m + 2));
				}
				break;
			}
			case kGpIoRange:
			{
				if(7 == msgLen)
				{
					state = kMsgEmpty;
					gp_setModeIoRange(m[0], m[1], m[2], getUint14(m + 3), getUint14(m + 5));
				}
				break;
			}
			case kGpColor:
			{
				if(5 == msgLen)
				{
					state = kMsgEmpty;
					gp_setModeColor(m[0], m[1], m[2], m[3], m[4]);
				}
				break;
			}
			case kGpMenuColor:
			{
				if(4 == msgLen)
				{
					state = kMsgEmpty;
					gp_setMenuColor(m[0], m[1], m[2], m[3]);
				}
				break;
			}
			case kGpDebugFlags:
			{
				if(2 == msgLen)
				{
					state = kMsgEmpty;
					extern void setDebugFlags(uint8_t flags);
					setDebugFlags(m[0]);
				}
			}
			}
		}
	}
private:
	static uint16_t getUint14(const uint8_t* data)
	{
		return (data[0] & 0x7f) | ((data[1] & 0x7f) << 7);
	}
	Queue inQ;
	static constexpr size_t kMaxCmdLength = 10;
	std::array<uint8_t,kMaxCmdLength> msg;
	uint8_t* const m = msg.data(); // allows terser style above
	size_t msgLen;
	size_t msgDesiredLen;
	enum State {
		kMsgEmpty,
		kMsgInProgress,
		kMsgReady,
	} state = kMsgEmpty;
	ProtocolCmd cmd;
	static constexpr uint8_t kReserved = 255;
};

static std::array<GlissProtocolProcessor,2> processors;

int gp_incoming(ProtocolPeripheral src, const void* data, size_t len)
{
	return processors[src].push((const uint8_t*)data, len);
}

void gp_processIncoming()
{
	for(auto& p : processors)
		p.process();
}
