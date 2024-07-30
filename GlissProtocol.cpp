#include "GlissProtocol.h"
#include <array>
#include <sys/types.h>
#include <stdio.h>

static rgb_t protocolMakeColor(const uint8_t* data)
{
	rgb_t color;
	for(size_t n = 0; n < color.size(); ++n)
		color[n] = !!data[n] + 2 * data[n]; // 0, 3, 5, 7 ... 255
	return color;
}

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
	ssize_t find(uint8_t val, size_t maxLen)
	{
		size_t r = readPtr;
		for(size_t count = 0; count < maxLen; ++count)
		{
			if(r == writePtr) // end of buffer
				break;
			if(raw[r] == val)
				return count;
			if(++r >= raw.size())
				r = 0;
		}
		return -1;
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
	int msgIncoming(const uint8_t* data, size_t len)
	{
		int ret = inQ.push(data, len);
		uint8_t c = kReserved;
		ret |= inQ.push(&c, sizeof(c)); // end of message
		return ret;
	}
	size_t msgOutgoing(uint8_t* data, size_t maxLen)
	{
		ssize_t count = outQ.find(kReserved, maxLen);
		if(count >= 0)
		{
			outQ.pop(data, count + 1);
			if(kReserved != data[count])
			{
				// uh-oh that shouldn't be the case. WTF?
				printf("msgOutgoing found wrong value\n\r");
				return 0;
			}
			if(0 == count)
				return 0; // an empty (spurious?) message, discard
			else
				return count - 1;
		}
		return 0;
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
			case kGpModeColor:
			{
				if(5 == msgLen)
				{
					state = kMsgEmpty;
					gp_setModeColor(m[0], m[1], protocolMakeColor(m + 2));
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
				break;
			}
			case kGpStore:
			{
				if(0 == msgLen)
				{
					state = kMsgEmpty;
					gp_store();
				}
				break;
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
	Queue outQ;
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

static std::array<GlissProtocolProcessor,kGpNumPp> processors;

int gp_incoming(ProtocolPeripheral src, const void* data, size_t len)
{
	return processors[src].msgIncoming((const uint8_t*)data, len);
}

void gp_processIncoming()
{
	for(auto& p : processors)
		p.process();
}

int gp_outgoing(ProtocolPeripheral dst, int (*callback)(const uint8_t* data, size_t maxLen))
{
	constexpr size_t kMaxSent = 48; // arbitrary limit. Outgoing queues for the peripherals are the actual limiting factors
	std::array<uint8_t,kMaxSent> buffer;
	size_t sent = 0;
	int ret = 0;
	while(sent < kMaxSent && !ret)
	{
		size_t count = processors[dst].msgOutgoing(buffer.data(), std::min(kMaxSent - sent, buffer.size()));
		if(count)
		{
			ret |= callback(buffer.data(), count);
			sent += count;
		} else
			break;
	}
	return ret;
}
