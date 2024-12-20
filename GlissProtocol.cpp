#include "GlissProtocol.h"
#include <array>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
extern "C" {
#include "../../common_stuff/z_ringbuffer.h"
};
extern "C" { void HAL_Delay(uint32_t Delay); void Error_Handler(); };

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
		size_t available = rb_available_to_write(&rb);
		if(available < len)
			return -1;
		return rb_write_to_buffer(&rb, 1, (char*)data, len);
	}
	ssize_t pop(uint8_t* data, size_t len)
	{
		size_t available = rb_available_to_read(&rb);
		if(len > available)
			return -1;
		int ret = rb_read_from_buffer(&rb, (char*)data, len);
		if(0 == ret)
			return len;
		else
			return -std::abs(ret);
	}
	ssize_t find(uint8_t val, size_t maxLen)
	{
		return rb_find(&rb, maxLen, val);
	}
	Queue()
	{
		if(rb_init_preallocated(&rb, (char*)raw.data(), raw.size()))
			Error_Handler();
	}
private:
	std::array<uint8_t,256> raw;
	ring_buffer rb;
};

class GlissProtocolProcessor
{
	static int msgPush(Queue& q, const uint8_t* data, size_t len)
	{
		if(len > kMaxMsgLength)
			return -1;
		// use a temp buffer to push all at once
		// TODO: write a variadic push instead to save this copy
		uint8_t msg[len + 1];
		memcpy(msg, data, len);
		msg[len] = kReserved;
		return q.push(msg, sizeof(msg));
	}
	static int msgPop(Queue& q, uint8_t* data, size_t maxLen)
	{
		ssize_t count = q.find(kReserved, std::min(maxLen, kMaxMsgLength));
		if(count >= 0)
		{
			// get message body
			ssize_t ret = q.pop(data, count);
			uint8_t c;
			// remove separator
			ssize_t secondRet = q.pop(&c, sizeof(c));
			if(kReserved != c)
			{
				// uh-oh that shouldn't be the case. WTF?
				printf("msgPop bad sep\n\r");
				return 0;
			}
			if(secondRet != 1 || ret != count)
			{
				// uh-oh that shouldn't be the case. WTF?
				printf("msgPop bad ret: %d %d\n\r", ret, secondRet);
				return 0;
			}
			if(0 == count)
				return 0; // an empty (spurious?) message, discard
			else
				return count;
		}
		return 0;
	}
public:
	static constexpr size_t kMaxMsgLength = 50;
	int msgPushIncoming(const uint8_t* data, size_t len)
	{
		return msgPush(inQ, data, len);
	}
	int msgPopIncoming(uint8_t* data, size_t maxLen)
	{
		return msgPop(inQ, data, maxLen);
	}
	int msgPushOutgoing(const uint8_t* data, size_t len)
	{
		return msgPush(outQ, data, len);
	}
	size_t msgPopOutgoing(uint8_t* data, size_t maxLen)
	{
		return msgPop(outQ, data, maxLen);
	}
	// return true if it did process something (and there may be something more waiting to be processed)
	bool process()
	{
		uint8_t msg[kMaxMsgLength];
		size_t len = msgPopIncoming(msg, sizeof(msg));
		if(len <= 0)
			return false;
		uint8_t* m = msg + 1;
#if 0
		// we refactored this to pop the message at once, and we now have a full message
		// before the loop, so a lot of the state machine
		// and the char-by-char processing is not really needed anymore, but we keep it
		for(size_t n = 0; n < len; ++n)
		{
			uint8_t c = msg[n];
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
				msgLen++;
			}
#else
		if(len >= 1)
		{
			msgLen = len - 1; // remove cmd
			cmd = ProtocolCmd(msg[0]);
#endif
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
					gp_setModeIoRange(m[0], m[1], {
							.cvRange = m[2],
							.min = getUint14(m + 3),
							.max = getUint14(m + 5),
						});
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
			case kGpRecorderModeGesture:
			{
				if(msgLen >= 4)
				{
					uint8_t status = m[0];
					size_t recorder = m[1];
					constexpr size_t kHeaderLength = 6; // 1 'status', 1 recorder, 2 offset, 2 length
					if(status < 2 && msgLen >= kHeaderLength)
					{
						size_t offset = getUint14(m + 2);
						size_t length = getUint14(m + 4);
						if(kGpRmgEndpoints == status && 6 == msgLen)
						{
							gp_RecorderMode_setGestureEndpoints(recorder, { .offset = offset, .length = length });
							state = kMsgEmpty;
						}
						if(kGpRmgContent == status)
						{
							constexpr size_t kDataWidth = sizeof(uint16_t);
							if(length * kDataWidth + kHeaderLength == msgLen)
							{
								gp_RecorderMode_setGestureContent(recorder, offset, length, m + kHeaderLength);
								state = kMsgEmpty;
							}
						}
					}
					if(kGpRmgPlayHead == status && 4 == msgLen)
					{
						size_t playHead = getUint14(m + 2);
						gp_recorderMode_setGesturePlayHead(recorder, playHead);
						state = kMsgEmpty;

					}
					if(kGpRmgPlayRate == status && 6 == msgLen)
					{
						uint32_t rate = getUint28(m + 2);
						gp_recorderMode_setGesturePlayRate(recorder, rate);
						state = kMsgEmpty;
					}
				}
				break;
			}
			case kGpDebugFlags:
			{
				if(2 == msgLen)
				{
					state = kMsgEmpty;
					gp_setDebugFlags(getUint14(m));
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
			case kGpGet:
			{
				if(msgLen > 0)
				{
					if(handleGpGet(m, msgLen, outQ))
					{
						state = kMsgEmpty;
					}
				}
				break;
			}
			case kGpGetResponse:
				break;
			}
		}
		return true;
	}
private:
	// only allow uint16_t to be passed to lsb()_ and msb()
	template <typename T> uint8_t lsb(T) = delete;
	template <typename T> uint8_t msb(T) = delete;
	static uint8_t lsb(uint16_t arg)
	{
		return arg & 0x7f;
	}
	static uint8_t lsbF(float arg)
	{
		return lsb(uint16_t(arg * ((1 << 14) - 1)));
	}
	static uint8_t msb(uint16_t arg)
	{
		return (arg >> 7) & 0x7f;
	}
	template <typename T>
	static size_t setUintT(uint8_t* data, T val, size_t bytes)
	{
		size_t n = 0;
		size_t b = 0;
		for( ; b < 7 * bytes; b += 7, ++n)
		{
			data[n] = val & 0x7f;
			val >>= 7;
		}
		return n;
	}
	static size_t setUint28(uint8_t* data, uint32_t val)
	{
		return setUintT(data, val, 4);
	}
	static size_t setUint14(uint8_t* data, uint32_t val)
	{
		return setUintT(data, val, 2);
	}
	// make it static to avoid confusion with variable names
	static int handleGpGet(const uint8_t* m, size_t msgLen, Queue& outQ)
	{
		if(0 == msgLen)
			return 0;
		ProtocolCmd cmd = ProtocolCmd(m[0]);
		m++;
		msgLen--;
		std::array<uint8_t,kMaxMsgLength> data;
		constexpr size_t kHeaderBytes = 2; // leave space for kGpGetResponse, cmd and copying the request
		const size_t initialN = msgLen + kHeaderBytes;
		size_t n = initialN;
		switch(cmd)
		{
		case kGpMode:
		{
			if(0 == msgLen)
			{
				data[n++] = gp_getMode();
			}
			break;
		}
		case kGpParameter:
		{
			if(2 == msgLen)
			{
				uint16_t par = gp_getModeParameter(m[0], m[1]);
				data[n++] = lsb(par);
				data[n++] = msb(par);
			}
			break;
		}
		case kGpIoRange:
		{
			if(2 == msgLen)
			{
				GpIoRange range = gp_getModeIoRange(m[0], m[1]);
				data[n++] = range.cvRange;
				data[n++] = lsb(range.min);
				data[n++] = msb(range.min);
				data[n++] = lsb(range.max);
				data[n++] = msb(range.max);
			}
			break;
		}
		case kGpModeColor:
		{
			if(2 == msgLen)
			{
				rgb_t color = gp_getModeColor(m[0], m[1]);
				for(size_t c = 0 ; c < color.size(); ++c)
					data[n++] = color[c] / 2;
			}
			break;
		}
		case kGpDebugFlags:
		{
			if(0 == msgLen)
			{
				uint16_t flags = gp_getDebugFlags();
				data[n++] = lsb(flags);
				data[n++] = msb(flags);
			}
			break;
		}
		case kGpRecorderModeGesture:
		{
			handleGpRecorderModeGestureGet(data, n, ProtocolCmdRecorderModeGesture(m[0]), m + 1, msgLen - 1);
			break;;
		}
		case kGpStore:
		case kGpGet:
		case kGpGetResponse:
			break;
		}
		if(n > initialN)
		{
			// an actual response was provided
			if(n >= data.size() - 1)
			{
				// buffer overflow
				printf("ERROR: we wrote too much data: %d\n\r", n);
				return 1; // pretend it's handled to avoid making things worse in subsequent calls
			}
			// fill the space we reserved in the first part of the buffer with the request data
			// so it identifies the actual payload that has been written in the switch()
			data[0] = kGpGetResponse;
			data[1] = cmd;
			memcpy(data.data() + kHeaderBytes, m, msgLen);
			// mark the end of the message
			data[n++] = kReserved;
#if 0
			printf("SENT ");
			for(size_t c = 0; c < n; ++c)
				printf("%d ", data[c]);
			printf("\n\r");
#endif
			outQ.push(data.data(), n);
			return 1;
		}
		return 0;
	}
	static void handleGpRecorderModeGestureGet(std::array<uint8_t,kMaxMsgLength>& data, size_t& n,
			ProtocolCmdRecorderModeGesture cmd, const uint8_t* m, size_t msgLen)
	{
		if(msgLen > 0)
		{
			uint8_t recorder = m[0];
			switch(cmd)
			{
			case kGpRmgEndpoints:
			{
				GpRmgEndpoints eps = gp_RecorderMode_getGestureEndpoints(recorder);
				n += setUint14(data.data() + n, eps.offset);
				n += setUint14(data.data() + n, eps.length);
			}
				break;
			case kGpRmgContent:
			{
				size_t offset = getUint14(m + 1);
				size_t length = m[3];
				size_t numBytes = length * sizeof(uint16_t);
				if(numBytes < data.size() - n)
				{
					gp_RecorderMode_getGestureContent(recorder, offset, length, data.data() + n);
					n += numBytes;
				}
			}
				break;
			case kGpRmgPlayRate:
			{
				uint32_t playRate = gp_recorderMode_getGesturePlayRate(recorder);
				n += setUint28(data.data() + n, playRate);
			}
				break;
			case kGpRmgPlayHead:
			{
				size_t playHead = gp_recorderMode_getGesturePlayHead(recorder);
				n += setUint14(data.data() + n, playHead);
			}
				break;
			}
		}
	}
	static uint16_t getUint14(const uint8_t* data)
	{
		return (data[0] & 0x7f) | ((data[1] & 0x7f) << 7);
	}
	static uint32_t getUint28(const uint8_t* data)
	{
		return getUint14(data) | (getUint14(data + 2) << 14);
	}

	Queue inQ;
	Queue outQ;
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

static std::array<GlissProtocolProcessor,kSysexNumSp> processors;

int gp_incoming(SysexPeripheral src, const void* data, size_t len)
{
	if(len > GlissProtocolProcessor::kMaxMsgLength)
	{
		printf("discarding long msg [%u]\n\r", len);
		return 1;
	}
	while(processors[src].msgPushIncoming((const uint8_t*)data, len))
	{
		printf("W\n\r");
		HAL_Delay(1);
	}
	return 0;
}

void gp_processIncoming()
{
	for(auto& p : processors)
	{
		while(p.process())
		{
			// call as long as there are messages to process
		}
	}
}

int gp_outgoing(SysexPeripheral dst, int (*callback)(SysexPeripheral sp, const uint8_t* data, size_t maxLen))
{
	constexpr size_t kMaxSent = 48; // arbitrary limit. Outgoing queues for the peripherals are the actual limiting factors
	std::array<uint8_t,kMaxSent> buffer;
	size_t sent = 0;
	int ret = 0;
	while(sent < kMaxSent && !ret)
	{
		size_t count = processors[dst].msgPopOutgoing(buffer.data(), std::min(kMaxSent - sent, buffer.size()));
		if(count)
		{
			ret |= callback(dst, buffer.data(), count);
			sent += count;
		} else
			break;
	}
	return ret;
}
