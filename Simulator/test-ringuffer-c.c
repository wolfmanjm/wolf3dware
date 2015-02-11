#include "RingBuffer.h"
#include <assert.h>

int main(int argc, char const *argv[])
{
	RingBuffer_t *rb= CreateRingBuffer(10);
	assert(RingBufferEmpty(rb));
	assert(!RingBufferFull(rb));
	assert(RingBufferPut(rb, 1));
	assert(!RingBufferEmpty(rb));
	assert(!RingBufferFull(rb));
	uint8_t i;
	for (i = 2; i <= 9; ++i) {
	    assert(RingBufferPut(rb, i));
		if(i < 9) assert(!RingBufferFull(rb));
		else assert(RingBufferFull(rb));
	}
	assert(!RingBufferEmpty(rb));
	assert(!RingBufferPut(rb, 10));
	assert(RingBufferFull(rb));

	uint8_t x;
	for (i = 1; i <= 8; ++i) {
		assert(!RingBufferEmpty(rb));
	    assert(RingBufferGet(rb, &x));
		assert(!RingBufferFull(rb));
		assert(x == i);
	}
	assert(!RingBufferEmpty(rb));
	assert(RingBufferGet(rb, &x));
	assert(x == 9);
	assert(RingBufferEmpty(rb));

	DeleteRingBuffer(rb);
}
