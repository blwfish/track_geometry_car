#include "ring_buffer.h"
#include <string.h>

static imu_sample_t buffer[RING_BUFFER_SIZE];
static uint32_t head = 0;   // Next write position
static uint32_t count = 0;  // Number of valid samples

void ringBufferInit() {
    head = 0;
    count = 0;
}

void ringBufferPush(const imu_sample_t *sample) {
    buffer[head] = *sample;
    head = (head + 1) % RING_BUFFER_SIZE;
    if (count < RING_BUFFER_SIZE) {
        count++;
    }
}

const imu_sample_t* ringBufferGetRecent(uint32_t recency) {
    if (recency >= count) {
        return nullptr;
    }
    // head points to next write position, so most recent is at (head - 1)
    // recency=0 -> most recent, recency=1 -> second most recent, etc.
    uint32_t index = (head + RING_BUFFER_SIZE - 1 - recency) % RING_BUFFER_SIZE;
    return &buffer[index];
}

uint32_t ringBufferCount() {
    return count;
}

uint32_t ringBufferCopyRecent(imu_sample_t *dest, uint32_t requested) {
    uint32_t available = (requested < count) ? requested : count;
    // Copy oldest-to-newest into dest[]
    for (uint32_t i = 0; i < available; i++) {
        // recency of the oldest we want = (available - 1), newest = 0
        uint32_t recency = available - 1 - i;
        uint32_t index = (head + RING_BUFFER_SIZE - 1 - recency) % RING_BUFFER_SIZE;
        dest[i] = buffer[index];
    }
    return available;
}
