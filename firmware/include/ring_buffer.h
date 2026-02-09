#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "config.h"

// Initialize the ring buffer (resets head and count to 0)
void ringBufferInit();

// Push a sample into the ring buffer. Overwrites oldest if full.
void ringBufferPush(const imu_sample_t *sample);

// Get a pointer to the Nth most recent sample (0 = most recent).
// Returns nullptr if index >= count.
const imu_sample_t* ringBufferGetRecent(uint32_t recency);

// Get the number of samples currently stored
uint32_t ringBufferCount();

// Copy the most recent N samples into dest[], ordered oldest-to-newest.
// Returns actual number copied (may be less than requested).
uint32_t ringBufferCopyRecent(imu_sample_t *dest, uint32_t count);

#endif // RING_BUFFER_H
