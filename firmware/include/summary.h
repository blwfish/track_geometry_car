#ifndef SUMMARY_H
#define SUMMARY_H

#include "config.h"

// Compute 1-second summary statistics from the ring buffer.
// Reads the most recent samples (up to 100 = 1 second at 100Hz).
// Returns true if enough samples were available (minimum 10).
bool summaryCompute(summary_1s_t *summary, uint32_t totalSamples, float sampleRate);

// Compute summary from a pre-copied sample array (thread-safe variant).
// samples[] should contain sampleCount samples (most-recent-first order).
// Returns true if sampleCount >= 10.
bool summaryComputeFromArray(summary_1s_t *summary, const imu_sample_t *samples,
                             uint32_t sampleCount, uint32_t totalSamples, float sampleRate);

#endif // SUMMARY_H
