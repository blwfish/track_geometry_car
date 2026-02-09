#ifndef SUMMARY_H
#define SUMMARY_H

#include "config.h"

// Compute 1-second summary statistics from the ring buffer.
// Reads the most recent samples (up to 100 = 1 second at 100Hz).
// Returns true if enough samples were available (minimum 10).
bool summaryCompute(summary_1s_t *summary, uint32_t totalSamples, float sampleRate);

#endif // SUMMARY_H
