// Minimal Arduino.h mock for native unit tests.
// Provides just enough to compile config.h and modules under test.

#ifndef ARDUINO_H_MOCK
#define ARDUINO_H_MOCK

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// millis() mock — returns a controllable value
#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t _mock_millis;
static inline uint32_t millis() { return _mock_millis; }

#ifdef __cplusplus
}
#endif

#endif // ARDUINO_H_MOCK
