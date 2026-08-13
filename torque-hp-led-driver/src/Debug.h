#pragma once
/**
 * Debug.h
 *
 * Simple debug logging macro (same pattern used in the Leonardo code).
 * Set DEBUG_LEVEL to 0 to compile out debug prints.
 *
 * Note: On MKRZero Serial refers to the USB CDC serial port used for logs.
 */
#define DEBUG_LEVEL 1

#if DEBUG_LEVEL >= 1
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINTF(...)
#endif
