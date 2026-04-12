#pragma once

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)       \
    (byte & 0x80 ? '1' : '0'),     \
        (byte & 0x40 ? '1' : '0'), \
        (byte & 0x20 ? '1' : '0'), \
        (byte & 0x10 ? '1' : '0'), \
        (byte & 0x08 ? '1' : '0'), \
        (byte & 0x04 ? '1' : '0'), \
        (byte & 0x02 ? '1' : '0'), \
        (byte & 0x01 ? '1' : '0')

#define TWO_BYTES_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
#define TWO_BYTES_TO_BINARY(twoBytes)    \
    (twoBytes & 0x8000 ? '1' : '0'),     \
        (twoBytes & 0x4000 ? '1' : '0'), \
        (twoBytes & 0x2000 ? '1' : '0'), \
        (twoBytes & 0x1000 ? '1' : '0'), \
        (twoBytes & 0x0800 ? '1' : '0'), \
        (twoBytes & 0x0400 ? '1' : '0'), \
        (twoBytes & 0x0200 ? '1' : '0'), \
        (twoBytes & 0x0100 ? '1' : '0'), \
        (twoBytes & 0x0080 ? '1' : '0'), \
        (twoBytes & 0x0040 ? '1' : '0'), \
        (twoBytes & 0x0020 ? '1' : '0'), \
        (twoBytes & 0x0010 ? '1' : '0'), \
        (twoBytes & 0x0008 ? '1' : '0'), \
        (twoBytes & 0x0004 ? '1' : '0'), \
        (twoBytes & 0x0002 ? '1' : '0'), \
        (twoBytes & 0x0001 ? '1' : '0')

#define TWELVE_BIT_SENSOR_PATTERN "%c%c%c%c%c%c%c%c%c%c%c%c"
#define TO_TWELVE_BIT_SENSOR_PATTERN(twoBytes) \
    (twoBytes & 0x2000 ? '1' : '0'),           \
        (twoBytes & 0x1000 ? '1' : '0'),       \
        (twoBytes & 0x0800 ? '1' : '0'),       \
        (twoBytes & 0x0400 ? '1' : '0'),       \
        (twoBytes & 0x0200 ? '1' : '0'),       \
        (twoBytes & 0x0100 ? '1' : '0'),       \
        (twoBytes & 0x0080 ? '1' : '0'),       \
        (twoBytes & 0x0040 ? '1' : '0'),       \
        (twoBytes & 0x0020 ? '1' : '0'),       \
        (twoBytes & 0x0010 ? '1' : '0'),       \
        (twoBytes & 0x0008 ? '1' : '0'),       \
        (twoBytes & 0x0004 ? '1' : '0')

uint16_t getSensorReadings();
int getCalculatedError(uint16_t sensorReading, int fallbackError);
int isOutOfLine(uint16_t sensorReadings);
int isOnCenter(uint16_t sensorReadings);

// Patterns check if the outermost 3-4 sensors on either side are high.
// This identifies L-junctions/90°/Acute turns before the bot leaves the track.
// Bits: [unused][unused][s1][s2][s3][s4][s5][s6][s7][s8][s9][s10][s11][s12][unused][unused]
// Indices (shift): 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2
#define is90DegreeLeft(r)   ((r & 0b0011100000000000) == 0b0011100000000000)
#define is90DegreeRight(r)  ((r & 0b0000000000011100) == 0b0000000000011100)

#endif