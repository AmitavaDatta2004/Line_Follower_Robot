
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include "GlobalVariables.h"
#include "PinDefinitions.h"
#include "Config.h"
#include "Wire.h"

bool sensorInitialized = false;

uint8_t shiftIn165(gpio_num_t dataPin, gpio_num_t clockPin, uint8_t bitOrder)
{
    uint8_t value = 0;
    uint8_t i;

    for (i = 0; i < 8; ++i)
    {
        uint8_t r = gpio_get_level(dataPin);
        if (bitOrder == LSBFIRST)
            value |= r << i;
        else
            value |= r << (7 - i);
        gpio_set_level(clockPin, 1);
        delayMicroseconds(3);   // Hold HIGH long enough for 74HC165 to register the pulse.
        gpio_set_level(clockPin, 0); // Without this, aged/slow hardware never shifts — same bit read 16×.
        delayMicroseconds(3);   // Also hold LOW briefly before next read (setup time).
    }

    return value;
}

uint8_t horizontalFlip(uint8_t byte)
{
    uint8_t flippedByte = 0x00;
    for (int i = 0; i < 4; i++)
    {
        uint8_t bs = (byte >> i) & 0x01;
        uint8_t be = (byte >> (7 - i)) & 0x01;
        flippedByte |= (bs << (7 - i)) | (be << i);
    }
    return flippedByte;
}

uint16_t getProcessedSensorData(uint8_t leftHalfPinState, uint8_t rightHalfPinState)
{
    leftHalfPinState = horizontalFlip(leftHalfPinState);
    rightHalfPinState = horizontalFlip(rightHalfPinState);

    uint16_t processedSensorData = leftHalfPinState << 8 | rightHalfPinState;

    return processedSensorData;
}

// returns a 2-byte data where each bit represent the state of each sensor
uint16_t getSensorReadings()
{
    uint16_t reading = 0x0000;

    if (!sensorInitialized)
    {
        gpio_config_t io_config;
        io_config.pin_bit_mask = (1ULL << DATA_PIN);
        io_config.mode = GPIO_MODE_INPUT;
        io_config.pull_up_en = GPIO_PULLUP_DISABLE;    // Fixed: 74HC165 drives DATA_PIN actively; pull-up was forcing all-1s
        io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_config.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_config);

        io_config.pin_bit_mask = (1ULL << CLOCK_PIN) | (1ULL << CLOCK_ENABLE_PIN) | (1ULL << LOAD_PIN);
        io_config.mode = GPIO_MODE_OUTPUT;
        io_config.pull_up_en = GPIO_PULLUP_DISABLE;
        io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_config.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_config);
        delay(10);
        // 74HC165 CLK INH is active LOW: LOW = clock enabled, HIGH = clock disabled.
        // Keep LOW (enabled) as the idle state so shifts work correctly.
        gpio_set_level(CLOCK_ENABLE_PIN, 0);

        sensorInitialized = true;
    }

    // Write pulse to load pin
    gpio_set_level(LOAD_PIN, 0);
    delayMicroseconds(50);
    gpio_set_level(LOAD_PIN, 1);
    delayMicroseconds(50);

    // Get data from 74HC165
    gpio_set_level(CLOCK_ENABLE_PIN, 0);   // Enable clock (CLK INH active LOW)
    uint8_t leftHalfPinState  = shiftIn165(DATA_PIN, CLOCK_PIN, LSBFIRST);
    uint8_t rightHalfPinState = shiftIn165(DATA_PIN, CLOCK_PIN, LSBFIRST);
    // Leave CLK INH LOW (idle-enabled) — do not disable after reading

    reading = getProcessedSensorData(leftHalfPinState, rightHalfPinState);
    reading &= 0b0011111111111100;

#if INVERSION_ENABLED == 1

    // Detect track type from well-known center-strip patterns.
    // A single continuous if-else chain is used so only one branch fires per reading.
    if      (reading == 0b0000110000000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000011000000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000111000000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000111100000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000010000000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000011100000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000001110000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000001100000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000011110000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000000100000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000000110000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000000111000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000000011000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000001111000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000000010000000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000000011100000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000000001100000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000000001110000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000000011110000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0000000000100000) trackType = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b0011001111111100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011100111111100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011000111111100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011101111111100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011100011111100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011110001111100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011110011111100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011111011111100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011111001111100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011111000111100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011111100111100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011111101111100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011111100011100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011111110011100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011111110001100) trackType = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b0011111111011100) trackType = BLACK_LINE_WHITE_TRACK;

#endif

    // invert the sensor readings
    if (trackType == BLACK_LINE_WHITE_TRACK)
    {
        reading ^= 0b0011111111111100;
    }

    return reading;
}

// Calculates PID error from a pre-read sensor snapshot.
// Pass the value already obtained from getSensorReadings() to avoid a second hardware read.
int getCalculatedError(uint16_t sensorReading, int fallbackError)
{
    int numeratorSum = 0, denominatorSum = 0;

    // Assuming that the the MSB represents the index 0 of the array (left to right)
    for (int i = 2; i <= (SENSOR_COUNT + 1); i++)
    {
        // Check the digital values at the ith bit
        uint16_t sensorValue = ((sensorReading & (1 << (15 - i))) >> (15 - i));
        numeratorSum += (i - 1) * 100 * (int)sensorValue;
        denominatorSum += sensorValue;
    }

    int error = fallbackError;
    if (denominatorSum != 0)
        error = ((numeratorSum / (denominatorSum * 50)) - (SENSOR_COUNT + 1));
    return error;
}

int isOutOfLine(uint16_t sensorReadings)
{
    // Any active sensor bit means the bot is touching the line somewhere.
    // Zero reading (all bits 0) means no sensors see the line → out of line.
    // This replaces the previous hardcoded pattern list, which missed valid
    // curve / wide-line patterns and could exit recovery too early.
    return (sensorReadings == 0) ? 1 : 0;
}
