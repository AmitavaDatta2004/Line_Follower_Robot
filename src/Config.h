#pragma once

#ifndef CONFIG_H
#define CONFIG_H

// ─────────────────────────────────────────────────────────────────────────────
// USB Serial Monitor Logging (Arduino IDE Serial Monitor)
// Set to 1 to enable logs via USB cable → Arduino IDE Serial Monitor at 115200.
// Works completely independently of Bluetooth. Safe to use with BT at the same time.
// ─────────────────────────────────────────────────────────────────────────────
#define USB_SERIAL_LOGGING_ENABLED 1

// ── Bluetooth credentials ─────────────────────────────────────────────────────
#define BT_PIN   "9637"
#define BT_NAME  "HELIOS v1.0"

// ── ESP32 LEDC hardware PWM ───────────────────────────────────────────────────
// Using LEDC instead of analogWrite() gives configurable frequency and is the
// recommended approach for ESP32 (analogWrite is a compatibility shim only).
#define PWM_FREQ_HZ    5000   // Motor PWM frequency in Hz
#define PWM_RESOLUTION    8   // Bit depth (8-bit = duty cycle 0–255)

// Enable logging over Serial / Bluetooth
#define BLUETOOTH_LOGGING_ENABLED 1

// // Enable PID tuning over Serial / Bluetooth
// #define BLUETOOTH_TUNING_ENABLED 0
#define BLUETOOTH_TUNING_ENABLED 0

// Enable Communication over Serial ( Serial monitor / external bluetooth host )
#define USE_SERIAL_BLUETOOTH 0

// Enable Communication over Inbuilt bluetooth.
#define USE_INBUILT_BLUETOOTH 1

// THe last two configurations can co-exist.

#if ((BLUETOOTH_LOGGING_ENABLED == 0 && BLUETOOTH_TUNING_ENABLED == 0) && (USE_SERIAL_BLUETOOTH == 1))
#error "Serial Bluetooth should be disabled when both logging and PID tuning over bluetooth is disabled"
#endif

#if ((BLUETOOTH_LOGGING_ENABLED == 0 && BLUETOOTH_TUNING_ENABLED == 0) && (USE_INBUILT_BLUETOOTH == 1))
#error "Inbuilt Bluetooth should be disabled when both logging and PID tuning over bluetooth is disabled"
#endif

#if ((BLUETOOTH_LOGGING_ENABLED == 1 || BLUETOOTH_TUNING_ENABLED == 1) && (USE_INBUILT_BLUETOOTH == 0 && USE_SERIAL_BLUETOOTH == 0))
#error "Either Inbuilt bluetooth or Serial Bluetooth should be enabled when logging or tuning over bluetooth is enabled"
#endif

#if (USE_INBUILT_BLUETOOTH == 1 && (!defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)))
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// This config allows to reverse the default polarity of the motors
// #define REVERSE_MOTOR_POLARITY 1
#define REVERSE_MOTOR_POLARITY 1

// Allow the robot to reduce the speed by a certain percent when making a
// clockwise / counter clockwise rotation. The percent value is defined in Defaults.h
#define TURN_SPEED_REDUCTION_ENABLED 1

#define BRAKING_ENABLED 1    // Essential for 60° triangle/arrow corners — sheds momentum before recovery spin
#define GAPS_ENABLED 1
#define INVERSION_ENABLED 1

// Automatically reduce motor speed proportional to turn sharpness (error magnitude).
// effectiveSpeed = baseMotorSpeed - (abs(error) * SPEED_SCALE_FACTOR)
// Tune SPEED_SCALE_FACTOR and MIN_MOTOR_SPEED in Defaults.h.
#define DYNAMIC_SPEED_ENABLED 1

#define SERIAL_BAUD_RATE 115200

#endif