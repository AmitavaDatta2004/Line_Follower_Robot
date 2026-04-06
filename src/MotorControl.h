#pragma once

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

// Call once in setup() to configure LEDC PWM channels for both motors.
void initMotorPWM();

void moveStraight(int leftMotorSpeed, int rightMotorSpeed, int baseMotorSpeed);
void turnCCW(int leftMotorSpeed, int rightMotorSpeed, int baseMotorSpeed);
void turnCW(int leftMotorSpeed, int rightMotorSpeed, int baseMotorSpeed);
void shortBrake(int durationMillis);
void stop();

#endif