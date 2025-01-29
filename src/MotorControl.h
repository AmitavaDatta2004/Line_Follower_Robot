#pragma once

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

void moveStraight(int leftMotorSpeed, int rightMotorSpeed, int baseMotorSpeed);
void turnCCW(int leftMotorSpeed, int rightMotorSpeed, int baseMotorSpeed);
void turnCW(int leftMotorSpeed, int rightMotorSpeed, int baseMotorSpeed);
void shortBrake(int durationMillis);
void stop();

#endif