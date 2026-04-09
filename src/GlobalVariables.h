
#ifndef GLOBALVARIABLES_H
#define GLOBALVARIABLES_H

#include <Arduino.h>
#include "Defaults.h"

// Variables that can be updated via the app
extern int Kp;
extern int Ki;
extern int Kd;
extern int baseMotorSpeed;
extern int loopDelay;

// Variables that are used for calculations
extern int error;
extern int leftMotorOffset;
extern int rightMotorOffset;
extern int P;
extern int I;
extern int D;
extern int error_dir;
extern int previousError;
extern int lastRealError;   // last error when sensors were ACTIVE (not out-of-line placeholder)
                             // used to distinguish gap (≈0) from corner (large) in controlMotors()
extern int PID_value;
extern bool sensorInitialized;

// ── Loop/Intersection logic ──
extern uint8_t intersectionCount;
extern unsigned long lastIntersectionTime;
extern bool loopEscapeActive;

// This variable is used for tracking line inversions
// For black coloured tracks on white background this values is set to 1 and 0 for vice - versa
extern uint8_t trackType;

#endif