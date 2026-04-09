#include "MotorControl.h"
#include "PinDefinitions.h"
#include "Config.h"
#include "Defaults.h"
#include "GlobalVariables.h"

// ─────────────────────────────────────────────────────────────────────────────
// initMotorPWM
// Configures both LEDC channels once during setup().
// Must be called before any motor function is used.
// ─────────────────────────────────────────────────────────────────────────────
void initMotorPWM()
{
    ledcSetup(LEFT_MOTOR_PWM_CHANNEL,  PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcAttachPin(LEFT_MOTOR_PWM_PIN,  LEFT_MOTOR_PWM_CHANNEL);

    ledcSetup(RIGHT_MOTOR_PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcAttachPin(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_PWM_CHANNEL);
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal helper — applies runtime motor offset trim and clamps to [0, base].
// leftMotorOffset / rightMotorOffset are global runtime variables (default = 0).
// Adjust them at run-time (or set LEFT/RIGHT_MOTOR_OFFSET in Defaults.h) to
// compensate for motor mismatch / mechanical drift.
// ─────────────────────────────────────────────────────────────────────────────
static inline int applyLeftOffset(int speed, int base)
{
    return constrain(speed + leftMotorOffset, -base, base);
}
static inline int applyRightOffset(int speed, int base)
{
    return constrain(speed + rightMotorOffset, -base, base);
}

static inline void setMotorDirectionAndPWM(int channel, int speed, int pin1, int pin2) 
{
    if (speed >= 0) {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    } else {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
    }
    ledcWrite(channel, abs(speed));
}

void moveStraight(int leftMotorSpeed, int rightMotorSpeed, int baseMotorSpeed)
{
    int finalLeft = applyLeftOffset(leftMotorSpeed, baseMotorSpeed);
    int finalRight = applyRightOffset(rightMotorSpeed, baseMotorSpeed);
    
    setMotorDirectionAndPWM(LEFT_MOTOR_PWM_CHANNEL, finalLeft, LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2);
    setMotorDirectionAndPWM(RIGHT_MOTOR_PWM_CHANNEL, finalRight, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2);
}

void turnCCW(int leftMotorSpeed, int rightMotorSpeed, int baseMotorSpeed)
{
#if (TURN_SPEED_REDUCTION_ENABLED == 1)
    leftMotorSpeed  = (leftMotorSpeed  * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
    rightMotorSpeed = (rightMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif

    int finalLeft = applyLeftOffset(leftMotorSpeed, baseMotorSpeed);
    int finalRight = applyRightOffset(rightMotorSpeed, baseMotorSpeed);

    // CCW: Left backward, Right forward
    setMotorDirectionAndPWM(LEFT_MOTOR_PWM_CHANNEL, -abs(finalLeft), LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2);
    setMotorDirectionAndPWM(RIGHT_MOTOR_PWM_CHANNEL, abs(finalRight), RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2);
}

void turnCW(int leftMotorSpeed, int rightMotorSpeed, int baseMotorSpeed)
{
#if (TURN_SPEED_REDUCTION_ENABLED == 1)
    leftMotorSpeed  = (leftMotorSpeed  * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
    rightMotorSpeed = (rightMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif

    int finalLeft = applyLeftOffset(leftMotorSpeed, baseMotorSpeed);
    int finalRight = applyRightOffset(rightMotorSpeed, baseMotorSpeed);

    // CW: Left forward, Right backward
    setMotorDirectionAndPWM(LEFT_MOTOR_PWM_CHANNEL, abs(finalLeft), LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2);
    setMotorDirectionAndPWM(RIGHT_MOTOR_PWM_CHANNEL, -abs(finalRight), RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2);
}

void shortBrake(int durationMillis)
{
    ledcWrite(LEFT_MOTOR_PWM_CHANNEL,  0);
    ledcWrite(RIGHT_MOTOR_PWM_CHANNEL, 0);

    // Both H-bridge inputs HIGH → active brake
    digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN_2, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN_2, HIGH);

    delay(durationMillis);
}

void stop()
{
    // Cut drive signals first, then disable H-bridge
    ledcWrite(LEFT_MOTOR_PWM_CHANNEL,  0);
    ledcWrite(RIGHT_MOTOR_PWM_CHANNEL, 0);

    digitalWrite(LEFT_MOTOR_PIN_1, LOW);
    digitalWrite(LEFT_MOTOR_PIN_2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_2, LOW);
}