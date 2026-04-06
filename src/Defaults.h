
#ifndef DEFAULTS_H
#define DEFAULTS_H

// ── Out-of-line recovery ───────────────────────────────────────────────────────
#define OUT_OF_LINE_ERROR_VALUE  20
#define LOOP_ESCAPE_ERROR_VALUE   3

// Maximum time (ms) the bot will spin CW or CCW during recovery before giving up.
// Prevents an infinite loop if the line is never found (e.g. hardware fault).
#define RECOVER_TIMEOUT_MS      3000

// Braking duration (ms) applied before each recovery spin (BRAKING_ENABLED = 1).
#define BRAKE_DURATION_MILLIS    100

// ── Stop patch behaviour ───────────────────────────────────────────────────────
// Time (ms) to drive forward to confirm the stop patch is real (not a false positive).
#define STOP_CHECK_DELAY         100

// Active brake duration (ms) applied once a stop patch is confirmed.
#define STOP_BRAKE_DURATION_MS  1000

// How long (ms) the bot remains stopped after a confirmed stop patch.
#define STOP_HALT_DURATION_MS  10000

// ── PID defaults ──────────────────────────────────────────────────────────────
#define DEFAULT_LOOP_DELAY   20
#define DEFAULT_KP           18
#define DEFAULT_KI            0
#define DEFAULT_KD           10

// Start at a safe speed; tune upward once the bot is tracking reliably on the track.
#define DEFAULT_MOTOR_SPEED 150

// ── PID integral clamp ────────────────────────────────────────────────────────
// Caps the accumulated integral term to prevent windup during long off-line events.
#define I_CLAMP 200

// ── Motor calibration trim ────────────────────────────────────────────────────
// Compensates for motor speed mismatch / mechanical drift.
// Positive = that side runs faster, negative = slower.
// These are the compile-time defaults; the runtime globals are initialised from these.
#define LEFT_MOTOR_OFFSET    0
#define RIGHT_MOTOR_OFFSET   0

// ── Track types ───────────────────────────────────────────────────────────────
#define WHITE_LINE_BLACK_TRACK  0
#define BLACK_LINE_WHITE_TRACK  1

// ── Turn speed reduction ──────────────────────────────────────────────────────
// Percentage by which base motor speed is reduced during CW / CCW recovery turns.
#define TURN_SPEED_REDUCTION_PERCENT  10

#endif