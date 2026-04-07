
#ifndef DEFAULTS_H
#define DEFAULTS_H

// ── Out-of-line recovery ───────────────────────────────────────────────────────
#define OUT_OF_LINE_ERROR_VALUE  20
#define LOOP_ESCAPE_ERROR_VALUE   3

// Maximum time (ms) the bot will spin CW or CCW during recovery before giving up.
// Triangle corners on HYPERDRIVE track require longer recovery window — 4 seconds.
#define RECOVER_TIMEOUT_MS      4000

// Braking duration (ms) applied before each recovery spin (BRAKING_ENABLED = 1).
// At 600 RPM / 200 PWM (~1.6 m/s), 200ms is needed to shed momentum before spinning.
#define BRAKE_DURATION_MILLIS    200

// ── Intersection & Loop Trap Handling ─────────────────────────────────────────
// At Y-junctions or crossovers where BOTH extreme left and right sensors trigger
// simultaneously, the bot needs a strict rule to avoid indecision or "loop traps".
// -1 = Always prefer Right (CW spin)
//  1 = Always prefer Left (CCW spin)
// For a standard loop that forks and merges, picking a consistent side guarantees 
// the bot exits the loop instead of endlessly circling.
#define INTERSECTION_TURN_BIAS   -1

// ── Gap handling ──────────────────────────────────────────────────────────────
// When sensors go dark, the bot first drives straight for GAP_TIMEOUT_MS before
// deciding it's a real corner. If the line returns within this window → gap handled,
// no braking, no recovery spin. If still dark → Phase 2 (brake + CW/CCW spin).
// At 200 PWM (~1.6 m/s): 150ms × 1.6 = ~24cm gap coverage.
// Right-angle turns: bot drives ~8cm past corner at MIN speed before recovery starts.
#define GAP_TIMEOUT_MS          150

// Maximum |lastRealError| to qualify as a gap (vs. a corner).
// Gap   → bot was going STRAIGHT → lastRealError ≈ 0  (≤ threshold) → Phase 1 runs.
// Corner → error was BUILDING UP  → lastRealError large (> threshold) → skip to Phase 2.
// Tune: higher = more turns treated as gaps (risky); lower = fewer gaps recognised.
#define GAP_ERROR_THRESHOLD       3

// ── Stop patch behaviour ───────────────────────────────────────────────────────
// Time (ms) to drive forward to confirm the stop patch is real (not a false positive).
#define STOP_CHECK_DELAY         100

// Active brake duration (ms) applied once a stop patch is confirmed.
#define STOP_BRAKE_DURATION_MS  1000

// How long (ms) the bot remains stopped after a confirmed stop patch.
#define STOP_HALT_DURATION_MS  10000

// ── PID defaults ──────────────────────────────────────────────────────────────
// Tuned for HYPERDRIVE track (600 RPM, 65mm wheels, PWM 0-255):
//   Kp=25  — aggressive enough to correct fast at full speed
//   Ki=0   — track is too dynamic; integral causes corner windup
//   Kd=10  — capped by D_CLAMP; balanced for spiral & 60° corners
#define DEFAULT_LOOP_DELAY   20
#define DEFAULT_KP           25
#define DEFAULT_KI            0
#define DEFAULT_KD           10

// Start at 200 (not 255) for first runs — scale up once bot is reliable.
// At 600 RPM: PWM 200 ≈ 1.6 m/s. Dynamic scaling slows it further on turns.
#define DEFAULT_MOTOR_SPEED 200

// ── PID integral clamp ────────────────────────────────────────────────────────
// Caps the accumulated integral term to prevent windup during long off-line events.
#define I_CLAMP 200

// ── PID derivative clamp ──────────────────────────────────────────────────────
// Caps the MAXIMUM contribution the derivative term (Kd*D) can add to PID_value.
// Tighter value (45) needed for HYPERDRIVE track — triangle 60° corners cause a
// more violent D spike than obtuse turns (error: -11 → +11 in a single loop).
#define D_CLAMP 45

// ── Motor calibration trim ────────────────────────────────────────────────────
// Compensates for motor speed mismatch / mechanical drift.
// Positive = that side runs faster, negative = slower.
// These are the compile-time defaults; the runtime globals are initialised from these.
#define LEFT_MOTOR_OFFSET    0
#define RIGHT_MOTOR_OFFSET   0

// ── Dynamic speed scaling ──────────────────────────────────────────────────────
// Primary slowdown — proportional to current error magnitude:
//   effectiveSpeed = base - (|error| × SPEED_SCALE_FACTOR)
// Predictive slowdown — also proportional to D (rate of error change):
//   effectiveSpeed -= (|D| × D_SPEED_FACTOR)
// This gives advance warning at triangle/arrow corners where error is still low
// but D is already spiking (line starting to shift before sensor fully detects it).
// Calibrated for HYPERDRIVE track — 600 RPM / 65mm wheels / PWM 0-255:
//   Base 200 = ~1.6 m/s. Triangle 60° corners require MIN speed of 70.
//
//   error=0  (straight)    -> speed=200  (~1.6 m/s)
//   error=5  (circle/S)    -> speed=100  (~0.8 m/s)  ← significant slowdown
//   error=10 (spiral apex) -> speed= 70  (floored)
//   error=11 (triangle tip)-> speed= 70  (floored)
#define SPEED_SCALE_FACTOR  20
#define MIN_MOTOR_SPEED     70   // enough torque for 60° triangle corners

// Predictive speed factor — additional speed reduction per unit of D (rate of error change).
// D_SPEED_FACTOR=5 means: if the error is changing at rate 4/loop, reduce speed by 20 extra.
// Set to 0 to disable predictive slowdown.
#define D_SPEED_FACTOR       5

// ── Serial logging throttle ───────────────────────────────────────────────────────
// How many PID loops to skip between serial log prints.
// At DEFAULT_LOOP_DELAY=20ms, LOG_INTERVAL_LOOPS=25 → logs every 500ms.
// Without this, [SPEED] logs fire 50×/second and flood the Serial Monitor.
#define LOG_INTERVAL_LOOPS  25

// ── Track types ───────────────────────────────────────────────────────────────
#define WHITE_LINE_BLACK_TRACK  0
#define BLACK_LINE_WHITE_TRACK  1

// ── Turn speed reduction ────────────────────────────────────────────────────────
// Percentage by which base motor speed is reduced during CW / CCW recovery turns.
// At 10% (old): recovery spin = 180 PWM at base 200 → too fast for 60° corners.
// At 40% (new): recovery spin = 120 PWM → slow enough to detect line precisely.
#define TURN_SPEED_REDUCTION_PERCENT  40

#endif