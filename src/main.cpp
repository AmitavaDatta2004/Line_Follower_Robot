#include <Arduino.h>
#include "PinDefinitions.h"
#include "Config.h"
#include "Defaults.h"
#include "GlobalVariables.h"
#include "MotorControl.h"
#include "Sensor.h"
#include "Wire.h"
#include "FastLED.h"

#if OLED_ENABLED == 1
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins are defined in Config.h and passed to Wire.begin()
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#endif

// Include ArduinoJson only when bluetooth logging / tuning is enabled.
#if BLUETOOTH_LOGGING_ENABLED == 1 || BLUETOOTH_TUNING_ENABLED == 1
#include <ArduinoJson.h>

// TX_DOC_MAX_DATA_LEN removed — logging now uses printf; txDoc was dead code.
#define RX_DOC_MAX_DATA_LEN 192

#if USE_INBUILT_BLUETOOTH == 1
#include <BluetoothSerial.h>
#endif
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Macro definitions for checkpoint / event detection
// ─────────────────────────────────────────────────────────────────────────────
// MID_8_SENSORS_* macros removed — they were defined but never used.

#define MID_6_SENSORS_HIGH (s4 == 1 && s5 == 1 && s6 == 1 && s7 == 1 && s8 == 1 && s9 == 1)
#define MID_6_SENSORS_LOW  (s4 == 0 && s5 == 0 && s6 == 0 && s7 == 0 && s8 == 0 && s9 == 0)

// #########################################################################################################################
///////////////////////////////////////////////     VARIABLE DEFINITIONS     ///////////////////////////////////////////////
// #########################################################################################################################

// rxDoc used only when BT tuning is enabled (TX doc removed — was dead code).
#if BLUETOOTH_TUNING_ENABLED == 1
StaticJsonDocument<RX_DOC_MAX_DATA_LEN> rxDoc;
#endif

int Kp = DEFAULT_KP;
int Ki = DEFAULT_KI;
int Kd = DEFAULT_KD;
int baseMotorSpeed = DEFAULT_MOTOR_SPEED;
int loopDelay      = DEFAULT_LOOP_DELAY;
int error          = 0;

// Initialised from Defaults.h compile-time constants; can be tweaked at runtime.
int leftMotorOffset  = LEFT_MOTOR_OFFSET;
int rightMotorOffset = RIGHT_MOTOR_OFFSET;

int P             = 0;
int I             = 0;
int D             = 0;
int error_dir     = 0;
int previousError = 0;
int lastRealError  = 0;  // last error when sensors actively saw the line (≠ OUT_OF_LINE placeholder)
int PID_value     = 0;

// Make sure to update this variable according to the type of track the LFR is about to run on.
// When the macro for enabling inversion is set to 1, this variable updates itself automatically.
// ⚠️  Set this to the ACTUAL track type you start on. If wrong, all sensor
//      readings will be inverted and the bot will spin in recovery immediately.
uint8_t trackType = WHITE_LINE_BLACK_TRACK;   // ← change to BLACK_LINE_WHITE_TRACK if needed

CRGB leds[NUM_LEDS];

#if ((BLUETOOTH_LOGGING_ENABLED == 1 || BLUETOOTH_TUNING_ENABLED == 1) && USE_INBUILT_BLUETOOTH == 1)
BluetoothSerial SerialBT;
#endif

// #########################################################################################################################
///////////////////////////////////////////////     FUNCTION DEFINITIONS     ///////////////////////////////////////////////
// #########################################################################################################################

/**
 * @brief  Indicates inversion is active: LEDs 0-1 glow red.
 */
void indicateInversionOn()
{
	leds[0] = CRGB(255, 0, 0);
	leds[1] = CRGB(255, 0, 0);
	FastLED.show();
}

/**
 * @brief  Clears the inversion indicator LEDs only.
 */
void indicateInversionOff()
{
	leds[0] = CRGB(0, 0, 0);
	leds[1] = CRGB(0, 0, 0);
	FastLED.show();
}

/**
 * @brief  Indicates a checkpoint / event: buzzer + blue LED + LEDs 2-5 purple.
 */
void indicateOn()
{
	digitalWrite(BUZZER, HIGH);
	digitalWrite(BLUE_LED, HIGH);
	leds[2] = CRGB(80, 0, 255);
	leds[3] = CRGB(80, 0, 255);
	leds[4] = CRGB(80, 0, 255);
	leds[5] = CRGB(80, 0, 255);
	FastLED.show();
}

/**
 * @brief  Clears the checkpoint indicator only.
 */
void indicateOff()
{
	digitalWrite(BUZZER, LOW);
	digitalWrite(BLUE_LED, LOW);
	leds[2] = CRGB(0, 0, 0);
	leds[3] = CRGB(0, 0, 0);
	leds[4] = CRGB(0, 0, 0);
	leds[5] = CRGB(0, 0, 0);
	FastLED.show();
}

/**
 * @brief  Step 1 — Read sensors, update error and error_dir.
 *
 *  - error_dir is now updated from the full-width PID error (more reliable than
 *    edge sensors only). Edge sensors s1/s12 are kept as a secondary fallback.
 *  - When all sensors are dark AND error_dir is still 0, previousError is used
 *    as a last-resort direction hint so the bot always picks a recovery direction.
 */
void readSensors()
{
	uint16_t sensorData = getSensorReadings();
	error = getCalculatedError(sensorData, 0);

	// Extract individual sensor bits (s1 = leftmost, s12 = rightmost).
	int s1  = (sensorData & (1 << 13)) >> 13;
	int s2  = (sensorData & (1 << 12)) >> 12;  // kept for future use
	int s3  = (sensorData & (1 << 11)) >> 11;
	int s4  = (sensorData & (1 << 10)) >> 10;
	int s5  = (sensorData & (1 <<  9)) >>  9;
	int s6  = (sensorData & (1 <<  8)) >>  8;
	int s7  = (sensorData & (1 <<  7)) >>  7;
	int s8  = (sensorData & (1 <<  6)) >>  6;
	int s9  = (sensorData & (1 <<  5)) >>  5;
	int s10 = (sensorData & (1 <<  4)) >>  4;
	int s11 = (sensorData & (1 <<  3)) >>  3;  // kept for future use
	int s12 = (sensorData & (1 <<  2)) >>  2;

	// Suppress unused-variable warnings (s2, s3, s11 reserved for user extension).
	(void)s2; (void)s3; (void)s11;

	// ── Update direction memory ───────────────────────────────────────────────
	// Primary: use the full-width PID error — more informative than edge sensors.
	//   error > 0  →  line is to the right  →  error_dir = -1  →  recover CW
	//   error < 0  →  line is to the left   →  error_dir = +1  →  recover CCW
	if (sensorData != 0 && error != 0)
		error_dir = (error > 0) ? -1 : 1;
	else if (s1 != s12)   // Secondary fallback: raw edge sensors
		error_dir = s1 - s12;

	// ── Checkpoint / inversion indicators ────────────────────────────────────
	if (MID_6_SENSORS_HIGH)
		indicateOn();
	else
		indicateOff();

	if (trackType == WHITE_LINE_BLACK_TRACK)
		indicateInversionOn();
	else
		indicateInversionOff();

	// ── Save last real error before it may be overwritten by OUT_OF_LINE value ─
	// lastRealError holds the most recent error captured while sensors saw the line.
	// Checked in controlMotors() to tell a gap (lastRealError≈0) from a corner.
	if (sensorData != 0)
		lastRealError = error;

	// ── Out-of-line: set recovery error ──────────────────────────────────────
	if (sensorData == 0b0000000000000000)
	{
		if (error_dir < 0)
			error = OUT_OF_LINE_ERROR_VALUE;
		else if (error_dir > 0)
			error = -1 * OUT_OF_LINE_ERROR_VALUE;
		else
			// error_dir hasn't been set yet (start of run, or just after a gap reset).
			// Use the sign of the last known PID error as a direction hint.
			error = (previousError >= 0) ? OUT_OF_LINE_ERROR_VALUE
			                             : -1 * OUT_OF_LINE_ERROR_VALUE;
	}
	// ── Stop patch: all 12 active sensors ────────────────────────────────────
	else if (sensorData == 0b0011111111111100)
	{
		// Drive forward briefly to confirm it's a real stop patch, not a glitch.
		moveStraight(baseMotorSpeed, baseMotorSpeed, baseMotorSpeed);
		delay(STOP_CHECK_DELAY);
		uint16_t sensorDataAgain = getSensorReadings();
		if (sensorDataAgain == 0b0011111111111100)
		{
			indicateOff();
			shortBrake(STOP_BRAKE_DURATION_MS);
			stop();

#if OLED_ENABLED == 1
			display.clearDisplay();
			display.setTextSize(2);
			display.setTextColor(SSD1306_WHITE);
			display.setCursor(20, 25);
			display.println(F("FINISH!"));
			display.display();
#endif
			delay(STOP_HALT_DURATION_MS);
		}
	}

	// ── USB Serial logging ────────────────────────────────────────────────────
#if BLUETOOTH_LOGGING_ENABLED == 1

#if USE_SERIAL_BLUETOOTH == 1
	{
		char ss[16];
		sprintf(ss, "" TWELVE_BIT_SENSOR_PATTERN, TO_TWELVE_BIT_SENSOR_PATTERN(sensorData));
		Serial.print(F("I|Readings : "));
		Serial.print(String(ss));
		Serial.print(F(" | Error : "));
		Serial.print(error);
		Serial.print(F(" | Inv : "));
		Serial.println(trackType == WHITE_LINE_BLACK_TRACK ? F("WHITE_LINE_BLACK_TRACK") : F("BLACK_LINE_WHITE_TRACK"));
	}
#endif

#if USE_INBUILT_BLUETOOTH == 1
	SerialBT.printf("I|Readings : " TWELVE_BIT_SENSOR_PATTERN "| Error : %d | Inv :  %s\n",
	                TO_TWELVE_BIT_SENSOR_PATTERN(sensorData),
	                error, trackType == WHITE_LINE_BLACK_TRACK ? "WHITE_LINE_BLACK_TRACK" : "BLACK_LINE_WHITE_TRACK");
#endif

#endif

#if USB_SERIAL_LOGGING_ENABLED == 1
	{
		char usbBuf[48];
		sprintf(usbBuf, "" TWELVE_BIT_SENSOR_PATTERN, TO_TWELVE_BIT_SENSOR_PATTERN(sensorData));
		Serial.print(F("[SENSOR] "));
		Serial.print(usbBuf);
		Serial.print(F("  err="));
		Serial.print(error);
		Serial.print(F("  track="));
		Serial.println(trackType == WHITE_LINE_BLACK_TRACK ? F("WHITE/BLACK") : F("BLACK/WHITE"));
	}
#endif
}

/**
 * @brief  Step 2 — Compute PID_value from the current error.
 */
void calculatePID()
{
	P = error;

	if (error == 0)
		I = 0;
	else
		I = I + error;

	I = constrain(I, -I_CLAMP, I_CLAMP);

	D = error - previousError;

	// Clamp the derivative contribution BEFORE adding to PID.
	// Prevents "derivative kick": at an obtuse turn apex the error sign reverses
	// suddenly (e.g. -9 → +4), making D spike to +13 and Kd*D = 130 — which
	// fires a massive correction in the WRONG direction.
	// D_CLAMP caps this spike while leaving Kp and Ki completely unaffected.
	int D_term = constrain(Kd * D, -D_CLAMP, D_CLAMP);

	PID_value = (Kp * P) + (Ki * I) + D_term;
	PID_value = constrain(PID_value, -255, 255);
	previousError = error;
}

/**
 * @brief  Step 3 — Drive motors based on PID_value / recovery error.
 *
 *  Recovery loops now have a RECOVER_TIMEOUT_MS timeout so a hardware fault
 *  cannot lock the bot into an infinite spin.
 *  Motor offsets are applied inside the motor functions (MotorControl.cpp),
 *  so they are NOT subtracted here.
 */
void controlMotors()
{
	if (error == OUT_OF_LINE_ERROR_VALUE || error == (-1 * OUT_OF_LINE_ERROR_VALUE))
	{
		// ══════════════════════════════════════════════════════════════════════
		// PHASE 1 — Gap Test  (only runs when GAPS_ENABLED == 1)
		// Drive straight at full base speed for GAP_TIMEOUT_MS.
		// • Line returns within window  → it was a track gap.  No brake, no spin.
		// • Still dark after window     → real line-loss (corner). Go to Phase 2.
		// This prevents the 200ms brake + recovery spin firing at every dotted gap.
		// ══════════════════════════════════════════════════════════════════════
#if GAPS_ENABLED == 1
		// Gate: only do Phase 1 if the bot was travelling STRAIGHT when sensors went dark.
		// Gap   → lastRealError ≈ 0 (straight travel) → drive straight to cross it.
		// Corner → lastRealError was building up (large) → skip straight to Phase 2.
		if (abs(lastRealError) <= GAP_ERROR_THRESHOLD)
		{
			uint16_t      gapSensor = getSensorReadings();
			unsigned long gapStart  = millis();

			while (isOutOfLine(gapSensor) && (millis() - gapStart < GAP_TIMEOUT_MS))
			{
				// Drive straight at full speed — get over the gap fast.
				moveStraight(baseMotorSpeed, baseMotorSpeed, baseMotorSpeed);
				gapSensor = getSensorReadings();
			}

			if (!isOutOfLine(gapSensor))
			{
				// ✅ Line found again within the timeout → it was a gap.
				error_dir = 0;   // reset so next out-of-line uses fresh sensor data
#if USB_SERIAL_LOGGING_ENABLED == 1
				Serial.println(F("[GAP] Crossed ✓ — resuming PID"));
#endif
				return;   // exit controlMotors(); next loop() call handles PID
			}

			// Still dark → not a gap. Fall through to recovery spin.
#if USB_SERIAL_LOGGING_ENABLED == 1
			Serial.println(F("[GAP] Still dark → starting recovery spin"));
#endif
		}
		else
		{
#if USB_SERIAL_LOGGING_ENABLED == 1
			Serial.printf("[TURN] lastRealError=%d > threshold=%d → skipping gap test\n",
			              lastRealError, GAP_ERROR_THRESHOLD);
#endif
		}
#endif  // GAPS_ENABLED

		// ══════════════════════════════════════════════════════════════════════
		// PHASE 2 — Recovery Spin
		// Real line-loss (corner, overshoot, or hardware fault).
		// Brake first to shed momentum, then spin toward the last known line side.
		// ══════════════════════════════════════════════════════════════════════
#if BRAKING_ENABLED == 1
		shortBrake(BRAKE_DURATION_MILLIS);
#endif

		if (error == OUT_OF_LINE_ERROR_VALUE)
		{
#if USB_SERIAL_LOGGING_ENABLED == 1
			Serial.println(F("[RECOVERY] Turning CW"));
#endif
#if BLUETOOTH_LOGGING_ENABLED == 1 && USE_INBUILT_BLUETOOTH == 1
			SerialBT.println(F("E|Turning Clockwise"));
#endif
			uint16_t      sensorReadings = getSensorReadings();
			unsigned long recoveryStart  = millis();
			while (isOutOfLine(sensorReadings) &&
			       (millis() - recoveryStart < RECOVER_TIMEOUT_MS))
			{
				turnCW(baseMotorSpeed, baseMotorSpeed, baseMotorSpeed);
				sensorReadings = getSensorReadings();
			}
		}
		else
		{
#if USB_SERIAL_LOGGING_ENABLED == 1
			Serial.println(F("[RECOVERY] Turning CCW"));
#endif
#if BLUETOOTH_LOGGING_ENABLED == 1 && USE_INBUILT_BLUETOOTH == 1
			SerialBT.println(F("E|Turning Counter Clockwise"));
#endif
			uint16_t      sensorReadings = getSensorReadings();
			unsigned long recoveryStart  = millis();
			while (isOutOfLine(sensorReadings) &&
			       (millis() - recoveryStart < RECOVER_TIMEOUT_MS))
			{
				turnCCW(baseMotorSpeed, baseMotorSpeed, baseMotorSpeed);
				sensorReadings = getSensorReadings();
			}
		}
	}
	else
	{
		// ── Dynamic speed scaling ─────────────────────────────────────────────
		// Reduces effective speed proportional to turn sharpness so the bot
		// naturally slows before curves and accelerates on straight sections.
		// Formula:  effectiveSpeed = baseMotorSpeed - (|error| × SPEED_SCALE_FACTOR)
		// Individual motors can still exceed effectiveSpeed for differential
		// correction — baseMotorSpeed is kept as the upper clamp in moveStraight().
#if DYNAMIC_SPEED_ENABLED == 1
		// Primary: slow down proportional to current error magnitude.
		int effectiveSpeed = baseMotorSpeed - (abs(error) * SPEED_SCALE_FACTOR);
		// Predictive: also slow down based on how fast the error is changing (D term).
		// This brakes BEFORE the error peaks — critical for 60° triangle/arrow corners.
		effectiveSpeed    -= (abs(D) * D_SPEED_FACTOR);
		effectiveSpeed     = constrain(effectiveSpeed, MIN_MOTOR_SPEED, baseMotorSpeed);
#else
		int effectiveSpeed = baseMotorSpeed;
#endif

#if USB_SERIAL_LOGGING_ENABLED == 1
		// Throttle: only log every LOG_INTERVAL_LOOPS loops (~500ms) to avoid flooding.
		static uint16_t logCounter = 0;
		if (++logCounter >= LOG_INTERVAL_LOOPS)
		{
			logCounter = 0;
			Serial.printf("[SPEED] eff=%d  base=%d  err=%d  D=%d\n",
			              effectiveSpeed, baseMotorSpeed, error, D);
		}
#endif

		int leftMotorSpeed  = effectiveSpeed + PID_value;
		int rightMotorSpeed = effectiveSpeed - PID_value;

		moveStraight(leftMotorSpeed, rightMotorSpeed, baseMotorSpeed);

		// Extra delay when D != 0 makes the derivative term more impactful.
		if (D != 0)
			delay(loopDelay);
	}
}

// #########################################################################################################################
//////////////////////////////////////////////////////     SETUP     //////////////////////////////////////////////////////
// #########################################################################################################################

void setup()
{
	pinMode(LEFT_MOTOR_PIN_1,    OUTPUT);
	pinMode(LEFT_MOTOR_PIN_2,    OUTPUT);
	pinMode(RIGHT_MOTOR_PIN_1,   OUTPUT);
	pinMode(RIGHT_MOTOR_PIN_2,   OUTPUT);

	// initMotorPWM() replaces analogWrite() — configures LEDC hardware PWM.
	initMotorPWM();

	pinMode(BLUE_LED, OUTPUT);
	pinMode(BUZZER,   OUTPUT);

#if USB_SERIAL_LOGGING_ENABLED == 1
	Serial.begin(SERIAL_BAUD_RATE);
	Serial.println(F("[BOOT] Line Follower Robot started."));
	Serial.println(F("[BOOT] USB Serial logging active at 115200 baud."));
#endif

#if BLUETOOTH_LOGGING_ENABLED == 1 || BLUETOOTH_TUNING_ENABLED == 1

#if USE_SERIAL_BLUETOOTH == 1
	// Only start Serial here if USB logging hasn't already done it.
#if USB_SERIAL_LOGGING_ENABLED == 0
	Serial.begin(SERIAL_BAUD_RATE);
#endif
#endif

#if USE_INBUILT_BLUETOOTH == 1
	SerialBT.setPin(BT_PIN);    // defined in Config.h
	SerialBT.begin(BT_NAME);   // defined in Config.h
#endif

#endif

	FastLED.addLeds<WS2812, NEOPIXEL_LED_PIN, GRB>(leds, NUM_LEDS);

#if OLED_ENABLED == 1
	// Initialize I2C with the custom pins
	Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
	
	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
#if USB_SERIAL_LOGGING_ENABLED == 1
		Serial.println(F("SSD1306 allocation failed"));
#endif
	} else {
		display.clearDisplay();
		display.setTextSize(2);
		display.setTextColor(SSD1306_WHITE);
		display.setCursor(20, 20);
		display.println(F("HELIOS"));
		display.setTextSize(1);
		display.setCursor(35, 40);
		display.println(F("v1.0 Ready"));
		display.display();
		
		delay(1500); // Hold splash screen briefly
		
		// Show active configuration before the run starts
		display.clearDisplay();
		display.setTextSize(1);
		display.setCursor(0, 5);
		display.println(F("- CURRENT CONFIG -"));
		
		display.setCursor(0, 25);
		display.printf("KP: %d  KI: %d  KD: %d\n", Kp, Ki, Kd);
		
		display.setCursor(0, 40);
		display.printf("SPD: %d  TRK: %s\n", baseMotorSpeed, (trackType == WHITE_LINE_BLACK_TRACK) ? "Inv" : "Nrm");
		
#if USE_INBUILT_BLUETOOTH == 1 || USE_SERIAL_BLUETOOTH == 1
		display.setCursor(0, 55);
		display.print(F("Status: BT Active"));
#endif
		
		display.display();
	}
#endif
}

// #########################################################################################################################
//////////////////////////////////////////////////////     LOOP     ///////////////////////////////////////////////////////
// #########################################################################################################################

void loop()
{

#if BLUETOOTH_TUNING_ENABLED == 1

	// Sample message format: {"P":18,"I":0,"D":10,"ms":150,"de":20}

#if USE_SERIAL_BLUETOOTH == 1
	if (Serial.available())
	{
		Serial.flush();
		String data = Serial.readStringUntil('\n');

		Serial.print(F("I|Data received : "));
		Serial.println(data);

		DeserializationError btError = deserializeJson(rxDoc, data);

		if (btError)
		{
			Serial.print(F("E|deserialize json failed : "));
			Serial.println(btError.f_str());
			rxDoc.clear();
			return;
		}

		// Validate and constrain all incoming values to safe ranges.
		Kp            = constrain((int)rxDoc["P"],   0, 200);
		Ki            = constrain((int)rxDoc["I"],   0, 200);
		Kd            = constrain((int)rxDoc["D"],   0, 200);
		baseMotorSpeed = constrain((int)rxDoc["ms"], 0, 255);
		loopDelay     = constrain((int)rxDoc["de"],  0, 500);
		rxDoc.clear();

		char buff[64];
		sprintf(buff, "W|Kp : %d | Ki : %d | Kd : %d | ms : %d | de : %d",
		        Kp, Ki, Kd, baseMotorSpeed, loopDelay);
		Serial.println(buff);

#if OLED_ENABLED == 1
		display.clearDisplay();
		display.setTextSize(1);
		display.setTextColor(SSD1306_WHITE);
		display.setCursor(0, 5);
		display.println(F("- USB/BT UPDATE -"));
		display.setCursor(0, 25);
		display.printf("KP:%d KI:%d KD:%d", Kp, Ki, Kd);
		display.setCursor(0, 40);
		display.printf("SPD:%d DLY:%d", baseMotorSpeed, loopDelay);
		display.display();
#endif
	}
#endif

#if USE_INBUILT_BLUETOOTH == 1
	if (SerialBT.available())
	{
		SerialBT.flush();
		String data = SerialBT.readStringUntil('\n');

		SerialBT.printf("I|Data received : %s\n", data.c_str());

		DeserializationError btError = deserializeJson(rxDoc, data);

		if (btError)
		{
			SerialBT.printf("E|deserialize json failed : %s\n", btError.f_str());
			rxDoc.clear();
			return;
		}

		// Validate and constrain all incoming values to safe ranges.
		Kp            = constrain((int)rxDoc["P"],   0, 200);
		Ki            = constrain((int)rxDoc["I"],   0, 200);
		Kd            = constrain((int)rxDoc["D"],   0, 200);
		baseMotorSpeed = constrain((int)rxDoc["ms"], 0, 255);
		loopDelay     = constrain((int)rxDoc["de"],  0, 500);
		rxDoc.clear();

		SerialBT.printf("W|Kp : %d | Ki : %d | Kd : %d | ms : %d | de : %d\n",
		                Kp, Ki, Kd, baseMotorSpeed, loopDelay);

#if OLED_ENABLED == 1
		display.clearDisplay();
		display.setTextSize(1);
		display.setTextColor(SSD1306_WHITE);
		display.setCursor(0, 5);
		display.println(F("- BLUETOOTH UPDATE -"));
		display.setCursor(0, 25);
		display.printf("KP:%d KI:%d KD:%d", Kp, Ki, Kd);
		display.setCursor(0, 40);
		display.printf("SPD:%d DLY:%d", baseMotorSpeed, loopDelay);
		display.display();
#endif
	}
#endif

#endif

	// Step 1 — Read sensors, update error
	readSensors();

	// Step 2 — Compute PID
	calculatePID();

	// Step 3 — Drive motors
	controlMotors();
}