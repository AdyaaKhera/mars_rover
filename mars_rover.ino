// MARS ROVER CORE v4 (Motors + Sensors)
// Integrates sensor telemetry with motor control and collision safety stop.
// Serial motor commands (from dashboard): F, B, L, R, S

#include <math.h>

#define USE_LCD 1
#define USE_BAROMETER 1
// 1 = ornamental/simulated barometer values (no physical sensor required)
// 0 = read real sensor hardware
#define BAROMETER_ORNAMENTAL 1

// Barometer driver selection:
// 1 = Grove BMP280 (Seeed library)
// 0 = Adafruit BMP280/BME280-compatible BMP280 library
#define USE_GROVE_BAROMETER 1

#if USE_LCD || USE_BAROMETER
#include <Wire.h>
#endif

#if USE_LCD
#include <rgb_lcd.h>
rgb_lcd lcd;
bool lcdReady = false;
unsigned long lastLcdInitTryMs = 0;
#endif

#if USE_BAROMETER
bool baroOk = false;
uint8_t baroAddr = 0;
const float BARO_SEA_LEVEL_HPA = 1013.25;

#if !BAROMETER_ORNAMENTAL
#if USE_GROVE_BAROMETER
#include <Seeed_BMP280.h>
BMP280 baro;
#else
#include <Adafruit_BMP280.h>
Adafruit_BMP280 baro;
#endif

unsigned long lastBaroRetryMs = 0;
const unsigned long BARO_RETRY_MS = 2500;

uint8_t detectBaroAddress() {
	for (uint8_t addr = 0x76; addr <= 0x77; addr++) {
		Wire.beginTransmission(addr);
		if (Wire.endTransmission() == 0) return addr;
	}
	return 0;
}

bool initBarometer() {
	#if USE_GROVE_BAROMETER
	baroAddr = detectBaroAddress();
	if (!baro.init()) {
		baroAddr = 0;
		return false;
	}
	if (baroAddr == 0) baroAddr = 0x77;
	return true;
	#else
	if (baro.begin(0x76)) {
		baroAddr = 0x76;
		return true;
	}
	if (baro.begin(0x77)) {
		baroAddr = 0x77;
		return true;
	}
	baroAddr = 0;
	return false;
	#endif
}
#endif
#endif

// Analog sensors
const int LIGHT_PIN = A0;
const int TEMP_PIN = A2;
#define USE_MOISTURE_SENSOR 0

#if USE_MOISTURE_SENSOR
const int MOISTURE_PIN = A1;
#endif

// Ultrasonic wiring mode:
// 1 = single-pin mode (keeps A4/A5 free for LCD I2C)
// 0 = classic two-pin mode (TRIG + ECHO on separate pins)
#define USE_ULTRASONIC_SINGLE_PIN 0

#if USE_ULTRASONIC_SINGLE_PIN
// Single-pin mode: wire TRIG and ECHO together through ~2.2k resistor to A3.
const int ULTRASONIC_PIN = A3;
#else
// Two-pin mode using free analog pins while keeping A4/A5 for I2C.
const int TRIG_PIN = A1;
const int ECHO_PIN = A3;
#endif

// L298N/L293 style motor driver pins
// FRONT MOTORS (Motor Driver 1)
const int FRONT_ENA = 3;    // Front left PWM
const int FRONT_IN1 = 2;    // Front left forward
const int FRONT_IN2 = 4;    // Front left backward
const int FRONT_ENB = 5;    // Front right PWM
const int FRONT_IN3 = 7;    // Front right forward
const int FRONT_IN4 = 8;    // Front right backward

// BACK MOTORS (Motor Driver 2)
const int BACK_ENA = 6;     // Back left PWM
const int BACK_IN1 = 10;    // Back left forward
const int BACK_IN2 = 11;    // Back left backward
const int BACK_ENB = 9;     // Back right PWM
const int BACK_IN3 = 12;    // Back right forward
const int BACK_IN4 = 13;    // Back right backward

// Drive behavior tuning
#define FRONT_DRIVE_ONLY 1
#define FRONT_LEFT_INVERTED 1
#define FRONT_RIGHT_INVERTED 1
#define SWAP_LEFT_RIGHT_COMMANDS 1

// Critical alert LED
const float CRITICAL_DIST_CM = 10.0;
const int RED_LED_PIN = LED_BUILTIN;
bool redBlinkOn = false;
unsigned long lastRedBlinkMs = 0;
const unsigned long RED_BLINK_MS = 180;

// Thermistor constants (common Grove/Elegoo temp module)
const float TH_BETA = 3975.0;
const float TH_R0 = 10000.0;
const float TH_T0K = 298.15;

// Dynamic thresholds after calibration
float HAZARD_DIST_CM = 20.0;
int DARK_LIGHT_TH = 250;
int SAMPLE_MOIST_TH = 700;
float HOT_TEMP_TH = 35.0;

// Calibration timing
const unsigned long CAL_TIME_MS = 5000;

// Telemetry timing
const unsigned long LOOP_MS = 80;
const unsigned long COMMAND_PUMP_MS = 2;
const unsigned long SUMMARY_MS = 30000;

// Distance smoothing
float dBuf[5] = {0, 0, 0, 0, 0};
int dIdx = 0;
bool dFull = false;

const uint8_t BOOT = 0;
const uint8_t CALIBRATING = 1;
const uint8_t SCAN = 2;
const uint8_t HAZARD = 3;
const uint8_t SAMPLE_CANDIDATE = 4;
const uint8_t DARK_ZONE = 5;

const uint8_t DRIVE_STOP = 0;
const uint8_t DRIVE_FWD = 1;
const uint8_t DRIVE_REV = 2;
const uint8_t DRIVE_LEFT = 3;
const uint8_t DRIVE_RIGHT = 4;

uint8_t state = BOOT;
uint8_t prevState = BOOT;
uint8_t driveMode = DRIVE_STOP;

unsigned long bootStart = 0;
unsigned long lastSummaryMs = 0;
unsigned long packetCount = 0;

// Simulated sector progression
int sectorX = 0;
int sectorY = 0;
unsigned long lastSectorStep = 0;

// Mission score counters
unsigned long hazardsDetected = 0;
unsigned long samplesFlagged = 0;
unsigned long darkZonesMapped = 0;
unsigned long stateChanges = 0;

// Calibration accumulators
unsigned long calCount = 0;
unsigned long calLightSum = 0;
unsigned long calMoistSum = 0;
unsigned long calTempRawSum = 0;

// Motor command and safety state
char requestedCmd = 'S';
char effectiveCmd = 'S';
bool hazardStopActive = false;
int motorPwm = 170;

// Motion hazard tracking from dashboard
float motionScore = 0.0;
bool motionHazardActive = false;

#if USE_BAROMETER
float baroPressureKPa = -1.0;
float baroAltitudeM = -1.0;

float normalizeToKPa(float pRaw) {
	if (isnan(pRaw) || pRaw <= 0.0) return -1.0;
	// Accept Pa, hPa, or kPa output variants from different libraries/boards.
	if (pRaw > 20000.0) return pRaw / 1000.0;   // Pa -> kPa
	if (pRaw > 300.0) return pRaw / 10.0;       // hPa -> kPa
	if (pRaw > 20.0) return pRaw;               // already kPa
	return -1.0;
}

void updateBarometer() {
	#if BAROMETER_ORNAMENTAL
	// Decorative telemetry when no real barometer is connected.
	float t = millis() * 0.001f;
	baroPressureKPa = 100.8f + 0.55f * sin(t * 0.22f) + 0.15f * sin(t * 1.15f);
	baroAltitudeM = 24.0f + 1.8f * sin(t * 0.18f) - 0.7f * sin(t * 0.63f);
	return;
	#endif

	#if !BAROMETER_ORNAMENTAL

	if (!baroOk) {
		unsigned long now = millis();
		if (now - lastBaroRetryMs >= BARO_RETRY_MS) {
			lastBaroRetryMs = now;
			baroOk = initBarometer();
			if (baroOk) {
				Serial.print("MARS_BARO|status=RECOVERED|addr=0x");
				Serial.println(baroAddr, HEX);
			}
		}
		baroPressureKPa = -1.0;
		baroAltitudeM = -1.0;
		return;
	}

	#if USE_GROVE_BAROMETER
	float pRaw = baro.getPressure();
	float kPa = normalizeToKPa(pRaw);
	if (kPa < 0.0) {
		baroPressureKPa = -1.0;
		baroAltitudeM = -1.0;
		return;
	}

	baroPressureKPa = kPa;
	baroAltitudeM = baro.calcAltitude(kPa * 1000.0);
	#else
	float pPa = baro.readPressure();
	float kPa = normalizeToKPa(pPa);
	if (kPa < 0.0) {
		baroPressureKPa = -1.0;
		baroAltitudeM = -1.0;
		return;
	}

	baroPressureKPa = kPa;
	baroAltitudeM = baro.readAltitude(BARO_SEA_LEVEL_HPA);
	#endif

	#endif
}
#endif

float readDistanceRawCm() {
	#if USE_ULTRASONIC_SINGLE_PIN
	pinMode(ULTRASONIC_PIN, OUTPUT);
	digitalWrite(ULTRASONIC_PIN, LOW);
	delayMicroseconds(3);
	digitalWrite(ULTRASONIC_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(ULTRASONIC_PIN, LOW);

	pinMode(ULTRASONIC_PIN, INPUT);
	unsigned long duration = pulseIn(ULTRASONIC_PIN, HIGH, 35000);
	#else
	digitalWrite(TRIG_PIN, LOW);
	delayMicroseconds(3);
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);

	unsigned long duration = pulseIn(ECHO_PIN, HIGH, 35000);
	#endif
	if (duration == 0) return -1.0;
	return duration / 58.0;
}

float readDistanceCmSmoothed() {
	float d = readDistanceRawCm();
	if (d > 0) {
		dBuf[dIdx] = d;
		dIdx++;
		if (dIdx >= 5) {
			dIdx = 0;
			dFull = true;
		}
	}

	int n = dFull ? 5 : dIdx;
	if (n == 0) return -1.0;

	float sum = 0;
	for (int i = 0; i < n; i++) sum += dBuf[i];
	return sum / n;
}

float tempCFromRaw(int raw) {
	if (raw <= 0 || raw >= 1023) return -999.0;
	float resistance = (1023.0 - raw) * TH_R0 / raw;
	float tempK = 1.0 / (log(resistance / TH_R0) / TH_BETA + (1.0 / TH_T0K));
	return tempK - 273.15;
}

const char* stateName(uint8_t s) {
	if (s == BOOT) return "BOOT";
	if (s == CALIBRATING) return "CAL";
	if (s == SCAN) return "SCAN";
	if (s == HAZARD) return "HAZ";
	if (s == SAMPLE_CANDIDATE) return "SAMP";
	return "DARK";
}

const char* driveName(uint8_t m) {
	if (m == DRIVE_FWD) return "FWD";
	if (m == DRIVE_REV) return "REV";
	if (m == DRIVE_LEFT) return "LEFT";
	if (m == DRIVE_RIGHT) return "RIGHT";
	return "STOP";
}

uint8_t decideState(float distCm, int lightRaw, int moistRaw, float tempC) {
	// Distance hazard is highest priority for safety.
	if (distCm > 0 && distCm < HAZARD_DIST_CM) return HAZARD;
	if (motionScore > 50.0) return HAZARD;
	if (moistRaw > SAMPLE_MOIST_TH || tempC > HOT_TEMP_TH) return SAMPLE_CANDIDATE;
	if (lightRaw < DARK_LIGHT_TH) return DARK_ZONE;
	return SCAN;
}

void updateSector() {
	unsigned long now = millis();
	if (now - lastSectorStep >= 2000) {
		lastSectorStep = now;
		sectorX++;
		if (sectorX > 7) {
			sectorX = 0;
			sectorY++;
			if (sectorY > 7) sectorY = 0;
		}
	}
}

void updateCriticalAlert(float distCm) {
	bool critical = (distCm > 0 && distCm < CRITICAL_DIST_CM);

	if (critical) {
		unsigned long now = millis();
		if (now - lastRedBlinkMs >= RED_BLINK_MS) {
			lastRedBlinkMs = now;
			redBlinkOn = !redBlinkOn;
			digitalWrite(RED_LED_PIN, redBlinkOn ? HIGH : LOW);
		}
	} else {
		redBlinkOn = false;
		digitalWrite(RED_LED_PIN, LOW);
	}
}

void driveStop() {
	// Front motors
	digitalWrite(FRONT_IN1, LOW);
	digitalWrite(FRONT_IN2, LOW);
	digitalWrite(FRONT_IN3, LOW);
	digitalWrite(FRONT_IN4, LOW);
	analogWrite(FRONT_ENA, 0);
	analogWrite(FRONT_ENB, 0);
	
	// Back motors
	digitalWrite(BACK_IN1, LOW);
	digitalWrite(BACK_IN2, LOW);
	digitalWrite(BACK_IN3, LOW);
	digitalWrite(BACK_IN4, LOW);
	analogWrite(BACK_ENA, 0);
	analogWrite(BACK_ENB, 0);
	
	driveMode = DRIVE_STOP;
}

void applyFrontDrive(bool leftForward, bool rightForward, int pwm) {
	#if FRONT_LEFT_INVERTED
	leftForward = !leftForward;
	#endif
	#if FRONT_RIGHT_INVERTED
	rightForward = !rightForward;
	#endif

	digitalWrite(FRONT_IN1, leftForward ? HIGH : LOW);
	digitalWrite(FRONT_IN2, leftForward ? LOW : HIGH);
	digitalWrite(FRONT_IN3, rightForward ? HIGH : LOW);
	digitalWrite(FRONT_IN4, rightForward ? LOW : HIGH);
	analogWrite(FRONT_ENA, pwm);
	analogWrite(FRONT_ENB, pwm);
}

void applyBackDrive(bool leftForward, bool rightForward, int pwm) {
	digitalWrite(BACK_IN1, leftForward ? HIGH : LOW);
	digitalWrite(BACK_IN2, leftForward ? LOW : HIGH);
	digitalWrite(BACK_IN3, rightForward ? HIGH : LOW);
	digitalWrite(BACK_IN4, rightForward ? LOW : HIGH);
	analogWrite(BACK_ENA, pwm);
	analogWrite(BACK_ENB, pwm);
}

void stopBackDrive() {
	digitalWrite(BACK_IN1, LOW);
	digitalWrite(BACK_IN2, LOW);
	digitalWrite(BACK_IN3, LOW);
	digitalWrite(BACK_IN4, LOW);
	analogWrite(BACK_ENA, 0);
	analogWrite(BACK_ENB, 0);
}

void driveForward(int pwm) {
	applyFrontDrive(true, true, pwm);
	#if FRONT_DRIVE_ONLY
	stopBackDrive();
	#else
	applyBackDrive(true, true, pwm);
	#endif
	
	driveMode = DRIVE_FWD;
}

void driveBackward(int pwm) {
	applyFrontDrive(false, false, pwm);
	#if FRONT_DRIVE_ONLY
	stopBackDrive();
	#else
	applyBackDrive(false, false, pwm);
	#endif
	
	driveMode = DRIVE_REV;
}

void driveLeft(int pwm) {
	applyFrontDrive(false, true, pwm);
	#if FRONT_DRIVE_ONLY
	stopBackDrive();
	#else
	applyBackDrive(false, true, pwm);
	#endif
	
	driveMode = DRIVE_LEFT;
}

void driveRight(int pwm) {
	applyFrontDrive(true, false, pwm);
	#if FRONT_DRIVE_ONLY
	stopBackDrive();
	#else
	applyBackDrive(true, false, pwm);
	#endif
	
	driveMode = DRIVE_RIGHT;
}

void applyDriveCommand(char cmd) {
	#if SWAP_LEFT_RIGHT_COMMANDS
	if (cmd == 'L') cmd = 'R';
	else if (cmd == 'R') cmd = 'L';
	#endif

	int pwm = constrain(motorPwm, 80, 255);
	if (cmd == 'F') driveForward(pwm);
	else if (cmd == 'B') driveBackward(pwm);
	else if (cmd == 'L') driveLeft(pwm);
	else if (cmd == 'R') driveRight(pwm);
	else driveStop();
}

void serviceSerialCommands() {
	while (Serial.available() > 0) {
		char c = Serial.peek();
		
		// Check if this is the start of a MARS_MOTION packet; if so, consume it
		if (c == 'M') {
			String line = "";
			while (Serial.available() > 0) {
				char ch = Serial.read();
				line += ch;
				if (ch == '\n') break;
			}
			if (line.startsWith("MARS_MOTION")) {
				parseMotionScore(line);
			}
			continue;
		}
		
		// Check for diagnostic command
		if (c == 'D') {
			Serial.read();  // consume 'D'
			printMotorDiagnostics();
			continue;
		}
		
		// Otherwise, consume the character and process as motor command
		c = Serial.read();

		if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S') {
			requestedCmd = c;
			Serial.print("MARS_CMD|motor=");
			Serial.print(c);
			Serial.print("|pwm=");
			Serial.println(motorPwm);
			continue;
		}

		if (c >= '0' && c <= '9') {
			int pct = (c - '0') * 10;
			motorPwm = map(pct, 0, 90, 0, 255);
			Serial.print("MARS_SPEED|pct=");
			Serial.print(pct);
			Serial.print("|pwm=");
			Serial.println(motorPwm);
			continue;
		}

		if (c == 'q' || c == 'Q') {
			motorPwm = 255;
			Serial.println("MARS_SPEED|pct=100|pwm=255");
		}
	}
}

void printMotorDiagnostics() {
	Serial.println("MARS_DIAG|MOTOR_TEST_START");
	Serial.println("MARS_DIAG|FRONT: ENA=3 IN1=2 IN2=4 | ENB=5 IN3=7 IN4=8");
	Serial.println("MARS_DIAG|BACK:  ENA=6 IN1=10 IN2=11 | ENB=9 IN3=12 IN4=13");
	
	// Test each direction
	Serial.println("MARS_DIAG|Testing FORWARD...");
	driveForward(200);
	delay(1000);
	printMotorState();
	
	Serial.println("MARS_DIAG|Testing BACKWARD...");
	driveBackward(200);
	delay(1000);
	printMotorState();
	
	Serial.println("MARS_DIAG|Testing LEFT...");
	driveLeft(200);
	delay(1000);
	printMotorState();
	
	Serial.println("MARS_DIAG|Testing RIGHT...");
	driveRight(200);
	delay(1000);
	printMotorState();
	
	Serial.println("MARS_DIAG|Testing STOP...");
	driveStop();
	delay(500);
	printMotorState();
	
	Serial.println("MARS_DIAG|MOTOR_TEST_COMPLETE");
}

void printMotorState() {
	int f_in1 = digitalRead(FRONT_IN1);
	int f_in2 = digitalRead(FRONT_IN2);
	int f_in3 = digitalRead(FRONT_IN3);
	int f_in4 = digitalRead(FRONT_IN4);
	int b_in1 = digitalRead(BACK_IN1);
	int b_in2 = digitalRead(BACK_IN2);
	int b_in3 = digitalRead(BACK_IN3);
	int b_in4 = digitalRead(BACK_IN4);
	
	Serial.print("MARS_STATE|FRONT: IN1=");
	Serial.print(f_in1);
	Serial.print(" IN2=");
	Serial.print(f_in2);
	Serial.print(" IN3=");
	Serial.print(f_in3);
	Serial.print(" IN4=");
	Serial.print(f_in4);
	Serial.print(" | BACK: IN1=");
	Serial.print(b_in1);
	Serial.print(" IN2=");
	Serial.print(b_in2);
	Serial.print(" IN3=");
	Serial.print(b_in3);
	Serial.print(" IN4=");
	Serial.print(b_in4);
	Serial.print(" | mode=");
	Serial.println(driveName(driveMode));
}
void parseMotionScore(const String &line) {
	// Parse MARS_MOTION|score=X.X packets from dashboard
	if (!line.startsWith("MARS_MOTION")) return;
	
	int scorePos = line.indexOf("score=");
	if (scorePos < 0) return;
	
	scorePos += 6; // Skip "score="
	int pipePos = line.indexOf('|', scorePos);
	if (pipePos < 0) pipePos = line.length();
	
	String scoreStr = line.substring(scorePos, pipePos);
	motionScore = scoreStr.toFloat();

	// Optional explicit state from dashboard for hazard transitions.
	bool explicitStateFound = false;
	bool explicitHazardState = false;
	int statePos = line.indexOf("state=");
	if (statePos >= 0) {
		explicitStateFound = true;
		statePos += 6; // Skip "state="
		int stateEnd = line.indexOf('|', statePos);
		if (stateEnd < 0) stateEnd = line.length();
		String stateStr = line.substring(statePos, stateEnd);
		stateStr.trim();
		stateStr.toUpperCase();
		explicitHazardState = (stateStr == "HAZARD");
	}

	bool newMotionHazard = explicitStateFound ? explicitHazardState : (motionScore > 50.0);
	if (newMotionHazard && !motionHazardActive) {
		motionHazardActive = true;
		Serial.print("MARS_ALERT|ms=");
		Serial.print(millis());
		Serial.print("|event=MOTION_HAZARD|score=");
		Serial.println(motionScore, 1);
	} else if (!newMotionHazard && motionHazardActive) {
		motionHazardActive = false;
		Serial.print("MARS_ALERT|ms=");
		Serial.print(millis());
		Serial.print("|event=MOTION_CLEAR|score=");
		Serial.println(motionScore, 1);
	}
}
void updateMotionSafety(float distCm) {
	bool distanceHazardNow = (distCm > 0 && distCm < HAZARD_DIST_CM);
	hazardStopActive = distanceHazardNow;

	if (distanceHazardNow) {
		// Hard safety override: hazardous distance always stops the rover.
		effectiveCmd = 'S';
		requestedCmd = 'S';
		applyDriveCommand('S');
		return;
	}

	effectiveCmd = requestedCmd;
	applyDriveCommand(effectiveCmd);
}

void pumpCommandsFor(unsigned long windowMs, float distCm) {
	unsigned long start = millis();
	while (millis() - start < windowMs) {
		serviceSerialCommands();
		updateMotionSafety(distCm);
		delay(COMMAND_PUMP_MS);
	}
}

void printPacket(unsigned long ms, int lightRaw, int moistRaw, int tempRaw, float tempC, float distCm, uint8_t s) {
	char col = 'A' + sectorX;
	int row = sectorY + 1;

	Serial.print("MARS_PKT");
	Serial.print("|id=");
	Serial.print(packetCount++);
	Serial.print("|ms=");
	Serial.print(ms);
	Serial.print("|sector=");
	Serial.print(col);
	Serial.print(row);
	Serial.print("|state=");
	Serial.print(stateName(s));
	Serial.print("|light=");
	Serial.print(lightRaw);
	Serial.print("|moist=");
	if (moistRaw < 0) Serial.print("NA");
	else Serial.print(moistRaw);
	Serial.print("|tempRaw=");
	Serial.print(tempRaw);
	Serial.print("|tempC=");
	if (tempC < -200.0) Serial.print("NA");
	else Serial.print(tempC, 1);
	Serial.print("|distCm=");
	if (distCm < 0) Serial.print("NA");
	else Serial.print(distCm, 1);

	// Extra motor telemetry for dashboard consumption.
	Serial.print("|motorReq=");
	Serial.print(requestedCmd);
	Serial.print("|motorEff=");
	Serial.print(effectiveCmd);
	Serial.print("|motorMode=");
	Serial.print(driveName(driveMode));
	Serial.print("|motorPwm=");
	Serial.print(motorPwm);
	Serial.print("|hazStop=");
	Serial.print(hazardStopActive ? 1 : 0);
	#if USE_BAROMETER
	Serial.print("|baroOk=");
	Serial.print(baroOk ? 1 : 0);
	Serial.print("|baroKPa=");
	if (baroPressureKPa < 0) Serial.print("NA");
	else Serial.print(baroPressureKPa, 2);
	Serial.print("|baroAltM=");
	if (baroAltitudeM < -9000) Serial.print("NA");
	else Serial.print(baroAltitudeM, 1);
	#endif
	Serial.println();
}

void printLiveValues(unsigned long ms, int lightRaw, int moistRaw, float tempC, float distCm) {
	Serial.print("MARS_VAL");
	Serial.print("|ms=");
	Serial.print(ms);
	Serial.print("|dist=");
	if (distCm < 0) Serial.print("NA");
	else Serial.print(distCm, 1);
	Serial.print("|temp=");
	if (tempC < -200.0) Serial.print("NA");
	else Serial.print(tempC, 1);
	Serial.print("|light=");
	Serial.print(lightRaw);
	Serial.print("|moist=");
	if (moistRaw < 0) Serial.print("NA");
	else Serial.print(moistRaw);
	Serial.print("|motor=");
	Serial.print(driveName(driveMode));
	#if USE_BAROMETER
	Serial.print("|baroKPa=");
	if (baroPressureKPa < 0) Serial.print("NA");
	else Serial.print(baroPressureKPa, 2);
	#endif
	Serial.println();
}

void printAlert(unsigned long ms, uint8_t fromS, uint8_t toS, int lightRaw, int moistRaw, float tempC, float distCm) {
	Serial.print("MARS_ALERT");
	Serial.print("|ms=");
	Serial.print(ms);
	Serial.print("|from=");
	Serial.print(stateName(fromS));
	Serial.print("|to=");
	Serial.print(stateName(toS));
	Serial.print("|light=");
	Serial.print(lightRaw);
	Serial.print("|moist=");
	if (moistRaw < 0) Serial.print("NA");
	else Serial.print(moistRaw);
	Serial.print("|tempC=");
	if (tempC < -200.0) Serial.print("NA");
	else Serial.print(tempC, 1);
	Serial.print("|distCm=");
	if (distCm < 0) Serial.print("NA");
	else Serial.print(distCm, 1);
	Serial.println();
}

void printSummary(unsigned long ms) {
	Serial.print("MARS_SUMMARY");
	Serial.print("|ms=");
	Serial.print(ms);
	Serial.print("|hazards=");
	Serial.print(hazardsDetected);
	Serial.print("|samples=");
	Serial.print(samplesFlagged);
	Serial.print("|darkZones=");
	Serial.print(darkZonesMapped);
	Serial.print("|stateChanges=");
	Serial.print(stateChanges);
	Serial.println();
}

#if USE_LCD
bool initLcd() {
	// Retry init a few times because I2C peripherals may power up slowly.
	for (uint8_t attempt = 0; attempt < 3; attempt++) {
		lcd.begin(16, 2);
		lcd.setRGB(0, 255, 255);
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("MARS ROVER V4");
		lcd.setCursor(0, 1);
		lcd.print("Booting...");
		delay(20);
	}

	lcdReady = true;
	lastLcdInitTryMs = millis();
	return lcdReady;
}

void updateLcd(float distCm, float tempC, uint8_t s) {
	if (!lcdReady) {
		if (millis() - lastLcdInitTryMs > 2000) {
			initLcd();
		}
		if (!lcdReady) return;
	}

	char col = 'A' + sectorX;
	int row = sectorY + 1;

	lcd.setCursor(0, 0);
	lcd.print(stateName(s));
	lcd.print(" ");
	lcd.print(col);
	lcd.print(row);
	lcd.print(" ");
	lcd.print(driveName(driveMode));
	lcd.print("   ");

	lcd.setCursor(0, 1);
	lcd.print("D:");
	if (distCm < 0) lcd.print("NA");
	else lcd.print(distCm, 0);
	lcd.print(" T:");
	if (tempC < -200.0) lcd.print("NA");
	else lcd.print(tempC, 0);
	lcd.print("   ");

	bool distanceHazardNow = (distCm > 0 && distCm < HAZARD_DIST_CM);
	bool hazardNow = distanceHazardNow || (s == HAZARD) || hazardStopActive || motionHazardActive;
	if (hazardNow) {
		lcd.setRGB(255, 0, 0);
		return;
	}

	if (distCm > 0 && distCm < CRITICAL_DIST_CM) lcd.setRGB(255, 0, 0);
	else if (s == SCAN) lcd.setRGB(0, 80, 255);
	else if (s == SAMPLE_CANDIDATE) lcd.setRGB(255, 180, 0);
	else if (s == DARK_ZONE) lcd.setRGB(180, 0, 255);
	else lcd.setRGB(0, 255, 255);
}
#endif

void setup() {
	Serial.begin(115200);
	delay(150);
	Serial.println("MARS_ROVER_BOOT");
	Serial.print("MARS_CFG|ultraMode=");
	Serial.print(USE_ULTRASONIC_SINGLE_PIN ? "single" : "two-pin");
	Serial.print("|lcd=");
	Serial.print(USE_LCD ? 1 : 0);
	Serial.print("|baro=");
	Serial.println(USE_BAROMETER ? 1 : 0);
	Serial.print("MARS_CFG|moisture=");
	Serial.println(USE_MOISTURE_SENSOR ? 1 : 0);
	#if !USE_ULTRASONIC_SINGLE_PIN
	Serial.print("MARS_CFG|trigPin=");
	Serial.print(TRIG_PIN);
	Serial.print("|echoPin=");
	Serial.println(ECHO_PIN);
	#endif

	#if USE_ULTRASONIC_SINGLE_PIN
	pinMode(ULTRASONIC_PIN, INPUT);
	#else
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
	#endif
	pinMode(RED_LED_PIN, OUTPUT);
	digitalWrite(RED_LED_PIN, LOW);

	// Front motor pins
	pinMode(FRONT_ENA, OUTPUT);
	pinMode(FRONT_ENB, OUTPUT);
	pinMode(FRONT_IN1, OUTPUT);
	pinMode(FRONT_IN2, OUTPUT);
	pinMode(FRONT_IN3, OUTPUT);
	pinMode(FRONT_IN4, OUTPUT);
	
	// Back motor pins
	pinMode(BACK_ENA, OUTPUT);
	pinMode(BACK_ENB, OUTPUT);
	pinMode(BACK_IN1, OUTPUT);
	pinMode(BACK_IN2, OUTPUT);
	pinMode(BACK_IN3, OUTPUT);
	pinMode(BACK_IN4, OUTPUT);
	
	driveStop();

	#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_ESP32)
	analogReadResolution(10);
	#endif

	#if USE_LCD || USE_BAROMETER
	Wire.begin();
	#endif

	#if USE_BAROMETER
	#if BAROMETER_ORNAMENTAL
	baroOk = true;
	baroAddr = 0;
	Serial.println("MARS_BARO|status=ORNAMENTAL");
	#else
	baroOk = initBarometer();
	if (!baroOk) {
		Serial.println("MARS_BARO|status=ERR");
	} else {
		Serial.print("MARS_BARO|status=OK|addr=0x");
		Serial.println(baroAddr, HEX);
	}
	#endif
	#endif

#if USE_LCD
	initLcd();
#endif

	bootStart = millis();
	lastSectorStep = millis();
	lastSummaryMs = millis();
}

void loop() {
	unsigned long now = millis();

	serviceSerialCommands();

	int lightRaw = analogRead(LIGHT_PIN);
	int moistRaw = -1;
	#if USE_MOISTURE_SENSOR
	moistRaw = analogRead(MOISTURE_PIN);
	#endif
	int tempRaw = analogRead(TEMP_PIN);
	float tempC = tempCFromRaw(tempRaw);
	float distCm = readDistanceCmSmoothed();
	#if USE_BAROMETER
	updateBarometer();
	#endif

	updateCriticalAlert(distCm);
	updateSector();

	if (now - bootStart < 1000) {
		state = BOOT;
	} else if (now - bootStart < 1000 + CAL_TIME_MS) {
		state = CALIBRATING;
		calLightSum += lightRaw;
		#if USE_MOISTURE_SENSOR
		calMoistSum += moistRaw;
		#endif
		calTempRawSum += tempRaw;
		calCount++;
	} else {
		static bool calibrated = false;
		if (!calibrated) {
			if (calCount > 0) {
				int baseLight = calLightSum / calCount;
				int baseTempRaw = calTempRawSum / calCount;
				float baseTempC = tempCFromRaw(baseTempRaw);

				DARK_LIGHT_TH = (int)(baseLight * 0.55);
				#if USE_MOISTURE_SENSOR
				int baseMoist = calMoistSum / calCount;
				SAMPLE_MOIST_TH = baseMoist + 80;
				#endif
				HOT_TEMP_TH = baseTempC + 6.0;
			}

			calibrated = true;

			Serial.print("MARS_CAL");
			Serial.print("|darkTh=");
			Serial.print(DARK_LIGHT_TH);
			Serial.print("|moistTh=");
			Serial.print(SAMPLE_MOIST_TH);
			Serial.print("|hotTh=");
			Serial.print(HOT_TEMP_TH, 1);
			Serial.println();
		}

		state = decideState(distCm, lightRaw, moistRaw, tempC);
	}

	updateMotionSafety(distCm);

	if (state != prevState) {
		stateChanges++;
		printAlert(now, prevState, state, lightRaw, moistRaw, tempC, distCm);

		if (state == HAZARD) hazardsDetected++;
		if (state == SAMPLE_CANDIDATE) samplesFlagged++;
		if (state == DARK_ZONE) darkZonesMapped++;

		prevState = state;
	}

	printPacket(now, lightRaw, moistRaw, tempRaw, tempC, distCm, state);
	printLiveValues(now, lightRaw, moistRaw, tempC, distCm);

	if (now - lastSummaryMs >= SUMMARY_MS) {
		lastSummaryMs = now;
		printSummary(now);
	}

#if USE_LCD
	updateLcd(distCm, tempC, state);
#endif

	pumpCommandsFor(LOOP_MS, distCm);
}
