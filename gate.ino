#include <Wire.h>

// PIN DEFINITIONS
const int PIN_PHOTOCELL        = 2;
const int PIN_RECEIVER         = 3;
const int PIN_FLASHER          = 4;
const int PIN_EMERGENCY        = 5;
const int PIN_MOTOR_SPEED      = 6;
const int PIN_MOTOR_DIRECTION  = 7;   // HIGH = open, LOW = close
const int PIN_LIMIT_OPEN       = A0;
const int PIN_LIMIT_CLOSED     = A1;
const int PIN_CURRENT_SENSE    = A2;

// CONSTANTS
const int    LIMIT_SWITCH_THRESHOLD   = 200;
const float  OVERCURRENT_DEFAULT      = 11.0f;
const float  OVERCURRENT_SLOW_PHASE   = 7.0f;
const int    CURRENT_SAMPLES          = 300;
const int    RECEIVER_DEBOUNCE_MS     = 1500;
const int    LEARN_MOTOR_SPEED        = 98;
const float  ADAPT_WEIGHT             = 0.25f; // weight of new measurement in moving average

// SLOW_PHASE_RATIO defines what fraction of the total travel distance
// is covered in the slow phase. This is used during learning to split
// the measured total time into fullTime and slowTime.
// During normal operation each phase is updated independently via adaptive timing.
const float  SLOW_PHASE_RATIO         = 0.25f;
// Speed used when recovering from a mid-travel start (unknown position).
const int    SAFE_RECOVERY_SPEED      = 92;
// Minimum speed during the slow phase
const int    SLOW_PHASE_MIN_SPEED     = 85;

// Flasher toggle intervals
const unsigned long FLASHER_TOGGLE_INTERVAL          = 500;
const unsigned long FLASHER_TOGGLE_INTERVAL_LEARNING = 200;

// STATE MACHINE
enum GateState {
  STATE_IDLE      = 0,
  STATE_CLOSING   = 1,
  STATE_OPENING   = 2,
  STATE_LEARNING  = 3
};

volatile GateState currentState = STATE_IDLE;


// Each direction has two independently tracked times:
//   fullTime  = time spent at full speed (255)
//   slowTime  = time spent decelerating to the limit switch
// Both are updated adaptively after every complete travel.
unsigned long openTimeFull   = 0;
unsigned long openTimeSlow   = 0;
unsigned long closeTimeFull  = 0;
unsigned long closeTimeSlow  = 0;

// Backup timing: last known-good configuration
// These are saved periodically when the gate operates correctly,
// and restored if a travel is detected to be too fast.
unsigned long openTimeFullBackup   = 0;
unsigned long openTimeSlowBackup   = 0;
unsigned long closeTimeFullBackup  = 0;
unsigned long closeTimeSlowBackup  = 0;

int successfulCycles = 0;
const int BACKUP_INTERVAL = 5;

// Safety threshold: minimum acceptable slow phase duration as a fraction of expected
const float SLOW_PHASE_MIN_RATIO = 0.3f;

// GATE POSITION & TRAVEL STATE
unsigned long position         = 0;
unsigned long learnStartMillis = 0;
bool          learned          = false;
bool          returnFlag       = false;

unsigned long lastKnownPosition = 0;
bool          positionValid     = false;
int directionChanges            = 0;
bool cycleWasComplete           = false;

const int MAX_DIRECTION_CHANGES = 5;

// CURRENT SENSOR
int   currentSensorOffset      = 0;
float currentValue             = 0.0f;
float overCurrentThreshold     = OVERCURRENT_DEFAULT;
bool  overCurrentFlag          = false;

const bool OVERCURRENT_ENABLED = false;

// RECEIVER
unsigned long lastReceiverTime = 0;

// I2C
char i2cCommand  = 0;
char i2cBuffer[19];

// LIMIT SWITCH HELPERS
bool isFullyOpen() {
  return analogRead(PIN_LIMIT_OPEN) < LIMIT_SWITCH_THRESHOLD;
}

bool isFullyClosed() {
  return analogRead(PIN_LIMIT_CLOSED) < LIMIT_SWITCH_THRESHOLD;
}

// CURRENT SENSOR
void sampleCurrent() {
  float sum = 0.0f;
  for (int i = 0; i < CURRENT_SAMPLES; i++) {
    sum += (analogRead(PIN_CURRENT_SENSE) - currentSensorOffset) / 21.0f;
  }
  currentValue = sum / CURRENT_SAMPLES;
  if (currentValue < 0.0f) currentValue = 0.0f;

  if (OVERCURRENT_ENABLED && currentValue >= overCurrentThreshold) {
    overCurrentFlag = true;
    Serial.print("OverCurrent: ");
    Serial.println(currentValue);
  }
}

// MOTOR HELPERS
void motorStop() {
  analogWrite(PIN_MOTOR_SPEED, 0);
}

void motorOpen(int speed) {
  digitalWrite(PIN_MOTOR_DIRECTION, HIGH);
  analogWrite(PIN_MOTOR_SPEED, speed);
}

void motorClose(int speed) {
  digitalWrite(PIN_MOTOR_DIRECTION, LOW);
  analogWrite(PIN_MOTOR_SPEED, speed);
}

// FLASHER
void flasherOn()     { digitalWrite(PIN_FLASHER, HIGH); }
void flasherOff()    { digitalWrite(PIN_FLASHER, LOW); }
void flasherToggle() { digitalWrite(PIN_FLASHER, !digitalRead(PIN_FLASHER)); }

bool flasherUpdate(unsigned long startMillis, unsigned long interval) {
  unsigned long elapsed = millis() - startMillis;
  bool shouldBeOn = ((elapsed / interval) % 2) == 0;
  digitalWrite(PIN_FLASHER, shouldBeOn ? HIGH : LOW);
  return shouldBeOn;
}

// SAFETY HELPERS
bool isEmergencyPressed() {
  return digitalRead(PIN_EMERGENCY) == HIGH;
}

bool isPhotocellBlocked() {
  return digitalRead(PIN_PHOTOCELL) == LOW;
}

void waitForPhotocellClear() {
  while (isPhotocellBlocked()) {
    if (currentState == STATE_IDLE) return;
  }
}

// Returns the motor speed for a given moment inside the slow phase.
// elapsed = time since the slow phase started
// totalSlowTime = total duration of the slow phase
// Speed ramps linearly from 255 down to SLOW_PHASE_MIN_SPEED.
int slowPhaseSpeed(unsigned long elapsed, unsigned long totalSlowTime) {
  if (totalSlowTime == 0) return SLOW_PHASE_MIN_SPEED;
  float ratio = (float)elapsed / (float)totalSlowTime;  // 0.0 at start, 1.0 at end
  if (ratio > 1.0f) ratio = 1.0f;
  int speed = (int)(255.0f - (255.0f - SLOW_PHASE_MIN_SPEED) * ratio);
  return speed;
}

// Called after a complete travel (limit switch to limit switch).
// measuredFullMs and measuredSlowMs are the actual durations of each phase.
void updateOpenTiming(unsigned long measuredFullMs, unsigned long measuredSlowMs) {
  openTimeFull = (unsigned long)((1.0f - ADAPT_WEIGHT) * openTimeFull + ADAPT_WEIGHT * measuredFullMs);
  openTimeSlow = (unsigned long)((1.0f - ADAPT_WEIGHT) * openTimeSlow + ADAPT_WEIGHT * measuredSlowMs);
  Serial.print("Adaptive open — full: "); Serial.print(openTimeFull);
  Serial.print(" slow: "); Serial.println(openTimeSlow);
}

void updateCloseTiming(unsigned long measuredFullMs, unsigned long measuredSlowMs) {
  closeTimeFull = (unsigned long)((1.0f - ADAPT_WEIGHT) * closeTimeFull + ADAPT_WEIGHT * measuredFullMs);
  closeTimeSlow = (unsigned long)((1.0f - ADAPT_WEIGHT) * closeTimeSlow + ADAPT_WEIGHT * measuredSlowMs);
  Serial.print("Adaptive close — full: "); Serial.print(closeTimeFull);
  Serial.print(" slow: "); Serial.println(closeTimeSlow);
}

// Saves the current timing as a known-good backup
void backupTiming() {
  openTimeFullBackup  = openTimeFull;
  openTimeSlowBackup  = openTimeSlow;
  closeTimeFullBackup = closeTimeFull;
  closeTimeSlowBackup = closeTimeSlow;
  Serial.println("Timing backup saved");
}

// Restores timing from the last known-good backup
void restoreTiming() {
  if (openTimeFullBackup > 0 && closeTimeFullBackup > 0) {
    openTimeFull  = openTimeFullBackup;
    openTimeSlow  = openTimeSlowBackup;
    closeTimeFull = closeTimeFullBackup;
    closeTimeSlow = closeTimeSlowBackup;
    Serial.println("Timing restored from backup");
  } else {
    Serial.println("No backup available, keeping current timing");
  }
}

// Validates that the slow phase duration was acceptable
// Returns true if safe, false if the gate arrived too fast
bool validateSlowPhase(unsigned long expectedSlowMs, unsigned long actualSlowMs, const char* direction) {
  if (expectedSlowMs == 0) return true; // can't validate without expected time

  float ratio = (float)actualSlowMs / (float)expectedSlowMs;

  Serial.println(expectedSlowMs);
  Serial.println(ratio);


  if (ratio < SLOW_PHASE_MIN_RATIO) {
    Serial.print("WARNING: ");
    Serial.print(direction);
    Serial.print(" slow phase too short! Expected: ");
    Serial.print(expectedSlowMs);
    Serial.print("ms, Actual: ");
    Serial.print(actualSlowMs);
    Serial.print("ms (");
    Serial.print(ratio * 100.0f);
    Serial.println("%)");
    return false;
  }

  return true;
}

// Simple linear interpolation based on elapsed time vs total expected time.
unsigned long estimatePosition(unsigned long elapsed, unsigned long totalTime, bool opening) {
  if (totalTime == 0) return 50;
  unsigned long pct = elapsed * 100 / totalTime;
  if (pct > 100) pct = 100;
  return opening ? (100 - pct) : pct;
}

// STATE TRANSITION LOGIC
void checkStateTransition() {
  if (!learned) {
    currentState = STATE_LEARNING;
    return;
  }

  if (currentState == STATE_IDLE) {
    if (isFullyOpen())                               currentState = STATE_CLOSING;
    else if (isFullyClosed())                        currentState = STATE_OPENING;
    else /* mid-travel */                            currentState = STATE_CLOSING;

  } else if (currentState == STATE_CLOSING) {
    currentState = STATE_OPENING;

  } else if (currentState == STATE_OPENING) {
    if (isFullyOpen())                               currentState = STATE_CLOSING;
    else if (!isFullyOpen() && !isFullyClosed())     currentState = STATE_IDLE;
  }
}

// OPENING LOGIC
void doOpening() {
  overCurrentThreshold = OVERCURRENT_DEFAULT;
  flasherOn();

  bool cycleStartedAsInterrupted = cycleWasComplete;
  cycleWasComplete = false;

  if (!isFullyClosed()) {
    bool useIntelligentRecovery = false;
    unsigned long estimatedTimeRemaining = 0;

    if (positionValid && directionChanges < MAX_DIRECTION_CHANGES) {
      if (lastKnownPosition > 70) {
        useIntelligentRecovery = true;
        unsigned long estimatedFullTime = (100 - lastKnownPosition) * openTimeFull / 100;
        estimatedTimeRemaining = min(estimatedFullTime, openTimeFull / 2);
        Serial.print("Opening: intelligent recovery from pos ");
        Serial.println(lastKnownPosition);
      }
    }

    if (!useIntelligentRecovery) {
      Serial.println("Opening: safe recovery");
      motorOpen(SAFE_RECOVERY_SPEED);
      unsigned long startMillis = millis();

      while (!isFullyOpen()) {
        delay(100);
        sampleCurrent();
        position = 50;

        if (overCurrentFlag || currentState == STATE_IDLE || isEmergencyPressed()) {
          motorStop();
          flasherOff();
          overCurrentFlag = false;
          overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
          currentState = STATE_IDLE;
          returnFlag = false;
          lastKnownPosition = position;
          positionValid = false;
          return;
        }

        flasherUpdate(startMillis, FLASHER_TOGGLE_INTERVAL);
      }

      position = 0;
      motorStop();
      flasherOff();
      positionValid = false;
      directionChanges = 0;
      return;
    }

    Serial.print("Opening: fast recovery, time limit: ");
    Serial.println(estimatedTimeRemaining);

    learnStartMillis = millis();
    unsigned long travelStart = millis();
    motorOpen(255);
    unsigned long fullPhaseEnd = 0;

    while (!isFullyOpen()) {
      delay(100);
      unsigned long elapsed = millis() - travelStart;
      position = lastKnownPosition - (elapsed * 100 / (openTimeFull + openTimeSlow));
      if (position > 100) position = 0;

      sampleCurrent();

      if (elapsed >= estimatedTimeRemaining && fullPhaseEnd == 0) {
        fullPhaseEnd = millis();
        Serial.println("Opening: recovery entering slow phase");
      }

      if (fullPhaseEnd > 0) {
        unsigned long slowElapsed = millis() - fullPhaseEnd;
        int speed = slowPhaseSpeed(slowElapsed, openTimeSlow);
        motorOpen(speed);
        overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
      }

      if (overCurrentFlag || currentState == STATE_IDLE || isEmergencyPressed()) {
        motorStop();
        flasherOff();
        overCurrentFlag = false;
        overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
        currentState = STATE_IDLE;
        returnFlag = false;
        lastKnownPosition = position;
        positionValid = true;
        directionChanges++;
        return;
      }

      flasherUpdate(travelStart, FLASHER_TOGGLE_INTERVAL);
    }

    position = 0;
    motorStop();
    flasherOff();
    overCurrentThreshold = OVERCURRENT_DEFAULT;
    positionValid = false;
    directionChanges = 0;
    return;
  }

  Serial.println("Opening: full travel");
  learnStartMillis = millis();
  unsigned long travelStart = millis();

  motorOpen(255);
  unsigned long fullPhaseEnd = 0;

  while (!isFullyOpen()) {
    delay(100);
    unsigned long elapsed = millis() - travelStart;
    unsigned long totalExpected = openTimeFull + openTimeSlow;

    position = estimatePosition(elapsed, totalExpected, true);

    sampleCurrent();

    // Check if it's time to enter slow phase
    if (elapsed >= openTimeFull && fullPhaseEnd == 0) {
      fullPhaseEnd = millis();
      Serial.println("Opening: entering slow phase");
    }

    if (fullPhaseEnd > 0) {
      unsigned long slowElapsed = millis() - fullPhaseEnd;
      int speed = slowPhaseSpeed(slowElapsed, openTimeSlow);
      motorOpen(speed);
      overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
    }

    // Safety checks
    if (overCurrentFlag || currentState == STATE_IDLE || isEmergencyPressed()) {
      motorStop();
      flasherOff();
      overCurrentFlag = false;
      overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
      currentState = STATE_IDLE;
      returnFlag = false;
      learnStartMillis = 0;
      lastKnownPosition = position;
      positionValid = true;
      directionChanges++;
      cycleWasComplete = false;
      return;
    }

    flasherUpdate(travelStart, FLASHER_TOGGLE_INTERVAL);
  }

  position = 0;

  // Adaptive update: only if we started from a known position
  if (learnStartMillis > 0 && fullPhaseEnd > 0) {
    unsigned long measuredFull = fullPhaseEnd - learnStartMillis;
    unsigned long measuredSlow = millis() - fullPhaseEnd;

    bool safeTravel = validateSlowPhase(openTimeSlow, measuredSlow, "Opening");

    if (safeTravel && !cycleStartedAsInterrupted) {
      updateOpenTiming(measuredFull, measuredSlow);
      successfulCycles++;

      if (successfulCycles >= BACKUP_INTERVAL) {
        backupTiming();
        successfulCycles = 0;
      }
    } else if (!safeTravel) {
      restoreTiming();
      successfulCycles = 0;
    }
  }

  motorStop();
  flasherOff();
  overCurrentThreshold = OVERCURRENT_DEFAULT;
  lastKnownPosition = 0;
  positionValid = false;
  directionChanges = 0;
  cycleWasComplete = true;
}

// CLOSING LOGIC
void doClosing() {
  overCurrentThreshold = OVERCURRENT_DEFAULT;
  currentState = STATE_CLOSING;
  flasherOn();

  bool cycleStartedAsInterrupted = cycleWasComplete;
  cycleWasComplete = false;

  waitForPhotocellClear();

  if (!isFullyOpen()) {
    bool useIntelligentRecovery = false;
    unsigned long estimatedTimeRemaining = 0;

    if (positionValid && directionChanges < MAX_DIRECTION_CHANGES) {
      if (lastKnownPosition < 30) {
        useIntelligentRecovery = true;
        unsigned long estimatedFullTime = lastKnownPosition * closeTimeFull / 100;
        estimatedTimeRemaining = min(estimatedFullTime, closeTimeFull / 2);
        Serial.print("Closing: intelligent recovery from pos ");
        Serial.println(lastKnownPosition);
      }
    }

    if (!useIntelligentRecovery) {
      Serial.println("Closing: safe recovery");

      waitForPhotocellClear();

      motorClose(SAFE_RECOVERY_SPEED);
      unsigned long startMillis = millis();

      while (!isFullyClosed()) {
        delay(100);
        sampleCurrent();
        position = 50;

        if (isPhotocellBlocked() || overCurrentFlag || currentState == STATE_OPENING || isEmergencyPressed()) {
          motorStop();
          delay(2000);
          overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
          overCurrentFlag = false;
          returnFlag = true;
          currentState = STATE_OPENING;
          flasherOff();
          lastKnownPosition = position;
          positionValid = false;
          return;
        }

        flasherUpdate(startMillis, FLASHER_TOGGLE_INTERVAL);
      }

      position = 100;
      motorStop();
      flasherOff();
      currentState = STATE_IDLE;
      returnFlag = false;
      positionValid = false;
      directionChanges = 0;
      return;
    }

    Serial.print("Closing: fast recovery, time limit: ");
    Serial.println(estimatedTimeRemaining);

    learnStartMillis = millis();
    unsigned long travelStart = millis();
    motorClose(255);
    unsigned long fullPhaseEnd = 0;

    while (!isFullyClosed()) {
      delay(100);
      unsigned long elapsed = millis() - travelStart;
      position = lastKnownPosition + (elapsed * 100 / (closeTimeFull + closeTimeSlow));
      if (position > 100) position = 100;

      sampleCurrent();

      if (elapsed >= estimatedTimeRemaining && fullPhaseEnd == 0) {
        fullPhaseEnd = millis();
        Serial.println("Closing: recovery entering slow phase");
      }

      if (fullPhaseEnd > 0) {
        unsigned long slowElapsed = millis() - fullPhaseEnd;
        int speed = slowPhaseSpeed(slowElapsed, closeTimeSlow);
        motorClose(speed);
        overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
      }

      if (isPhotocellBlocked() || overCurrentFlag || currentState == STATE_OPENING || isEmergencyPressed()) {
        motorStop();
        delay(2000);
        overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
        overCurrentFlag = false;
        returnFlag = true;
        currentState = STATE_OPENING;
        flasherOff();
        lastKnownPosition = position;
        positionValid = true;
        directionChanges++;
        return;
      }

      flasherUpdate(travelStart, FLASHER_TOGGLE_INTERVAL);
    }

    position = 100;
    motorStop();
    flasherOff();
    overCurrentThreshold = OVERCURRENT_DEFAULT;
    currentState = STATE_IDLE;
    returnFlag = false;
    positionValid = false;
    directionChanges = 0;
    return;
  }

  Serial.println("Closing: full travel");
  learnStartMillis = millis();
  unsigned long travelStart = millis();

  motorClose(255);
  unsigned long fullPhaseEnd = 0;

  while (!isFullyClosed()) {
    delay(100);
    unsigned long elapsed = millis() - travelStart;
    unsigned long totalExpected = closeTimeFull + closeTimeSlow;

    position = estimatePosition(elapsed, totalExpected, false);

    sampleCurrent();

    if (elapsed >= closeTimeFull && fullPhaseEnd == 0) {
      fullPhaseEnd = millis();
      Serial.println("Closing: entering slow phase");
    }

    // Phase 2: slow phase — ramp speed down over closeTimeSlow
    if (fullPhaseEnd > 0) {
      unsigned long slowElapsed = millis() - fullPhaseEnd;
      int speed = slowPhaseSpeed(slowElapsed, closeTimeSlow);
      motorClose(speed);
      overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
    }

    // Safety checks
    if (isPhotocellBlocked() || overCurrentFlag || currentState == STATE_OPENING || isEmergencyPressed()) {
      motorStop();
      delay(2000);
      overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
      overCurrentFlag = false;
      returnFlag = true;
      learnStartMillis = 0;
      currentState = STATE_OPENING;
      flasherOff();
      lastKnownPosition = position;
      positionValid = true;
      directionChanges++;
      cycleWasComplete = false;
      return;
    }

    flasherUpdate(travelStart, FLASHER_TOGGLE_INTERVAL);
  }

  position = 100;

  // Adaptive update: only if we started from a known position
  if (learnStartMillis > 0 && fullPhaseEnd > 0) {
    unsigned long measuredFull = fullPhaseEnd - learnStartMillis;
    unsigned long measuredSlow = millis() - fullPhaseEnd;

    bool safeTravel = validateSlowPhase(closeTimeSlow, measuredSlow, "Closing");

    if (safeTravel && !cycleStartedAsInterrupted) {
      updateCloseTiming(measuredFull, measuredSlow);
      successfulCycles++;

      if (successfulCycles >= BACKUP_INTERVAL) {
        backupTiming();
        successfulCycles = 0;
      }
    } else if (!safeTravel) {
      restoreTiming();
      successfulCycles = 0;
    }
  }

  motorStop();
  flasherOff();
  overCurrentThreshold = OVERCURRENT_DEFAULT;
  currentState = STATE_IDLE;
  returnFlag = false;
  lastKnownPosition = 100;
  positionValid = false;
  directionChanges = 0;
  cycleWasComplete = true;
}

// LEARNING LOGIC
void doLearning() {
  Serial.println("Learning");
  flasherOff();
  motorStop();

  // Speed correction factor: learning measures at LEARN_MOTOR_SPEED, but real travel
  // uses 255 for the full-speed phase. We need to scale the measured times.
  const float speedFactor = (float)LEARN_MOTOR_SPEED / 255.0f;

  bool startFromClosed = false;
  bool startFromOpen = false;

  if (isFullyClosed()) {
    Serial.println("Learning: starting from closed position");
    startFromClosed = true;
  } else if (isFullyOpen()) {
    Serial.println("Learning: starting from open position");
    startFromOpen = true;
  } else {
    Serial.println("Learning: mid-travel detected, recovering to closed");

    while (isPhotocellBlocked()) {
      delay(100);
      if (digitalRead(PIN_RECEIVER)) {
        Serial.println("Learning: aborted during photocell wait");
        return;
      }
    }

    unsigned long recoveryStart = millis();
    flasherOn();
    motorClose(LEARN_MOTOR_SPEED);

    while (!isFullyClosed()) {
      delay(50);
      sampleCurrent();
      flasherUpdate(recoveryStart, FLASHER_TOGGLE_INTERVAL_LEARNING);
      if (isPhotocellBlocked() || overCurrentFlag || isEmergencyPressed() || digitalRead(PIN_RECEIVER)) {
        motorStop();
        flasherOff();
        delay(1000);
        overCurrentFlag = false;
        Serial.println("Learning: recovery interrupted");
        return;
      }
    }
    motorStop();
    flasherOff();
    delay(1000);
    startFromClosed = true;
  }

  // ── Measurement sequence based on starting position ──
  if (startFromClosed) {

    Serial.println("Learning: measuring open time");
    {
      unsigned long startMillis = millis();
      flasherOn();
      motorOpen(LEARN_MOTOR_SPEED);
      while (!isFullyOpen()) {
        delay(50);
        sampleCurrent();
        flasherUpdate(startMillis, FLASHER_TOGGLE_INTERVAL_LEARNING);
        if (overCurrentFlag || digitalRead(PIN_RECEIVER)) {
          motorStop();
          flasherOff();
          delay(1000);
          overCurrentFlag = false;
          Serial.println("Learning: open measurement interrupted");
          return;
        }
      }
      motorStop();
      flasherOff();

      unsigned long totalOpenMs = millis() - startMillis;

      // Apply speed correction and split into phases
      openTimeSlow = (unsigned long)(totalOpenMs * SLOW_PHASE_RATIO);
      openTimeFull = (unsigned long)((totalOpenMs - openTimeSlow) * speedFactor);

      Serial.print("Learning open — full: "); Serial.print(openTimeFull);
      Serial.print(" slow: "); Serial.println(openTimeSlow);
    }

    delay(3000);

    Serial.println("Learning: measuring close time");

    while (isPhotocellBlocked()) {
      delay(100);
      if (digitalRead(PIN_RECEIVER)) {
        Serial.println("Learning: aborted during photocell wait");
        return;
      }
    }

    {
      unsigned long startMillis = millis();
      flasherOn();
      motorClose(LEARN_MOTOR_SPEED);
      while (!isFullyClosed()) {
        delay(50);
        sampleCurrent();
        flasherUpdate(startMillis, FLASHER_TOGGLE_INTERVAL_LEARNING);
        if (isPhotocellBlocked() || overCurrentFlag || isEmergencyPressed() || digitalRead(PIN_RECEIVER)) {
          motorStop();
          flasherOff();
          delay(1000);
          overCurrentFlag = false;
          Serial.println("Learning: close measurement interrupted");
          return;
        }
      }
      motorStop();
      flasherOff();

      unsigned long totalCloseMs = millis() - startMillis;

      // Apply speed correction and split into phases
      closeTimeSlow = (unsigned long)(totalCloseMs * SLOW_PHASE_RATIO);
      closeTimeFull = (unsigned long)((totalCloseMs - closeTimeSlow) * speedFactor);

      Serial.print("Learning close — full: "); Serial.print(closeTimeFull);
      Serial.print(" slow: "); Serial.println(closeTimeSlow);
    }

  } else if (startFromOpen) {

    Serial.println("Learning: measuring close time");

    // Wait for photocell before closing
    while (isPhotocellBlocked()) {
      delay(100);
      if (digitalRead(PIN_RECEIVER)) {
        Serial.println("Learning: aborted during photocell wait");
        return;
      }
    }

    {
      unsigned long startMillis = millis();
      flasherOn();
      motorClose(LEARN_MOTOR_SPEED);
      while (!isFullyClosed()) {
        delay(50);
        sampleCurrent();
        flasherUpdate(startMillis, FLASHER_TOGGLE_INTERVAL_LEARNING);
        if (isPhotocellBlocked() || overCurrentFlag || isEmergencyPressed() || digitalRead(PIN_RECEIVER)) {
          motorStop();
          flasherOff();
          delay(1000);
          overCurrentFlag = false;
          Serial.println("Learning: close measurement interrupted");
          return;
        }
      }
      motorStop();
      flasherOff();

      unsigned long totalCloseMs = millis() - startMillis;

      // Apply speed correction and split into phases
      closeTimeSlow = (unsigned long)(totalCloseMs * SLOW_PHASE_RATIO);
      closeTimeFull = (unsigned long)((totalCloseMs - closeTimeSlow) * speedFactor);

      Serial.print("Learning close — full: "); Serial.print(closeTimeFull);
      Serial.print(" slow: "); Serial.println(closeTimeSlow);
    }

    delay(3000);

    // 2. Measure open (closed -> open)
    Serial.println("Learning: measuring open time");
    {
      unsigned long startMillis = millis();
      flasherOn();
      motorOpen(LEARN_MOTOR_SPEED);
      while (!isFullyOpen()) {
        delay(50);
        sampleCurrent();
        flasherUpdate(startMillis, FLASHER_TOGGLE_INTERVAL_LEARNING);
        if (overCurrentFlag || digitalRead(PIN_RECEIVER)) {
          motorStop();
          flasherOff();
          delay(1000);
          overCurrentFlag = false;
          Serial.println("Learning: open measurement interrupted");
          return;
        }
      }
      motorStop();
      flasherOff();

      unsigned long totalOpenMs = millis() - startMillis;

      // Apply speed correction and split into phases
      openTimeSlow = (unsigned long)(totalOpenMs * SLOW_PHASE_RATIO);
      openTimeFull = (unsigned long)((totalOpenMs - openTimeSlow) * speedFactor);

      Serial.print("Learning open — full: "); Serial.print(openTimeFull);
      Serial.print(" slow: "); Serial.println(openTimeSlow);
    }
  }

  // ── Both directions measured successfully ──
  if (openTimeFull > 0 && closeTimeFull > 0) {
    learned = true;
    currentState = STATE_IDLE;

    // Create initial backup immediately after learning
    backupTiming();
    successfulCycles = 0;

    Serial.println("Learning complete.");
  }
}


// RECEIVER INTERRUPT
void onReceiverSignal() {
  unsigned long now = millis();
  if ((now - lastReceiverTime) > RECEIVER_DEBOUNCE_MS) {
    lastReceiverTime = now;
    Serial.println("Receiver: command received");
    checkStateTransition();
  }
}


// I2C CALLBACKS
void onI2CReceive() {
  i2cCommand = Wire.read();
}

void onI2CRequest() {
  Serial.print("I2C command: ");
  Serial.println(i2cCommand);

  if (i2cCommand == '0') {
    Wire.write("pong");

  } else if (i2cCommand == '1') {
    Wire.write("ok");
    checkStateTransition();

  } else if (i2cCommand == '2') {
    Wire.write("ok");
    if (isFullyClosed()) {
      checkStateTransition();
    }

  } else if (i2cCommand == '3') {
    int fap   = isFullyOpen()        ? 1 : 0;
    int fch   = isFullyClosed()      ? 1 : 0;
    int foto  = isPhotocellBlocked() ? 1 : 0;
    int coste = isEmergencyPressed() ? 1 : 0;
    int ric   = digitalRead(PIN_RECEIVER);

    int status = 0;
    if      (currentState == STATE_IDLE && fch)                  status = 0; // closed
    else if (currentState == STATE_IDLE && fap)                  status = 1; // open
    else if (currentState == STATE_IDLE && !fap && !fch)         status = 2; // stopped mid-travel
    else if (currentState == STATE_OPENING)                      status = 3; // opening
    else if (currentState == STATE_CLOSING)                      status = 4; // closing

    char curBuf[4];
    dtostrf(currentValue, 4, 2, curBuf);
    sprintf(i2cBuffer, "%d,%03lu,%d,%d,%d,%d,%s,%d", status, position, fap, fch, foto, coste, curBuf, ric);

    Wire.write(i2cBuffer);
    Serial.println(i2cBuffer);
  }
}


// SETUP
void setup() {
  pinMode(PIN_PHOTOCELL,        INPUT);
  pinMode(PIN_RECEIVER,         INPUT);
  pinMode(PIN_FLASHER,          OUTPUT);
  pinMode(PIN_EMERGENCY,        INPUT_PULLUP);
  pinMode(PIN_MOTOR_SPEED,      OUTPUT);
  pinMode(PIN_MOTOR_DIRECTION,  OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER), onReceiverSignal, RISING);

  currentSensorOffset = analogRead(PIN_CURRENT_SENSE);

  Serial.begin(9600);

  Wire.begin(8);
  Wire.setClock(250);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
}


// MAIN LOOP
void loop() {
  switch (currentState) {

    case STATE_IDLE:
      sampleCurrent();
      if (isFullyOpen())   position = 0;
      if (isFullyClosed()) position = 100;
      flasherOff();
      motorStop();
      break;

    case STATE_CLOSING:
      doClosing();
      break;

    case STATE_OPENING: {
      // Opening includes an auto-close cycle:
      //   open -> wait 60s -> close
      // If the close is interrupted (obstacle/emergency), the cycle repeats.
      do {
        returnFlag = false;
        doOpening();

        if (currentState != STATE_IDLE) {
          unsigned long waitStart = millis();
          while ((millis() - waitStart) < 60000) {
            sampleCurrent();
            delay(50);
            waitForPhotocellClear();

            if (currentState == STATE_CLOSING) {
              Serial.println("Closing command received during wait");
              break;
            }
            if (currentState != STATE_OPENING) break;
            if (isEmergencyPressed()) { currentState = STATE_IDLE; break; }
          }
          if (currentState == STATE_CLOSING || (currentState == STATE_OPENING && (millis() - waitStart) >= 60000)) {
            doClosing();
          }
        }
      } while (returnFlag);

      if (!returnFlag) currentState = STATE_IDLE;
      break;
    }

    case STATE_LEARNING:
      doLearning();
      break;
  }
}
