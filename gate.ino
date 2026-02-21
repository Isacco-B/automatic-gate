#include <Wire.h>

// ═══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════
const int PIN_PHOTOCELL       = 2;
const int PIN_RECEIVER        = 3;
const int PIN_FLASHER         = 4;
const int PIN_EMERGENCY       = 5;
const int PIN_MOTOR_SPEED     = 9;   // PWM @ 122 Hz
const int PIN_MOTOR_DIRECTION = 7;
const int PIN_LIMIT_OPEN      = A0;
const int PIN_LIMIT_CLOSED    = A1;
const int PIN_CURRENT_SENSE   = A2;

// ═══════════════════════════════════════════════════════════════════
// CONSTANTS
// ═══════════════════════════════════════════════════════════════════
const int   LIMIT_SWITCH_THRESHOLD  = 200;
const float OVERCURRENT_DEFAULT     = 11.0f;
const float OVERCURRENT_SLOW_PHASE  = 7.0f;
const int   CURRENT_SAMPLES         = 300;
const int   RECEIVER_DEBOUNCE_MS    = 1500;
const int   LEARN_MOTOR_SPEED       = 98;
const float ADAPT_WEIGHT            = 0.25f;
const float SLOW_PHASE_RATIO        = 0.25f;
const int   SAFE_RECOVERY_SPEED     = 150;
const int   SLOW_PHASE_MIN_SPEED    = 85;
const float SLOW_PHASE_MIN_RATIO    = 0.3f;
const int   MAX_DIRECTION_CHANGES   = 5;
const int   BACKUP_INTERVAL         = 10;

const unsigned long OPEN_WAIT_MS                     = 60000;
const unsigned long WALK_THROUGH_OPEN_PCT            = 80;
const unsigned long FLASHER_TOGGLE_INTERVAL          = 500;
const unsigned long FLASHER_TOGGLE_INTERVAL_LEARNING = 200;

const unsigned long RECOVERY_INTELLIGENT_OPEN_THRESHOLD  = 70;
const unsigned long RECOVERY_INTELLIGENT_CLOSE_THRESHOLD = 40;

const bool OVERCURRENT_ENABLED = false;

// ═══════════════════════════════════════════════════════════════════
// STATE MACHINE
// ═══════════════════════════════════════════════════════════════════
enum GateState {
  STATE_IDLE         = 0,
  STATE_CLOSING      = 1,
  STATE_OPENING      = 2,
  STATE_LEARNING     = 3,
  STATE_OPEN_WAITING = 4
};

volatile GateState currentState = STATE_IDLE;

// ═══════════════════════════════════════════════════════════════════
// TIMING
// ═══════════════════════════════════════════════════════════════════
unsigned long openTimeFull  = 0, openTimeSlow  = 0;
unsigned long closeTimeFull = 0, closeTimeSlow = 0;

unsigned long openTimeFullBackup  = 0, openTimeSlowBackup  = 0;
unsigned long closeTimeFullBackup = 0, closeTimeSlowBackup = 0;

int successfulCycles = 0;

// ═══════════════════════════════════════════════════════════════════
// POSITION & TRAVEL STATE
// ═══════════════════════════════════════════════════════════════════
unsigned long position          = 0;
unsigned long lastKnownPosition = 0;
bool          positionValid     = false;
int           directionChanges  = 0;

// True only after a cycle completes by reaching an endstop normally.
bool cycleWasComplete = false;

// ═══════════════════════════════════════════════════════════════════
// MISC GLOBALS
// ═══════════════════════════════════════════════════════════════════
bool          learned              = false;
bool          walkThroughMode      = false;
int           currentSensorOffset  = 0;
float         currentValue         = 0.0f;
float         overCurrentThreshold = OVERCURRENT_DEFAULT;
bool          overCurrentFlag      = false;
unsigned long lastReceiverTime     = 0;
char          i2cCommand           = 0;

// ═══════════════════════════════════════════════════════════════════
// HARDWARE HELPERS
// ═══════════════════════════════════════════════════════════════════

bool isFullyOpen()        { return analogRead(PIN_LIMIT_OPEN)   < LIMIT_SWITCH_THRESHOLD; }
bool isFullyClosed()      { return analogRead(PIN_LIMIT_CLOSED) < LIMIT_SWITCH_THRESHOLD; }
bool isEmergencyPressed() { return digitalRead(PIN_EMERGENCY) == HIGH; }
bool isPhotocellBlocked() { return digitalRead(PIN_PHOTOCELL)  == LOW; }

void motorStop()           { analogWrite(PIN_MOTOR_SPEED, 0); }
void motorOpen(int speed)  { digitalWrite(PIN_MOTOR_DIRECTION, HIGH); analogWrite(PIN_MOTOR_SPEED, speed); }
void motorClose(int speed) { digitalWrite(PIN_MOTOR_DIRECTION, LOW);  analogWrite(PIN_MOTOR_SPEED, speed); }

void flasherOn()  { digitalWrite(PIN_FLASHER, HIGH); }
void flasherOff() { digitalWrite(PIN_FLASHER, LOW);  }
void flasherUpdate(unsigned long startMillis, unsigned long interval) {
  digitalWrite(PIN_FLASHER, ((millis() - startMillis) / interval) % 2 == 0 ? HIGH : LOW);
}

void waitForPhotocellClear() {
  while (isPhotocellBlocked())
    if (currentState == STATE_IDLE) return;
}

// ═══════════════════════════════════════════════════════════════════
// CURRENT SENSOR
// ═══════════════════════════════════════════════════════════════════

void sampleCurrent() {
  float sum = 0.0f;
  for (int i = 0; i < CURRENT_SAMPLES; i++)
    sum += (analogRead(PIN_CURRENT_SENSE) - currentSensorOffset) / 21.0f;
  currentValue = max(0.0f, sum / CURRENT_SAMPLES);

  if (OVERCURRENT_ENABLED && currentValue >= overCurrentThreshold) {
    overCurrentFlag = true;
    Serial.print("OverCurrent: "); Serial.println(currentValue);
  }
}

// ═══════════════════════════════════════════════════════════════════
// MOTION HELPERS
// ═══════════════════════════════════════════════════════════════════

// Returns a linearly decreasing speed as the slow phase progresses.
int slowPhaseSpeed(unsigned long elapsed, unsigned long totalSlowTime) {
  if (totalSlowTime == 0) return SLOW_PHASE_MIN_SPEED;
  float ratio = min(1.0f, (float)elapsed / (float)totalSlowTime);
  return (int)(255.0f - (255.0f - SLOW_PHASE_MIN_SPEED) * ratio);
}

// Estimates gate position from elapsed time.
unsigned long estimatePosition(unsigned long elapsed, unsigned long totalTime, bool opening) {
  if (totalTime == 0) return 50;
  unsigned long pct = min(100UL, elapsed * 100 / totalTime);
  return opening ? (100UL - pct) : pct;
}

// ═══════════════════════════════════════════════════════════════════
// TIMING ADAPTATION
// ═══════════════════════════════════════════════════════════════════

void backupTiming() {
  openTimeFullBackup  = openTimeFull;  openTimeSlowBackup  = openTimeSlow;
  closeTimeFullBackup = closeTimeFull; closeTimeSlowBackup = closeTimeSlow;
  Serial.println("Timing backup saved");
}

void restoreTiming() {
  if (openTimeFullBackup > 0 && closeTimeFullBackup > 0) {
    openTimeFull  = openTimeFullBackup;  openTimeSlow  = openTimeSlowBackup;
    closeTimeFull = closeTimeFullBackup; closeTimeSlow = closeTimeSlowBackup;
    Serial.println("Timing restored from backup");
  } else {
    Serial.println("No backup available, keeping current timing");
  }
}

void updateOpenTiming(unsigned long measuredFull, unsigned long measuredSlow) {
  openTimeFull = (unsigned long)((1.0f - ADAPT_WEIGHT) * openTimeFull + ADAPT_WEIGHT * measuredFull);
  openTimeSlow = (unsigned long)((1.0f - ADAPT_WEIGHT) * openTimeSlow + ADAPT_WEIGHT * measuredSlow);
  Serial.print("Adaptive open  — full: "); Serial.print(openTimeFull);
  Serial.print("  slow: "); Serial.println(openTimeSlow);
}

void updateCloseTiming(unsigned long measuredFull, unsigned long measuredSlow) {
  closeTimeFull = (unsigned long)((1.0f - ADAPT_WEIGHT) * closeTimeFull + ADAPT_WEIGHT * measuredFull);
  closeTimeSlow = (unsigned long)((1.0f - ADAPT_WEIGHT) * closeTimeSlow + ADAPT_WEIGHT * measuredSlow);
  Serial.print("Adaptive close — full: "); Serial.print(closeTimeFull);
  Serial.print("  slow: "); Serial.println(closeTimeSlow);
}

bool validateSlowPhase(unsigned long expectedMs, unsigned long actualMs, const char* dir) {
  if (expectedMs == 0) return true;
  float ratio = (float)actualMs / (float)expectedMs;
  Serial.println(expectedMs); Serial.println(ratio);
  if (ratio < SLOW_PHASE_MIN_RATIO) {
    Serial.print("WARNING: "); Serial.print(dir);
    Serial.print(" slow phase too short! Expected: "); Serial.print(expectedMs);
    Serial.print("ms, Actual: "); Serial.print(actualMs);
    Serial.print("ms ("); Serial.print(ratio * 100.0f); Serial.println("%)");
    return false;
  }
  return true;
}

void tryAdaptTiming(unsigned long travelStart, unsigned long fullPhaseEnd,
                    unsigned long expectedSlow, bool prevCycleWasComplete,
                    void (*updateFn)(unsigned long, unsigned long),
                    const char* dir) {
  if (fullPhaseEnd == 0) return;

  unsigned long measuredFull = fullPhaseEnd - travelStart;
  unsigned long measuredSlow = millis() - fullPhaseEnd;
  bool valid = validateSlowPhase(expectedSlow, measuredSlow, dir);

  if (valid && !prevCycleWasComplete) {
    updateFn(measuredFull, measuredSlow);
    if (++successfulCycles >= BACKUP_INTERVAL) {
      backupTiming();
      successfulCycles = 0;
    }
  } else if (!valid) {
    restoreTiming();
    successfulCycles = 0;
  }
}

// ═══════════════════════════════════════════════════════════════════
// STATE MACHINE TRANSITION
// ═══════════════════════════════════════════════════════════════════

void checkStateTransition() {
  if (!learned) { currentState = STATE_LEARNING; return; }

  switch (currentState) {
    case STATE_IDLE:
      if      (isFullyOpen())   currentState = STATE_CLOSING;
      else if (isFullyClosed()) currentState = STATE_OPENING;
      else                      currentState = STATE_CLOSING;
      break;
    case STATE_OPEN_WAITING:
      currentState = STATE_CLOSING;
      break;
    case STATE_CLOSING:
      currentState = STATE_OPENING;
      break;
    case STATE_OPENING:
      currentState = isFullyOpen() ? STATE_CLOSING : STATE_IDLE;
      break;
    default: break;
  }
}

// ═══════════════════════════════════════════════════════════════════
// OPENING
// ═══════════════════════════════════════════════════════════════════

void doOpening() {
  overCurrentThreshold = OVERCURRENT_DEFAULT;
  flasherOn();

  bool isWalkThrough = walkThroughMode;
  walkThroughMode = false;

  bool prevCycleWasComplete = cycleWasComplete;
  cycleWasComplete = false;

  // ── Recovery: gate is not at the closed endstop ──────────────────
  if (!isFullyClosed()) {
    bool useIntelligentRecovery = false;
    unsigned long estimatedTimeRemaining = 0;

    if (positionValid && directionChanges < MAX_DIRECTION_CHANGES && lastKnownPosition > RECOVERY_INTELLIGENT_OPEN_THRESHOLD) {
      useIntelligentRecovery = true;
      unsigned long estimatedFullTime = (100 - lastKnownPosition) * openTimeFull / 100;
      estimatedTimeRemaining = min(estimatedFullTime, openTimeFull / 2);
      Serial.print("Opening: intelligent recovery from pos "); Serial.println(lastKnownPosition);
    }

    if (!useIntelligentRecovery) {
      // ── Safe recovery (low speed, no timing update) ───────────────
      Serial.println("Opening: safe recovery");
      unsigned long safeStartPos = (positionValid && lastKnownPosition <= 100) ? lastKnownPosition : 50;
      position = safeStartPos;

      unsigned long totalOpenTime = openTimeFull + openTimeSlow;
      unsigned long safeRecoveryEstimate = totalOpenTime > 0
        ? (unsigned long)((float)totalOpenTime * safeStartPos / 100.0f * 255.0f / SAFE_RECOVERY_SPEED)
        : 0;

      motorOpen(SAFE_RECOVERY_SPEED);
      unsigned long startMillis = millis();

      while (!isFullyOpen()) {
        delay(100);
        sampleCurrent();

        if (safeRecoveryEstimate > 0) {
          unsigned long elapsed = millis() - startMillis;
          unsigned long travel = (unsigned long)((float)elapsed * safeStartPos / (float)safeRecoveryEstimate);
          position = (travel >= safeStartPos) ? 0 : (safeStartPos - travel);
        }

        if (overCurrentFlag || currentState == STATE_IDLE || isEmergencyPressed()) {
          motorStop(); flasherOff();
          overCurrentFlag = false;
          overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
          currentState = STATE_IDLE;
          lastKnownPosition = position;
          positionValid = (position > 0 && position < 100);
          return;
        }
        flasherUpdate(startMillis, FLASHER_TOGGLE_INTERVAL);
      }

      position = 0;
      motorStop(); flasherOff();
      overCurrentThreshold = OVERCURRENT_DEFAULT;
      lastKnownPosition = 0;
      positionValid = false;
      directionChanges = 0;
      cycleWasComplete = true;
      currentState = STATE_OPEN_WAITING;
      return;
    }

    // ── Intelligent (fast) recovery ───────────────────────────────
    Serial.print("Opening: fast recovery, time limit: "); Serial.println(estimatedTimeRemaining);
    unsigned long recoveryStartPos = lastKnownPosition;
    position = recoveryStartPos;

    unsigned long travelStart = millis();
    motorOpen(255);
    unsigned long fullPhaseEnd = 0;
    unsigned long totalRecoveryTime = estimatedTimeRemaining + openTimeSlow;

    while (!isFullyOpen()) {
      delay(100);
      unsigned long elapsed = millis() - travelStart;

      if (totalRecoveryTime > 0) {
        unsigned long travel = (unsigned long)((float)elapsed * recoveryStartPos / (float)totalRecoveryTime);
        position = (travel >= recoveryStartPos) ? 0 : (recoveryStartPos - travel);
      }

      sampleCurrent();

      if (elapsed >= estimatedTimeRemaining && fullPhaseEnd == 0) {
        fullPhaseEnd = millis();
        Serial.println("Opening: recovery entering slow phase");
      }
      if (fullPhaseEnd > 0) {
        motorOpen(slowPhaseSpeed(millis() - fullPhaseEnd, openTimeSlow));
        overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
      }

      if (overCurrentFlag || currentState == STATE_IDLE || isEmergencyPressed()) {
        motorStop(); flasherOff();
        overCurrentFlag = false;
        overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
        currentState = STATE_IDLE;
        lastKnownPosition = position;
        positionValid = true;
        directionChanges++;
        return;
      }
      flasherUpdate(travelStart, FLASHER_TOGGLE_INTERVAL);
    }

    position = 0;
    motorStop(); flasherOff();
    overCurrentThreshold = OVERCURRENT_DEFAULT;
    lastKnownPosition = 0;
    positionValid = false;
    directionChanges = 0;
    cycleWasComplete = true;
    currentState = STATE_OPEN_WAITING;
    return;
  }

  // ── Full travel from closed endstop ──────────────────────────────
  Serial.println("Opening: full travel");
  if (isWalkThrough) {
    Serial.print("Opening: walk-through mode, stopping at ");
    Serial.print(WALK_THROUGH_OPEN_PCT); Serial.println("%");
  }
  position = 100;

  unsigned long travelStart = millis();
  motorOpen(255);
  unsigned long fullPhaseEnd = 0;

  while (!isFullyOpen()) {
    delay(100);
    unsigned long elapsed = millis() - travelStart;
    position = estimatePosition(elapsed, openTimeFull + openTimeSlow, true);

    sampleCurrent();

    if (elapsed >= openTimeFull && fullPhaseEnd == 0) {
      fullPhaseEnd = millis();
      Serial.println("Opening: entering slow phase");
    }
    if (fullPhaseEnd > 0) {
      motorOpen(slowPhaseSpeed(millis() - fullPhaseEnd, openTimeSlow));
      overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
    }

    // Walk-through: stop when the gate has opened to the requested percentage.
    if (isWalkThrough && position <= (100UL - WALK_THROUGH_OPEN_PCT)) {
      motorStop(); flasherOff();
      overCurrentThreshold = OVERCURRENT_DEFAULT;
      lastKnownPosition = position;
      positionValid = true;
      directionChanges = 0;
      cycleWasComplete = false;
      currentState = STATE_OPEN_WAITING;
      Serial.print("Opening: walk-through stop at pos "); Serial.println(position);
      return;
    }

    if (overCurrentFlag || currentState == STATE_IDLE || isEmergencyPressed()) {
      motorStop(); flasherOff();
      overCurrentFlag = false;
      overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
      currentState = STATE_IDLE;
      lastKnownPosition = position;
      positionValid = true;
      directionChanges++;
      cycleWasComplete = false;
      return;
    }
    flasherUpdate(travelStart, FLASHER_TOGGLE_INTERVAL);
  }

  position = 0;
  tryAdaptTiming(travelStart, fullPhaseEnd, openTimeSlow, prevCycleWasComplete, updateOpenTiming, "Opening");

  motorStop(); flasherOff();
  overCurrentThreshold = OVERCURRENT_DEFAULT;
  lastKnownPosition = 0;
  positionValid = false;
  directionChanges = 0;
  cycleWasComplete = true;
  currentState = STATE_OPEN_WAITING;
}

// ═══════════════════════════════════════════════════════════════════
// CLOSING
// ═══════════════════════════════════════════════════════════════════

void doClosing() {
  overCurrentThreshold = OVERCURRENT_DEFAULT;
  currentState = STATE_CLOSING;
  flasherOn();

  bool prevCycleWasComplete = cycleWasComplete;
  cycleWasComplete = false;

  waitForPhotocellClear();

  // ── Recovery: gate is not at the open endstop ────────────────────
  if (!isFullyOpen()) {
    bool useIntelligentRecovery = false;
    unsigned long estimatedTimeRemaining = 0;

    if (positionValid && directionChanges < MAX_DIRECTION_CHANGES && lastKnownPosition < RECOVERY_INTELLIGENT_CLOSE_THRESHOLD) {
      useIntelligentRecovery = true;
      unsigned long estimatedFullTime = lastKnownPosition * closeTimeFull / 100;
      estimatedTimeRemaining = min(estimatedFullTime, closeTimeFull / 2);
      Serial.print("Closing: intelligent recovery from pos "); Serial.println(lastKnownPosition);
    }

    if (!useIntelligentRecovery) {
      // ── Safe recovery (low speed, no timing update) ───────────────
      Serial.println("Closing: safe recovery");
      unsigned long safeStartPos = (positionValid && lastKnownPosition <= 100) ? lastKnownPosition : 50;
      position = safeStartPos;

      unsigned long remaining = 100 - safeStartPos;
      unsigned long totalCloseTime = closeTimeFull + closeTimeSlow;
      unsigned long safeRecoveryEstimate = (totalCloseTime > 0 && remaining > 0)
        ? (unsigned long)((float)totalCloseTime * remaining / 100.0f * 255.0f / SAFE_RECOVERY_SPEED)
        : 0;

      waitForPhotocellClear();
      motorClose(SAFE_RECOVERY_SPEED);
      unsigned long startMillis = millis();

      while (!isFullyClosed()) {
        delay(100);
        sampleCurrent();

        if (safeRecoveryEstimate > 0) {
          unsigned long elapsed = millis() - startMillis;
          unsigned long travel = (unsigned long)((float)elapsed * remaining / (float)safeRecoveryEstimate);
          position = min(100UL, safeStartPos + travel);
        }

        if (isPhotocellBlocked() || overCurrentFlag || currentState == STATE_OPENING || isEmergencyPressed()) {
          motorStop(); delay(2000); flasherOff();
          overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
          overCurrentFlag = false;
          currentState = STATE_OPENING;
          lastKnownPosition = position;
          positionValid = (position > 0 && position < 100);
          return;
        }
        flasherUpdate(startMillis, FLASHER_TOGGLE_INTERVAL);
      }

      position = 100;
      motorStop(); flasherOff();
      overCurrentThreshold = OVERCURRENT_DEFAULT;
      currentState = STATE_IDLE;
      lastKnownPosition = 100;
      positionValid = false;
      directionChanges = 0;
      cycleWasComplete = true;
      return;
    }

    // ── Intelligent (fast) recovery ───────────────────────────────
    Serial.print("Closing: fast recovery, time limit: "); Serial.println(estimatedTimeRemaining);
    unsigned long recoveryStartPos = lastKnownPosition;
    position = recoveryStartPos;

    unsigned long travelStart = millis();
    motorClose(255);
    unsigned long fullPhaseEnd = 0;
    unsigned long totalRecoveryTime = estimatedTimeRemaining + closeTimeSlow;
    unsigned long remainingTravel = 100 - recoveryStartPos;

    while (!isFullyClosed()) {
      delay(100);
      unsigned long elapsed = millis() - travelStart;

      if (totalRecoveryTime > 0 && remainingTravel > 0) {
        unsigned long travel = (unsigned long)((float)elapsed * remainingTravel / (float)totalRecoveryTime);
        position = min(100UL, recoveryStartPos + travel);
      }

      sampleCurrent();

      if (elapsed >= estimatedTimeRemaining && fullPhaseEnd == 0) {
        fullPhaseEnd = millis();
        Serial.println("Closing: recovery entering slow phase");
      }
      if (fullPhaseEnd > 0) {
        motorClose(slowPhaseSpeed(millis() - fullPhaseEnd, closeTimeSlow));
        overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
      }

      if (isPhotocellBlocked() || overCurrentFlag || currentState == STATE_OPENING || isEmergencyPressed()) {
        motorStop(); delay(2000); flasherOff();
        overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
        overCurrentFlag = false;
        currentState = STATE_OPENING;
        lastKnownPosition = position;
        positionValid = true;
        directionChanges++;
        return;
      }
      flasherUpdate(travelStart, FLASHER_TOGGLE_INTERVAL);
    }

    position = 100;
    motorStop(); flasherOff();
    overCurrentThreshold = OVERCURRENT_DEFAULT;
    currentState = STATE_IDLE;
    lastKnownPosition = 100;
    positionValid = false;
    directionChanges = 0;
    cycleWasComplete = true;
    return;
  }

  // ── Full travel from open endstop ────────────────────────────────
  Serial.println("Closing: full travel");
  position = 0;

  unsigned long travelStart = millis();
  motorClose(255);
  unsigned long fullPhaseEnd = 0;

  while (!isFullyClosed()) {
    delay(100);
    unsigned long elapsed = millis() - travelStart;
    position = estimatePosition(elapsed, closeTimeFull + closeTimeSlow, false);

    sampleCurrent();

    if (elapsed >= closeTimeFull && fullPhaseEnd == 0) {
      fullPhaseEnd = millis();
      Serial.println("Closing: entering slow phase");
    }
    if (fullPhaseEnd > 0) {
      motorClose(slowPhaseSpeed(millis() - fullPhaseEnd, closeTimeSlow));
      overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
    }

    if (isPhotocellBlocked() || overCurrentFlag || currentState == STATE_OPENING || isEmergencyPressed()) {
      motorStop(); delay(2000); flasherOff();
      overCurrentThreshold = OVERCURRENT_SLOW_PHASE;
      overCurrentFlag = false;
      currentState = STATE_OPENING;
      lastKnownPosition = position;
      positionValid = true;
      directionChanges++;
      cycleWasComplete = false;
      return;
    }
    flasherUpdate(travelStart, FLASHER_TOGGLE_INTERVAL);
  }

  position = 100;
  tryAdaptTiming(travelStart, fullPhaseEnd, closeTimeSlow, prevCycleWasComplete, updateCloseTiming, "Closing");

  motorStop(); flasherOff();
  overCurrentThreshold = OVERCURRENT_DEFAULT;
  currentState = STATE_IDLE;
  lastKnownPosition = 100;
  positionValid = false;
  directionChanges = 0;
  cycleWasComplete = true;
}

// ═══════════════════════════════════════════════════════════════════
// LEARNING — helpers
// ═══════════════════════════════════════════════════════════════════

// Drives the gate to fully open and records openTimeFull / openTimeSlow.
// Returns false if interrupted.
bool learnMeasureOpen(float speedFactor) {
  Serial.println("Learning: measuring open time");
  unsigned long startMillis = millis();
  flasherOn();
  motorOpen(LEARN_MOTOR_SPEED);

  while (!isFullyOpen()) {
    delay(50);
    sampleCurrent();
    flasherUpdate(startMillis, FLASHER_TOGGLE_INTERVAL_LEARNING);
    if (overCurrentFlag || digitalRead(PIN_RECEIVER)) {
      motorStop(); flasherOff(); delay(1000);
      overCurrentFlag = false;
      Serial.println("Learning: open measurement interrupted");
      return false;
    }
  }
  motorStop(); flasherOff();

  unsigned long totalMs = millis() - startMillis;
  openTimeSlow = (unsigned long)(totalMs * SLOW_PHASE_RATIO);
  openTimeFull = (unsigned long)((totalMs - openTimeSlow) * speedFactor);
  Serial.print("Learning open  — full: "); Serial.print(openTimeFull);
  Serial.print("  slow: "); Serial.println(openTimeSlow);
  return true;
}

// Drives the gate to fully closed and records closeTimeFull / closeTimeSlow.
// Returns false if interrupted.
bool learnMeasureClose(float speedFactor) {
  Serial.println("Learning: measuring close time");

  while (isPhotocellBlocked()) {
    delay(100);
    if (digitalRead(PIN_RECEIVER)) {
      Serial.println("Learning: aborted during photocell wait");
      return false;
    }
  }

  unsigned long startMillis = millis();
  flasherOn();
  motorClose(LEARN_MOTOR_SPEED);

  while (!isFullyClosed()) {
    delay(50);
    sampleCurrent();
    flasherUpdate(startMillis, FLASHER_TOGGLE_INTERVAL_LEARNING);
    if (isPhotocellBlocked() || overCurrentFlag || isEmergencyPressed() || digitalRead(PIN_RECEIVER)) {
      motorStop(); flasherOff(); delay(1000);
      overCurrentFlag = false;
      Serial.println("Learning: close measurement interrupted");
      return false;
    }
  }
  motorStop(); flasherOff();

  unsigned long totalMs = millis() - startMillis;
  closeTimeSlow = (unsigned long)(totalMs * SLOW_PHASE_RATIO);
  closeTimeFull = (unsigned long)((totalMs - closeTimeSlow) * speedFactor);
  Serial.print("Learning close — full: "); Serial.print(closeTimeFull);
  Serial.print("  slow: "); Serial.println(closeTimeSlow);
  return true;
}

// ═══════════════════════════════════════════════════════════════════
// LEARNING
// ═══════════════════════════════════════════════════════════════════

void doLearning() {
  Serial.println("Learning");
  flasherOff();
  motorStop();

  const float speedFactor = (float)LEARN_MOTOR_SPEED / 255.0f;

  bool startFromClosed = isFullyClosed();
  bool startFromOpen   = isFullyOpen();

  if (!startFromClosed && !startFromOpen) {
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
        motorStop(); flasherOff(); delay(1000);
        overCurrentFlag = false;
        Serial.println("Learning: recovery interrupted");
        return;
      }
    }
    motorStop(); flasherOff(); delay(1000);
    startFromClosed = true;
  }

  bool success;
  if (startFromClosed) {
    success = learnMeasureOpen(speedFactor);
    if (!success) return;
    delay(3000);
    success = learnMeasureClose(speedFactor);
  } else {
    success = learnMeasureClose(speedFactor);
    if (!success) return;
    delay(3000);
    success = learnMeasureOpen(speedFactor);
  }

  if (!success) return;

  if (openTimeFull > 0 && closeTimeFull > 0) {
    learned = true;
    currentState = STATE_IDLE;
    backupTiming();
    successfulCycles = 0;
    Serial.println("Learning complete.");
  }
}

// ═══════════════════════════════════════════════════════════════════
// RECEIVER INTERRUPT
// ═══════════════════════════════════════════════════════════════════

void onReceiverSignal() {
  unsigned long now = millis();
  if ((now - lastReceiverTime) > RECEIVER_DEBOUNCE_MS) {
    lastReceiverTime = now;
    Serial.println("Receiver: command received");
    checkStateTransition();
  }
}

// ═══════════════════════════════════════════════════════════════════
// I2C CALLBACKS
// ═══════════════════════════════════════════════════════════════════

void onI2CReceive(int howMany) {
  if (howMany > 0) i2cCommand = Wire.read();
}

void onI2CRequest() {
  Serial.print("I2C command: "); Serial.println(i2cCommand);

  if (i2cCommand == '0') {
    Wire.write("pong");

  } else if (i2cCommand == '1') {
    Wire.write("ok");
    checkStateTransition();

  } else if (i2cCommand == '2') {
    Wire.write("ok");
    if (currentState == STATE_IDLE && isFullyClosed()) {
      walkThroughMode = true;
      checkStateTransition();
    }

  } else if (i2cCommand == '3') {
    int fap   = isFullyOpen()        ? 1 : 0;
    int fch   = isFullyClosed()      ? 1 : 0;
    int foto  = isPhotocellBlocked() ? 1 : 0;
    int coste = isEmergencyPressed() ? 1 : 0;
    int ric   = digitalRead(PIN_RECEIVER);

    int status;
    if      (currentState == STATE_IDLE && fch)  status = 0;  // closed
    else if (currentState == STATE_OPEN_WAITING) status = 1;  // open, waiting to auto-close
    else if (currentState == STATE_IDLE)         status = 2;  // stopped mid-travel
    else if (currentState == STATE_OPENING)      status = 3;  // opening
    else                                         status = 4;  // closing

    char curBuf[8];
    dtostrf(currentValue, 4, 2, curBuf);

    char i2cBuffer[32];
    sprintf(i2cBuffer, "%d,%03lu,%d,%d,%d,%d,%s,%d", status, position, fap, fch, foto, coste, curBuf, ric);
    Wire.write(i2cBuffer);
    Serial.println(i2cBuffer);

  } else if (i2cCommand == '4') {
    Wire.write("ok");
    if (learned) { learned = false; currentState = STATE_LEARNING; Serial.println("Learning mode triggered via I2C");}
  }
}

// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════

void setup() {
  pinMode(PIN_PHOTOCELL,       INPUT);
  pinMode(PIN_RECEIVER,        INPUT);
  pinMode(PIN_FLASHER,         OUTPUT);
  pinMode(PIN_EMERGENCY,       INPUT_PULLUP);
  pinMode(PIN_MOTOR_SPEED,     OUTPUT);
  pinMode(PIN_MOTOR_DIRECTION, OUTPUT);

  // ~122 Hz PWM
  TCCR1B = (TCCR1B & 0b11111000) | 0x04;

  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER), onReceiverSignal, RISING);

  currentSensorOffset = analogRead(PIN_CURRENT_SENSE);

  Serial.begin(9600);

  Wire.begin(8);
  Wire.setClock(250);

  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
}

// ═══════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════

void loop() {
  switch (currentState) {

    case STATE_IDLE:
      sampleCurrent();
      if      (isFullyOpen())   position = 0;
      else if (isFullyClosed()) position = 100;
      flasherOff();
      motorStop();
      break;

    case STATE_OPENING:
      doOpening();
      break;

    case STATE_OPEN_WAITING: {
      Serial.println("Open waiting: auto-close in 60s");
      unsigned long waitStart = millis();
      while (currentState == STATE_OPEN_WAITING) {
        sampleCurrent();
        delay(50);
        waitForPhotocellClear();
        if (isEmergencyPressed())                    { currentState = STATE_IDLE;   break; }
        if ((millis() - waitStart) >= OPEN_WAIT_MS)  { currentState = STATE_CLOSING; break; }
      }
      if (currentState == STATE_CLOSING) doClosing();
      break;
    }

    case STATE_CLOSING:
      doClosing();
      break;

    case STATE_LEARNING:
      doLearning();
      break;
  }
}
