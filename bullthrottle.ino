#include <Joystick.h>

/*
 * Code for BullThrottle exergaming controller
 * Design by FTS Design Pte Ltd
 * https://ftsdesign.biz/projects/bullthrottle/
 * For hardware v.1
 * 21-May-2020
 * 
 * Based on:
 * https://github.com/MHeironimus/ArduinoJoystickLibrary
 * 
 * Windows 10 doesn't seem to recognize it as a gamepad when type set to 
 * JOYSTICK_TYPE_GAMEPAD, issue raised:
 * https://github.com/MHeironimus/ArduinoJoystickLibrary/issues/143
 */

Joystick_ Joystick = Joystick_(
    0x03, // Default: 0x03 JOYSTICK_DEFAULT_REPORT_ID
    JOYSTICK_TYPE_JOYSTICK, //joystickType JOYSTICK_TYPE_JOYSTICK | JOYSTICK_TYPE_GAMEPAD | JOYSTICK_TYPE_MULTI_AXIS
    6, //buttonCount JOYSTICK_DEFAULT_BUTTON_COUNT
    0, //hatSwitchCount JOYSTICK_DEFAULT_HATSWITCH_COUNT
    true, //includeXAxis
    true, //includeYAxis
    false, //includeZAxis
    false, //includeRxAxis
    false, //includeRyAxis
    false, //includeRzAxis
    false, //includeRudder
    true, //includeThrottle
    false, //includeAccelerator
    false, //includeBrake
    false //includeSteering
    );

const byte PIN_SENSOR = 7; // With interrupt PE6 7 INT4
const byte PIN_BUTTON_1 = 8; // Joystick button
const byte PIN_BUTTON_2 = 6; // Trigger button
const byte PIN_BUTTON_3 = 9;
const byte PIN_BUTTON_4 = 10;
const byte PIN_BUTTON_5 = 11;
const byte PIN_BUTTON_6 = 12;
const byte PIN_JOYSTICK_X = A0;
const byte PIN_JOYSTICK_Y = A1;

// Debouncing
unsigned long lastInterrupted = 0;
const unsigned long DEBOUNCE_MS = 100;

const unsigned long REFRESH_INTERVAL_MS = 100;
unsigned long lastSentMs = 0;

// Value constraints and mid values
// Range is 0x0-0x3FF
const int16_t MAX_AXIS = JOYSTICK_DEFAULT_AXIS_MAXIMUM; // 1023 = 0x3FF
const int16_t HALF_AXIS = MAX_AXIS >> 1;

// These constants define the throttle dynamics
const int16_t THROTTLE_BOOST = 400;
const int16_t RELEASE_PER_TICK = 70;
const int16_t DECAY_PER_TICK = 25;
uint16_t throttleBuffer = 0;
uint16_t throttle = 0;

const byte INVERT_THROTTLE = 1; // 0 for normal, 1 for inverse
const byte INVERT_X = 1;
const byte INVERT_Y = 0;

void setup() {
  pinMode(PIN_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), interruptHandler, LOW);

  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);
  pinMode(PIN_BUTTON_3, INPUT_PULLUP);
  pinMode(PIN_BUTTON_4, INPUT_PULLUP);
  pinMode(PIN_BUTTON_5, INPUT_PULLUP);
  pinMode(PIN_BUTTON_6, INPUT_PULLUP);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Joystick.begin(false);
//  blink(100);
}

void loop() {
  unsigned long now = millis();
  if (lastSentMs + REFRESH_INTERVAL_MS < now) {
    lastSentMs = now;

    int x = analogRead(PIN_JOYSTICK_X);
    if (INVERT_X) {
      x = invertAxis(x);
    }
    Joystick.setXAxis(x);
    
    int y = analogRead(PIN_JOYSTICK_Y);
    if (INVERT_Y) {
      y = invertAxis(y);
    }
    Joystick.setYAxis(y);

    calculateThrottle();
    if (INVERT_THROTTLE) {
      Joystick.setThrottle(invertAxis(throttle));
    } else {
      Joystick.setThrottle(throttle);
    }

    handleButton(digitalRead(PIN_BUTTON_1), 0);
    handleButton(digitalRead(PIN_BUTTON_2), 1);
    handleButton(digitalRead(PIN_BUTTON_3), 2);
    handleButton(digitalRead(PIN_BUTTON_4), 3);
    handleButton(digitalRead(PIN_BUTTON_5), 4);
    handleButton(digitalRead(PIN_BUTTON_6), 5);

    // Send state once for everything
    Joystick.sendState();
  }
}

uint8_t buttonStates[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};

void handleButton(uint8_t pinValue, uint8_t joystickButton) {
  if (pinValue != buttonStates[joystickButton]) {
    if (pinValue == LOW) {
      Joystick.pressButton(joystickButton);
    } else {
      Joystick.releaseButton(joystickButton);
    }
    buttonStates[joystickButton] = pinValue;
  }
}

uint16_t invertAxis(uint16_t value) {
  return MAX_AXIS - value;
}

void calculateThrottle() {
  if (throttleBuffer < RELEASE_PER_TICK) {
    throttle += throttleBuffer;
    throttleBuffer = 0;
  } else {
    throttle += RELEASE_PER_TICK;
    throttleBuffer -= RELEASE_PER_TICK;
  }

  if (throttle > MAX_AXIS) {
    throttle = MAX_AXIS;
  }
  
  if (throttle > DECAY_PER_TICK) {
    throttle -= DECAY_PER_TICK;
  } else {
    throttle = 0;
  }
}

void interruptHandler() {
  unsigned long now = millis();
  if (lastInterrupted + DEBOUNCE_MS < now) {
    lastInterrupted = now;
    throttleBuffer += THROTTLE_BOOST;
    if (throttleBuffer > MAX_AXIS) {
      throttleBuffer = MAX_AXIS;
    }
  }
}

void blink(unsigned long durationMs) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(durationMs);
  digitalWrite(LED_BUILTIN, LOW);
}
