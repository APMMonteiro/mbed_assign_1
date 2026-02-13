#include "esp32-hal-timer.h"


const bool DEBUG = false;
const bool PRINTER = false;

int IRAM_ATTR DebugSlower();

// Parameters
// first 4 letters = "MONTE" = 13 12 13 7 5
// alt behaviour 5 % 4 + 1 = 2
const unsigned long a = 13 * 100;
const unsigned long b = 12 * 100;
const unsigned long c = 13 + 4;
const unsigned long d = 7 * 500;
const unsigned long T_SYNC_ON = 50 * DebugSlower();

const unsigned long BASE_PULSE_LEN = a * DebugSlower();
const unsigned long PULSE_LOW_LEN = b * DebugSlower();
const unsigned long NUM_PULSES = c;
const unsigned long PULSE_PAUSE_LEN = d * DebugSlower();

bool isEnabled = false;
bool isStateAlternative = false;

// todo force n > 2 ?
inline unsigned int T_ON_n(uint8_t n) {
  return (a + (n - 1) * 50) * DebugSlower();
}

const int RED_OUTPUT_ENABLE_PIN = 22;    // wire from G22 to GND
const int GREEN_OUTPUT_SELECT_PIN = 18;  // wire from G18 to GND
const int DATA_LED = 32;
const int SIGNAL_LED = 12;
const int EXTRA_GND = 25;
const int DEBOUNCE_CD = 250;

enum pulse_state_t {
  STATE_STOPPED,
  STATE_SYNC,
  STATE_PULSING,
  STATE_PAUSE
};

void IRAM_ATTR ISR_SELECT_Handler();
void IRAM_ATTR ISR_ENABLE_Handler();
void stateMachine();

hw_timer_t* timer = NULL;

pulse_state_t state = STATE_STOPPED;
int dataPinState = LOW;
int signalPinState = LOW;
int dataPulseNum = 0;

volatile bool timerTriggered = false;
void IRAM_ATTR timerISR() {
  timerTriggered = true;
}

bool reachedEnd;
void stateMachine() {
  u_int64_t nextAlarm = 0;

  switch (state) {

    case STATE_STOPPED:
      timerStop(timer);

      dataPinState = LOW;
      digitalWrite(DATA_LED, dataPinState);

      signalPinState = LOW;
      digitalWrite(SIGNAL_LED, signalPinState);

      if (PRINTER) { Serial.println("Stopped"); }
      return;

    case STATE_SYNC:
      state = STATE_PULSING;
      nextAlarm = T_SYNC_ON;

      dataPinState = LOW;
      digitalWrite(DATA_LED, dataPinState);

      dataPulseNum = isStateAlternative ? NUM_PULSES + 1 : 0;

      signalPinState = HIGH;
      digitalWrite(SIGNAL_LED, signalPinState);

      if (PRINTER) { Serial.println("Syncing"); }
      break;

    case STATE_PULSING:
      if (signalPinState == HIGH) {
        signalPinState = LOW;
        digitalWrite(SIGNAL_LED, signalPinState);
      }

      if (dataPinState == HIGH) {
        nextAlarm = PULSE_LOW_LEN;
        dataPinState = LOW;

        if (PRINTER) { Serial.println("LOW"); }
      } else {
        isStateAlternative ? dataPulseNum-- : dataPulseNum++;  // starts on 1
        nextAlarm = T_ON_n(dataPulseNum);
        dataPinState = HIGH;

        if (PRINTER) {
          Serial.print(dataPulseNum);
          Serial.println(" HIGH");
        }
      }

      reachedEnd = isStateAlternative ? dataPulseNum <= 0 : dataPulseNum >= NUM_PULSES + 1;
      if (!reachedEnd) {
        break;
      }
      if (PRINTER) { Serial.println("Pause"); }
      state = STATE_PAUSE;

    case STATE_PAUSE:
      dataPinState = LOW;
      dataPulseNum = isStateAlternative ? NUM_PULSES + 1 : 0;

      nextAlarm = PULSE_PAUSE_LEN;
      state = STATE_SYNC;

      if (PRINTER) { Serial.println("Resync"); }
      break;
  }

  // Serial.println(nextAlarm);
  timerAlarm(timer, nextAlarm, false, 0);
  digitalWrite(DATA_LED, dataPinState);
}

int DebugSlower() {
  return DEBUG ? 1000 : 1;
}

void setup() {
  pinMode(RED_OUTPUT_ENABLE_PIN, INPUT_PULLUP);
  pinMode(GREEN_OUTPUT_SELECT_PIN, INPUT_PULLUP);
  pinMode(DATA_LED, OUTPUT);
  pinMode(SIGNAL_LED, OUTPUT);

  pinMode(EXTRA_GND, OUTPUT);
  digitalWrite(EXTRA_GND, LOW);

  for (int i = 0; i < 3; i++) {
    digitalWrite(DATA_LED, HIGH);
    delay(100);
    digitalWrite(DATA_LED, LOW);
    delay(100);
  }

  digitalWrite(DATA_LED, LOW);
  digitalWrite(SIGNAL_LED, LOW);

  attachInterrupt(GREEN_OUTPUT_SELECT_PIN, ISR_SELECT_Handler, FALLING);
  attachInterrupt(RED_OUTPUT_ENABLE_PIN, ISR_ENABLE_Handler, FALLING);

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, timerISR);

  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("ESP32 serial up");
  // startStateMachine();
}

void IRAM_ATTR ISR_SELECT_Handler() {
  static volatile unsigned long lastTrigger = 0;

  unsigned long now = millis();
  if (now - lastTrigger < DEBOUNCE_CD) { return; }
  lastTrigger = now;

  isStateAlternative = !isStateAlternative;
  if (isEnabled) {
    stopStateMachine();
    startStateMachine();
  }
  Serial.println("SEL");
}

void IRAM_ATTR ISR_ENABLE_Handler() {
  static volatile unsigned long lastTrigger = 0;

  unsigned long now = millis();
  if (now - lastTrigger < DEBOUNCE_CD) { return; }
  lastTrigger = now;

  Serial.println("EN");

  if (isEnabled) {
    stopStateMachine();
  } else {
    startStateMachine();
  }
}

void loop() {
  static long loops = 0;
  static unsigned long timer1000 = 0;

  if (timerTriggered) {
    timerTriggered = false;
    timerWrite(timer, 0);
    stateMachine();
  }

  loops++;
  unsigned long now = micros();
  // if (now - timer1000 >= 1000000) {
  //   timer1000 = now;
  //   Serial.print(loops);
  //   Serial.println("Hz");
  //   loops = 0;
  // }
}

void startStateMachine() {
  noInterrupts();
  isEnabled = true;
  state = STATE_SYNC;
  timerWrite(timer, 0);
  timerStart(timer);
  stateMachine();
  interrupts();
}

void stopStateMachine() {
  noInterrupts();
  isEnabled = false;
  state = STATE_STOPPED;
  stateMachine();
  interrupts();
}
