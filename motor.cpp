#include "motor.h"
#include <driver/gpio.h>

static uint8_t       pulsesLeft = 0;
static bool          motorOn    = false;
static unsigned long tChange    = 0;

void motorInit() {
  gpio_hold_dis((gpio_num_t)MOTOR_PIN);    // lepas kunci dari deep sleep sebelumnya
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  pulsesLeft = 0;
  motorOn = false;
}

void motorBuzz(uint8_t pulses) {
  if (pulses == 0) return;
  pulsesLeft = pulses;
  motorOn = true;
  tChange = millis();
  digitalWrite(MOTOR_PIN, HIGH);
}

void motorTick() {
  if (!motorOn && pulsesLeft == 0) return;
  unsigned long now = millis();
  if (motorOn) {
    if (now - tChange >= MOTOR_ON_MS) {
      digitalWrite(MOTOR_PIN, LOW);
      motorOn = false;
      tChange = now;
      if (pulsesLeft > 0) pulsesLeft--;
    }
  } else if (pulsesLeft > 0 && now - tChange >= MOTOR_OFF_MS) {
    motorOn = true;
    tChange = now;
    digitalWrite(MOTOR_PIN, HIGH);
  }
}

bool motorBusy() { return motorOn || pulsesLeft > 0; }

void motorPrepareSleep() {
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  gpio_hold_en((gpio_num_t)MOTOR_PIN);     // tetap LOW selama deep sleep (hold global diaktifkan displayPrepareSleep)
  motorOn = false;
  pulsesLeft = 0;
}