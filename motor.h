#pragma once
#include <Arduino.h>

#ifndef MOTOR_PIN
#define MOTOR_PIN 7        // sesuaikan dengan pin driver motor (transistor/MOSFET), aktif HIGH
#endif

const uint8_t       MOTOR_PULSES = 2;     // jumlah getaran
const unsigned long MOTOR_ON_MS  = 400;   // lama tiap getaran
const unsigned long MOTOR_OFF_MS = 300;   // jeda antar getaran

void motorInit();                          // di setup(): pin output LOW, lepas kunci hold dari deep sleep
void motorBuzz(uint8_t pulses = MOTOR_PULSES);   // mulai pola getar (tidak memblok)
void motorTick();                          // panggil tiap loop()
bool motorBusy();                          // true selama pola getar masih berjalan
void motorPrepareSleep();                  // sebelum deep sleep: pin LOW + dikunci