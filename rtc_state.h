#pragma once
#include <Arduino.h>

// Status yang bertahan lewat deep sleep (RTC memory) + waktu kontinu.
enum BootKind { BOOT_COLD, BOOT_TIMER, BOOT_BUTTON };

bool        rtcInit();                 // panggil paling awal di setup(); true bila state RTC valid (bukan boot dingin)
BootKind    bootKind();
const char* bootName(BootKind k);
uint64_t    uptimeMs();                // waktu sejak boot dingin, termasuk lama tidur
bool        rtcManualOff();            // true bila bangun dari mode mati manual (serial '0'), belum dilanjutkan dengan '1'
void        rtcSetManualOff(bool on);
// sleepMs = 0 -> tanpa timer, hanya tombol yang bisa membangunkan
[[noreturn]] void rtcDeepSleep(uint32_t sleepMs);