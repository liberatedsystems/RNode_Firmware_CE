// Copyright (C) 2026, Emanuele Calo

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef BUZZER_H
  #define BUZZER_H

  #if HAS_BUZZER
    // Build with -DBUZZER_ENABLED=false for a silent firmware.
    #ifndef BUZZER_ENABLED
      #define BUZZER_ENABLED true
    #endif

    void buzzer_init() {
      pinMode(pin_buzzer, OUTPUT);
      noTone(pin_buzzer);
    }

    void buzzer_beep(unsigned int freq, unsigned long duration_ms) {
      if (BUZZER_ENABLED) { tone(pin_buzzer, freq, duration_ms); }
    }

    // 2.7 kHz is the rated resonant frequency of the onboard passive buzzer
    void buzzer_boot_signal()    { buzzer_beep(2700, 60); }
    void buzzer_pairing_signal() { buzzer_beep(2200, 40); }
  #else
    void buzzer_init() {}
    void buzzer_boot_signal() {}
    void buzzer_pairing_signal() {}
  #endif
#endif
