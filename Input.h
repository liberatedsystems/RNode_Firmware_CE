// Copyright (C) 2024, Mark Qvist

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

#ifndef INPUT_H
  #define INPUT_H
  
  #define PIN_BUTTON pin_btn_usr1

  #define PRESSED LOW
  #define RELEASED HIGH

  #define EVENT_ALL                 0x00
  #define EVENT_CLICKS              0x01
  #define EVENT_BUTTON_DOWN         0x11
  #define EVENT_BUTTON_UP           0x12
  #define EVENT_BUTTON_CLICK        0x13
  #define EVENT_BUTTON_DOUBLE_CLICK 0x14
  #define EVENT_BUTTON_TRIPLE_CLICK 0x15
  
  int button_events = EVENT_CLICKS;
  int button_state = RELEASED;
  int debounce_state = button_state;
  unsigned long button_debounce_last = 0;
  unsigned long button_debounce_delay = 25;
  unsigned long button_down_last = 0;
  unsigned long button_up_last = 0;

  // Forward declaration
  void button_event(uint8_t event, unsigned long duration);

  #if HAS_JOYSTICK
    int joy_press_state = RELEASED;
    int joy_press_debounce_state = RELEASED;
    unsigned long joy_press_debounce_last = 0;
    unsigned long joy_press_down_last = 0;
    unsigned long joy_dir_last = 0;
    const unsigned long joy_dir_min_interval = 150;

    // Forward declaration
    void joystick_direction_event();

    void joystick_init() {
      pinMode(pin_joy_press, INPUT_PULLUP);
      pinMode(pin_joy_up, INPUT_PULLUP);
      pinMode(pin_joy_down, INPUT_PULLUP);
      pinMode(pin_joy_left, INPUT_PULLUP);
      pinMode(pin_joy_right, INPUT_PULLUP);
    }

    // Center press feeds the normal button event pipeline, directions
    // only fire joystick_direction_event (rate limited, no debouncer).
    void joystick_read() {
      int reading = digitalRead(pin_joy_press);
      if (reading != joy_press_debounce_state) {
        joy_press_debounce_last = millis();
        joy_press_debounce_state = reading;
      }

      if ((millis() - joy_press_debounce_last) > button_debounce_delay) {
        if (reading != joy_press_state) {
          int previous_state = joy_press_state;
          joy_press_state = reading;
          if (previous_state == PRESSED && joy_press_state == RELEASED) {
            button_event(EVENT_BUTTON_CLICK, millis()-joy_press_down_last);
          } else if (previous_state == RELEASED && joy_press_state == PRESSED) {
            joy_press_down_last = millis();
          }
        }
      }

      if ((millis() - joy_dir_last) > joy_dir_min_interval) {
        if (digitalRead(pin_joy_up) == PRESSED || digitalRead(pin_joy_down) == PRESSED ||
            digitalRead(pin_joy_left) == PRESSED || digitalRead(pin_joy_right) == PRESSED) {
          joy_dir_last = millis();
          joystick_direction_event();
        }
      }
    }
  #endif

  void input_init() {
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    #if HAS_JOYSTICK
      joystick_init();
    #endif
  }

  void input_get_all_events() {
    button_events = EVENT_ALL;
  }

  void input_get_click_events() {
    button_events = EVENT_CLICKS;
  }

  void input_read() {
    #if HAS_JOYSTICK
      joystick_read();
    #endif

    int button_reading = digitalRead(PIN_BUTTON);
    if (button_reading != debounce_state) {
      button_debounce_last = millis();
      debounce_state = button_reading;
    }

    if ((millis() - button_debounce_last) > button_debounce_delay) {
      if (button_reading != button_state) {
        // State changed
        int previous_state = button_state;
        button_state = button_reading;

        if (button_events == EVENT_ALL) {
          if (button_state == PRESSED) {
            button_event(EVENT_BUTTON_DOWN, 0);
          } else if (button_state == RELEASED) {
            button_event(EVENT_BUTTON_UP, 0);
          }
        } else if (button_events == EVENT_CLICKS) {
          if (previous_state == PRESSED && button_state == RELEASED) {
            button_up_last = millis();
            button_event(EVENT_BUTTON_CLICK, button_up_last-button_down_last);
          } else if (previous_state == RELEASED && button_state == PRESSED) {
            button_down_last = millis();
          }
        }
      }
    }

  }

  bool button_pressed() {
    if (button_state == PRESSED) {
      return true;
    } else {
      return false;
    }
  }

#endif