// MeshPocket USB-serial debug helpers (115200)
#pragma once

#if defined(BOARD_MODEL) && (BOARD_MODEL == BOARD_HELTEC_MESHP) && defined(MESHP_DEBUG) && MESHP_DEBUG
  #define MP_DBG(...) do { Serial.printf(__VA_ARGS__); Serial.flush(); } while (0)
#else
  #define MP_DBG(...) do {} while (0)
#endif
