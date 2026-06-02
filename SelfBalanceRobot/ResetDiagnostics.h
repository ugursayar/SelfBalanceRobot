#ifndef RESET_DIAGNOSTICS_H
#define RESET_DIAGNOSTICS_H

#include <stdint.h>

namespace ResetDiagnostics {
  constexpr uint8_t PowerOn = 1u << 0;
  constexpr uint8_t External = 1u << 1;
  constexpr uint8_t BrownOut = 1u << 2;
  constexpr uint8_t Watchdog = 1u << 3;
  constexpr uint8_t Jtag = 1u << 4;
  constexpr uint8_t KnownMask = PowerOn | External | BrownOut | Watchdog | Jtag;

  struct ResetCauseFlags {
    bool powerOn;
    bool external;
    bool brownOut;
    bool watchdog;
    bool jtag;
    bool unknown;
  };

  ResetCauseFlags decode(uint8_t rawFlags);
}

#endif
