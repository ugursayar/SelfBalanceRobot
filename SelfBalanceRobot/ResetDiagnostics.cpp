#include "ResetDiagnostics.h"

namespace ResetDiagnostics {

ResetCauseFlags decode(uint8_t rawFlags) {
  ResetCauseFlags flags;
  flags.powerOn = (rawFlags & PowerOn) != 0;
  flags.external = (rawFlags & External) != 0;
  flags.brownOut = (rawFlags & BrownOut) != 0;
  flags.watchdog = (rawFlags & Watchdog) != 0;
  flags.jtag = (rawFlags & Jtag) != 0;
  flags.unknown = (rawFlags & ~KnownMask) != 0;
  return flags;
}

}
