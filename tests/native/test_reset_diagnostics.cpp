#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/ResetDiagnostics.h"

static void test_decodes_no_reset_flags() {
  const ResetDiagnostics::ResetCauseFlags flags = ResetDiagnostics::decode(0);

  assert(!flags.powerOn);
  assert(!flags.external);
  assert(!flags.brownOut);
  assert(!flags.watchdog);
  assert(!flags.jtag);
  assert(!flags.unknown);
}

static void test_decodes_known_reset_flags() {
  const ResetDiagnostics::ResetCauseFlags flags =
      ResetDiagnostics::decode(ResetDiagnostics::PowerOn |
                               ResetDiagnostics::BrownOut |
                               ResetDiagnostics::Watchdog);

  assert(flags.powerOn);
  assert(!flags.external);
  assert(flags.brownOut);
  assert(flags.watchdog);
  assert(!flags.jtag);
  assert(!flags.unknown);
}

static void test_reports_unknown_reset_flags() {
  const ResetDiagnostics::ResetCauseFlags flags =
      ResetDiagnostics::decode(0x80);

  assert(!flags.powerOn);
  assert(!flags.external);
  assert(!flags.brownOut);
  assert(!flags.watchdog);
  assert(!flags.jtag);
  assert(flags.unknown);
}

int main() {
  test_decodes_no_reset_flags();
  test_decodes_known_reset_flags();
  test_reports_unknown_reset_flags();

  std::cout << "test_reset_diagnostics PASS\n";
  return EXIT_SUCCESS;
}
