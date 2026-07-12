#ifndef SAFETY_CUTOFF_H
#define SAFETY_CUTOFF_H

#include <stdint.h>

class SafetyCutoff {
public:
  SafetyCutoff();

  void configure(float angleErrorDegrees, int16_t motorCommand,
                 unsigned long cutoffMillis);
  void reset();
  bool update(float angleErrorDegrees, int16_t motorCommand,
              unsigned long nowMillis);

private:
  float angleErrorDegrees_;
  int16_t motorCommand_;
  unsigned long cutoffMillis_;
  unsigned long unsafeStartMillis_;
  bool unsafeActive_;

  bool unsafe(float angleErrorDegrees, int16_t motorCommand) const;
};

#endif
