#ifndef POWER_BATCH_H
#define POWER_BATCH_H

/*
  Power-measurement bench (power_batch_test in config.h): runs every board
  configuration back to back, each preceded by a current pattern that
  power_measurement/power_batch_analyze.py finds in the supply log. Tests 1-10 in one
  run, those with the sd-card last; the card is inserted by hand when the bench waits
  for it. Never returns.
*/
void runPowerBatch(void);

#endif  // POWER_BATCH_H
