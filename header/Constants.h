// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_CONSTANTS_H
#define SCHEDULER_CONSTANTS_H

#include <cmath>
#define NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET 10000

#define NON_SCHEDULEABLE_OFFSET 100000

#define DEFAULT_REGISTER_HEURISTIC 32676

#define PORT_OPTIMAL_FITNESS_OFFSET 1000

#define PORT_OPTIMAL_CONFLICT_OFFSET 100

#define PORT_OPTIMAL_FIRST_OFFSET 10000

#define MAX_REGISTER 64
namespace CONSTANT {
extern bool minPower;
extern float powerEqualThreshold;
} // namespace CONSTANT

// do not have linking errors with multiple definitions
/**
 * Private Function, should not be called directly.
 * Please call compareTransitionEnergy
 * @param val1
 * @param val2
 * @return
 */
static bool betterPowerTarget(const double &val1, const double &val2) {
  if (CONSTANT::minPower) {
    return val1 < val2;
  } else {
    return val1 > val2;
  }
}
/***
 *
 * @param val1
 * @param val2
 * @return True if val1 is better according to power optimization target
 */
static bool compareTransitionEnergy(const double &val1, const double &val2,
                                    const int &transition1,
                                    const int &transition2) {
  // to maximize energy consumption, swap operators
  if (val1 < 0) {
    return false;
  }
  if (val2 < 0) {
    return true;
  }
  // to minimize energy consumption: val1 < val2
  // to maximize energy consumption, val1 > val2
  bool val1Better = betterPowerTarget(val1, val2);
  if (transition1 != -1 and transition2 != -1) {
    double denominator = val2;
    if (val1Better) {
      denominator = val1;
    }

    int percentDiff = ((fabs(val1 - val2) / denominator) * 100);
    if (percentDiff < CONSTANT::powerEqualThreshold) {
      if (transition1 != transition2) {
        if (CONSTANT::minPower) {
          return transition1 < transition2;
        } else {
          return transition1 > transition2;
        }
      }
    }
  }
  return betterPowerTarget(val1, val2);
}

/***
 *
 * @param val1
 * @param val2
 * @return True if val1 is better according to power optimization target
 */
static bool compareTransitionEnergy(const double &val1, const double &val2) {
  // to maximize energy consumption, swap operators
  if (val1 < 0) {
    return false;
  }
  if (val2 < 0) {
    return true;
  }

  return betterPowerTarget(val1, val2);
}

extern bool enteredGeneticRA;

#define numberResolution 5

#define DEBUG_TEXT false

#endif // SCHEDULER_CONSTANTS_H
