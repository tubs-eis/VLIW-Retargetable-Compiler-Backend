// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHED_STATS_H_
#define SCHED_STATS_H_

#include "gen_sched.h"
#include <fstream>

class SchedulingStats {
private:
  std::ofstream _ostream;

  static SchedulingStats *_instance;
  SchedulingStats(const char *path);

  bool containsGeneratedMV(SLM *slm);

public:
  ~SchedulingStats();

  static void init(const char *path);
  static SchedulingStats *instance();
  static void release();
  static bool hasInstance();

  /**
   * Prints stats about the SLM.
   * @param round
   * @param slm
   */
  void printSchedulingStats(uint round, SLM *slm);
};

#endif /* HEADER_SCHED_STATS_H_ */
