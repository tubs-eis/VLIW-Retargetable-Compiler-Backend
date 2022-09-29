// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
/*
 * sched_ga_stats.h
 *
 * Analysis of genetic scheduling.
 * Compute statistics of chromosomes and resulting scheduling.
 *
 */

#ifndef HEADER_SCHED_GA_STATS_H_
#define HEADER_SCHED_GA_STATS_H_

#include "ga_stats.h"
#include <set>
#include <unordered_map>

namespace gen_sched {

class sched_chromosome;

}

namespace sched_ga_stats {

class ScheduleGAStat : public ga_stats::GAStat {
public:
  void add(const gen_sched::sched_chromosome *chromosome,
           unsigned int chromosomeLength);
};

} // namespace sched_ga_stats

#endif /* HEADER_SCHED_GA_STATS_H_ */
