// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "sched_ga_stats.h"
#include "MI.h"
#include "gen_sched.h"

namespace sched_ga_stats {

void ScheduleGAStat::add(const gen_sched::sched_chromosome *chromosome,
                         unsigned int chromosomeLength) {
  ga_stats::NumberSequence sched(chromosome->instructions);
  ga_stats::NumberSequence indiv(chromosome->weights, chromosomeLength);
  _data[sched].insert(indiv);
}

} // namespace sched_ga_stats
