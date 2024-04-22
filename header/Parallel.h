// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_PARALLEL_H
#define SCHEDULER_PARALLEL_H

#if defined(_OPENMP)
#include "global.h"
#include <omp.h>
#endif

void set_parallelism() {
  /// THREADLIMIT
#ifdef _OPENMP
  omp_set_nested(0);
  // limit the number of threads in parallel
  if (params.threads > MAX_THREAD_COUNT) {
    LOG_OUTPUT(LOG_M_ALWAYS, "This program supports at most %d threads!\n",
               MAX_THREAD_COUNT);
    params.threads = MAX_THREAD_COUNT;
  }
  if (params.threads != 0) {
    omp_set_num_threads(params.threads);
  }
#endif
}

#endif // SCHEDULER_PARALLEL_H
