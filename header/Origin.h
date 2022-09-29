// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_ORIGIN_H
#define SCHEDULER_ORIGIN_H

#include <string>

namespace CHROMOSOME {
enum Origin {
  RANDOM,
  HALF,
  COPY,
  COMBINE,
  MUTATE,
  NONE,
  COMBMUT,
  COMREPAIR,
  HEURISITIC
};
}

std::string toStr(CHROMOSOME::Origin const &origin);

#endif // SCHEDULER_ORIGIN_H
