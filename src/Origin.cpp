// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "Origin.h"

std::string toStr(CHROMOSOME::Origin const &origin) {
  using namespace CHROMOSOME;
  switch (origin) {
  case RANDOM:
    return "random ";
  case HALF:
    return "half   ";
  case COPY:
    return "copy   ";
  case COMBINE:
    return "combine";
  case MUTATE:
    return "mutate ";
  case NONE:
    return "<>     ";
  case COMBMUT:
    return "combmut";
  case COMREPAIR:
    return "combrepair";
  case HEURISITIC:
    return "heuristic";
  default:
    return "X      ";
  }
}