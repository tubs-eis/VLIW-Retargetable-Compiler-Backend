// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_DOUBLEREGISTER_H
#define SCHEDULER_DOUBLEREGISTER_H

#include <cstdlib>
#include <utility>

/**
 * Bit Vector containing blocked register.
 */
#define ass_reg_t long long unsigned int

/** \brief Get a free register pair from a specific register file, where the
 * already blocked registers are given. */
int getDoubleFreeRegisters(const ass_reg_t *blocked, int regFile,
                           bool concurrent);

/** \brief Get a free register pair from a specific register file, where the
 * already blocked registers are given. */
std::pair<int, int> getDoubleFreeRegistersPair(const ass_reg_t *blocked,
                                               int regFile, bool concurrent);

/**
 * \brief Get random free Register Pair for coupled Register Access.
 * @param blocked Blocked register will be immediately blocked! This is
 * different from the other methods!!!
 * @param concurrent
 * @param seed
 * @return
 */
std::pair<int, int> getDoubleFreeRegistersPairRandom(ass_reg_t *blocked,
                                                     bool concurrent,
                                                     uint *seed);

#endif // SCHEDULER_DOUBLEREGISTER_H
