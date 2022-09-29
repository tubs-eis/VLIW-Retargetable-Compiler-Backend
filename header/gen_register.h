// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef HEADER_GEN_REGISTER_H_
#define HEADER_GEN_REGISTER_H_

#include "ga_stats.h"
#include "register.h"
#include "virtual_reg.h"

class Context;

namespace gen_reg {

struct reg_gene {
  int *genes;
  /**
   * Number of not allocatable Register
   */
  ga_stats::ChromosomeFitness _chromosomeFitness;

  bool operator<(const reg_gene &other) {
    //    return _chromosomeFitness < other._chromosomeFitness;
    if (_chromosomeFitness.getFitness() <
        other._chromosomeFitness.getFitness()) {
      return true;
    } else if (_chromosomeFitness.getFitness() == 0 and
               other._chromosomeFitness.getFitness() == 0) {
      // to maximize energy consumption, swap operators
      return _chromosomeFitness.transitionEnergy <
             other._chromosomeFitness.transitionEnergy;
    } else {
      return false;
    }
  }
};

int *init(ass_reg_t *blocked, int *freeReg,
          std::vector<VirtualRegisterMapping> *map, uint *seed);
int *combine(int *first, int *second, int size, uint *seed);
int *mutate(const int *old, int size, uint *seed);
ga_stats::ChromosomeFitness fitness(const ass_reg_t *blocked,
                                    const Processor *pro, const Program *ins,
                                    VirtualRegisterMap *map,
                                    const RegisterCoupling *couplings,
                                    const int *freeReg, const Context &ctx);
void insertInOrder(struct reg_gene g,
                   std::vector<struct reg_gene> *populations);

/**
 * Genetic Algorithm to search for a good register allocation.
 * Population size is hard coded in the Source File.
 * @param blocked
 * @param pro
 * @param ins
 * @param map
 * @param couplings
 * @param freeReg
 * @param ctx
 * @return The number of non allocatable Register
 */
int genetic(ass_reg_t *blocked, Processor *pro, Program *ins,
            VirtualRegisterMap *map, RegisterCoupling *couplings, int *freeReg,
            const Context &ctx);

bool abortGeneticRA(int fitness);
} // namespace gen_reg

#endif /* HEADER_GEN_REGISTER_H_ */
