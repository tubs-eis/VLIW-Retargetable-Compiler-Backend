// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

/* Idea:
 * the idea should be to make list of integers, one entry for every virtual
 * register. The Entrys represent the register File which has been choosen to
 * store the virtual registers. Then make e getFitness function, which creates
 * the mapping from virtual to real registers and count the registers which
 * could not be mapped. (maybe make a second list which will then be evaluated.)
 */

#include "gen_register.h"
#include "../powerEstimation/TransitionEnergyEstimator.h"
#include "CompilerContext.h"
#include "processor.h"
#include "register.h"
#include "virtual_reg.h"
#include <ga_stats.h>
#include <unordered_map>
using namespace gen_reg;

#define POPSIZE 40
#define MUTATE (POPSIZE * 5 / 100)
#define COPY (POPSIZE * 5 / 100)
#define CREATE (POPSIZE / 10)
#define COMBINE (POPSIZE * 8 / 10)
#define MODDILEIN (POPSIZE / 2)

int *gen_reg::init(ass_reg_t *blocked, int *freeReg,
                   vector<VirtualRegisterMapping> *map, uint *seed) {
  int size = map->size();
  int *ret = (int *)malloc(sizeof(int) * size);
  int num = registers::getNumRegisterFiles();
  for (int i = 0; i < size; i++) {
    ret[i] = rand_r(seed) % num;
  }
  return ret;
}

ga_stats::ChromosomeFitness
gen_reg::fitness(const ass_reg_t *blocked, const Processor *pro,
                 const Program *ins, VirtualRegisterMap *map,
                 const RegisterCoupling *couplings, const int *freeReg,
                 const Context &ctx) {
  // moved outside
  //  resetVirtualRegisterMap(map, values);

  // transfer free and blocked register State to new datastructure
  ass_reg_t blockedRegs[registers::getNumRegisterFiles()];
  int freeRegs[registers::getNumRegisterFiles()];
  memcpy(blockedRegs, blocked,
         sizeof(ass_reg_t) * registers::getNumRegisterFiles());
  memcpy(freeRegs, freeReg, sizeof(int) * registers::getNumRegisterFiles());

  // register allocation
  int ret = allocate_virtual(blockedRegs, pro, ins, map, couplings, freeRegs,
                             ctx, nullptr);

  auto chromosomeFitness = ga_stats::ChromosomeFitness(ret);
  // if RA successfull, calculate Reg transition Energy for program and map
  if (ret == 0) {
    TransitionEnergyEstimator transitionEnergyEstimator;
    for (uint i = 0; i < ins->size() - 1; i++) {
      transitionEnergyEstimator.push(ins->getMI(i), map);
    }
    chromosomeFitness.setTransitionEnergy(
        transitionEnergyEstimator.getTransitionEnergy());
  }

  return chromosomeFitness;
}

int *gen_reg::combine(int *first, int *second, int size, uint *seed) {
  int *ret = (int *)malloc(sizeof(int) * size);
  int part = 0;
  int i = 0;
  int z = 0;
  int *actual[2];
  actual[0] = first;
  actual[1] = second;
  do {
    part = min(part + rand_r(seed) % (size + 1),
               size); // plus 1 to not have a division by zero is size is zero
                      // (should never happen).
    while (i < part) {
      ret[i] = actual[z][i];
      i++;
    }
    z = (z + 1) % 2;
  } while (part != size);
  return ret;
}

int *gen_reg::mutate(const int *old, int size, uint *seed) {
  int *ret = (int *)malloc(sizeof(int) * size);
  int num = registers::getNumRegisterFiles();
  for (int i = 0; i < size; i++) {
    if (rand_r(seed) % 20 ==
        0) ///////////////////////////// 5% chance of mutation
      ret[i] = rand_r(seed) % num;
    else
      ret[i] = old[i];
  }
  return ret;
}

void gen_reg::insertInOrder(struct reg_gene g,
                            std::vector<struct reg_gene> *populations) {
  bool inserted = false;
  for (auto it = populations->begin(); it != populations->end(); it++) {
    if (g < (*it)) {
      populations->insert(it, g);
      inserted = true;
      break;
    }
  }
  // if it is not bigger, than any element of the list, then add it at the end.
  // also happens if the list is empty
  if (!inserted)
    populations->push_back(g);
}

/** \brief A small helper function to get rid of the population
 *
 * This function just deletes the population, so i don't have to write it
 * multiple times in the code.
 */
void clear_population(std::vector<struct reg_gene> *population) {

  for (auto &it : *population) {
    free(it.genes);
  }
  delete population;
}

bool gen_reg::abortGeneticRA(int fitness) {
  if (!params.powerOptimizationFile.empty()) {
    return false;
  } else {
    return fitness == 0;
  }
}

int gen_reg::genetic(ass_reg_t *blocked, Processor *pro, Program *ins,
                     VirtualRegisterMap *map, RegisterCoupling *couplings,
                     int *freeReg, const Context &ctx) {

  int size = map->size();
  int num_reg = registers::getNumRegisterFiles();

  auto *population = new std::vector<struct reg_gene>();
  uint seed = 500; //(unsigned int)time(nullptr);

  for (int i = 0; i < POPSIZE; i++) {
    struct reg_gene g {};
    g.genes = (int *)malloc(sizeof(int) * size);
    for (int x = 0; x < size; x++) {
      g.genes[x] = rand_r(&seed) % (num_reg);
    }
    resetVirtualRegisterMap(map, g.genes);
    g._chromosomeFitness = fitness(blocked, pro, ins, map, couplings, freeReg,
                                   Context::RegisterAlloc(ctx, 0, i));
    if (abortGeneticRA(g._chromosomeFitness.getFitness())) {
      clear_population(population);
      return 0;
    }
    insertInOrder(g, population);
  }

  int noImprovementRounds = 0;
  auto best = population->front();

  while (!killed && noImprovementRounds++ < 50) {

    std::vector<struct reg_gene> *copy = population;
    population = new std::vector<struct reg_gene>();

    // copy he best.
    for (int x = 0; x < COPY; x++) {
      population->push_back(copy->at(x));
    }

    // combine the genes.
    for (int x = 0; x < COMBINE; x++) {
      int p1 = rand_r(&seed) % MODDILEIN, p2 = rand_r(&seed) % MODDILEIN;
      while (p1 == p2) {
        p2 = rand_r(&seed) % MODDILEIN;
      }
      struct reg_gene next {};
      next.genes = combine(copy->at(p1).genes, copy->at(p2).genes, size, &seed);

      resetVirtualRegisterMap(map, next.genes);
      next._chromosomeFitness =
          fitness(blocked, pro, ins, map, couplings, freeReg,
                  Context::RegisterAlloc(ctx, noImprovementRounds, x));
      if (abortGeneticRA(next._chromosomeFitness.getFitness())) {
        clear_population(population);
        return 0;
      }
      insertInOrder(next, population);
    }

    // mutate existing genes
    for (int x = 0; x < MUTATE; x++) {
      int p1 = rand_r(&seed) % MODDILEIN;
      struct reg_gene next {};
      next.genes = mutate(copy->at(p1).genes, size, &seed);

      resetVirtualRegisterMap(map, next.genes);
      next._chromosomeFitness = fitness(
          blocked, pro, ins, map, couplings, freeReg,
          Context::RegisterAlloc(ctx, noImprovementRounds, COMBINE + x));

      if (abortGeneticRA(next._chromosomeFitness.getFitness())) {
        clear_population(population);
        return 0;
      }
      insertInOrder(next, population);
    }

    // add new Genes.
    for (int x = 0; x < CREATE; x++) {
      struct reg_gene g {};
      g.genes = (int *)malloc(sizeof(int) * size);
      for (int x = 0; x < size; x++) {
        g.genes[x] = rand_r(&seed) % (num_reg);
      }
      resetVirtualRegisterMap(map, g.genes);
      g._chromosomeFitness =
          fitness(blocked, pro, ins, map, couplings, freeReg,
                  Context::RegisterAlloc(ctx, noImprovementRounds,
                                         COMBINE + MUTATE + x));
      if (abortGeneticRA(g._chromosomeFitness.getFitness())) {
        clear_population(population);
        return 0;
      }
      insertInOrder(g, population);
    }
    LOG_OUTPUT(LOG_M_RA_DEBUG,
               "Register, best: %d\tCumulative TransitionPower[pJ]:%f\n",
               best._chromosomeFitness.getFitness(),
               best._chromosomeFitness.transitionEnergy);

    if (population->front() < best) {
      auto population_front_fitness =
          population->front()._chromosomeFitness.getFitness();
      auto population_front_transitionEnergy =
          population->front()._chromosomeFitness.getTransitionEnergy();
      LOG_OUTPUT(
          LOG_M_RA_DEBUG,
          "[genetic][RA] found better Individual: %d\tCumulative "
          "TransitionPower[pJ]:%f -> %d\tCumulative TransitionPower[pJ]:%f  \n",
          best._chromosomeFitness.getFitness(),
          best._chromosomeFitness.transitionEnergy, population_front_fitness,
          population_front_transitionEnergy);
      noImprovementRounds = 0;
      best = population->front();
    }

    for (uint x = COPY, end = copy->size(); x < end; x++) {
      free(copy->at(x).genes);
    }
    delete copy;
  }
  clear_population(population);
  return best._chromosomeFitness.getFitness();
}
