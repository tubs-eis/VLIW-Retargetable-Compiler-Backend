// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "../export/ScheduledMOCharacteristics.h"
#include "../powerEstimation/Estimator.h"
#include "../powerEstimation/TransitionEnergyEstimator.h"
#include "CompilerContext.h"
#include "MI.h"
#include "SLM.h"
#include "gen_sched.h"
#include "processor.h"
#include "sched_stats.h"
#include "utility.h"
#include "virtual_reg.h"
#include <algorithm>
#include <string>
#if SCHED_DUP_CHECK
#include "ga_stats.h"
#endif
using namespace gen_sched;

#if defined(_OPENMP)
#include <omp.h>
#include <opts.h>
#else
#define omp_lock_t int
#endif

#if CHECK_RA_TIMING
RA_Timings ra_timings;
#endif

int gen_sched::heuristicRACount, gen_sched::geneticRACount,
    gen_sched::failedRACount;

std::string SchedGeneOrigin::toString(Origin origin) {
  switch (origin) {
  case RANDOM:
    return "randomPopulation";
  case CROSSOVER:
    return "crossover";
  case MUTATE:
    return "mutate";
  case COPY:
    return "copy";
  case WEIGHTS:
    return "weight";
  default:
    return "<unknown>";
  }
}

// liste erstellen mit int werten.
// liste geht von ID der ersten MO bis ID der letzten MO.
// mit randomPopulation integer werten initialisieren.
// je hoeher, desto eher wird scheduliert.
// wenn 1, dann wird erst scheduliert, wenn alles andere bereits gescheduled.

int count_used_dummy_registers(Program *ins, VirtualRegisterMap *map) {
  int dummy_count = 0;
  for (auto mi : *ins) {
    unsigned int x = 0;
    while (x < MI::getNumberIssueSlots()) {
      MO *mo = mi->getOperations()[x];
      if (!mo) {
        ++x;
        continue;
      }

      char32_t *args = mo->getArguments();
      OPtype *types = mo->getTypes();
      for (int e = 0; e < mo->getArgNumber(); ++e) {
        if (types[e] == REG) {
          int regNumber = registers::getVirtualRegisterNumber(args[e]);
          if (regNumber < 0)
            continue;
          VirtualRegisterMapping *m = getMapping(map, args[e]);
          if (!m)
            continue;
          if (registers::isDummyRegister(m->getReal()))
            ++dummy_count;
        }
      }
      x += mo->opLengthMultiplier();
    }
  }
  return dummy_count;
}

Program *gen_sched::scheduleSLM(sched_chromosome *individual, SLM *slm,
                                Processor *processor, int &notScheduableCSCR,
                                const Context &ctx) {
  ListSearch *alg;
  if (individual) {
    LOG_OUTPUT(
        LOG_M_SCHED_DETAIL,
        "%s Starting scheduling routine for given individual for SLM %d\n",
        ctx.asString().c_str(), slm->getOriginalID());
    alg = new ListSearch(slm, processor, individual);
  } else {
    LOG_OUTPUT(
        LOG_M_SCHED_DETAIL,
        "%s Starting scheduling routine with heuristic weight for SLM %d\n",
        ctx.asString().c_str(), slm->getOriginalID());
    alg = new ListSearch(slm, processor);
  }

  Program *instructions = alg->scheduleMOs(processor, notScheduableCSCR);
  if (instructions) {
    if (slm->getBranchOperation()) {
      // alg->insertBranch(processor, *instructions, slm->getBranchOperation());
      alg->scheduleBranch(slm->getBranchOperation(), processor, instructions);
    }
    alg->fillLatencyCycles(instructions);
    LOG_OUTPUT(
        LOG_M_SCHED_DETAIL,
        "%s Scheduling routine ended with instruction size %lu for SLM %d\n",
        ctx.asString().c_str(), instructions->size(), slm->getOriginalID());
  }
  delete alg;
  return instructions;
}

/***
 *  Calculates Couplings on the map and returns if there are (free) virtual
 * register.
 * @param slm
 * @param pro
 * @param ins
 * @param map
 * @param couplings
 * @param ctx
 * @return True if there are free virtual register for allocation.
 */
bool gen_sched::prepareRegisterAllocation(SLM *slm, Processor *pro,
                                          Program *ins,
                                          VirtualRegisterMap **map,
                                          RegisterCoupling **couplings,
                                          const Context &ctx) {
  *map = new VirtualRegisterMap();
  *couplings = new RegisterCoupling();
  calculateCouplings(ins, *map, slm->getFreeReg(), **couplings, pro);
  if ((*map)->size() == 0) {
    deleteCouplings(**couplings);
    delete *couplings;
    *couplings = nullptr;
    LOG_OUTPUT(LOG_M_SCHED, "%s SLM %d contains no virtual registers\n",
               ctx.asString().c_str(), slm->getOriginalID());
    return false;
  }
  return true;
}

void gen_sched::registerAllocation(SLM *slm, Processor *processor,
                                   Program *instructions,
                                   VirtualRegisterMap **map,
                                   RegisterCoupling **couplings,
                                   int &failedRegs, const Context &ctx) {
  bool hasRegs =
      prepareRegisterAllocation(slm, processor, instructions, map, couplings,
                                Context::nextStage(ctx, "reg_prep"));
  if (hasRegs)
    failedRegs = virtual_test(slm, processor, instructions, map, *couplings,
                              Context::nextStage(ctx, "reg"));
  if (*couplings) {
    deleteCouplings(**couplings);
    delete *couplings;
    *couplings = nullptr;
  }
  // add energy transition estimation
  if (successRA(failedRegs)) {
    if (!params.powerOptimizationFile.empty()) {
      if (instructions->size() > 0) {
        LOG_OUTPUT(LOG_M_RA, "-------------- heuristic RA Energy estimation "
                             "------------------\n");
        TransitionEnergyEstimator transitionEnergyEstimator;
        for (uint i = 0; i < instructions->size(); i++) {
          //          std::cout << instructions->getMI(i)->to_string(*map) <<
          //          "\n";
          transitionEnergyEstimator.push(instructions->getMI(i), *map);
        }
        //      chrm->setTransitionEnergy();
        LOG_OUTPUT(LOG_M_RA, "Energy Estimation is: %d \n",
                   transitionEnergyEstimator.getTransitionEnergy());

        instructions->setCumulativeTransitionEnergy(
            transitionEnergyEstimator.getTransitionEnergy());
      }
    }
  }
}

sched_chromosome *gen_sched::mutate(sched_chromosome *pop, int size, uint *seed,
                                    int maxweight, float partProb,
                                    float aloneProb) {
  auto *mut = new sched_chromosome();
  mut->origin = pop->origin;
  mut->setFitness(pop->getFitness());
  mut->p1Fit = pop->p1Fit;
  mut->p2Fit = pop->p2Fit;
  mut->weights = (int *)malloc(sizeof(int) * size);
  mut->partings = (bool *)malloc(sizeof(bool) * size);
  mut->alone = (bool *)malloc(sizeof(bool) * size);
  for (int i = 0; i < size; i++) {
    if (rand_r(seed) % 1000 < 8) {
      mut->weights[i] = rand_r(seed) % (maxweight + 1);
      mut->origin = SchedGeneOrigin::MUTATE;
    } else
      mut->weights[i] = pop->weights[i];
    if (rand_r(seed) % 100 < 1)
      mut->partings[i] = util::uniRandom(seed) < partProb;
    else
      mut->partings[i] = pop->partings[i];
    if (rand_r(seed) % 100 < 1)
      mut->alone[i] = util::uniRandom(seed) < aloneProb;
    else
      mut->alone[i] = pop->alone[i];
  }
  return mut;
}

sched_chromosome *gen_sched::tournamentSelect(vector<sched_chromosome *> *pop,
                                              int tournamentSize, uint *seed) {
  int popsize = pop->size();
  int bestIndex = util::uniRandom(0, popsize - 1, seed);

  for (int i = 0; i < tournamentSize - 1; ++i) {
    int index =
        util::uniRandom(0, popsize - 1, seed); // rand_r(seed) % popsize;
    if (index < bestIndex)
      bestIndex = index;
  }

  return pop->at(bestIndex);
}

#define FIRSTCOMBINE 1

sched_chromosome *gen_sched::combine(sched_chromosome *first,
                                     sched_chromosome *second, int size,
                                     uint *seed) {
  auto *pop = new sched_chromosome();
  pop->weights = (int *)malloc(sizeof(int) * size);
  pop->partings = (bool *)malloc(sizeof(bool) * size);
  pop->alone = (bool *)malloc(sizeof(bool) * size);
#ifdef FIRSTCOMBINE
  for (int i = 0; i < size; i++) {
    if (rand_r(seed) % 2) {
      pop->weights[i] = first->weights[i];
      pop->partings[i] = first->partings[i];
      pop->alone[i] = first->alone[i];
    } else {
      pop->weights[i] = second->weights[i];
      pop->partings[i] = second->partings[i];
      pop->alone[i] = second->alone[i];
    }
  }
#else
  int part = 0;
  int i = 0;
  int z = 0;
  sched_chromosome *actual[2];
  actual[0] = first;
  actual[1] = second;
  do {
    part = min(part + rand_r(seed) % (size + 1),
               size); // plus 1 to not have a division by zero is size is zero
                      // (should never happen).
    while (i < part) {
      pop->weight[i] = actual[z]->weight[i];
      pop->parting[i] = actual[z]->parting[i];
      pop->alone[i] = actual[z]->alone[i];
      i++;
    }
    z = (z + 1) % 2;
  } while (part != size);
#endif
  pop->origin = SchedGeneOrigin::CROSSOVER;
  pop->p1Fit = first->getFitness();
  pop->p2Fit = second->getFitness();
  return pop;
}

sched_chromosome *gen_sched::initialise(int chromosomeLength, int maxweight,
                                        float partProb, float aloneProb,
                                        uint *seed) {
  auto *pop = new sched_chromosome();
  pop->weights = (int *)malloc(sizeof(int) * chromosomeLength);
  pop->partings = (bool *)malloc(sizeof(bool) * chromosomeLength);
  pop->alone = (bool *)malloc(sizeof(bool) * chromosomeLength);
  for (int i = 0; i < chromosomeLength; i++) {
    pop->weights[i] = rand_r(seed) % (maxweight + 1);
    pop->partings[i] = util::uniRandom(seed) < partProb;
    pop->alone[i] = util::uniRandom(seed) < aloneProb;
  }
  pop->origin = SchedGeneOrigin::RANDOM;
  return pop;
}

#define WEIGHT_FACTOR 8

sched_chromosome *gen_sched::initialise_from_weights(int size, uint *seed,
                                                     int partValue, SLM *slm,
                                                     int maxweight) {
  auto *individual = new sched_chromosome();
  individual->weights = (int *)malloc(sizeof(int) * size);
  individual->partings = (bool *)malloc(sizeof(bool) * size);
  individual->alone = (bool *)malloc(sizeof(bool) * size);
  vector<MO *> *ops = slm->getOperations();
  if (ops->size() != (size_t)size) {
    throw "initialization Size != ops.size();\n";
  }
  for (int i = 0; i < size; i++) {
    individual->weights[i] =
        (maxweight == -1)
            ? WEIGHT_FACTOR * ops->at(i)->getWeight()
            : maxweight - WEIGHT_FACTOR * ops->at(i)->getWeight() + 1;
    individual->partings[i] = false;
    individual->alone[i] = false;
  }
  individual->origin = SchedGeneOrigin::WEIGHTS;
  return individual;
}

int getSchedPopulationSize(SLM *slm) {
  int popsize;
  if (slm->getOptimizationLevel() != -1 || params.schedPopulation == -1) {
    //        popsize = std::max(25lu, slm->getOperations()->size() / 5);
    popsize = slm->getOperations()->size();
    popsize *= slm->getBooster();
    int optLvl = slm->getOptimizationLevel();
    optLvl = (optLvl == -1) ? params.optimization : optLvl;
    for (int o = 2; o < optLvl; ++o)
      popsize *= 2;
  } else {
    popsize = params.schedPopulation;
  }
  return std::max(popsize / 5, 10);
}

/**
 *
 * @param chrm Delete and free memory of sched_chromosome
 */
void delete_sched_chromosome(sched_chromosome **chrm) {
  sched_chromosome *c = *chrm;
  free(c->weights);
  free(c->partings);
  free(c->alone);
  if (c->instructions)
    delete c->instructions;
  c->releaseVirtualMapping();
  c->releaseCouplings();
  delete c;
  *chrm = nullptr;
}

int get_max_weight(sched_chromosome *gene, uint size) {
  int maxWeight = 0;
  for (uint i = 0; i < size; ++i)
    if (gene->weights[i] > maxWeight)
      maxWeight = gene->weights[i];
  return maxWeight;
}

void gen_sched::printPopulation(std::vector<sched_chromosome *> *population,
                                int generation, int noImprovement, int opcount,
                                int slm_id, int minsize,
                                ga_stats::ChromosomeFitness lastBestSize,
                                ga_stats::ChromosomeFitness lastBestFit,
                                int skip_size, const Context &ctx) {
  if (isPowerOptimization()) {
    LOG_OUTPUT(LOG_M_ALWAYS,
               "%s Generation Title %2d population size %lu (noImprovement %d, "
               "best size: %d, "
               "best fit.: %d + %f pJ, ra_skip: %d)\n",
               ctx.asString().c_str(), generation, population->size(),
               noImprovement, lastBestSize.getFitness(),
               lastBestFit.getFitness(), lastBestFit.getTransitionEnergy(),
               skip_size);
  } else {
    LOG_OUTPUT(LOG_M_ALWAYS,
               "%s Generation Title %2d population size %lu (noImprovement %d, "
               "best size: %d, "
               "best fit.: %d, ra_skip: %d)\n",
               ctx.asString().c_str(), generation, population->size(),
               noImprovement, lastBestSize.getFitness(),
               lastBestFit.getFitness(), skip_size);
  }
  for (uint n = 0; n < population->size(); ++n) {
    sched_chromosome *chrm = population->at(n);
    if (isPowerOptimization()) {
      if (isPowerOptimization()) {
        if (params.instructionModelFile == "") {
          LOG_OUTPUT(LOG_M_ALWAYS,
                     "%s generation %2d SLM %3d indiv %3d "
                     "fit/bestFitness/pJ/InstrTrans "
                     "%7d/%4d/%4.4f/%d "
                     "size %2d origin %-9s",
                     ctx.asString().c_str(), generation, slm_id, n,
                     chrm->getFitness().getFitness(), minsize,
                     chrm->getTransitionEnergy(),
                     chrm->getInstructionTransitions(), chrm->sched_size,
                     SchedGeneOrigin::toString(chrm->origin).c_str());
        } else {
          // instruction model used
          LOG_OUTPUT(LOG_M_ALWAYS,
                     "%s generation %2d SLM %3d indiv %3d "
                     "fit/bestFitness/pJ/InstrTrans/InstrPower "
                     "%7d/%4d/%4.4f/%d/%4.4f "
                     "size %2d origin %-9s",
                     ctx.asString().c_str(), generation, slm_id, n,
                     chrm->getFitness().getFitness(), minsize,
                     chrm->getTransitionEnergy(),
                     chrm->getInstructionTransitions(),
                     InstructionModel::getInstance().generateInstructionEnergy(
                         chrm->instructions),
                     chrm->sched_size,
                     SchedGeneOrigin::toString(chrm->origin).c_str());
        }
      } else { // should be redundant
        LOG_OUTPUT(LOG_M_ALWAYS,
                   "%s generation %2d SLM %3d indiv %3d fit/bestFitness "
                   "%7d/%4d/%4.4f "
                   "size %2d origin %-9s",
                   ctx.asString().c_str(), generation, slm_id, n,
                   chrm->getFitness().getFitness(), minsize, chrm->sched_size,
                   SchedGeneOrigin::toString(chrm->origin).c_str());
      }

    } else {
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "%s generation %2d SLM %3d indiv %3d fit/bestFitness %7d/%-4d "
                 "size %2d origin %-9s",
                 ctx.asString().c_str(), generation, slm_id, n,
                 chrm->getFitness().getFitness(), minsize, chrm->sched_size,
                 SchedGeneOrigin::toString(chrm->origin).c_str());
    }
    if (chrm->origin == SchedGeneOrigin::CROSSOVER ||
        chrm->origin == SchedGeneOrigin::MUTATE) {
      LOG_OUTPUT(LOG_M_ALWAYS, " (%7d + %7d)", chrm->p1Fit.getFitness(),
                 chrm->p2Fit.getFitness());
    } else
      LOG_OUTPUT(LOG_M_ALWAYS, "                    ");
    if (chrm->ra_skipped) {
      LOG_OUTPUT(LOG_M_ALWAYS, " Reg Allocation: skipped              ");
    } else {
      if (isPowerOptimization()) {
        if (chrm->getTransitionEnergy() > 0) {
          LOG_OUTPUT(LOG_M_ALWAYS, " Reg Allocation: Energy = %3f pJ",
                     chrm->getTransitionEnergy());
        } else {
          LOG_OUTPUT(LOG_M_ALWAYS,
                     " Reg Allocation: Heuristic = %3d Genetic = %9d",
                     chrm->getHeuristicRegister(), // heuristicRegister,
                     chrm->getGeneticRegister());
        }
      } else {
        LOG_OUTPUT(LOG_M_ALWAYS,
                   " Reg Allocation: Heuristic = %3d Genetic = %9d",
                   chrm->getHeuristicRegister(), // heuristicRegister,
                   chrm->getGeneticRegister());  // geneticRegister);
      }
    }
    LOG_OUTPUT(LOG_M_ALWAYS, " Sched-Hash: %s", chrm->getHash(opcount).c_str());

    if (chrm->failedHeuristicRA()) {
      if (chrm->failedGeneticRA())
        failedRACount++;
      else
        geneticRACount++;
    } else
      heuristicRACount++;

#if SCHED_DUP_CHECK
    LOG_OUTPUT(LOG_M_ALWAYS, " Dup: %d%d", chrm->duplicateWeights ? 1 : 0,
               chrm->duplicateSched ? 1 : 0);
#endif
    LOG_OUTPUT(LOG_M_ALWAYS, "\n");
  }
}

#if STORE_PARTINGS_AND_ALONE
void storePartingsAndAlone(SLM *slm, sched_chromosome *indiv) {
  vector<MO *> *ops = slm->getOperations();
  uint size = ops->size();
  bool *parting = indiv->parting;
  bool *alone = indiv->alone;
  for (uint i = 0; i < size; ++i) {
    MO *mo = ops->at(i);
    mo->wasParting = parting[i];
    mo->wasAlone = alone[i];
  }
}
#endif

void gen_sched::scheduleWithHeuristicReg(sched_chromosome *chrm, SLM *slm,
                                         Processor *pro, bool enableRASkip,
                                         int last_best,
                                         const Context &sched_ctx,
                                         bool overWriteSLMAndDeleteCHRM) {
  // todo: why am I not initialized?
  int notScheduableCSCR;
  chrm->instructions =
      scheduleSLM(chrm, slm, pro, notScheduableCSCR, sched_ctx);
  int fitn = 0;
  if (chrm->instructions) {
    fitn = chrm->instructions->size();

    chrm->sched_size = fitn;
    chrm->setHeuristicRegister(
        DEFAULT_REGISTER_HEURISTIC); // heuristicRegister = ;
    chrm->setFitness(fitn);
    chrm->ra_skipped = false;

    chrm->map = nullptr;
    if (!enableRASkip || last_best == -1 ||
        fitn <= last_best + params.RASkipOffset) {
      bool hasRegs = prepareRegisterAllocation(
          slm, pro, chrm->instructions, &chrm->map, &chrm->couplings,
          Context::nextStage(sched_ctx, "reg_prep"));
      if (hasRegs) {
        LOG_OUTPUT(
            LOG_M_SCHED_DETAIL,
            "%s Starting register allocation for SLM %d (getFitness: %d)\n",
            sched_ctx.asString().c_str(), slm->getOriginalID(),
            chrm->getFitness().getFitness());
        int ret = DEFAULT_REGISTER_HEURISTIC;
        if (params.heuristicReg) {
          ret = heuristicRegisterAllocation(
              slm, pro, chrm->instructions, chrm->couplings, chrm->map,
              &chrm->rdg, Context::nextStage(sched_ctx, "heur_reg"));
          if (successRA(ret)) {
            // successfull heuristic RA. Set chromosome as working SLM.
            //            auto powerConsumption =
            //            chrm->calculatePowerConsumption();
            chrm->dummy_count =
                count_used_dummy_registers(chrm->instructions, chrm->map);
            if (overWriteSLMAndDeleteCHRM) {
              slm->setInstructions(&chrm->instructions, &chrm->map);
            }
          }
        }
        chrm->setHeuristicRegister(ret); // heuristicRegister = ret;
        if (!successRA(ret)) {
          chrm->map = nullptr;
          chrm->incrementFitness(NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET * ret);
        }
      } else {
        chrm->ra_skipped = true;
        if (overWriteSLMAndDeleteCHRM) {
          slm->setInstructions(&chrm->instructions, &chrm->map);
        }
      }
    } else {
      LOG_OUTPUT(
          LOG_M_SCHED,
          "%s Skipping register allocation in SLM %d (getFitness %d > best "
          "getFitness %d + %d)\n",
          sched_ctx.asString().c_str(), slm->getOriginalID(), fitn, last_best,
          params.RASkipOffset);
      chrm->incrementFitness(params.RAsmallerPenalty);
      chrm->setHeuristicRegister(
          NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET); // heuristicRegister =
                                                  // NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET;
      chrm->setGeneticRegister(
          NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET); // geneticRegister =
                                                  // NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET;
      chrm->ra_skipped = true;
    }
  } else {
    fitn = NON_SCHEDULEABLE_OFFSET + notScheduableCSCR;
    LOG_OUTPUT(LOG_M_SCHED, "%s Scheduling of SLM %d failed\n",
               sched_ctx.asString().c_str(), slm->getOriginalID());
    chrm->map = nullptr;
    chrm->sched_size = fitn;
    chrm->setFitness(fitn);
    chrm->ra_skipped = true;
  }
}

void gen_sched::performGeneticRegisterAllocation(sched_chromosome *chrm,
                                                 SLM *slm, Processor *pro,
                                                 const Context &sched_ctx) {
  int ret = chrm->getHeuristicRegister();       // heuristicRegister;
  if (!chrm->ra_skipped && (!successRA(ret))) { // chrm->heuristicSchedFailed()
    ret =
        geneticRegisterAllocation(slm, pro, chrm->instructions, chrm->couplings,
                                  chrm->map, &chrm->rdg, sched_ctx);
    if (!params.powerOptimizationFile.empty()) {
      if (successRA(ret)) {
        LOG_OUTPUT(LOG_M_SCHED_DETAIL,
                   "-------------- Found GA RA ------------------\n");
        TransitionEnergyEstimator transitionEnergyEstimator;
        for (uint i = 0; i < chrm->instructions->size() - 1; i++) {
          transitionEnergyEstimator.push(chrm->instructions->getMI(i),
                                         chrm->map);
        }
        LOG_OUTPUT(LOG_M_SCHED_DETAIL, "Energy Estimation is: %f \n",
                   transitionEnergyEstimator.getTransitionEnergy());
        chrm->instructions->setCumulativeTransitionEnergy(
            transitionEnergyEstimator.getTransitionEnergy());
      }
    }
    chrm->setGeneticRegister(ret); // geneticRegister = ret;
    if (successRA(ret)) {
      slm->setInstructions(&chrm->instructions, &chrm->map);
      chrm->setFitness(chrm->sched_size);
    }
  }
  LOG_OUTPUT(
      LOG_M_SCHED,
      "%s Genetic scheduling resulting getFitness value for SLM %d: %d\n",
      sched_ctx.asString().c_str(), slm->getOriginalID(),
      chrm->getFitness().getFitness());
}

int getRASkipIndex(std::vector<sched_chromosome *> *pop, int COPY) {
  int skip_index = COPY - 1;
  sched_chromosome *skip_chrm = pop->at(skip_index);
  while (skip_index > 0 && skip_chrm->failedHeuristicRA() &&
         skip_chrm->failedGeneticRA()) {
    skip_index--;
    skip_chrm = pop->at(skip_index);
  }
  if (skip_chrm->failedHeuristicRA() && skip_chrm->failedGeneticRA())
    return COPY - 1;
  else
    return skip_index;
}

template <class T> T getParameterPreferSLM(T slmPar, T consolePar) {
  return (slmPar == -1) ? consolePar : slmPar;
}

bool gen_sched::abortCondition(int maxRounds, int &noImprovement, bool killed,
                               ga_stats::ChromosomeFitness last_best_fitness,
                               ga_stats::ChromosomeFitness best_possible) {
  if (isPowerOptimization()) {
    return (++noImprovement < maxRounds && !killed);
  } else {
    return (++noImprovement < maxRounds && !killed) &&
           last_best_fitness.getFitness() > best_possible.getFitness();
  }
}

int gen_sched::legacy_genetic_scheduling(SLM *slm, Processor *pro,
                                         const Context &ctx) {
  slm->calculateGraph(pro, true);
  LOG_OUTPUT(isLog(LOG_M_SCHED) || params.printSchedPopulation,
             "%s Starting genetic scheduling for SLM %d (op count: %lu, "
             "minimal size: %d, critical path: %d)\n",
             ctx.asString().c_str(), slm->getOriginalID(),
             slm->getOperations()->size(), slm->getMinimalSize(),
             slm->getCriticalPath());

#if CHECK_RA_TIMING
  ra_timings.reset();
#endif

#if SCHED_DUP_CHECK
  ga_stats::ChromosomesSet sched_chromosomes;
  ga_stats::ChromosomesSet sched_schedules;
  uint64_t processed_chromosomes = 0;
  uint64_t duplicate_chromosomes = 0;
  uint64_t duplicate_schedules = 0;
#endif
  uint seed = (unsigned int)time(0);
  ;
  float partProb = getParameterPreferSLM(slm->getPartProb(), params.partProb);

  std::vector<MO *> *ops = slm->getOperations();
  uint size = ops->size();
  uint number = size;
  if (number == 0) {
    if (params.sched_stats) {
      SchedulingStats::instance()->printSchedulingStats(0, slm);
    }
    sched_chromosome *individual =
        initialise_from_weights(number, &seed, partProb, slm);
    scheduleWithHeuristicReg(individual, slm, pro, params.enableRASkip, -1,
                             Context::Schedule(ctx, 0, 0));
    if (individual->getHeuristicRegister() != 0)
      performGeneticRegisterAllocation(individual, slm, pro,
                                       Context::Schedule(ctx, 0, 0));
    slm->collectInstructionsFromThreads();

    LOG_OUTPUT(
        LOG_M_SCHED, "%s SLM %d contains no schedulable MOs. Final size = %d\n",
        ctx.asString().c_str(), slm->getOriginalID(), individual->sched_size);
    delete_sched_chromosome(&individual);
    return 0;
  }

  int popsize = getSchedPopulationSize(slm);
  int COPY = params.schedEliteCount;
  int CREATE = max(3, (int)(popsize * params.schedRandomRatio));
  int COMBINE = popsize - COPY - CREATE;
  LOG_OUTPUT(LOG_M_SCHED_DETAIL,
             "%s SLM %d genetic scheduling population size: %d\n",
             ctx.asString().c_str(), slm->getOriginalID(), popsize);

  std::vector<sched_chromosome *> *population =
      new std::vector<sched_chromosome *>();
  population->resize(popsize);
  int maxweight = number;
  float aloneProb =
      getParameterPreferSLM(slm->getAloneProb(), params.aloneProb);
  int startIndex = 1;
  /* First individual is different... */
  if (params.includeDDGWeights) {
    sched_chromosome *individual =
        initialise_from_weights(number, &seed, partProb, slm);
    maxweight = get_max_weight(individual, number);
    scheduleWithHeuristicReg(individual, slm, pro, params.enableRASkip, -1,
                             Context::Schedule(ctx, 0, 0));
    population->at(0) = individual;

    //        individual = initialise_from_weights(number, seed, partProb, slm,
    //        maxweight); scheduleWithHeuristicReg(individual, slm, pro,
    //        params.enableRASkip, -1, Context::Schedule(ctx, 0, 1));
    //        population->at(1) = individual;
    //        startIndex = 2;
  } else {
    sched_chromosome *individual = initialise(number, maxweight, 0, 0, &seed);
    scheduleWithHeuristicReg(individual, slm, pro, params.enableRASkip, -1,
                             Context::Schedule(ctx, 0, 0));
    population->at(0) = individual;
  }
#pragma omp parallel for
  for (int i = startIndex; i < popsize; i++) {
    sched_chromosome *individual;
    float pprob = ((float)i / (popsize - 1)) * partProb;
    float aprob = ((float)i / (popsize - 1)) * aloneProb;
    individual = initialise(number, maxweight, pprob, aprob, &seed);
    population->at(i) = individual;
    scheduleWithHeuristicReg(individual, slm, pro, params.enableRASkip, -1,
                             Context::Schedule(ctx, 0, i));
  }
  std::sort(population->begin(), population->end(), sched_chromosome::compare);
  int RAPruneCount =
      slm->haveRAPruneCount() ? slm->getRAPruneCount() : params.RAPruneCount;
  int geneticRACount = 0;
  for (int i = 0; i < popsize; i++) {
    sched_chromosome *chrm = population->at(i);
    if (chrm->getHeuristicRegister() == 0 || chrm->getGeneticRegister() == 0)
      continue;
    if (RAPruneCount == -1 || geneticRACount++ < RAPruneCount)
      performGeneticRegisterAllocation(chrm, slm, pro,
                                       Context::Schedule(ctx, 0, i));
  }
  slm->collectInstructionsFromThreads();
  std::sort(population->begin(), population->end(), sched_chromosome::compare);
  int last_best_fitness =
      population->front()->getFitness().getFitness(); // fitness;
  int last_best_size = population->front()->sched_size;
  int skip_index = getRASkipIndex(population, COPY);
  int skip_size = population->at(skip_index)->sched_size;
  if (params.printSchedPopulation) {
    LOG_OUTPUT(LOG_M_SCHED_DETAIL, "%s SLM %d maxWeight %d\n",
               ctx.asString().c_str(), slm->getOriginalID(), maxweight);
    printPopulation(population, 0, 0, number, slm->getOriginalID(),
                    slm->getMinimalSize(), last_best_size, last_best_fitness,
                    skip_size, ctx);
  }

#if SCHED_DUP_CHECK
  for (int i = 0; i < popsize; ++i) {
    sched_chromosome *chrm = population->at(i);
    ga_stats::add_processed(ga_stats::NumberSequence(chrm->weights, number),
                            chrm->fitness, sched_chromosomes);
    ++processed_chromosomes;
    ga_stats::add_processed(ga_stats::NumberSequence(chrm->instructions),
                            chrm->fitness, sched_schedules);
  }
#endif

  uint generation = 0;
  int i = 0;

#if STORE_PARTINGS_AND_ALONE
  storePartingsAndAlone(slm, population->front());
#endif

  int best_possible = slm->getMinimalSize();
  LOG_OUTPUT(LOG_M_SCHED_DETAIL,
             "%s MI count for SLM %d for initial population is %d. (op count: "
             "%u, minimal size: %d, critical path: %d)\n",
             ctx.asString().c_str(), slm->getOriginalID(), last_best_fitness,
             size, best_possible, slm->getCriticalPath());
  if (params.sched_stats)
    SchedulingStats::instance()->printSchedulingStats(0, slm);

  int maxRounds = params.schedTrialRounds;
  bool solutionFound = false;

  while ((++i < maxRounds && !killed) && last_best_fitness > best_possible) {
    generation++;

    std::vector<struct sched_chromosome *> *parents = population;
    population = new std::vector<struct sched_chromosome *>();
    population->resize(popsize);

    /***
     * 1. Elitism: copy best individuals from previous generation.
     */
    for (int x = 0; x < COPY; x++) {
      sched_chromosome *chrm = parents->at(x);
      population->at(x) = chrm;
      chrm->origin = SchedGeneOrigin::COPY;
    }

    /***
     * 2. Generate individuals for next generation.
     */
#pragma omp parallel shared(population)
    {
      /* Crossover and mutation */
#pragma omp for nowait
      for (int x = 0; x < COMBINE; x++) {
        Context cmb_ctx = Context::Schedule(ctx, generation, x, "combine");
        sched_chromosome *parent1 =
            tournamentSelect(parents, params.schedTournamentSize, &seed);
        sched_chromosome *parent2 =
            tournamentSelect(parents, params.schedTournamentSize, &seed);
        int searchCount = 0;
        while (searchCount < 20 && parent1 == parent2) {
          parent2 =
              tournamentSelect(parents, params.schedTournamentSize, &seed);
          searchCount++;
        }
        LOG_OUTPUT(
            searchCount == 10,
            "%s Did not find different parents in instruction scheduling\n",
            ctx.asString().c_str());
        sched_chromosome *combined = combine(parent1, parent2, number, &seed);
        sched_chromosome *mutated =
            mutate(combined, number, &seed, maxweight, partProb, aloneProb);
        delete_sched_chromosome(&combined);
        population->at(COPY + x) = mutated;
      }

      /* Random individuals */
#pragma omp for
      for (int x = 0; x < CREATE; x++) {
        float pprob = ((float)x / (CREATE + 1)) * partProb;
        float aprob = ((float)x / (CREATE + 1)) * aloneProb;
        struct sched_chromosome *individual =
            initialise(number, maxweight, pprob, aprob, &seed);
        population->at(COPY + COMBINE + x) = individual;
      }

    } // end pragma parallel

    /***
     * 3. Perform scheduling of new individuals
     *    Use only heuristic register allocation!
     */
#pragma omp parallel for
    for (int x = 0; x < COMBINE + CREATE; ++x) {
      Context sched_ctx = Context::Schedule(ctx, generation, x);
      sched_chromosome *chrm = population->at(COPY + x);
      scheduleWithHeuristicReg(chrm, slm, pro, params.enableRASkip, skip_size,
                               sched_ctx);
    }
    flushLog();
    std::sort(population->begin(), population->end(),
              sched_chromosome::compare);

#if SCHED_DUP_CHECK
    for (int x = 0; x < popsize; ++x) {
      sched_chromosome *chrm = population->at(x);
      if (chrm->origin == SchedGeneOrigin::COPY)
        continue;
      ++processed_chromosomes;
      ga_stats::NumberSequence chrm_seq(chrm->weights, number);
      ga_stats::NumberSequence sched_seq(chrm->instructions);
      std::pair<bool, int> found =
          ga_stats::already_processed(chrm_seq, sched_chromosomes);
      if (found.first) {
        chrm->duplicateWeights = true;
        ++duplicate_chromosomes;
      } else {
        found = ga_stats::already_processed(sched_seq, sched_schedules);
        if (found.first) {
          chrm->duplicateSched = true;
          ++duplicate_schedules;
        }
      }
      ga_stats::add_processed(chrm_seq, chrm->fitness, sched_chromosomes);
      ga_stats::add_processed(sched_seq, chrm->fitness, sched_schedules);
    }
#endif

    /***
     * 4. Genetic register allocation
     */
    int geneticRACount = 0;
    int best_size = population->front()->sched_size;
    for (int x = 0; x < popsize; ++x) {
      sched_chromosome *c = population->at(x);
      if (c->getHeuristicRegister() == 0 || c->getGeneticRegister() >= 0 ||
          c->getGeneticRegister() == -2)
        continue;
      if (c->sched_size == best_size) {
        Context sched_ctx = Context::Schedule(ctx, generation, x);
        performGeneticRegisterAllocation(c, slm, pro, sched_ctx);
        ++geneticRACount;
        continue;
      }
      if (RAPruneCount == -1 || geneticRACount < RAPruneCount) {
        Context sched_ctx = Context::Schedule(ctx, generation, x);
        performGeneticRegisterAllocation(c, slm, pro, sched_ctx);
        ++geneticRACount;
      }
    }
    std::sort(population->begin(), population->end(),
              sched_chromosome::compare);
    slm->collectInstructionsFromThreads();

    if (params.printSchedPopulation) {
      LOG_OUTPUT(LOG_M_SCHED_DETAIL, "%s SLM %d maxWeight %d\n",
                 ctx.asString().c_str(), slm->getOriginalID(), maxweight);
      printPopulation(population, generation, i, number, slm->getOriginalID(),
                      slm->getMinimalSize(), last_best_size, last_best_fitness,
                      skip_size, ctx);
    }

    for (uint x = 0; x < population->size(); ++x) {
      sched_chromosome *indiv = population->at(x);
      if (indiv->getFitness().getFitness() <= best_possible) {
        LOG_OUTPUT(params.printSchedPopulation && isLog(LOG_M_ALWAYS),
                   "Found minimal size by %s\n",
                   indiv->getHeuristicRegister() == 0 ? "heuristic"
                                                      : "genetic");
      }
    }

    for (uint x = COPY, end = parents->size(); x < end; x++) {
      sched_chromosome *individual = parents->at(x);
      delete_sched_chromosome(&individual);
    }
    delete parents;

    struct sched_chromosome *individual = population->front();

#if STORE_PARTINGS_AND_ALONE
    if (individual->fitness <= last_best_fitness)
      storePartingsAndAlone(slm, individual);
#endif

    LOG_OUTPUT(LOG_M_SCHED_SIZE_HIST,
               "%s Best fitness in generation %d for SLM %d is %d\n",
               ctx.asString().c_str(), generation, slm->getOriginalID(),
               individual->getFitness().getFitness());
    if (individual->sched_size < last_best_size ||
        individual->sched_size > last_best_size) {
      last_best_size = individual->sched_size;
      last_best_fitness = individual->getFitness().getFitness();
      LOG_OUTPUT(LOG_M_SCHED_DETAIL,
                 "%s Improvement in scheduling in round %d for SLM %d to "
                 "fitness: %d\n",
                 ctx.asString().c_str(), generation, slm->getOriginalID(),
                 last_best_fitness);
      if (params.schedRounds > 0)
        i = 0;
    } else if (individual->sched_size == last_best_size &&
               individual->getFitness().getFitness() < last_best_fitness) {
      last_best_fitness = individual->getFitness().getFitness();
      LOG_OUTPUT(LOG_M_SCHED_DETAIL,
                 "%s Improvement in scheduling in round %d for SLM %d to "
                 "fitness: %d\n",
                 ctx.asString().c_str(), generation, slm->getOriginalID(),
                 last_best_fitness);
      if (params.schedRounds > 0)
        i = 0;
    }
    skip_index = getRASkipIndex(population, COPY);
    skip_size = population->at(skip_index)->sched_size;

    if (!solutionFound &&
        (last_best_fitness < NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET)) {
      /* Found the first solution. Now restart the genetic algorithms with the
       * usual schedRounds parameter.
       */
      solutionFound = true;
      maxRounds = abs(params.schedRounds);
      i = 0;

      slm->setHeuristicRegisterFitness(individual->getHeuristicRegister());
      slm->setGeneticRegisterFitness(individual->getGeneticRegister());
    }

    if (params.sched_stats)
      SchedulingStats::instance()->printSchedulingStats(generation, slm);
    flushLog();

#if CHECK_RA_TIMING
    printRATimings();
#endif
  } // end while

  sched_chromosome *final = population->front();
  slm->setHeuristicRegisterFitness(final->getHeuristicRegister());
  slm->setGeneticRegisterFitness(final->getGeneticRegister());

  for (uint x = 0; x < population->size(); x++) {
    sched_chromosome *individual = population->at(x);
    delete_sched_chromosome(&individual);
  }
  delete population;
  LOG_OUTPUT(LOG_M_SCHED,
             "%s Genetic scheduling for SLM %d returned fitness %d after %d "
             "generations\n",
             ctx.asString().c_str(), slm->getOriginalID(), last_best_fitness,
             generation);

#if SCHED_DUP_CHECK
  LOG_OUTPUT(LOG_M_ALWAYS,
             "%s Genetic scheduling processed %d chromosomes. Duplicates: %d "
             "(%6.2f) chrm, %d (%6.2f) sched\n",
             ctx.asString().c_str(), processed_chromosomes,
             duplicate_chromosomes,
             ((float)duplicate_chromosomes / processed_chromosomes) * 100,
             duplicate_schedules,
             ((float)duplicate_schedules / processed_chromosomes) * 100);
#endif

#if CHECK_RA_TIMING
  printRATimings();
#endif

  if (slm->getShortestInstruction()) {
    slm->writeOutReadable(std::cout);
    // print Virtual Register Count
    VirtualRegisterMap *map = nullptr;
    RegisterCoupling *coupling = nullptr;
    bool hasRegs =
        prepareRegisterAllocation(slm, pro, slm->getShortestInstruction(), &map,
                                  &coupling, Context("EMPTY"));
    int virtualRegisterCount = 0;
    for (auto ma : *map) {
      if (registers::isVirtualReg(ma.first)) {
        virtualRegisterCount++;
      }
    }
    releaseVirtualMap(&map);
    // manual memory cleaning
    if (coupling) {
      deleteCouplings(*coupling);
      delete coupling;
      coupling = nullptr;
    }
    LOG_OUTPUT(LOG_M_ALWAYS,
               "<--------------> SLM %d has %d virtual register and %d MOs "
               "<-------------->\n",
               slm->getOriginalID(), virtualRegisterCount,
               slm->getOperations()->size());
  }

  return last_best_size;
}

int gen_sched::genetic_scheduling(SLM *slm, Processor *pro,
                                  const Context &ctx) {
  if (!isPowerOptimization()) {
    return legacy_genetic_scheduling(slm, pro, ctx);
  } else {
    return power_genetic_scheduling(slm, pro, ctx);
  }
}

bool gen_sched::hasVirtualRegister(SLM *slm, Processor *pro, Program *program) {
  VirtualRegisterMap *map = nullptr;
  RegisterCoupling *coupling = nullptr;
  bool hasRegs = prepareRegisterAllocation(slm, pro, program, &map, &coupling,
                                           Context("EMPTY"));
  int virtualRegisterCount = 0;
  for (auto ma : *map) {
    if (registers::isVirtualReg(ma.first)) {
      virtualRegisterCount++;
    }
  }
  releaseVirtualMap(&map);
  // manual memory cleaning
  if (coupling) {
    deleteCouplings(*coupling);
    delete coupling;
    coupling = nullptr;
  }
  return virtualRegisterCount > 0;
}

int gen_sched::power_genetic_scheduling(SLM *slm, Processor *pro,
                                        const Context &ctx) {
  slm->calculateGraph(pro, true);
  LOG_OUTPUT(isLog(LOG_M_SCHED) || params.printSchedPopulation,
             "%s Starting genetic scheduling for SLM %d (op count: %lu, "
             "minimal size: %d, critical path: %d)\n",
             ctx.asString().c_str(), slm->getOriginalID(),
             slm->getOperations()->size(), slm->getMinimalSize(),
             slm->getCriticalPath());

#if CHECK_RA_TIMING
  ra_timings.reset();
#endif

#if SCHED_DUP_CHECK
  ga_stats::ChromosomesSet sched_chromosomes;
  ga_stats::ChromosomesSet sched_schedules;
  uint64_t processed_chromosomes = 0;
  uint64_t duplicate_chromosomes = 0;
  uint64_t duplicate_schedules = 0;
#endif

  //  uint seed = 500;
  uint seed = (unsigned int)time(nullptr);
  float partProb = getParameterPreferSLM(slm->getPartProb(), params.partProb);

  std::vector<MO *> *ops = slm->getOperations();
  uint size = ops->size();
  uint number = size;
  if (number == 0) {
    // todo: why do we schedule and RA for an empty BB (SLM)?
    if (params.sched_stats) {
      SchedulingStats::instance()->printSchedulingStats(0, slm);
    }
    // initialize individual Scheduling
    sched_chromosome *individual =
        initialise_from_weights(number, &seed, partProb, slm);

    //    if (!params.powerOptimizationFile.empty()) {
    //      //      allocateEnergyRegisterIndividual(individual, slm, pro);
    //      throw logic_error("not implemented yet!");
    //    } else {

    // by default perform RA with heursistic, only if it fails, perform RA by
    // GA.
    scheduleWithHeuristicReg(individual, slm, pro, params.enableRASkip, -1,
                             Context::Schedule(ctx, 0, 0));
    if (individual->failedHeuristicRA())
      performGeneticRegisterAllocation(individual, slm, pro,
                                       Context::Schedule(ctx, 0, 0));

    slm->collectInstructionsFromThreads();
    //    }

    LOG_OUTPUT(
        LOG_M_SCHED, "%s SLM %d contains no schedulable MOs. Final size = %d\n",
        ctx.asString().c_str(), slm->getOriginalID(), individual->sched_size);
    delete_sched_chromosome(&individual);
    return 0;
  }

  int popsize = getSchedPopulationSize(slm);
  int COPY = params.schedEliteCount;
  int CREATE = max(3, (int)(popsize * params.schedRandomRatio));
  int COMBINE = popsize - COPY - CREATE;
  LOG_OUTPUT(LOG_M_SCHED_DETAIL,
             "%s SLM %d genetic scheduling population size: %d\n",
             ctx.asString().c_str(), slm->getOriginalID(), popsize);

  auto *population = new std::vector<sched_chromosome *>();
  population->resize(popsize);
  int maxweight = number;
  float aloneProb =
      getParameterPreferSLM(slm->getAloneProb(), params.aloneProb);
  int startIndex = 1;
  /* First individual is different... */
  sched_chromosome *individual;
  if (params.includeDDGWeights) {
    individual = initialise_from_weights(number, &seed, partProb, slm);
    maxweight = get_max_weight(individual, number);
  } else {
    individual = initialise(number, maxweight, 0, 0, &seed);
  }
  scheduleWithHeuristicReg(individual, slm, pro, params.enableRASkip, -1,
                           Context::Schedule(ctx, 0, 0), false);
  if (!params.powerOptimizationFile.empty())
    individual->setTransitionEnergy(
        Estimator::getInstance().getCumulativeTransitionEnergy(
            individual->instructions, individual->map));
  if (individual->getTransitionEnergy() > 0) {
    individual->setHeuristicRegister(0);
  }
  population->at(0) = individual;

  // initialize Population
#pragma omp parallel for
  for (int i = startIndex; i < popsize; i++) {
    sched_chromosome *individual;
    float pprob = ((float)i / (popsize - 1)) * partProb;
    float aprob = ((float)i / (popsize - 1)) * aloneProb;
    individual = initialise(number, maxweight, pprob, aprob, &seed);
    population->at(i) = individual;
    scheduleWithHeuristicReg(individual, slm, pro, params.enableRASkip, -1,
                             Context::Schedule(ctx, 0, i), false);
    if (!params.powerOptimizationFile.empty()) {
      individual->setTransitionEnergy(
          Estimator::getInstance().getCumulativeTransitionEnergy(
              individual->instructions, individual->map));
    }
    if (individual->getTransitionEnergy() > 0) {
      individual->setHeuristicRegister(0);
    }
  }
  std::sort(population->begin(), population->end(), sched_chromosome::compare);
  int RAPruneCount =
      slm->haveRAPruneCount() ? slm->getRAPruneCount() : params.RAPruneCount;
  int geneticRACount = 0;
  for (int i = 0; i < popsize; i++) {
    sched_chromosome *chrm = population->at(i);
    if (chrm->successHeuristicRA() || chrm->successGeneticRA())
      continue;
    if (RAPruneCount == -1 || geneticRACount++ < RAPruneCount) {
      performGeneticRegisterAllocation(chrm, slm, pro,
                                       Context::Schedule(ctx, 0, i));
      if (chrm->successGeneticRA()) {
        chrm->setTransitionEnergy(
            Estimator::getInstance().getCumulativeTransitionEnergy(
                individual->instructions, individual->map));
      } else {
        chrm->setTransitionEnergy(-1.0);
      }
    }
  }
  slm->collectInstructionsFromThreads();
  std::sort(population->begin(), population->end(), sched_chromosome::compare);
  ga_stats::ChromosomeFitness last_best_fitness =
      population->front()->getFitness();
  int last_best_instr_transition =
      population->front()->getInstructionTransitions();
  ga_stats::ChromosomeFitness last_best_size = population->front()->sched_size;
  int skip_index = getRASkipIndex(population, COPY);
  int skip_size = population->at(skip_index)->sched_size;
  if (params.printSchedPopulation) {
    LOG_OUTPUT(LOG_M_SCHED_DETAIL, "%s SLM %d maxWeight %d\n",
               ctx.asString().c_str(), slm->getOriginalID(), maxweight);
    printPopulation(population, 0, 0, number, slm->getOriginalID(),
                    slm->getMinimalSize(), last_best_size.getFitness(),
                    last_best_fitness.getFitness(), skip_size, ctx);
  }

#if SCHED_DUP_CHECK
  for (int i = 0; i < popsize; ++i) {
    sched_chromosome *chrm = population->at(i);
    ga_stats::add_processed(ga_stats::NumberSequence(chrm->weight, number),
                            chrm->_chromosomeFitness, sched_chromosomes);
    ++processed_chromosomes;
    ga_stats::add_processed(ga_stats::NumberSequence(chrm->instructions),
                            chrm->getFitness, sched_schedules);
  }
#endif

  uint generation = 0;
  //  int i = 0;
  int noImprovement = 0;
  const bool hasVirtualReg =
      hasVirtualRegister(slm, pro, population->front()->instructions);

#if STORE_PARTINGS_AND_ALONE
  storePartingsAndAlone(slm, population->front());
#endif

  int best_possible = slm->getMinimalSize();
  LOG_OUTPUT(LOG_M_SCHED_DETAIL,
             "%s MI count for SLM %d for initial population is %d. (op count: "
             "%u, minimal size: %d, critical path: %d)\n",
             ctx.asString().c_str(), slm->getOriginalID(),
             last_best_fitness.getFitness(), size, best_possible,
             slm->getCriticalPath());
  if (params.sched_stats)
    SchedulingStats::instance()->printSchedulingStats(0, slm);

  int maxRounds = params.schedTrialRounds;
  bool solutionFound = false;

  while (abortCondition(maxRounds, noImprovement, killed, last_best_fitness,
                        best_possible)) {
    generation++;

    std::vector<sched_chromosome *> *parents = population;
    population = new std::vector<sched_chromosome *>();
    population->resize(popsize);

    /***
     * 1. Elitism: copy best individuals from previous generation.
     */
    for (int x = 0; x < COPY; x++) {
      sched_chromosome *chrm = parents->at(x);
      population->at(x) = chrm;
      chrm->origin = SchedGeneOrigin::COPY;
    }

    /***
     * 2. Generate individuals for next generation.
     */
#pragma omp parallel shared(population)
    {
      /* Crossover and mutation */
#pragma omp for nowait
      for (int x = 0; x < COMBINE; x++) {
        Context cmb_ctx = Context::Schedule(ctx, generation, x, "combine");
        sched_chromosome *parent1 =
            tournamentSelect(parents, params.schedTournamentSize, &seed);
        sched_chromosome *parent2 =
            tournamentSelect(parents, params.schedTournamentSize, &seed);
        int searchCount = 0;
        while (searchCount < 20 && parent1 == parent2) {
          parent2 =
              tournamentSelect(parents, params.schedTournamentSize, &seed);
          searchCount++;
        }
        LOG_OUTPUT(
            searchCount == 10,
            "%s Did not find different parents in instruction scheduling\n",
            ctx.asString().c_str());
        sched_chromosome *combined = combine(parent1, parent2, number, &seed);
        sched_chromosome *mutated =
            mutate(combined, number, &seed, maxweight, partProb, aloneProb);
        delete_sched_chromosome(&combined);
        population->at(COPY + x) = mutated;
      }

      /* Random individuals */
#pragma omp for
      for (int x = 0; x < CREATE; x++) {
        float pprob = ((float)x / (CREATE + 1)) * partProb;
        float aprob = ((float)x / (CREATE + 1)) * aloneProb;
        sched_chromosome *individual =
            initialise(number, maxweight, pprob, aprob, &seed);
        population->at(COPY + COMBINE + x) = individual;
      }

    } // end pragma parallel

    /***
     * 3. Perform scheduling of new individuals
     *    Use only heuristic register allocation!
     *    If Power RA enabled, also perform power GA on individuals that
     *    are at least as small or smaller than the lastBest SLM to save
     * computation.
     */
#pragma omp parallel for
    for (int x = 0; x < COMBINE + CREATE; ++x) {
      Context sched_ctx = Context::Schedule(ctx, generation, x);
      sched_chromosome *chrm = population->at(COPY + x);

      if (isPowerOptimization()) {

        // assign scheduling stats to chromosome
        int notScheduableCSCR;
        chrm->instructions =
            scheduleSLM(chrm, slm, pro, notScheduableCSCR, sched_ctx);
        int fitn = 0;
        if (chrm->instructions) {
          fitn = chrm->instructions->size();
          chrm->sched_size = fitn;
          chrm->setFitness(fitn);

          // perform RA
          int ind_Sched_Before =
              chrm->instructions->size(); // chrm->sched_size;
          if (ind_Sched_Before <= last_best_size.getFitness()) {
            // power RA
            Context sched_ctx = Context::Schedule(ctx, generation, x);
            // recreate scheduling of SLM
            //          int notScheduableCSCR;
            //          chrm->instructions =
            //              scheduleSLM(chrm, slm, pro, notScheduableCSCR,
            //              sched_ctx);
            // recreate VirtualRegisterMap
            //            slm->writeOutReadable(cout);
            bool hasRegs = prepareRegisterAllocation(
                slm, pro, chrm->instructions, &chrm->map, &chrm->couplings,
                Context::nextStage(sched_ctx, "reg_prep"));

            if (hasRegs) {
              if (params.raRegPopSize > 1) {
                // heuristic RA for seed
                VirtualRegisterMap *mapIntern = NULL;
                RegisterCoupling *couplingsIntern = NULL;
                int failedRegsIntern = 0;
                gen_sched::registerAllocation(
                    slm, pro, chrm->instructions, &mapIntern, &couplingsIntern,
                    failedRegsIntern, Context::nextStage(ctx, "first"));
                // allocate failed, free memory
                if (failedRegsIntern != 0) {
                  if (mapIntern) {
                    releaseVirtualMap(&mapIntern);
                  }
                  mapIntern = nullptr;
                }

                auto returnVal = geneticRegisterAllocationPower(
                    slm, pro, chrm->instructions, chrm->couplings, chrm->map,
                    &chrm->rdg, sched_ctx, mapIntern);
                chrm->setGeneticEnergyRegister(returnVal.getFitness());
                double transitionEnergy = returnVal.getTransitionEnergy();

                if (transitionEnergy == -1 and (not hasVirtualReg)) {
                  //                  cout << "Transition Energy == -1 and no
                  //                  Virtual Register, but still want to
                  //                  calculate transition energy" << endl;
                  transitionEnergy =
                      Estimator::getInstance().getCumulativeTransitionEnergy(
                          chrm->instructions, chrm->map);
                }
                //                cout << "normal set transition energy " <<
                //                endl;
                chrm->setTransitionEnergy(transitionEnergy);
              } else {
                // register balance RA
                scheduleWithHeuristicReg(chrm, slm, pro, params.enableRASkip,
                                         -1, Context::Schedule(ctx, 0, 0),
                                         false);
                if (!params.powerOptimizationFile.empty()) {
                  //                  cout << "Register Balance Energy
                  //                  Calculation" << endl;
                  chrm->setTransitionEnergy(
                      Estimator::getInstance().getCumulativeTransitionEnergy(
                          chrm->instructions, chrm->map));
                }
              }

            } else {
              // no virtual register in SLM
              // calculate Energy without RA
              //              std::cout << "No Virtual Register to Schedule" <<
              //              endl;
              chrm->instructions->calculateTransitionEnergy(chrm->map);
              chrm->setGeneticEnergyRegister(0);
              chrm->setTransitionEnergy(
                  chrm->instructions->getCumulativeTransitionEnergy());
            }

            if (population->at(COPY + x)->getTransitionEnergy() != -1) {
              if (fabs(
                      population->at(COPY + x)->getTransitionEnergy() +
                      InstructionModel::getInstance().generateInstructionEnergy(
                          population->at(COPY + x)->instructions) -
                      Estimator::getInstance().getCumulativeTransitionEnergy(
                          chrm->instructions, chrm->map)) > 0.5) {
                throw logic_error("Fitnesses dont match!\n");
              }
            }

            // todo: checker, remove for production
            if (ind_Sched_Before != chrm->instructions->size()) {
              throw logic_error("Fitnesses dont match ind_SChed_before!\n");
            }
          } else {
            // heuristic RA
            scheduleWithHeuristicReg(chrm, slm, pro, params.enableRASkip,
                                     skip_size, sched_ctx, false);
            chrm->setTransitionEnergy(
                Estimator::getInstance().getCumulativeTransitionEnergy(
                    chrm->instructions, chrm->map));
          }
        } else {
          fitn = NON_SCHEDULEABLE_OFFSET + notScheduableCSCR;
          chrm->map = NULL;
          chrm->sched_size = fitn;
          chrm->setFitness(fitn);
          chrm->setTransitionEnergy(-1);
          chrm->setGeneticEnergyRegister(-1);
          chrm->ra_skipped = true;
        }

      } else {
        scheduleWithHeuristicReg(chrm, slm, pro, params.enableRASkip, skip_size,
                                 sched_ctx, false);
      }
    }

    auto chrm = population->at(0);
    if (!(chrm->instructions && chrm->map)) {
      throw logic_error("No Instr and Map at population[0] NEW");
    }

    flushLog();
    std::sort(population->begin(), population->end(),
              sched_chromosome::compare);

#if SCHED_DUP_CHECK
    for (int x = 0; x < popsize; ++x) {
      sched_chromosome *chrm = population->at(x);
      if (chrm->origin == SchedGeneOrigin::COPY)
        continue;
      ++processed_chromosomes;
      ga_stats::NumberSequence chrm_seq(chrm->weight, number);
      ga_stats::NumberSequence sched_seq(chrm->instructions);
      std::pair<bool, int> found =
          ga_stats::already_processed(chrm_seq, sched_chromosomes);
      if (found.first) {
        chrm->duplicateWeights = true;
        ++duplicate_chromosomes;
      } else {
        found = ga_stats::already_processed(sched_seq, sched_schedules);
        if (found.first) {
          chrm->duplicateSched = true;
          ++duplicate_schedules;
        }
      }
      ga_stats::add_processed(chrm_seq, chrm->_chromosomeFitness,
                              sched_chromosomes);
      ga_stats::add_processed(sched_seq, chrm->getFitness, sched_schedules);
    }
#endif

    /***
     * 4. Genetic register allocation
     */
    int geneticRACount = 0;
    int best_size = population->front()->sched_size;
    for (int x = 0; x < popsize; ++x) {
      sched_chromosome *c = population->at(x);
      if (c->successHeuristicRA() || c->getGeneticRegister() >= 0 ||
          c->getGeneticRegister() == -2 || c->successEnergyRA())
        continue;
      if (c->sched_size == best_size) {
        Context sched_ctx = Context::Schedule(ctx, generation, x);
        performGeneticRegisterAllocation(c, slm, pro, sched_ctx);
        ++geneticRACount;
        continue;
      }
      if (RAPruneCount == -1 || geneticRACount < RAPruneCount) {
        Context sched_ctx = Context::Schedule(ctx, generation, x);
        performGeneticRegisterAllocation(c, slm, pro, sched_ctx);
        ++geneticRACount;
      }
    }
    std::sort(population->begin(), population->end(),
              sched_chromosome::compare);
    slm->collectInstructionsFromThreads();

    if (params.printSchedPopulation) {
      LOG_OUTPUT(LOG_M_SCHED_DETAIL | LOG_M_REG_ENERGY_DEBUG,
                 "%s SLM %d maxWeight %d\n", ctx.asString().c_str(),
                 slm->getOriginalID(), maxweight);
      printPopulation(population, generation, noImprovement, number,
                      slm->getOriginalID(), slm->getMinimalSize(),
                      last_best_size, last_best_fitness, skip_size, ctx);
      auto instr = population->front()->instructions;
      auto map = population->front()->map;
      if (instr) {
        if (map) {
          stringstream ss;
          //          instr->printInstructions(ss, map, false);
          instr->printInstructionsCompilable(ss, map);
          LOG_OUTPUT(LOG_M_ALWAYS, ss.str().c_str());
        }
      }
    }

    for (auto indiv : *population) {
      if (indiv->getFitness().getFitness() <= best_possible) {
        LOG_OUTPUT(params.printSchedPopulation &&
                       isLog(LOG_M_ALWAYS | LOG_M_REG_ENERGY),
                   "Found minimal size by %s\n",
                   indiv->successHeuristicRA() ? "heuristic" : "genetic");
      }
    }

    for (uint x = COPY, end = parents->size(); x < end; x++) {
      sched_chromosome *individual = parents->at(x);
      delete_sched_chromosome(&individual);
    }
    delete parents;

    sched_chromosome *individual = population->front();

#if STORE_PARTINGS_AND_ALONE
    if (individual->getFitness <= last_best_fitness)
      storePartingsAndAlone(slm, individual);
#endif

    LOG_OUTPUT(LOG_M_SCHED_SIZE_HIST,
               "%s Best getFitness in generation %d for SLM %d is %d\n",
               ctx.asString().c_str(), generation, slm->getOriginalID(),
               individual->getFitness().getFitness());
    if (individual->sched_size < last_best_size.getFitness() ||
        individual->sched_size > last_best_size.getFitness()) {
      last_best_size = individual->sched_size;
      last_best_fitness = individual->getFitness();
      LOG_OUTPUT(LOG_M_SCHED_DETAIL | LOG_M_REG_ENERGY_DEBUG,
                 "%s Improvement in scheduling in round %d for SLM %d to "
                 "getFitness: %d\n",
                 ctx.asString().c_str(), generation, slm->getOriginalID(),
                 last_best_fitness.getFitness());
      if (params.schedRounds > 0)
        noImprovement = 0;
    } else if (individual->sched_size == last_best_size.getFitness() &&
               compareTransitionEnergy(
                   individual->getFitness().getTransitionEnergy(),
                   last_best_fitness.getTransitionEnergy(),
                   individual->getInstructionTransitions(),
                   last_best_instr_transition)) {
      last_best_fitness = individual->getFitness();
      last_best_instr_transition = individual->getInstructionTransitions();
      LOG_OUTPUT(LOG_M_SCHED_DETAIL | LOG_M_REG_ENERGY_DEBUG,
                 "%s Improvement in scheduling in round %d for SLM %d to "
                 "getFitness: %d\n",
                 ctx.asString().c_str(), generation, slm->getOriginalID(),
                 last_best_fitness.getFitness());
      if (params.schedRounds > 0)
        noImprovement = 0;
    }
    skip_index = getRASkipIndex(population, COPY);
    skip_size = population->at(skip_index)->sched_size;

    if (!solutionFound &&
        (last_best_fitness < NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET)) {
      /* Found the first solution. Now restart the genetic algorithms with the
       * usual schedRounds parameter.
       */
      solutionFound = true;
      maxRounds = abs(params.schedRounds);
      noImprovement = 0;

      slm->setHeuristicRegisterFitness(
          individual->getHeuristicRegister()); // heuristicRegister;
      slm->setGeneticRegisterFitness(
          individual->getGeneticRegister()); // geneticRegister;
    }

    if (params.sched_stats)
      SchedulingStats::instance()->printSchedulingStats(generation, slm);
    flushLog();

#if CHECK_RA_TIMING
    printRATimings();
#endif
  } // end while

  if (!params.powerOptimizationFile.empty()) {
    std::cout << "GA Energy = " << last_best_fitness.getTransitionEnergy()
              << endl;
    auto chrm = population->at(0);
    if (chrm->instructions && chrm->map) {
      std::cout << "END: Calculated "
                << Estimator::getInstance().getCumulativeTransitionEnergy(
                       chrm->instructions, chrm->map)
                << std::endl;

    } else {
      throw logic_error("No Instr and Map at population[0]");
    }
    // assign Program and Virtual Register MAp to SLM
    // afterwards chromosome looses Program and Virtual REgister Map
    slm->setInstructionAndMap(&(population->front()->instructions),
                              &(population->front()->map));

    slm->getShortestInstruction()->setCumulativeTransitionEnergy(
        last_best_fitness.getTransitionEnergy());
    slm->calculateTransitionEnergy();
    double calculatedEnergy =
        slm->getShortestInstruction()->getCumulativeTransitionEnergy();
    std::cout << "GA Energy = " << last_best_fitness.getTransitionEnergy()
              << " Recalculated Energy = " << calculatedEnergy << std::endl;
  }
  sched_chromosome *final = population->front();
  slm->setHeuristicRegisterFitness(
      final->getFitness().getFitness()); // heuristicRegister;
  slm->setGeneticRegisterFitness(
      final->getGeneticRegister()); // geneticRegister;

  // function call will create endless loop (only in release, debug has no
  // problems)
  //  saveCharacteristics(slm, final);
  for (uint i = 0; i < slm->getOperations()->size(); i++) {
    ScheduledMOCharacteristics::getInstance().setChromosome(
        slm->getOperations()->at(i)->getID(), final->weights[i],
        final->partings[i], final->alone[i]);
  }

  int n = 3;
  for (auto individual : *population) {
    delete_sched_chromosome(&individual);
  }
  delete population;
  LOG_OUTPUT(LOG_M_SCHED | LOG_M_REG_ENERGY,
             "%s Genetic scheduling for SLM %d returned getFitness %d after %d "
             "generations\n",
             ctx.asString().c_str(), slm->getOriginalID(),
             last_best_fitness.getFitness(), generation);

#if SCHED_DUP_CHECK
  LOG_OUTPUT(LOG_M_ALWAYS,
             "%s Genetic scheduling processed %d chromosomes. Duplicates: %d "
             "(%6.2f) chrm, %d (%6.2f) sched\n",
             ctx.asString().c_str(), processed_chromosomes,
             duplicate_chromosomes,
             ((float)duplicate_chromosomes / processed_chromosomes) * 100,
             duplicate_schedules,
             ((float)duplicate_schedules / processed_chromosomes) * 100);
#endif

#if CHECK_RA_TIMING
  printRATimings();
#endif

  if (slm->getShortestInstruction()) {
    slm->writeOutReadable(std::cout);
    // print Virtual Register Count
    VirtualRegisterMap *map = nullptr;
    RegisterCoupling *coupling = nullptr;
    bool hasRegs =
        prepareRegisterAllocation(slm, pro, slm->getShortestInstruction(), &map,
                                  &coupling, Context("EMPTY"));
    int virtualRegisterCount = 0;
    for (auto ma : *map) {
      if (registers::isVirtualReg(ma.first)) {
        virtualRegisterCount++;
      }
    }
    releaseVirtualMap(&map);
    // manual memory cleaning
    if (coupling) {
      deleteCouplings(*coupling);
      delete coupling;
      coupling = nullptr;
    }
    LOG_OUTPUT(LOG_M_ALWAYS,
               "<--------------> SLM %d has %d virtual register and %d MOs "
               "<-------------->\n",
               slm->getOriginalID(), virtualRegisterCount,
               slm->getOperations()->size());
  }

  return last_best_size.getFitness();
}

string sched_chromosome::getString() const {
  stringstream ss;
  for (auto mi : *instructions) {
    if (mi) {
      ss << mi->getString(map) << "\n";
    }
  }
  return ss.str();
}

uint sched_chromosome::getInstructionTransitions() const {
  if (instructions) {
    return instructions->getInstructionTransitions();
  } else {
    return -1;
  }
}

string sched_chromosome::getHash(int opSize) const {
  string result = "";
  for (int i = 0; i < opSize; i++) {
    result += to_string(weights[i]) + "-";
  }
  return result;
}

bool sched_chromosome::failedHeuristicRA() const {
  return heuristicRegister != 0;
}

bool sched_chromosome::successHeuristicRA() const {
  return heuristicRegister == 0;
}

bool sched_chromosome::successEnergyRA() const {
  return geneticEnergyRegister == 0;
}

bool sched_chromosome::failedGeneticRA() const { return geneticRegister != 0; }

bool sched_chromosome::successGeneticRA() const { return geneticRegister == 0; }

bool sched_chromosome::successRA() const {
  return successHeuristicRA() || successGeneticRA() || successEnergyRA();
}

bool gen_sched::successRA(int returnValue) { return returnValue == 0; }