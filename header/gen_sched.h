// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef HEADER_GEN_SCHED_H_
#define HEADER_GEN_SCHED_H_

#include "listsearch.h"
#include "rdg.h"
#include "virtual_reg.h"
#include <vector>

class SLM;
class MI;
class Processor;
class Context;

#define SCHED_DUP_CHECK 0

#define NOENERGY_OFFSET 50000

namespace gen_sched {

class SchedGeneOrigin {
public:
  enum Origin { RANDOM, CROSSOVER, MUTATE, COPY, WEIGHTS };
  static std::string toString(Origin origin);
};

/**
 *
 * Scheduling information are mutated inside this sched_chromosome.
 * !!! Be careful: Chromosome and sched_chromosome are different!!!!!
 *
 * This Class contains vectors for weight, parting and alone information of
 * the MOs it contains.
 *
 */
class sched_chromosome {
private:
  /**
   * Fitness of the chromosome. This is the MI size of the MI.
   * Consists of 2 Parts: 1) found scheduled+heuristic 2) not found (offset with
   * NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET)
   */
  ga_stats::ChromosomeFitness _chromosomeFitness;
  //  int fitness;

  /**
   * The number of non allocatable Register for this Chromosome.
   */
  int heuristicRegister = -1;
  /**
   * The number of non allocatable Register for this Chromosome.
   */
  int geneticRegister = -1;
  /**
   * The number of non allocatable Register for this Chromosome.
   */
  int geneticEnergyRegister = -1;

public:
  int sched_size;
  /**
   * //todo: What am I?
   */
  bool ra_skipped = false;

  ga_stats::ChromosomeFitness p1Fit, p2Fit;
  int dummy_count; // number of dummy registers used
  int *weights = nullptr;
  bool *partings = nullptr;
  bool *alone = nullptr;
  bool duplicateWeights = false;
  bool duplicateSched = false;
  SchedGeneOrigin::Origin origin;
  VirtualRegisterMap *map;
  RegisterCoupling *couplings = nullptr;
  Program *instructions = nullptr;
  rdg::RDG *rdg = nullptr;

  void setHeuristicRegister(int heuristic) { heuristicRegister = heuristic; }
  void setGeneticRegister(int genetic) { geneticRegister = genetic; }
  int getHeuristicRegister() const { return heuristicRegister; }
  int getGeneticRegister() const { return geneticRegister; }
  bool failedGeneticRegisterAllocation() const { return geneticRegister < 0; }
  double getTransitionEnergy() const {
    return _chromosomeFitness.getTransitionEnergy();
  }
  void setTransitionEnergy(double transitionEnergy) {
    _chromosomeFitness.setTransitionEnergy(transitionEnergy);
    if (transitionEnergy < 0) {
      if (successRA()) {
        throw runtime_error("Successfull RA but no Energy!");
      }
      int previousFitness = _chromosomeFitness.getFitness();
      _chromosomeFitness.setFitness(NOENERGY_OFFSET + previousFitness);
    }
  }
  void setGeneticEnergyRegister(int geneticEnergy) {
    geneticEnergyRegister = geneticEnergy;
  }
  int getGeneticEnergyRegister() { return geneticEnergyRegister; }

  int getAloneCount() const {
    int count = 0;
    for (int i = 0; i < sched_size; i++) {
      if (alone[i]) {
        count++;
      }
    }
    return count;
  }

  ~sched_chromosome() {
    releaseVirtualMapping();
    releaseCouplings();
  }

  void releaseVirtualMapping() {
    if (map) {
      for (auto mit : *map) // VirtualRegisterMap::iterator mit = map->begin();
                            // mit != map->end();++mit)
      //      for (VirtualRegisterMap::iterator mit = map->begin(); mit !=
      //      map->end();++mit)
      {
        delete mit.second;
      }
      delete map;
      map = nullptr;
    }
  }

  void releaseCouplings() {
    if (couplings) {
      deleteCouplings(*couplings);
      delete couplings;
      couplings = nullptr;
    }
  }

  /**
   * Currently only checking base name of instruction.
   * X2, CR, CS, etc not considered.
   * @return The number of instruction changes in the program.
   */
  uint getInstructionTransitions() const;

  bool operator<(const sched_chromosome &chrm) const {
    if (successRA() and chrm.successRA()) {
      // genetic scheduling + genetic RA power
      if (!params.powerOptimizationFile.empty()) {
        if (sched_size == chrm.sched_size) {
          if (CONSTANT::powerEqualThreshold >= 0) {
            return compareTransitionEnergy(
                _chromosomeFitness.getTransitionEnergy(),
                chrm.getTransitionEnergy(), getInstructionTransitions(),
                chrm.getInstructionTransitions());
          } else {
            return compareTransitionEnergy(
                _chromosomeFitness.getTransitionEnergy(),
                chrm.getTransitionEnergy());
          }
        } else {
          return sched_size < chrm.sched_size;
        }
      } else {
        return sched_size < chrm.sched_size;
      }
    } else if (successRA() and not chrm.successRA()) {
      return true;
    } else if (not successRA() and chrm.successRA()) {
      return false;
    } else {
      if (sched_size == chrm.sched_size) {
        if (!params.powerOptimizationFile.empty()) {
          if (CONSTANT::powerEqualThreshold >= 0) {
            return compareTransitionEnergy(
                _chromosomeFitness.getTransitionEnergy(),
                chrm.getTransitionEnergy(), getInstructionTransitions(),
                chrm.getInstructionTransitions());
          } else {
            return compareTransitionEnergy(
                _chromosomeFitness.getTransitionEnergy(),
                chrm.getTransitionEnergy());
          }
        } else {
          if (failedGeneticRegisterAllocation() ||
              chrm.failedGeneticRegisterAllocation()) { // one genetic RA failed
            return heuristicRegister <
                   chrm.getHeuristicRegister(); // heuristicRegister;
          } else {
            return geneticRegister <
                   chrm.getGeneticRegister(); // geneticRegister;
          }
        }

      } else {
        return sched_size < chrm.sched_size;
      }
    }
  }

  static bool compare(const sched_chromosome *c1, const sched_chromosome *c2) {
    return (*c1) < (*c2);
  }

  string getString() const;

  string getHash(int opSize) const;

  ga_stats::ChromosomeFitness getFitness() const { return _chromosomeFitness; }

  void setFitness(int fitness, double transitionEnergy = -1.0) {
    this->_chromosomeFitness.setFitness(fitness);
    if (transitionEnergy > 0) {
      _chromosomeFitness.setTransitionEnergy(transitionEnergy);
    }
  }
  void setFitness(ga_stats::ChromosomeFitness referenceFitness) {
    _chromosomeFitness = ga_stats::ChromosomeFitness(referenceFitness);
  }

  /**
   * getFitness += fitnessIncrement
   * @param fitnessIncrement
   */
  void incrementFitness(int fitnessIncrement) {
    _chromosomeFitness.incrementPrimary(fitnessIncrement);
  }

  //  int calculatePowerConsumption() const;
  bool failedHeuristicRA() const;
  bool successHeuristicRA() const;
  bool successEnergyRA() const;
  bool failedGeneticRA() const;
  bool successGeneticRA() const;
  bool successRA() const;

  void saveCharacteristics() const;
};

sched_chromosome *initialise(int chromosomeLength, int maxweight,
                             float partProb, float aloneProb, uint *seed);
sched_chromosome *initialise_from_weights(int size, uint *seed, int partValue,
                                          SLM *slm, int maxweight = -1);
sched_chromosome *tournamentSelect(std::vector<sched_chromosome *> *pop,
                                   int tournamentSize, uint *seed);
sched_chromosome *combine(sched_chromosome *first, sched_chromosome *second,
                          int size, uint *seed);
sched_chromosome *mutate(sched_chromosome *pop, int size, uint *seed,
                         int maxweight, float partProb, float aloneProb);
void printPopulation(std::vector<sched_chromosome *> *population,
                     int generation, int noImprovement, int opcount, int slm_id,
                     int minsize, ga_stats::ChromosomeFitness lastBestSize,
                     ga_stats::ChromosomeFitness lastBestFit, int skip_size,
                     const Context &ctx);

// int allocateEnergyRegisterIndividual(sched_chromosome *chrm, SLM* slm,
// Processor *pro, const Context &sched_ctx){
//  int notScheduableCSCR;
//
//  chrm->instructions =
//      scheduleSLM(chrm, slm, pro, notScheduableCSCR, sched_ctx );
//  int fitn = 0;
//  if (chrm->instructions) {
//    fitn = chrm->instructions->size();
//
//  }
//}

/**
 * Schedule SLM with Processorconfiguration. Scheduling and Register Allocation
 * are done here. This method also contains the Genetic Algorithm control-loop.
 * !! This will also perform power optimization is selected.
 * @param slm
 * @param pro
 * @param ctx
 * @return the number of MIs inside this BB (SLM).
 */
int genetic_scheduling(SLM *slm, Processor *pro, const Context &ctx);

/**
 * Schedule SLM with Processorconfiguration. Scheduling and Register Allocation
 * are done here. This method also contains the Genetic Algorithm control-loop.
 * @param slm
 * @param pro
 * @param ctx
 * @return the number of MIs inside this BB (SLM).
 */
int legacy_genetic_scheduling(SLM *slm, Processor *pro, const Context &ctx);

/**
 * Schedule SLM with Processorconfiguration. Scheduling and Register Allocation
 * are done here. This method also contains the Genetic Algorithm control-loop.
 * @param slm
 * @param pro
 * @param ctx
 * @return the number of MIs inside this BB (SLM).
 */
int power_genetic_scheduling(SLM *slm, Processor *pro, const Context &ctx);

/***
 *
 * @param maxRounds get number of max rounds to run genetic Scheduling
 * @param noImprovement current generation count
 * @param killed
 * @param last_best_fitness  curent best SLM Size found {SLM Size +
 * TransitionEnergy}
 * @param best_possible shortest SLM size {SLM Size + TransitionEnergy}
 * @return
 */
bool abortCondition(int maxRounds, int &noImprovement, bool killed,
                    ga_stats::ChromosomeFitness last_best_fitness,
                    ga_stats::ChromosomeFitness best_possible);

/**
 *
 *
 * @param chrm
 * @param slm
 * @param pro
 * @param enableRASkip
 * @param last_best
 * @param sched_ctx
 */
void scheduleWithHeuristicReg(sched_chromosome *chrm, SLM *slm, Processor *pro,
                              bool enableRASkip, int last_best,
                              const Context &sched_ctx,
                              bool overWriteSLMAndDeleteCHRM = true);
void performGeneticRegisterAllocation(sched_chromosome *chrm, SLM *slm,
                                      Processor *pro, const Context &sched_ctx);

/**
 * Schedule individual with List Scheduling. No RA done here!
 * @param individual
 * @param slm
 * @param processor
 * @param notScheduableCSCR
 * @param ctx
 * @return a scheduled program if possible, if FAIL, return 0
 */
Program *scheduleSLM(sched_chromosome *individual, SLM *slm,
                     Processor *processor, int &notScheduableCSCR,
                     const Context &ctx);

bool prepareRegisterAllocation(SLM *slm, Processor *pro, Program *ins,
                               VirtualRegisterMap **map,
                               RegisterCoupling **couolings,
                               const Context &ctx);

/***
 *
 * Execute a register allocation for a SLM.
 * @param slm
 * @param processor
 * @param instructions
 * @param map
 * @param couplings
 * @param failedRegs Returnvalue: Returns the number of missing Register for
 * allocation.
 * @param ctx
 */
void registerAllocation(SLM *slm, Processor *processor, Program *instructions,
                        VirtualRegisterMap **map, RegisterCoupling **couplings,
                        int &failedRegs, const Context &ctx);
void checkWeakFollowers(MO *mo, std::set<int> &scheduled, ListSearch *alg);
bool allPredecessorsRunning(MO *m, std::set<int> &scheduled, MO *currentMO);

extern int heuristicRACount, geneticRACount, failedRACount;

bool successRA(int returnValue);

bool hasVirtualRegister(SLM *slm, Processor *pro, Program *program);

} // namespace gen_sched

#endif /* HEADER_GEN_SCHED_H_ */
