// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef HEADER_PORTOPTIMALREG_H_
#define HEADER_PORTOPTIMALREG_H_

#include "MI.h"
#include "Origin.h"
#include "ga_stats.h"
#include "global.h"
#include "processor.h"
#include "rdg.h"
#include "register.h"
#include "virtual_reg.h"
#include <unordered_map>
#include <vector>

#define REG_GA_STATS 0
#define PORT_OPT_RA_DUP_CHRM_CHECK 1

#if REG_GA_STATS
extern reg_ga_stats::RegGAStat regGAStats[];
#endif

class Context;

/**
 * Contains the port optimal register allocation with genetic algorithm.
 */
namespace portOptReg {

typedef Program::iterator Instructions;

/**
 * \brief Entry point to the port optimal register allocation with genetic
 * algorithm.
 */
int allocateRegisters(int slm_id, const rdg::RDG &rdg, ass_reg_t *blocked,
                      Processor *pro, Program *ins, VirtualRegisterMap *map,
                      const RegisterCoupling &couplings, const Context &,
                      VirtualRegisterMap *heuristicMap = nullptr);

/**
 * \brief Entry point to the port optimal register allocation with genetic
 * algorithm.
 */
ga_stats::ChromosomeFitness
allocateRegistersPower(int slm_id, const rdg::RDG &rdg, ass_reg_t *blocked,
                       Processor *pro, Program *ins, VirtualRegisterMap *map,
                       const RegisterCoupling &couplings, const Context &,
                       VirtualRegisterMap *heuristicMap = nullptr);

/* ###################################################################################################
 */
/* ################ Internal data
 * #################################################################### */
/* ###################################################################################################
 */

struct FitnessCounters {
  int read_conflicts[2];
  int write_conflicts[2];
  int extra_regs[2];
  //    int usedRegs[2];
  int conflictsWeighted;
  int balance;

  FitnessCounters() {
    read_conflicts[0] = read_conflicts[1] = 0;
    write_conflicts[0] = write_conflicts[1] = 0;
    extra_regs[0] = extra_regs[1] = 0;
    //        usedRegs[0] = usedRegs[1] = 0;
    conflictsWeighted = 0;
    balance = 0;
  }
};

/* ***************** Genetic algorithm ************************************** */

class Chromosome;
typedef int (*FitnessFunction)(Chromosome *, Program *, VirtualRegisterMap *,
                               const rdg::RDG &rdg, ass_reg_t *blockedRegisters,
                               Processor *);

/**
 * \brief Description of the chromosome used in port optimal genetic register
 * allocation.
 */
class Chromosome {
public:
private:
  size_t _length;
  int *_genes;
  int *_conflictCount;
  Chromosome(size_t length);
  int _fitness;
  FitnessCounters _fitCounters;
  CHROMOSOME::Origin _origin;
  VirtualAllocation *_regMapping;
  bool _duplicate;

  int _parent1_fit, _parent2_fit;

public:
  /** \brief Random initialization of a chromosome. */
  static Chromosome *random(size_t length, uint *seed);
  /** \brief Create a chromosome, where the first half is regfile 0, the second
   * half is regfile 1. */
  static Chromosome *half(size_t length);

  /** \brief Copy constructor. */
  Chromosome(const Chromosome &chrm);
  ~Chromosome();

  int &fitness() { return _fitness; }
  int fitness() const { return _fitness; }

  size_t length() const { return _length; }

  bool isDuplicate() const { return _duplicate; }

  void setDuplicate(bool duplicate = true) { _duplicate = duplicate; }

  int gene(size_t index) const {
    if (index < _length)
      return _genes[index];
    else {
      LOG_OUTPUT(LOG_M_ALWAYS, "Invalid index for gene in gene: %lu\n", index);
      return -1;
    }
  }

  int &conflictCount(size_t index) {
    if (index >= _length) {
      LOG_OUTPUT(LOG_M_ALWAYS, "ERROR: conflictCount index too large!\n");
      exit(0);
    }
    return _conflictCount[index];
  }

  const int &conflictCount(size_t index) const {
    if (index >= _length) {
      LOG_OUTPUT(LOG_M_ALWAYS, "ERROR: conflictCount index too large!\n");
      exit(0);
    }
    return _conflictCount[index];
  }

  int &parent1Fit() { return _parent1_fit; }

  const int &parent1Fit() const { return _parent1_fit; }

  int &parent2Fit() { return _parent2_fit; }

  const int &parent2Fit() const { return _parent2_fit; }

  const int *genes() const { return _genes; }

  void setGene(size_t index, int gene) {
    if (index < _length)
      _genes[index] = gene;
    else
      LOG_OUTPUT(LOG_M_ALWAYS, "Invalid index for gene in setGene: %lu\n",
                 index);
  }

  void flipGene(size_t index) { setGene(index, 1 - gene(index)); }

  VirtualAllocation *&regMapping() { return _regMapping; }

  const VirtualAllocation *regMapping() const { return _regMapping; }

  bool operator<(const Chromosome &chrm) const {
    return _fitness < chrm.fitness();
  }

  static bool compare(const Chromosome *c1, const Chromosome *c2) {
    return (*c1) < (*c2);
  }

  int evaluate(FitnessFunction f, Program *ins, VirtualRegisterMap *map,
               const rdg::RDG &rdg, ass_reg_t *blockedRegisters,
               Processor *pro);

  static Chromosome *combine(Chromosome *parent1, Chromosome *parent2,
                             uint *seed, float mutationProb);

  FitnessCounters &fitCounters();

  CHROMOSOME::Origin &origin() { return _origin; }
  const CHROMOSOME::Origin &origin() const { return _origin; }

  void printGenes() const;
  void printConflicts();
};

/**
 * \brief A population of Chromosomes.
 */
class Population {
private:
  size_t _size;
  std::vector<Chromosome *> _individuals;
  bool _ownsIndividuals;

public:
  /** \brief Construct a new, empty population with the given size. */
  explicit Population(size_t size, bool ownsIndividuals = true);
  Population(const Population &pop);

  static Population *random(size_t size, size_t chromosomeLength, uint *seed);
  ~Population();

  size_t size() const { return _size; }

  /** \brief Access an individual in the population. */
  const Chromosome *individual(size_t index) const;
  Chromosome *individual(size_t index);
  /** \brief Set an individual at the given position in the population. */
  void setIndividual(Chromosome *chrm, size_t index);

  /**
   * \brief Evaluate all individuals with the given fitness function.
   *
   * Returns the fittest individual.
   */
  Chromosome *evaluate(FitnessFunction f, Program *ins, VirtualRegisterMap *map,
                       const rdg::RDG &rdg, ass_reg_t *blockedRegisters,
                       Processor *pro);

  /** \brief Sort individuals in population by fitness.
   *
   * The fittest individual will be at individual(0) afterwards.
   */
  void sort();

  /**
   * \brief Select one individual. All individuals are selected uniformly.
   */
  Chromosome *selectUniform(uint *seed);

  /**
   * Select an individual by tournament selection.
   */
  Chromosome *tournamentSelect(int tournamentSize, uint *seed);

  void releaseIndividuals();

  bool &ownsIndividuals();
  const bool &ownsIndividuals() const;

  void print();
};

/* ***************** Register allocation algorithms ************************* */

extern std::unordered_map<int, int> successful_RA_hist, failed_RA_hist;
void clearRAHists();
void printRAHists();
} // namespace portOptReg

#endif /* HEADER_PORTOPTIMALREG_H_ */
