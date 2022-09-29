// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_ENERGYCHROMOSOME_H
#define SCHEDULER_ENERGYCHROMOSOME_H

#include <memory>

#include "../../Processor/ProcessorConfiguration.h"
#include "CombineReturn.h"
#include "EnergyGene.h"
#include <Origin.h>
#include <algorithm>
#include <gen_register.h>
#include <iterator>
#include <sstream>

namespace portOptReg {
class EnergyChromosome {
private:
  void repairGene(std::vector<std::pair<EnergyGene *, const EnergyGene *>>
                      &availableGenesForCombination,
                  int rootGene, uint *seed, const Program *ins);

  void repairGeneGreedy(uint *seed, const Program *ins, int gene);

  void
  combineSingleGene(std::vector<std::pair<EnergyGene *, const EnergyGene *>>
                        &availableGenesForCombination,
                    uint *seed, const Program *ins);

  /***
   * Only combine a Gene, if it increases the power metric.
   * @param availableGenesForCombination
   * @param seed
   * @param ins
   * @param True if combine successfull.
   */
  bool combineSingleGeneGreedy(
      std::vector<std::pair<EnergyGene *, const EnergyGene *>>
          &availableGenesForCombination,
      uint *seed, const Program *ins);

  /**
   * randomly combine a number of genes. Minimum combine 1 gene, max
   * raRegMaxCombineRoots
   * @param maxDiffGenes
   * @param seed
   * @return number of combines
   */
  int calcRandomMaxCombineGenes(int maxDiffGenes, uint *seed);

  // std::shared_ptr<RegisterLivelihood> _registerLivelihood;
  std::vector<EnergyGene> genes;
  /**
   * Owner of Virtual Registers, Genes only get pointer.
   */
  std::vector<std::shared_ptr<VirtualRegisterMapping>> virtualRegisters;

  //  int _fitness;
  ga_stats::ChromosomeFitness _chromosomeFitness;

  bool _duplicate;

  ga_stats::ChromosomeFitness _parent1_fit, _parent2_fit;
  int parent1ID, parent2ID;
  std::string parent1HASH, parent2HASH;
  CHROMOSOME::Origin parent1Origin, parent2Origin;
  CHROMOSOME::Origin origin;
  int mutationAttempts;
  int maxMutationNumber;
  int differentVirtualRegister;
  //  VirtualAllocation *_regMapping;
  //  unique_ptr<VirtualRegisterMap> virtualRegisterMap;

  const ass_reg_t *_blocked;

  COMBINE_RETURN combineReturn;
  std::vector<int> realRegisterStateRepair;
  std::vector<int> realRegisterStateCombine;

  // VirtualRegisterMap *collectVirtualRegisterMap();

  void createCoupledEnergyGeneReferences();
  std::vector<VirtualRegisterMapping *> getCoupledRegister();
  EnergyGene *getEnergyGene(VirtualRegisterMapping *virtualRegister);

  /**
   * Temporarily save Virtual Register States (physical register).
   */
  void saveRegStateRepair() {
    realRegisterStateRepair.clear();
    for (int i = 0; i < virtualRegisters.size(); i++) {
      realRegisterStateRepair.push_back(virtualRegisters[i]->getReal());
    }
  }

  /**
   * Restore a previous state of Virtual Registers (physical registers)
   */
  void restoreRegStateRepair() {
    for (int i = 0; i < virtualRegisters.size(); i++) {
      virtualRegisters[i]->setReal(realRegisterStateRepair[i]);
    }
  }

  /**
   * Temporarily save Virtual Register States (physical register).
   */
  void saveRegStateCombine() {
    realRegisterStateCombine.clear();
    for (int i = 0; i < virtualRegisters.size(); i++) {
      realRegisterStateCombine.push_back(virtualRegisters[i]->getReal());
    }
  }

  /**
   * Restore a previous state of Virtual Registers (physical registers)
   */
  void restoreRegStateCombine() {
    for (int i = 0; i < virtualRegisters.size(); i++) {
      virtualRegisters[i]->setReal(realRegisterStateCombine[i]);
    }
  }

public:
  EnergyChromosome() {}

  EnergyChromosome(
      std::vector<EnergyGene> genes,
      std::vector<std::shared_ptr<VirtualRegisterMapping>> virtualRegisters,
      CHROMOSOME::Origin origin, const ass_reg_t *blocked)
      : genes(genes), virtualRegisters(virtualRegisters), _duplicate(false),
        origin(origin), _blocked(blocked) {
    createCoupledEnergyGeneReferences();
  }

  void copyPhysical(const EnergyChromosome &energyChromosome);

  std::vector<EnergyGene> &getGenes() { return genes; }

  std::string toString() const {
    std::stringstream ss;
    ss << "Energy Chromosome: " << hash() << " Parent2 Hash:" << parent2HASH
       << " : ";
    for (auto &gene : genes) {
      ss << gene.toString() << " - ";
    }
    ss << toStr(origin);
    ss << " " << _chromosomeFitness.toString();
    return ss.str();
  }

  std::string gene2String(string delimiter = "  ", bool simple = false) const {
    //    std::stringstream ss;
    std::string ss;
    ss += "Energy Chromosome: \n";
    for (auto &gene : genes) {
      ss += " - " + gene.toString(delimiter, simple);
    }
    return ss;
  }

  ga_stats::ChromosomeFitness getFitness() const { return _chromosomeFitness; }

  const ga_stats::ChromosomeFitness &getChromosomeFitness() const {
    return _chromosomeFitness;
  }

  void setFitness(ga_stats::ChromosomeFitness fitness) {
    _chromosomeFitness = fitness;
  }

  //  CHROMOSOME::Origin getOrigin(){return origin;}

  bool isDuplicate() const { return _duplicate; }

  void setDuplicate(bool duplicate = true) { _duplicate = duplicate; }

  void setParent1Fitness(ga_stats::ChromosomeFitness fitness) {
    _parent1_fit = fitness;
  }
  ga_stats::ChromosomeFitness getParent1Fitness() const { return _parent1_fit; }

  void setParent2Fitness(ga_stats::ChromosomeFitness fitness) {
    _parent2_fit = fitness;
  }
  ga_stats::ChromosomeFitness getParent2Fitness() const { return _parent2_fit; }

  /***
   * Throw error if Coupled Register are not inside the same EnergyGene and
   * other sanity check fail.
   */
  void checkCouplings();

  void setParent1(const EnergyChromosome &parent1, int index) {
    parent1ID = index;
    parent1Origin = parent1.getOrigin();
    parent1HASH = parent1.hash();
    _parent1_fit = parent1.getFitness();
  }

  void setParent2(const EnergyChromosome &parent2, int index) {
    parent2ID = index;
    parent2Origin = parent2.getOrigin();
    parent2HASH = parent2.hash();
    _parent2_fit = parent2.getFitness();
  }

  void setParent1ID(int id) { parent1ID = id; }
  void setParent2ID(int id) { parent2ID = id; }
  void setParent1HASH(std::string hash) { parent1HASH = hash; }
  void setParent2HASH(std::string hash) { parent2HASH = hash; }

  void setParent1Origin(CHROMOSOME::Origin origin) { parent1Origin = origin; }
  CHROMOSOME::Origin getParent1Origin() const { return parent1Origin; }
  void setParent2Origin(CHROMOSOME::Origin origin) { parent2Origin = origin; }
  CHROMOSOME::Origin getParent2Origin() const { return parent2Origin; }

  VirtualRegisterMapping *getFirstCollision();

  int mutateGeneGreedy(const ass_reg_t *blocked, uint *seed,
                       CHROMOSOME::Origin targetOrigin, const Program *ins);

  /**
   * Mutate 1 to raRegMaxMutationFraction genes. will ensure different
   * Chromosome.
   * @param blocked
   * @param seed
   * @param targetOrigin
   * @return Number of Genes to Mutate
   */
  int mutateGene(const ass_reg_t *blocked, uint *seed,
                 CHROMOSOME::Origin targetOrigin, const Program *ins) {

    if (params.raRegMutateGreedy) {
      return mutateGeneGreedy(blocked, seed, targetOrigin, ins);
    } else {
      std::string preHash = hash();
      int mutateGene = util::uniRandom(0, virtualRegisters.size() - 1, seed);
      do {
        std::vector<int> freeRegs = genes[0].getFreePhysicalRegister();
        int index = util::uniRandom(0, freeRegs.size() - 1, seed);
        virtualRegisters[mutateGene]->setReal(freeRegs[index]);
        for (int i = 0; i < genes.size(); i++) {
          genes[i].repairConflicts(seed, blocked);
        }
      } while (preHash == hash());
      _chromosomeFitness = ga_stats::ChromosomeFitness();
      origin = targetOrigin;
      return mutateGene;
    }
  }

  //  VirtualAllocation *&regMapping() { return _regMapping; }
  //  const VirtualAllocation *regMapping() const { return _regMapping; }

  VirtualRegisterMap *getVirtualRegisterMap() {
    VirtualRegisterMap *virtualRegisterMap = new VirtualRegisterMap();
    for (auto &gene : genes) {
      gene.collectVirtualRegisterMap(virtualRegisterMap);
    }
    return virtualRegisterMap;
  }

  void setVirtualRegisterMap(VirtualRegisterMap *exampleMap) {
    for (auto &gene : genes) {
      gene.setVirtualRegisterMap(exampleMap);
    }
    if (checkConflict(true)) {
      cout << gene2String() << endl;
      //      throw runtime_error("Heuristik RA leads to invalid Register
      //      Allocation!");
      LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG, "Could not find heuristic SEED.\n");
    }
  }

  int evaluate(const Program *ins);
  // bool successfulAllocation() {exit(26); // return false;}
  // int calculateFitnessValue() { exit(27); }
  const CHROMOSOME::Origin getOrigin() const { return origin; }
  void setOrigin(CHROMOSOME::Origin origin) { this->origin = origin; }

  void setTransitionEnergy(double transitionEnergy) {
    _chromosomeFitness.setTransitionEnergy(transitionEnergy);
  }
  double getTransitionEnergy() const {
    return _chromosomeFitness.transitionEnergy;
  }

  bool validRegisterAllocation() { return not checkConflict(); }

  void randomize(uint *seed, const ass_reg_t *blocked) {
    for (int i = 0; i < genes.size(); i++) {
      genes[i].randomize(seed, blocked);
    }
  }

  void deserialize(std::unordered_map<int, int> virtual2Physical) {
    for (int i = 0; i < genes.size(); i++) {
      genes[i].deserialize(virtual2Physical);
    }
  }

  ga_stats::NumberSequence getNumberSequence() {
    std::vector<int> tmp;
    for (int i = 0; i < genes.size(); i++) {
      auto physicalRegisters = genes[i].getPhysicalRegister();
      for (auto &phy : physicalRegisters) {
        tmp.push_back(phy);
      }
    }
    return ga_stats::NumberSequence(tmp);
  }

  /**
   *
   * @param parent2
   * @param seed
   * @return True if mutation, False if only combine
   */
  bool combine(const EnergyChromosome &parent2, uint *seed, const Program *ins);

  /**
   * Gredy Combine Chromosomes.
   * Randomly select different Genes to combine.
   * First Gene is combined no matter what.
   * All other genes are evaluated, if they increase power metric. If so, they
   * will be added.
   * @param parent2
   * @param seed
   * @return True if mutation, False if only combine
   */
  bool combineGreedy(const EnergyChromosome &parent2, uint *seed,
                     const Program *ins);

  std::string hash() const;
  std::string getOriginString() const;

  std::string getParentsString() const;

  int getGeneNumber() const { return genes.size(); }

  std::vector<double> evaluateDetails(const Program *ins);

  bool operator==(const EnergyChromosome &other) const;

  std::string printHierarchyDepth() const;
  int getVirtualRegisterCount() const;

  VirtualRegisterMapping *getVirtualRegister(string virtualRegisterName) {
    for (int i = 0; i < genes.size(); i++) {
      auto gene = genes[i].getVirtualRegister(virtualRegisterName);
      if (gene) {
        return gene;
      }
    }
    return nullptr;
  }

  std::vector<VirtualRegisterMapping *>
  getBlockedRegister(string virtualRegisterName) {
    for (int i = 0; i < genes.size(); i++) {
      auto gene = genes[i].getBlockedRegister(virtualRegisterName);
      if (gene.size() > 0) {
        return gene;
      }
    }
    return std::vector<VirtualRegisterMapping *>();
  }

  /***
   *
   * @return True if conflict detected.
   */
  bool checkConflict(bool verbose = false) {
    for (int i = 0; i < genes.size(); i++) {
      if (genes[i].checkConflict(verbose)) {
        _chromosomeFitness.transitionEnergy = -1;
        return true;
      }
    }
    return false;
  }

  std::vector<uint> getSlmBlockedPhysicalRegister() {
    return genes[0].getSlmBlockedPhysicalRegister();
  }

  /**
   *
   * @param parent2
   * @return A list of Pairs of Chromosomes, that are available for combination.
   * First Pointer is to a gene from this Chromsome, second Pointer is to gene
   * from parent2
   */
  std::vector<std::pair<EnergyGene *, const EnergyGene *>>
  getAvailableGenesForCombination(const EnergyChromosome &parent2);

  int numberEqualGenes(const EnergyChromosome &energyChromosome) const;

  void
  setNumberDifferentVirtRegs(const EnergyChromosome &comparableChromeosome);

  void appendCombineMutationNumberDifferentVirtRegs();

  string getDifferentNumberVirtualRegsString() const;
};

static bool smaller(const EnergyChromosome &e1, const EnergyChromosome &e2) {
  return e1.getChromosomeFitness() < e2.getChromosomeFitness();
}

} // namespace portOptReg

#endif // SCHEDULER_ENERGYCHROMOSOME_H
