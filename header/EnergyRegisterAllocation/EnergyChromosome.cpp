// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "CompilerContext.h"
#include <boost/format.hpp>
#include <boost/histogram.hpp>
#include <fstream>

#include "../../powerEstimation/TransitionEnergyEstimator.h"
#include "EnergyChromosome.h"
#include "EnergyRegisterAllocation/RegisterLivelihood.h"
#include "MI.h"

namespace portOptReg {

int EnergyChromosome::calcRandomMaxCombineGenes(int maxDiffGenes, uint *seed) {
  int geneSelectionVar = util::uniRandom(1, params.raRegMaxCombineRoots, seed);
  if (maxDiffGenes < params.raRegMaxCombineRoots) {
    geneSelectionVar = maxDiffGenes;
  }
  LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG,
             "Availalbe Genes for Combination: %d - choose %d of %d \n",
             maxDiffGenes, geneSelectionVar, getGeneNumber());
  combineReturn.numberCombineRoots = geneSelectionVar;
  return geneSelectionVar;
}

bool EnergyChromosome::combine(const EnergyChromosome &parent2, uint *seed,
                               const Program *ins) {
  combineReturn = COMBINE_RETURN();
  string preCombineHash = hash();
  origin = CHROMOSOME::COMBINE;
  bool startedWithConflict = checkConflict();

  std::vector<std::pair<EnergyGene *, const EnergyGene *>>
      availableGenesForCombination = getAvailableGenesForCombination(parent2);

  int geneSelectionVar =
      calcRandomMaxCombineGenes(availableGenesForCombination.size(), seed);

  for (int i = 0; i < geneSelectionVar; i++) {
    combineSingleGene(availableGenesForCombination, seed, ins);
  }

  if (startedWithConflict) {
    for (int i = 0; i < genes.size(); i++) {
      genes[i].repairConflicts(seed, _blocked);
    }
    origin = CHROMOSOME::COMREPAIR;
  } else {
    if (checkConflict()) {
      //      throw runtime_error("Error Snuck in Combination\n");
      for (int i = 0; i < genes.size(); i++) {
        genes[i].repairConflicts(seed, _blocked);
      }
      origin = CHROMOSOME::COMREPAIR;
    }
  }

  return combineReturn.combine;
}

bool EnergyChromosome::combineGreedy(const EnergyChromosome &parent2,
                                     uint *seed, const Program *ins) {
  combineReturn = COMBINE_RETURN();
  string preCombineHash = hash();
  origin = CHROMOSOME::COMBINE;
  bool startedWithConflict = checkConflict();

  std::vector<std::pair<EnergyGene *, const EnergyGene *>>
      availableGenesForCombination = getAvailableGenesForCombination(parent2);

  int geneSelectionVar =
      calcRandomMaxCombineGenes(availableGenesForCombination.size(), seed);

  int abort = 0;
  if (geneSelectionVar != 0) {
    // always add first available Gene
    combineSingleGene(availableGenesForCombination, seed, ins);

    for (int i = 1; i < geneSelectionVar; i++) {
      if (combineSingleGeneGreedy(availableGenesForCombination, seed, ins)) {
        abort++;
      }
    }
  }

  LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG,
             "RA REG POWER: Aborted %d/%d Combine Genes \n", abort,
             geneSelectionVar);

  if (startedWithConflict && checkConflict()) {
    for (int i = 0; i < genes.size(); i++) {
      genes[i].repairConflicts(seed, _blocked);
    }
    origin = CHROMOSOME::COMREPAIR;
  }
  return combineReturn.combine;
}

void EnergyChromosome::combineSingleGene(
    std::vector<std::pair<EnergyGene *, const EnergyGene *>>
        &availableGenesForCombination,
    uint *seed, const Program *ins) {
  // select random root Genes
  int rootGene =
      util::uniRandom(0, availableGenesForCombination.size() - 1, seed);

  COMBINE_RETURN temp = availableGenesForCombination[rootGene].first->combine(
      *availableGenesForCombination[rootGene].second, seed, _blocked);

  combineReturn.append(temp);

  if (temp.conflict) {
    repairGene(availableGenesForCombination, rootGene, seed, ins);
  }

  // remove already combined Root genes
  auto index = std::find(availableGenesForCombination.begin(),
                         availableGenesForCombination.end(),
                         availableGenesForCombination[rootGene]);
  availableGenesForCombination.erase(index);
}

bool EnergyChromosome::combineSingleGeneGreedy(
    std::vector<std::pair<EnergyGene *, const EnergyGene *>>
        &availableGenesForCombination,
    uint *seed, const Program *ins) {
  // select random root Genes
  int rootGene =
      util::uniRandom(0, availableGenesForCombination.size() - 1, seed);

  saveRegStateCombine();
  evaluate(ins);
  double previousEnergy = _chromosomeFitness.getTransitionEnergy();

  COMBINE_RETURN temp = availableGenesForCombination[rootGene].first->combine(
      *availableGenesForCombination[rootGene].second, seed, _blocked);

  combineReturn.append(temp);

  if (temp.conflict) {
    repairGene(availableGenesForCombination, rootGene, seed, ins);
  }

  // remove already combined Root genes
  auto index = std::find(availableGenesForCombination.begin(),
                         availableGenesForCombination.end(),
                         availableGenesForCombination[rootGene]);
  availableGenesForCombination.erase(index);

  evaluate(ins);
  double newEnergy = _chromosomeFitness.getTransitionEnergy();
  if (compareTransitionEnergy(previousEnergy, newEnergy)) {
    restoreRegStateCombine();
    return false;
  }
  return true;
}

void EnergyChromosome::repairGene(
    std::vector<std::pair<EnergyGene *, const EnergyGene *>>
        &availableGenesForCombination,
    int rootGene, uint *seed, const Program *ins) {

  if (params.raRegRepairCombineGreedy) {
    int gene = 0;
    for (int k = 0; k < genes.size(); k++) {
      if (*availableGenesForCombination[rootGene].first == genes[k]) {
        gene = k;
        break;
      }
    }
    repairGeneGreedy(seed, ins, gene);
  } else {
    availableGenesForCombination[rootGene].first->repairConflicts(seed,
                                                                  _blocked);
  }

  if (checkConflict()) {
    for (int i = 0; i < genes.size(); i++) {
      genes[i].repairConflicts(seed, _blocked);
    }
  }
  origin = CHROMOSOME::COMREPAIR;
}

// VirtualRegisterMap *EnergyChromosome::collectVirtualRegisterMap() {
//  VirtualRegisterMap *virtualRegisterMap = new VirtualRegisterMap();
//  for (auto &gene : genes) {
//    gene.collectVirtualRegisterMap(virtualRegisterMap);
//  }
//
//  return virtualRegisterMap;
//}

int EnergyChromosome::evaluate(const Program *ins) {
  checkConflict();
  bool allocatable = true;
  int nonAllocatable = 0;
  for (auto &gene : genes) {
    if (!gene.hasFoundAllocation()) {
      allocatable = false;
      nonAllocatable += gene.getNonAllocatable();
    }
  }

  _chromosomeFitness.setFitness(nonAllocatable);

  if (allocatable != validRegisterAllocation()) {
    cout << "EnergyChromosome::evaluate : allocatable and "
            "validRegisterAllocation() do not match!\n";
  }

  if (allocatable) {
    TransitionEnergyEstimator energyEstimator;
    VirtualRegisterMap *mapping = getVirtualRegisterMap();

    try {
      for (auto mis : *ins) {
        energyEstimator.push(mis, mapping);
      }
      releaseVirtualMap(&mapping);
      _chromosomeFitness.setTransitionEnergy(
          energyEstimator.getTransitionEnergy());
    } catch (const exception &e) {
      cout << "EnergyChromosome::evaluate : has an error!\n";
      _chromosomeFitness.setFitness(100);
      _chromosomeFitness.setTransitionEnergy(-1);
      toString();
      releaseVirtualMap(&mapping);
    }
  }

  return _chromosomeFitness.getFitness();
}

std::vector<double> EnergyChromosome::evaluateDetails(const Program *ins) {
  checkConflict();
  bool allocatable = true;
  int nonAllocatable = 0;
  for (auto gene : genes) {
    if (!gene.hasFoundAllocation()) {
      allocatable = false;
      nonAllocatable += gene.getNonAllocatable();
    }
  }
  _chromosomeFitness.setFitness(nonAllocatable);

  if (allocatable != validRegisterAllocation()) {
    cout << "EnergyChromosome::evaluateDetails : allocatable and "
            "validRegisterAllocation() do not match!\n";
  }

  if (allocatable) {
    TransitionEnergyEstimator energyEstimator;
    VirtualRegisterMap *mapping = getVirtualRegisterMap();

    try {
      for (auto mis : *ins) {
        energyEstimator.push(mis, mapping);
      }
      releaseVirtualMap(&mapping);
      _chromosomeFitness.setTransitionEnergy(
          energyEstimator.getTransitionEnergy());
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "\nRecalculated Bests Individual : Chromosome Fitness: %f\n",
                 _chromosomeFitness.getTransitionEnergy());
      return energyEstimator.getHistory();
    } catch (const exception &e) {
      cout << "EnergyChromosome::evaluateDetails : has an error!\n";
      _chromosomeFitness.setFitness(100);
      _chromosomeFitness.setTransitionEnergy(-1);
      releaseVirtualMap(&mapping);
      return std::vector<double>();
    }
  }
  return std::vector<double>();
}

std::string EnergyChromosome::hash() const {
  string s;
  for (int i = 0; i < virtualRegisters.size(); i++) {
    s += std::to_string(virtualRegisters[i]->getReal()) + "-";
  }
  return to_string(std::hash<std::string>{}(s)); // gene2String()));
}

// std::string EnergyChromosome::getOriginString() const {
//   stringstream ss;
//   ss << toStr(origin) << " " << hash() << " p1: " << toStr(parent1Origin) <<
//   " "
//      << parent1HASH << " p2: " << toStr(parent2Origin) << " " << parent2HASH;
//   return ss.str();
// }

bool EnergyChromosome::operator==(const EnergyChromosome &other) const {
  return genes == other.genes;
}

std::string EnergyChromosome::printHierarchyDepth() const {
  if (origin == CHROMOSOME::COMBINE or origin == CHROMOSOME::COMREPAIR) {
    stringstream ss;
    ss << combineReturn.toString();
    return ss.str();

  } else {
    return "";
  }
}

void EnergyChromosome::copyPhysical(const EnergyChromosome &energyChromosome) {
  for (int i = 0; i < genes.size(); i++) {
    genes[i].copyPhysical(energyChromosome.genes[i]);
  }
  _blocked = energyChromosome._blocked;
}

int EnergyChromosome::numberEqualGenes(
    const EnergyChromosome &energyChromosome) const {
  int equalGenes = 0;
  for (int i = 0; i < genes.size(); i++) {
    if (genes[i].size() > 1) {
      if (energyChromosome.genes[i] == genes[i]) {
        equalGenes++;
      }
    }
  }
  return equalGenes;
}

std::vector<VirtualRegisterMapping *> EnergyChromosome::getCoupledRegister() {
  std::vector<VirtualRegisterMapping *> coupledVirtualRegister;
  for (auto &gene : genes) {
    auto v = gene.getCoupledRegisters();
    for (auto virtualReg : v) {
      coupledVirtualRegister.push_back(virtualReg);
    }
  }
  return coupledVirtualRegister;
}

EnergyGene *
EnergyChromosome::getEnergyGene(VirtualRegisterMapping *virtualRegister) {
  //  std::cout << "Looking for VirtualRegister: "
  //            << virtualRegister->getVirtualName() << std::endl;
  for (auto &gene : genes) {
    auto energyGene = gene.getGene(virtualRegister);
    if (energyGene) {
      return energyGene;
    }
  }
  //  std::cout << gene2String() << std::endl;
  throw logic_error("Could not find Gene of VirtualRegister: " +
                    virtualRegister->toString());
}

void EnergyChromosome::createCoupledEnergyGeneReferences() {

  std::vector<VirtualRegisterMapping *> coupledVirtualRegister =
      getCoupledRegister();

  for (auto virtualRegister : coupledVirtualRegister) {
    auto firstRegister = virtualRegister;
    auto secondRegister = virtualRegister->getCoupled();

    auto firstEnergyGene = getEnergyGene(firstRegister);
    auto secondEnergyGene = getEnergyGene(secondRegister);

    firstEnergyGene->addCoupledEnergyGene(firstRegister, secondEnergyGene);
    secondEnergyGene->addCoupledEnergyGene(secondRegister, firstEnergyGene);
  }
}

void EnergyChromosome::checkCouplings() {
  bool error = false;
  for (int i = 0; i < genes.size(); i++) {
    if (not genes[i].checkGene()) {
      error = true;
      LOG_OUTPUT(LOG_M_ALWAYS, "Error in Gene:\n%s\n",
                 genes[i].toString().c_str());
    }
  }
  if (error) {
    LOG_OUTPUT(LOG_M_ALWAYS, "Error Chromosome:\n%s\n", toString().c_str());
    throw logic_error("Chromosome has an error!\n");
  }
}

std::vector<std::pair<EnergyGene *, const EnergyGene *>>
EnergyChromosome::getAvailableGenesForCombination(
    const EnergyChromosome &parent2) {
  std::vector<std::pair<EnergyGene *, const EnergyGene *>> result;
  for (int i = 0; i < genes.size(); i++) {
    if (genes[i] != parent2.genes[i]) {
      result.push_back(std::pair<EnergyGene *, const EnergyGene *>(
          &genes[i], &parent2.genes[i]));
    }
  }
  // test result
  //  if(result.size() > 0){
  //    uint seed = 33;
  //    cout << "native Genes: " << hash() << endl;
  //    for(int i=0; i< result.size(); i++){
  //      result[i].first->randomize(&seed,blocked);
  //    }
  //    cout << "after Modification Genes: " << hash() << endl;
  //  }

  return result;
}

string EnergyChromosome::getOriginString() const {
  string ss;
  ss += toStr(origin);
  if (origin == CHROMOSOME::MUTATE or origin == CHROMOSOME::COMBMUT) {
    ss += " " + std::to_string(mutationAttempts) + "/" +
          std::to_string(maxMutationNumber);
  }
  return ss;
}

void EnergyChromosome::setNumberDifferentVirtRegs(
    const EnergyChromosome &comparableChromeosome) {
  differentVirtualRegister = 0;
  for (int i = 0; i < virtualRegisters.size(); i++) {
    if (virtualRegisters[i]->getReal() !=
        comparableChromeosome.virtualRegisters[i]->getReal()) {
      if (virtualRegisters[i]->getVirtual() !=
          comparableChromeosome.virtualRegisters[i]->getVirtual()) {
        throw runtime_error("Virtual Registers have different orders !!! \n");
      }
      differentVirtualRegister++;
    }
  }
}

void EnergyChromosome::appendCombineMutationNumberDifferentVirtRegs() {
  differentVirtualRegister += mutationAttempts;
}

string EnergyChromosome::getDifferentNumberVirtualRegsString() const {
  if (origin != CHROMOSOME::RANDOM) {
    return " diff-reg " + std::to_string(differentVirtualRegister) + "/" +
           std::to_string(virtualRegisters.size());
  } else {
    return "";
  }
}

std::string EnergyChromosome::getParentsString() const {
  stringstream ss;
  ss << " parent1: " << getParent1Fitness().getTransitionEnergy() << " "
     << parent1ID << " ORIGIN_p1: " << toStr(getParent1Origin()) << " HASH "
     << parent1HASH;
  if (parent2HASH != "") {
    ss << " parent2: " << getParent2Fitness().getTransitionEnergy()
       << " ORIGIN_p2: " << toStr(getParent2Origin()) << " " << parent2ID
       << " HASH " << parent2HASH;
  }
  ss << " Individual HASH " << hash() << endl;
  return ss.str();
}

void EnergyChromosome::repairGeneGreedy(uint *seed, const Program *ins,
                                        int gene) {
  //  VirtualRegisterMapping* collision = getFirstCollision();
  VirtualRegisterMapping *collision = genes[gene].getFirstCollision();
  int firstConflict = collision->getReal();

  // solve all conflicts
  //  for(int i=0;i < genes.size(); i++ ){
  //    genes[i].repairConflicts(seed, _blocked);
  //  }
  genes[gene].repairConflicts(seed, _blocked);

  // optimize first Collision to power targets
  saveRegStateRepair();
  std::vector<int> freeRegs = genes[0].getFreePhysicalRegister();

  // set first FreeRegs as solution
  collision->setReal(freeRegs[0]);
  for (int k = 0; k < genes.size(); k++) {
    genes[k].repairConflicts(seed, _blocked);
  }
  evaluate(ins);
  double bestEnergy = _chromosomeFitness.getTransitionEnergy();
  int bestIndex = 0;
  std::vector<int> bestRegisters;
  for (int k = 0; k < virtualRegisters.size(); k++) {
    bestRegisters.push_back(virtualRegisters[k]->getReal());
  }
  // iterate over the remaining freeRegs to find a greedy better power solution
  for (int i = 1; i < freeRegs.size(); i++) {
    restoreRegStateRepair();
    collision->setReal(freeRegs[i]);
    for (int k = 0; k < genes.size(); k++) {
      genes[k].repairConflicts(seed, _blocked);
    }
    evaluate(ins);
    if (compareTransitionEnergy(_chromosomeFitness.getTransitionEnergy(),
                                bestEnergy)) {
      bestIndex = i;
      bestEnergy = _chromosomeFitness.getTransitionEnergy();
      // save best physical registre combination
      bestRegisters.clear();
      for (int k = 0; k < virtualRegisters.size(); k++) {
        bestRegisters.push_back(virtualRegisters[k]->getReal());
      }
    }
  }

  // set best power solution to register
  for (int i = 0; i < virtualRegisters.size(); i++) {
    virtualRegisters[i]->setReal(bestRegisters[i]);
  }
  bool conflict = checkConflict();
  if (conflict) {
    int n = 3;
  }
}

VirtualRegisterMapping *EnergyChromosome::getFirstCollision() {
  for (int i = 0; i < genes.size(); i++) {
    if (genes[i].checkConflict()) {
      return genes[i].getFirstCollision();
    }
  }
  return nullptr;
}

int EnergyChromosome::mutateGeneGreedy(const ass_reg_t *blocked, uint *seed,
                                       CHROMOSOME::Origin targetOrigin,
                                       const Program *ins) {
  std::string preHash = hash();
  //    int mutateGene = util::uniRandom(0, virtualRegisters.size() - 1,
  //    seed);
  // mutate 1 to raRegMaxMutate Genes
  maxMutationNumber =
      max(1, int(virtualRegisters.size() * params.raRegMaxMutationFraction));

  mutationAttempts = util::uniRandom(1, maxMutationNumber, seed);

  int counter = 0;
  do {
    std::vector<int> freeRegs = genes[0].getFreePhysicalRegister();
    // mutate all but one register random
    for (int i = 0; i < mutationAttempts - 1; i++) {
      int mutateGene = util::uniRandom(0, virtualRegisters.size() - 1, seed);
      int index = util::uniRandom(0, freeRegs.size() - 1, seed);
      virtualRegisters[mutateGene]->setReal(freeRegs[index]);
      for (int i = 0; i < genes.size(); i++) {
        genes[i].repairConflicts(seed, blocked);
      }
    }
    // deepcopy original virtual Register
    std::vector<int> originalVirtualRegisters;
    for (int i = 0; i < virtualRegisters.size(); i++) {
      originalVirtualRegisters.push_back(virtualRegisters[i]->getReal());
    }
    // last mutation is directed towards optimization goal.
    int mutateGene = util::uniRandom(0, virtualRegisters.size() - 1, seed);
    freeRegs.erase(freeRegs.begin(),
                   std::find(freeRegs.begin(), freeRegs.end(),
                             virtualRegisters[mutateGene]->getReal()));
    virtualRegisters[mutateGene]->setReal(freeRegs[0]);
    for (int i = 0; i < genes.size(); i++) {
      genes[i].repairConflicts(seed, blocked);
    }
    evaluate(ins);
    double bestPower = _chromosomeFitness.getTransitionEnergy();
    int bestIndex = 0;
    std::vector<int> bestRegisters = originalVirtualRegisters;
    for (int i = 1; i < freeRegs.size(); i++) {
      // restore Virtual Registers for test
      for (int k = 0; k < virtualRegisters.size(); k++) {
        virtualRegisters[k]->setReal(originalVirtualRegisters[k]);
      }
      virtualRegisters[mutateGene]->setReal(freeRegs[i]);
      for (int i = 0; i < genes.size(); i++) {
        genes[i].repairConflicts(seed, blocked);
      }
      evaluate(ins);

      if (compareTransitionEnergy(_chromosomeFitness.getTransitionEnergy(),
                                  bestPower)) {
        bestPower = _chromosomeFitness.getTransitionEnergy();
        bestIndex = i;
        //          cout << "Found Better Physical Register in " +
        //          std::to_string(freeRegs[i]) + " with " +
        //          std::to_string(bestPower) << endl;
        bestRegisters.clear();
        for (int k = 0; k < virtualRegisters.size(); k++) {
          bestRegisters.push_back(virtualRegisters[k]->getReal());
        }
      }
    }

    // set best real register
    // restore Virtual Registers for test
    for (int k = 0; k < virtualRegisters.size(); k++) {
      virtualRegisters[k]->setReal(bestRegisters[k]);
    }

    //      for (int i = 0; i < genes.size(); i++) {
    //        genes[i].repairConflicts(seed, blocked);
    //      }
    counter++;
  } while (preHash == hash() &&
           counter < params.raRegMaxMutationDoublicateAbort);
  if (not(counter < params.raRegMaxMutationDoublicateAbort)) {
    LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG,
               "Mutation max Rounds reached! Aborting Mutation Search for "
               "better individual.\n ");
  }
  _chromosomeFitness = ga_stats::ChromosomeFitness();
  origin = targetOrigin;
  return mutationAttempts;
}

} // namespace portOptReg
