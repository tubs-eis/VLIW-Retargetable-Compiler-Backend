// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "EnergyGene.h"
#include "DoubleRegister.h"
#include <cstring>
#include <iostream>

namespace portOptReg {

void EnergyGene::setCoupledRegister(
    std::pair<int, int> &allocatedCoupledPhysicalRegisters,
    VirtualRegisterMapping *coupledVirtualRegister) {

  if (coupledVirtualRegister->getVirtual() >
      coupledVirtualRegister->getCoupled()->getVirtual()) {
    coupledVirtualRegister->setCoupledRegister(
        allocatedCoupledPhysicalRegisters.second,
        allocatedCoupledPhysicalRegisters.first);
  } else {
    coupledVirtualRegister->setCoupledRegister(
        allocatedCoupledPhysicalRegisters.first,
        allocatedCoupledPhysicalRegisters.second);
  }
}

void EnergyGene::randomize(uint *seed, const ass_reg_t *blocked) {
  if (!childGenes.empty()) {
    nonAllocatable = 0;
    for (auto &childEnergyGenes : childGenes) {
      childEnergyGenes.randomize(seed, blocked);
      nonAllocatable += childEnergyGenes.nonAllocatable;
    }
  } else {

    std::vector<int> usedRegister;

    //    std::vector<int> originalAvailableRegisters(MAX_REGISTER);
    //    std::iota(std::begin(originalAvailableRegisters),
    //              std::end(originalAvailableRegisters), 0);
    std::vector<int> originalAvailableRegisters = getFreePhysicalRegister();

    ass_reg_t *Copyblocked = new ass_reg_t[registers::getNumRegisterFiles()];
    memcpy(Copyblocked, blocked,
           registers::getNumRegisterFiles() * sizeof(ass_reg_t));

    // get previously allocated coupled Virtual Register
    for (auto &coupledIndices : coupledVirtualRegisterIndex) {
      if (virtualRegisters[coupledIndices]->isAllocated()) {
        addBlockedReg(Copyblocked, virtualRegisters[coupledIndices]->getReal());
        addBlockedReg(
            Copyblocked,
            virtualRegisters[coupledIndices]->getCoupled()->getReal());
      }
    }

    // Randomly Allocate Coupled VirtualRegisters
    for (auto &coupledIndices : coupledVirtualRegisterIndex) {
      if (!virtualRegisters[coupledIndices]->isAllocated()) {
        std::pair<int, int> allocatedCoupledPhysicalRegisters =
            getDoubleFreeRegistersPairRandom(
                Copyblocked, virtualRegisters[coupledIndices]->isConcurrent(),
                seed);

        // get ordering of coupled virtual allocatedCoupledPhysicalRegisters
        // correct
        setCoupledRegister(allocatedCoupledPhysicalRegisters,
                           virtualRegisters[coupledIndices]);

        // add allocated coupled Registers to used Registers
        usedRegister.push_back(allocatedCoupledPhysicalRegisters.first);
        usedRegister.push_back(allocatedCoupledPhysicalRegisters.second);
      }
    }
    delete[] Copyblocked;

    for (int i = 0; i < virtualRegisters.size(); i++) {
      auto virtualRegister = virtualRegisters[i];

      // coupled register have already been allocated and dont need another
      // allocation!
      if (virtualRegister->isVirtualReg() and
          (not virtualRegister->isCoupled())) {
        // tmp phycial register can be assigned without generating conflicts
        std::vector<int> tmp = originalAvailableRegisters;

        // remove already used physical register.
        for (int v : usedRegister) {
          tmp.erase(std::remove(tmp.begin(), tmp.end(), v), tmp.end());
        }

        // remove already allocated register
        for (auto &blockedVirtualRegister : blockedRegisters[i]) {
          if (blockedVirtualRegister->isAllocated()) {
            tmp.erase(std::remove(tmp.begin(), tmp.end(),
                                  blockedVirtualRegister->getReal()),
                      tmp.end());
          }
        }

        // if no allocation possible, abort
        if (tmp.size() < 1) {
          //          std::cout
          //              << " EnergyGene randomize:: Missing Register may be
          //              counted "
          //                 "wrong if coupled register exist! ";
          nonAllocatable = virtualRegisters.size() - i;
          return;
        }

        int newPhysicalRegister = getRandomPhysicalRegister(
            tmp, seed); // tmp[util::uniRandom(0, tmp.size(), seed)];
        virtualRegister->setReal(newPhysicalRegister);
        usedRegister.push_back(newPhysicalRegister);
      }
    }

    //  for (auto regs : virtualRegisters) {
    //    std::cout << regs->toString() << std::endl;
    //  }
    //  std::cout << "New Line" << std::endl;
    nonAllocatable = 0;
  }
}

void EnergyGene::collectVirtualRegisterMap(
    VirtualRegisterMap *virtualRegisterMap) {
  if (childGenes.empty()) {
    for (VirtualRegisterMapping *virtualRegister : virtualRegisters) {
      VirtualRegisterMapping *m = new VirtualRegisterMapping(virtualRegister);
      auto virtualReg = virtualRegister->getVirtual();
      virtualRegisterMap->insert(
          std::pair<const char32_t, VirtualRegisterMapping *>(virtualReg, m));
    }
  } else {
    for (int i = 0; i < childGenes.size(); i++) {
      childGenes[i].collectVirtualRegisterMap(virtualRegisterMap);
    }
  }
}

bool EnergyGene::hasMutated() const { return mutation; }

bool EnergyGene::checkConflict(bool verbose) {
  // check MAC conflict
  nonAllocatable = 0;
  if (childGenes.empty()) {
    calculateCollisions(verbose);

    if (physicalCollisionIndex.size() > 0 or
        physicalCollisionIndexCoupled.size() > 0) {
      nonAllocatable =
          physicalCollisionIndex.size() + physicalCollisionIndexCoupled.size();
      return true;
    }
    return false;
  } else {
    bool conflict = false;
    for (int i = 0; i < childGenes.size(); i++) {
      if (childGenes[i].checkConflict()) {
        nonAllocatable += childGenes[i].nonAllocatable;
        conflict = true;
      }
    }
    return conflict;
  }
}

bool EnergyGene::repairConflicts(uint *seed, const ass_reg_t *blocked) {
  mutation = false;

  if (childGenes.empty()) {
    nonAllocatable = 0;

    calculateCollisions();
    if (physicalCollisionIndex.size() > 0) {
      mutation = true;
    }

    if (physicalCollisionIndexCoupled.size() >= 1) {
      repairCoupledCollision(seed, blocked);
      calculateCollisions();
      if (physicalCollisionIndexCoupled.size() > 0) {
        // could not fix coupled Collision (probably global physical Register
        // are blocking coupled allocation)
        nonAllocatable += NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET +
                          physicalCollisionIndexCoupled.size();
      }
    }

    usedPhysicalRegister = std::vector<int>();
    std::vector<int> originalAvailableRegisters = getFreePhysicalRegister();
    // remove already selected physiacl Register from same cluster
    //    std::cout << "Already Used Physical Register" << std::endl;
    //    for (auto newPhysicalRegister : usedPhysicalRegister) {
    //      std::cout << newPhysicalRegister << std::endl;
    //      originalAvailableRegisters.erase(
    //          std::remove(originalAvailableRegisters.begin(),
    //                      originalAvailableRegisters.end(),
    //                      newPhysicalRegister),
    //          originalAvailableRegisters.end());
    //    }

    for (int i = 0; i < physicalCollisionIndex.size(); i++) {
      int index = physicalCollisionIndex[i];
      auto s = registers::getName(virtualRegisters[index]->getVirtual());

      // tmp phycial register can be assigned without generating conflicts
      auto sizeOriginal = originalAvailableRegisters.size();
      std::vector<int> tmp = originalAvailableRegisters;

      // remove physical register that live at the same time.
      removeBlockedRegister(tmp, index);

      if (tmp.size() <= 0) {
        nonAllocatable += physicalCollisionIndex.size() - i;
        return false;
      }
      int newPhysicalRegister = getRandomPhysicalRegister(tmp, seed);
      if (DEBUG_TEXT) {
        std::cout << virtualRegisters[index]->getVirtualName() << " : "
                  << virtualRegisters[index]->getRealName() << " -> "
                  << registers::getName(newPhysicalRegister) << std::endl;
      }
      virtualRegisters[index]->setReal(newPhysicalRegister);
      usedPhysicalRegister.push_back(newPhysicalRegister);
      originalAvailableRegisters.erase(
          std::remove(originalAvailableRegisters.begin(),
                      originalAvailableRegisters.end(), newPhysicalRegister),
          originalAvailableRegisters.end());
    }
    return nonAllocatable == 0;
  } else {
    nonAllocatable = 0;
    std::vector<int> usedPhysicalRegisterInGeneCluster;
    for (int i = 0; i < childGenes.size(); i++) {
      childGenes[i].usedPhysicalRegister = usedPhysicalRegisterInGeneCluster;
      childGenes[i].repairConflicts(seed, blocked);
      nonAllocatable += childGenes[i].nonAllocatable;
      if (childGenes[i].hasMutated()) {
        mutation = true;
        usedPhysicalRegisterInGeneCluster.insert(
            usedPhysicalRegisterInGeneCluster.end(),
            childGenes[i].usedPhysicalRegister.begin(),
            childGenes[i].usedPhysicalRegister.end());
      }
    }
    return nonAllocatable == 0;
  }
}

bool EnergyGene::repairCoupledCollision(uint *seed, const ass_reg_t *blocked) {
  bool repaired = true;
  std::vector<int> processedCouples;
  for (int i = 0; i < physicalCollisionIndexCoupled.size(); i++) {

    int collisionIndex = physicalCollisionIndexCoupled[i];
    // check to not check already fixed coupled register
    if (std::find(processedCouples.begin(), processedCouples.end(),
                  collisionIndex) == processedCouples.end()) {

      ass_reg_t *Copyblocked = new ass_reg_t[registers::getNumRegisterFiles()];
      memcpy(Copyblocked, blocked,
             sizeof(ass_reg_t) * registers::getNumRegisterFiles());

      removeCoupledBlockedRegister(Copyblocked, i);
      std::pair<int, int> coupledRegister = getDoubleFreeRegistersPairRandom(
          Copyblocked, virtualRegisters[collisionIndex]->isConcurrent(), seed);
      if (coupledRegister.first == -1 or coupledRegister.second == -1) {
        repaired = false;
      }
      setCoupledRegister(coupledRegister, virtualRegisters[collisionIndex]);

      processedCouples.push_back(collisionIndex);
      auto coupledIt =
          std::find(virtualRegisters.begin(), virtualRegisters.end(),
                    virtualRegisters[collisionIndex]->getCoupled());
      if (coupledIt != virtualRegisters.end()) {
        processedCouples.push_back(
            std::distance(virtualRegisters.begin(), coupledIt));
      }

      delete[] Copyblocked;
    }
  }
  return repaired;
}

void EnergyGene::calculateBlockedPhysicalRegister() {
  blockedPhysicalRegisterMap = std::vector<std::vector<int>>();
  for (int specificVirtualRegister = 0;
       specificVirtualRegister < blockedRegisters.size();
       specificVirtualRegister++) {
    blockedPhysicalRegisterMap.push_back(std::vector<int>());
    for (auto &virtualRegister : blockedRegisters[specificVirtualRegister]) {
      blockedPhysicalRegisterMap[specificVirtualRegister].push_back(
          virtualRegister->getReal());
    }
  }
}

VirtualRegisterMapping *EnergyGene::getFirstCollision() {
  for (int i = 0; i < childGenes.size(); i++) {
    auto result = childGenes[i].getFirstCollision();
    if (result) {
      return result;
    }
  }
  if (physicalCollisionIndex.empty()) {
    return nullptr;
  }
  return virtualRegisters[physicalCollisionIndex[0]];
}

bool EnergyGene::calculateCollisions(bool verbose) {
  if (!childGenes.empty()) {
    throw std::logic_error("Calcualte Collision can only be called if no "
                           "childgenes are avaiable.\n");
  }
  calculateBlockedPhysicalRegister();
  physicalCollisionIndex = std::vector<int>();
  physicalCollisionIndexCoupled = std::vector<int>();

  for (uint i = 0; i < getPhysicalRegister().size(); i++) {
    int physicalRegister = getVirtualRegisters()[i]->getReal();
    if (std::find(blockedPhysicalRegisterMap[i].begin(),
                  blockedPhysicalRegisterMap[i].end(),
                  physicalRegister) != blockedPhysicalRegisterMap[i].end()) {
      physicalCollisionIndex.push_back(i);
      if (verbose) {
        std::cout << "Found Conflict in " << virtualRegisters[i]->toString()
                  << std::endl;
      }
      if (getVirtualRegisters()[i]->isCoupled()) {
        physicalCollisionIndexCoupled.push_back(i);
      }
    }
    if (getVirtualRegisters()[i]->getReal() == -1) {
      physicalCollisionIndex.push_back(i);
    }
    // check coupled Register Constraint
    if (virtualRegisters[i]->isCoupled()) {
      if (not virtualRegisters[i]->isValidCoupling()) {
        physicalCollisionIndexCoupled.push_back(i);
      }
    }
  }
  if (DEBUG_TEXT) {
    if (!physicalCollisionIndex.empty()) {
      std::cout << "Found " << physicalCollisionIndex.size() << " Collisions "
                << std::endl;
      for (int i = 0; i < physicalCollisionIndex.size(); i++) {
        std::cout
            << registers::getName(
                   getVirtualRegisters()[physicalCollisionIndex[i]]
                       ->getVirtual())
            << " --> "
            << registers::getName(
                   getVirtualRegisters()[physicalCollisionIndex[i]]->getReal())
            << std::endl;
      }
    }
  }
  return physicalCollisionIndex.size() > 0;
}

// VirtualRegisterMapping* getFirstCollision(){
//  return vi
//}

int EnergyGene::getRandomPhysicalRegister(std::vector<int> &freeRegister,
                                          uint *seed) {
  return freeRegister[util::uniRandom(0, freeRegister.size() - 1, seed)];
}

std::vector<int> EnergyGene::getFreePhysicalRegister() {
  std::vector<int> originalAvailableRegisters(MAX_REGISTER);
  std::iota(std::begin(originalAvailableRegisters),
            std::end(originalAvailableRegisters), 0);
  // remove slm blocked Register
  for (int v : slmBlockedPhysicalRegister) {
    originalAvailableRegisters.erase(
        std::remove(originalAvailableRegisters.begin(),
                    originalAvailableRegisters.end(), v),
        originalAvailableRegisters.end());
  }
  return originalAvailableRegisters;
}

void EnergyGene::removeBlockedRegister(std::vector<int> &freeRegister,
                                       int index) {
  // remove physical register that live at the same time.
  for (int v : blockedPhysicalRegisterMap[index]) {
    freeRegister.erase(std::remove(freeRegister.begin(), freeRegister.end(), v),
                       freeRegister.end());
    //    std::remove(freeRegister.begin(), freeRegister.end(), v);
  }
}

void EnergyGene::removeCoupledBlockedRegister(ass_reg_t *blockedRegister,
                                              int coupledPhysicalIndex) {

  std::vector<int> temp = blockedPhysicalRegisterMap
      [physicalCollisionIndexCoupled[coupledPhysicalIndex]];
  std::vector<int> list;
  VirtualRegisterMapping *coupledRegister =
      virtualRegisters[physicalCollisionIndexCoupled[coupledPhysicalIndex]]
          ->getCoupled();

  auto itCoupledVirtualRegister = std::distance(
      coupledVirtualRegisterIndex.begin(),
      std::find(coupledVirtualRegisterIndex.begin(),
                coupledVirtualRegisterIndex.end(),
                physicalCollisionIndexCoupled[coupledPhysicalIndex]));
  if (coupledVirtualRegisterDifferentGene[itCoupledVirtualRegister]) {
    // coupled is in different gene
    // find index
    list = coupledVirtualRegisterDifferentGene[itCoupledVirtualRegister]
               ->getBlockedPhysicalRegister(coupledRegister);

  } else {
    list = getBlockedPhysicalRegister(coupledRegister);
  }
  for (auto blockedPhysicalRegister : list) {
    if (temp.end() ==
        std::find(temp.begin(), temp.end(), blockedPhysicalRegister)) {
      temp.push_back(blockedPhysicalRegister);
    }
  }

  for (auto i : temp) {
    addBlockedReg(blockedRegister, i);
  }
}

std::vector<int> EnergyGene::getBlockedPhysicalRegister(
    VirtualRegisterMapping *virtualRegister) {
  for (int i = 0; i < virtualRegisters.size(); i++) {
    if (virtualRegister == virtualRegisters[i]) {
      std::vector<int> temp;
      for (int j = 0; j < blockedRegisters[i].size(); j++) {
        temp.push_back(blockedRegisters[i][j]->getReal());
      }
      return temp;
    }
  }
  throw std::runtime_error("Could not get blocked Physical Register");
}

bool EnergyGene::operator==(const EnergyGene &other) const {
  if (virtualRegisters.size() != other.virtualRegisters.size()) {
    return false;
  }
  for (int i = 0; i < childGenes.size(); i++) {
    if (childGenes[i] != other.childGenes[i]) {
      return false;
    }
  }
  for (int i = 0; i < virtualRegisters.size(); i++) {
    if (!virtualRegisters[i]->valueEqual(other.virtualRegisters[i])) {
      return false;
    }
  }
  return true;
}

void EnergyGene::copyPhysical(const EnergyGene &e) {
  if (e.childGenes.size() > 0 && e.virtualRegisters.size() > 0) {
    throw std::runtime_error(
        "ChildGenes and VirtualRegisters violate Assumption of separation!");
  }
  if (childGenes.empty()) {
    // copy virtualRegister
    for (size_t i = 0; i < e.virtualRegisters.size(); i++) {
      virtualRegisters[i]->setReal(e.virtualRegisters[i]->getReal());
    }

  } else {
    for (int i = 0; i < childGenes.size(); i++) {
      childGenes[i].copyPhysical(e.childGenes[i]);
    }
  }
  nonAllocatable = e.nonAllocatable;
}

COMBINE_RETURN EnergyGene::combine(EnergyGene gene, uint *seed,
                                   const ass_reg_t *blocked) {
  COMBINE_RETURN returnValue;
  mutation = false;
  if (childGenes.empty()) {
    returnValue.virtualRegisterCountMax = getVirtualRegisterCount();
    // if Gene is selected for combine, always combine!
    //      std::cout << " Combination";
    if (DEBUG_TEXT) {
      std::cout << " Combine" << std::endl;
    }
    // copy gene over
    for (uint i = 0; i < virtualRegisters.size(); i++) {
      virtualRegisters[i]->setReal(gene.getVirtualRegisters()[i]->getReal());
      //        std::cout << " " << virtualRegisters[i]->getVirtualName() ;
    }
    //      std::cout << "\n";

    returnValue.combine = true;
    if (checkConflict()) {
      returnValue.conflict = true;
    }
    //      repairConflicts(seed, blocked);

    returnValue.add(layer, maxLayer);
    returnValue.virtualRegisterCount = getVirtualRegisterCount();
    return returnValue;

  } else {
    // copy complete cluster
    if (descendClusterCombine(seed)) {
      // select 1 gene to combine!
      int geneSelectionVar = util::uniRandom(0, childGenes.size() - 1, seed);

      returnValue.virtualRegisterCountMax +=
          childGenes[geneSelectionVar].getVirtualRegisterCount();

      COMBINE_RETURN temp = childGenes[geneSelectionVar].combine(
          gene.childGenes[geneSelectionVar], seed, blocked);
      returnValue.append(temp);

      return returnValue;
    } else {
      // if Gene is selected for combine, always combine!
      returnValue.virtualRegisterCountMax = getVirtualRegisterCount();
      if (DEBUG_TEXT) {
        std::cout << " Combine" << std::endl;
      }

      for (int i = 0; i < childGenes.size(); i++) {
        childGenes[i].copyPhysical(gene.childGenes[i]);
      }

      //        checkConflict();
      //        repairConflicts(seed, blocked);
      returnValue.combine = true;
      if (checkConflict()) {
        returnValue.conflict = true;
      }

      returnValue.add(layer, maxLayer);
      returnValue.virtualRegisterCount = getVirtualRegisterCount();
      return returnValue;
    }
  }
}

bool EnergyGene::hasFoundAllocation() { return nonAllocatable == 0; }

std::string EnergyGene::toString(std::string delimiter, bool simple) const {
  //  std::stringstream ss;
  std::string ss;
  if (childGenes.empty()) {
    ss += delimiter + "[";
    for (int i = 0; i < virtualRegisters.size(); i++) {
      ss += virtualRegisters[i]->toString();
      if (not simple) {
        for (int j = 0; j < blockedRegisters[i].size(); j++) {
          ss += "Blocked by: " + blockedRegisters[i][j]->toString() + ", ";
        }
      }
      //      ss << std::endl;
      //      if(v->isCoupled()){
      //        ss << "<<coupled=" << v->getCoupled()->toString() << ">>";
      //      }
      ss + " , ";
    }
    ss += delimiter + "]\n"; //<< std::endl;
  } else {
    ss += delimiter + "[\n"; //<< std::endl;
    for (auto &gene : childGenes) {
      ss += delimiter + gene.toString(delimiter = delimiter, simple = simple);
    }
    ss += delimiter + "]\n"; // << std::endl;
  }
  return ss;
}

std::vector<int> EnergyGene::getPhysicalRegister() const {
  std::vector<int> tmp;
  if (childGenes.empty()) {
    for (auto virtualRegister : virtualRegisters) {
      tmp.push_back(virtualRegister->getReal());
    }
  } else {
    for (int i = 0; i < childGenes.size(); i++) {
      std::vector<int> localTmp = childGenes[i].getPhysicalRegister();
      tmp.insert(tmp.end(), localTmp.begin(), localTmp.end());
    }
  }
  return tmp;
}

void EnergyGene::addVirtualRegister(
    VirtualRegisterMapping *virtualReg,
    std::vector<VirtualRegisterMapping *> blockedRegister) {
  if (virtualReg->isCoupled()) {
    coupledVirtualRegisterIndex.push_back(virtualRegisters.size());
  }
  virtualRegisters.push_back(virtualReg);
  blockedRegisters.push_back(blockedRegister);
}

EnergyGene *EnergyGene::getGene(VirtualRegisterMapping *virtualRegister) {
  if (childGenes.empty()) {
    for (int i = 0; i < virtualRegisters.size(); i++) {
      if (virtualRegisters[i] == virtualRegister) {
        return this;
      }
    }
    return nullptr;
  } else {
    for (int i = 0; i < childGenes.size(); i++) {
      auto energyGene = childGenes[i].getGene(virtualRegister);
      if (energyGene) {
        return energyGene;
      }
    }
    return nullptr;
  }
}

std::vector<VirtualRegisterMapping *> EnergyGene::getCoupledRegisters() {
  std::vector<VirtualRegisterMapping *> temp;
  if (childGenes.empty()) {
    for (auto index : coupledVirtualRegisterIndex) {
      temp.push_back(virtualRegisters[index]);
    }
    return temp;
  } else {
    for (int i = 0; i < childGenes.size(); i++) {
      auto t = childGenes[i].getCoupledRegisters();
      for (VirtualRegisterMapping *virtualRegs : t) {
        temp.push_back(virtualRegs);
      }
    }
    return temp;
  }
}

void EnergyGene::deserialize(std::unordered_map<int, int> virtual2Physical) {
  if (childGenes.empty()) {

    for (int i = 0; i < virtualRegisters.size(); i++) {
      //        std::cout << virtualRegisters[i]->virt ;
      virtualRegisters[i]->setReal(
          virtual2Physical[virtualRegisters[i]->getVirtual()]);
      //        std::cout  << " " << virtualRegisters[i]->real << std::endl;
    }
  } else {
    for (int i = 0; i < childGenes.size(); i++) {
      childGenes[i].deserialize(virtual2Physical);
    }
  }
}

bool EnergyGene::addCoupledEnergyGene(VirtualRegisterMapping *virtualRegister,
                                      EnergyGene *energyGeneCoupled) {
  if (childGenes.empty()) {
    // initialize coupledVirtualRegisterDifferentGenes, if not already happened
    if (coupledVirtualRegisterDifferentGene.empty()) {
      for (int i = 0; i < coupledVirtualRegisterIndex.size(); i++) {
        coupledVirtualRegisterDifferentGene.push_back(nullptr);
      }
    }

    for (int i = 0; i < coupledVirtualRegisterIndex.size(); i++) {
      if (virtualRegister == virtualRegisters[coupledVirtualRegisterIndex[i]]) {
        // add only different genes to coupled Virtual Register
        if (this != energyGeneCoupled) {
          coupledVirtualRegisterDifferentGene[i] = energyGeneCoupled;
        }
        return true;
      }
    }
    return false;
  } else {
    for (auto &gene : childGenes) {
      bool success =
          gene.addCoupledEnergyGene(virtualRegister, energyGeneCoupled);
      if (success) {
        return true;
      }
    }
    return false;
  }
}

bool EnergyGene::checkGene() {
  if (childGenes.size() > 0 and virtualRegisters.size() > 0) {
    LOG_OUTPUT(
        LOG_M_ALWAYS,
        "Cluster and Genes exist at the same time. This should not exist.");
    return false;
  }

  if (childGenes.empty()) {
    // collect all coupled registers
    std::vector<VirtualRegisterMapping *> totalCoupledRegisters;
    for (int i = 0; i < virtualRegisters.size(); i++) {
      if (virtualRegisters[i]->isCoupled()) {
        totalCoupledRegisters.push_back(virtualRegisters[i]);
        totalCoupledRegisters.push_back(virtualRegisters[i]->getCoupled());
      }
    }
    // check if all coupled register are in virtualRegisters
    bool noError = true;
    for (int i = 0; i < totalCoupledRegisters.size(); i++) {
      if (std::find(virtualRegisters.begin(), virtualRegisters.end(),
                    totalCoupledRegisters[i]) == virtualRegisters.end()) {
        noError = false;
      }
    }
    return noError;
  } else {
    bool noError = true;
    for (int i = 0; i < childGenes.size(); i++) {
      if (not childGenes[i].checkGene()) {
        noError = false;
      }
    }
    return noError;
  }
}

} // namespace portOptReg