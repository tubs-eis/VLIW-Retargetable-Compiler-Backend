// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_ENERGYGENE_H
#define SCHEDULER_ENERGYGENE_H

#include "CombineReturn.h"
#include <Constants.h>
#include <algorithm>
#include <iostream>
#include <memory>
#include <numeric>
#include <sstream>
#include <utility.h>
#include <vector>
#include <virtual_reg.h>
namespace portOptReg {
/*
 * EnergyGene either has ChildGenes (because it consists of a cluster) or it
 * only has Virtualregister.
 * */
class EnergyGene {
private:
  /***
   * VirtualRegister Reference. Owner is EnergyChromosome of the
   * VirtualRegisterMappings.
   */
  std::vector<VirtualRegisterMapping *> virtualRegisters;

  /**
   * Indicate Index of VirtualRegisters that are coupled.
   */
  std::vector<int> coupledVirtualRegisterIndex;

  std::vector<EnergyGene *> coupledVirtualRegisterDifferentGene;
  /***
   * For each virtualRegister, map Virtualregister that live at the same time.
   */
  std::vector<std::vector<VirtualRegisterMapping *>> blockedRegisters;

  std::vector<EnergyGene> childGenes;

  /**
   * Every Coupled VirtualRegister creates a read & write port allocation
   * difficulty. Here are all the read constraints.
   */
  std::vector<std::vector<VirtualRegisterMapping *>>
      coupledReadAllocationConstraints;

  /**
   * Every Coupled VirtualRegister creates a read & write port allocation
   * difficulty. Here are all the write constraints.
   */
  std::vector<std::vector<VirtualRegisterMapping *>>
      coupledWriteAllocationConstraints;

  /**
   * permanant data structure
   */
  std::vector<uint> slmBlockedPhysicalRegister;

  /**
   * temp data structure
   * Keeps track of physical Register Conflicts for a specific Gene.
   */
  std::vector<int> physicalCollisionIndex;

  /**
   * temp data sturcture
   * Keeps track of coupled Register collisions.
   */
  std::vector<int> physicalCollisionIndexCoupled;

  /**
   * temp data structure
   * keep track of blocked Physical Registers.
   * VirtualRegister < Vector<Blocked Physical Register>>
   */
  std::vector<std::vector<int>> blockedPhysicalRegisterMap;

  bool mutation = false;

  void removeBlockedRegister(std::vector<int> &freeRegister, int index);
  /***
   * Remove coupled Register at coupledIndex (both registers)
   * @param blockedRegister legacy datastructure for maintaining used register
   * @param coupledIndexVirtualRegister index of blocked coupled register
   */
  void removeCoupledBlockedRegister(ass_reg_t *blockedRegister,
                                    int coupledIndexVirtualRegister);
  int getRandomPhysicalRegister(std::vector<int> &freeRegister, uint *seed);

  /***
   * Retrieve the blocked physical register for each gene.
   *
   * Iterates over all blocked Register and extracts the physical registers.
   */
  void calculateBlockedPhysicalRegister();

  /***
   * If collision happened on coupled registers, resolve the collision.
   * @param seed
   * @return
   */
  bool repairCoupledCollision(uint *seed, const ass_reg_t *blocked);

  /**
   *
   * @param allocatedCoupledPhysicalRegisters physical register to set.
   * @param coupledVirtualRegister Set physical register in coupled Register
   */
  void
  setCoupledRegister(std::pair<int, int> &allocatedCoupledPhysicalRegisters,
                     VirtualRegisterMapping *coupledVirtualRegister);

  int layer;
  int maxLayer;

  int getVirtualRegisterCount() {
    if (childGenes.empty()) {
      return virtualRegisters.size();
    } else {
      int registerCount = 0;
      for (int i = 0; i < childGenes.size(); i++) {
        registerCount += childGenes[i].getVirtualRegisterCount();
      }
      return registerCount;
    }
  }

public:
  int nonAllocatable;

  /***
   * Calculate If a physical register is allocated twice and therefore causes an
   * allocation error.
   * WARNING: Only works if no childGenes are present!
   */
  bool calculateCollisions(bool verbose = false);

  VirtualRegisterMapping *getFirstCollision();

  /***
   * Check for collisions. If they occur, resolve them.
   * @param seed Random seed number for randomly choosing new physical register
   * to resolve collisions.
   * @return True if collision could be resolved.
   */
  bool repairConflicts(uint *seed, const ass_reg_t *blocked);

  /***
   *
   * @return True if conflict detected.
   */
  bool checkConflict(bool verbose = false);
  std::vector<int> usedPhysicalRegister;

  EnergyGene() : nonAllocatable(-1) {}

  /**
   *
   * @param virtualReg add to EnergyGene as standalone VirtualRegister
   * @param blockedRegister add Blocked register of  virtualReg
   */
  void
  addVirtualRegister(VirtualRegisterMapping *virtualReg,
                     std::vector<VirtualRegisterMapping *> blockedRegister);

  void addChildGene(EnergyGene gene) { childGenes.push_back(gene); }

  void addBlockedVirtualRegister(std::vector<uint> blockeRegister) {
    slmBlockedPhysicalRegister = blockeRegister;
  }

  bool empty() { return virtualRegisters.size() == 0; }

  std::vector<VirtualRegisterMapping *> getVirtualRegisters() {
    return virtualRegisters;
  }

  VirtualRegisterMapping *getVirtualRegister(std::string virtualRegisterName) {
    for (int i = 0; i < childGenes.size(); i++) {
      auto regs = childGenes[i].getVirtualRegister(virtualRegisterName);
      if (regs) {
        return regs;
      }
    }
    for (int i = 0; i < virtualRegisters.size(); i++) {
      if (virtualRegisterName == virtualRegisters[i]->getVirtualName()) {
        return virtualRegisters[i];
      }
    }
    return nullptr;
  }

  std::vector<VirtualRegisterMapping *>
  getBlockedRegister(std::string virtualRegisterName) {
    for (int i = 0; i < childGenes.size(); i++) {
      auto regs = childGenes[i].getBlockedRegister(virtualRegisterName);
      if (regs.size() > 0) {
        return regs;
      }
    }
    for (int i = 0; i < virtualRegisters.size(); i++) {
      if (virtualRegisterName == virtualRegisters[i]->getVirtualName()) {
        return blockedRegisters[i];
      }
    }
    return std::vector<VirtualRegisterMapping *>();
  }

  bool hasMutated() const;

  void collectVirtualRegisterMap(VirtualRegisterMap *virtualRegisterMap);

  bool descendClusterCombine(uint *seed) {
    return util::uniRandom(seed) < params.raRegHierarchyDescend;
  }

  bool combineOther(uint *seed) { return util::uniRandom(seed) < .5; }

  /***
   *
   * @param gene
   * @param seed
   * @param blocked need for coupled register repairGeneGreedy mechanism
   * @return Did combination occur? , if descended to hierarchy: return
   * hierarchylevel + max
   */
  COMBINE_RETURN combine(EnergyGene gene, uint *seed, const ass_reg_t *blocked);

  bool hasFoundAllocation();

  int getNonAllocatable() const { return nonAllocatable; }

  void randomize(uint *seed, const ass_reg_t *blocked);

  std::string toString(std::string delimiter = " ", bool simple = false) const;

  std::vector<int> getPhysicalRegister() const;

  void addAllocationConstraints(
      std::vector<VirtualRegisterMapping *> readConstraints,
      std::vector<VirtualRegisterMapping *> writeConstraints) {
    coupledReadAllocationConstraints.push_back(readConstraints);
    coupledWriteAllocationConstraints.push_back(writeConstraints);
    if (coupledReadAllocationConstraints.size() !=
        coupledVirtualRegisterIndex.size()) {
      throw std::logic_error("Coupled Constraints are not applied properly.");
    }
  }

  /**
   * Copy Physical virtualRegisters and BlockedRegister for a "deepcopy".
   * @param e
   */
  void copyPhysical(const EnergyGene &e);

  bool operator==(const EnergyGene &other) const;

  bool operator!=(const EnergyGene &other) const {
    return not operator==(other);
  }

  int size() const { return virtualRegisters.size(); }

  /**
   *
   * @param virtualRegister  VirtualRegister to search
   * @return Get EnergyGene which contains virtualRegister.
   */
  EnergyGene *getGene(VirtualRegisterMapping *virtualRegister);

  std::vector<VirtualRegisterMapping *> getCoupledRegisters();

  /**
   * Create Mapping of Energygene which contains virtualRegisters couple.
   * @param virtualRegister virtualRegister, is a coupled register.
   * @param energyGeneCoupled Energygene which contains the couple of the
   * virtualRegister.
   * @return
   */
  bool addCoupledEnergyGene(VirtualRegisterMapping *virtualRegister,
                            EnergyGene *energyGeneCoupled);

  /**
   *  Copy physical register values from argument.
   * @param virtual2Physical Map of Virtual to Physical Register
   */
  void deserialize(std::unordered_map<int, int> virtual2Physical);

  std::vector<int>
  getBlockedPhysicalRegister(VirtualRegisterMapping *virtualRegister);

  std::vector<int> getCoupledVirtualRegisterIndex() const {
    return coupledVirtualRegisterIndex;
  }
  const std::vector<EnergyGene *>
  getCoupledVirtualRegisterDifferentGene() const {
    return coupledVirtualRegisterDifferentGene;
  }
  /***
   *
   * @return list of non-blocked physical Register, which are not used across
   * BBs/SLMs
   */
  std::vector<int> getFreePhysicalRegister();

  void setLayer(int layer) { this->layer = layer; };
  void setMaxLayer(int maxLayer) { this->maxLayer = maxLayer; }

  void setVirtualRegisterMap(VirtualRegisterMap *map) {
    for (int i = 0; i < childGenes.size(); i++) {
      childGenes[i].setVirtualRegisterMap(map);
    }
    bool notEntered = false;
    for (int i = 0; i < virtualRegisters.size(); i++) {
      auto virtualReg = virtualRegisters[i]->getVirtual();
      // legacy data Structure: a physical register does not have a mapping
      // EnergyGene: a physical register mapps to itself.
      // to harmonize both datastructures, Energygene physical register should
      // not be overwritten (with invalid mapping from Energygene perspective).
      if (registers::isVirtualReg(virtualReg)) {
        auto mapIT = map->find(virtualReg);
        if (mapIT != map->end()) { //   virtualRegisterName ==
                                   //   virtualRegisters[i]->getVirtualName()){
          virtualRegisters[i]->setReal(mapIT->second->getReal());
        } else {
          notEntered = true;
        }
        if (notEntered) {
          throw std::runtime_error("A Virtual Register was not set!");
        }
      }
    }
  }

  /***
   * Sanity Check of Gene.
   * @return True if Gene passes basic sanity check.
   */
  bool checkGene();

  std::vector<uint> getSlmBlockedPhysicalRegister() {
    return slmBlockedPhysicalRegister;
  }
};
} // namespace portOptReg

#endif // SCHEDULER_ENERGYGENE_H
