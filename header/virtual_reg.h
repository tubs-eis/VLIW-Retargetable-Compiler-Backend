// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#ifndef VIRTUAL_REG_H
#define VIRTUAL_REG_H

class Processor;
class MI;
class SLM;
#include "Constants.h"
#include "ga_stats.h"
#include "register.h"
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdlib.h>
#include <unordered_map>
#include <vector>

namespace rdg {
class RDG;
}

class Context;
class Program;

/** \brief struct for mapping a virtual to a real register.
 *
 * Also stores information about the usage of the virtual register in the SLM
 * (first and last occurrence)
 *
 */
struct VirtualRegisterMapping {
private:
  std::string virtualRegisterName;
  std::string realRegisterName;
  VirtualRegisterMapping *coupled;

  /** the virtual register */
  char32_t virt;
  /** name of the real register */
  char32_t real;
  /** link to the first occurrence in the code */
  std::vector<MI *>::iterator first_occurrence;
  /** link to the last occurrence in the code */
  std::vector<MI *>::iterator last_occurrence;
  /** this is a helper for the genetic algorithm, to not have extra structures.
   * It indicates in which register File the real register should be
   * allocated.*/
  int regFile;

  bool coupledConcurrent;

  std::vector<std::shared_ptr<VirtualRegisterMapping>> readAllocationConstraint;
  std::vector<std::shared_ptr<VirtualRegisterMapping>>
      writeAllocationConstraint;

public:
  VirtualRegisterMapping()
      : virt(-1), real(-1), coupled(nullptr), coupledConcurrent(false),
        virtualRegisterName(""), realRegisterName("") {}

  ~VirtualRegisterMapping() {}

  VirtualRegisterMapping(char32_t virt)
      : virt(virt), real(-1), coupled(nullptr), coupledConcurrent(false),
        virtualRegisterName(registers::getName(virt)), realRegisterName("") {
    virtualRegisterName = registers::getName(virt);
    if (!registers::isVirtualReg(virt)) {
      real = virt;
      realRegisterName = registers::getName(real);
    }
  }

  VirtualRegisterMapping(VirtualRegisterMapping *old)
      : virt(old->virt), real(old->real),
        first_occurrence(old->first_occurrence),
        last_occurrence(old->last_occurrence), regFile(old->regFile),
        coupled(old->coupled), coupledConcurrent(old->coupledConcurrent) {
    virtualRegisterName = registers::getName(virt);
    realRegisterName = registers::getName(real);
  }

  VirtualRegisterMapping *getCoupled() { return coupled; }

  bool isVirtualReg() const { return registers::isVirtualReg(virt); }

  /***
   *
   * @return Real Register already Allocated
   */
  bool isAllocated() const { return real != (char32_t)-1; }

  void addReadAllocationConstraint(
      std::shared_ptr<VirtualRegisterMapping> readConstraint) {
    readAllocationConstraint.push_back(readConstraint);
  }

  void addWriteAllocationConstraint(
      std::shared_ptr<VirtualRegisterMapping> writeConstraint) {
    writeAllocationConstraint.push_back(writeConstraint);
  }

  const std::vector<std::shared_ptr<VirtualRegisterMapping>>
  getWriteAllocationConstraint() const {
    return writeAllocationConstraint;
  }

  const std::vector<std::shared_ptr<VirtualRegisterMapping>>
  getReadAllocationConstraint() const {
    return readAllocationConstraint;
  }

  void setVirtual(char32_t virtualValue) {
    virt = virtualValue;
    virtualRegisterName = registers::getName(virt);
  }

  void setReal(char32_t realVal) {
    if (registers::isPhysicalRegister(virt)) {
      real = virt;
      realRegisterName = registers::getName(real);
      //      if( realVal != virt){
      //        std::stringstream ss ;
      //        ss << "allocate physical: " << registers::getName(virt) << " ->
      //        " << registers::getName(realVal) << std::endl;
      //          throw std::logic_error(ss.str());
      //      }
    } else {
      real = realVal;
    }
    realRegisterName = registers::getName(real);
  }

  void setCoupledRegister(char32_t real, char32_t coupledReal) {
    this->real = real;
    realRegisterName = registers::getName(real);
    coupled->setReal(coupledReal);
    isValidCoupling();
    if (virt < coupled->virt) {
      if (real + 1 != coupledReal) {
        if (real != -1 or coupledReal != -1) {
          throw std::runtime_error("Coupled Register are not set properly!");
        }
      }
    } else if (virt > coupled->virt) {
      if (real != coupledReal + 1) {
        if (real != -1 or coupledReal != -1) {
          throw std::runtime_error("Coupled Register are not set properly!");
        }
      }
    } else {
      throw std::runtime_error(
          "Setting Coupled Register to not Coupled register!");
    }
  }

  void setRegFile(int file) { regFile = file; }

  int getRegFile() const { return regFile; }

  char32_t getReal() const { return real; }
  char32_t getVirtual() const { return virt; }

  std::string getRealName() const { return realRegisterName; }

  std::string getVirtualName() const { return virtualRegisterName; }

  void setFirstOccurrence(std::vector<MI *>::iterator first) {
    first_occurrence = first;
  }
  void setLastOccurrence(std::vector<MI *>::iterator last) {
    last_occurrence = last;
  }

  std::vector<MI *>::iterator getFirstOccurrence() { return first_occurrence; }
  std::vector<MI *>::iterator getLastOccurrence() { return last_occurrence; }

  bool isCoupled() { return coupled; }

  bool isConcurrent() const { return coupledConcurrent; }

  bool isBlockedRegister(const unsigned long long *blocked) {
    uint i = registers::getRegFile(real);
    ass_reg_t temp = 1;
    return blocked[i] & (temp << registers::getRegNumber(real));
  }

  void addCoupledVirtualRegisterMapping(VirtualRegisterMapping *couple,
                                        bool concurrent) {
    coupled = couple;
    coupledConcurrent = concurrent;
  }

  bool isFreeRegister(const unsigned long long *blocked) {
    return !isBlockedRegister(blocked);
  }

  std::string toString() {
    std::string ss;
    ss += virtualRegisterName + "->" + realRegisterName;
    return ss;
  }

  bool valueEqual(const VirtualRegisterMapping *mapping) const {
    return (virt == mapping->virt) and (real == mapping->real);
  }

  bool isValidCoupling() {
    if (coupled) {
      if (virt < coupled->virt) {
        if (real + 1 != coupled->real) {
          return false; // throw std::runtime_error("Coupled Register are not
                        // set properly!");
        }
      } else if (virt > coupled->virt) {
        if (coupled->real + 1 != real) {
          return false; // throw std::runtime_error("Coupled Register are not
                        // set properly!");
        }
      } else {
        throw std::runtime_error(
            "Setting Coupled Register to not Coupled register!");
      }
      return true;
    } else {
      return false;
    }
  }
};

/** \brief Allocation of virtual to real registers. */
typedef std::unordered_map<char32_t, char32_t> VirtualAllocation;

/** \brief Struct for coupled registers.
 *
 * A register is coupled, if it has an position relation with another one. This
 * happens for MAC operations and all X2 Operations.
 */
struct RegisterCouple {
  /* The first registers */
  char32_t first;
  /* The second register */
  char32_t second;
  /* \brief Indicator if concurrent or parallel */
  bool concurrent;
};

/**
 * \brief Map between Virtual Register and virtualRegisterMapping Structure.
 *
 */
typedef std::unordered_map<char32_t, VirtualRegisterMapping *>
    VirtualRegisterMap;

void resetVirtualRegisterMap(VirtualRegisterMap *map, const int *regFile);

void printVirtualRegisterMap(VirtualRegisterMap *map);

/** \brief Map with all coupled registers.
 *
 * A register is coupled, if it has to be concurrent with another one. this
 * happens for MAC operations and all X2 Operations. The map contains all ID's
 * for coupled registers and points to a struct, where one can read if it is the
 * first or second part.
 */
typedef std::unordered_map<char32_t, RegisterCouple *> RegisterCoupling;

VirtualRegisterMap *fromVirtualAlloc(VirtualAllocation *alloc);
void releaseVirtualMap(VirtualRegisterMap **map);

void printRegisterCouplings(const RegisterCoupling &couplings);

extern std::unordered_map<int, ass_reg_t *> blockedRegsInSLM;
/** @brief A function to initialize the mapping for the virtual registers.
 *
 * this has to be called before an actual mapping takes place and with all the
 * SLM's in their order natural order.
 * @param slm The SLM in their order
 * @param pro This is just for compabilitie and ignored.
 */
void virtual_init(SLM *slm, Processor *pro);
/**
 *
 * @brief Test if a mapping of virtual registers is possible for the given List
 * of MIs.
 *
 * By default try a heuristic register allocation, if this does not work
 * (or is not enabled), do a genentic register allocation.
 *
 * the List of MI is build by going the tree up from the last MI and then using
 * the same way down.
 * @param slm A SLM as reference to know where we are in the Code.
 * @param pro A Processor, where the Code has to run on.
 * @param ins The List of MI.
 * @return true If it possible to allocate the virtual registers.
 */
int virtual_test(SLM *slm, Processor *pro, Program *ins,
                 VirtualRegisterMap **map, RegisterCoupling *coupling,
                 const Context &ctx);

/** @brief Performs the actual scheduling of the virtual registers.
 *
 * After this Operation, there are no virtual registers in the path of the
 * shortest instruction. All virtual registers are replaced by a real
 * representation.
 *
 * @param slm The SLM to schedule.
 * @param pro A Processor, where the SLM is executed on.
 * @return true on success
 * @return false if it is not possible, or there has been an error.
 */
bool virtual_schedule(SLM *slm, Processor *pro);

/** @brief Does a renaming of virtual registers, every time they are written to.
 *
 * If a virtual register is written to, it can be renamed, this is performed
 * here.
 *
 * @param slm The SLM with a List of MO
 * @param pro This is just for compabilitie and ignored.
 */
void virtualRenaming(SLM *slm, Processor *pro);

/** \brief Creates a dot representation of the virtual registers.
 *
 * This function is no longer used. But it was designed to give an overview of
 * the virtual registers and how they match together as a helper for designing a
 * spilling algorithm. The result is written to a file called "RegGraph" and can
 * be visualized using dot.
 */
void DEPRECATED(virtualRegisterGraph(SLM *slm, Processor *pro));

char32_t getDoubleFirstFreeReg(int *freeReg, ass_reg_t *blockedFile,
                               int *usedWritePorts, char dir, bool concurrent,
                               bool usedummy, int preferredFile = -1);
char32_t getFreeReg(int *freeReg, ass_reg_t *blockedFile, int *usedWritePorts,
                    int regFile = -1, bool usedummy = false);
/**
 * Checks if there is a valid virtual Mapping and the MIs are executable.
 * @param map
 * @param regMapping
 * @param pro
 * @param start
 * @param end
 * @return true, if all MIs have valid mappings and are executable.
 */
bool checkMapping(VirtualRegisterMap *map, VirtualAllocation *regMapping,
                  const Processor *pro,
                  const std::vector<MI *>::const_iterator start,
                  const std::vector<MI *>::const_iterator end);
void updatePosition(VirtualRegisterMap **map, char32_t reg, int arg,
                    std::vector<MI *>::iterator it);
VirtualRegisterMapping *getMapping(const VirtualRegisterMap *map,
                                   char32_t virt);

/**
 * Virtually allocate Register.
 * @param blocked
 * @param pro
 * @param ins
 * @param map
 * @param couplings
 * @param freeReg
 * @param ctx
 * @param slm
 * @return The number of non allocatable Register. Abort for special reasons is
 * #NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET
 */
uint allocate_virtual(ass_reg_t *blocked, const Processor *pro,
                      const Program *ins, VirtualRegisterMap *map,
                      const RegisterCoupling *couplings, int *freeReg,
                      const Context &ctx, SLM *slm);

/**
 *
 * @param slm
 * @param pro
 * @param ins
 * @param couplings
 * @param map
 * @param rdg
 * @param ctx
 * @return Return the number of non allocatable Register.
 */
int heuristicRegisterAllocation(SLM *slm, Processor *pro, Program *ins,
                                RegisterCoupling *couplings,
                                VirtualRegisterMap *map, rdg::RDG **rdg,
                                const Context &ctx);
/**
 *
 * @param slm
 * @param pro
 * @param ins
 * @param couplings
 * @param map
 * @param rdg
 * @param ctx
 * @return defaults to -1, if nothing is done.
 */
int geneticRegisterAllocation(SLM *slm, Processor *pro, Program *ins,
                              RegisterCoupling *couplings,
                              VirtualRegisterMap *map, rdg::RDG **rdg,
                              const Context &ctx,
                              VirtualRegisterMap *heuristicMap = nullptr);

/**
 *
 * @param slm
 * @param pro
 * @param ins
 * @param couplings
 * @param map
 * @param rdg
 * @param ctx
 * @return defaults to -1, if nothing is done.
 */
ga_stats::ChromosomeFitness geneticRegisterAllocationPower(
    SLM *slm, Processor *pro, Program *ins, RegisterCoupling *couplings,
    VirtualRegisterMap *map, rdg::RDG **rdg, const Context &ctx,
    VirtualRegisterMap *heuristicMap = nullptr);

void printBlockedRegisters(const ass_reg_t *blocked, int *freeReg = NULL);

std::vector<uint> getBlockedRegisters(const ass_reg_t *blocked);

char32_t getReal(const VirtualRegisterMap *map,
                 const VirtualAllocation *regMapping, char32_t virt);

void calcFreeReg(const ass_reg_t *blocked, int *freeReg);

void addBlockedReg(ass_reg_t *blocked, int newlyBlocked);
void removeBlockedReg(ass_reg_t *blocked, int newlyBlocked);

void calculateCouplings(Program *ins, VirtualRegisterMap *map,
                        std::set<char32_t> *freeing,
                        RegisterCoupling &couplings, Processor *pro);
void deleteCouplings(RegisterCoupling &couplings);
void storeCoupling(RegisterCoupling &couplings, char32_t reg1, char32_t reg2);
void storeCouplings(RegisterCoupling &couplings, MO *x2Op);

#endif // VIRTUAL_REG_H
