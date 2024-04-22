// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "virtual_reg.h"
#include "CompilerContext.h"
#include "SLM.h"
#include "functionalunit.h"
#include "gen_merging.h"
#include "gen_register.h"
#include "global.h"
#include "label.h"
#include "portOptimalReg.h"
#include "processor.h"
#include "rdg.h"
#include "register.h"
#include <fstream>
#include <iostream>
#include <stack>
#include <unordered_map>
#include <unordered_set>

ass_reg_t *blocked = nullptr;

unordered_map<int, ass_reg_t *> blockedRegsInSLM;

#define REG_OFFSET                                                             \
  ((registers::getNumRegisterFiles() + 1) * registers::getNumRegisters())

VirtualRegisterMap *fromVirtualAlloc(VirtualAllocation *alloc) {
  auto *map = new VirtualRegisterMap();
  for (auto &it : *alloc) {
    char32_t virt = it.first;
    char32_t real = it.second;

    if (map->find(virt) == map->end()) {
      auto *mapping = new VirtualRegisterMapping();
      mapping->setVirtual(virt);
      mapping->setReal(real);
      map->insert(
          std::pair<const char32_t, VirtualRegisterMapping *>(virt, mapping));
    }
  }
  return map;
}

void releaseVirtualMap(VirtualRegisterMap **map) {
  for (auto it = (*map)->begin(); it != (*map)->end(); ++it) {
    if (it->second)
      delete it->second;
  }
  delete *map;
  *map = nullptr;
}

void checkBlocked(MO *m, ass_reg_t *b) {
  char32_t *args = m->getArguments();
  OPtype *types = m->getTypes();
  for (int e = 0; e < m->getArgNumber(); ++e) {
    if (types[e] == REG) {
      int file = registers::getRegFile(args[e]);
      if (file == -1)
        continue;
      int number = registers::getRegNumber(args[e]);
      if (number == -1)
        continue;
      ass_reg_t temp = 1;
      b[file] |= (temp << number);
    }
  }
}

void printRegisterCouplings(const RegisterCoupling &couplings) {
  auto it = couplings.begin();
  LOG_OUTPUT(LOG_M_ALWAYS, "Register _couplings:\n");
  int c = 0;
  while (it != couplings.end()) {
    if (c++ == 0)
      LOG_OUTPUT(LOG_M_ALWAYS, "    ");
    const RegisterCouple *couple = it++->second;
    int r1 = registers::getVirtualRegisterNumber(couple->first);
    int r2 = registers::getVirtualRegisterNumber(couple->second);
    LOG_OUTPUT(LOG_M_ALWAYS, "(%d - %d) ", r1, r2);
    if (c == 10) {
      LOG_OUTPUT(LOG_M_ALWAYS, "\n");
      c = 0;
    }
  }
  LOG_OUTPUT(LOG_M_ALWAYS, "\n");
}

void rec_print(std::vector<MO *> *prev, char *regFile,
               std::set<std::string> &output) {
  for (auto mo : *prev) {
    char32_t *args_depend = mo->getArguments();
    OPtype *types_depend = mo->getTypes();
    char *dir_depend = mo->getDirections();
    for (int e = 0, end = mo->getArgNumber(); e < end; ++e) {
      if ((dir_depend[e] & WRITE) && types_depend[e] == REG &&
          registers::getVirtualRegisterNumber(args_depend[e]) >= 0) {
        char buffer[512];
        sprintf(buffer, "%s -> %s [style=\"dotted\"]",
                registers::getName(args_depend[e]).c_str(), regFile);
        std::string foo(buffer);
        output.insert(foo);
        goto whatever;
      }
    }
    rec_print(mo->getPrevious(), regFile, output);
  whatever:;
  }
}

void virtualRegisterGraph(SLM *slm, Processor *pro) {
  slm->calculateGraph(pro, false);
  std::set<std::string> output;
  std::ofstream out;

  //    char buffer[64];
  //    sprintf(buffer,"RegGraph.dot",slm->getID());
  out.open("RegGraph", std::ofstream::out | std::ofstream::app);
  out << endl << "digraph " << slm->getID() << "{" << endl;
  std::vector<MO *> *ops = slm->getOperations();
  char buffer[64];
  for (auto &op : *ops) {
    char32_t *args_depend = op->getArguments();
    OPtype *types_depend = op->getTypes();
    char *dir_depend = op->getDirections();
    for (int e = 0, end = op->getArgNumber(); e < end; ++e) {
      if ((dir_depend[e] & WRITE) && types_depend[e] == REG &&
          registers::getVirtualRegisterNumber(args_depend[e]) >= 0) {
        strcpy(buffer, registers::getName(args_depend[e]).c_str());
        for (int i = 0; i < end; i++) {
          if ((dir_depend[i] & READ) && types_depend[i] == REG &&
              registers::getVirtualRegisterNumber(args_depend[i]) >= 0) {
            out << registers::getName(args_depend[i]);
            out << " -> " << buffer << ";" << endl;
          }
        }
        rec_print(op->getPrevious(), buffer, output);
      }
    }
  }
  for (const auto &it : output) {
    out << it << endl;
  }
  out << "}" << endl;
  out.close();
}

void virtual_init(SLM *slm, Processor *pro) {
  // initialization of all blocked registers. (only done once)
  if (blocked) {
    free(blocked);
    blocked = nullptr;
    blockedRegsInSLM.clear();
  }
  if (blocked == nullptr) {
    blocked = (ass_reg_t *)malloc(sizeof(ass_reg_t) *
                                  registers::getNumRegisterFiles());
    memset(blocked, 0, sizeof(ass_reg_t) * registers::getNumRegisterFiles());
  }

  // Manually fixed registers (FIXREG) are blocked here
  for (char32_t it : *slm->getFixReg()) {
    int regFile = registers::getRegFile(it);
    if (regFile == -1)
      continue;
    int number = registers::getRegNumber(it);
    if (number == -1)
      continue;
    ass_reg_t temp = 1;
    blocked[regFile] |= (temp << number);
  }

  // iterate over all the Operations and find the virtual registers, which they
  // use to block them for use with virtual registers.
  vector<MO *> *ops = slm->getOperations();
  for (auto &op : *ops) {
    checkBlocked(op, blocked);
  }
  // / also do this for branch operations.
  MO *branch = slm->getBranchOperation();
  if (branch != nullptr) {
    checkBlocked(branch, blocked);
  }
  // then make an individual copy of the blocked registers for this SLM.
  ass_reg_t *forThis =
      (ass_reg_t *)malloc(sizeof(ass_reg_t) * registers::getNumRegisterFiles());
  memcpy(forThis, blocked,
         sizeof(ass_reg_t) * registers::getNumRegisterFiles());
  blockedRegsInSLM[slm->getID()] = forThis;

  // debug output.
  LOG_OUTPUT(LOG_M_RA_DETAIL, "SLM Nr. %3d blocked registers: ", slm->getID());
  for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
    ass_reg_t reg = blocked[i];
    LOG_OUTPUT(LOG_M_RA_DETAIL, " ");
    for (uint x = 0; x < registers::getNumRegister1RF(); x++) {
      LOG_OUTPUT(LOG_M_RA_DETAIL, "%d", (int)(1 & reg >> x));
    }
  }
  LOG_OUTPUT(LOG_M_RA_DETAIL, "\n");

  // remove the blocking for the next slm.
  // all registers which had been free's using the FREEREG command are now made
  // available for the next SLM.
  for (char32_t it : *slm->getFreeReg()) {
    int file = registers::getRegFile(it);
    if (file == -1)
      continue;
    int number = registers::getRegNumber(it);
    if (number == -1)
      continue;
    ass_reg_t temp = 1;
    blocked[file] &= ~(temp << number);
  }
}

int getDoubleFirstFreeConcurrent(ass_reg_t regBlock, bool usedummy) {
  unsigned int start, end;
  if (usedummy) {
    start = 0;
    end = registers::getLastDummyRegNumber() + 1;
  } else {
    start = 0;
    end = registers::getNumRegister1RF() - 1;
  }

  ass_reg_t temp = 1;
  for (unsigned int i = start; i < end; i += 2)
    if (!(regBlock & (temp << i)) && !(regBlock & (temp << (i + 1))))
      return i;
  return -1;
}

int getDoubleFirstFreeParallel(ass_reg_t regBlock1, ass_reg_t regBlock2,
                               bool usedummy) {
  unsigned int start, end;
  if (usedummy) {
    start = 0;
    end = registers::getLastDummyRegNumber() + 1;
  } else {
    start = registers::getLastDummyRegNumber() + 1;
    end = registers::getNumRegister1RF();
  }

  ass_reg_t temp = 1;
  for (unsigned int i = start; i < end; i++)
    if (!(regBlock1 & (temp << i)) && !(regBlock2 & (temp << i)))
      return i;
  return -1;
}

char32_t getDoubleFirstFreeReg(int *freeReg, ass_reg_t *blockedFile,
                               int *usedWritePorts, char dir, bool concurrent,
                               bool usedummy, int regFile) {
  int reg;
  ass_reg_t blockedRegFiles = 0;
  do {
    regFile = -1;
    int free = 0;
    for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
      if ((dir == WRITE) &&
          (registers::getWritePorts(i) - usedWritePorts[i]) < 2)
        continue;
      if (blockedRegFiles & (1 << i))
        continue;
      if (freeReg[i] > free) {
        free = freeReg[i];
        regFile = i;
      }
    }
    if (regFile == -1) {
      //			cerr << "there are no free register files!" <<
      // endl;
      if (usedummy) {
        char32_t reg_rec =
            getDoubleFirstFreeReg(freeReg, blockedFile, usedWritePorts, dir,
                                  concurrent, false, regFile);
        if (reg_rec != (char32_t)-1) {
          return reg_rec;
        }
      }
      return -1;
    }
    LOG_OUTPUT(LOG_M_RA_DEBUG, "Searching register file %d\n", regFile);
    blockedRegFiles |= (1 << regFile);
    if (concurrent)
      reg = getDoubleFirstFreeConcurrent(blockedFile[regFile], usedummy);
    else if (regFile < (int)registers::getNumRegisterFiles() - 1)
      reg = getDoubleFirstFreeParallel(blockedFile[regFile],
                                       blockedFile[regFile + 1], usedummy);
    else
      reg = -1;
    if (reg == -1)
      LOG_OUTPUT(LOG_M_RA_DEBUG,
                 "Could not allocate register pair in regfile %d\n", regFile);
  } while (reg == -1);
  // block the register for further use.
  if (concurrent) {
    ass_reg_t temp = 3;
    freeReg[regFile] -= 2;
    blockedFile[regFile] |= (temp << reg);
  } else {
    ass_reg_t temp = 1;
    freeReg[regFile] -= 1;
    freeReg[regFile + 1] -= 1;
    blockedFile[regFile] |= (temp << reg);
    blockedFile[regFile + 1] |= (temp << reg);
  }
  return registers::createRegister(regFile, reg);
}

int getFree(ass_reg_t regBlock, bool usedummy) {
  int start, end;
  LOG_OUTPUT(LOG_M_RA_DEBUG,
             "[vreg] [getFree] Searching for register. Dummy = %d\n", usedummy);
  if (usedummy) {
    start = 0;
    end = registers::getLastDummyRegNumber() + 1;
  } else {
    start = 0;
    end = registers::getNumRegister1RF();
  }
  ass_reg_t temp = 1;
  // first try to allocate a register, which is free, but could not be used for
  // a double register.
  for (int i = start; i < end - 1; i += 2) {
    if ((regBlock & (temp << (i + 1))) && !(regBlock & (temp << i)))
      return i;
    if (!(regBlock & (temp << (i + 1))) && (regBlock & (temp << i)))
      return i + 1;
  }
  // if this fails, then any register is fine.
  for (int i = start; i < end; i++)
    if (!(regBlock & (temp << i)))
      return i;
  return -1;
}

char32_t getFreeReg(int *freeReg, ass_reg_t *blockedFile, int *usedWritePorts,
                    int regFile, bool usedummy) {
  if (regFile == -1) {
    int free = 0;
    for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
      if ((registers::getWritePorts(i) - usedWritePorts[i]) <= 0)
        continue;
      if (freeReg[i] > free) {
        free = freeReg[i];
        regFile = i;
      }
    }
  }
  if (regFile == -1) {
    //        ASS_INFO_COUT("Warning: All registers in use. No free register
    //        available!\n");
    LOG_OUTPUT(LOG_M_RA_DEBUG,
               "[vreg] [getFreeReg] Not enough physical registers\n");
    return -1;
  }
  LOG_OUTPUT(LOG_M_RA_DEBUG, "[vreg] [getFreeReg] Searching register file %d\n",
             regFile);
  int reg = getFree(blockedFile[regFile], usedummy);

  if (usedummy && reg == -1) {
    if (regFile == -1) {
      int i = 0;
      for (i = 0; i < (int)registers::getNumRegisterFiles(); i++) {
        reg = getFree(blockedFile[i], usedummy);
        if (reg != -1) {
          break;
        }
      }

      if (reg == -1) {
        reg = getFree(blockedFile[regFile], false);
      } else {
        regFile = i;
      }
    } else {
      reg = getFree(blockedFile[regFile], false);
    }
  }

  if (reg == -1) { // this should never happen.
    LOG_OUTPUT(LOG_M_RA_DEBUG, "[vreg] [getFreeReg] The impossible happened! "
                               "Not enough physical registers\n");
    return reg;
  }
  LOG_OUTPUT(LOG_M_RA_DEBUG,
             "[vreg] [getFreeReg] Selected register %d from register file %d\n",
             reg, regFile);
  // block the register for further use.
  ass_reg_t temp = 1;
  freeReg[regFile]--;
  blockedFile[regFile] |= (temp << reg);

  if (reg >= (int)registers::getNumRegister1RF()) {
    LOG_OUTPUT(LOG_M_RA_DEBUG, "[ASSERT] Assertion failed: Selected an invalid "
                               "register in register allocation!\n");
    EXIT_ERROR;
  }

  return registers::createRegister(regFile, reg);
}

VirtualRegisterMapping *getMapping(const VirtualRegisterMap *map,
                                   char32_t virt) {
  // i tried using a map for this, but the execution time is about 10% higher.
  auto it = map->find(virt);
  if (it == map->end())
    return nullptr;
  return it->second;
}

char32_t getReal(const VirtualRegisterMap *map,
                 const VirtualAllocation *regMapping, char32_t virt) {
  if (regMapping) {
    if (regMapping->find(virt) != regMapping->end())
      return regMapping->at(virt);
    else
      return -1;
  }

  auto it = map->find(virt);
  if (it == map->end())
    return -1;
  return it->second->getReal();
}

bool doesVirtualMappingExist(char32_t virtualMappedPhysicalRegister) {
  return virtualMappedPhysicalRegister != (char32_t)-1;
}

/**
 * Does the virtual Allocation of the MI work for the virtualRegistermap and the
 * Processor?
 * @param ins
 * @param map
 * @param regMapping
 * @param pro
 * @return Does the virtual Allocation of the MI work for the virtualRegistermap
 * and the Processor?
 *
 * If no virtual Register in MI -> True.
 *
 * Else can the Processor execute the MI?
 */
bool validVirtualMappingAndExecutableMI(MI *ins, VirtualRegisterMap *map,
                                        VirtualAllocation *regMapping,
                                        const Processor *pro) {

  bool replaced = false;

  MO **ops = ins->getOperations();
  uint slots = MI::getNumberIssueSlots();
  unsigned int currentSlot = 0;
  while (currentSlot < slots) {
    MO *currentMO = ops[currentSlot];
    if (currentMO == nullptr) { // the operation is not (yet) set.
      currentSlot++;
      continue;
    }
    // find virt 2 phy mappings
    char32_t *args = currentMO->getArguments();
    OPtype *types = currentMO->getTypes();
    for (int argIdx = 0; argIdx < currentMO->getArgNumber(); ++argIdx) {
      if (types[argIdx] == REG) {
        int regNumber = registers::getVirtualRegisterNumber(args[argIdx]);
        if (regNumber < 0)
          continue;

        // functionally from here assume reg is a virtual register
        // registers::isVirtualReg(args[argIdx])
        //        if (registers::isVirtualReg(args[argIdx])) {
        //          char32_t real = getReal(map, regMapping, args[argIdx]);
        //          if (doesVirtualMappingExist(real)) {
        //            args[argIdx] = real;
        //            replaced = true;
        //          }
        // check if there is already a mapping
        char32_t real = getReal(map, regMapping, args[argIdx]);
        if (doesVirtualMappingExist(real)) {
          //        if (real != (char32_t)-1) {
          args[argIdx] = real;
          replaced = true;
        }
        //        }
      }
    }
    currentSlot += currentMO->opLengthMultiplier();
  }
  if (!replaced)
    return true;
  return pro->isExecuteable(ins);
}

bool checkMapping(VirtualRegisterMap *map, VirtualAllocation *regMapping,
                  const Processor *pro, const Program::const_iterator start,
                  const Program::const_iterator end) {
  bool ret = true;
  for (auto it = start; it <= end; ++it) {
    // if we would use the original MI and MO, then the mapping we make at this
    // point would be final for all eternity, or at least until the program
    // finishes
    MI *testMI = new MI();
    MO **ops = (*it)->getOperations();
    MO **testOps = testMI->getOperations();
    unsigned int x = 0;
    while (ret && x < MI::getNumberIssueSlots()) {
      MO *m = ops[x];
      if (m == nullptr) { // the operation is not (yet) set.
        x++;
        continue;
      }
      testOps[x] = new MO(m->getOperation(), m->getArguments(), m->getTypes(),
                          m->getDirections());
      testOps[x]->setLineNumber(m->getLineNumber());
      x += m->opLengthMultiplier();
    }
    // if we can not execute one operation with the given mapping, it is
    // useless.
    if (!validVirtualMappingAndExecutableMI(testMI, map, regMapping, pro)) {
      LOG_OUTPUT(LOG_M_RA_DEBUG,
                 "[checkMapping] could not validate with virtual\n");
      if (it != start)
        cerr << "not the first one..." << endl;
      ret = false;
    }
    x = 0;
    // destructor of testMI
    //    while (x < testMI->getNumberIssueSlots()) {
    //      MO *m = testOps[x];
    //      if (m == nullptr) { // the operation is not (yet) set.
    //        x++;
    //      } else {
    //        int i = m->opLengthMultiplier();
    //        delete m;
    //        testOps[x] = nullptr;
    //        x += i;
    //      }
    //    }
    while (x < testMI->getNumberIssueSlots()) {
      MO *m = testOps[x];
      if (m == nullptr) { // the operation is not (yet) set.
        x++;
        continue;
      }
      int i = m->opLengthMultiplier();
      delete m;
      testOps[x] = nullptr;
      x += i;
    }
    delete testMI;
  }
  return ret;
}

/**
 * If mapping for register exists, the usage will be updated.
 * If the register mapping (virtual to physical) does not exist,
 * create a new entry in the map.
 * @param map
 * @param reg register to be checked.
 * @param arg
 * @param it Pointer to an MI instruction. Will be used to define the DU chain.
 */
void updatePosition(VirtualRegisterMap *map, char32_t reg, int arg,
                    Program::iterator it) {
  auto ot = map->find(reg);
  if (ot == map->end()) {
    LOG_OUTPUT(LOG_M_RA_PREP, "  Creating virtual register mapping for %s\n",
               registers::getName(reg).c_str());
    auto *m = new struct VirtualRegisterMapping;
    m->setLastOccurrence(it);
    m->setFirstOccurrence(it);
    m->setVirtual(reg);
    m->setReal(-1);
    m->setRegFile(-1);
    map->insert(std::pair<const char32_t, VirtualRegisterMapping *>(reg, m));
  } else {
    if (ot->second->getLastOccurrence() < it) {
      if (arg == 0) { // if target register is last occurence, prevent using the
                      // same target register multiple times
        ot->second->setLastOccurrence(it + 1);
      } else {
        ot->second->setLastOccurrence(it);
      }
    }
  }
}

void deleteCouplings(RegisterCoupling &couplings) {
  std::unordered_set<RegisterCouple *> c;
  for (auto &coupling : couplings)
    c.insert(coupling.second);
  for (auto it : c) {
    delete it;
  }
}

void storeCoupling(RegisterCoupling &couplings, char32_t reg1, char32_t reg2) {
  auto *c = new RegisterCouple();
  c->first = reg1;
  c->second = reg2;
  c->concurrent = true;
  auto p1 =
      couplings.insert(std::pair<const char32_t, RegisterCouple *>(reg1, c));
  auto p2 =
      couplings.insert(std::pair<const char32_t, RegisterCouple *>(reg2, c));
  if (!p1.second && !p2.second)
    delete c;
}

void storeCouplings(RegisterCoupling &couplings, MO *x2Op) {
  if (!x2Op->isX2Operation())
    return;
  char32_t *args = x2Op->getArguments();
  OPtype *types = x2Op->getTypes();
  for (int i = 0; i < 4; ++i) {
    if (types[i] == REG && types[i + 4] == REG)
      storeCoupling(couplings, args[i], args[i + 4]);
  }
}

void calcFreeReg(const ass_reg_t *blocked, int *freeReg) {
  for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
    int x = 0;
    ass_reg_t copy = blocked[i];
    // count the number of ones.
    if (copy == (ass_reg_t)-1)
      freeReg[i] = 0;
    else {
      while (copy) {
        x += copy & 1;
        copy = copy >> 1;
      }
      freeReg[i] = registers::getNumRegister1RF() - x;
    }
  }
}

/***
 *
 * @param blocked Add newlyBlocked Register to blocked Data Structure
 * @param newlyBlocked Register to block
 */
void addBlockedReg(ass_reg_t *blocked, int newlyBlocked) {
  int regFile = 0;
  int movePos = newlyBlocked;
  // check for RF1
  if (newlyBlocked >= numRegisters) {
    regFile = 1;
    movePos = newlyBlocked - numRegisters;
  }
  blocked[regFile] |= ((ass_reg_t)1) << movePos;
}

/***
 *
 * @param blocked Remove newlyBlocked Register to blocked Data Structure
 * @param newlyBlocked Register to unblock
 */
void removeBlockedReg(ass_reg_t *blocked, int newlyBlocked) {
  int regFile = 0;
  int movePos = newlyBlocked;
  // check for RF1
  if (newlyBlocked >= numRegisters) {
    regFile = 1;
    movePos = newlyBlocked - numRegisters;
  }
  blocked[regFile] |= ((ass_reg_t)0) << movePos;
}

/***
 *
 * @param ins iterates over the MikroInstructions
 * @param map update Map with virtual Register mappings.
 * @param freeing
 * @param couplings
 * @param pro
 */
void calculateCouplings(Program *ins, VirtualRegisterMap *map,
                        std::set<char32_t> *freeing,
                        RegisterCoupling &couplings, Processor *pro) {
  // generate the vector with all virtual registers.
  for (auto it = ins->begin(); it != ins->end(); ++it) {
    MI *mi = (*it);
    unsigned int slot = 0;
    if (isLog(LOG_M_RA_PREP)) {
      LOG_OUTPUT(LOG_M_RA_PREP, "Computing register _couplings for MI:\n");
      stringstream ss;
      mi->writeOutReadable(ss);
      ss << std::endl;
      LOG_OUTPUT(LOG_M_RA_PREP, ss.str().c_str());
    }
    while (slot < MI::getNumberIssueSlots()) {
      MO *mo = mi->getOperations()[slot];
      if (mo == nullptr) { // the operation is not (yet) set.
        slot++;
        continue;
      }
      if (isLog(LOG_M_RA_PREP)) {
        stringstream ss;
        ss << "Computing register _couplings for MO:";
        mo->writeOutReadable(ss);
        ss << std::endl;
        LOG_OUTPUT(LOG_M_RA_PREP, ss.str().c_str());
      }
      if (mo->getID() == 0) {
        int n = 3;
      }
      char32_t *args = mo->getArguments();
      OPtype *types = mo->getTypes();
      if (mo->isX2Operation() ||
          (getFU((char *)"AA")
               ->contains(mo->getOperation()->getName().c_str()) &&
           strncmp(mo->getOperation()->getName().c_str(), "MUL", 3))) {
        for (int e = 0; e < 4; e++) {
          if (args[e] != args[e + 4] && types[e + 4] == REG) {
            // first, check if there's at least one virtual register
            if (!registers::isVirtualReg(args[e]) &&
                !registers::isVirtualReg(args[e + 4])) {
              continue;
            }

            auto cpl_it1 = couplings.find(args[e]);
            auto cpl_it2 = couplings.find(args[e + 4]);
            if (cpl_it1 == couplings.end() || cpl_it2 == couplings.end()) {
              LOG_OUTPUT(LOG_M_RA_PREP, "  Recording register couple %s+%s\n",
                         registers::getName(args[e]).c_str(),
                         registers::getName(args[e + 4]).c_str());
              auto *c = new RegisterCouple();
              c->first = args[e];
              c->second = args[e + 4];
              c->concurrent = pro->isX2Supported();
              couplings.insert(std::make_pair(args[e], c)).first;
              couplings.insert(std::make_pair(args[e + 4], c));
            }
          }
        }
      }
      for (int e = 0; e < mo->getArgNumber(); ++e) {
        if (types[e] == REG) {
          // first check if it is a virtual register.
          int regNumber = registers::getVirtualRegisterNumber(args[e]);
          // if it is not a virtual register.
          if (regNumber < 0 && freeing->find(args[e]) == freeing->end()) {
            continue;
          }
          // at this point we have a virtual register.
          updatePosition(map, args[e], e, it);
          LOG_OUTPUT(LOG_M_RA_PREP, "  Update register  %s with ID=%d\n",
                     registers::getName(args[e]).c_str(),
                     it.operator*()->getOperation(0)->getID());
        }
      }
      slot += mo->opLengthMultiplier();
    }
  }
}

void freeMappings(vector<VirtualRegisterMapping *> *map,
                  Program::const_iterator it, ass_reg_t *blocked,
                  int *freeReg) {
  // after everything is done, we can check if we can open the register for
  // further scheduling.
  for (auto mit = map->begin(); mit != map->end(); ++mit) {
    VirtualRegisterMapping *m = *mit;
    if (m->getLastOccurrence() == it) {
      // delete the foundings, since they are no longer needed for the next
      // iteration.
      map->erase(mit);
      // decrease by one, otherwise there is an error inside the vector.
      mit--;

#if VERBOSE_REGISTER_ALLOC
      LOG_OUTPUT(LOG_M_RA_DEBUG, registers::getName(m->getVirtual()).c_str());
      LOG_OUTPUT(LOG_M_RA_DEBUG, " -> ");
      LOG_OUTPUT(LOG_M_RA_DEBUG, registers::getName(m->getReal()).c_str());
      LOG_OUTPUT(LOG_M_RA_DEBUG, " ");
#endif
    }
    if (m->getLastOccurrence() == (it + 1)) {
      if (m->getReal() == (char32_t)-1 || m->getReal() == (char32_t)-2) {
        continue;
      }

      uint i = registers::getRegFile(m->getReal());
      ass_reg_t temp = 1;
      if (m->isFreeRegister(
              blocked)) { //!(blocked[i] & (temp <<
                          //! registers::getRegNumber(m->real)))) {
        if (m->getFirstOccurrence() != m->getLastOccurrence()) {
          LOG_OUTPUT(LOG_M_ALWAYS, "Freeing nonfree register! %s - %s\n",
                     m->getVirtualName().c_str(), m->getRealName().c_str());
          (*it)->writeOutReadable(cout);
          (*(it + 1))->writeOutReadable(cout);
          LOG_OUTPUT(LOG_M_ALWAYS, "\n");
          //          EXIT_ERROR;
        }
      }
      LOG_OUTPUT(LOG_M_RA_DEBUG, "Freeing register %s - %s\n",
                 m->getRealName().c_str(), m->getVirtualName().c_str());
      blocked[i] &=
          (ass_reg_t) ~(temp << registers::getRegNumber(m->getReal()));
      freeReg[i]++;
    }
    if (m->getLastOccurrence() == it &&
        m->getLastOccurrence() == m->getFirstOccurrence()) {
      if (m->getReal() == (char32_t)-1 || m->getReal() == (char32_t)-2) {
        continue;
      }
      ass_reg_t temp = 1;
      uint i = registers::getRegFile(m->getReal());
      LOG_OUTPUT(LOG_M_RA_DEBUG, "Freeing register %s - %s\n",
                 registers::getName(m->getReal()).c_str(),
                 registers::getName(m->getVirtual()).c_str());
      blocked[i] &=
          (ass_reg_t) ~(temp << registers::getRegNumber(m->getReal()));
      freeReg[i]++;
    }
  }
}

void updateRegisterMapping(const std::vector<VirtualRegisterMapping *> &map,
                           char32_t virt, int regFile, int regNum) {
  for (auto it : map)
    if (it->getVirtual() == virt) {
      it->setReal(registers::createRegister(regFile, regNum));
      it->setRegFile(regFile);
      return;
    }
  LOG_OUTPUT(LOG_M_RA_DEBUG,
             "WARNING: Was asked to update register mapping for %s, but could "
             "not find it!\n",
             registers::getName(virt).c_str());
}

bool mappingCorrect(const std::vector<VirtualRegisterMapping *> &map,
                    char32_t virt, int regFile, int regNum) {
  for (auto it : map) {
    if (it->getVirtual() == virt) {
      char32_t real = it->getReal();
      return regFile == registers::getRegFile(real) &&
             regNum == registers::getRegNumber(real);
    }
  }
  return false;
}

/**
 * Allocate (and block) virtual registers that are coupled with fixed registers
 * @param map
 * @param couplings
 * @param blocked
 * @return
 */
bool preallocateCoupled(const std::vector<VirtualRegisterMapping *> &map,
                        const RegisterCoupling *couplings, ass_reg_t *blocked) {
  for (auto coupling : *couplings) {
    char32_t reg1 = coupling.second->first;
    char32_t reg2 = coupling.second->second;
    LOG_OUTPUT(LOG_M_RA_DEBUG,
               "Checking couple %s+%s for preallocation of virtual and fix\n",
               registers::getName(reg1).c_str(),
               registers::getName(reg2).c_str());
    char32_t virtualReg;
    int fixedNumber, fixedFile;
    if (registers::isVirtualReg(reg1) && registers::isPhysicalRegister(reg2)) {
      if (!gen_X2::getMatchingX2Reg(reg2, gen_X2::REG_PAIR_POS::SECOND,
                                    coupling.second->concurrent, fixedNumber,
                                    fixedFile))
        return false;
      virtualReg = reg1;
    } else if (registers::isPhysicalRegister(reg1) &&
               registers::isVirtualReg(reg2)) {
      if (!gen_X2::getMatchingX2Reg(reg1, gen_X2::REG_PAIR_POS::FIRST,
                                    coupling.second->concurrent, fixedNumber,
                                    fixedFile))
        return false;
      virtualReg = reg2;
    } else
      continue;

    if (!(blocked[fixedFile] & ((ass_reg_t)1) << fixedNumber)) {
      LOG_OUTPUT(LOG_M_RA_DEBUG, "Preallocating %s to V%dR%d\n",
                 registers::getName(virtualReg).c_str(), fixedFile,
                 fixedNumber);
      blocked[fixedFile] |= ((ass_reg_t)1) << fixedNumber;
      updateRegisterMapping(map, virtualReg, fixedFile, fixedNumber);
    } else {
      if (!mappingCorrect(map, virtualReg, fixedFile, fixedNumber)) {
        LOG_OUTPUT(LOG_M_RA_DEBUG,
                   "Could not preallocate %s to V%dR%d (already blocked)\n",
                   registers::getName(virtualReg).c_str(), fixedFile,
                   fixedNumber);
        return false;
      }
    }
  }

  return true;
}

std::string get_error_localization(MI *instruction, int line, SLM *slm) {
  auto mo0 = instruction->getOperations()[0];
  MO *mo1 = nullptr;
  if (instruction->getNumberIssueSlots() > 1) {
    MO *mo1 = instruction->getOperations()[1];
  }
  int id = mo0->getID();

  std::stringstream ss;
  ss << "Failed: ";
  ss << "\nSLM=" + std::to_string(slm->getOriginalID()) +
            "; Failed=" + std::to_string(id);
  if (mo1) {
    id = mo1->getID();
    ss << "," + std::to_string(id);
  }
  ss << ";Line=" + std::to_string(line) + "\n";
  instruction->writeOutReadable(ss);
  return ss.str();
}

void print_error_message(MI *instruction, int line, SLM *slm) {
  // do not execute this function with parallelism, will cause exceptions
  if (params.application_mode) {
    if (params.debugging) {
      instruction->writeOutReadable(cout);
      std::string result_str = get_error_localization(instruction, line, slm);
      std::cout << result_str;
    }
  } else {

    std::string result_str = get_error_localization(instruction, line, slm);
    slm->error_string += result_str;
    if (params.debugging) {
      std::cout << result_str;
    }
  }
}

uint allocate_virtual(ass_reg_t *blocked, const Processor *pro,
                      const Program *ins, VirtualRegisterMap *map,
                      const RegisterCoupling *couplings, int *freeReg,
                      const Context &ctx, SLM *slm) {
  uint max_concurrent_blocked = 0;
  uint init_blocked = getNumBlockedRegs(blocked);

  std::vector<MI *>::const_iterator *dynFreeRegs =
      new std::vector<MI *>::const_iterator[registers::getNumRegister1RF() *
                                            registers::getNumRegisterFiles()];

  // enter this for heuristic RA
  if (slm) {
    if (!slm->getFreeReg()->empty()) {
      for (auto insPtr = ins->begin(); insPtr != ins->end(); ++insPtr) {
        MI *instruction = (*insPtr);
        unsigned int slot = 0;
        while (slot < MI::getNumberIssueSlots()) {
          MO *operation = instruction->getOperations()[slot];
          if (operation == nullptr) { // the operation is not (yet) set.
            slot++;
            continue;
          }
          char32_t *args = operation->getArguments();
          OPtype *types = operation->getTypes();
          for (int argIdx = 0; argIdx < operation->getArgNumber(); ++argIdx) {
            if (types[argIdx] == REG &&
                registers::isPhysicalRegister(args[argIdx])) {
              if (slm->getFreeReg()->find(args[argIdx]) !=
                  slm->getFreeReg()->end()) {
                dynFreeRegs[registers::getRegNumber(args[argIdx]) +
                            (registers::getRegFile((args[argIdx])) *
                             registers::getNumRegister1RF())] = insPtr;
              }
            }
          }
          slot += operation->opLengthMultiplier();
        }
      }
    }
  }

  int nonMappableRegisters = 0;
  int lastDummyRegNumber = registers::getLastDummyRegNumber();
  int dummyUsageTime = registers::getDummyUsageTime();

  /* It seems contradictory to create a vector by iterating over a map just to
   * iterate over it in the subfunction, but c can iterate over vectors much
   * faster then over maps and the subfunction is called several times. So, we
   * increase the speed quite a bit.
   */
  vector<VirtualRegisterMapping *> mapVec;
  mapVec.reserve(map->size());
  for (auto &mit : *map) {
    mapVec.push_back(mit.second);
  }

  if (params.mergeFixAndVirtual) {
    if (!preallocateCoupled(mapVec, couplings, blocked)) {
      delete[] dynFreeRegs;
      return NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET;
    }
  }

  // go through whole program and allocate register
  int line = 0;
  bool firstError = true;
  for (auto insPtr = ins->begin(); insPtr != ins->end(); ++insPtr) {
    MI *instruction = (*insPtr);

    unique_ptr<int[]> usedWritePorts(new int[registers::getNumRegisterFiles()]);
    instruction->determineUsedWritePorts(usedWritePorts.get());

    LOG_OUTPUT(LOG_M_RA_DEBUG,
               "\n[vreg] [allocate_virtual] %s Allocating registers for "
               "instruction:\n\t",
               ctx.asString().c_str());
    if (isLog(LOG_M_RA_DEBUG)) {
      stringstream ss;
      instruction->writeOutReadable(ss);
      LOG_OUTPUT(LOG_M_RA_DEBUG, "%s\t", ss.str().c_str());
      ss.str("");
      ss.clear();
      instruction->writeOutReadable(ss, map);
      LOG_OUTPUT(LOG_M_RA_DEBUG, "%s", ss.str().c_str());
      LOG_OUTPUT(LOG_M_RA_DEBUG,
                 "[vreg] [allocate_virtual] Blocked registers:\n");
      printBlockedRegisters(blocked, freeReg);
    }

    max_concurrent_blocked =
        std::max(max_concurrent_blocked, getNumBlockedRegs(blocked));

    if (!checkMapping(map, nullptr, pro, insPtr, insPtr)) {
      print_error_message(instruction, line, slm);
      if (not params.application_mode) {
        slm->error_string +=
            "Cannot allocate virtual registers. I don't even try...\n";
      }

      LOG_OUTPUT(
          LOG_M_RA_DEBUG,
          "[vreg] %s Cannot allocate virtual registers. I don't even try...\n",
          ctx.asString().c_str());
      nonMappableRegisters++;
      continue;
    }

    unsigned int slot = 0;
    while (slot < MI::getNumberIssueSlots()) {
      MO *operation = instruction->getOperations()[slot];
      if (operation == nullptr) { // the operation is not (yet) set.
        slot++;
        continue;
      }
      char32_t *args = operation->getArguments();
      OPtype *types = operation->getTypes();
      char *dirs = operation->getDirections();
      for (int argIdx = 0; argIdx < operation->getArgNumber(); ++argIdx) {
        if (types[argIdx] == REG) {
          // first check if it is a virtual register.
          int regNumber = registers::getVirtualRegisterNumber(args[argIdx]);
          if (regNumber < 0)
            continue;

          VirtualRegisterMapping *mapping = getMapping(map, args[argIdx]);
          if (mapping == nullptr) {
            LOG_OUTPUT(
                LOG_M_MERGE_DEBUG,
                "[virtual_test] %s Could not find mapping for register %s\n",
                ctx.asString().c_str(),
                registers::getName(args[argIdx]).c_str());
            return NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET;
          }
          if (mapping->getReal() != (char32_t)-1) {
            // could not be allocated last time OR already allocated
            continue;
          }
          max_concurrent_blocked =
              std::max(max_concurrent_blocked, getNumBlockedRegs(blocked));

          auto coupleIter = couplings->find(args[argIdx]);
          if (coupleIter != couplings->end()) {
            RegisterCouple *couple = coupleIter->second;
            LOG_OUTPUT(LOG_M_RA_DEBUG,
                       "[vreg] [allocate_virtual] Trying to allocate coupled "
                       "%s - %s\n",
                       registers::getName(couple->first).c_str(),
                       registers::getName(couple->second).c_str());
            VirtualRegisterMapping *coupleMapping =
                getMapping(map, couple->second);
            mapping = getMapping(map, couple->first);
            if (mapping == nullptr || coupleMapping == nullptr) {
              LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                         "[virtual_test] Could not find mapping for registers "
                         "%s - %s\n",
                         registers::getName(couple->first).c_str(),
                         registers::getName(couple->second).c_str());
              return NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET;
            }
            max_concurrent_blocked =
                std::max(max_concurrent_blocked, getNumBlockedRegs(blocked));

            Program::iterator last_occurence;
            if (coupleMapping->getLastOccurrence() >
                mapping->getLastOccurrence())
              last_occurence = coupleMapping->getLastOccurrence();
            else
              last_occurence = mapping->getLastOccurrence();
            if (mapping->getRegFile() == -1) {
              int *freeReg_copy = new int[registers::getNumRegisterFiles()];
              auto *blocked_copy =
                  new ass_reg_t[registers::getNumRegisterFiles()];
              for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
                blocked_copy[i] = blocked[i];
                freeReg_copy[i] = freeReg[i];
              }
              do {
                bool usedummy =
                    lastDummyRegNumber > 0 &&
                    (mapping->getLastOccurrence() -
                     mapping->getFirstOccurrence()) <= dummyUsageTime &&
                    (coupleMapping->getLastOccurrence() -
                     coupleMapping->getFirstOccurrence()) <= dummyUsageTime;
                char32_t reg = getDoubleFirstFreeReg(
                    freeReg_copy, blocked_copy, usedWritePorts.get(),
                    dirs[argIdx], couple->concurrent, usedummy, -1);
                mapping->setReal(reg);
                if (reg == (char32_t)-1) {
                  LOG_OUTPUT(
                      LOG_M_RA_DEBUG,
                      "[vreg] [allocate_virtual] %s could not be mapped\n",
                      mapping->getVirtualName().c_str());

                  print_error_message(instruction, line, slm);
                  if (not params.application_mode) {
                    slm->error_string += mapping->getVirtualName().c_str();
                    slm->error_string += " could not be mapped\n";
                  }

                  break;
                }
                max_concurrent_blocked = std::max(max_concurrent_blocked,
                                                  getNumBlockedRegs(blocked));

                LOG_OUTPUT(LOG_M_RA_DEBUG,
                           "[vreg] [allocate_virtual] allocated %s to %s\n",
                           registers::getName(couple->first).c_str(),
                           mapping->getRealName().c_str());
                // mapping for the second registers
                if (couple->concurrent) // if we have concurrent registers, use
                                        // the next one
                  coupleMapping->setReal(reg + 1);
                else { // else create the register in the next register file.
                  coupleMapping->setReal(
                      registers::createRegister(registers::getRegFile(reg) + 1,
                                                registers::getRegNumber(reg)));
                }
              } while (!checkMapping(map, nullptr, pro, insPtr, insPtr));
              int regfile = registers::getRegFile(mapping->getReal());
              if (regfile >= 0 &&
                  regfile <= (int)registers::getNumRegisterFiles())
                usedWritePorts[regfile] += 2;
              delete[] freeReg_copy;
              delete[] blocked_copy;
            } else {
              bool usedummy =
                  lastDummyRegNumber > 0 &&
                  (mapping->getLastOccurrence() -
                   mapping->getFirstOccurrence()) <= dummyUsageTime &&
                  (coupleMapping->getLastOccurrence() -
                   coupleMapping->getFirstOccurrence()) <= dummyUsageTime;
              mapping->setReal(getDoubleFirstFreeReg(
                  freeReg, blocked, usedWritePorts.get(), dirs[argIdx],
                  couple->concurrent, usedummy, mapping->getRegFile()));

              if (mapping->getReal() != (char32_t)-1) {
                LOG_OUTPUT(LOG_M_RA_DEBUG,
                           "[vreg] [allocate_virtual] allocated %s to %s\n",
                           registers::getName(couple->first).c_str(),
                           registers::getName(mapping->getReal()).c_str());
                int regfile = registers::getRegFile(mapping->getReal());
                if (regfile >= 0 &&
                    regfile <= (int)registers::getNumRegisterFiles())
                  usedWritePorts[regfile] += 2;
                // mapping for the second registers
                if (couple->concurrent) // if we have concurrent registers, use
                                        // the next one
                  coupleMapping->setReal(mapping->getReal() + 1);
                else { // else create the register in the next register file.
                  coupleMapping->setReal(registers::createRegister(
                      registers::getRegFile(mapping->getReal()) + 1,
                      registers::getRegNumber(mapping->getReal())));
                }
                max_concurrent_blocked = std::max(max_concurrent_blocked,
                                                  getNumBlockedRegs(blocked));
              } else {
                LOG_OUTPUT(LOG_M_RA_DEBUG,
                           "[vreg] [allocate_virtual] %s could not be mapped\n",
                           registers::getName(mapping->getVirtual()).c_str());
              }
            }
#if VERBOSE_REGISTER_ALLOC
            if ((int)mapping->getReal() >= 0 &&
                (int)coupleMapping->getReal() >= 0) {
              fprintf(params.logFile,
                      registers::getName(mapping->getVirtual()).c_str());
              fprintf(params.logFile, " -> ");
              fprintf(params.logFile,
                      registers::getName(mapping->getReal()).c_str());
              fprintf(params.logFile, " ");
              fprintf(params.logFile,
                      registers::getName(coupleMapping->getVirtual()).c_str());
              fprintf(params.logFile, " -> ");
              fprintf(params.logFile,
                      registers::getName(coupleMapping->getReal()).c_str());
              fprintf(params.logFile, " ");
            }
#endif
            if (mapping->getReal() != (char32_t)-1) {
              if (mapping->getRegFile() == -1) {
                ass_reg_t temp = 1;
                blocked[registers::getRegFile(mapping->getReal())] |=
                    (temp << registers::getRegNumber(mapping->getReal()));
                freeReg[registers::getRegFile(mapping->getReal())] -= 1;
                blocked[registers::getRegFile(coupleMapping->getReal())] |=
                    (temp << registers::getRegNumber(coupleMapping->getReal()));
                freeReg[registers::getRegFile(coupleMapping->getReal())] -= 1;
              }
            } else {
              mapping->setReal(-2);
              coupleMapping->setReal(-2);
              nonMappableRegisters += 2;

              print_error_message(instruction, line, slm);
            }
          } else {
            LOG_OUTPUT(
                LOG_M_RA_DEBUG,
                "[vreg] [allocate_virtual] Trying to allocate single %s\n",
                mapping->getVirtualName().c_str());
            bool usedummy = lastDummyRegNumber > 0 &&
                            (mapping->getLastOccurrence() -
                             mapping->getFirstOccurrence()) <= dummyUsageTime;
            // allocation for single virtual registers.
            mapping->setReal(getFreeReg(freeReg, blocked, usedWritePorts.get(),
                                        mapping->getRegFile(), usedummy));
            max_concurrent_blocked =
                std::max(max_concurrent_blocked, getNumBlockedRegs(blocked));
            if (mapping->getReal() == (char32_t)-1) {
              mapping->setReal(-2);
              nonMappableRegisters++;
              LOG_OUTPUT(LOG_M_RA_DEBUG,
                         "[vreg] [allocate_virtual] %s could not be mapped\n",
                         registers::getName(mapping->getVirtual()).c_str());

              print_error_message(instruction, line, slm);
              if (not params.application_mode) {
                slm->error_string += mapping->getVirtualName().c_str();
                slm->error_string += " could not be mapped\n";
              }
              continue;
            }
            LOG_OUTPUT(LOG_M_RA_DEBUG,
                       "[vreg] [allocate_virtual] allocated %s to %s\n",
                       registers::getName(mapping->getVirtual()).c_str(),
                       registers::getName(mapping->getReal()).c_str());
            if (mapping->getRegFile() == -1) {
              char32_t already = 0;
              while (!checkMapping(map, nullptr, pro, insPtr, insPtr)) {
                LOG_OUTPUT(LOG_M_RA_DEBUG, "[vreg] [allocate_virtual] This "
                                           "allocation was not possible!\n");
                // if this mapping was not possible, then we have to clean up!
                already |= (1 << registers::getRegFile(mapping->getReal()));
                ass_reg_t temp = 1;
                //								cout
                //<< "Freeing blocked register VxR" <<
                // registers::getVirtualRegisterNumber(mapping->getVirtual()) <<
                // " in instruction" << endl;
                //								(*it)->writeOutReadable(cout);
                blocked[registers::getRegFile(mapping->getReal())] &=
                    (ass_reg_t) ~(
                        temp << registers::getRegNumber(mapping->getReal()));
                freeReg[registers::getRegFile(mapping->getReal())]++;

                bool possible = false;
                for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
                  if ((1 << i) & already)
                    continue;
                  bool usedummy =
                      lastDummyRegNumber > 0 &&
                      (mapping->getLastOccurrence() -
                       mapping->getFirstOccurrence()) <= dummyUsageTime;
                  LOG_OUTPUT(
                      LOG_M_RA_DEBUG,
                      "[vreg] [allocate_virtual] Trying to allocate %s again\n",
                      registers::getName(mapping->getVirtual()).c_str());
                  mapping->setReal(getFreeReg(
                      freeReg, blocked, usedWritePorts.get(), i, usedummy));
                  if (mapping->getReal() == (char32_t)-1) {
                    print_error_message(instruction, line, slm);

                    if (not params.application_mode) {
                      slm->error_string += mapping->getVirtualName().c_str();
                      slm->error_string += " could not be mapped\n";
                    }

                    LOG_OUTPUT(
                        LOG_M_RA_DEBUG,
                        "[vreg] [allocate_virtual] %s could not be mapped\n",
                        registers::getName(mapping->getVirtual()).c_str());
                    continue;
                  }
                  LOG_OUTPUT(LOG_M_RA_DEBUG,
                             "[vreg] [allocate_virtual] Allocated %s to %s\n",
                             registers::getName(mapping->getVirtual()).c_str(),
                             registers::getName(mapping->getReal()).c_str());
                  // block the register for further use.
                  possible = true;
                  break;
                }
                if (!possible) {
                  nonMappableRegisters++;
                  mapping->setReal(-2);
                  LOG_OUTPUT(LOG_M_RA_DEBUG,
                             "[vreg] [allocate_virtual] Register %s could not "
                             "be mapped, giving up\n",
                             registers::getName(mapping->getVirtual()).c_str());
                  break;
                }
              }
              int regfile = registers::getRegFile(mapping->getReal());
              if (regfile >= 0 &&
                  regfile <= (int)registers::getNumRegisterFiles())
                usedWritePorts[regfile] += 1;
            }
            max_concurrent_blocked =
                std::max(max_concurrent_blocked, getNumBlockedRegs(blocked));
#if VERBOSE_REGISTER_ALLOC
            if ((int)mapping->getReal() >= 0) {
              fprintf(params.logFile,
                      registers::getName(mapping->getVirtual()).c_str());
              fprintf(params.logFile, " -> ");
              fprintf(params.logFile,
                      registers::getName(mapping->getReal()).c_str());
              fprintf(params.logFile, " ");
            }
#endif
          }
          if (slm) {
            if (!slm->getFreeReg()->empty()) {
              if (types[argIdx] == REG &&
                  registers::isPhysicalRegister(args[argIdx])) {
                if (dynFreeRegs[registers::getRegNumber(args[argIdx]) +
                                (registers::getRegFile((args[argIdx])) *
                                 registers::getNumRegister1RF())] == insPtr) {
                  ass_reg_t temp = 1;
                  blocked[registers::getRegFile((args[argIdx]))] &=
                      (ass_reg_t) ~(temp
                                    << registers::getRegNumber(args[argIdx]));
                  freeReg[registers::getRegFile((args[argIdx]))]++;
                }
              }
            }
            max_concurrent_blocked =
                std::max(max_concurrent_blocked, getNumBlockedRegs(blocked));
          }
          LOG_OUTPUT(LOG_M_RA_DEBUG, "[virtual test] mapping %s to %s\n",
                     registers::getName(mapping->getVirtual()).c_str(),
                     registers::getName(mapping->getReal()).c_str());
        }
      }
      slot += operation->opLengthMultiplier();
      if (isLog(LOG_M_RA_DEBUG)) {
        LOG_OUTPUT(LOG_M_RA_DEBUG,
                   "[vreg] [allocate_virtual] Blocked registers:\n");
        printBlockedRegisters(blocked, freeReg);
      }
    }
//        if (!checkMapping(map, 0, pro, it, it)) {
//            nonMappableRegisters++;
//        }
#if VERBOSE_REGISTER_ALLOC
    LOG_OUTPUT(LOG_M_RA_DEBUG, "\nblocked registers: ");
    for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
      ass_reg_t reg = blocked[i];
      LOG_OUTPUT(LOG_M_RA_DEBUG, " ");
      for (uint x = 0; x < registers::getNumRegister1RF(); x++) {
        LOG_OUTPUT(LOG_M_RA_DEBUG, "%d", (int)(1 & reg >> x));
      }
      LOG_OUTPUT(LOG_M_RA_DEBUG, " %2d", freeReg[i]);
    }
    LOG_OUTPUT(LOG_M_RA_DEBUG, "\n");
#endif
    freeMappings(&mapVec, insPtr, blocked, freeReg);
#if VERBOSE_REGISTER_ALLOC
    LOG_OUTPUT(LOG_M_RA_DEBUG, "\nblocked registers: ");
    for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
      ass_reg_t reg = blocked[i];
      LOG_OUTPUT(LOG_M_RA_DEBUG, " ");
      for (uint x = 0; x < registers::getNumRegister1RF(); x++) {
        LOG_OUTPUT(LOG_M_RA_DEBUG, "%d", (int)(1 & reg >> x));
      }
      LOG_OUTPUT(LOG_M_RA_DEBUG, " %2d", freeReg[i]);
    }
    LOG_OUTPUT(LOG_M_RA_DEBUG, "\n");
#endif
    line++;
  }
  // std::cout << "MAX BLOCKED REGISTER = "
  //           << max_concurrent_blocked - init_blocked << std::endl;
  if (slm) {
    slm->_heuristic_sched_max_blocked_regs =
        max_concurrent_blocked - init_blocked;
  }

  delete[] dynFreeRegs;
  return nonMappableRegisters;
}

#ifdef PRINTOUT_DISTANCES
void insertIntoDistances(vector<VirtualRegisterMapping> *map) {
  for (vector<VirtualRegisterMapping>::iterator mit = map->begin();
       mit != map->end(); ++mit) {
    VirtualRegisterMapping m = *mit;
    int distance = m.last_occurrence - m.first_occurrence;
    distances[distance]++;
  }
}
#endif

void printBlockedRegisters(const ass_reg_t *blocked, int *freeReg) {
  for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
    LOG_OUTPUT(LOG_M_ALWAYS, " ");
    ass_reg_t reg = blocked[i];
    for (uint x = 0; x < registers::getNumRegister1RF(); x++) {
      LOG_OUTPUT(LOG_M_ALWAYS, "%d", (int)(1 & reg >> x));
      if (!((x + 1) % 4)) {
        LOG_OUTPUT(LOG_M_ALWAYS, ".");
      }
    }
    if (freeReg) {
      LOG_OUTPUT(LOG_M_ALWAYS, " (%2d)", freeReg[i]);
    }
    LOG_OUTPUT(LOG_M_ALWAYS, "\n");
  }
}

std::vector<uint> getBlockedRegisters(const ass_reg_t *blocked) {
  std::vector<uint> blockedRegisters;

  uint counter = 0;
  for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
    ass_reg_t reg = blocked[i];
    for (uint x = 0; x < registers::getNumRegister1RF(); x++) {
      if ((int)(1 & reg >> x) == 1) {
        blockedRegisters.push_back(counter);
      }
      counter++;
    }
  }

  return blockedRegisters;
}

uint getNumBlockedRegs(const ass_reg_t *blocked) {
  uint counter = 0;
  for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
    ass_reg_t reg = blocked[i];
    for (uint x = 0; x < registers::getNumRegister1RF(); x++) {
      if ((int)(1 & reg >> x) == 1) {
        counter++;
      }
    }
  }
  return counter;
}

int heuristicRegisterAllocation(SLM *slm, Processor *pro, Program *ins,
                                RegisterCoupling *couplings,
                                VirtualRegisterMap *map, rdg::RDG **rdg,
                                const Context &ctx) {
  int virtualRegisterCount = 0;
  for (auto ma : *map) {
    if (registers::isVirtualReg(ma.first)) {
      virtualRegisterCount++;
    }
  }
  *rdg = nullptr;
  ass_reg_t *blocked = blockedRegsInSLM.find(slm->getOriginalID())->second;
  int *freeReg = new int[registers::getNumRegisterFiles()];
  calcFreeReg(blocked, freeReg);
  auto *b = new ass_reg_t[registers::getNumRegisterFiles()];
  memcpy(b, blocked, sizeof(ass_reg_t) * registers::getNumRegisterFiles());
  LOG_OUTPUT(LOG_M_RA,
             "%s Starting heuristic register allocation for SLM %d with %d "
             "instructions\n",
             ctx.asString().c_str(), slm->getOriginalID(), ins->size());
  if (isLog(LOG_M_RA_DEBUG)) {
    LOG_OUTPUT(LOG_M_RA_DEBUG, "Blocked registers:\n");
    printBlockedRegisters(b, freeReg);
  }
#if CHECK_RA_TIMING
  ra_timings.heuristicTimer.start();
#endif
  int ret = allocate_virtual(b, pro, ins, map, couplings, freeReg, ctx, slm);
#if CHECK_RA_TIMING
  ra_timings.heuristicTimer.stop();
  ra_timings.heuristicCount++;
#endif
  LOG_OUTPUT(LOG_M_RA,
             "%s Result of heuristic register allocation for SLM %d: %d with "
             "%d virtual register\n",
             ctx.asString().c_str(), slm->getOriginalID(), ret,
             virtualRegisterCount);
  //  LOG_OUTPUT(LOG_M_ALWAYS, "<--------------> SLM %d has %d virtual register
  //  <-------------->\n", slm->getOriginalID(), virtualRegisterCount);
  delete[] freeReg;
  delete[] b;

  if (ret != 0 && params.portReg != 0) {
    // Prepare RDG for this SLM
    //    	LOG_OUTPUT(LOG_M_ALWAYS, "Preparing RDG\n");
    *rdg = rdg::RDG::analyseRegisterDependencies(ins, couplings, map, blocked);
    if (!(*rdg) && !(ctx.schedRound() != -1 && ctx.schedIndiv() != -1)) {
      slm->error_string += "RDG could not be created\n";
      print_error_message(ins->getMI(params.best_sched_length),
                          params.best_sched_length, slm);
    }
  }

  // add transition Energy
  if (ret == 0) {
  }

  return ret;
}

void printRDG(SLM *slm, Program *ins, VirtualRegisterMap *map, rdg::RDG *rdg) {
  LOG_OUTPUT(LOG_M_ALWAYS, "---- RDG for SLM %d\n", slm->getOriginalID());
  for (auto mi : *ins) {
    mi->writeOutReadable(cout, nullptr);
  }
  LOG_OUTPUT(LOG_M_ALWAYS, "---- RDG "
                           "]--------------------------------------------------"
                           "---------------------------\n");
  rdg->printDot();
  LOG_OUTPUT(LOG_M_ALWAYS, "---- END RDG "
                           "]--------------------------------------------------"
                           "---------------------------\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "---- DEPTH = %d\n", rdg->maxDepth());
}

int geneticRegisterAllocation(SLM *slm, Processor *pro, Program *ins,
                              RegisterCoupling *couplings,
                              VirtualRegisterMap *map, rdg::RDG **rdg,
                              const Context &ctx,
                              VirtualRegisterMap *heuristicMap) {

  if (params.portReg != 0 or params.raRegPopSize > 0) {
    ass_reg_t *blocked = blockedRegsInSLM[slm->getOriginalID()];

    //    *rdg = rdg::RDG::analyseRegisterDependencies(ins, couplings, map,
    //    blocked);
    flushLog();
    //    if(! isPowerOptimization()){
    if (!(*rdg)) { // register allocation not possible
      LOG_OUTPUT(LOG_M_RDG_STAT, "No RDG for SLM %d\n", slm->getOriginalID());
      return -2;
    } else {
      LOG_OUTPUT(LOG_M_RDG_STAT, "RDG for SLM %d completed.\n",
                 slm->getOriginalID());
    }
    if (map) {
      LOG_OUTPUT(LOG_M_RDG_STAT,
                 "RDG for SLM %d has %d roots for %lu registers\n",
                 slm->getOriginalID(), (*rdg)->rootCount(), map->size());
    } else {
      LOG_OUTPUT(LOG_M_RDG_STAT,
                 "RDG for SLM %d has %d roots for unknown registers\n",
                 slm->getOriginalID(), (*rdg)->rootCount());
    }

    if (isLog(LOG_M_RDG_DEBUG)) {
      printRDG(slm, ins, map, *rdg);
      //            exit(-1);
    }
    //    }

    //        LOG_OUTPUT(LOG_M_ALWAYS, "Performing port optimal RA\n");
    int genret = portOptReg::allocateRegisters(
        slm->getOriginalID(), **rdg, blocked, pro, ins, map, *couplings,
        Context::nextStage(ctx, "port_reg"), heuristicMap);
    delete *rdg;
    *rdg = nullptr;
    LOG_OUTPUT(LOG_M_RA,
               "%s Result of genetic port optimal register allocation for SLM "
               "%d: %d\n",
               ctx.asString().c_str(), slm->getOriginalID(), genret);

    // write Failure of best register allocation to SLM->errorMessage
    // it is a new error message, so that heuristic failure can be ignored!
    if (genret == 0) {
      slm->initErrorMessage(ins);
    }

    return genret;
  }
  return -1;
}

ga_stats::ChromosomeFitness geneticRegisterAllocationPower(
    SLM *slm, Processor *pro, Program *ins, RegisterCoupling *couplings,
    VirtualRegisterMap *map, rdg::RDG **rdg, const Context &ctx,
    VirtualRegisterMap *heuristicMap) {

  if (params.raRegPopSize > 0) {
    ass_reg_t *blocked = blockedRegsInSLM[slm->getOriginalID()];

    *rdg = rdg::RDG::analyseRegisterDependencies(ins, couplings, map, blocked);
    flushLog();
    if (!(*rdg)) { // register allocation not possible
      LOG_OUTPUT(LOG_M_ALWAYS, "No RDG for SLM %d\n", slm->getOriginalID());
      return -2;
    } else {
      LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG, "RDG for SLM %d completed.\n",
                 slm->getOriginalID());
    }
    LOG_OUTPUT(LOG_M_RDG_STAT,
               "RDG for SLM %d has %d roots for %lu registers\n",
               slm->getOriginalID(), (*rdg)->rootCount(), map->size());

    if (isLog(LOG_M_RDG_DEBUG)) {
      printRDG(slm, ins, map, *rdg);
      //            exit(-1);
    }

    //        LOG_OUTPUT(LOG_M_ALWAYS, "Performing port optimal RA\n");
    ga_stats::ChromosomeFitness genret = portOptReg::allocateRegistersPower(
        slm->getOriginalID(), **rdg, blocked, pro, ins, map, *couplings,
        Context::nextStage(ctx, "port_reg"), heuristicMap);
    delete *rdg;
    *rdg = nullptr;
    LOG_OUTPUT(LOG_M_RA,
               "%s Result of genetic port optimal register allocation for SLM "
               "%d: %d with %f pJ\n",
               ctx.asString().c_str(), slm->getOriginalID(),
               genret.getFitness(), genret.getTransitionEnergy());
    return genret;
  }
  return -1;
}

int virtual_test(SLM *slm, Processor *pro, Program *ins,
                 VirtualRegisterMap **map, RegisterCoupling *couplings,
                 const Context &ctx) {
  int ret = 32676;
  rdg::RDG *rdg = nullptr;
  if (params.heuristicReg) {
    ret = heuristicRegisterAllocation(slm, pro, ins, couplings, *map, &rdg,
                                      Context::nextStage(ctx, "h_reg"));
    slm->setHeuristicRegisterFitness(ret);
  }
#ifdef PRINTOUT_DISTANCES
  insertIntoDistances(map);
#endif
  if (not gen_sched::successRA(ret)) {
    slm->error_string += "Heuristic register allocation failed\n";
    ret = geneticRegisterAllocation(slm, pro, ins, couplings, *map, &rdg, ctx);
    slm->setGeneticRegisterFitness(ret);
    if (not gen_sched::successRA(ret))
      releaseVirtualMap(map);
    delete rdg;
  }
  return ret;
}

bool virtual_schedule(SLM *slm, Processor *pro) {
  // cleanup
  if (blocked != nullptr) {
    free(blocked);
    blocked = nullptr;
  }
  if (blockedRegsInSLM.find(slm->getID()) != blockedRegsInSLM.end()) {
    ass_reg_t *b = blockedRegsInSLM.find(slm->getID())->second;
    free(b);
  }
  Program *ins = slm->getShortestInstruction();
  if (ins == nullptr)
    return true;
  bool ret = true;
  VirtualRegisterMap *map = slm->getVirtualMapping();
  for (auto &in : *ins) {
    ret &= validVirtualMappingAndExecutableMI(in, map, nullptr, pro);
    if (!ret) {
      cerr << "could not validate: " << std::endl;
      in->writeOutReadable(std::cerr);
      return false;
    }
  }
  return ret;
}

int detectHighestVirtual(SLM *slm) {
  return (registers::getNumRegisterFiles() + 1) *
         registers::getNumRegister1RF();
}

/***
 * Rename Virtual Register except for READWRITE register. They will not be
 * renamed and destroy the SSA form.
 * @param mo
 * @param maps
 * @param maxnumber
 */
void renameRegisters(MO *mo, unordered_map<char32_t, char32_t> &maps,
                     int &maxnumber) {
  char32_t *args = mo->getArguments();
  OPtype *types = mo->getTypes();
  char *dir = mo->getDirections();
  // this has to be done two times.
  // first time is to set the read registers to the right values.
  for (int e = 0; e < mo->getArgNumber(); ++e) {
    if (types[e] == REG) {
      int regNumber = registers::getVirtualRegisterNumber(args[e]);
      if (regNumber >= 0) {
        // if this is a write operation
        if (dir[e] & WRITE) {
          continue;
        }
        auto updated = maps.find(args[e]);
        if (updated == maps.end()) {
          // error a virtual register is first read before it is written.
          cerr << "[line " << mo->getLineNumber()
               << "][virtualRemapping] a virtual register is first read before "
                  "it is written! "
               << registers::getName(args[e]) << endl;
          EXIT_ERROR;
        } else {
          // replace the old value with the new mapping
          args[e] = updated->second;
        }
      }
    }
  }
  // second time is to add a new mapping for the write registers!
  for (int e = 0; e < mo->getArgNumber(); ++e) {
    if (types[e] == REG) {
      int regNumber = registers::getVirtualRegisterNumber(args[e]);
      if (regNumber >= 0 && ((dir[e] & LOWERPART) == WRITE)) {
        // if we write to it, then we start a new remapping.
        maps[args[e]] = maxnumber; // the maxnumber++ only works,
        args[e] = maxnumber++;
        // because i know about the structure of the register ID's
      }
      if (regNumber >= 0 && ((dir[e] & LOWERPART) == READWRITE)) {
        auto updated = maps.find(args[e]);
        // since MAC Operations read an write from the same register, we get the
        // mapping which has been usd before and do not make new one.
        if (updated == maps.end()) {
          // error a virtual register is first read before it is written.
          cerr << "[line " << mo->getLineNumber()
               << "][virtualRemapping] a virtual register is first read before "
                  "it is written! "
               << registers::getName(args[e]) << endl;
          EXIT_ERROR;
          // if we write to it, then we start a new remapping.
          maps[args[e]] = maxnumber; // the maxnumber++ only works,
          // because i know about the structure of the register ID's
          args[e] = maxnumber++;
        } else {
          args[e] = updated->second;
        }
      }
    }
  }
}

void virtualRenaming(SLM *slm, Processor *pro) {
  // start by selecting the highest virtual register.
  int maxnumber = detectHighestVirtual(slm);

  vector<MO *> *ops = slm->getOperations();
  unordered_map<char32_t, char32_t> maps;
  for (auto m : *ops) {
    renameRegisters(m, maps, maxnumber);
  }
  if (slm->getBranchOperation())
    renameRegisters(slm->getBranchOperation(), maps, maxnumber);
}

void resetVirtualRegisterMap(VirtualRegisterMap *map, const int *regFile) {
  int i = 0;
  for (auto mit = map->begin(); mit != map->end(); ++mit, i++) {
    mit->second->setRegFile(regFile[i]);
    mit->second->setReal(-1);
  }
}

void printVirtualRegisterMap(VirtualRegisterMap *map) {
  for (auto &it : *map) {
    cout << registers::getName(it.first) << "-> ";
    cout << registers::getName(it.second->getReal()) << endl;
  }
}

int VirtualRegisterMapping::getFirstOccurenceID() {
  if (first_occurrence == std::vector<MI *>::iterator()) {
    return -1;
  }
  return (*first_occurrence)->getOperations()[0]->getID();
}
int VirtualRegisterMapping::getLastOccurenceID() {
  if (last_occurrence == std::vector<MI *>::iterator()) {
    return -1;
  }
  return (*last_occurrence)->getOperations()[0]->getID();
}