// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "rdg.h"
#include "portOptimalReg.h"
#include <fstream>

// #include "PortOptimalReg/Chromosome.h"

namespace rdg {

const char32_t RDG::NO_REG = static_cast<char32_t>(-1);
char32_t *RDG::DEFAULT_REG = 0;
unsigned int RDG::_maxReadPorts = 0;
unsigned int RDG::_maxWritePorts = 0;

#if DEBUG_INFEASIBLE_RDG
static unsigned int INFEASIBLE_RDG_COUNT = 0;

static Program *CURRENT_PROG;
static ass_reg_t *BLOCKED;

static void copyBlocked(ass_reg_t *blocked) {
  BLOCKED = new ass_reg_t[2];
  memcpy(BLOCKED, blocked, 2 * sizeof(ass_reg_t));
}

static void writeOutCurrentProg(std::ofstream &outFile) {
  if (!CURRENT_PROG)
    return;
  for (auto it = CURRENT_PROG->begin(); it != CURRENT_PROG->end(); ++it) {
    auto mi = *it;
    mi->writeOutReadable(outFile, 0);
  }
}

static void printBlocked(ostream &outStr) {
  for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
    outStr << "  ";
    ass_reg_t reg = BLOCKED[i];
    for (uint x = 0; x < registers::getNumRegisters(); x++) {
      outStr << (int)(1 & reg >> x);
      if (!((x + 1) % 4))
        outStr << ".";
    }
    outStr << endl;
  }
}

static void writeoutInfeasibleRDG(RDG *rdg, const std::string &message) {
  if (INFEASIBLE_RDG_COUNT >= MAX_INFEASIBLE_DEBUG) {
    LOG_OUTPUT(LOG_M_ALWAYS,
               "Reached maximum number of infeasible RDGs. STOPPING.\n");
    exit(-1);
  }

  LOG_OUTPUT(LOG_M_ALWAYS, "Writing infeasible RDG %d\n",
             ++INFEASIBLE_RDG_COUNT);
  ofstream outFile("infeasbileRDG_" + std::to_string(INFEASIBLE_RDG_COUNT) +
                   ".log");
  outFile << message << endl << endl;
  outFile << "Blocked registers:" << endl;
  printBlocked(outFile);
  outFile << endl;
  outFile << "Program:" << endl;
  writeOutCurrentProg(outFile);
  outFile << endl;
  outFile << "RDG:" << endl;
  rdg->printDot(outFile);
}
#endif

/* ********** The main routine ********************************************** */

RDG *RDG::analyseRegisterDependencies(Program *instructions,
                                      RegisterCoupling *couplings,
                                      VirtualRegisterMap *map,
                                      ass_reg_t *blocked, bool delOnError) {
#if DEBUG_INFEASIBLE_RDG
  CURRENT_PROG = instructions;
  copyBlocked(blocked);
#endif

  LOG_OUTPUT(LOG_M_RDG_DEBUG, "[RDG] Computing RDG for SLM:\n");
  if (isLog(LOG_M_RDG_DEBUG)) {
    stringstream ss;
    instructions->writeOutInstructions(ss, nullptr);
    LOG_OUTPUT(LOG_M_RDG_DEBUG, ss.str().c_str());
  }

  auto rdg = new RDG(map->size());

  LOG_OUTPUT(LOG_M_RDG_DETAIL | LOG_M_REG_ENERGY_DEBUG,
             "[RDG] Recording Register Couplings:\n");
  if (!rdg->recordRegisterCouplings(couplings) & delOnError) {
    delete rdg;
    return 0;
  }
  LOG_OUTPUT(LOG_M_RDG_DETAIL | LOG_M_REG_ENERGY_DEBUG,
             "[RDG] Analyzing instructions:\n");
  if (!rdg->analyseInstructions(instructions, map) & delOnError) {
    delete rdg;
    return 0;
  }
  rdg->compress();
  LOG_OUTPUT(LOG_M_RDG_DETAIL | LOG_M_REG_ENERGY_DEBUG,
             "[RDG] Checking feasibility\n");
  if (!rdg->checkFeasibility(instructions, map, blocked) & delOnError) {
    delete rdg;
    return 0;
  }

  LOG_OUTPUT(LOG_M_RDG_DETAIL, "[RDG] Done.\n\n");
  return rdg;
}

/* ********** Building the RDG ********************************************** */

bool RDG::recordRegisterCouplings(const RegisterCoupling *couplings) {
  for (auto it = couplings->begin(); it != couplings->end(); ++it) {
    auto couple = it->second;
    if (!recordRegisterCouple(couple)) {
#if DEBUG_INFEASIBLE_RDG
      writeoutInfeasibleRDG(
          this,
          "Invalid register coupling: " + registers::getName(couple->first) +
              " - " + registers::getName(couple->second));
#endif
      return false;
    }
  }
  return true;
}

bool RDG::analyseInstructions(Program *instructions, VirtualRegisterMap *map) {
  /* ATTENTION!
   * This code is written for a processor configuration with (exactly) two issue
   * slots!
   */

  for (auto iter = instructions->begin(); iter != instructions->end(); ++iter) {
    auto mi = *iter;

    LOG_OUTPUT(LOG_M_RDG_DEBUG, "] ");
    if (isLog(LOG_M_RDG_DEBUG))
      mi->writeOutReadable(cout, 0);

    MO *mo1 = mi->getOperations()[0];
    MO *mo2 = nullptr;
    if (mo1) {
      if (mo1->opLengthMultiplier() < (int)mi->getNumberIssueSlots())
        mo2 = mi->getOperations()[1];
    } else {
      if ((int)mi->getNumberIssueSlots() > 1) {
        mo2 = mi->getOperations()[1];
      }
    }

    if (mo1)
      countReadRegisters(mo1);
    if (mo2)
      countReadRegisters(mo2);
    if (!checkRegisterDependencies(mo1, mo2, map))
      return false;
  }
  return true;
}

bool RDG::recordDependency(char32_t reg1, char32_t reg2, bool same_rf,
                           int cond_RF) {
  if (reg1 >= _size || reg2 >= _size)
    return true;

  if (isLog(LOG_M_RDG_DEBUG)) {
    LOG_OUTPUT(LOG_M_RDG_DEBUG, "[RDG] Recording register dependency ");
    printEdge(reg1, reg2, same_rf, cond_RF);
    flushLog();
  }

  Entry *r_reg1 = find(reg1, same_rf);
  Entry *r_reg2 = find(reg2, same_rf);

  if (r_reg1 == r_reg2) {
    if (cond_RF == -1)
      return same_rf;
    else {
      if (r_reg1->isVirtual())
        return recordDependency(r_reg1->reg, getDefaultReg(cond_RF), same_rf,
                                -1);
      else
        return true;
    }
  } else {
    if (r_reg1->isPhysical()) {
      int reg1_regfile = registers::getRegFile(r_reg1->reg);
      if (r_reg2->isVirtual()) {
        if (cond_RF != -1 && reg1_regfile == cond_RF) {
          setParent(r_reg2, _entries[getDefaultReg(reg1_regfile)], same_rf, -1);
        } else if (cond_RF == -1)
          setParent(r_reg2, _entries[getDefaultReg(reg1_regfile)], same_rf, -1);
      } else
        return checkFixRegisters(r_reg1, r_reg2, same_rf);
    } else if (r_reg2->isPhysical()) {
      int reg2_regfile = registers::getRegFile(r_reg2->reg);
      if (r_reg1->isVirtual()) {
        if (cond_RF != -1 && reg2_regfile == cond_RF)
          setParent(r_reg1, _entries[getDefaultReg(reg2_regfile)], same_rf, -1);
        else if (cond_RF == -1)
          setParent(r_reg1, _entries[getDefaultReg(reg2_regfile)], same_rf, -1);
      } else
        return checkFixRegisters(r_reg1, r_reg2, same_rf);
    } else if (r_reg1->depth >= r_reg2->depth) {
      setParent(r_reg2, r_reg1, same_rf, cond_RF);

      if (isLog(LOG_M_RDG_DEBUG)) {
        if (cond_RF != -1)
          LOG_OUTPUT(LOG_M_ALWAYS, "[RDG]   (%d) { ", cond_RF);
        else
          LOG_OUTPUT(LOG_M_ALWAYS, "[RDG]   { ");
        printNode(r_reg2->reg, true);
        LOG_OUTPUT(LOG_M_ALWAYS, " -> ");
        printNode(r_reg1->reg, true);
        LOG_OUTPUT(LOG_M_ALWAYS, " %s }\n", (same_rf ? "=" : "~"));
      }
    } else {
      setParent(r_reg1, r_reg2, same_rf, cond_RF);

      if (isLog(LOG_M_RDG_DEBUG)) {
        if (cond_RF != -1)
          LOG_OUTPUT(LOG_M_ALWAYS, "[RDG]   (%d) { ", cond_RF);
        else
          LOG_OUTPUT(LOG_M_ALWAYS, "[RDG]   { ");
        printNode(r_reg1->reg, true);
        LOG_OUTPUT(LOG_M_ALWAYS, " -> ");
        printNode(r_reg2->reg, true);
        LOG_OUTPUT(LOG_M_ALWAYS, " %s }\n", (same_rf ? "=" : "~"));
      }
    }
    return true;
  }
}

bool RDG::recordWriteConflicts(const RDG::RegArgument &a1,
                               const RDG::RegArgument &a2, int cond_RF) {
  bool ok = true;
  ok &= recordDependency(a1.reg, a2.reg, RDG::DIFFERENT_RF, cond_RF);
  if (ok && a1.isX2()) {
    ok &= recordDependency(a1.reg_X2, a2.reg, RDG::DIFFERENT_RF, cond_RF);
    if (ok && a2.isX2()) {
      ok &= recordDependency(a1.reg, a2.reg_X2, RDG::DIFFERENT_RF, cond_RF);
      ok &= recordDependency(a1.reg_X2, a2.reg_X2, RDG::DIFFERENT_RF, cond_RF);
    }
  } else if (ok && a2.isX2())
    ok &= recordDependency(a1.reg, a2.reg_X2, RDG::DIFFERENT_RF, cond_RF);
  return ok;
}

bool RDG::recordRegisterCouple(const RegisterCouple *cpl) {
  char32_t r1 = cpl->first;
  char32_t r2 = cpl->second;
  _entries[r1]->couple = cpl;
  _entries[r2]->couple = cpl;
  return recordDependency(r1, r2, RDG::SAME_RF);
}

/* ********** Feasibility checks ******************************************** */

/* Perform feasibility checks on the program and the RDG. The RDG has to be
 * compressed! If this function detects that the register allocation for the
 * given program with the given RDG cannot be performed, the RDG is released
 * (deleted) and the function returns 0.
 */
bool RDG::checkFeasibility(Program *instructions, VirtualRegisterMap *map,
                           ass_reg_t *blockedRegs) {
  auto freeRegs = new int[registers::getNumRegisterFiles()];
  auto maxReadPorts = RDG::maxReadPorts();
  calcFreeReg(blockedRegs, freeRegs);
  auto freeRegCount = freeRegs[0] + freeRegs[1];

  LOG_OUTPUT(LOG_M_RDG_DEBUG | LOG_M_REG_ENERGY_DEBUG,
             "Feasibility check for RDG:\n");
  LOG_OUTPUT(LOG_M_RDG_DEBUG | LOG_M_REG_ENERGY_DEBUG,
             "] Blocked registers:\n");
  if (isLog(LOG_M_RDG_DEBUG | LOG_M_REG_ENERGY_DEBUG))
    printBlockedRegisters(blockedRegs, freeRegs);
  LOG_OUTPUT(LOG_M_RDG_DEBUG | LOG_M_REG_ENERGY_DEBUG, "\n");
  LOG_OUTPUT(LOG_M_RDG_DEBUG | LOG_M_REG_ENERGY_DEBUG,
             "] Free registers: %d + %d = %d\n", freeRegs[0], freeRegs[1],
             freeRegCount);

  int liveRegs = 0;
  int lineNumber = 0;
  for (auto miIt = instructions->begin(); miIt != instructions->end(); ++miIt) {
    std::set<char32_t> dying;

    auto mi = *miIt;

    LOG_OUTPUT(LOG_M_RDG_DEBUG | LOG_M_REG_ENERGY_DEBUG,
               "] Checking instruction\n");
    if (isLog(LOG_M_RDG_DEBUG | LOG_M_REG_ENERGY_DEBUG))
      mi->writeOutReadable(cout);

    auto registerArgs = collectRegisterArgs(mi);
    unsigned int registerCount = registerArgs.size();

    for (unsigned int argIdx = 0; argIdx < registerCount; ++argIdx) {
      char32_t arg = registerArgs[argIdx].reg;
      updateLiveRegs(miIt, arg, map, dying, liveRegs);
      if (!checkReadPorts(registerArgs, registerCount, maxReadPorts, argIdx)) {
        LOG_OUTPUT(LOG_M_RDG | LOG_M_REG_ENERGY,
                   "Too few read ports -> deleting RDG\n");
        params.best_sched_length = lineNumber;
        delete[] freeRegs;
#if DEBUG_INFEASIBLE_RDG
        writeoutInfeasibleRDG(this, "Using too many read ports in MI " +
                                        mi->to_string());
#endif
        return false;
      }
    }

    LOG_OUTPUT(LOG_M_RDG_DEBUG | LOG_M_REG_ENERGY_DEBUG,
               "] We have %d live registers\n", liveRegs);
    if (liveRegs > freeRegCount) {
      LOG_OUTPUT(LOG_M_RDG | LOG_M_REG_ENERGY_DEBUG,
                 "Not enough registers -> deleting RDG\n");

      LOG_OUTPUT(LOG_M_RDG_DEBUG, "]] liveRegs = %d\n", liveRegs);
      LOG_OUTPUT(LOG_M_RDG_DEBUG, "]] freeRegs = %d + %d = %d\n", freeRegs[0],
                 freeRegs[1], freeRegCount);

#if DEBUG_INFEASIBLE_RDG
      writeoutInfeasibleRDG(this, "Not enough registers\n  (MI " +
                                      mi->to_string() + ")");
#endif

      params.best_sched_length = lineNumber;
      delete[] freeRegs;
      return false;
    }
    lineNumber++;
  }
  delete[] freeRegs;
  return true;
}

bool RDG::detectUsedPorts(const vector<RegArgument> &args,
                          unsigned int *portCounts) {
  for (auto &arg : args) {
    if (arg.isPhysical()) {
      int regfile = registers::getRegFile(arg.reg);
      if (portCounts[regfile]-- < 0)
        return false;
      if (arg.isX2())
        if (portCounts[regfile]-- < 0)
          return false;
    }
  }
  return true;
}

bool RDG::checkPortConflicts(const RegArgument &a, unsigned int *freePorts) {
  if (a.isVirtual()) {
    for (int regfile = 0; regfile <= 1; ++regfile) {
      if (freePorts[regfile] < 1 || (a.isX2() && freePorts[regfile] < 2)) {
        if (!recordDependency(a.reg, getDefaultReg(1 - regfile), RDG::SAME_RF,
                              -1)) {
          LOG_OUTPUT(LOG_M_RDG_DEBUG, "[RDG] Portconflict between ");
          if (isLog(LOG_M_RDG_DEBUG))
            printNode(a.reg, true);
          LOG_OUTPUT(LOG_M_RDG_DEBUG, " and V%dR0\n", regfile);
          return false;
        }
        if (freePorts[1 - regfile]-- < 0)
          return false;
        if (a.isX2() && freePorts[1 - regfile] < 0)
          return false;
        return true;
      }
    }
  }
  return true;
}

/*! \brief Check register dependencies of two MOs which are executed in
 * parallel. */
bool RDG::checkRegisterDependencies(MO *mo1, MO *mo2, VirtualRegisterMap *map) {
  auto writeArgs1 = RDG::collectWriteArgs(mo1);
  auto readArgs1 = RDG::collectReadArgs(mo1);
  auto writeArgs2 = RDG::collectWriteArgs(mo2);
  auto readArgs2 = RDG::collectReadArgs(mo2);

  unsigned int freeReadPorts[] = {registers::getReadPorts(0),
                                  registers::getReadPorts(1)};
  unsigned int freeWritePorts[] = {registers::getWritePorts(0),
                                   registers::getWritePorts(1)};

  if (!detectUsedPorts(writeArgs1, freeWritePorts) ||
      !detectUsedPorts(writeArgs2, freeWritePorts) ||
      !detectUsedPorts(readArgs1, freeReadPorts) ||
      !detectUsedPorts(readArgs2, freeReadPorts)) {
#if DEBUG_INFEASIBLE_RDG
    writeoutInfeasibleRDG(this, "Port conflicts at MI " +
                                    std::to_string(mo1->getLineNumber()) + "+" +
                                    std::to_string(mo2->getLineNumber()));
#endif
    return false;
  }

  for (auto &a : writeArgs1) {
    if (!checkPortConflicts(a, freeWritePorts)) {
#if DEBUG_INFEASIBLE_RDG
      writeoutInfeasibleRDG(this, "Write port conflicts with argument " +
                                      registers::getName(a.reg) + " in MI " +
                                      std::to_string(mo1->getLineNumber()) +
                                      "+" +
                                      std::to_string(mo2->getLineNumber()));
#endif
      return false;
    }

    if (a.isX2() && RDG::maxWritePorts() <= 2) {
      for (auto &a2 : writeArgs2)
        if (!recordWriteConflicts(a, a2, -1)) {
#if DEBUG_INFEASIBLE_RDG
          writeoutInfeasibleRDG(this,
                                "Write conflicts with registers " +
                                    registers::getName(a.reg) + " and " +
                                    registers::getName(a2.reg) + " in MI " +
                                    std::to_string(mo1->getLineNumber()) + "+" +
                                    std::to_string(mo2->getLineNumber()));
#endif
          return false;
        }
    } else if (registers::getWritePorts(1) < 2) {
      for (auto &a2 : writeArgs2)
        if (!recordWriteConflicts(a, a2, 1)) {
#if DEBUG_INFEASIBLE_RDG
          writeoutInfeasibleRDG(this,
                                "Write conflicts with registers " +
                                    registers::getName(a.reg) + " and " +
                                    registers::getName(a2.reg) + " in MI " +
                                    std::to_string(mo1->getLineNumber()) + "+" +
                                    std::to_string(mo2->getLineNumber()));
#endif
          return false;
        }
    }
  }
  for (auto &a : readArgs1) {
    if (!checkPortConflicts(a, freeReadPorts)) {
#if DEBUG_INFEASIBLE_RDG
      writeoutInfeasibleRDG(this, "Read port conflicts with argument " +
                                      registers::getName(a.reg) + " in MI " +
                                      std::to_string(mo1->getLineNumber()) +
                                      "+" +
                                      std::to_string(mo2->getLineNumber()));
#endif
      return false;
    }
  }
  for (auto &a : writeArgs2) {
    if (!checkPortConflicts(a, freeWritePorts)) {
#if DEBUG_INFEASIBLE_RDG
      writeoutInfeasibleRDG(this, "Write port conflicts with argument " +
                                      registers::getName(a.reg) + " in MI " +
                                      std::to_string(mo1->getLineNumber()) +
                                      "+" +
                                      std::to_string(mo2->getLineNumber()));
#endif
      return false;
    }

    if (a.isX2() && RDG::maxWritePorts() <= 2) {
      for (auto &a1 : writeArgs1)
        if (!recordWriteConflicts(a1, a, -1)) {
#if DEBUG_INFEASIBLE_RDG
          writeoutInfeasibleRDG(this,
                                "Write conflicts with registers " +
                                    registers::getName(a.reg) + " and " +
                                    registers::getName(a1.reg) + " in MI " +
                                    std::to_string(mo1->getLineNumber()) + "+" +
                                    std::to_string(mo2->getLineNumber()));
#endif
          return false;
        }
    } else if (registers::getWritePorts(1) < 2) {
      for (auto &a1 : writeArgs1)
        if (!recordWriteConflicts(a1, a, 1)) {
#if DEBUG_INFEASIBLE_RDG
          writeoutInfeasibleRDG(this,
                                "Write conflicts with registers " +
                                    registers::getName(a.reg) + " and " +
                                    registers::getName(a1.reg) + " in MI " +
                                    std::to_string(mo1->getLineNumber()) + "+" +
                                    std::to_string(mo2->getLineNumber()));
#endif
          return false;
        }
    }
  }
  for (auto &a : readArgs2) {
    if (!checkPortConflicts(a, freeReadPorts)) {
#if DEBUG_INFEASIBLE_RDG
      writeoutInfeasibleRDG(this, "Read port conflicts with argument " +
                                      registers::getName(a.reg) + " in MI " +
                                      std::to_string(mo1->getLineNumber()) +
                                      "+" +
                                      std::to_string(mo2->getLineNumber()));
#endif
      return false;
    }
  }
  return true;
}

bool RDG::checkReadPorts(std::vector<RDG::RegisterArg> &registerArgs,
                         unsigned int registerCount, unsigned int maxReadPorts,
                         unsigned int argIdx) {
  RegisterArg arg = registerArgs[argIdx];
  if (!registers::isPhysicalRegister(arg.reg) &&
      !registers::isVirtualReg(arg.reg))
    return true;
  unsigned int readPortCount = 0;
  if (arg.type == REG && (arg.dir == READ || arg.dir == READWRITE ||
                          arg.dir == READWRITEPSEUDOREAD)) {
    for (unsigned int secondArgIdx = argIdx + 1; secondArgIdx < registerCount;
         ++secondArgIdx) {
      RegisterArg secondArg = registerArgs[secondArgIdx];
      if ((registers::isPhysicalRegister(secondArg.reg) ||
           registers::isVirtualReg(secondArg.reg)) &&
          getDependency(arg.reg, secondArg.reg) == RDG::DEP_SAME_RF)
        ++readPortCount;
    }
    return readPortCount <= maxReadPorts;
  } else
    return true;
}

void RDG::countReadRegisters(MO *mo) {
  char32_t *args = mo->getArguments();
  OPtype *types = mo->getTypes();
  char *dirs = mo->getDirections();
  for (int i = 0; i < 8; ++i) {
    if (types[i] == REG && dirs[i] == READ &&
        registers::isVirtualReg(args[i])) {
      _entries[args[i]]->readCount++;
    }
  }
}

bool RDG::checkFixRegisters(Entry *r1, Entry *r2, bool same_rf) {
  int regfile1 = registers::getRegFile(r1->reg);
  int regfile2 = registers::getRegFile(r2->reg);
  return same_rf ? regfile1 == regfile2 : regfile1 != regfile2;
}

/* ********** Utility Methods *********************************************** */

void RDG::collectArgs(MO *mo, std::vector<RegArgument> &arguments,
                      std::function<bool(char)> filter) {
  auto args = mo->getArguments();
  auto types = mo->getTypes();
  auto dirs = mo->getDirections();

  for (int i = 0; i < 4; ++i) {
    if (filter(dirs[i]) && types[i] == REG) {
      if (types[i + 4] == REG && args[i] != args[i + 4])
        arguments.push_back(RegArgument(args[i], args[i + 4]));
      else
        arguments.push_back(RegArgument(args[i]));
    }
  }
}

std::vector<RDG::RegArgument> RDG::collectWriteArgs(MO *mo) {
  std::vector<RegArgument> arguments;
  if (mo)
    collectArgs(mo, arguments, [](char t) {
      return t == WRITE || t == READWRITE || t == READWRITEPSEUDOREAD;
    });
  return arguments;
}

std::vector<RDG::RegArgument> RDG::collectReadArgs(MO *mo) {
  std::vector<RegArgument> arguments;
  if (mo)
    collectArgs(mo, arguments, [](char t) {
      return t == READ || t == READWRITE || t == READWRITEPSEUDOREAD;
    });
  return arguments;
}

std::vector<RDG::RegisterArg> RDG::collectRegisterArgs(MI *mi) {
  std::vector<RegisterArg> registerArgs;
  std::set<char32_t> processed;
  unsigned int slot = 0;
  while (slot < mi->getNumberIssueSlots()) {
    auto mo = mi->getOperations()[slot];
    if (!mo) {
      ++slot;
      continue;
    }

    auto args = mo->getArguments();
    auto types = mo->getTypes();
    auto dirs = mo->getDirections();
    for (auto argIdx = 0; argIdx < MAXARGNUMBER; ++argIdx) {
      auto arg = args[argIdx];
      if (types[argIdx] == REG && processed.insert(arg).second) {
        RegisterArg a;
        a.reg = arg;
        a.type = types[argIdx];
        a.dir = dirs[argIdx];
        registerArgs.push_back(a);
      }
    }
    slot += mo->opLengthMultiplier();
  }
  return registerArgs;
}

void RDG::updateLiveRegs(Program::iterator miIt, char32_t arg,
                         VirtualRegisterMap *map, std::set<char32_t> &dying,
                         int &liveRegs) {
  auto mapIt = map->find(arg);
  if (mapIt != map->end()) {
    auto mapping = mapIt->second;
    if (mapping->getFirstOccurrence() == miIt) {
      LOG_OUTPUT(LOG_M_RDG_DEBUG, "] Register %s comes to life\n",
                 registers::getName(arg).c_str());
      ++liveRegs;
    }
    if (mapping->getLastOccurrence() == miIt) {
      if (dying.insert(arg).second) {
        LOG_OUTPUT(LOG_M_RDG_DEBUG, "] Register %s dies\n",
                   registers::getName(arg).c_str());
        --liveRegs;
      } else {
        // already dead!
        LOG_OUTPUT(LOG_M_RDG_DEBUG, "] Register %s is already dead\n",
                   registers::getName(arg).c_str());
      }
    }
  }
}

// --------------- RDG implementation
// -----------------------------------------------------

/* The encoding of registers (see registers::parseRegister) assigns
 * 0..(numRegisterFiles * numRegisters) to fix registers, then there's a gap of
 * numRegisters numbers, and then the virtual registers follow. With two
 * register files of 32 registers, virtual registers start at ID 96.
 */
RDG::RDG(unsigned int size) : _rootCount(0) {
  _size =
      (registers::getNumRegisterFiles() + 1) * registers::getNumRegister1RF() +
      1 + size;
  _entries = new Entry *[_size];
  for (unsigned int i = 0; i < _size; ++i) {
    _entries[i] = new Entry(static_cast<char32_t>(i));
  }
}

RDG::~RDG() {
  for (unsigned int i = 0; i < _size; ++i)
    delete _entries[i];
  delete[] _entries;
}

RDG::Entry *RDG::find(char32_t reg, bool &same_rf) const {
  Entry *e = _entries[reg];
  while (e->parent) {
    if (!e->same_rf)
      same_rf = !same_rf;
    e = e->parent;
  }
  return e;
}

RDG::DEP_TYPE RDG::getDependency(char32_t reg1, char32_t reg2) const {
  bool same_rf = true;
  Entry *e1 = find(reg1, same_rf);
  Entry *e2 = find(reg2, same_rf);
  if (e1 == e2)
    return same_rf ? RDG::DEP_SAME_RF : RDG::DEP_DIFFERENT_RF;
  else
    return RDG::DEP_NO_DEP;
}

void RDG::printNode(char32_t reg, bool in_edge) const {
  unsigned int registerFiles = registers::getNumRegisterFiles();
  unsigned int regCount = registers::getNumRegister1RF();
  if (!in_edge && reg >= registerFiles * regCount &&
      reg < (registerFiles + 1) * regCount)
    return;
  if (reg < registers::getNumRegister1RF())
    LOG_OUTPUT(LOG_M_ALWAYS, "V0R%d", registers::getRegNumber(reg));
  else if (reg < registerFiles * registers::getNumRegister1RF())
    LOG_OUTPUT(LOG_M_ALWAYS, "V1R%d", registers::getRegNumber(reg));
  else
    LOG_OUTPUT(LOG_M_ALWAYS, "VxR%d", registers::getVirtualRegisterNumber(reg));
  if (!in_edge)
    LOG_OUTPUT(LOG_M_ALWAYS, ";\n");
}

void RDG::printNode(ostream &outStr, char32_t reg, bool in_edge) const {
  unsigned int registerFiles = registers::getNumRegisterFiles();
  unsigned int regCount = registers::getNumRegister1RF();
  if (!in_edge && reg >= registerFiles * regCount &&
      reg < (registerFiles + 1) * regCount)
    return;
  outStr << registers::getName(reg);
  if (!in_edge)
    outStr << ";" << endl;
}

void RDG::printEdge(char32_t reg1, char32_t reg2, bool same_rf,
                    int cond_RF) const {
  printNode(reg1, true);
  LOG_OUTPUT(LOG_M_ALWAYS, " -> ");
  printNode(reg2, true);
  LOG_OUTPUT(LOG_M_ALWAYS, " [label=\"%s\"", (same_rf ? "=" : "~"));
  if (cond_RF == 0)
    LOG_OUTPUT(LOG_M_ALWAYS, ", color=\"green\"");
  else if (cond_RF == 1)
    LOG_OUTPUT(LOG_M_ALWAYS, ", color=\"red\"");
  LOG_OUTPUT(LOG_M_ALWAYS, "]\n");

  printNode(reg1, true);
  LOG_OUTPUT(LOG_M_ALWAYS, "[label=\"");
  printNode(reg1, true);
  LOG_OUTPUT(LOG_M_ALWAYS, "\\n%d\"];\n", _entries[reg1]->index);

  printNode(reg2, true);
  LOG_OUTPUT(LOG_M_ALWAYS, "[label=\"");
  printNode(reg2, true);
  LOG_OUTPUT(LOG_M_ALWAYS, "\\n%d\"];\n", _entries[reg2]->index);
}

void RDG::printEdge(ostream &outStr, char32_t reg1, char32_t reg2, bool same_rf,
                    int cond_RF) const {
  printNode(outStr, reg1, true);
  outStr << " -> ";
  printNode(outStr, reg2, true);
  outStr << " [label=\"" << (same_rf ? "=" : "~") << "\"";
  if (cond_RF == 0)
    outStr << ", color=\"green\"";
  else if (cond_RF == 1)
    outStr << ", color=\"red\"";
  outStr << "]" << endl;

  printNode(outStr, reg1, true);
  outStr << "[label=\"";
  printNode(outStr, reg1, true);
  outStr << "\\n" << _entries[reg1]->index << "\"];" << endl;

  printNode(outStr, reg2, true);
  outStr << "[label=\"";
  printNode(outStr, reg2, true);
  outStr << "\\n" << _entries[reg2]->index << "\"];" << endl;
}

void RDG::setParent(Entry *sub, Entry *parent, bool same_rf, int cond_RF) {
  if (cond_RF == -1) {
    //    sub->parent = parent;
    sub->setParent(parent);
    sub->same_rf = same_rf;
    parent->depth += sub->depth;
  } else {
    sub->cond_parent = parent;
    sub->same_rf_cond = same_rf;
    sub->cond_rf = cond_RF;
  }
}

void RDG::compress() {
  _rootCount = 0;
  for (unsigned int i = 0; i < _size; ++i) {
    Entry *e = _entries[i];
    if (!e->parent && e->isVirtual()) // this is a root
      e->index = _rootCount++; // position in the register allocation chromosome
    compress(e);
  }
}

void RDG::compress(Entry *e) {
  if (e->parent && e->parent->parent) {
    if (!e->parent->same_rf)
      e->same_rf = !e->same_rf;
    e->parent = e->parent->parent;

    //        LOG_OUTPUT(LOG_M_ALWAYS, "[RDG::compress] Setting parent of ");
    //        printNode(e->reg, true);
    //        LOG_OUTPUT(LOG_M_ALWAYS, " to ");
    //        printNode(e->parent->reg, true);
    //        LOG_OUTPUT(LOG_M_ALWAYS, "\n");

    compress(e);
    return;
  }
  if (e->cond_parent && e->cond_parent->parent) {
    Entry *o = e->cond_parent->parent;
    if (!e->cond_parent->same_rf) {
      e->same_rf_cond = !e->same_rf_cond;
      e->cond_rf = 1 - e->cond_rf;
    }

    if (o->isVirtual()) {
      e->cond_parent = o;

      //            LOG_OUTPUT(LOG_M_ALWAYS, "[RDG::compress] Setting
      //            conditional parent of "); printNode(e->reg, true);
      //            LOG_OUTPUT(LOG_M_ALWAYS, " to ");
      //            printNode(e->cond_parent->reg, true);
      //            LOG_OUTPUT(LOG_M_ALWAYS, "\n");

      compress(e);
      return;
    } else {
      int regfile = registers::getRegFile(o->reg);
      if (e->cond_rf != regfile) {
        //                LOG_OUTPUT(LOG_M_ALWAYS, "[RDG::compress] Deleting
        //                conditional parent of "); printNode(e->reg, true);
        //                LOG_OUTPUT(LOG_M_ALWAYS, "\n");
        e->cond_parent = 0;
      } else {
        e->parent = o;
        e->same_rf = e->same_rf_cond;
        e->cond_parent = 0;

        //                LOG_OUTPUT(LOG_M_ALWAYS, "[RDG::compress] Switching
        //                conditional parent of "); printNode(e->reg, true);
        //                LOG_OUTPUT(LOG_M_ALWAYS, " to unconditional parent ");
        //                printNode(e->parent->reg, true);
        //                LOG_OUTPUT(LOG_M_ALWAYS, "\n");
      }
    }
  }
}

int RDG::rootCount() const { return _rootCount; }

int RDG::virtualRegCount() const {
  return _size - ((registers::getNumRegisterFiles() + 1) *
                  registers::getNumRegister1RF());
}

const RDG::Entry *RDG::getEntry(char32_t reg) const {
  try {
    //    cout << reg << endl;
    return (reg < _size) ? _entries[reg] : 0;
  } catch (const std::exception &e) {
    cout << "Caught Exception: " << e.what() << endl;
    return 0;
  }
}

const RDG::Entry *RDG::getRootEntry(char32_t reg) const {
  bool same_rf = true;
  return getRootEntry(reg, same_rf);
}

const RDG::Entry *RDG::getRootEntry(char32_t reg, bool &same_rf) const {
  const Entry *e = getEntry(reg);
  while (e && !e->isRoot()) {
    same_rf = e->same_rf;
    e = e->parent;
  }
  return e;
}

const RDG::Entry *RDG::getEntryByIndex(unsigned int index) const {
  return _entries[index];
}

void RDG::printDot() const {
  LOG_OUTPUT(LOG_M_ALWAYS, "digraph {\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "  rankdir = \"BT\"\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "  page = \"8,11.5\"\n");

  for (unsigned int i = 0; i < _size; ++i) {
    Entry *e1 = _entries[i];
    if (e1->parent)
      printEdge(e1->reg, e1->parent->reg, e1->same_rf, -1);
    if (e1->cond_parent)
      printEdge(e1->reg, e1->cond_parent->reg, e1->same_rf_cond, e1->cond_rf);
  }

  LOG_OUTPUT(LOG_M_ALWAYS, "}\n");
}

void RDG::printDot(ostream &outStr) const {
  outStr << "digraph {\n  rankdir = \"BT\"\n  page = \"8,11.5\"" << endl;

  for (unsigned int i = 0; i < _size; ++i) {
    Entry *e1 = _entries[i];
    if (e1->parent)
      printEdge(outStr, e1->reg, e1->parent->reg, e1->same_rf, -1);
    if (e1->cond_parent)
      printEdge(outStr, e1->reg, e1->cond_parent->reg, e1->same_rf_cond,
                e1->cond_rf);
  }
  outStr << "}" << endl;
}

int RDG::maxDepth() const {
  int depth = 1;
  for (unsigned int i = 0; i < _size; ++i) {
    int d = _entries[i]->treeDepth();
    if (d > depth)
      depth = d;
  }
  return depth;
}

unsigned int RDG::maxReadPorts() {
  if (_maxReadPorts == 0) {
    for (auto i = 0u; i < registers::getNumRegisterFiles(); ++i) {
      if (registers::getReadPorts(i) > _maxReadPorts)
        _maxReadPorts = registers::getReadPorts(i);
    }
  }
  return _maxReadPorts;
}

unsigned int RDG::maxWritePorts() {
  if (_maxWritePorts == 0) {
    for (auto i = 0u; i < registers::getNumRegisterFiles(); ++i) {
      if (registers::getWritePorts(i) > _maxWritePorts)
        _maxWritePorts = registers::getWritePorts(i);
    }
  }
  return _maxWritePorts;
}

char32_t RDG::getDefaultReg(int regfile) {
  if (!RDG::DEFAULT_REG) {
    RDG::DEFAULT_REG = new char32_t[registers::getNumRegisterFiles()];
    for (unsigned int i = 0; i < registers::getNumRegisterFiles(); ++i)
      RDG::DEFAULT_REG[i] = registers::createRegister(i, 0);
  }
  return RDG::DEFAULT_REG[regfile];
}

void RDG::setAllocation(int *allocations, const int *genes, const Entry *e,
                        bool *visited, int depth) const {
  if (depth > 200) {
    LOG_OUTPUT(LOG_M_ALWAYS,
               "[RDG] Reached depth %d in setAllocation. Trying to set "
               "allocation for %s!\n",
               depth, registers::getName(e->reg).c_str());
    this->printDot();
    exit(-1);
  }
  if (!e)
    return;
  if (allocations[e->reg] != -1)
    return;

  if (visited[e->reg]) { // cycle detection!
    allocations[e->reg] = genes[e->index];
    return;
  }
  visited[e->reg] = true;

  if (e->parent) {
    setAllocation(allocations, genes, e->parent, visited, depth + 1);
    int parent_rf = allocations[e->parent->reg];
    allocations[e->reg] = e->same_rf ? parent_rf : 1 - parent_rf;
  } else if (e->cond_parent) {
    setAllocation(allocations, genes, e->cond_parent, visited, depth + 1);
    int parent_rf = allocations[e->cond_parent->reg];
    if (e->cond_rf == parent_rf)
      allocations[e->reg] = e->same_rf_cond ? parent_rf : 1 - parent_rf;
    else
      allocations[e->reg] = genes[e->index];
  } else
    allocations[e->reg] = genes[e->index];
}

int *RDG::deriveAllocations(const portOptReg::Chromosome *chrm) const {
  const unsigned int VxOffset =
      (registers::getNumRegisterFiles() + 1) * registers::getNumRegister1RF();
  /** Allocate Register File for each spot **/
  int *allocations = new int[_size];
  bool *visited = new bool[_size];
  for (unsigned int i = 0; i < _size; ++i) {
    if (i < registers::getNumRegister1RF())
      allocations[i] = 0;
    else if (i < (registers::getNumRegisterFiles() *
                  registers::getNumRegister1RF()))
      allocations[i] = 1;
    else
      allocations[i] = -1;
    visited[i] = false;
  }
  for (unsigned int i = VxOffset; i < _size; ++i) {
    setAllocation(allocations, chrm->genes(), _entries[i], visited);
  }
  delete[] visited;
  return allocations;
}

} // namespace rdg
