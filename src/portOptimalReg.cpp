// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "portOptimalReg.h"
#include "CompilerContext.h"
#include "DoubleRegister.h"
#include "EnergyRegisterAllocation/EnergyAllocation.h"
#include "Origin.h"
#include "ga_stats.h"
#include "gen_merging.h"
#include "utility.h"
#include <algorithm>
#include <cmath>

using namespace std;
using namespace rdg;

#if REG_GA_STATS
reg_ga_stats::RegGAStat regGAStats[MAX_THREAD_COUNT];
#endif

namespace portOptReg {

/** \brief Try allocating all virtual registers in the MI */
static void allocateRegisters(MI *ins, const rdg::RDG &rdg,
                              ass_reg_t *blockedRegistes, int *blockedCount,
                              Chromosome *chrm, const int *allocations);
/** \brief Check the number of read and write port for register access (update
 * chrm->fitCounters()) */
static void checkPortConflicts(MI *ins, Chromosome *chrm,
                               const int *blockedCount, const rdg::RDG &rdg);
/** \brief Release blocked registers, if virtual register is used no longer */
static void freeRegisters(ass_reg_t *blocked, int *blockedCount,
                          Instructions it, VirtualRegisterMap *map,
                          VirtualAllocation *regMapping);

std::unordered_map<int, int> successful_RA_hist, failed_RA_hist;

void clearRAHists() {
  successful_RA_hist.clear();
  failed_RA_hist.clear();
}

static void printRAHist(const std::unordered_map<int, int> &hist) {
  std::map<int, int> ordered(hist.begin(), hist.end());
  for (auto it = ordered.begin(); it != ordered.end(); ++it) {
    LOG_OUTPUT(LOG_M_ALWAYS, "Generations %3d count %4d\n", it->first,
               it->second);
  }
}

void printRAHists() {
  LOG_OUTPUT(LOG_M_ALWAYS, "Successful Register Allocations:\n");
  printRAHist(successful_RA_hist);
  LOG_OUTPUT(LOG_M_ALWAYS, "Failed Register Allocations:\n");
  printRAHist(failed_RA_hist);
}

static void finishedRA(int generation, int fitness) {
  if (fitness == 0)
    successful_RA_hist[generation] += 1;
  else
    failed_RA_hist[generation] += 1;
}

// *************************************************************************************************************************

static int getPopulationSize(int registerCount) {
  int popsize = registerCount;
  float shift = (params.portReg / 8.0) - 5;
  popsize = popsize * pow(2, shift);
  popsize = max(5, popsize);
  return popsize;
}

static int getPopulationSizeEnergyAllocation() {
  float shift = 2 * (params.raRegPopSize) - 1;
  int popsize = pow(2, shift);
  popsize = max(8, popsize);
  return popsize;
}

static int getTournamentSize() { return params.portRegTournament; }

static float getMutationProbability() { return params.raMutationRate; }

/**
 * \brief Allocate and block virtual registers that are coupled to fix
 * registers.
 *
 * Go throught the register dependency graph and find virtual registers, that
 * are coupled to fix registers. For those, block them and preallocate them to
 * the corresponding fix register.
 */
bool preallocateCoupled(const RDG &rdg, VirtualAllocation *allocation,
                        ass_reg_t *blocked) {
  char32_t V0R0 =
      (registers::getNumRegisterFiles() + 1) * registers::getNumRegister1RF();
  for (int regIndex = 0; regIndex < rdg.virtualRegCount(); ++regIndex) {
    char32_t reg = V0R0 + regIndex;
    auto regEntry = rdg.getEntry(reg);
    if (regEntry && !regEntry->isRoot() && regEntry->isVirtual()) {
      auto couple = regEntry->couple;
      if (couple) {
        auto reg1 = couple->first;
        auto reg2 = couple->second;
        char32_t virtualReg;
        int fixedNumber, fixedFile;
        if (registers::isVirtualReg(reg1) &&
            registers::isPhysicalRegister(reg2)) {
          if (!gen_X2::getMatchingX2Reg(reg2, gen_X2::REG_PAIR_POS::SECOND,
                                        couple->concurrent, fixedNumber,
                                        fixedFile))
            return false;
          virtualReg = reg1;
        } else if (registers::isPhysicalRegister(reg1) &&
                   registers::isVirtualReg(reg2)) {
          if (!gen_X2::getMatchingX2Reg(reg1, gen_X2::REG_PAIR_POS::FIRST,
                                        couple->concurrent, fixedNumber,
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
          (*allocation)[virtualReg] =
              registers::createRegister(fixedFile, fixedNumber);
        } else {
          LOG_OUTPUT(LOG_M_RA_DEBUG,
                     "Could not preallocate %s to V%dR%d (already blocked)\n",
                     registers::getName(virtualReg).c_str(), fixedFile,
                     fixedNumber);
          return false;
        }
      }
    }
  }
  return true;
}

static int countUsedRegs(ass_reg_t blocked) {
  int count = 0;
  while (blocked > 0) {
    count += (blocked & 1);
    blocked >>= 1;
  }
  return count;
}

/* For the fitness value, count the number of conflicts in read and write ports
 * for the register files (r0_read, r0_write, r1_read, r1_write). Also note the
 * number of extra registers needed (extra_registers). Then compute some nice
 * value out of all these numbers...
 *
 * Maybe we need two fitness values:
 * - fitness used in evolution: weight of port conflicts high, extra_registers
 * low,
 * - something we report back to scheduling: weight of extra_registers high,
 * port conflicts low (we believe: scheduling has not such a big influence on
 * port conflicts, therefore extra_registers provides more useful info to
 * scheduling)
 *
 * Number of port conflicts per register file could also be used to influence
 * the algorithm (e.g., mutation) in order to better distribute the accesses
 * (balancing)
 */
static int fitness(Chromosome *chrm, Program *ins, VirtualRegisterMap *map,
                   const RDG &rdg, ass_reg_t *blockedRegisters,
                   Processor *pro) {
  ass_reg_t *blocked = new ass_reg_t[registers::getNumRegisterFiles()];
  memcpy(blocked, blockedRegisters,
         registers::getNumRegisterFiles() * sizeof(ass_reg_t));
  int blockedCount[2];
  blockedCount[0] = countUsedRegs(blocked[0]);
  blockedCount[1] = countUsedRegs(blocked[1]);

  chrm->regMapping() = new VirtualAllocation();
  FitnessCounters &fit = chrm->fitCounters();
  int balance = 0;
  int extraReg0 = fit.extra_regs[0];
  int extraReg1 = fit.extra_regs[1];
  if (!preallocateCoupled(rdg, chrm->regMapping(), blocked))
    fit.extra_regs[0] += 100; // Incompatible fix+virtual register pair
  else {
#if CHECK_RA_TIMING
    ra_timings.geneticTimer.start();
#endif
    int *allocations = rdg.deriveAllocations(chrm);

    for (Instructions it = ins->begin(); it != ins->end(); ++it) {
      MI *mi = *it;
      allocateRegisters(mi, rdg, blocked, blockedCount, chrm, allocations);
      checkPortConflicts(mi, chrm, blockedCount, rdg);
      LOG_OUTPUT(LOG_M_RA_DEBUG, "[PortOptReg] [alloc] Conflicts so far: [");
      if (DEBUG_VIRTUAL_ALLOC)
        chrm->printConflicts();
      LOG_OUTPUT(LOG_M_RA_DEBUG, "]\n");
      //        if (!checkMapping(map, chrm->regMapping(), pro, it, it))
      //            chrm->fitCounters().notExecutable++;

      freeRegisters(blocked, blockedCount, it, map, chrm->regMapping());
      if (extraReg0 < fit.extra_regs[0] || extraReg1 < fit.extra_regs[1]) {
        balance += abs(blockedCount[0] - blockedCount[1]);
      }
      extraReg0 = fit.extra_regs[0];
      extraReg1 = fit.extra_regs[1];
    }

    delete[] allocations;
#if CHECK_RA_TIMING
    ra_timings.geneticTimer.stop();
    ra_timings.geneticCount++;
#endif
  }
  delete[] blocked;

  int conflWeighted = 0;
  for (size_t i = 0; i < rdg.size(); ++i) {
    const RDG::Entry *e = rdg.getEntryByIndex(i);
    const RDG::Entry *root = (e->isRoot()) ? e : e->parent;

    if (root && root->index > -1) {
      conflWeighted +=
          (chrm->conflictCount(root->index) > 0) ? e->readCount : 0;
    }
  }
  fit.conflictsWeighted = conflWeighted;

  int fitness = (fit.extra_regs[0] + fit.extra_regs[1]);
  fit.balance = balance;
  fitness = fitness * 1000 + balance;
  int conflicts = (fit.read_conflicts[0] + fit.read_conflicts[1] +
                   fit.write_conflicts[0] + fit.write_conflicts[1]);
  fitness = 10000 * fitness + 100 * conflicts + conflWeighted;
  return fitness;
}

/** \brief Get a free register from a specific register file, where the already
 * blocked registers are given. */
static int getSingleFreeRegister(ass_reg_t blocked);

static void allocateRegisters(MI *ins, const RDG &rdg,
                              ass_reg_t *blockedRegisters, int *blockedCount,
                              Chromosome *chrm, const int *allocations) {
  LOG_OUTPUT(LOG_M_RA_DEBUG, "\n[PortOptReg] [alloc] Allocating virtual "
                             "registers for instruction:\n\t");
  if (isLog(LOG_M_RA_DEBUG)) {
    ins->writeOutReadable(cout);
    VirtualRegisterMap *mapping = fromVirtualAlloc(chrm->regMapping());
    cout << "\t";
    ins->writeOutReadable(cout, mapping);
    releaseVirtualMap(&mapping);
    LOG_OUTPUT(LOG_M_RA_DEBUG, "[PortOptReg] [alloc] Blocked registers:\n");
    if (isLog(LOG_M_RA_DEBUG))
      printBlockedRegisters(blockedRegisters);
    LOG_OUTPUT(LOG_M_RA_DEBUG, "\n");
  }
  unsigned int issueSlot = 0;
  MO **ops = ins->getOperations();
  while (issueSlot < MI::getNumberIssueSlots()) {
    MO *op = ops[issueSlot];
    if (!op) { // MO is not set
      ++issueSlot;
      continue;
    }

    // go through the operands
    OPtype *types = op->getTypes();
    char32_t *args = op->getArguments();
    char *dirs = op->getDirections();
    for (int argIdx = 0; argIdx < op->getArgNumber(); ++argIdx) {
      char32_t argument = args[argIdx];
      if (types[argIdx] == REG &&
          (dirs[argIdx] == WRITE ||
           dirs[argIdx] ==
               READWRITEPSEUDOREAD)) { // only target registers are interesting
        if (!registers::isVirtualReg(argument))
          continue;

        char32_t real;
        if (chrm->regMapping()->find(argument) != chrm->regMapping()->end()) {
          real = chrm->regMapping()->at(argument);
          if (real != RDG::NO_REG) {
            if (isLog(LOG_M_RA_DEBUG))
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[PortOptReg] [alloc] Register '%s' is already "
                         "allocated: '%s'\n",
                         registers::getName(argument).c_str(),
                         registers::getName(real).c_str());
            continue;
          }
        } else {
          chrm->regMapping()->insert(std::make_pair(argument, RDG::NO_REG));
          real = RDG::NO_REG;
        }

        const RDG::Entry *e = rdg.getEntry(argument);
        //                if (!e) {
        //                    LOG_OUTPUT(LOG_M_ALWAYS, "[PortOptReg] [alloc]
        //                    Could not find RDG entry for argument %u (%s)\n",
        //                    argument, registers::getName(argument).c_str());
        //                    exit(1);
        //                }
        //                bool swap_rf = false;
        //                int regFile = -1;
        //                if (e->isRoot())
        //                    regFile = e->isVirtual() ? chrm->gene(e->index) :
        //                    registers::getRegFile(e->reg);
        //                else {
        //                    swap_rf = !e->same_rf;
        //                    const RDG::Entry *f = e->parent;
        //                    regFile = f->isVirtual() ? chrm->gene(f->index) :
        //                    registers::getRegFile(f->reg);
        //                }
        //                if (swap_rf)
        //                    regFile = !regFile;
        int regFile = allocations[argument];

        if (regFile == -1) {
          LOG_OUTPUT(LOG_M_ALWAYS,
                     "[PortOptReg] [alloc] Chromosome did not contain entry "
                     "for argument %u (%s):\n",
                     argument, registers::getName(argument).c_str());
          while (e) {
            LOG_OUTPUT(
                LOG_M_ALWAYS,
                "]]] e %p { reg = %s, index = %d, same_rf = %d, parent %p }\n",
                e, registers::getName(e->reg).c_str(), e->index, e->same_rf,
                e->parent);
            e = e->parent;
          }
        }

        if (!e->couple) { // single register
          if (isLog(LOG_M_RA_DEBUG))
            LOG_OUTPUT(LOG_M_ALWAYS,
                       "[PortOptReg] [alloc] Trying to allocate '%s' in "
                       "register file %d\n",
                       registers::getName(argument).c_str(), regFile);
          int regNo = getSingleFreeRegister(blockedRegisters[regFile]);
          if (regNo == -1) {
            if (isLog(LOG_M_RA_DEBUG))
              LOG_OUTPUT(
                  LOG_M_ALWAYS,
                  "[PortOptReg] [alloc] No free register in register file %d\n",
                  regFile);
            chrm->fitCounters().extra_regs[regFile]++;
          } else {
            char32_t reg = registers::createRegister(regFile, regNo);
            if (isLog(LOG_M_RA_DEBUG))
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[PortOptReg] [alloc] Allocating %s to %s\n",
                         registers::getName(argument).c_str(),
                         registers::getName(reg).c_str());
            chrm->regMapping()->at(argument) = reg;
            blockedRegisters[regFile] |= ((ass_reg_t)1) << regNo;
            blockedCount[regFile]++;
            if (isLog(LOG_M_RA_DEBUG)) {
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[PortOptReg] [alloc] Blocked registers:\n");
              printBlockedRegisters(blockedRegisters);
            }
          }
        } else { // coupled register
          const RegisterCouple *couple = e->couple;
          if (isLog(LOG_M_RA_DEBUG)) {
            LOG_OUTPUT(LOG_M_ALWAYS,
                       "[PortOptReg] [alloc] Register %s is coupled: %s+%s\n",
                       registers::getName(argument).c_str(),
                       registers::getName(couple->first).c_str(),
                       registers::getName(couple->second).c_str());
            printf("[PortOptReg] [alloc] Register %s is coupled: %s+%s\n",
                   registers::getName(argument).c_str(),
                   registers::getName(couple->first).c_str(),
                   registers::getName(couple->second).c_str());
            int n = 3;
          }
          if (chrm->regMapping()->find(couple->first) ==
              chrm->regMapping()->end()) {
            chrm->regMapping()->insert(
                std::make_pair(couple->first, RDG::NO_REG));
          }
          if (chrm->regMapping()->find(couple->second) ==
              chrm->regMapping()->end()) {
            chrm->regMapping()->insert(
                std::make_pair(couple->second, RDG::NO_REG));
          }
          if (isLog(LOG_M_RA_DEBUG)) {
            LOG_OUTPUT(LOG_M_ALWAYS,
                       "[PortOptReg] [alloc] Trying to allocate %s+%s in "
                       "register file %d\n",
                       registers::getName(couple->first).c_str(),
                       registers::getName(couple->second).c_str(), regFile);
          }
          int regNo1 = getDoubleFreeRegisters(blockedRegisters, regFile,
                                              couple->concurrent);
          if (regNo1 == -1) {
            LOG_OUTPUT(LOG_M_RA_DEBUG,
                       "[PortOptReg] [alloc] No free register pair\n");
            chrm->fitCounters()
                .extra_regs[regFile]++; // increment by 1 (not by 2), because
            // the second register of the pair will
            // also be checked
          } else {
            char32_t reg1 = registers::createRegister(regFile, regNo1);
            chrm->regMapping()->at(couple->first) = reg1;
            blockedRegisters[regFile] |= ((ass_reg_t)1) << regNo1;
            blockedCount[regFile]++;
            int regNo2 = couple->concurrent ? regNo1 + 1 : regNo1;
            int regFile2 = couple->concurrent ? regFile : regFile + 1;
            char32_t reg2 = registers::createRegister(regFile2, regNo2);
            chrm->regMapping()->at(couple->second) = reg2;
            blockedRegisters[regFile2] |= ((ass_reg_t)1) << (regNo2);
            blockedCount[regFile2]++;

            if (isLog(LOG_M_RA_DEBUG)) {
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[PortOptReg] [alloc] Allocating %s+%s to %s+%s\n",
                         registers::getName(couple->first).c_str(),
                         registers::getName(couple->second).c_str(),
                         registers::getName(reg1).c_str(),
                         registers::getName(reg2).c_str());
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[PortOptReg] [alloc] Blocked registers:\n");
              printBlockedRegisters(blockedRegisters);
            }
          }
        }
      }
    }
    issueSlot += op->opLengthMultiplier();
  }
}

static void printMI(MI *ins, Chromosome *chrm) {
  LOG_OUTPUT(LOG_M_ALWAYS, "\t");
  ins->writeOutReadable(cout);
  LOG_OUTPUT(LOG_M_ALWAYS, "\t");
  VirtualRegisterMap *mapping = fromVirtualAlloc(chrm->regMapping());
  ins->writeOutReadable(cout, mapping);
  releaseVirtualMap(&mapping);
}

static void checkPortConflicts(MI *ins, Chromosome *chrm,
                               const int *blockedCount, const RDG &rdg) {
  LOG_OUTPUT(
      LOG_M_RA_CONFLICT_DETAIL,
      "[PortOptReg] [check] Starting register port check for instruction:\n");
  if (isLog(LOG_M_RA_CONFLICT_DETAIL))
    printMI(ins, chrm);

  int *readPorts = new int[registers::getNumRegisterFiles()];
  int *writePorts = new int[registers::getNumRegisterFiles()];
  for (uint i = 0; i < registers::getNumRegisterFiles(); ++i) {
    writePorts[i] = registers::getWritePorts(i);
    readPorts[i] = registers::getReadPorts(i);
  }

  unsigned int issueSlot = 0;
  MO **ops = ins->getOperations();
  while (issueSlot < MI::getNumberIssueSlots()) {
    if (isLog(LOG_M_RA_CONFLICT_DETAIL))
      LOG_OUTPUT(LOG_M_ALWAYS, "[PortOptReg] [check] Checking MO %d\n",
                 issueSlot);
    MO *op = ops[issueSlot];
    if (op == NULL) {
      if (isLog(LOG_M_RA_CONFLICT_DETAIL))
        LOG_OUTPUT(LOG_M_ALWAYS, "[PortOptReg] [check] Slot %d is empty\n",
                   issueSlot);
      ++issueSlot;
      continue;
    }

    OPtype *types = op->getTypes();
    char32_t *arguments = op->getArguments();
    char *dir = op->getDirections();
    int maxArg = op->getArgNumber();

    // suffix X2 implies two writes
    if (op->isX2Operation()) {
      OPtype *types = op->getTypes();
      char *dir = op->getDirections();
      if (types[4] == REG)
        dir[4] = WRITE;
    }

    for (int argIdx = 0; argIdx < maxArg; argIdx++) {
      char32_t argument = arguments[argIdx];
      char32_t virtual_reg = rdg::RDG::NO_REG;
      if (types[argIdx] == REG && !registers::isIndFirReg(argument)) {
        int regfile = registers::getRegFile(argument);
        if (regfile < 0) { // if it is a virtual register, consult the mapping
          // for the corresponding real
          if (registers::getVirtualRegisterNumber(argument) >= 0) {
            char32_t real = static_cast<char32_t>(-1);
            if (chrm->regMapping()->find(argument) != chrm->regMapping()->end())
              real = chrm->regMapping()->at(argument);
            if (real == (char32_t)-1 || real == (char32_t)-2) {
              if (isLog(LOG_M_RA_CONFLICT_DETAIL))
                LOG_OUTPUT(LOG_M_ALWAYS,
                           "[PortOptReg] [check] Skipping non allocated "
                           "virtual register %s\n",
                           registers::getName(argument).c_str());
              continue;
            }
            regfile = registers::getRegFile(real);
            if (isLog(LOG_M_RA_CONFLICT_DETAIL))
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[PortOptReg] [check] Checking port conflicts for "
                         "argument %s = %s\n",
                         registers::getName(argument).c_str(),
                         registers::getName(real).c_str());
            virtual_reg = argument;
            argument = real;
          } else {
            if (isLog(LOG_M_RA_CONFLICT_DETAIL))
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[PortOptReg] [check] Skipping argument '%s'\n",
                         registers::getName(argument).c_str());
            continue;
          }
        }
        if ((dir[argIdx] & WRITE) && !(dir[argIdx] & PSEUDOWRITE) &&
            !registers::isDummyRegister(argument)) {
          if (isLog(LOG_M_RA_CONFLICT_DETAIL))
            LOG_OUTPUT(LOG_M_ALWAYS,
                       "[PortOptReg] [check] %s consumes a write port in "
                       "regfile %d (%d remaining)\n",
                       registers::getName(argument).c_str(), regfile,
                       writePorts[regfile] - 1);
          if (writePorts[regfile]-- <= 0) {
            chrm->fitCounters().write_conflicts[regfile]++;
            if (isLog(LOG_M_RA_CONFLICT))
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[PortOptReg] [check] This is a write port conflict "
                         "in regfile %d\n",
                         regfile);

            //                        LOG_OUTPUT(LOG_M_ALWAYS,
            //                        "[WritePortConflict] Instruction:\n");
            //                        printMI(ins, chrm);
          }
        }
        if ((dir[argIdx] & READ) && !(dir[argIdx] & PSEUDOREAD) &&
            !registers::isDummyRegister(argument)) {
          if (isLog(LOG_M_RA_CONFLICT_DETAIL))
            LOG_OUTPUT(LOG_M_ALWAYS,
                       "[PortOptReg] [check] %s consumes a read port in "
                       "regfile %d (%d remaining)\n",
                       registers::getName(argument).c_str(), regfile,
                       readPorts[regfile] - 1);
          if (readPorts[regfile]-- <= 0) {
            chrm->fitCounters().read_conflicts[regfile]++;
            const RDG::Entry *e = rdg.getRootEntry(virtual_reg);
            if (e && e->index > -1)
              chrm->conflictCount(e->index)++;
            if (isLog(LOG_M_RA_CONFLICT))
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[PortOptReg] [check] This is a read port conflict in "
                         "regfile %d\n",
                         regfile);

            //                        LOG_OUTPUT(LOG_M_ALWAYS,
            //                        "[ReadPortConflict] Instruction:\n");
            //                        printMI(ins, chrm);
          }
        }
        if ((argIdx == 0 || argIdx == 4) &&
            op->getOperation()->getName().compare(0, 2, "MV") == 0 &&
            !strncmp(op->getOperation()->getCond(), "CR", 2)) {
          if (isLog(LOG_M_RA_CONFLICT_DETAIL))
            LOG_OUTPUT(LOG_M_ALWAYS,
                       "[PortOptReg] [check] %s consumes a read port in "
                       "regfile %d (%d remaining)\n",
                       registers::getName(argument).c_str(), regfile,
                       readPorts[regfile] - 1);
          if (readPorts[regfile]-- <= 0) {
            chrm->fitCounters().read_conflicts[regfile]++;
            const RDG::Entry *e = rdg.getRootEntry(virtual_reg);
            if (e && e->index > -1)
              chrm->conflictCount(e->index)++;
            if (isLog(LOG_M_RA_CONFLICT))
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[PortOptReg] [check] This is a read port conflict in "
                         "regfile %d\n",
                         regfile);

            //                        LOG_OUTPUT(LOG_M_ALWAYS,
            //                        "[ReadPortConflict] Instruction:\n");
            //                        printMI(ins, chrm);
          }
        }
      }
    }

    issueSlot += op->opLengthMultiplier();
  }

  //    if (readPorts[0] < 0) {
  //        int free = readPorts[1] + readPorts[0];
  //        if (free < 0)
  //            chrm->fitCounters().usedRegs[0] += -readPorts[0];
  //    }
  //    if (readPorts[1] < 0) {
  //        int free = readPorts[0] + readPorts[1];
  //        if (free < 0)
  //            chrm->fitCounters().usedRegs[1] += -readPorts[1];
  //    }

  delete[] readPorts;
  delete[] writePorts;
}

static void freeRegisters(ass_reg_t *blocked, int *blockedCount,
                          Instructions it, VirtualRegisterMap *map,
                          VirtualAllocation *regMapping) {
  MI *mi = *it;
  MO **ops = mi->getOperations();
  unsigned int issueSlot = 0;
  while (issueSlot < MI::getNumberIssueSlots()) {
    MO *op = ops[issueSlot];
    if (op == NULL) {
      ++issueSlot;
      continue;
    }

    char32_t *arguments = op->getArguments();
    OPtype *types = op->getTypes();
    int maxArg = op->getArgNumber();
    for (int argIdx = 0; argIdx < maxArg; ++argIdx) {
      char32_t arg = arguments[argIdx];
      if (types[argIdx] == REG) {
        int vreg = registers::getVirtualRegisterNumber(arg);
        if (vreg < 0)
          continue;
        VirtualRegisterMapping *mapping = getMapping(map, arg);
        if (!mapping) {
          LOG_OUTPUT(LOG_M_ALWAYS, "No mapping for virtual register '%s'\n",
                     registers::getName(arg).c_str());
          EXIT_ERROR
        }

        if (mapping->getLastOccurrence() == it) {
          char32_t real = getReal(map, regMapping, arg);
          if (real != (char32_t)-1) {
            int regfile = registers::getRegFile(real);
            int regno = registers::getRegNumber(real);
            if (isLog(LOG_M_RA_DEBUG))
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[PortOptReg] [free ] Freeing register %s, as it's no "
                         "longer used\n",
                         registers::getName(real).c_str());
            blocked[regfile] &= ~(1 << regno);
            blockedCount[regfile]--;
          }
        }
      }
    }
    issueSlot += op->opLengthMultiplier();
  }
}

static int getSingleFreeRegister(ass_reg_t blocked) {
  ass_reg_t temp = 1;
  // first try to allocate a register, which is free, but could not be used in a
  // register pair
  for (unsigned int i = 0; i < registers::getNumRegister1RF() - 1; i += 2) {
    if ((blocked & (temp << (i + 1))) && !(blocked & (temp << i)))
      return i;
    if (!(blocked & (temp << (i + 1))) && (blocked & (temp << i)))
      return i + 1;
  }
  // if this fails, then any register is fine
  for (unsigned int i = 0; i < registers::getNumRegister1RF(); i++)
    if (!(blocked & (temp << i)))
      return i;
  return -1; // no free register found
}

// static std::pair<int, int> getConcurrentDoubleFreeRegisters(ass_reg_t
// blocked) {
//  ass_reg_t temp = 1;
//  for (unsigned int i = 0; i < registers::getNumRegister1RF() - 1; i += 2)
//    if (!(blocked & (temp << i)) && !(blocked & (temp << (i + 1))))
//      return  {i, i+1};
//  return {-1, -1};
//}
//
// static std::pair<int, int> getParallelDoubleFreeRegisters(ass_reg_t *blocked)
// {
//  ass_reg_t temp = 1;
//  for (unsigned int i = 0; i < registers::getNumRegister1RF(); i++)
//    if (!(blocked[0] & (temp << i)) && !(blocked[1] & (temp << i)))
//      return {i, i+ numRegisters};
//  return {-1, -1};
//}
//
// static int getDoubleFreeRegisters(ass_reg_t *blocked, int regFile,
//                                  bool concurrent) {
//  return getDoubleFreeRegistersPair(blocked, regFile, concurrent).first;
//
//}
//
// static std::pair<int, int> getDoubleFreeRegistersPair(ass_reg_t *blocked, int
// regFile,
//                                                      bool concurrent){
//  if (concurrent)
//    return getConcurrentDoubleFreeRegisters(blocked[regFile]);
//  else
//    return getParallelDoubleFreeRegisters(blocked);
//}

/* **************************************************************** */

/**
 * \brief The genetic algorithm.
 *
 * This is the implementation of the genetic algorithm. Initialization and
 * evolution of the population is done here.
 *
 * \note This function expects the virtual registers to be consecutively
 * numbered, starting with 1 (as it should be after register renaming)!
 */
int allocateRegisters(int slm_id, const RDG &rdg, ass_reg_t *blocked,
                      Processor *pro, Program *ins, VirtualRegisterMap *map,
                      const RegisterCoupling &couplings, const Context &ctx,
                      VirtualRegisterMap *heuristicMap) {
  LOG_OUTPUT(LOG_M_RA,
             "%s Starting register allocation with GA for SLM %d (MIs: %d)\n",
             ctx.asString().c_str(), slm_id, ins->size());

#if CHECK_RA_TIMING
  ra_timings.gen_CompleteTimer.start();
#endif

  size_t registerCount = rdg.rootCount(); // only the independent registers
  size_t popSize = getPopulationSize(registerCount);
  int tournamentSize = getTournamentSize();
  float mutationProb = getMutationProbability();

  std::vector<int> improvementSteps;

  size_t COPY = 1;
  size_t RANDOM = max((size_t)2, (popSize - COPY) / 10);
  size_t COMBINE = popSize - COPY - RANDOM;

  //    LOG_OUTPUT(LOG_M_ALWAYS, "RA for SLM %d: register count: %d  popsize:
  //    %d\n", slm_id, registerCount, popSize);
  LOG_OUTPUT(
      LOG_M_RA_DETAIL,
      "%s Population size: %lu, COPY = %lu, COMBINE = %lu, RANDOM = %lu\n",
      ctx.asString().c_str(), popSize, COPY, COMBINE, RANDOM);
  //    if (ctx.schedIndiv() == 0)
  //        LOG_OUTPUT(LOG_M_ALWAYS, "%s Population size: %lu, COPY = %lu,
  //        COMBINE = %lu, RANDOM = %lu\n", ctx.asString().c_str(), popSize,
  //        COPY, COMBINE, RANDOM);

#if REG_GA_STATS
  regGAStats[CURRENT_THREAD_NUM].reset();
#endif

  //  uint seed = 500;
  uint seed = (unsigned int)time(0);
  ga_stats::ChromosomesSet processed_chromosomes;

  Population *pop = Population::random(popSize, registerCount, &seed);
  pop->ownsIndividuals() = false;
  Chromosome *best = pop->evaluate(fitness, ins, map, rdg, blocked, pro);
#if REG_GA_STATS
  for (size_t i = 0; i < popSize; ++i) {
    regGAStats[CURRENT_THREAD_NUM].add(pop->individual(i));
  }
#endif
  int bestFitness = best->fitness();
  if (bestFitness == 0) {
    VirtualAllocation *regMapping = best->regMapping();
    if (regMapping) {
      for (VirtualRegisterMap::iterator it = map->begin(); it != map->end();
           ++it) {
        char32_t v = it->second->getVirtual();
        int virtRegNo = registers::getVirtualRegisterNumber(v);
        if (virtRegNo < 0)
          continue;
        if (regMapping->find(v) != regMapping->end())
          it->second->setReal(regMapping->at(v));
        else {
          LOG_OUTPUT(LOG_M_ALWAYS,
                     "Found no register mapping for virtual register '%s' "
                     "during port optimal register allocation\n",
                     registers::getName(v).c_str());
          EXIT_ERROR
        }
      }
    }
  }
  for (size_t i = 0; i < popSize; ++i) {
    Chromosome *c = pop->individual(i);

#if PORT_OPT_RA_DUP_CHRM_CHECK
    ga_stats::add_processed(ga_stats::NumberSequence(c->genes(), c->length()),
                            c->fitness(), processed_chromosomes);
#endif

    delete c->regMapping();
    c->regMapping() = 0;
  }
  LOG_OUTPUT(LOG_M_RA_DETAIL, "%s Fitness of initial population: %d\n",
             ctx.asString().c_str(), bestFitness);

  if (params.printRegPopulation) {
    LOG_OUTPUT(LOG_M_ALWAYS, "Generation 0\n");
    pop->sort();
    pop->print();
  }

  int generation = 0;
  int lastBestGeneration = 0;
  int round = 0;
  while (bestFitness > 0 && round < abs(params.portRegRounds)) {
    bool newBest = false;
    ++generation;
    ++round;

    Population *parents = pop;
    pop = new Population(popSize, false);

    parents->sort();
    best = parents->individual(0);
    for (size_t i = 0; i < COPY; ++i) {
      pop->setIndividual(parents->individual(i), i);
      pop->individual(i)->origin() = CHROMOSOME::Origin::COPY;
    }

#pragma omp parallel
    {
#pragma omp for nowait
      for (size_t i = 0; i < COMBINE; ++i) {
        Chromosome *parent1 = parents->tournamentSelect(tournamentSize, &seed);
        Chromosome *parent2 = parents->tournamentSelect(tournamentSize, &seed);
        int searchCount = 0;
        while (searchCount < 30 && parent1 == parent2) {
          parent2 = parents->tournamentSelect(tournamentSize, &seed);
          searchCount++;
        }
        LOG_OUTPUT(searchCount == 10,
                   "%s Did not find different parents in port optimal register "
                   "allocation\n",
                   ctx.asString().c_str());
        Chromosome *child =
            Chromosome::combine(parent1, parent2, &seed, mutationProb);
        pop->setIndividual(child, COPY + i);
      }

#pragma omp for
      for (size_t i = 0; i < RANDOM; ++i) {
        Chromosome *c = Chromosome::random(registerCount, &seed);
        pop->setIndividual(c, COPY + COMBINE + i);
      }

#if PORT_OPT_RA_DUP_CHRM_CHECK
      for (size_t i = 0; i < COMBINE + RANDOM; ++i) {
        Chromosome *c = pop->individual(COPY + i);
        std::pair<bool, ga_stats::ChromosomeFitness> r =
            ga_stats::already_processed(
                ga_stats::NumberSequence(c->genes(), c->length()),
                processed_chromosomes);
        if (r.first) {
          c->setDuplicate();
          c->fitness() = r.second.getFitness();
        }
      }
#endif
    } // pragma omp parallel

#pragma omp parallel for
    for (size_t i = 0; i < COMBINE + RANDOM; ++i) {
      Chromosome *c = pop->individual(COPY + i);
      if (!c->isDuplicate())
        c->fitness() = c->evaluate(fitness, ins, map, rdg, blocked, pro);
    }

#if REG_GA_STATS
    for (size_t i = 0; i < COPY + COMBINE + RANDOM; ++i) {
      regGAStats[CURRENT_THREAD_NUM].add(pop->individual(COPY + i));
    }
#endif

    for (size_t i = 0; i < COPY + COMBINE + RANDOM; ++i) {
      Chromosome *c = pop->individual(i);
      int fit = c->fitness();
      if (fit < best->fitness()) {
        best = c;
        if (params.portRegRounds > 0)
          round = 0;
        //                if (params.printRegPopulation)
        //                    LOG_OUTPUT(LOG_M_ALWAYS, "%s Genetic RA improved
        //                    fitness to %d in generation %d in SLM %d\n",
        //                    ctx.asString().c_str(), bestFitness, generation,
        //                    slm_id);
        if (bestFitness > 0 && fit == 0) {
          VirtualAllocation *regMapping = c->regMapping();
          if (regMapping) {
            for (VirtualRegisterMap::iterator it = map->begin();
                 it != map->end(); ++it) {
              char32_t v = it->second->getVirtual();
              if (regMapping->find(v) != regMapping->end())
                it->second->setReal(regMapping->at(v));
              else {
                LOG_OUTPUT(LOG_M_ALWAYS,
                           "Found no register mapping for virtual register "
                           "'%s' during port optimal register allocation\n",
                           registers::getName(v).c_str());
                EXIT_ERROR
              }
            }
          }
        }
        bestFitness = fit;
        newBest = true;
      }
      delete c->regMapping();
      c->regMapping() = 0;
    }

    for (size_t i = COPY; i < popSize; ++i)
      delete parents->individual(i);
    delete parents;

    if (params.printRegPopulation) {
      LOG_OUTPUT(LOG_M_ALWAYS, "Generation %d\n", generation);
      pop->sort();
      pop->print();
    }

    if (isLog(LOG_M_RA_SIZE_HIST) && newBest) {
      // LOG_OUTPUT(LOG_M_RA_SIZE_HIST, "Genetic RA improved best fitness to
      // %d after %d generations for SLM %d\n", bestFitness, generation -
      // lastBestGeneration, slm_id);
      improvementSteps.push_back(generation - lastBestGeneration);
      lastBestGeneration = generation;
    }
  }

#if REG_GA_STATS
  cout << "Register allocation GA statistics for SLM " << slm_id << ": ";
  regGAStats[CURRENT_THREAD_NUM].printSummary(cout, "register allocations");
  cout << endl;
//    regGAStats[CURRENT_THREAD_NUM].printStats(cout);
#endif

  int fitness = bestFitness;
  LOG_OUTPUT(LOG_M_RA_SIZE_HIST, "Genetic RA ended with fitness %d\n", fitness);
  if (isLog(LOG_M_RA_SIZE_HIST)) {
    if (fitness == 0 && improvementSteps.empty())
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Genetic RA fitness improved after 0 generations\n");
    for (auto &i : improvementSteps) {
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Genetic RA fitness improved after %d generations\n", i);
    }
  }
  if (isLog(LOG_M_RA_ROUNDS_HIST))
    finishedRA(generation, fitness);
  flushLog();
  pop->releaseIndividuals();
  delete pop;

#if CHECK_RA_TIMING
  ra_timings.gen_CompleteTimer.stop();
  ra_timings.gen_CompleteCount += popSize * generation;
#endif
  return fitness;
}

/**
 * \brief The genetic algorithm.
 *
 * This is the implementation of the genetic algorithm. Initialization and
 * evolution of the population is done here.
 *
 * \note This function expects the virtual registers to be consecutively
 * numbered, starting with 1 (as it should be after register renaming)!
 */
ga_stats::ChromosomeFitness
allocateRegistersPower(int slm_id, const RDG &rdg, ass_reg_t *blocked,
                       Processor *pro, Program *ins, VirtualRegisterMap *map,
                       const RegisterCoupling &couplings, const Context &ctx,
                       VirtualRegisterMap *heuristicMap) {
  LOG_OUTPUT(LOG_M_RA,
             "%s Starting register allocation with GA for SLM %d (MIs: %d)\n",
             ctx.asString().c_str(), slm_id, ins->size());

#if CHECK_RA_TIMING
  ra_timings.gen_CompleteTimer.start();
#endif

  int VIRTUAL_REGISTER_IN_SLM = 0;

  size_t registerCount = rdg.rootCount(); // only the independent registers
  size_t popSize = getPopulationSize(registerCount);

  if (params.raRegPopSize > 0) {
    for (auto ma : *map) {
      if (registers::isVirtualReg(ma.first)) {
        VIRTUAL_REGISTER_IN_SLM++;
      }
    }
    popSize = getPopulationSizeEnergyAllocation();
    //    cout << "Population Size:" << params.raRegPopSize << endl;
    //    cout << "Number Virt Register:" << VIRTUAL_REGISTER_IN_SLM << "in SLM
    //    ID "
    //         << slm_id << endl;
    //        auto original = params.raRegPopSize;
    //        for(int i =1; i < 13; i++){
    //          params.raRegPopSize = i;
    //          cout << params.raRegPopSize << " = " <<
    //          getPopulationSizeEnergyAllocation() << endl;
    //        }
    //        params.raRegPopSize = original;
  }
  int tournamentSize = getTournamentSize();
  float mutationProb = getMutationProbability();

  std::vector<int> improvementSteps;

  size_t COPY = min(5.0f, max(1.0f, popSize * params.raRegCopy));
  size_t RANDOM = max(1.0f, popSize * params.raRegRandom);
  size_t COMBINE = max(1.0f, popSize * params.raRegCombine);
  size_t MUTATE = max(1.0f, popSize * params.raRegMutate);
  size_t COMBINEMUTATE = COMBINE * params.raRegCombineMutate;
  size_t popSize_t = COPY + RANDOM + COMBINE + MUTATE - popSize;
  RANDOM = RANDOM - popSize_t;
  LOG_OUTPUT(
      LOG_M_ALWAYS,
      "%s Population size: %lu, COPY = %lu, COMBINE = %lu, RANDOM = %lu\n",
      ctx.asString().c_str(), popSize, COPY, COMBINE, RANDOM);
  if (RANDOM < 0) {
    throw runtime_error(
        "RANDOM is negative, please update population probabilites.");
  }
  if (popSize != COPY + RANDOM + COMBINE + MUTATE) {
    throw runtime_error("Population size does not match.");
  }

  //    LOG_OUTPUT(LOG_M_ALWAYS, "RA for SLM %d: register count: %d  popsize:
  //    %d\n", slm_id, registerCount, popSize);
  LOG_OUTPUT(
      LOG_M_RA_DETAIL,
      "%s Population size: %lu, COPY = %lu, COMBINE = %lu, RANDOM = %lu\n",
      ctx.asString().c_str(), popSize, COPY, COMBINE, RANDOM);
  //    if (ctx.schedIndiv() == 0)
  //        LOG_OUTPUT(LOG_M_ALWAYS, "%s Population size: %lu, COPY = %lu,
  //        COMBINE = %lu, RANDOM = %lu\n", ctx.asString().c_str(), popSize,
  //        COPY, COMBINE, RANDOM);

#if REG_GA_STATS
  regGAStats[CURRENT_THREAD_NUM].reset();
#endif

  //  uint seed = 500;
  uint seed = (unsigned int)time(0);

  //  ins->printInstructions(std::cout);
  // debugging seed
  if (VIRTUAL_REGISTER_IN_SLM != 0) {
    LOG_OUTPUT(LOG_M_RA_DETAIL, "Entering SLM %d\n", slm_id);
    //    seed = 500;
    // adapt tournamentSize for small populations
    int originalTournamentSize = params.raRegTournamentSize;

    auto fitness = allocateEnergyRegister(
        popSize, ins, map, rdg, blocked, pro, couplings, COPY, RANDOM, COMBINE,
        COMBINEMUTATE, MUTATE, &seed, ctx, heuristicMap);

    params.raRegTournamentSize = originalTournamentSize;
    return fitness;

  } else {
    return 0;
  }
}

/* ********** Chromosome **************************************** */

Chromosome::Chromosome(size_t length)
    : _length(length), _fitness(0), _origin(CHROMOSOME::NONE), _regMapping(0),
      _duplicate(false), _parent1_fit(0), _parent2_fit(0) {
  _genes = new int[_length];
  memset(_genes, 0, _length * sizeof(int));
  _conflictCount = new int[_length];
  memset(_conflictCount, 0, _length * sizeof(int));
}

Chromosome::Chromosome(const Chromosome &chrm)
    : _length(chrm._length), _fitness(chrm.fitness()), _origin(chrm.origin()),
      _regMapping(0), _duplicate(chrm._duplicate),
      _parent1_fit(chrm._parent1_fit), _parent2_fit(chrm._parent2_fit) {
  _genes = new int[_length];
  memcpy(_genes, chrm._genes, _length * sizeof(int));
  _conflictCount = new int[_length];
  memset(_conflictCount, 0, _length * sizeof(int));
}

Chromosome::~Chromosome() {
  delete[] _genes;
  delete[] _conflictCount;
}

Chromosome *Chromosome::random(size_t length, uint *seed) {
  Chromosome *c = new Chromosome(length);
  for (size_t i = 0; i < length; ++i)
    c->setGene(i, rand_r(seed) % 2);
  c->origin() = CHROMOSOME::RANDOM;
  return c;
}

Chromosome *Chromosome::half(size_t length) {
  Chromosome *c = new Chromosome(length);
  for (size_t i = length / 2; i < length; ++i)
    c->setGene(i, 1);
  c->origin() = CHROMOSOME::HALF;
  return c;
}

FitnessCounters &Chromosome::fitCounters() { return _fitCounters; }

int Chromosome::evaluate(FitnessFunction f, Program *ins,
                         VirtualRegisterMap *map, const RDG &rdg,
                         ass_reg_t *blockedRegisters, Processor *pro) {
  _fitness = f(this, ins, map, rdg, blockedRegisters, pro);
  return _fitness;
}

Chromosome *Chromosome::combine(Chromosome *parent1, Chromosome *parent2,
                                uint *seed, float mutationProb) {
  size_t length = parent1->length();
  Chromosome *child = new Chromosome(*parent1);
  child->parent1Fit() = parent1->fitness();
  child->parent2Fit() = parent2->fitness();
  child->origin() = CHROMOSOME::COMBINE;
  for (size_t i = 0; i < length; ++i) {
    if (util::uniRandom(seed) < .5)
      child->setGene(i, parent2->gene(i));

    if (util::uniRandom(seed) < mutationProb) {
      child->flipGene(i);
      child->origin() = CHROMOSOME::MUTATE;
    }
  }
  return child;
}

/*string Chromosome::toStr(Chromosome::Origin &o) {
  switch (o) {
  case RANDOM:
    return "random";
  case HALF:
    return "half";
  case COPY:
    return "copy";
  case COMBINE:
    return "combine";
  case MUTATE:
    return "mutate";
  case NONE:
    return "<>";
  default:
    return "X";
  }
}*/

void Chromosome::printGenes() const {
  for (size_t i = 0; i < _length; ++i)
    LOG_OUTPUT(LOG_M_RA, "%d", _genes[i]);
}

void Chromosome::printConflicts() {
  LOG_OUTPUT(LOG_M_ALWAYS,
             "R: (%3d, %3d) W: (%3d, %3d) | Extra: (%3d, %3d) | Confl.Weight: "
             "%3d | Bal: %3d",
             _fitCounters.read_conflicts[0], _fitCounters.read_conflicts[1],
             _fitCounters.write_conflicts[0], _fitCounters.write_conflicts[1],
             _fitCounters.extra_regs[0], _fitCounters.extra_regs[1],
             _fitCounters.conflictsWeighted, _fitCounters.balance);
}

/* ********** Population ****************************************** */

Population::Population(size_t size, bool ownsIndividuals)
    : _size(size), _individuals(_size, 0), _ownsIndividuals(ownsIndividuals) {}

Population::Population(const Population &pop)
    : _size(pop._size), _individuals(pop._size),
      _ownsIndividuals(pop.ownsIndividuals()) {
  for (size_t i = 0; i < _size; ++i)
    _individuals[i] = new Chromosome(*(pop.individual(i)));
}

Population *Population::random(size_t size, size_t chromosomeLength,
                               uint *seed) {
  Population *pop = new Population(size);
  for (size_t i = 0; i < size; ++i)
    pop->setIndividual(Chromosome::random(chromosomeLength, seed), i);
  return pop;
}

Population::~Population() {
  if (_ownsIndividuals)
    releaseIndividuals();
}

void Population::releaseIndividuals() {
  for (size_t i = 0; i < _size; ++i)
    delete _individuals[i];
}

const Chromosome *Population::individual(size_t index) const {
  return _individuals[index];
}

Chromosome *Population::individual(size_t index) { return _individuals[index]; }

void Population::setIndividual(Chromosome *chrm, size_t index) {
  _individuals[index] = chrm;
}

Chromosome *Population::evaluate(FitnessFunction f, Program *ins,
                                 VirtualRegisterMap *map, const RDG &rdg,
                                 ass_reg_t *blockedRegisters, Processor *pro) {
  if (_size > 0) {
    Chromosome *best = _individuals[0];
    best->evaluate(f, ins, map, rdg, blockedRegisters, pro);

    for (size_t i = 1; i < _size && best->fitness() > 0; ++i) {
      int fit =
          _individuals[i]->evaluate(f, ins, map, rdg, blockedRegisters, pro);
      if (fit < best->fitness())
        best = _individuals[i];
    }
    return best;
  } else
    return 0;
}

bool &Population::ownsIndividuals() { return _ownsIndividuals; }

const bool &Population::ownsIndividuals() const { return _ownsIndividuals; }

void Population::sort() {
  std::sort(_individuals.begin(), _individuals.end(), Chromosome::compare);
}

Chromosome *Population::selectUniform(uint *seed) {
  bool onlyDuplicates = true;
  for (size_t i = 0; i < _size; ++i) {
    if (!_individuals[i]->isDuplicate()) {
      onlyDuplicates = false;
      break;
    }
  }
  while (true) {
    int index = rand_r(seed) % _size;
    if (onlyDuplicates || !_individuals[index]->isDuplicate())
      return _individuals[index];
  }
}

Chromosome *Population::tournamentSelect(int tournamentSize, uint *seed) {
  Chromosome *chrm = selectUniform(seed);
  for (int i = 1; i < tournamentSize; ++i) {
    Chromosome *c = selectUniform(seed);
    if (c->fitness() < chrm->fitness())
      chrm = c;
  }
  return chrm;
}

void Population::print() {
  for (size_t i = 0; i < _size; ++i) {
    Chromosome *c = _individuals[i];
    LOG_OUTPUT(LOG_M_ALWAYS, "[PortOptReg] Nr. %3lu fit %5d origin %7s (%s) [",
               i, c->fitness(), toStr(c->origin()).c_str(),
               c->isDuplicate() ? "D" : " ");
    c->printConflicts();
    if (c->origin() == CHROMOSOME::COMBINE || c->origin() == CHROMOSOME::MUTATE)
      LOG_OUTPUT(LOG_M_ALWAYS, " p1_fit: %5d p2_fit: %5d", c->parent1Fit(),
                 c->parent2Fit());
    LOG_OUTPUT(LOG_M_ALWAYS, "]\n");
  }
}

} // namespace portOptReg
