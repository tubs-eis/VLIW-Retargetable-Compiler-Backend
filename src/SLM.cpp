// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "SLM.h"
#include "../powerEstimation/InstructionModel.h"
#include "../powerEstimation/TransitionEnergyEstimator.h"
#include "CompilerContext.h"
#include "gen_merging.h"
#include "label.h"
#include "opts.h"
#include "processor.h"
#include "virtual_reg.h"
#include <cmath>
#include <cstring>
#include <fstream>
using namespace gen_X2;

MergeCandidates::MergeCandidates(int size) {
  _size = size;
  _mergeCandidates = new CandidateList_t[_size];
}

void MergeCandidates::copyFrom(const MergeCandidates &candidates) {
  delete[] _mergeCandidates;
  _size = candidates._size;
  _mergeCandidates = new CandidateList_t[_size];
  for (int index = 0; index < _size; ++index)
    _mergeCandidates[index] = candidates.getCandidates(index);
}

MergeCandidates::MergeCandidates(const MergeCandidates &candidates) {
  copyFrom(candidates);
}

MergeCandidates::MergeCandidates(SLM *slm, Processor *pro)
    : MergeCandidates(slm->getOperations()->size()) {
  RegisterCoupling couplings;
  std::vector<MO *> &ops = *(slm->getOperations());
#pragma omp parallel for
  for (int i = 0; i < _size; ++i) {
    MO *mo_i = ops[i];
    Operation *op_i = mo_i->getOperation();
    if (mo_i->isX2Operation() || !isMergeable(mo_i, pro))
      continue;
    for (int j = i + 1; j < _size; ++j) {
      MO *mo_j = ops[j];
      Operation *op_j = mo_j->getOperation();
      if (op_i != op_j) {
        if (util::isOpPair(op_i, op_j, "RFU7", "RFU8", false)) {
          if (gen_X2::areMergeable(mo_i, mo_j, slm->getOriginalID(), couplings,
                                   pro))
            addCandidate(i, j);
        } else if (!util::isOpPair(op_i, op_j, "ADDEXPNDH", "ADDEXPNDL") &&
                   !util::isOpPair(op_i, op_j, "ADDEXPNDHI", "ADDEXPNDLI"))
          continue;
      }
      if (mo_i->getID() == mo_j->getID() || mo_j->isX2Operation() ||
          !isMergeable(mo_j, pro))
        continue;
      if (gen_X2::areMergeable(mo_i, mo_j, slm->getOriginalID(), couplings,
                               pro)) {
        addCandidate(i, j);
      }
    }
  }
}

MergeCandidates::~MergeCandidates() { delete[] _mergeCandidates; }

void MergeCandidates::addCandidate(int firstMOIndex, int candidateIndex) {
  auto candidates = getCandidates(firstMOIndex);
  if (std::find(candidates.begin(), candidates.end(), candidateIndex) ==
      candidates.end())
    _mergeCandidates[firstMOIndex].push_back(candidateIndex);
}

const MergeCandidates::CandidateList_t &
MergeCandidates::getCandidates(int MOIndex) const {
  return _mergeCandidates[MOIndex];
}

MergeCandidates &MergeCandidates::operator=(const MergeCandidates &candidates) {
  copyFrom(candidates);
  return *this;
}

int MergeCandidates::totalCandidateCount() const {
  int total = 0;
  for (int index = 0; index < _size; ++index)
    total += getCandidates(index).size();
  return total;
}

int getMONum(const std::vector<MO *> *ops, int i) {
  if (ops)
    if (i != -1)
      return ops->at(i)->getLineNumber();
    else
      return -1;
  else
    return i;
}

uint MergeCandidates::Xpos(const CandidateList_t &candidates) {
  auto index = 0u;
  while (index < candidates.size()) {
    if (candidates[index] == -1)
      return index;
    else
      ++index;
  }
  return -1;
}

void MergeCandidates::print(LOG_MASK_T logMask,
                            const MergeCandidates &candidates,
                            const std::vector<MO *> *ops) {
  for (int index = 0; index < candidates.size(); ++index) {
    LOG_OUTPUT(logMask, "%4d -> ", getMONum(ops, index));
    printMergeCandidates(logMask, candidates.getCandidates(index), ops);
    LOG_OUTPUT(logMask, "\n");
  }
}

void gen_X2::printMergeCandidates(
    LOG_MASK_T logMask, const MergeCandidates::CandidateList_t &candidates,
    const std::vector<MO *> *ops) {
  for (auto &idx : candidates)
    LOG_OUTPUT(logMask, "%4d, ", getMONum(ops, idx));
}

int SLM::counter[MAX_THREAD_COUNT];

const int SLM::emptyMaxDistanceX2Merging = 50000;

SLM::SLM() {
  this->minDistanceX2Merging = 0;
  this->maxDistanceX2Merging = emptyMaxDistanceX2Merging;
  this->booster = 1.0;
  this->label = -2;
  this->offset = 0;
  this->ops = new vector<MO *>();
  this->id = ++counter[CURRENT_THREAD_NUM];
  this->orignalid = this->id;
  this->minimalSize = -1;
  this->criticalPath = -1;
  this->branchOperation = NULL;
  this->ins = NULL;
  this->map = NULL;
  this->alternative = NULL;
  this->optimizationLevel = -1;
  this->mergeLevel = -1;
  memset(this->ins_per_thread, 0, MAX_THREAD_COUNT * sizeof(Program *));
  memset(this->map_per_thread, 0,
         MAX_THREAD_COUNT * sizeof(VirtualRegisterMap *));
  // #if defined(_OPENMP)
  //     omp_init_lock(&addLastInsLock);
  // #endif
  _registerHeuristicFit = -1;
  _registerGeneticFit = -1;
  partProb = -1;
  aloneProb = -1;
  _haveRAPruneCount = false;
  RApruneCount = -1;
}

SLM::~SLM() {
  /* The SLM contains a list of operations and a list of instructions. The
   * instructions contain the same MOs as the list 'mos'. Therefore, we do not
   * delete MOs through the MIs. But, the MIs can also contain MOs not in 'mos'.
   * To distinguish them, we use the line number. First, we set the line number
   * of all MOs in 'mos' and the branchOperation to 1, the MI-Destructor deletes
   * any MOs with line number 0 and finally, the SLM can delete the other
   * operations. This was necessary to prevent MOs from getting freed twice.
   */
  if (ops) {
    for (std::vector<MO *>::iterator it = ops->begin(), end = ops->end();
         it != end; it++) {
      (*it)->setLineNumber(1);
    }
  }
  if (branchOperation)
    branchOperation->setLineNumber(1);
  if (ins) {
    delete ins;
    ins = 0;
  }
  if (ops) {
    for (std::vector<MO *>::iterator it = ops->begin(), end = ops->end();
         it != end; it++) {
      delete *it;
    }
    delete ops;
    ops = 0;
  }
  if (branchOperation)
    delete branchOperation;
  if (map != NULL) {
    for (VirtualRegisterMap::iterator mit = map->begin(); mit != map->end();
         ++mit) {
      delete mit->second;
    }
    delete map;
    map = 0;
  }
  if (alternative)
    delete alternative;
}

void SLM::releaseMIs() {
  for (auto it = ins->begin(); it != ins->end(); ++it)
    delete (*it);
  ins->clear();
}

void SLM::setAlternative(SLM **alt) {
  if (!(*alt))
    return;
  if (!(*alt)->ins || !(*alt)->map ||
      (ins && (*alt)->ins->size() > ins->size())) {
    delete (*alt);
    *alt = 0;
    return;
  }

  if (!alternative) {
    alternative = *alt;
    *alt = 0;
  } else if ((*alt)->ins->size() <= alternative->ins->size()) {
    delete alternative;
    alternative = (*alt);
    // The chromosome does no longer own the merged SLM, so that we can delete
    // the merged SLM independently of the chromosome.
    *alt = 0;
  } else {
    delete (*alt);
    *alt = 0;
  }
}

void releaseRegisterMappings(VirtualRegisterMap *map) {
  for (VirtualRegisterMap::iterator mit = map->begin(); mit != map->end();
       ++mit)
    delete mit->second;
  delete map;
}

bool SLM::setInstructions(Program **ins, VirtualRegisterMap **map) {
  if (!(*ins) && !(*map))
    return false;
  if (!(*ins)) {
    releaseRegisterMappings(*map);
    *map = 0;
    return false;
  }
  if (!(*map)) {
    delete *ins;
    *ins = 0;
    return false;
  }
  bool ret;

  Program **this_ins = &this->ins_per_thread[CURRENT_THREAD_NUM];
  VirtualRegisterMap **this_map = &this->map_per_thread[CURRENT_THREAD_NUM];
  if (!(*this_ins)) {
    *this_ins = *ins;
    *this_map = *map;
    *ins = 0;
    *map = 0;
    ret = true;
  } else if ((*this_ins)->size() >= (*ins)->size()) {
    // >= since slm might contain more dummy registers, size of slm can only be
    // checked here
    delete *this_ins;
    releaseRegisterMappings(*this_map);
    *this_ins = *ins;
    *this_map = *map;
    *ins = 0;
    *map = 0;
    ret = true;
  } else {
    delete *ins;
    releaseRegisterMappings(*map);
    *ins = 0;
    *map = 0;
    ret = false;
  }

  // cleanup. If the newly set instructions are better then the alternative SLM,
  // then set the alternative NULL otherwise it would be used for future use,
  // even though this one is better. it is also called if they are equally good,
  // because, why keep 2 SLM's, if one is sufficient. Save the memory!
  if (ret && alternative && alternative->ins->size() >= (*this_ins)->size()) {
    delete alternative;
    alternative = 0;
  }
  return ret;
}

void SLM::setInstructionAndMap(Program **ins, VirtualRegisterMap **map) {
  if (!(*ins) && !(*map))
    return;
  if (!(*ins)) {
    releaseRegisterMappings((*map));
    *map = 0;
    return;
  }
  if (!(*map)) {
    delete *ins;
    (*ins) = 0;
    return;
  }

  if (this->ins == NULL) {
    this->ins = *ins;
    this->map = *map;
    *ins = 0;
    *map = 0;
  } else if (this->ins->size() >= (*ins)->size()) {
    if (isPowerOptimization()) {
      // in power optimization only set instruction and map if
      // size is equal and new Cumulative TransitionEnergy is better
      if (this->ins->size() == (*ins)->size()) {
        if (compareTransitionEnergy(
                (*ins)->getCumulativeTransitionEnergy(),
                this->ins->getCumulativeTransitionEnergy())) {
          delete this->ins;
          releaseRegisterMappings(this->map);
          this->ins = *ins;
          this->map = *map;
          *ins = 0;
          *map = 0;
        }
      } else {
        // new solution is better, replace it
        delete this->ins;
        releaseRegisterMappings(this->map);
        this->ins = *ins;
        this->map = *map;
        *ins = 0;
        *map = 0;
      }

    } else {
      // default replacement of better/equal solutions
      delete this->ins;
      releaseRegisterMappings(this->map);
      this->ins = *ins;
      this->map = *map;
      *ins = 0;
      *map = 0;
    }
  } else {
    delete *ins;
    releaseRegisterMappings(*map);
    *ins = 0;
    *map = 0;
  }
}

void SLM::collectInstructionsFromThreads() {
  for (int i = 0; i < MAX_THREAD_COUNT; ++i) {
    this->setInstructionAndMap(&(this->ins_per_thread[i]),
                               &(this->map_per_thread[i]));
  }
}

void SLM::addMO(MO *operation) {
  if (operation->isBranchOperation()) {
    if (branchOperation)
      delete branchOperation;
    branchOperation = operation;
  } else {
    ops->push_back(operation);
  }
}

void SLM::setLabel(char32_t label) {
  this->label = label;
  char32_t error =
      -1; // otherwise we have a warning about comparing signed and unsigned.
  if (label != error) {
    label::registerSLM(label, this);
  }
}

void SLM::releaseGraph() {
  for (vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it) {
    (*it)->deleteFollowers();
  }
}

void SLM::releaseRegisterMapping() {
  if (map) {
    for (VirtualRegisterMap::iterator mit = map->begin(); mit != map->end();
         ++mit) {
      delete mit->second;
      mit->second = 0;
    }
    delete map;
    map = 0;
  }
}

int SLM::weightSum() {
  int sum = 0;
  for (vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it)
    sum += (*it)->getWeight();
  return sum;
}

int SLM::depthSum() {
  int sum = 0;
  for (vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it)
    sum += (*it)->getDepth();
  return sum;
}

void SLM::calculateGraph(Processor *pro, bool initialGraph) {
  if (this->minimalSize != -1)
    return;
  int row = 0;
  Processor::DMAAddrConfig *dmaConfig = pro->getDMAConfig();
  bool checkDMA = dmaConfig != NULL;
  for (vector<MO *>::iterator it = ops->begin(); it != ops->end();
       ++it, row++) {
    MO *mo1 = *it;
    if (registers::getNumberOfCondselRegister() > 1) {
      if (initialGraph) {
        if (!strncmp(mo1->getOperation()->getCond(), "CS", 2) ||
            !strncmp(mo1->getOperation()->getCond(), "CRS", 3)) {
          for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
            if (!strncmp((*ot)->getOperation()->getCond(), "CRS", 3)) {
              mo1->addFollowerFlags(*ot);
              mo1->addFollowerCondition(*ot);
              break;
            }
            if (!strncmp((*ot)->getOperation()->getCond(), "CS", 2)) {
              break;
            }
            if (!strncmp((*ot)->getOperation()->getCond(), "CR", 2)) {
              mo1->addFollowerFlags(*ot);
              mo1->addFollowerCondition(*ot);
            }
            char32_t *args_depend = (*ot)->getArguments();
            if (registers::isCondSel(args_depend[0]) ||
                registers::isFlag(args_depend[0])) {
              break;
            }
          }
        }
      }
    } else {
      if (!strncmp(mo1->getOperation()->getCond(), "CS", 2) ||
          !strncmp(mo1->getOperation()->getCond(), "CRS", 3)) {
        bool foundprev = false;
        for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
          if (!strncmp((*ot)->getOperation()->getCond(), "CS", 2) ||
              !strncmp((*ot)->getOperation()->getCond(), "CRS", 3)) {
            mo1->addFollowerFlags(*ot);
            foundprev = true;
            break;
          }
          if (!strncmp((*ot)->getOperation()->getCond(), "CR", 2)) {
            mo1->addFollowerFlags(*ot);
          }
        }
        if (branchOperation != NULL && !foundprev &&
            branchOperation->getOperation()->getName().compare(0, 2, "BS") == 0)
          mo1->addFollowerFlags(branchOperation);
      }
      if (!strncmp(mo1->getOperation()->getCond(), "CR", 2)) {
        for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
          if (!strncmp((*ot)->getOperation()->getCond(), "CS", 2) ||
              !strncmp((*ot)->getOperation()->getCond(), "CRS", 3)) {
            (*it)->addWeakFollowerFlags(*ot);
            //						break;
          }
          char32_t *args_depend = (*ot)->getArguments();
          for (int e = 0; e < (*ot)->getArgNumber(); ++e) {
            if (registers::isCondSel(args_depend[e])) {
              mo1->addFollower(*ot);
            }
          }
        }
      }
    }
    // all RCU have dependencies between them. so the store and load operation
    // must preserve their order.
    if (mo1->getOperation()->getName().substr(0, 11) == "STORERCUPPL") {
      for (auto ot = it + 1; ot != ops->end(); ++ot) {
        if ((*ot)->getOperation()->getName().substr(0, 11) == "STORERCUPPL") {
          mo1->addFollower(*ot);
          break;
        }
        if ((*ot)->getOperation()->getName().substr(0, 7) == "LOADRCU") {
          mo1->addFollower(*ot);
          // no break, because we want to find next STORERCUPPL, if exists
        }
      }
    } else if (mo1->getOperation()->getName().substr(0, 8) == "STORERCU") {
      for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
        if (!strncmp((*ot)->getOperation()->getName().c_str(),
                     (*it)->getOperation()->getName().c_str(), 9)) {
          mo1->addFollower(*ot);
          break;
        }
        if ((*ot)->getOperation()->getName().substr(0, 7) == "LOADRCU" &&
            (*it)->getOperation()->getName().c_str()[8] ==
                (*ot)->getOperation()->getName().c_str()[7]) {
          mo1->addFollower(*ot);
          break;
        }
      }
    }

    if (!strncmp(mo1->getOperation()->getName().c_str(), "LOADRCU", 7)) {
      for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
        if (!strncmp((*ot)->getOperation()->getName().c_str(),
                     mo1->getOperation()->getName().c_str(), 8)) {
          mo1->addFollower(*ot);
          break;
        }
        if (strncmp((*ot)->getOperation()->getName().c_str(), "STORERCUPPL",
                    11) &&
            !strncmp((*ot)->getOperation()->getName().c_str(), "STORERCU", 8) &&
            mo1->getOperation()->getName().c_str()[7] ==
                (*ot)->getOperation()->getName().c_str()[8]) {
          mo1->addWeakFollower(*ot);
          break;
        }
      }
    }

    /* Set dependencies between DMA operations.
     * - A STORE to a DMA control register has the next STORE to DMA_CTRL_START
     * as a follower
     * - A STORE to DMA_CTRL_START has all following STOREs to DMA control
     * registers as follower; up to the next CTRL_START...
     */
    if (checkDMA && dmaConfig->isDMAStore(mo1)) {
      bool dmaCtrlAddr = dmaConfig->isDMACtrlAddr(mo1);
      for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
        if (dmaCtrlAddr) {
          if (dmaConfig->isDMAStore(*ot)) {
            mo1->addFollower(*ot);
            if (dmaConfig->isDMACtrlAddr(*ot))
              break;
          }
        } else {
          if (dmaConfig->isDMACtrlAddr(*ot)) {
            mo1->addFollower(*ot);
            break;
          }
        }
      }
    }

    /* Write indirect should have dependency to following PUT_DMA (STORE to
     * DMACtrlAddr) */
    if (checkDMA && mo1->isWriteIndirect()) {
      for (auto ot = it + 1; ot != ops->end(); ++ot) {
        if (dmaConfig->isDMACtrlAddr(*ot)) {
          mo1->addFollower(*ot);
          break;
        }
      }
    }

    // Special move into PERMREG has to have a dependency with the operation.
    if (!strncmp(mo1->getOperation()->getName().c_str(), "SMV", 3)) {
      std::string reg = registers::getName(mo1->getArguments()[0]);
      if (!strncmp(reg.c_str(), "PERMREG", 7)) {
        for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
          if (!strncmp(reg.c_str(), (*ot)->getOperation()->getName().c_str(),
                       8)) {
            mo1->addFollower(*ot);
          }
        }
      }
    }

    // PERMREG operation has to have dependency with SMV to special register
    if (!strncmp(mo1->getOperation()->getName().c_str(), "PERMREG", 7)) {
      char name[9];
      memset(name, 0, 9);
      strncpy(name, mo1->getOperation()->getName().c_str(), 8);
      for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
        if (!strncmp((*ot)->getOperation()->getName().c_str(), "SMV", 3)) {
          std::string reg = registers::getName((*ot)->getArguments()[0]);
          if (!reg.compare(name)) {
            mo1->addWeakFollower(*ot);
          }
        }
      }
    }

    // Special move into RNDCTRL has to have a dependency with the operation.
    if (!strncmp(mo1->getOperation()->getName().c_str(), "SMV", 3)) {
      std::string reg = registers::getName(mo1->getArguments()[0]);
      if (!strncmp(reg.c_str(), "RNDCTRL", 7)) {
        for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
          if (!strncmp((*ot)->getOperation()->getName().c_str(), "AVGRC", 5)) {
            mo1->addFollower(*ot);
          }
        }
      }
    }

    // RNDCTRL operation has to have dependency with SMV to special register
    if (!strncmp(mo1->getOperation()->getName().c_str(), "AVGRC", 5)) {
      //      char name[7];
      //      memset(name, 0, 7);
      //      strncpy(name, mo1->getOperation()->getName().c_str(), 6);
      for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
        if (!strncmp((*ot)->getOperation()->getName().c_str(), "SMV", 3)) {
          std::string reg = registers::getName((*ot)->getArguments()[0]);
          if (!strncmp(reg.c_str(), "RNDCTRL", 7)) {
            mo1->addWeakFollower(*ot);
          }
        }
      }
    }

    // Add generic condition dependency between CR and SMV to REGISTER!
    if (mo1->getOperation()->isCROperation()) {
      // find next SMV with FLAG or CONDSEL to add dependency
      for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
        if (!strncmp((*ot)->getOperation()->getName().c_str(), "SMV", 3)) {
          std::string reg = registers::getName((*ot)->getArguments()[0]);
          std::string reg2 = registers::getName((*ot)->getArguments()[1]);

          bool foundCONDSELTarget = reg.find("CONDSEL") != std::string::npos;
          bool foundCONDSELSource = reg2.find("CONDSEL") != std::string::npos;
          bool foundFLAGSTarget = reg.find("FLAGS") != std::string::npos;
          bool foundFLAGSSource = reg2.find("FLAGS") != std::string::npos;
          if (foundCONDSELTarget or foundCONDSELSource or foundFLAGSTarget or
              foundFLAGSSource) {
            mo1->addWeakFollower(*ot);
          }
        }
      }
    }

    // If an operation is selfdependent, generate dependencies to following
    // operations with the same name
    if (mo1->getOperation()->isSelfdependent()) {
      string opname = mo1->getOperation()->getBaseName();
      for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
        if ((*ot)->getOperation()->getBaseName() == opname)
          mo1->addFollower(*ot);
      }
    }

    // Check argument dependencies
    char32_t *args_first = mo1->getArguments();
    OPtype *types_first = mo1->getTypes();
    char *dir_first = mo1->getDirections();
    for (int firstArgIdx = 0; firstArgIdx < mo1->getArgNumber();
         ++firstArgIdx) {
      char32_t firstArg = args_first[firstArgIdx];
      char firstDir = dir_first[firstArgIdx];
      if (registers::getNumberOfCondselRegister() > 1) {
        if (initialGraph && firstArgIdx == 0) {
          if (registers::isCondSel(args_first[0])) {
            for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
              if (!strncmp((*ot)->getOperation()->getCond(), "CS", 2) ||
                  !strncmp((*ot)->getOperation()->getCond(), "CR", 2) ||
                  !strncmp((*ot)->getOperation()->getCond(), "CRS", 3)) {
                mo1->addFollower(*ot);
                mo1->addFollowerCondition(*ot);
              }
              char32_t *follower_args_depend = (*ot)->getArguments();
              if (registers::isFlag(follower_args_depend[0])) {
                mo1->addFollower(*ot);
                mo1->addFollowerCondition(*ot);
              }
              if (registers::isCondSel(follower_args_depend[0])) {
                goto break_condel;
              }
            }
          }
        break_condel:;

          if (registers::isFlag(args_first[0])) {
            for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
              if (!strncmp((*ot)->getOperation()->getCond(), "CR", 2)) {
                mo1->addFollower(*ot);
                mo1->addFollowerCondition(*ot);
              }
              char32_t *follower_args_depend = (*ot)->getArguments();
              if (registers::isCondSel(follower_args_depend[0])) {
                mo1->addFollower(*ot);
                mo1->addFollowerCondition(*ot);
              }
              if (registers::isFlag(follower_args_depend[0]) ||
                  !strncmp((*ot)->getOperation()->getCond(), "CS", 2)) {
                goto break_flag;
              }
            }
          }
        break_flag:;
        }
      } else {
        if (registers::isCondSel(firstArg) || registers::isFlag(firstArg)) {
          for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
            if (!strncmp((*ot)->getOperation()->getCond(), "CR", 2)) {
              mo1->addFollower(*ot);
            }
          }
        }
      }

      // iterate through the next MOs and find dependencies
      for (vector<MO *>::iterator ot = it + 1; ot != ops->end(); ++ot) {
        MO *mo2 = *ot;
        char32_t *args_second = mo2->getArguments();
        OPtype *types_second = mo2->getTypes();
        char *dir_second = mo2->getDirections();
        for (int secondArgIdx = 0; secondArgIdx < mo2->getArgNumber();
             ++secondArgIdx) {
          char32_t secondArg = args_second[secondArgIdx];
          char secondDir = dir_second[secondArgIdx];
          if ((types_second[secondArgIdx] == REG &&
               types_first[firstArgIdx] == REG)) { // and both are registers
            if (registers::equals(firstArg, secondArg)) { // and they are equal
              if (registers::getNumberOfCondselRegister() > 1) {
                if (strncmp(registers::getName(secondArg).c_str(), "CONDSEL",
                            7) &&
                    strncmp(registers::getName(secondArg).c_str(), "ALLCONDSEL",
                            10) &&
                    strncmp(registers::getName(secondArg).c_str(), "FLAGS",
                            5) &&
                    strncmp(registers::getName(secondArg).c_str(), "ALLFLAGS",
                            8)) {
                  if (registers::isFirReadReg(firstArg) ||
                      (registers::directFir(firstArg) &&
                       (firstDir & READ))) { // using address from FIR reg
                    /*[FG] TODO: At this point, we do not consider the latency
                     * of the SMV operation and only allow the SMV in parallel
                     * to the operation that reads from the FIR register. But
                     * actually, the SMV could be placed even before that
                     * operation! (The same is true for PERMREG and SMV
                     * combinations, which are handled above!)
                     */
                    if (registers::isFirWriteReg(secondArg) ||
                        (registers::directFir(secondArg) &&
                         (secondDir & WRITE)))
                      mo1->addWeakFollower(mo2);
                  } else if (registers::isFirWriteReg(firstArg) ||
                             (registers::directFir(firstArg) &&
                              (firstDir & WRITE))) { // we write FIR content
                    mo1->addFollower(mo2);
                  } else if (secondDir == READWRITE) {
                    // target of the register dependency is a readwrite
                    // operation. After this the following search is aborted!
                    if (firstDir & READ) {
                      // if it is also part of the source operands this has to
                      // be a read dependency
                      if (((firstArgIdx == 0 || firstArg == 4) &&
                           (registers::equals(firstArg, args_second[1]) ||
                            registers::equals(firstArg, args_second[2]))) ||
                          (firstDir == READWRITE &&
                           (registers::equals(firstArg, args_second[0]) ||
                            registers::equals(firstArg, args_second[4])))
                          //  || registers::equals(firstArg, args_second[0])
                          // || registers::equals(firstArg, args_second[4])
                          // mo2 has to read from the register
                      ) {
                        mo1->addFollower(mo2);
                      } else {
                        mo1->addWeakFollower(mo2);
                      }
                    } else {
                      mo1->addFollower(mo2);
                    }
                    goto short_path_for_me;
                  } else if (firstDir & WRITE) {
                    if (secondDir & WRITE) {
                      mo1->addWeakFollower(mo2);
                    } else {
                      mo1->addFollower(mo2);
                    }

                  } else if (secondDir & WRITE) {
                    mo1->addWeakFollower(mo2);
                    goto short_path_for_me;
                  }
                }
              } else {
                if (registers::isFirReadReg(firstArg) ||
                    (registers::directFir(firstArg) &&
                     (firstDir & READ))) { // using address from FIR reg
                  /*[FG] TODO: At this point, we do not consider the latency of
                   * the SMV operation and only allow the SMV in parallel to the
                   * operation that reads from the FIR register. But actually,
                   * the SMV could be placed even before that operation! (The
                   * same is true for PERMREG and SMV combinations, which are
                   * handled above!)
                   */
                  if (registers::isFirWriteReg(secondArg) ||
                      (registers::directFir(secondArg) && (secondDir & WRITE)))
                    mo1->addWeakFollower(mo2);
                } else if (registers::isFirWriteReg(firstArg) ||
                           (registers::directFir(firstArg) &&
                            (firstDir & WRITE))) { // we write FIR content
                  mo1->addFollower(mo2);
                } else if (firstDir & WRITE) {
                  mo1->addFollower(mo2);
                } else if (secondDir & WRITE) {
                  mo1->addWeakFollower(mo2);
                  goto short_path_for_me;
                }
              }
            } else if (registers::getName(firstArg) == "OFFSET" &&
                       (firstDir & WRITE) &&
                       registers::isFirOffsetReg(secondArg)) {
              mo1->addFollower(mo2);
            } else if (registers::isFirOffsetReg(firstArg) &&
                       (registers::getName(secondArg) == "OFFSET") &&
                       (secondDir & WRITE)) {
              mo1->addWeakFollower(mo2);
            }
          }
        }
      }
    short_path_for_me:;
    }
  }

  // Special Construction for the branch operation.
  // reverse check if there are operations having a dependency with it.
  if (branchOperation != NULL) {
    char32_t *args_depend = branchOperation->getArguments();
    OPtype *types_depend = branchOperation->getTypes();
    for (int e = 0; e < branchOperation->getArgNumber(); ++e) {
      if (types_depend[e] != REG)
        continue;
      for (vector<MO *>::reverse_iterator it = ops->rbegin(); it != ops->rend();
           ++it, row++) {
        char32_t *args_second = (*it)->getArguments();
        OPtype *types_second = (*it)->getTypes();
        for (int i = 0; i < (*it)->getArgNumber(); ++i) {
          if ((types_second[i] == REG)) { // is register
            if (registers::equals(args_depend[e],
                                  args_second[i])) { // and they are equal
              (*it)->addFollower(branchOperation);
              if (util::strStartsWith(
                      branchOperation->getOperation()->getBaseName(), "SJL") &&
                  util::strStartsWith((*it)->getOperation()->getName(),
                                      "SMV")) {
                (*it)->reSetLatency(getStackRegLatency());
              }
              goto second_short_path_for_me;
            }
          }
        }
      }
    second_short_path_for_me:;
    }
  }

  if (registers::getNumberOfCondselRegister() > 1) {
    // Special CRBranchOperation for the branch operation.
    // reverse check if there are operations having a dependency with it.
    if (branchOperation != NULL && branchOperation->isCRBranchOperation()) {
      for (vector<MO *>::reverse_iterator it = ops->rbegin(); it != ops->rend();
           ++it, row++) {
        if (!strncmp((*it)->getOperation()->getCond(), "CS", 2) ||
            !strncmp((*it)->getOperation()->getCond(), "CRS", 3)) {
          (*it)->addFollower(branchOperation);
          (*it)->addFollowerCondition(branchOperation);
          goto third_short_path_for_me;
        }
      }
    }
  third_short_path_for_me:;
  }

  // calculate the number of follower.
  for (vector<MO *>::iterator it = ops->begin(); it != ops->end();
       ++it, row++) {
    (*it)->getNumberOfFollower();
  }
  // calculate the minimal size of the SLM.
  // it is either if all operations fill all issue slots,
  int x = MI::getNumberIssueSlots() - 1;
  for (vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it) {
    x += (*it)->opLengthMultiplier();
  }
  minimalSize = x / MI::getNumberIssueSlots();

  // or the longest path through the Operations.
  for (vector<MO *>::iterator it = ops->begin(); it != ops->end();
       ++it, row++) {
    if ((*it)->getWeight() > (int)criticalPath) {
      criticalPath = (*it)->getWeight();
    }
  }
  if (criticalPath > minimalSize)
    minimalSize = criticalPath;
}

void SLM::writeOutDot(Processor *pro, const char *path) {
  std::ofstream ofs(path);
  writeOutDot(pro, ofs);
  ofs.close();
}

void SLM::writeOutDot(Processor *pro, ostream &out) {
  if (pro->getIssueSlotNumber() != 1) {
    cout << "WARNING: Coupled Register are only correctly calculated if only "
            "ONE Issue Slot exists"
         << std::endl;
  }
  calculateGraph(pro, false);
  MO *longest = NULL;
  out << endl << "subgraph " << id << "{" << endl;
  // print physical Register of previous SLM
  ass_reg_t *blocked = blockedRegsInSLM.find(getOriginalID())->second;
  int *freeReg = new int[registers::getNumRegisterFiles()];
  calcFreeReg(blocked, freeReg);
  auto blockedRegs = getBlockedRegisters(blocked);

  // update RDG so that the coupled register can be displayed
  // creators of coupled register block 2 register (bug in scheduler)
  std::unique_ptr<VirtualRegisterMap> map =
      std::make_unique<VirtualRegisterMap>();
  std::unique_ptr<RegisterCoupling> couplings =
      std::make_unique<RegisterCoupling>();
  int notScheduableCSCR;
  Program *ins =
      gen_sched::scheduleSLM(0, this, pro, notScheduableCSCR, Context("sched"));
  calculateCouplings(ins, map.get(), getFreeReg(), *(couplings.get()), pro);
  rdg::RDG *rdg = rdg::RDG::analyseRegisterDependencies(
      ins, couplings.get(), map.get(), blocked, false);

  // print creation blocks
  for (auto node : blockedRegs) {
    int regFile, regNumber;
    registers::getPhysicalRegister(node, &regFile, &regNumber);
    out << registers::getDotOffset() + node << " [label=\"V" << regFile << "R"
        << regNumber << "\",shape=box];" << endl;
  }

  for (vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it) {
    if (longest == NULL || longest->getWeight() < (*it)->getWeight())
      longest = *it;
  }
  for (vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it) {
    if ((*it)->getWeight() == longest->getWeight())
      (*it)->setOnLongestPath();
  }
  for (vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it) {
    (*it)->writeOutDot(out, nullptr, 0, rdg, map.get());
    out << endl;
  }
  if (branchOperation != NULL) {
    branchOperation->writeOutDot(out);
    out << endl;
  }
  out << "stats [label=\"Longest Path: ";
  if (longest != NULL)
    out << longest->getWeight();
  else if (branchOperation != NULL)
    out << branchOperation->getWeight();
  else
    out << "0";
  out << "\\nNodes: " << ops->size();
  if (label != char32_t(-2))
    out << "\\nLabel: " << label::getLabelName(label);
  out << "\",shape=polygon,color=blue]\n";

  // create Termination Node
  auto terminationID = 32000 + numRegisterFiles * numRegisters + 1;
  out << terminationID << " [label=\"  Termination Node\",shape=box,color=red];"
      << endl;
  for (auto node : blockedRegs) {
    out << registers::getDotOffset() + node << " -> " << terminationID << ";"
        << endl;
  }

  // delete pointer
  delete[] freeReg;
  delete ins;
  delete rdg;

  out << "}" << endl;
}

void SLM::writeOutBin(ostream &out) {
  if (alternative != NULL) {
    alternative->writeOutBin(out);
    return;
  }
  for (auto it = ins->begin(), end = ins->end(); it != end; it++)
    (*it)->writeOutBin(out);
}

void SLM::writeOutReadable(ostream &out, bool printEnergy) {
  //  if (printEnergy) {
  //    TransitionEnergyEstimator estimator;
  //    auto mapping = getVirtualMapping();
  //    for (Program::iterator it = ins->begin(), end = ins->end(); it != end;
  //         it++) {
  //      estimator.push(*it, mapping);
  //      it.operator*()->setTransitionEnergy(estimator.getLastTransitionEnergy());
  //    }
  //    ins->setCumulativeTransitionEnergy(estimator.getTransitionEnergy());
  //  }
  char32_t error = -2; // -1 is the Label ID for NO_SCHEDULE and -2 is the error
  // and default value.
  if (this->label != error)
    out << ":L_" << label::getLabelName(this->label) << " (" << this->label
        << ")" << endl;

  if (alternative != NULL) {
    alternative->writeOutReadable(out, printEnergy);
    return;
  }
  if (!ins || ins->size() == 0)
    return;

  int assemblerLine = offset;
  for (Program::iterator it = ins->begin(), end = ins->end(); it != end; it++) {
    (*it)->registerMO2AssemblerLine(assemblerLine);
    out << "(0x" << std::setw(3) << std::setfill('0') << std::hex
        << assemblerLine++ << std::dec << ") ";
    (*it)->writeOutReadable(out, nullptr, printEnergy);
  }

  if (printEnergy) {
    out << "Total TransitionEnergy [pJ] (RA + Instruction energy) = "
        << ins->getCumulativeTransitionEnergy() << " + "
        << InstructionModel::getInstance().generateInstructionEnergy(ins)
        << std::endl;
    out << "Instruction Transitions= " << ins->getInstructionTransitions()
        << std::endl;
  }
}

void SLM::writeOutCompilable(ostream &out) {
  char32_t error = -2; // -1 is the Label ID for NO_SCHEDULE and -2 is the error
  // and default value.
  if (this->label != error)
    out << ":L_" << label::getLabelName(this->label) << " (" << this->label
        << ")" << endl;

  if (alternative != NULL) {
    alternative->writeOutCompilable(out);
    return;
  }
  if (!ins || ins->size() == 0)
    return;

  int assemblerLine = offset;
  for (Program::iterator it = ins->begin(), end = ins->end(); it != end; it++) {
    (*it)->registerMO2AssemblerLine(assemblerLine);
    // out << "(0x" << std::setw(3) << std::setfill('0') << std::hex
    //    << assemblerLine++ << std::dec << ") ";
    (*it)->writeOutCompilable(out);
  }
}

void SLM::writeOutTransitionPowerEstimate(std::ostream &out) {
  if (alternative != NULL) {
    alternative->writeOutTransitionPowerEstimate(out);
    return;
  }
  if (!ins || ins->size() == 0)
    return;

  int assemblerLine = offset;

  TransitionEnergyEstimator estimator;
  auto mapping = getVirtualMapping();
  if (mapping) {
    double instructionEnergy =
        InstructionModel::getInstance().generateInstructionEnergy(ins);
    cout << " Instruction Energy " << instructionEnergy << std::endl;
    for (Program::iterator it = ins->begin(), end = ins->end(); it != end;
         it++) {
      estimator.push(*it, mapping);
      out << estimator.getLastTransitionEnergy() << endl;
    }
    cout << "InstructionEnergy = " << instructionEnergy << endl;
  }
}

void SLM::calculateTransitionEnergy() {
  if (alternative != NULL) {
    alternative->calculateTransitionEnergy();
    return;
  }
  if (!ins || ins->size() == 0)
    return;

  int assemblerLine = offset;
  TransitionEnergyEstimator estimator;
  double totalTransitionEnergy = 0;
  auto mapping = getVirtualMapping();
  if (mapping) {
    for (Program::iterator it = ins->begin(), end = ins->end(); it != end;
         it++) {
      estimator.push(*it, mapping);
      double energy = estimator.getLastTransitionEnergy();
      it.operator*()->setTransitionEnergy(energy);
      totalTransitionEnergy += energy;
    }

    if (fabs(totalTransitionEnergy - ins->getCumulativeTransitionEnergy()) >
        std::numeric_limits<double>::epsilon()) {
      if (ins->getCumulativeTransitionEnergy() > 0) {
        stringstream ss;
        ss << "total Recalculated Energy= " << totalTransitionEnergy
           << "differs from " << ins->getCumulativeTransitionEnergy() << endl;
        //        throw logic_error(ss.str());
      } else {
        ins->setCumulativeTransitionEnergy(totalTransitionEnergy);
      }
    }
  }
}

void SLM::initErrorMessage(Program *ins) {
  stringstream ss;
  if(ins)
    ins->writeOutScheduledWeight(ss, getOriginalID());
  else
      ss << "Scheduling ERROR!";
  this->error_string = ss.str();
}