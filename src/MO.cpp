// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "MO.h"
#include "../export/ScheduledMOCharacteristics.h"
#include "global.h"
#include "operation.h"
#include <algorithm>
#include <set>

int MO::counter[MAX_THREAD_COUNT];

MO::MO(Operation *operation, char32_t *arguments, OPtype *typ, char *opdir) {
  parting = false;
  op = operation;
  lineNumber = -1;
  lineNumber2 = -1;
  argNumber = 0;
  follower = -1;
  weight = 0;
  depth = 0;
  latency = operation->getLatency();
  issueSlotContraint = 0;
  reorderable = true;
  onLongesPath = false;
  CROperation = false;
  CSOperation = false;
  CRSOperation = false;
  SMVtoSingleCondselFlag = false;
  SMVtoAllCondselFlag = false;

  int i;
  for (i = 0; i < MAXARGNUMBER; i++) {
    arg[i] = arguments[i];
    types[i] = typ[i];
    if (arg[i] != 0 && typ[i] == Error)
      types[i] = REG;
    dir[i] = opdir[i];
    if (types[i] != Error)
      argNumber = i + 1;
  }
  this->id = counter[CURRENT_THREAD_NUM]++;

  idConditions = id;
#if STORE_PARTINGS_AND_ALONE
  wasParting = false;
  wasAlone = false;
#endif
  finalLatency = -1;
}

MO::MO(Operation *operation) {
  parting = false;
  op = operation;
  lineNumber = 0;
  lineNumber2 = 0;
  argNumber = 0;
  follower = -1;
  weight = 0;
  depth = 0;
  latency = operation->getLatency();
  issueSlotContraint = 0;
  reorderable = true;
  onLongesPath = false;
  CROperation = false;
  CRSOperation = false;
  CSOperation = false;
  SMVtoSingleCondselFlag = false;
  SMVtoAllCondselFlag = false;
  int i;
  for (i = 0; i < MAXARGNUMBER; i++) {
    arg[i] = 0;
    types[i] = Error;
    dir[0] = 0;
  }
  this->id = counter[CURRENT_THREAD_NUM]++;

  idConditions = id;
#if STORE_PARTINGS_AND_ALONE
  wasParting = false;
  wasAlone = false;
#endif
  finalLatency = -1;
}

MO *MO::copy() {
  MO *ret = new MO(op, arg, types, dir);
  ret->latency = latency;
  ret->lineNumber = lineNumber;
  ret->lineNumber2 = lineNumber2;
  ret->issueSlotContraint = issueSlotContraint;
  ret->parting = parting;
  ret->id = id;

#if STORE_PARTINGS_AND_ALONE
  ret->wasAlone = wasAlone;
  ret->wasParting = wasParting;
#endif

  ret->reorderable = reorderable;
  ret->CROperation = CROperation;
  ret->CRSOperation = CRSOperation;
  ret->CSOperation = CSOperation;
  ret->SMVtoSingleCondselFlag = SMVtoSingleCondselFlag;
  ret->SMVtoAllCondselFlag = SMVtoAllCondselFlag;
  ret->followingCondition = followingCondition;
  ret->previousCondition = previousCondition;
  ret->idConditions = idConditions;
  ret->finalLatency = finalLatency;

  /* Depdency related data is not copied here!
   * Currently, the only place, where MOs are copied is in the X2 merging. There
   * we generate a new SLM with copies of MOs and the dependency analysis is
   * performed afterwards. If we copy dependency information here, the vectors
   * (following, previous) might contain pointers to "original" MOs, which might
   * lead to problems during scheduling, as those "original" MOs are not placed
   * (they are not part of the new SLM), so the copies that depend on them never
   * get available/schedulable.
   */
  return ret;
}

void MO::setOperation(Operation *operation) {
  op = operation;
  latency = operation->getLatency();
}

bool MO::isWriteReg(int argIdx) {
  return types[argIdx] == REG &&
         (dir[argIdx] == WRITE || dir[argIdx] == READWRITE ||
          dir[argIdx] == READWRITEPSEUDOREAD);
}

bool MO::isVirtualWriteArg(int argIdx) {
  return isWriteReg(argIdx) && registers::isVirtualReg(arg[argIdx]);
}

bool MO::isPhysicalWriteArg(int argIdx) {
  return isWriteReg(argIdx) && registers::isPhysicalRegister(arg[argIdx]);
}

bool MO::isMemStore(Processor::DMAAddrConfig *dmaAddrConfig) {
  string name = getOperation()->getBaseName();
  if (name == "STORE" || name == "STOREI" || name == "STOREIL") {
    uint32_t addr = getArguments()[0];
    return (!dmaAddrConfig || addr < dmaAddrConfig->startAddr ||
            addr > dmaAddrConfig->endAddr);
  } else
    return false;
}

bool MO::isMemLoad() { return getOperation()->getBaseName() == "LOAD"; }

void MO::addFollower(MO *next) {
  // if there is a weak follower, remove it, since it is now a strong follower
  // only add weak follower, if there is no strong follower
  auto it = std::find_if(
      weakFollowing.begin(), weakFollowing.end(),
      [&next](const MO *mo) { return mo->getID() == next->getID(); });

  if (it != weakFollowing.end()) {
    // remove weak follower
    weakFollowing.erase(it);
  }
  // so that no follower can be added multiple times
  for (std::vector<MO *>::iterator it = following.begin();
       it != following.end(); it++) {
    if ((*it)->getID() == next->getID())
      return;
  }
  next->previous.push_back(this);
  following.push_back(next);
}

void MO::addWeakFollower(MO *next) {
  // only add weak follower, if there is no strong follower
  auto it =
      std::find_if(following.begin(), following.end(), [&next](const MO *mo) {
        return mo->getID() == next->getID();
      });

  if (it != following.end())
    return;
  for (std::vector<MO *>::iterator it = weakFollowing.begin();
       it != weakFollowing.end(); ++it) {
    if ((*it)->getID() == next->getID())
      return;
  }
  weakFollowing.push_back(next);
  next->weakPrevious.push_back(this);
}

void MO::addFollowerCondition(MO *next) {
  // so that no follower can be added multiple times
  for (std::vector<MO *>::iterator it = followingCondition.begin();
       it != followingCondition.end(); it++) {
    if ((*it)->getID() == next->getID())
      return;
  }
  next->previousCondition.push_back(this);
  followingCondition.push_back(next);
}

void MO::addFollowerFlags(MO *next) {
  for (auto it = followingFlags.begin(); it != followingFlags.end(); ++it) {
    if ((*it)->getID() == next->getID())
      return;
  }
  followingFlags.push_back(next);
  next->previousFlags.push_back(this);
}

void MO::addWeakFollowerFlags(MO *next) {
  for (auto it = weakFollowingFlags.begin(); it != weakFollowingFlags.end();
       ++it) {
    if ((*it)->getID() == next->getID())
      return;
  }
  weakFollowingFlags.push_back(next);
  next->weakPreviousFlags.push_back(this);
}

void MO::deleteFollowers() {
  following.clear();
  previous.clear();
  weakFollowing.clear();
  followingFlags.clear();
  previousFlags.clear();
  weakFollowingFlags.clear();
  weight = 0;
  follower = -1;
}

int MO::countFollower(int weight) {
  if (weight == 0)
    return following.size() + followingFlags.size();
  int count = following.size() + followingFlags.size();
  weight--;
  for (std::vector<MO *>::iterator it = following.begin();
       it != following.end(); it++)
    count += (*it)->countFollower(weight);
  for (auto it = followingFlags.begin(); it != followingFlags.end(); ++it)
    count += (*it)->countFollower(weight);
  return count;
}

int MO::getWeight() {
  if (weight == 0) {
    for (std::vector<MO *>::iterator it = following.begin();
         it != following.end(); it++) {
      int tmp = (*it)->getWeight();
      if (tmp > weight)
        weight = tmp;
    }
    for (auto it = followingFlags.begin(); it != followingFlags.end(); ++it) {
      int tmp = (*it)->getWeight();
      if (tmp > weight)
        weight = tmp;
    }
    int lat = (op->getBaseName() == "STORERCUPPL") ? 1 : latency;
    weight += lat;
  }
  return weight;
}

int MO::getDepth() {
  if (depth == 0) {
    for (std::vector<MO *>::iterator it = previous.begin();
         it != previous.end(); ++it) {
      int tmp = (*it)->getDepth();
      if (tmp > depth)
        depth = tmp;
    }
    for (auto it = previousFlags.begin(); it != previousFlags.end(); ++it) {
      int tmp = (*it)->getDepth();
      if (tmp > depth)
        depth = tmp;
    }
    depth += latency;
  }
  return depth;
}

void MO::calculateFollower(set<MO *> *IDs) {
  if (IDs->find(this) != IDs->end())
    return; // cancel if the node has already been processed.
  IDs->insert(this);
  for (std::vector<MO *>::iterator it = following.begin();
       it != following.end(); it++)
    (*it)->calculateFollower(IDs);
  for (auto it = followingFlags.begin(); it != followingFlags.end(); ++it)
    (*it)->calculateFollower(IDs);
}

int MO::getNumberOfFollower() {
  if (follower == -1) {
    set<MO *> allIDs;
    calculateFollower(&allIDs);
    follower = allIDs.size();
  }
  return follower;
}

bool MO::isWriteIndirect() {
  for (int argIdx = 0; argIdx < this->getArgNumber(); ++argIdx) {
    if (dir[argIdx] & WRITE) {
      if (registers::isFirReg(arg[argIdx]))
        return true;
    }
  }
  return false;
}

bool MO::isReadIndirect() {
  for (int argIdx = 0; argIdx < this->getArgNumber(); ++argIdx) {
    if (dir[argIdx] & READ) {
      if (registers::isFirReg(arg[argIdx]))
        return true;
    }
  }
  return false;
}

std::string
MO::printSchedChromosomeInfo(gen_sched::sched_chromosome *schedChromosome,
                             int index) {
  if (!schedChromosome) {
    return "";
  } else {
    stringstream ss;
    ss << ";weight=" << schedChromosome->weights[index];
    ss << ";alone=";
    if (schedChromosome->alone[index]) {
      ss << "1";
    } else {
      ss << "0";
    }
    ss << ";parting=";
    if (schedChromosome->partings[index]) {
      ss << "1";
    } else {
      ss << "0";
    }
    ss << ";";
    return ss.str();
  }
}

std::string MO::get_register_info() {
  bool equal_src = arg[1] == arg[2];
  bool equal_read2 = (arg[4] == arg[1]) or (arg[4] == arg[2]);
  bool equal_read1 = (arg[0] == arg[1]) or (arg[0] == arg[2]);
  bool equal_read_write = equal_read1 or equal_read2;
  stringstream ss;
  ss << "equal_src=" << equal_src << ";equal_read_write=" << equal_read_write;
  return ss.str();
}

const RegisterCouple *MO::isCoupledWriteArg(const rdg::RDG *rdg) const {
  for (int argIdx = 0; argIdx < argNumber; ++argIdx) {
    char32_t argument = arg[argIdx];
    if (types[argIdx] == REG &&
        (dir[argIdx] == WRITE ||
         dir[argIdx] ==
             READWRITEPSEUDOREAD)) { // only target registers are interesting

      const rdg::RDG::Entry *e = rdg->getEntry(argument);
      return e->couple;
    }
  }
  return nullptr;
}

void MO::writeOutDot(ostream &out, gen_sched::sched_chromosome *schedChromosome,
                     int index, const rdg::RDG *rdg,
                     const VirtualRegisterMap *map) {
  int n = 2;
  for (std::vector<MO *>::const_iterator it = following.begin();
       it != following.end(); it++) {
    out << id << " -> " << (*it)->getID();
    if (latency != 1)
      out << "[label=\"" << latency << "\"]";
    int lat = (op->getBaseName() == "STORERCUPPL") ? 1 : latency;
    if (onLongesPath && (*it)->getWeight() == (weight - lat))
      out << "[color=red]";
    out << ";" << std::endl;
  }
  for (auto it = weakFollowing.begin(); it != weakFollowing.end(); ++it) {
    out << id << " -> " << (*it)->getID();
    out << "[style=dashed];" << std::endl;
  }
  for (auto it = followingFlags.begin(); it != followingFlags.end(); ++it) {
    out << id << " -> " << (*it)->getID();
    out << "[label=\"F";
    if (latency != 1)
      out << latency - getForwarding();
    out << "\"]";
  }
  for (auto it = weakFollowingFlags.begin(); it != weakFollowingFlags.end();
       ++it) {
    out << id << " -> " << (*it)->getID();
    out << "[label=\"F";
    if (latency != 1)
      out << latency - getForwarding();
    out << "\",style=dashed]";
  }
  out << id << " [label=\"" << lineNumber;
  if (lineNumber2 > 0 && lineNumber2 != (uint)-1)
    out << "+" << lineNumber2;
  out << "\\n"
      << op->getName() << "\\n(" << getWeight() << ")"
      << ";LAT=" << getLatency() << ";ID=" << getID() << ";"
      << "Line=" << lineNumber << ";" << get_register_info() << ";"
      << printSchedChromosomeInfo(schedChromosome, index);
  out << getRegisterString();

  auto coupled_reg = isCoupledWriteArg(rdg);
  if (coupled_reg) {
    int first_mo_0 = getMapping(map, coupled_reg->first)->getFirstOccurenceID();
    int first_mo_1 =
        getMapping(map, coupled_reg->second)->getFirstOccurenceID();
    int last_mo_0 = getMapping(map, coupled_reg->first)->getLastOccurenceID();
    int last_mo_1 = getMapping(map, coupled_reg->second)->getLastOccurenceID();

    if (first_mo_0 == getID()) {
      out << ";coupledInstr=";
      out << first_mo_1;
    } else if (first_mo_1 == getID()) {
      out << ";coupledInstr=";
      out << first_mo_0;

    } else {
      std::string error_string =
          "Coupled register is not found in the instruction " +
          std::to_string(id);
      cout << error_string << endl;
      // throw logic_error(error_string);
    }
  }

  out << "\"";

  if (previous.size() == 0) {
    out << ",color=green";
  } else if (onLongesPath)
    out << ",color=red";

  out << "];" << std::endl;

  // draw the physical register from previous iteration
  _draw_dot_physical_register_from_previous_iteration(out);
}

bool isValidArg(OPtype &type, char32_t &argument) {
  return (type == OPtype::REG) && (registers::isPhysicalRegister(argument) ||
                                   registers::isVirtualReg(argument));
}

void MO::_draw_dot_physical_register_from_previous_iteration(ostream &out) {
  // calculate inputs
  std::set<int> knownRegs;

  if (isValidArg(types[1], arg[1])) {
    knownRegs.insert(arg[1]);
    // only if src1 is a register, can the second read port also be a register
    if (isValidArg(types[2], arg[2])) {
      knownRegs.insert(arg[2]);
    }
  }

  if (argNumber == 5) {
    if (isX2Operation()) {
      knownRegs.insert(arg[5]);
      if (isValidArg(types[6], arg[6])) {
        knownRegs.insert(arg[6]);
      }
    } else {
      // MAC operation other read ports are at 0 and 4
      knownRegs.insert(arg[0]);
      knownRegs.insert(arg[4]);
    }
  }
  int n = 3;

  // remove all previous registers
  for (auto *mo : previous) {
    // remove default 1 write register from knownRegs
    if (isValidArg(mo->types[0], mo->arg[0])) {
      knownRegs.erase(mo->arg[0]);
    }

    // if this has 2 write ports, check if second exists, if yes, remove it also
    if (mo->argNumber == 5 &&
        (registers::isPhysicalRegister(mo->arg[4]) ||
         registers::isVirtualReg(mo->arg[4]) || mo->isX2Operation())) {
      if (mo->isX2Operation()) {
        knownRegs.erase(mo->arg[5]);
        if (isValidArg(mo->types[6], mo->arg[6])) {
          knownRegs.erase(mo->arg[6]);
        }
      } else {
        knownRegs.erase(mo->arg[4]);
      }
    }
  }
  // remove all previous registers
  for (auto *mo : weakPrevious) {
    // can always remove source0
    if (isValidArg(mo->types[1], mo->arg[1])) {
      knownRegs.erase(mo->arg[1]);
    }
    if (isValidArg(mo->types[2], mo->arg[2])) {
      knownRegs.erase(mo->arg[2]);
    }
    // dont have to check mac dependency, because it is always translated into a
    // real following

    // check x2 dependency
    if (mo->argNumber == 5 && mo->isX2Operation()) {
      if (isValidArg(mo->types[5], mo->arg[5])) {
        knownRegs.erase(mo->arg[5]);
      }
      if (isValidArg(mo->types[6], mo->arg[6])) {
        knownRegs.erase(mo->arg[6]);
      }
    }
  }

  // connect physical register
  for (auto node : knownRegs) {
    out << registers::getDotID(node) << " -> " << id << ";" << endl;
  }
}
bool MO::hasFollower(const string &name) {
  int l = name.length();
  for (auto it = following.begin(); it != following.end(); ++it)
    if ((*it)->getOperation()->getName().substr(0, l) == name)
      return true;
  return false;
}
