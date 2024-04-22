// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "CompilerContext.h"
#include "MO.h"
#include "SLM.h"
#include "functionalunit.h"
#include "gen_merging.h"
#include "global.h"
#include "operation.h"
#include "processor.h"
#include "register.h"
#include "virtual_reg.h"
#include <set>
#include <unordered_map>
#include <vector>

bool linkDependent(MO *first, int depend, std::set<int> *visited) {
  //    std::vector<MO*> *prev = first->getPrevious();
  std::vector<MO *> prev = (*first->getPrevious());
  prev.insert(prev.end(), first->getPreviousFlags()->begin(),
              first->getPreviousFlags()->end());
  for (std::vector<MO *>::iterator it = prev.begin(), end = prev.end();
       it != end; it++) {
    MO *next = *it;
    int id = next->getID();
    if (id == depend)
      return true;
    if (visited->find(id) == visited->end()) {
      visited->insert(id);
      if (linkDependent(next, depend, visited))
        return true;
    }
  }
  std::vector<MO *> prevWeak = (*first->getWeakPrevious());
  prevWeak.insert(prevWeak.end(), first->getPreviousFlags()->begin(),
                  first->getPreviousFlags()->end());
  for (std::vector<MO *>::iterator it = prevWeak.begin(), end = prevWeak.end();
       it != end; it++) {
    MO *next = *it;
    int id = next->getID();
    if (id == depend)
      return true;
    if (visited->find(id) == visited->end()) {
      visited->insert(id);
      if (linkDependent(next, depend, visited))
        return true;
    }
  }
  return false;
}

bool isLinkDependent(MO *first, MO *DependsOn) {
  std::set<int> visited;
  return linkDependent(DependsOn, first->getID(), &visited);
}

MO *merge(MO *first, MO *second, bool swapArgs, RegisterCoupling &couplings,
          Processor *pro) {
  char32_t *old1_args = first->getArguments();
  OPtype *old1_types = first->getTypes();
  char *old1_dir = first->getDirections();

  char32_t *old2_args = second->getArguments();
  OPtype *old2_types = second->getTypes();
  char *old2_dir = second->getDirections();

  if (swapArgs) {
    util::swap(old1_args, old2_args);
    util::swap(old1_types, old2_types);
    util::swap(old1_dir, old2_dir);
  }

  char32_t args[MAXARGNUMBER];
  OPtype types[MAXARGNUMBER];
  char dir[MAXARGNUMBER];

  for (int i = 0; i < 4; i++) {
    args[i] = old1_args[i];
    args[i + 4] = old2_args[i];
    types[i] = old1_types[i];
    types[i + 4] = old2_types[i];
    dir[i] = old1_dir[i];
    dir[i + 4] = old2_dir[i];
  }

  for (int i = 0; i < 4; i++) {
    if (old1_types[i] == REG && old1_args[i] != old2_args[i]) {
      RegisterCoupling::iterator cpl_it = couplings.find(old1_args[i]);
      if (cpl_it == couplings.end()) {
        RegisterCouple *c = new RegisterCouple();
        c->first = old1_args[i];
        c->second = old2_args[i];
        c->concurrent = true;
        couplings.insert(
            std::pair<const char32_t, RegisterCouple *>(old1_args[i], c));
        couplings.insert(
            std::pair<const char32_t, RegisterCouple *>(old2_args[i], c));
      }
    }
  }
  if (registers::isFirReg(args[1])) {
    args[1] &= ~(0b100000); // removes the bit which makes the difference
                            // between FIR_INC and FIR_OFF
    args[2] = 2;            // Set the offset to two
    args[5] = 0;            // remove the second FIR register.
    types[5] = Error;
  }
  if (registers::isFirReg(args[0])) {
    args[0] &= ~(0b100000);
    args[2] = 2;
    args[4] = 0;
    types[4] = Error;
  }

  /* No extra code for ADDEXPNDH/L merging necessary, as the merger uses the
   * name of the first operation (ADDEXPNDH in this case) by default.
   */

  std::string name(first->getOperation()->getName());
  name += "_X2";
  Operation *x2 = pro->getOperation(name.c_str());

  MO *ret = new MO(x2, args, types, dir);
  ret->setLineNumber(first->getLineNumber());
  ret->setSecondLineNumber(second->getLineNumber());
  if (!first->getIssueSlotConstraint())
    ret->setIssueSlotConstraint(second->getIssueSlotConstraint());
  else if (!second->getIssueSlotConstraint())
    ret->setIssueSlotConstraint(first->getIssueSlotConstraint());
  else
    ret->setIssueSlotConstraint(second->getIssueSlotConstraint() &
                                first->getIssueSlotConstraint());
  ret->reSetLatency(max(first->getLatency(), second->getLatency()));
  ret->setParting(first->isParting() || second->isParting());
  ret->setReorderable(first->isReorderable() && second->isReorderable());
  return ret;
}

bool gen_X2::checkRegCouplings(const RegisterCoupling &couplings, char32_t arg1,
                               char32_t arg2) {
  RegisterCoupling::const_iterator found = couplings.find(arg1);
  if (found != couplings.end()) {
    RegisterCouple *c = found->second;
    if (arg1 != c->first || arg2 != c->second) {
      return false;
    }
  }
  found = couplings.find(arg2);
  if (found != couplings.end()) {
    RegisterCouple *c = found->second;
    if (arg1 != c->first || arg2 != c->second) {
      return false;
    }
  }
  return true;
}

bool gen_X2::isMergeable(MO *m, Processor *pro) {
  if (!strncmp(m->getOperation()->getName().c_str(), "STORE", 5))
    return false;
  if (!strncmp(m->getOperation()->getName().c_str(), "LOAD", 4))
    return false;
  if (!strncmp(m->getOperation()->getName().c_str(), "PRINT", 5))
    return false;
  if (getFU((char *)"AA")->contains(m->getOperation()->getName().c_str()) &&
      strncmp(m->getOperation()->getName().c_str(), "MUL", 3))
    return false;
  if (!strncmp(m->getOperation()->getCond(), "CRS", 3))
    return false;
  if (!strncmp(m->getOperation()->getCond(), "CR", 2))
    return false;
  if (!strncmp(m->getOperation()->getCond(), "CS", 2))
    return false;
  if (!strncmp(m->getOperation()->getCond(), "SMV", 3))
    return false;
  char32_t *args = m->getArguments();
  OPtype *types = m->getTypes();
  for (int i = 0, end = m->getArgNumber(); i < end; i++) {
    if (types[i] != REG)
      continue;
    if (registers::isFirReg(args[i]))
      return false; // continue; # HINT: X2 merging for memory access has to be
                    // done manually!
                    //		if(registers::getVirtualRegisterNumber(args[i])<0)
                    //			return false;
  }
  std::string name(m->getOperation()->getName());
  name += "_X2";
  Operation *x2 = pro->getOperation(name.c_str());
  if (x2 == NULL)
    return false;
  return true;
}

bool gen_X2::areMergeable(MO *first, MO *second, int slmId,
                          const RegisterCoupling &couplings, Processor *pro) {
  LOG_OUTPUT(LOG_M_MERGE_DEBUG, "Checking mergeability of:\n");
  if (isLog(LOG_M_MERGE_DEBUG)) {
    first->writeOutReadable(cout);
    cout << endl;
    second->writeOutReadable(cout);
    cout << endl;
  }
  if (first->getArgNumber() > 4 || second->getArgNumber() > 4)
    return false;
  if (MAXARGNUMBER < 8)
    return false;
  char32_t *old1_args = first->getArguments();
  OPtype *old1_types = first->getTypes();
  char *old1_dir = first->getDirections();
  char32_t *old2_args = second->getArguments();
  OPtype *old2_types = second->getTypes();
  char *old2_dir = second->getDirections();

  bool checkADDEXPND = (first->getOperation()->getBaseName() == "ADDEXPNDH" &&
                        second->getOperation()->getBaseName() == "ADDEXPNDL") ||
                       (first->getOperation()->getBaseName() == "ADDEXPNDHI" &&
                        second->getOperation()->getBaseName() == "ADDEXPNDLI");

  if (util::strStartsWith(first->getOperation()->getBaseName(), "RFU")) {
    if (util::strStartsWith(first->getOperation()->getBaseName(), "RFU7") &&
        util::strStartsWith(second->getOperation()->getBaseName(), "RFU8")) {
      if (first->getLatency() != second->getLatency()) {
        LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                   "     > RFU7 + RFU8: Have to have same latency\n");
        return false;
      } else if (first->getOperation()->isSelfdependent() !=
                 second->getOperation()->isSelfdependent()) {
        LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                   "     > RFU7 + RFU8: Self-dependent has to be equal\n");
        return false;
      }
    } else {
      LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                 "     > Only RFU7 + RFU8 merging possible\n");
      return false;
    }
  }

  // This check was disabled, as areMergeable is always preceded by calls to
  // isMergeable for both MOs, which already checks this:
  //    std::string name(first->getOperation()->getName());
  //    name += "_X2";
  //    Operation* x2 = pro->getOperation(name.c_str());
  //    if (x2 == NULL)
  //        return false;

  if ((!strncmp(first->getOperation()->getName().c_str(), "MV", 2)) &&
      ((((0b1111000 & old1_args[1]) == 0b1110000) &&
        registers::isFirReg(old1_args[1])) ||
       (((0b1111000 & old1_args[0]) == 0b1110000) &&
        registers::isFirReg(old1_args[0])))) {
    // check if second is in next of first.
    std::vector<MO *> *follower = first->getFollowing();
    bool right = false;
    for (std::vector<MO *>::iterator it = follower->begin(),
                                     end = follower->end();
         it != end; it++) {
      if (*it == second) {
        right = true;
        break;
      }
    }
    if (!right)
      return false;
  } else if (isLinkDependent(first, second)) {
    LOG_OUTPUT(LOG_M_MERGE_DEBUG, "     > MOs are dependent\n");
    return false;
  }

  ass_reg_t *blocked = blockedRegsInSLM[slmId];
  for (int i = 0; i < 4; i++) {
    if (old1_types[i] != old2_types[i]) {
      LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                 "    > Types for argument %d do not match\n", i);
      return false;
    }
    if (old1_dir[i] != old2_dir[i]) {
      LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                 "    > Directions for argument %d do not match\n", i);
      return false;
    }
    if (old1_types[i] == Immediate && old1_args[i] != old2_args[i]) {
      LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                 "    > Unequal immediate values for argument %d\n", i);
      return false;
    }
    if (old1_types[i] == Imm32 && old1_args[i] != old2_args[i]) {
      LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                 "    > Unequal long immediate values for argument %d\n", i);
      return false;
    }
    if (old1_types[i] == REG) {
      if (old1_args[i] == old2_args[i])
        continue;
      if (i > 0 && checkADDEXPND && old1_args[i] != old2_args[i]) {
        LOG_OUTPUT(LOG_M_MERGE_DEBUG, "    > Source registers for ADDEXPNDH/L "
                                      "merging have to be equal\n");
        return false;
      }
      if (registers::isVirtualReg(old1_args[i]) &&
          registers::isVirtualReg(old2_args[i])) {
        if (!checkRegCouplings(couplings, old1_args[i], old2_args[i])) {
          LOG_OUTPUT(
              LOG_M_MERGE_DEBUG,
              "     > Register _couplings for argument %d do not match\n", i);
          return false;
        }
      } else if (registers::isVirtualReg(old1_args[i]) &&
                 registers::isPhysicalRegister(old2_args[i])) {
        if (!params.mergeFixAndVirtual) {
          LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                     "     > Merging of virtual+fix is not enabled\n");
          return false;
        }
        int num, file;
        if (!gen_X2::getMatchingX2Reg(old2_args[i], REG_PAIR_POS::SECOND,
                                      pro->isX2Supported(), num, file)) {
          LOG_OUTPUT(LOG_M_MERGE_DEBUG, "     > Cannot merge arguments %d\n",
                     i);
          return false;
        }
        if (blocked[file] & ((ass_reg_t)1) << num) {
          LOG_OUTPUT(
              LOG_M_MERGE_DEBUG,
              "     > Register _couplings for argument %d already blocked\n",
              i);
          return false;
        }
        RegisterCoupling::const_iterator cpl_it = couplings.find(old1_args[i]);
        if (cpl_it != couplings.end() &&
            cpl_it->second->second != old2_args[i]) {
          LOG_OUTPUT(
              LOG_M_MERGE_DEBUG,
              "     > Register _couplings for argument %d not possbile\n", i);
          return false;
        }
      } else if (registers::isPhysicalRegister(old1_args[i]) &&
                 registers::isVirtualReg(old2_args[i])) {
        if (!params.mergeFixAndVirtual) {
          LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                     "     > Merging of fix+virtual is not enabled\n");
          return false;
        }
        int num, file;
        if (!gen_X2::getMatchingX2Reg(old1_args[i], REG_PAIR_POS::FIRST,
                                      pro->isX2Supported(), num, file)) {
          LOG_OUTPUT(LOG_M_MERGE_DEBUG, "     > Cannot merge arguments %d\n",
                     i);
          return false;
        }
        if (blocked[file] & ((ass_reg_t)1) << num) {
          LOG_OUTPUT(
              LOG_M_MERGE_DEBUG,
              "     > Register _couplings for argument %d already blocked\n",
              i);
          return false;
        }
        RegisterCoupling::const_iterator cpl_it = couplings.find(old2_args[i]);
        if (cpl_it != couplings.end() &&
            cpl_it->second->first != old1_args[i]) {
          LOG_OUTPUT(
              LOG_M_MERGE_DEBUG,
              "     > Register _couplings for argument %d not possbile\n", i);
          return false;
        }
      } else if (registers::isPhysicalRegister(old1_args[i]) &&
                 registers::isPhysicalRegister(old2_args[i])) {
        if (((old1_args[i] % 2 == 0) && old1_args[i] + 1 == old2_args[i]) ||
            ((old1_args[i] == old2_args[i]) & (i != 0))) {
          if (old1_dir[i] == WRITE) {
            int regfile = registers::getRegFile(old1_args[i]);
            if (registers::getWritePorts(regfile) < 2)
              return false;
          }
        } else {
          LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                     "     > Fix registers in argument %d do not match\n", i);
          return false;
        }
      }
    }
  }
  return true;
}

MO *generateMV(char32_t argument, char32_t maxVirtReg,
               RegisterCoupling &couplings, Processor *pro) {
  MO *mv = new MO(pro->getMVOperation());
  mv->getArguments()[0] = maxVirtReg;
  mv->getArguments()[1] = argument;
  mv->getTypes()[0] = REG;
  mv->getTypes()[1] = REG;
  mv->getDirections()[0] = WRITE;
  mv->getDirections()[1] = READ;
  mv->setArgNumber(2);
  RegisterCoupling::iterator result = couplings.find(argument);
  if (result != couplings.end()) {
    RegisterCouple *c = result->second;
    couplings.erase(c->first);
    couplings.erase(c->second);
    delete c;
  }
  return mv;
}

bool generateMVifNessesary(MO *first, MO *second, SLM *target, Processor *pro,
                           RegisterCoupling &couplings, bool permitted) {
  char32_t *old1_args = first->getArguments();
  OPtype *types = first->getTypes();

  char32_t *old2_args = second->getArguments();

  for (int i = 0; i < 4; i++) {
    if (types[i] == REG && old1_args[i] != old2_args[i]) {
      RegisterCoupling::iterator found = couplings.find(old1_args[i]);
      if (found != couplings.end()) {
        RegisterCouple *c = found->second;
        if (!permitted &&
            (old1_args[i] != c->first || old2_args[i] != c->second))
          return false;
        if (old1_args[i] != c->first) {
        }
      }
      found = couplings.find(old2_args[i]);
      if (found != couplings.end()) {
        RegisterCouple *c = found->second;
        if (old1_args[i] != c->first || old2_args[i] != c->second)
          return false;
      }
    }
  }
  return false;
}

template <class T> bool setContains(const std::set<T> &set, const T &element) {
  return set.find(element) != set.end();
}

/**
 * Add the given MO to the given SLM and update alreadyPlaced.
 * The function checks, if the MO is already present in alreadyPlaced!
 * In that case, the MO is deleted, as it is considered to be a copy!
 */
void placeMOInSLM(SLM *slm, MO *mo, std::set<int> &alreadyPlaced,
                  const Context &ctx) {
  if (alreadyPlaced.insert(mo->getID()).second) {
    if (mo->getSecondLineNumber() != (uint)-1)
      LOG_OUTPUT(LOG_M_MERGE_DEBUG, "%s Placing MO %d+%d (%d) in merged SLM\n",
                 ctx.asString().c_str(), mo->getLineNumber(),
                 mo->getSecondLineNumber(), mo->getID());
    else
      LOG_OUTPUT(LOG_M_MERGE_DEBUG, "%s Placing MO %d (%d) in merged SLM\n",
                 ctx.asString().c_str(), mo->getLineNumber(), mo->getID());
    slm->addMO(mo);
  } else {
    if (mo->getSecondLineNumber() != (uint)-1)
      LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                 "%s MO %d+%d (%d) already placed in merged SLM\n",
                 ctx.asString().c_str(), mo->getLineNumber(),
                 mo->getSecondLineNumber(), mo->getID());
    else
      LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                 "%s MO %d (%d) already placed in merged SLM\n",
                 ctx.asString().c_str(), mo->getLineNumber(), mo->getID());
    delete mo;
  }
}

/**
 * Find the index of the MO with the given id in the list of operations.
 */
int findMO(int moID, const std::vector<MO *> *ops) {
  for (size_t i = 0; i < ops->size(); ++i)
    if (ops->at(i)->getID() == moID)
      return i;
  return -1;
}

// Forward declaration
bool mergeMO(int MOIndex, const std::vector<MO *> *ops,
             gen_X2::X2_chromosome &chromosome, RegisterCoupling &couplings,
             int &noMerged, SLM *better, Processor *pro,
             std::set<int> &alreadyPlaced, std::set<int> &pending,
             const Context &ctx);

/**
 * Add all ancestors (from the DDG) for a given MO to a given SLM.
 *
 * \param slm The SLM, to which the ancestors should be added.
 * \param mo The MO, whose ancestors should be added.
 * \param alreadyPlaced A set of ids for those MOs that have already been added
 * to the SLM.
 */
void addAncestors(SLM *slm, MO *mo, std::set<int> &alreadyPlaced,
                  std::set<int> &pending, gen_X2::X2_chromosome &chromosome,
                  const std::vector<MO *> *ops, RegisterCoupling &couplings,
                  int &noMerged, Processor *pro, const Context &ctx) {
  std::vector<MO *> parents = (*mo->getPrevious());
  parents.insert(parents.end(), mo->getWeakPrevious()->begin(),
                 mo->getWeakPrevious()->end());
  parents.insert(parents.end(), mo->getPreviousFlags()->begin(),
                 mo->getPreviousFlags()->end());
  parents.insert(parents.end(), mo->getWeakPreviousFlags()->begin(),
                 mo->getWeakPreviousFlags()->end());
  for (std::vector<MO *>::iterator it = parents.begin(), end = parents.end();
       it != end; it++) {
    MO *parent = *it;
    int parentId = parent->getID();
    if (!setContains(alreadyPlaced, parentId) &&
        !setContains(pending, parentId)) {
      addAncestors(slm, parent, alreadyPlaced, pending, chromosome, ops,
                   couplings, noMerged, pro, ctx);
      if (!setContains(alreadyPlaced, parentId)) {
        if (params.recursiveMerge &&
            mergeMO(findMO(parentId, ops), ops, chromosome, couplings, noMerged,
                    slm, pro, alreadyPlaced, pending,
                    Context::nextStage(ctx, "merge")))
          continue;
        placeMOInSLM(slm, parent->copy(), alreadyPlaced, ctx);
      }
    }
  }
}

/**
 * Checks, if the given MO depends on a MO from a set of MOs.
 */
bool dependsOnPending(MO *mo, const std::set<int> &pending) {
  if (!mo)
    return false;
  //    const std::vector<MO*> *parents = mo->getPrevious();
  std::vector<MO *> parents = (*mo->getPrevious());
  parents.insert(parents.end(), mo->getPreviousFlags()->begin(),
                 mo->getPreviousFlags()->end());
  for (std::vector<MO *>::const_iterator it = parents.begin();
       it != parents.end(); ++it) {
    if (setContains(pending, (*it)->getID()))
      return true;
    if (dependsOnPending((*it), pending))
      return true;
  }
  return false;
}

bool mergeMO(int MOIndex, const std::vector<MO *> *ops,
             gen_X2::X2_chromosome &chromosome, RegisterCoupling &couplings,
             int &noMerged, SLM *better, Processor *pro,
             std::set<int> &alreadyPlaced, std::set<int> &pending,
             const Context &ctx) {
  if (MOIndex == -1)
    return false;
  MO *mo1 = ops->at(MOIndex);
  bool swapArgs = false;
  const gen_X2::MergeCandidates::CandidateList_t &candidates =
      chromosome.gene(MOIndex);

  LOG_OUTPUT(LOG_M_MERGE_DEBUG, "\n%s Start merging MO %d (%d)\n",
             ctx.asString().c_str(), mo1->getLineNumber(), mo1->getID());
  LOG_OUTPUT(LOG_M_MERGE_DEBUG, "%s  candidates: ", ctx.asString().c_str());
  gen_X2::printMergeCandidates(LOG_M_MERGE_DEBUG, candidates, ops);
  LOG_OUTPUT(LOG_M_MERGE_DEBUG, "\n");

  Operation *op1 = mo1->getOperation();
  if (candidates.size() == 0 || candidates[0] == -1) {
    LOG_OUTPUT(LOG_M_MERGE_DEBUG,
               "%s  [X] MO %d (%d) shall not be merged (chromosome)!\n",
               ctx.asString().c_str(), mo1->getLineNumber(), mo1->getID());
    return false;
  }
  if (setContains(alreadyPlaced, mo1->getID())) {
    LOG_OUTPUT(LOG_M_MERGE_DEBUG, "%s  [X] MO %d (%d) already merged\n",
               ctx.asString().c_str(), mo1->getLineNumber(), mo1->getID());
    return false;
  }
  if (mo1->isX2Operation() || !gen_X2::isMergeable(mo1, pro)) {
    LOG_OUTPUT(LOG_M_MERGE_DEBUG, "%s  [X] MO %d (%d) is X2 or not mergeable\n",
               ctx.asString().c_str(), mo1->getLineNumber(), mo1->getID());
  } else {
    // Scan through the candidate list for a matching partner
    for (size_t j = 0; j < candidates.size(); ++j) {
      int cand_idx = candidates[j];
      if (cand_idx == -1) {
        LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                   "%s  [X] MO %d (%d) shall not be merged (chromosome)!\n",
                   ctx.asString().c_str(), mo1->getLineNumber(), mo1->getID());
        break;
      }
      MO *mo2 = ops->at(cand_idx);
      LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                 "%s  [~] Trying MO %d (%d) as merging partner\n",
                 ctx.asString().c_str(), mo2->getLineNumber(), mo2->getID());

      if (mo1->getID() == mo2->getID()) {
        LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                   "%s  [X] Refused merging a MO to itself\n",
                   ctx.asString().c_str());
        continue;
      }

      if (mo1->getLineNumber() > mo2->getLineNumber()) {
        LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                   "%s  [~] [X] MO %d (%d) and %d (%d) are in wrong order\n",
                   ctx.asString().c_str(), mo1->getLineNumber(), mo1->getID(),
                   mo2->getLineNumber(), mo2->getID());
        continue;
      }
      if (mo2->isX2Operation()) {
        LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                   "%s  [~] [X] MO %d (%d) is already X2 operation\n",
                   ctx.asString().c_str(), mo2->getLineNumber(), mo2->getID());
        continue;
      }
      if (setContains(alreadyPlaced, mo2->getID())) {
        LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                   "%s  [~] [X] MO %d (%d) is already merged\n",
                   ctx.asString().c_str(), mo2->getLineNumber(), mo2->getID());
        continue;
      }
      if (dependsOnPending(mo2, pending)) {
        LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                   "%s  [X] MO %d (%d) depends on pending MOs\n",
                   ctx.asString().c_str(), mo2->getLineNumber(), mo2->getID());
        continue;
      }
      Operation *op2 = mo2->getOperation();
      if (op1 != op2 && !util::isOpPair(op1, op2, "ADDEXPNDH", "ADDEXPNDL") &&
          !util::isOpPair(op1, op2, "ADDEXPNDHI", "ADDEXPNDLI") &&
          !util::isOpPair(op1, op2, "RFU7", "RFU8", false)) {
        LOG_OUTPUT(
            LOG_M_MERGE_DEBUG,
            "%s  [~] [X] MO %d (%d) and %d (%d) are not the same operation\n",
            ctx.asString().c_str(), mo1->getLineNumber(), mo1->getID(),
            mo2->getLineNumber(), mo2->getID());
        continue;
      }
      if (!(util::strStartsWith(op1->getBaseName(), "RFU7") &&
            util::strStartsWith(op2->getBaseName(), "RFU8")) &&
          !gen_X2::isMergeable(mo2, pro)) {
        LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                   "%s  [~] [X] MO %d (%d) cannot be merged\n",
                   ctx.asString().c_str(), mo2->getLineNumber(), mo2->getID());
        continue;
      }
      if (!gen_X2::areMergeable(mo1, mo2, better->getOriginalID(), couplings,
                                pro)) {
        LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                   "%s  [~] [X] MO %d (%d) and %d (%d) cannot be merged\n",
                   ctx.asString().c_str(), mo1->getLineNumber(), mo1->getID(),
                   mo2->getLineNumber(), mo2->getID());
        continue;
      }

      // Merge operations
      LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                 "%s  [~] [~] Merging MOs %d (%d) and %d (%d)\n",
                 ctx.asString().c_str(), mo1->getLineNumber(), mo1->getID(),
                 mo2->getLineNumber(), mo2->getID());
      MO *m = merge(mo1, mo2, swapArgs, couplings, pro);
      if (m != NULL) {
        MI *mi = new MI();
        mi->putOperation(m, 0);
        if (!pro->isExecuteable(mi)) {
          LOG_OUTPUT(LOG_M_MERGE_DEBUG,
                     "%s  [~] [X] Merged MO is not executable\n",
                     ctx.asString().c_str());
          delete mi;
          delete m;
          continue;
        }
        delete mi;
        pending.insert(mo1->getID());
        pending.insert(mo2->getID());
        addAncestors(better, mo2, alreadyPlaced, pending, chromosome, ops,
                     couplings, noMerged, pro, ctx);
        pending.erase(mo1->getID());
        pending.erase(mo2->getID());
        alreadyPlaced.insert(mo1->getID());
        alreadyPlaced.insert(mo2->getID());
        better->addMO(m);
#if DISPLAY_MERGES
        chromosome.getMerges().push_back(
            std::make_pair(mo1->getLineNumber(), mo2->getLineNumber()));
#endif
        LOG_OUTPUT(LOG_M_MERGE_DEBUG, "%s Placing %d+%d in merged SLM\n",
                   ctx.asString().c_str(), mo1->getLineNumber(),
                   mo2->getLineNumber());
        ++noMerged;
        return true;
      }
    }
  }
  return false;
}

SLM *gen_X2::X2Automerge(SLM *slm, Processor *pro,
                         gen_X2::X2_chromosome *chromosome, int &noMerged,
                         const Context &ctx) {
  LOG_OUTPUT(
      LOG_M_MERGE_DEBUG,
      "%s Starting X2 automerge for SLM %d, generation %d, individual %d\n",
      ctx.asString().c_str(), slm->getOriginalID(), ctx.mergeRound(),
      ctx.mergeIndiv());
  if (isLog(LOG_M_MERGE_DEBUG)) {
    LOG_OUTPUT(LOG_M_MERGE_DEBUG, "X2 Chromosome:\n");
    X2_chromosome::print(chromosome, slm->getOperations());
  }
  slm->calculateGraph(pro, false);
  std::vector<MO *> *ops = slm->getOperations();

  RegisterCoupling couplings;

  std::set<int> alreadyPlaced;
  std::set<int> pending; // during merging, when operations are considered for
                         // recursive merge, but not yet placed.
  SLM *better = new SLM();
  better->setPartProb(slm->getPartProb());
  better->setAloneProb(slm->getAloneProb());
  if (slm->haveRAPruneCount())
    better->setRAPruneCount(slm->getRAPruneCount());
  better->setOptimizationLevel(slm->getOptimizationLevel());
  better->setOriginalID(slm->getOriginalID());

  for (std::vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it) {
    MO *op = *it;
    if (op->isX2Operation())
      storeCouplings(couplings, op);

    if (getFU((char *)"AA")->contains(op->getOperation()->getName().c_str()) &&
        strncmp(op->getOperation()->getName().c_str(), "MUL", 3)) {
      char32_t *old1_args = op->getArguments();
      storeCoupling(couplings, old1_args[0], old1_args[4]);
    }
  }

  /************************
   * Start merging
   */
  for (unsigned int i = 0; i < ops->size(); ++i)
    if (!mergeMO(i, ops, *chromosome, couplings, noMerged, better, pro,
                 alreadyPlaced, pending, ctx))
      placeMOInSLM(better, ops->at(i)->copy(), alreadyPlaced, ctx);

  deleteCouplings(couplings);
  int diff = ops->size() - better->getOperations()->size();
  if (diff <= 0) {
    delete better;
    return NULL;
  }
  if (slm->getBranchOperation() != NULL)
    better->addMO(slm->getBranchOperation()->copy());
  better->releaseGraph();
  better->calculateGraph(pro, false);
  return better;
}
