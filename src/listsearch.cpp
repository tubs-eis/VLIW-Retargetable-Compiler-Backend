// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#include "listsearch.h"
#include "MI.h"
#include "ML_schedule.h"
#include "MO.h"
#include "SLM.h"
#include "gen_sched.h"
#include "global.h"
#include "processor.h"
#include "register.h"
#include <algorithm>

void ListSearch::insertInOrder(entry &e, std::list<entry> *list) {
  // as a general rule it is, that branch operations are handeled by a greater
  // getInstance at the end they must not be scheduled twice.
  if (e.mo->isBranchOperation())
    return;

  if (e.mo->isReorderable()) {
    for (std::list<entry>::iterator it = list->begin(); it != list->end();
         it++) {
      if ((*it).value < e.value) {
        list->insert(it, e);
        return;
      }
    }
  }
  // if it is not bigger, than any element of the list, then add it at the end.
  // also happens if the list is empty
  list->push_back(e);
}

void ListSearch::makeAvailable(MO *mo) {
  // as a general rule it is, that branch operations are handeled by a greater
  // getInstance at the end they must not be scheduled twice.
  if (mo->isBranchOperation())
    return;
  for (std::vector<std::list<entry> *>::iterator ot = schedulable.begin(),
                                                 end = schedulable.end();
       ot != end; ot++) {
    std::list<entry> *list = *ot;
    for (std::list<entry>::iterator it = list->begin(); it != list->end();
         it++) {
      if ((*it).mo == mo) {
        (*it).schedulable = true;
        return;
      }
    }
  }
}

ListSearch::~ListSearch() {
  for (auto ot = schedulable.begin(), end = schedulable.end(); ot != end; ot++)
    delete (*ot);
}

ListSearch::ListSearch(SLM *slm, const Processor *processor,
                       gen_sched::sched_chromosome *individual)
    : _scheduableCROperationsAfterCS(processor->getIssueSlotNumber(),
                                     vector<int>(0)),
      _scheduableCRCSOperations(processor->getIssueSlotNumber(),
                                vector<int>(0)),
      _scheduableCROperationsAfterCS_STORED(processor->getIssueSlotNumber(),
                                            vector<int>(0)),
      _scheduableCRCSOperations_STORED(processor->getIssueSlotNumber(),
                                       vector<int>(0)) {
  std::vector<MO *> *mos = slm->getOperations();
  _moCount = mos->size();
  _schedulable_STORED = 0;
  std::list<entry> *list = new std::list<entry>();
  if (!individual) {
    if (params.fileMLModel != "") {
      // std::vector<int> mlWeights = getMLWeights(mos, slm->getID(),
      // params.fileMLModel);
      std::map<int, int> weightMap =
          getMLWeights(mos, slm->getID(), params.fileMLModel);

      // int counter = 0;
      // for (std::vector<MO *>::iterator it = mos->begin(); it != mos->end();
      // it++) { std::cout << mlWeights[counter] << " " << (*it)->getWeight() <<
      // "\t" << std::endl; counter++;
      // }

      int mo_counter = 0;

      for (std::vector<MO *>::iterator it = mos->begin(); it != mos->end();
           it++) {
        // for (std::map<int, int>::iterator it = weightMap; it !=
        // weightMap.end(); it++) {
        entry e;
        e.mo = *it;
        // e.value = mlWeights[mo_counter];
        if (weightMap.find(e.mo->getID()) == weightMap.end()) {
          std::cout << "MO ID " << e.mo->getID() << " Nicht gefunden\n";
        }
        std::cout << e.mo->getID() << " " << weightMap[e.mo->getID()]
                  << std::endl;
        e.value = weightMap[e.mo->getID()];

        _moWeights[e.mo->getID()] = e.value;
        if (!e.mo->hasPrev() && !e.mo->hasWeakPrev() && !e.mo->hasPrevFlags() &&
            !e.mo->hasWeakPrevFlags()) {
          e.schedulable = true;
        } else {
          e.schedulable = false;
        }
        insertInOrder(e, list);
        if (e.mo->isParting()) {
          schedulable.push_back(list);
          list = new std::list<entry>();
        }

        // mo_counter++;
      }
    } else {
      for (std::vector<MO *>::iterator it = mos->begin(); it != mos->end();
           it++) {
        entry e;
        e.mo = *it;
        e.value = e.mo->getWeight();
        _moWeights[e.mo->getID()] = e.value;
        if (!e.mo->hasPrev() && !e.mo->hasWeakPrev() && !e.mo->hasPrevFlags() &&
            !e.mo->hasWeakPrevFlags()) {
          e.schedulable = true;
        } else {
          e.schedulable = false;
        }
        insertInOrder(e, list);
        if (e.mo->isParting()) {
          schedulable.push_back(list);
          list = new std::list<entry>();
        }
      }
    }
  } else {
    int *weights = individual->weights;
    bool *partings = individual->partings;
    bool *alone = individual->alone;

    for (unsigned int i = 0; i < mos->size(); ++i) {
      entry e;
      e.mo = mos->at(i);
      e.value = weights[i];
      _moWeights[e.mo->getID()] = e.value;
      if (alone[i])
        singles.insert(e.mo->getID());
      if (!e.mo->hasPrev() && !e.mo->hasWeakPrev() && !e.mo->hasPrevFlags() &&
          !e.mo->hasWeakPrevFlags()) {
        e.schedulable = true;
      } else {
        e.schedulable = false;
      }
      insertInOrder(e, list);
      if (e.mo->isParting() || partings[i]) {
        schedulable.push_back(list);
        list = new std::list<entry>();
      }
    }
  }
  if (list->size() != 0)
    schedulable.push_back(list);
  else
    delete list;
}

void ListSearch::checkFollowers(MO *mo, std::vector<MO *> *followers) {
  for (std::vector<MO *>::iterator it = followers->begin();
       it != followers->end(); ++it) {
    MO *m = *it;
    if (allPredecessorsRunning(m, mo))
      makeAvailable(m);
  }
}

bool ListSearch::allMOsRunning(std::vector<MO *> *mos, MO *currentMO) {
  for (auto it = mos->begin(); it != mos->end(); ++it) {
    if (currentMO && currentMO == *it)
      continue;
    if (_alreadyScheduled.find((*it)->getID()) == _alreadyScheduled.end())
      return false;
  }
  return true;
}

bool ListSearch::allPredecessorsRunning(MO *m, MO *currentMO) {
  if (!allMOsRunning(m->getPrevious()))
    return false;
  if (!allMOsRunning(m->getWeakPrevious(), currentMO))
    return false;
  if (!allMOsRunning(m->getPreviousFlags()))
    return false;
  if (!allMOsRunning(m->getWeakPreviousFlags(), currentMO))
    return false;
  return true;
}

bool ListSearch::schedCandidatesLeft() {
  for (auto it = schedulable.begin(); it != schedulable.end(); ++it) {
    auto list = *it;
    if (list && list->size() > 0)
      return true;
  }
  return false;
}

bool ListSearch::isLastPartingSection() const {
  return schedulable.size() == 1;
}

int ListSearch::numSchedCandidates() const {
  return schedulable.front()->size();
}

void ListSearch::fillNextMI(MI *mi, const Processor *processor) {
  MO *candidate = findCandidateMOforMI(processor, mi);
  while (candidate) {
    checkFollowers(candidate, candidate->getWeakFollowers());
    checkFollowers(candidate, candidate->getWeakFollowingFlags());
    int latency = candidate->isReorderable() ? candidate->getLatency() : 1;
    _runningMOs.push_back(std::make_pair(candidate, latency));
    candidate = findCandidateMOforMI(processor, mi);
  }
}

void ListSearch::makeFlagDependentMOsAvailable(MO *mo) {
  _alreadyScheduled.insert(mo->getID());
  auto follower = mo->getFollowingFlags();
  for (auto jt = follower->begin(); jt != follower->end(); ++jt) {
    MO *f = *jt;
    if (allPredecessorsPlaced(f))
      makeAvailable(f);
  }
  _alreadyScheduled.erase(mo->getID());
}

void ListSearch::makeDependentMOsAvailable(MO *mo,
                                           std::function<bool(MO *)> pred) {
  // Temporarily put the MO in alreadyScheduled
  _alreadyScheduled.insert(mo->getID());
  auto follower = mo->getFollowing();
  for (auto jt = follower->begin(); jt != follower->end(); ++jt) {
    MO *f = *jt;
    if (pred(f)) {
      if (allPredecessorsPlaced(f))
        makeAvailable(f);
    }
  }

  follower = mo->getFollowingFlags();
  for (auto jt = follower->begin(); jt != follower->end(); ++jt) {
    MO *f = *jt;
    if (pred(f)) {
      if (allPredecessorsPlaced(f))
        makeAvailable(f);
    }
  }
  _alreadyScheduled.erase(mo->getID());
}

bool ListSearch::nextCycle(const Processor *processor) {
  for (auto it = _runningMOs.begin(); it != _runningMOs.end();) {
    MO *mo = it->first;
    --(it->second);
    if (it->second == getForwarding()) {
      // Operations that only depend over FLAGS (CS->CR) become available
      makeFlagDependentMOsAvailable(mo);
    }
    if (it->second == 0) { // latency passed
      if (!resultsReady(mo, processor))
        return false;
      _alreadyScheduled.insert(mo->getID());
      it = _runningMOs.erase(it);
    } else if (mo->getOperation()->getName().substr(0, 11) == "STORERCUPPL") {
      makeDependentMOsAvailable(mo, [](MO *f) {
        return f->getOperation()->getName().substr(0, 7) != "LOADRCU";
      });
      ++it;
    } else if (/*mo->getOperation()->isSelfdependent() && */ mo->getOperation()
                   ->isPipelined()) {
      // Latency of this operations should not be used in self-dependent ops
      string opname = mo->getOperation()->getBaseName();
      makeDependentMOsAvailable(mo, [opname](MO *f) {
        return f->getOperation()->getBaseName() == opname;
      });
      ++it;
    } else
      ++it;
  }
  return true;
}

Program *ListSearch::scheduleMOs(const Processor *processor,
                                 int &notSchedulableCSCR) {
  Program *instructions = new Program();
  MI *currentMI = new MI();

  while (_alreadyScheduled.size() < _moCount) {
    fillNextMI(currentMI, processor);

    if (!_scheduleNow.empty()) {
      // We have operations that should be placed in the current MI, but did not
      // fit!
      delete instructions;
      instructions = nullptr;
      break; // exit while loop
    }

    // when we go to the next MI, we can check which MO are becoming free now.
    if (!nextCycle(processor)) {
      delete instructions;
      instructions = nullptr;
      break; // exit while loop
    }
    instructions->push_back(currentMI);
    currentMI = new MI();

    /* If no further MOs have to be placed, we break here, so we don't introduce
     * further NOPs to fill the latencies of all placed MOs. Those NOPs will be
     * added later, if necessary.
     */
    if (!schedCandidatesLeft())
      break; // exit while loop

    // todo: why am I different than the other abort conditions?
    if (registers::getNumberOfCondselRegister() > 1) {
      if (instructions->size() > _moCount * 10) {
        delete instructions;
        instructions = nullptr;
        notSchedulableCSCR = 0;
        for (int i = 0; i < processor->getIssueSlotNumber(); i++) {
          notSchedulableCSCR += _scheduableCROperationsAfterCS.at(i).size();
          notSchedulableCSCR += _scheduableCRCSOperations.at(i).size();
        }
        delete currentMI;
        //                LOG_OUTPUT(LOG_M_SCHED_DETAIL, "%s Scheduling routine
        //                ended unsuccessfully due to CS/CR for SLM %d\n",
        //                ctx.asString().c_str(), slm->getOriginalID());
        return 0;
      }
    }
  }
  delete currentMI;
  return instructions;
}

void ListSearch::fillLatencyCycles(Program *instructions) {
  int latency = 0;
  for (auto &running : _runningMOs) {
    MO *mo = running.first;
    int l = mo->getFinalLatency() - (mo->getLatency() - running.second);
    if (l > latency)
      latency = l;
  }
  for (int l = 0; l < latency; ++l) {
    MI *mi = new MI();
    instructions->push_back(mi);
  }
}

void ListSearch::removeFromSchedulable(MO *mo) {
  for (auto it = schedulable.begin(); it != schedulable.end(); ++it)
    for (auto ot = (*it)->begin(); ot != (*it)->end(); ++ot)
      if (ot->mo == mo) {
        (*it)->erase(ot);
        break;
      }
}

bool ListSearch::collidesWithRunning(MO *mo) {
  if (mo->getOperation()->getName().substr(0, 11) != "STORERCUPPL")
    return false;
  if (!mo->hasFollower("LOADRCU"))
    return false;

  for (auto it = _runningMOs.begin(); it != _runningMOs.end(); ++it) {
    MO *m = it->first;
    int lat = it->second;
    if (m->getOperation()->getName().substr(0, 11) == "STORERCUPPL" &&
        lat >= mo->getLatency())
      return true;
  }
  return false;
}

MO *ListSearch::findCandidateMOforMI(const Processor *pro, MI *mi) {
  if (schedulable.size() == 0)
    return 0;

  std::list<entry> *list = schedulable.front();
  if (list && list->size() == 0) {
    schedulable.erase(schedulable.begin());
    delete list;
    return 0;
  }

  MO **mos = mi->getOperations();
  if (mos[0] && singles.find(mos[0]->getID()) != singles.end())
    return 0;

  if (!_scheduleNow.empty()) {
    auto it = _scheduleNow.begin();
    while (it != _scheduleNow.end()) {
      MO *mo = *it;
      if (allPredecessorsPlaced(mo) && scheduleIfPossible(mo, mi, pro)) {
        _scheduleNow.erase(it);
        removeFromSchedulable(mo);
        return mo;
      } else
        return 0;
    }
  }

  for (std::list<entry>::iterator it = list->begin(); it != list->end(); it++) {
    MO *mo = (*it).mo;
    if ((!mo->isReorderable() || (*it).schedulable) &&
        !collidesWithRunning(mo) && scheduleIfPossible(mo, mi, pro)) {
      list->erase(it);
      return mo;
    }
    if (!mo->isReorderable())
      return 0;
  }
  return 0;
}

bool ListSearch::resultsReady(MO *mo, const Processor *processor) {
  std::vector<MO *> follower = *mo->getFollowing();
  follower.insert(follower.end(), mo->getFollowingFlags()->begin(),
                  mo->getFollowingFlags()->end());
  if (mo->getOperation()->getName().substr(0, 11) == "STORERCUPPL") {
    bool schedulable = true;
    for (auto it = follower.begin(); it != follower.end(); ++it) {
      MO *f = *it;
      if (f->getOperation()->getName().substr(0, 7) == "LOADRCU") {
        // This LOADRCU operation has to be placed in the next MI!
        if (std::find(_scheduleNow.begin(), _scheduleNow.end(), f) ==
            _scheduleNow.end()) {
          if (!checkSchedulable(f, processor))
            return false;
          _scheduleNow.push_back(f);
        }
      } else {
        auto prev = *f->getPrevious();
        prev.insert(prev.end(), f->getPreviousFlags()->begin(),
                    f->getPreviousFlags()->end());
        for (auto ot = prev.begin(); ot != prev.end(); ++ot) {
          if (*ot != mo && (_alreadyScheduled.find((*ot)->getID()) ==
                            _alreadyScheduled.end()))
            schedulable = false;
        }
      }
      if (schedulable && !(*it)->isBranchOperation())
        makeAvailable(*it);
    }
    return true;
  }
  for (std::vector<MO *>::iterator it = follower.begin(); it != follower.end();
       it++) {
    bool schedulable = true;
    MO *f = *it;
    std::vector<MO *> prev = *f->getPrevious();
    prev.insert(prev.end(), f->getPreviousFlags()->begin(),
                f->getPreviousFlags()->end());
    for (std::vector<MO *>::iterator ot = prev.begin();
         schedulable && ot != prev.end(); ot++) {
      if (*ot != mo &&
          (_alreadyScheduled.find((*ot)->getID()) == _alreadyScheduled.end()))
        schedulable = false;
    }
    prev = *f->getWeakPrevious();
    prev.insert(prev.end(), f->getWeakPreviousFlags()->begin(),
                f->getWeakPreviousFlags()->end());
    for (std::vector<MO *>::iterator ot = prev.begin();
         schedulable && ot != prev.end(); ++ot) {
      if (*ot != mo &&
          (_alreadyScheduled.find((*ot)->getID()) == _alreadyScheduled.end()))
        schedulable = false;
    }
    // only schedule non branch operations!
    if (schedulable && !(*it)->isBranchOperation()) {
      makeAvailable(*it);
    }
  }

  follower = *mo->getWeakFollowers();
  follower.insert(follower.end(), mo->getWeakFollowingFlags()->begin(),
                  mo->getWeakFollowingFlags()->end());
  for (std::vector<MO *>::iterator it = follower.begin(); it != follower.end();
       it++) {
    bool schedulable = true;
    MO *f = *it;
    std::vector<MO *> prev = *f->getPrevious();
    prev.insert(prev.end(), f->getPreviousFlags()->begin(),
                f->getPreviousFlags()->end());
    for (std::vector<MO *>::iterator ot = prev.begin();
         schedulable && ot != prev.end(); ot++) {
      if (*ot != mo &&
          (_alreadyScheduled.find((*ot)->getID()) == _alreadyScheduled.end()))
        schedulable = false;
    }
    prev = *f->getWeakPrevious();
    prev.insert(prev.end(), f->getWeakPreviousFlags()->begin(),
                f->getWeakPreviousFlags()->end());
    for (std::vector<MO *>::iterator ot = prev.begin();
         schedulable && ot != prev.end(); ++ot) {
      if (*ot != mo &&
          (_alreadyScheduled.find((*ot)->getID()) == _alreadyScheduled.end()))
        schedulable = false;
    }
    // only schedule non branch operations!
    if (schedulable && !(*it)->isBranchOperation()) {
      makeAvailable(*it);
    }
  }
  return true;
}

bool ListSearch::scheduleIfPossible(MO *mo, MI *mi, const Processor *pro,
                                    bool trial, int force_slot) {
  for (int slot = 0; slot < pro->getIssueSlotNumber(); slot++) {
    if (force_slot == -1 || slot == force_slot) {
      if (!mi->isFree(slot))
        continue;

      if (mo->getIssueSlotConstraint()) { // if there is an issueSlotConstraint
        if ((mo->getIssueSlotConstraint() & (1 << slot)) ==
            0) { // and it forbids sceduling it here.
          LOG_OUTPUT(LOG_M_SCHED_DEBUG,
                     "slot %d is not possible for MO %d (%d) because there are "
                     "contraints.\n",
                     slot, mo->getLineNumber(), mo->getID());
          continue;
        }
      } else {
        // if we are in an --schedule off block, no :01 is set, then only allow
        // the first issue slot
        if (!mo->isReorderable() && slot != 0)
          continue;
      }

      if (!mi->hasFreeSlot() &&
          singles.find(mo->getID()) !=
              singles.end()) { // if the candidate MO should be alone, skip it
        continue;
      }

      if (registers::getNumberOfCondselRegister() > 1) {
        if (mo->isCSOperation() || mo->isCROperation() ||
            mo->isCRBranchOperation() || mo->isCRSOperation() ||
            mo->isSMVtoSingleCondselFlag() || mo->isSMVtoAllCondselFlag()) {

          LOG_OUTPUT(LOG_M_SCHED_DEBUG,
                     "[CSCR dep] Trying to schedule MO with IDCondition %d and "
                     "line number %d in slot %d \n",
                     mo->getIDConditions(), mo->getLineNumber(), slot);

          std::vector<MO *> *previousCondition = mo->getPreviousCondition();

          if (previousCondition->size() == 0) {
            if (mo->isSMVtoAllCondselFlag()) {
              for (int ii = 0; ii < pro->getIssueSlotNumber(); ii++) {
                if (!_scheduableCRCSOperations.at(ii).empty()) {
                  return false;
                }
              }
            } else {
              if (!_scheduableCRCSOperations.at(slot).empty()) {
                continue;
              }
            }
          } else {
            for (std::vector<MO *>::iterator it = previousCondition->begin();
                 it != previousCondition->end(); it++) {
              if ((find(_allScheduledCSCRMOs.begin(),
                        _allScheduledCSCRMOs.end(), (*it)->getIDConditions()) ==
                   _allScheduledCSCRMOs.end())) {
                return false;
              }
            }

            if (mo->isCSOperation()) {
              if (!_scheduableCROperationsAfterCS.at(slot).empty()) {
                continue;
              }
            }

            if ((find(_scheduableCRCSOperations.at(slot).begin(),
                      _scheduableCRCSOperations.at(slot).end(),
                      mo->getIDConditions()) ==
                 _scheduableCRCSOperations.at(slot).end())) {
              continue;
            }
          }
        }
      }

      if (mi->putOperation(mo, slot)) {
        if (pro->isExecuteable(mi)) {
          if (registers::getNumberOfCondselRegister() > 1) {
            if (mo->isCSOperation() || mo->isCROperation() ||
                mo->isCRSOperation() || mo->isSMVtoSingleCondselFlag()) {
              _allScheduledCSCRMOs.push_back(mo->getIDConditions());

              LOG_OUTPUT(LOG_M_SCHED_DEBUG,
                         "[CSCR dep] Scheduled MO with IDCondition %d and line "
                         "number %d in slot %d \n",
                         mo->getIDConditions(), mo->getLineNumber(), slot);

              std::vector<MO *> *followerCondition =
                  mo->getFollowingCondition();
              for (std::vector<MO *>::iterator it = followerCondition->begin();
                   it != followerCondition->end(); it++) {
                for (int ii = 0; ii < pro->getIssueSlotNumber(); ii++) {
                  _scheduableCRCSOperations.at(ii).erase(
                      remove(_scheduableCRCSOperations.at(ii).begin(),
                             _scheduableCRCSOperations.at(ii).end(),
                             (*it)->getIDConditions()),
                      _scheduableCRCSOperations.at(ii).end());
                }
                _scheduableCRCSOperations.at(slot).push_back(
                    (*it)->getIDConditions());

                LOG_OUTPUT(
                    LOG_M_SCHED_DEBUG,
                    "[CSCR dep] Added MO with IDCondition %d, line number %d "
                    "and slot %d to _scheduableCRCSOperations \n",
                    (*it)->getIDConditions(), (*it)->getLineNumber(), slot);
              }

              if (mo->isCSOperation() || mo->isCRSOperation()) {
                std::vector<MO *> *followerCondition =
                    mo->getFollowingCondition();

                for (std::vector<MO *>::iterator it =
                         followerCondition->begin();
                     it != followerCondition->end(); it++) {
                  _scheduableCROperationsAfterCS.at(slot).push_back(
                      (*it)->getIDConditions());

                  LOG_OUTPUT(
                      LOG_M_SCHED_DEBUG,
                      "[CSCR dep] Added MO with IDCondition %d, line number %d "
                      "and slot %d to _scheduableCROperationsAfterCS \n",
                      (*it)->getIDConditions(), (*it)->getLineNumber(), slot);
                }
              }

              for (int ii = 0; ii < pro->getIssueSlotNumber(); ii++) {
                _scheduableCROperationsAfterCS.at(ii).erase(
                    remove(_scheduableCROperationsAfterCS.at(ii).begin(),
                           _scheduableCROperationsAfterCS.at(ii).end(),
                           mo->getIDConditions()),
                    _scheduableCROperationsAfterCS.at(ii).end());
                _scheduableCRCSOperations.at(ii).erase(
                    remove(_scheduableCRCSOperations.at(ii).begin(),
                           _scheduableCRCSOperations.at(ii).end(),
                           mo->getIDConditions()),
                    _scheduableCRCSOperations.at(ii).end());
              }
            }

            if (mo->isSMVtoAllCondselFlag()) {
              LOG_OUTPUT(LOG_M_SCHED_DEBUG,
                         "[CSCR dep] Scheduled MO with IDCondition %d and line "
                         "number %d in slot %d \n",
                         mo->getIDConditions(), mo->getLineNumber(), slot);

              _allScheduledCSCRMOs.push_back(mo->getIDConditions());

              for (int ii = 0; ii < pro->getIssueSlotNumber(); ii++) {
                std::vector<MO *> *followerCondition =
                    mo->getFollowingCondition();
                for (std::vector<MO *>::iterator it =
                         followerCondition->begin();
                     it != followerCondition->end(); it++) {
                  _scheduableCRCSOperations.at(ii).push_back(
                      (*it)->getIDConditions());
                  LOG_OUTPUT(
                      LOG_M_SCHED_DEBUG,
                      "[CSCR dep] Added MO with IDCondition %d, line number %d "
                      "and slot %d to _scheduableCRCSOperations \n",
                      (*it)->getIDConditions(), (*it)->getLineNumber(), slot);
                }
              }
              _scheduableCRCSOperations.at(slot).erase(
                  remove(_scheduableCRCSOperations.at(slot).begin(),
                         _scheduableCRCSOperations.at(slot).end(),
                         mo->getIDConditions()),
                  _scheduableCRCSOperations.at(slot).end());
            }

            if (mo->isCRBranchOperation() && !trial) {
              LOG_OUTPUT(LOG_M_SCHED_DEBUG,
                         "[CSCR dep] Scheduled MO with IDCondition %d and line "
                         "number %d in slot %d \n",
                         mo->getIDConditions(), mo->getLineNumber(), slot);

              _allScheduledCSCRMOs.push_back(mo->getIDConditions());
              _scheduableCRCSOperations.at(slot).erase(
                  remove(_scheduableCRCSOperations.at(slot).begin(),
                         _scheduableCRCSOperations.at(slot).end(),
                         mo->getIDConditions()),
                  _scheduableCRCSOperations.at(slot).end());
              _scheduableCROperationsAfterCS.at(slot).erase(
                  remove(_scheduableCROperationsAfterCS.at(slot).begin(),
                         _scheduableCROperationsAfterCS.at(slot).end(),
                         mo->getIDConditions()),
                  _scheduableCROperationsAfterCS.at(slot).end());
            }
          }
          return true;
        } else {
          LOG_OUTPUT(LOG_M_SCHED_DEBUG,
                     "processor prohibits scheduling line %u at this moment.\n",
                     mo->getLineNumber());
          mi->removeOperation(slot);
        }
      }
    }
  }
  return false;
}

bool containsPrecessor(MI *mi, MO *testing) {
  MO **mos = mi->getOperations();
  vector<MO *> prev = *testing->getPrevious();
  prev.insert(prev.end(), testing->getPreviousFlags()->begin(),
              testing->getPreviousFlags()->end());
  for (uint i = 0; i < mi->getNumberIssueSlots();) {
    if (mos[i] == NULL) {
      i++;
      continue;
    }
    if (!mos[i]->isReorderable())
      return true;
    for (vector<MO *>::iterator it = prev.begin(), end = prev.end(); it != end;
         it++) {
      if (*it == mos[i]) {
        // found a dependency
        return true;
      }
    }
    i += mos[i]->opLengthMultiplier();
  }
  return false;
}

bool containsConditionPrecessor(MI *mi, MO *testing) {
  MO **mos = mi->getOperations();
  vector<MO *> *prev = testing->getPreviousCondition();
  for (uint i = 0; i < mi->getNumberIssueSlots();) {
    if (mos[i] == NULL) {
      i++;
      continue;
    }
    if (!mos[i]->isReorderable())
      return true;
    for (vector<MO *>::iterator it = prev->begin(), end = prev->end();
         it != end; it++) {
      if (*it == mos[i]) {
        // found a dependency
        return true;
      }
    }
    i += mos[i]->opLengthMultiplier();
  }
  return false;
}

bool ListSearch::checkSchedulable(MO *mo, const Processor *pro) {
  MI *mi = new MI();
  bool ret = scheduleIfPossible(mo, mi, pro, true);
  delete mi;
  return ret;
}

bool ListSearch::allPredecessorsPlaced(std::vector<MO *>::const_iterator begin,
                                       std::vector<MO *>::const_iterator end,
                                       bool pipelined,
                                       const std::string &basename) {
  for (auto it = begin; it != end; ++it) {
    MO *p = *it;
    bool running = false;
    if (pipelined && p->getOperation()->getBaseName() == basename) {
      for (auto jt = _runningMOs.begin(); !running && jt != _runningMOs.end();
           ++jt)
        if (jt->first->getID() == p->getID())
          running = true;
    }
    if (!running &&
        _alreadyScheduled.find(p->getID()) == _alreadyScheduled.end())
      return false;
  }
  return true;
}

bool ListSearch::allPredecessorsPlacedOrRunning(
    int latency, std::vector<MO *>::const_iterator begin,
    std::vector<MO *>::const_iterator end) {
  for (auto it = begin; it != end; ++it) {
    auto mo = *it;
    bool running = false;
    for (auto jt = _runningMOs.begin(); !running && jt != _runningMOs.end();
         ++jt) {
      if (jt->first->getID() == mo->getID() && jt->second == latency) {
        running = true;
      }
    }
    if (!running) {
      if (_alreadyScheduled.find((*it)->getID()) == _alreadyScheduled.end())
        return false;
    }
  }
  return true;
}

bool ListSearch::allPredecessorsPlaced(MO *mo) {
  auto previous = mo->getPrevious();
  if (allPredecessorsPlaced(previous->begin(), previous->end(),
                            mo->getOperation()->isPipelined(),
                            mo->getOperation()->getBaseName())) {
    auto weakPrev = mo->getWeakPrevious();
    if (!allPredecessorsPlaced(weakPrev->begin(), weakPrev->end(), true,
                               mo->getOperation()->getBaseName()))
      return false;
    auto prevFlags = mo->getPreviousFlags();
    auto weakPrevFlags = mo->getWeakPreviousFlags();
    return allPredecessorsPlacedOrRunning(getForwarding(), prevFlags->begin(),
                                          prevFlags->end()) &&
           allPredecessorsPlacedOrRunning(
               getForwarding(), weakPrevFlags->begin(), weakPrevFlags->end());
  } else
    return false;
}

bool ListSearch::allWeakPredecessorsPlaced(MO *mo) {
  auto weakPrev = mo->getWeakPrevious();
  if (allPredecessorsPlaced(weakPrev->begin(), weakPrev->end(),
                            mo->getOperation()->isPipelined(),
                            mo->getOperation()->getBaseName())) {
    auto weakPrevFlags = mo->getWeakPreviousFlags();
    return allPredecessorsPlacedOrRunning(
        getForwarding(), weakPrevFlags->begin(), weakPrevFlags->end());
  } else
    return false;
}

void ListSearch::releaseMO(MO *mo, std::list<entry> *list, const Processor *pro,
                           uint slot) {
  if (mo) {
    entry e;
    e.mo = mo;
    e.value = _moWeights[mo->getID()];
    e.schedulable = allPredecessorsPlaced(mo) && allWeakPredecessorsPlaced(mo);
    insertInOrder(e, list);
    _alreadyScheduled.erase(mo->getID());
    if (registers::getNumberOfCondselRegister() > 1) {
      if (mo->isCSOperation() || mo->isCROperation() || mo->isCRSOperation() ||
          mo->isSMVtoSingleCondselFlag() || mo->isSMVtoAllCondselFlag() ||
          mo->isCRBranchOperation()) {
        _allScheduledCSCRMOs.erase(remove(_allScheduledCSCRMOs.begin(),
                                          _allScheduledCSCRMOs.end(),
                                          mo->getIDConditions()),
                                   _allScheduledCSCRMOs.end());

        std::vector<MO *> *previousCondition = mo->getPreviousCondition();

        if (previousCondition->size() == 0) {

        } else {
          bool previousScheduled = true;
          for (std::vector<MO *>::iterator it = previousCondition->begin();
               it != previousCondition->end(); it++) {
            if ((find(_allScheduledCSCRMOs.begin(), _allScheduledCSCRMOs.end(),
                      (*it)->getIDConditions()) ==
                 _allScheduledCSCRMOs.end())) {
              previousScheduled = false;
            }
          }
          if (previousScheduled == true) {
            _scheduableCRCSOperations.at(slot).push_back(mo->getIDConditions());
            if (mo->isCROperation() || mo->isCRSOperation()) {
              _scheduableCROperationsAfterCS.at(slot).push_back(
                  mo->getIDConditions());
            }
          }
        }

        std::vector<MO *> *followerCondition = mo->getFollowingCondition();
        for (std::vector<MO *>::iterator it = followerCondition->begin();
             it != followerCondition->end(); it++) {
          for (int ii = 0; ii < pro->getIssueSlotNumber(); ii++) {
            _scheduableCRCSOperations.at(ii).erase(
                remove(_scheduableCRCSOperations.at(ii).begin(),
                       _scheduableCRCSOperations.at(ii).end(),
                       (*it)->getIDConditions()),
                _scheduableCRCSOperations.at(ii).end());
            _scheduableCROperationsAfterCS.at(ii).erase(
                remove(_scheduableCROperationsAfterCS.at(ii).begin(),
                       _scheduableCROperationsAfterCS.at(ii).end(),
                       (*it)->getIDConditions()),
                _scheduableCROperationsAfterCS.at(ii).end());
          }
        }
      }
    }
    for (auto it = _runningMOs.begin(); it != _runningMOs.end(); ++it) {
      if (it->first == mo) {
        _runningMOs.erase(it);
        break;
      }
    }
  }
}

void ListSearch::storeCurrentState(const Processor *pro) {
  _alreadyScheduled_STORED.clear();
  _alreadyScheduled_STORED.insert(_alreadyScheduled.begin(),
                                  _alreadyScheduled.end());

  if (registers::getNumberOfCondselRegister() > 1) {
    for (int ii = 0; ii < pro->getIssueSlotNumber(); ii++) {
      _scheduableCROperationsAfterCS_STORED.at(ii).clear();
      _scheduableCROperationsAfterCS_STORED.at(ii).insert(
          _scheduableCROperationsAfterCS_STORED.at(ii).begin(),
          _scheduableCROperationsAfterCS.at(ii).begin(),
          _scheduableCROperationsAfterCS.at(ii).end());

      _scheduableCRCSOperations_STORED.at(ii).clear();
      _scheduableCRCSOperations_STORED.at(ii).insert(
          _scheduableCRCSOperations_STORED.at(ii).begin(),
          _scheduableCRCSOperations.at(ii).begin(),
          _scheduableCRCSOperations.at(ii).end());
    }

    _allScheduledCSCRMOs_STORED.clear();
    _allScheduledCSCRMOs_STORED.insert(_allScheduledCSCRMOs_STORED.begin(),
                                       _allScheduledCSCRMOs.begin(),
                                       _allScheduledCSCRMOs.end());
  }

  _runningMOs_STORED.clear();
  _runningMOs_STORED.insert(_runningMOs_STORED.begin(), _runningMOs.begin(),
                            _runningMOs.end());

  if (_schedulable_STORED)
    delete _schedulable_STORED;
  _schedulable_STORED = new std::list<entry>();
  _schedulable_STORED->insert(_schedulable_STORED->begin(),
                              schedulable.front()->begin(),
                              schedulable.front()->end());
}

void ListSearch::restorePreviousState(const Processor *pro) {
  _alreadyScheduled.clear();
  _alreadyScheduled.insert(_alreadyScheduled_STORED.begin(),
                           _alreadyScheduled_STORED.end());

  if (registers::getNumberOfCondselRegister() > 1) {
    for (int ii = 0; ii < pro->getIssueSlotNumber(); ii++) {
      _scheduableCROperationsAfterCS.at(ii).clear();
      _scheduableCROperationsAfterCS.at(ii).insert(
          _scheduableCROperationsAfterCS.at(ii).begin(),
          _scheduableCROperationsAfterCS_STORED.at(ii).begin(),
          _scheduableCROperationsAfterCS_STORED.at(ii).end());

      _scheduableCRCSOperations.at(ii).clear();
      _scheduableCRCSOperations.at(ii).insert(
          _scheduableCRCSOperations.at(ii).begin(),
          _scheduableCRCSOperations_STORED.at(ii).begin(),
          _scheduableCRCSOperations_STORED.at(ii).end());
    }

    _allScheduledCSCRMOs.clear();
    _allScheduledCSCRMOs.insert(_allScheduledCSCRMOs.begin(),
                                _allScheduledCSCRMOs_STORED.begin(),
                                _allScheduledCSCRMOs_STORED.end());
  }

  _runningMOs.clear();
  _runningMOs.insert(_runningMOs.begin(), _runningMOs_STORED.begin(),
                     _runningMOs_STORED.end());

  if (schedulable.size()) {
    delete schedulable.front();
    schedulable.clear();
  }
  schedulable.insert(schedulable.begin(), _schedulable_STORED);
  _schedulable_STORED = 0;
}

void ListSearch::releaseStoredState(const Processor *pro) {
  _alreadyScheduled_STORED.clear();
  _runningMOs_STORED.clear();
  if (registers::getNumberOfCondselRegister() > 1) {
    _allScheduledCSCRMOs_STORED.clear();
    for (int ii = 0; ii < pro->getIssueSlotNumber(); ii++) {
      _scheduableCROperationsAfterCS_STORED.at(ii).clear();
      _scheduableCRCSOperations_STORED.at(ii).clear();
    }
    _scheduableCROperationsAfterCS_STORED.clear();
    _scheduableCRCSOperations_STORED.clear();
  }
  if (_schedulable_STORED) {
    delete _schedulable_STORED;
    _schedulable_STORED = 0;
  }
}

/**
 * @brief Check for operations, if their latency is fulfilled.
 *
 * For each MO in the given MI, check if their latency is bigger than delay. If
 * so, the operation is moved from _alreadyPlaced to _runningMOs.
 */
void ListSearch::checkFulfilledLatency(MI *mi, int delay) {
  uint slot = 0;
  while (slot < mi->getNumberIssueSlots()) {
    MO *mo = mi->getOperations()[slot];
    if (mo) {
      if (mo->getLatency() > delay) {
        _alreadyScheduled.erase(mo->getID());
        _runningMOs.push_back(std::make_pair(mo, mo->getLatency() - delay));
      } else if (mo->getOperation()->getName().substr(0, 11) == "STORERCUPPL" &&
                 mo->getLatency() == delay) {
        auto follower = *mo->getFollowing();
        follower.insert(follower.end(), mo->getFollowingFlags()->begin(),
                        mo->getFollowingFlags()->end());
        for (auto it = follower.begin(); it != follower.end(); ++it) {
          MO *f = *it;
          if (f->getOperation()->getName().substr(0, 7) == "LOADRCU") {
            // This LOADRCU operation has to be placed in the next MI!
            if (std::find(_scheduleNow.begin(), _scheduleNow.end(), f) ==
                _scheduleNow.end()) {
              _scheduleNow.push_back(f);
            }
          }
        }
      }
      slot += mo->opLengthMultiplier();
    } else
      slot += 1;
  }
}

void ListSearch::releaseLastOperations(int count, Program *instructions,
                                       const Processor *processor) {
  int delay = 1;
  for (auto it = instructions->rbegin() + count; it != instructions->rend();
       ++it) {
    checkFulfilledLatency(*it, delay);
    delay++;
  }

  std::list<entry> *list = new std::list<entry>();
  auto mi_it = instructions->end() - count;
  while (mi_it != instructions->end()) {
    auto mi = *mi_it;
    uint slot = 0;
    while (slot < mi->getNumberIssueSlots()) {
      auto mo = mi->getOperations()[slot];
      if (mo) {
        releaseMO(mo, list, processor, slot);
        slot += mo->opLengthMultiplier();
      } else
        slot += 1;
    }
    delete mi;
    mi_it = instructions->erase(mi_it);
  }
  if (!list->empty())
    schedulable.push_back(list);
  else
    delete list;
}

Program *ListSearch::placeBranch(MO *branch, int slot,
                                 const Processor *processor) {
  LOG_OUTPUT(LOG_M_SCHED_DEBUG,
             "Placing branch/loop operation in issue slot %d\n", slot);
  Program *instr = new Program();

  int cycles = branch->isReorderable() ? branch->getLatency() : 1;
  _runningMOs.push_back(std::make_pair(branch, cycles));
  MI *mi = new MI();

  //    bool hasIssueSlotConstraint = branch->getIssueSlotConstraint();
  //    if (!hasIssueSlotConstraint)
  //        branch->setIssueSlotConstraint(1 << slot);
  if (!scheduleIfPossible(branch, mi, processor, false, slot)) {
    delete mi;
    delete instr;
    //    	if (!hasIssueSlotConstraint)
    //    		branch->setIssueSlotConstraint(0);
    return 0;
  }

  while (_alreadyScheduled.size() < _moCount) {
    fillNextMI(mi, processor);
    if (isLog(LOG_M_SCHED_DEBUG)) {
      LOG_OUTPUT(LOG_M_SCHED_DEBUG, "Adding instruction:\n");
      mi->writeOutReadable(cout);
    }
    if (!nextCycle(processor)) {
      delete instr;
      instr = 0;
      break;
    }
    instr->push_back(mi);
    mi = new MI();
    if (--cycles == 0) {
      LOG_OUTPUT(LOG_M_SCHED_DEBUG, "Branch/loop slots are filled!\n");
      if (_alreadyScheduled.size() <
          _moCount + 1) { // +1 to count the branch operation!
        LOG_OUTPUT(LOG_M_SCHED_DEBUG, "We still have operations to place! "
                                      "Branch placement is invalid!\n");
        delete instr;
        instr = 0;
      }
      break;
    }
  }
  delete mi;
  //    if (!hasIssueSlotConstraint)
  //        branch->setIssueSlotConstraint(0);
  return instr;
}

void ListSearch::printSchedulable() {
  int schedulableIdx = 0;
  for (auto it = schedulable.begin(); it != schedulable.end(); ++it) {
    LOG_OUTPUT(LOG_M_ALWAYS, "Schedulable[%d]:\n", schedulableIdx);
    for (auto ot = (*it)->begin(); ot != (*it)->end(); ++ot) {
      entry e = (*ot);
      LOG_OUTPUT(LOG_M_ALWAYS, "  MO: %2d (%2d), schedulable: %d\n",
                 e.mo->getLineNumber(), e.mo->getID(), e.schedulable);
    }
    schedulableIdx++;
  }
}

void ListSearch::printAlreadyScheduled() {
  for (auto it = _alreadyScheduled.begin(); it != _alreadyScheduled.end(); ++it)
    LOG_OUTPUT(LOG_M_ALWAYS, "  MO (%2d)\n", *it);
}

void ListSearch::clearSchedulable() {
  for (auto it = schedulable.begin(); it != schedulable.end(); ++it)
    if (*it)
      delete *it;
  schedulable.clear();
}

void ListSearch::scheduleBranch(MO *branch, const Processor *processor,
                                Program *instructions) {
  clearSchedulable();
  _runningMOs.clear();

  if (isLog(LOG_M_SCHED_DEBUG)) {
    LOG_OUTPUT(LOG_M_SCHED_DEBUG, "[schedule branch] Instructions so far:\n");
    for (auto it = instructions->begin(); it != instructions->end(); ++it) {
      (*it)->writeOutReadable(cout);
    }
  }

  uint instrCount = instructions->size();
  uint branchDelay = branch->getLatency();

  int shift = min(instrCount, branchDelay);
  releaseLastOperations(shift, instructions, processor);

  int slot = 0;

  // Reschedule released operations and branch
  while (true) {
    if (allPredecessorsPlaced(branch)) {
      if (schedulable.empty()) {
        MI *mi = new MI();
        if (!scheduleIfPossible(branch, mi, processor)) {
          LOG_OUTPUT(LOG_M_ALWAYS,
                     "Could not place branch/loop operation in line %d\n",
                     branch->getLineNumber());
          EXIT_ERROR;
        }
        instructions->push_back(mi);
        if (isLog(LOG_M_SCHED_DEBUG)) {
          LOG_OUTPUT(LOG_M_SCHED_DEBUG,
                     "[schedule branch] Branch scheduled:\n");
          for (auto it = instructions->begin(); it != instructions->end();
               ++it) {
            (*it)->writeOutReadable(cout);
          }
        }
        int lat = branch->isReorderable() ? branch->getLatency() : 1;
        _runningMOs.push_back(std::make_pair(branch, lat));
        nextCycle(processor);
        break;
      } else {
        storeCurrentState(processor);
        Program *instr = placeBranch(branch, slot, processor);
        if (instr) {
          instructions->insert(instructions->end(), instr->begin(),
                               instr->end());
          instr->clear();
          delete instr;
          releaseStoredState(processor);
          break;
        } else
          restorePreviousState(processor);
      }
    }
    if (slot < processor->getIssueSlotNumber() - 1)
      ++slot;
    else {
      LOG_OUTPUT(LOG_M_SCHED_DEBUG,
                 "Branch/loop could not be placed. Filling next cycle:\n");
      MI *mi = new MI();
      fillNextMI(mi, processor);
      if (isLog(LOG_M_SCHED_DEBUG))
        mi->writeOutReadable(cout);
      nextCycle(processor);
      instructions->push_back(mi);
      slot = 0;
    }
  }
}

void ListSearch::insertBranch(Processor *pro, Program &instructions,
                              MO *branch) {
  MI *last = new MI();
  if (!scheduleIfPossible(branch, last, pro)) {
    cerr << "Could not schedule a branch operation in line "
         << branch->getLineNumber() << " into an empty MI" << endl;
    EXIT_ERROR;
  }
  // this will just add it at the end and then proceed as usual.
  // uncommented because it has been useful when trying to debug branch errors.
  /*	instructions.push_back(last);
          for (int i = 1; i < branch->getLatency(); i++) {
                  instructions.push_back(new MI());
          }
          return ; // */

  // first i get a pointer to the last element
  auto it = instructions.end(), end = instructions.end();

  if (branch->isReorderable()) {

    it--;
    // then i check whether it is possible to move the loop operation upwards
    // and how much.
    int possible = branch->getLatency();
    int size = instructions.size();
    int branch_moves = 0;
    for (int i = 0; i < possible; i++) {
      if (size - i == 0)
        break;
      MI *test = *it;
      if (containsPrecessor(test, branch))
        break;
      if (registers::getNumberOfCondselRegister() > 1) {
        if (containsConditionPrecessor(test, branch))
          break;
      }
      it--;
      if (i != 0)
        branch_moves++;
    }
    // after checking, that there is all free, we have to move forward again.
    it++;
    if (it != end) {
      MI *test = *it;
      // if it is not possible, to schedule the branch operation with an
      // existing mi,
      if (!scheduleIfPossible(branch, test, pro)) {
        if (possible <= size) {
          it++; // we have to insert it after the tested one.
        } else {
          branch_moves++;
        }
        instructions.insert(it, last);
      } else {
        delete last;
        it++;
      }
    } else { // if the SLM contains only of zero instructions, or an upmove is
             // not possible, this happens.
      instructions.push_back(last);
    }
    //		MI* l = instructions.back();
    //		if(l->getOperations()[0] != NULL) {
    //			if(l->getOperations()[0]->isReorderable()) {
    // add NOP's at the end, to fill up the latency if necessary.
    for (int i = 0; i < possible - branch_moves - 1; i++) {
      //					if(it==end){
      instructions.push_back(new MI());
      //					} else
      //						it++;
      //				}
      //			}
    }
  } else {
    if (branch->getIssueSlotConstraint() == 2) {
      it--;
      MI *test = *it;
      scheduleIfPossible(branch, test, pro);
      delete last;
    } else {
      instructions.push_back(last);
    }
  }
}