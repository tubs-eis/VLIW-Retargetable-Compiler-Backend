// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef GenSearch_H
#define GenSearch_H

#include "Program.h"
#include "global.h"
#include <functional>
#include <list>
#include <vector>
class MO;
class Processor;
class MI;
class SLM;
namespace gen_sched {
class sched_chromosome;
}

#include <set>
#include <unordered_set>

struct entry {
  bool schedulable;
  MO *mo;
  int value;
};

/** \brief Algorithm for scheduling the instructions using a genetic algorithm.
 *
 */
class ListSearch {
private:
  uint _moCount;
  std::unordered_set<int> singles;
  /** an ordered list of schedulable MO
   *
   * The list is ordered by the comparing function */
  std::vector<std::list<entry> *> schedulable;
  /** List of MOs that have to be placed in the next MI. */
  std::vector<MO *> _scheduleNow;

  std::vector<std::vector<int>> _scheduableCROperationsAfterCS;
  std::vector<std::vector<int>> _scheduableCRCSOperations;
  std::vector<int> _allScheduledCSCRMOs;
  std::set<int> _alreadyScheduled;
  std::vector<std::pair<MO *, int>>
      _runningMOs; // MOs that have been placed, but their latency is not yet
                   // over

  std::set<int> _alreadyScheduled_STORED;
  std::vector<std::pair<MO *, int>> _runningMOs_STORED;
  std::list<entry> *_schedulable_STORED;

  std::vector<std::vector<int>> _scheduableCROperationsAfterCS_STORED;
  std::vector<std::vector<int>> _scheduableCRCSOperations_STORED;
  std::vector<int> _allScheduledCSCRMOs_STORED;

  /***
   * Map between original ID to Scheduling Weights
   */
  std::map<int, int> _moWeights; // map from MO ids to scheduling weight

  /** Inserts an element into schedulable
   *
   * the element is inserted into the list in front of the
   * first element where compare(element,new) returns true.ListSearch
   * or at the end of the list if none returns true;
   */
  void insertInOrder(entry &e, std::list<entry> *list);
  /** A helper functon to schedule an MO
   *
   * This function tries to schedule an MO and verifies,
   * that it is executeable by the processor.
   *
   * @return true If it could be scheduled.
   * @return false Otherwise.
   */
  bool scheduleIfPossible(MO *mo, MI *mi, const Processor *pro,
                          bool trial = false, int force_slot = -1);
  /**
   * \brief Check fulfilled weak dependencies of a MO.
   *
   * For the given mo, check if the weak dependent operations are made available
   * for scheduling.
   */
  void checkFollowers(MO *mo, std::vector<MO *> *followers);
  bool allMOsRunning(std::vector<MO *> *mos, MO *currentMO = 0);
  bool allPredecessorsRunning(MO *m, MO *currentMO);
  bool checkSchedulable(MO *mo, const Processor *pro);
  void releaseLastOperations(int count, Program *instructions,
                             const Processor *processor);
  void releaseMO(MO *mo, std::list<entry> *list, const Processor *processor,
                 uint slot);
  bool schedCandidatesLeft();
  bool isLastPartingSection() const;
  int numSchedCandidates() const;
  bool allPredecessorsPlaced(std::vector<MO *>::const_iterator begin,
                             std::vector<MO *>::const_iterator end,
                             bool pipelined, const std::string &basename);
  bool allPredecessorsPlacedOrRunning(int latency,
                                      std::vector<MO *>::const_iterator begin,
                                      std::vector<MO *>::const_iterator end);
  bool allPredecessorsPlaced(MO *mo);
  bool allWeakPredecessorsPlaced(MO *mo);
  void checkFulfilledLatency(MI *mi, int delay);
  void fillNextMI(MI *mi, const Processor *processor);
  bool nextCycle(const Processor *processor);
  void makeFlagDependentMOsAvailable(MO *mo);
  void makeDependentMOsAvailable(MO *mo, std::function<bool(MO *)> pred);

  void storeCurrentState(const Processor *pro);
  void restorePreviousState(const Processor *pro);
  void releaseStoredState(const Processor *pro);
  Program *placeBranch(MO *branch, int slot, const Processor *processor);
  void removeFromSchedulable(MO *mo);
  bool collidesWithRunning(MO *mo);
  void clearSchedulable();

  // todo: revert printSchedulable to private
  //  void printSchedulable();
  void printAlreadyScheduled();

public:
  void makeAvailable(MO *mo);
  ListSearch(SLM *slm, const Processor *processor,
             gen_sched::sched_chromosome *individual = 0);
  ~ListSearch();

  /** Schedule and return the next MO
   *
   * @param pro A processor to check for scheduleability
   * @param mi A MI where the MO has to be added.
   * @return The scheduled MO
   * @return NULL if nothing is to schedule
   */
  MO *findCandidateMOforMI(const Processor *pro, MI *mi);

  /** Tell an algorithm, that the results are ready
   *
   * This function gets called, after the latency of a scheduled MO has been
   * done in clock cycles. It is to inform the algorithms, that they can now
   * schedule the MO depending on the results of this one. If the latency is 0,
   * it is called immediatly after scheduleNext returns with its return
   * parameter.
   * @param mo A MO which results are now ready
   * @returns false, if a problem occured that prevents further scheduling.
   */
  bool resultsReady(MO *mo, const Processor *processor);

  /**
   * Schedule this Listsearch getInstance of an BB (SLM).
   * @param pro
   * @param notSchedulableCSCR
   * @return if possible return a program with scheduled instructions. If
   * failed, return 0;
   */
  Program *scheduleMOs(const Processor *pro, int &notSchedulableCSCR);
  void scheduleBranch(MO *branch, const Processor *processor,
                      Program *instructions);
  void fillLatencyCycles(Program *instructions);

  void insertBranch(Processor *pro, Program &instructions, MO *branch);

  void printSchedulable();
};

#endif // GenSearch_H
