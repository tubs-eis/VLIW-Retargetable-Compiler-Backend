// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_PROGRAM_H
#define SCHEDULER_PROGRAM_H

class MI;
// #include "MI.h"
#include "virtual_reg.h"
#include <vector>

/***
 * A Program is a collection of MIs. Also useful to iterate over the MIs.
 *
 * Scheduling order. Will be used for Assembler printing.
 */
class Program {
private:
  std::vector<MI *> _mis;
  double cumulativeTransitionEnergy = -1;

public:
  typedef std::vector<MI *>::iterator iterator;
  typedef std::vector<MI *>::const_iterator const_iterator;
  typedef std::vector<MI *>::reverse_iterator reverse_iterator;

  Program(){};
  /** @brief Deletes all MIs.
   *
   * Deletes the MIs stored in this program. NOTE: The MI destructor deletes the
   * contained MOs only, if their line number is 0!
   */
  virtual ~Program();

  iterator begin() { return _mis.begin(); }
  iterator end() { return _mis.end(); }
  reverse_iterator rbegin() { return _mis.rbegin(); }
  reverse_iterator rend() { return _mis.rend(); }
  const_iterator begin() const { return _mis.begin(); }
  const_iterator end() const { return _mis.end(); }

  iterator erase(iterator it) { return _mis.erase(it); }
  iterator insert(iterator pos, iterator begin, iterator end) {
    return _mis.insert(pos, begin, end);
  }
  iterator insert(iterator pos, MI *mi) { return _mis.insert(pos, mi); }
  void clear() { _mis.clear(); }

  unsigned int size() const { return _mis.size(); }

  void push_back(MI *mi) { _mis.push_back(mi); }

  const MI *getMI(int index) const { return _mis[index]; }
  MI *getMI(int index) { return _mis[index]; }

  void setCumulativeTransitionEnergy(double energy) {
    cumulativeTransitionEnergy = energy;
  }

  double getCumulativeTransitionEnergy() const {
    return cumulativeTransitionEnergy;
  }

  int getMergeCount();

  void writeOutInstructions(std::ostream &out,
                            const VirtualRegisterMap *mapping = nullptr,
                            bool printEnergy = false) const;

  void writeOutInstructionsCompilable(
      std::ostream &out, const VirtualRegisterMap *mapping = nullptr) const;

  void writeOutScheduledWeight(std::ostream &out, int SLM_ID) const;

  uint getInstructionTransitions() const;

  void calculateTransitionEnergy(VirtualRegisterMap *mapping);
};

#endif // SCHEDULER_PROGRAM_H
