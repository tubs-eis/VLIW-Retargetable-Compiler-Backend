// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_TRANSITIONENERGYESTIMATOR_H
#define SCHEDULER_TRANSITIONENERGYESTIMATOR_H

#include <MI.h>
class TransitionEnergyEstimator {
private:
  // this should be 8 = 2 issue slots x (SCR1 + SRC2 + {X2/MAC register} )
  array<int, 8> physicalPortsT0 = {0};
  array<int, 8> physicalPortsT1 = {0};

  vector<array<int, 8>> history;
  vector<double> historyEnergy;
  double transitionEnergy = 0;

  void updateRegister(array<int, 8> &oldArrray, array<int, 8> &newArray);

  /***
   * Calculate next incremental Energy Consumption Estimate.
   */
  void updateModel();

public:
  TransitionEnergyEstimator() {
    //    std::cout << "starting Energy Estimation\n";
  }

  // template has to be defined here or else linker does not know what to link.
  //  template <typename T> void push(const MI *mi, T const *map) {
  void push(const MI *mi, VirtualRegisterMap const *map) {

    //  update time slots 1 and 0
    updateRegister(physicalPortsT1, physicalPortsT0);
    // copy new physical port configuration to time slot 0
    physicalPortsT0 = mi->getPhysicalRegister(map);
    // record current
    updateModel();
  }

  /***
   * Get last Transition Energy calculated.
   * @return Transition energy of the last pushed transition.
   */
  double getLastTransitionEnergy() const { return historyEnergy.back(); }

  double getTransitionEnergy() const { return transitionEnergy; }

  std::vector<double> getHistory() const { return historyEnergy; }

  void printHistory() const {
    stringstream ss;
    ss << "Energy Transition " << transitionEnergy << endl;
    for (int i = 0; i < history.size(); i++) {
      ss << historyEnergy[i] << " -- ";
      for (int j = 0; j < history[i].size(); j++) {
        ss << history[i][j] << " ";
      }
      ss << endl;
    }
    LOG_OUTPUT(LOG_M_ALWAYS, ss.str().c_str());
  }
};

#endif // SCHEDULER_TRANSITIONENERGYESTIMATOR_H
