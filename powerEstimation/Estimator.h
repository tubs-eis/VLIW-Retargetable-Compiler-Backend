// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_ESTIMATOR_H
#define SCHEDULER_ESTIMATOR_H

#include "InstructionModel.h"
#include "Program.h"
#include "TransitionEnergyEstimator.h"
#include "virtual_reg.h"

class Estimator {

private:
  Estimator() {}
  Estimator(Estimator const &);
  void operator=(Estimator const &);

public:
  static Estimator &getInstance() {
    static Estimator instance;
    return instance;
  }

  /**
   *
   * @param ins
   * @param map
   * @return RA Energy + Instruction Energy
   */
  double getCumulativeTransitionEnergy(Program *ins, VirtualRegisterMap *map) {
    TransitionEnergyEstimator estimator;
    double totalTransitionEnergy = -1;
    if (map) {
      totalTransitionEnergy = 0;
      for (Program::iterator it = ins->begin(), end = ins->end(); it != end;
           it++) {
        estimator.push(*it, map);
        double energy = estimator.getLastTransitionEnergy();
        it.operator*()->setTransitionEnergy(energy);
        totalTransitionEnergy += energy;
      }

      totalTransitionEnergy +=
          InstructionModel::getInstance().generateInstructionEnergy(ins);
    }

    return totalTransitionEnergy;
  }
};

#endif // SCHEDULER_ESTIMATOR_H
