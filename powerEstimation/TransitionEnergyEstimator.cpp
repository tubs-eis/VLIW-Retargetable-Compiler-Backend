// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "TransitionEnergyEstimator.h"
#include "ModelImporter.h"

void TransitionEnergyEstimator::updateRegister(array<int, 8> &oldArrray,
                                               array<int, 8> &newArray) {
  for (int i = 0; i < newArray.size(); i++) {
    if (newArray[i] != -1) {
      oldArrray[i] = newArray[i];
    }
  }
}

void TransitionEnergyEstimator::updateModel() {
  history.push_back(physicalPortsT0);
  double energy = ModelImporter::getInstance().predictTransitionEnergy(
      physicalPortsT1, physicalPortsT0);
  historyEnergy.push_back(energy);
  transitionEnergy += energy;
}
