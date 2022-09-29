// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#include "Program.h"
#include "../powerEstimation/TransitionEnergyEstimator.h"
#include "MI.h"

void Program::printInstructions(std::ostream &out,
                                const VirtualRegisterMap *mapping,
                                bool printEnergy) const {
  double totalEnergy = 0;
  if (printEnergy) {
    TransitionEnergyEstimator energyEstimator;
    for (auto it = begin(); it != end(); ++it) {
      energyEstimator.push(*it, mapping);
    }
    totalEnergy = energyEstimator.getTransitionEnergy();
  }
  int assemblerLine = 0;
  for (auto it = begin(); it != end(); ++it) {
    auto mi = *it;
    (*it)->registerMO2AssemblerLine(assemblerLine);
    out << "(0x" << std::setw(3) << std::setfill('0') << std::hex
        << assemblerLine++ << std::dec << ") ";
    mi->writeOutReadable(out, mapping, printEnergy);
  }
  if (printEnergy) {
    cout << "Total TransitionEnergy [pJ] = " << totalEnergy << std::endl;
  }
}

void Program::printInstructionsCompilable(
    std::ostream &out, const VirtualRegisterMap *mapping) const {

  int assemblerLine = 0;
  for (auto it = begin(); it != end(); ++it) {
    auto mi = *it;
    (*it)->registerMO2AssemblerLine(assemblerLine);

    mi->writeOutCompilable(out, mapping);
  }
}

uint Program::getInstructionTransitions() const {
  int transitions = 0;
  const MO *previousMo0 = NULL; // instructions->getMI(0)->getOperation(0);
  const MO *previousMo1 = NULL; // instructions->getMI(0)->getOperation(1);
  for (int i = 0; i < size(); i++) {
    auto mo0 = getMI(i)->getOperation(0);
    auto mo1 = getMI(i)->getOperation(1);

    //            if(isLog(LOG_M_REG_ENERGY_DEBUG)){
    //                string moName = "NOP";
    //                if (mo0){
    //                    moName = mo0->getOperation()->getBaseName();
    //                }
    //                string previousMoName = "NOP";
    //                if(previousMo0) {
    //                    previousMoName =
    //                    previousMo0->getOperation()->getBaseName();
    //                }
    //                std::cout << previousMoName << " - "<< moName ;
    //
    //            }
    // check Null
    if (!mo0 or !previousMo0) {
      if (!(!mo0 and !previousMo0)) {
        transitions++;
      }
    } else if (mo0->getOperation()->getBaseName() !=
               previousMo0->getOperation()->getBaseName()) {
      transitions++;
    }

    if (mo0) {
      if (mo0->opLengthMultiplier() == 1) {
        //      if(isLog(LOG_M_REG_ENERGY_DEBUG)) {
        //      moName = "NOP";
        //      if (mo1){
        //        moName = mo1->getOperation()->getBaseName();
        //      }
        //      previousMoName = "NOP";
        //      if(previousMo1) {
        //        previousMoName = previousMo1->getOperation()->getBaseName();
        //      }
        //
        //      std::cout << previousMoName << " - "<< moName ;
        //      }
        // check Null
        if (!mo1 or !previousMo1) {
          if (!(!mo1 and !previousMo1)) {
            transitions++;
          }
        } else if (mo1->getOperation()->getBaseName() !=
                   previousMo1->getOperation()->getBaseName()) {
          transitions++;
        }
        //      std::cout << "           " << transitions << std::endl;
        previousMo1 = mo1;
      }
    } else {
      // check Null
      if (!mo1 or !previousMo1) {
        if (!(!mo1 and !previousMo1)) {
          transitions++;
        }
      } else if (mo1->getOperation()->getBaseName() !=
                 previousMo1->getOperation()->getBaseName()) {
        transitions++;
      }
      previousMo1 = mo1;
    }

    previousMo0 = mo0;
  }
  return transitions;
}

void Program::calculateTransitionEnergy(VirtualRegisterMap *mapping) {
  TransitionEnergyEstimator energyEstimator;
  for (auto mis : _mis) {
    energyEstimator.push(mis, mapping);
  }
  cumulativeTransitionEnergy = energyEstimator.getTransitionEnergy();
}