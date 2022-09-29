// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "PhysicalRegisterModel.h"

void PhysicalRegisterModel::calculatePhysicalTransitions() {
  for (int i = 0; i < mi0.getSlots(); i++) {
    serdes::MikroOperation mo = mi0.getMikroOperation(i);
    for (int j = 0; j < mo.getRegisterSet().getReadOperands(); j++) {
      if (mo.getRegisterSet().isReadRegister(j)) {
        int regs = mo.getRegisterSet().getReadRegister(j);
        int banks = mo.getRegisterSet().getReadBank(j);
        findPhysicalPort(regs, banks);
      }
    }
  }
}

void PhysicalRegisterModel::findPhysicalPort(int reg, int bank) {
  vector<int> registerFile = usedPorts[bank];
  for (int i = 0; i < *readPorts; i++) {
    if (registerFile[i] == notUsedReadPort) {
      usedPorts[bank][i] = reg;
      return;
    }
  }
}

void PhysicalRegisterModel::initUsedPorts() {
  for (int i = 0; i < numRegisterFiles; i++) {
    vector<int> v;
    for (int j = 0; j < *readPorts; j++) {
      v.push_back(notUsedReadPort);
    }
    usedPorts.push_back(v);
  }
}