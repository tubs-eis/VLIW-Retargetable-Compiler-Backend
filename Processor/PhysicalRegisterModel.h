// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_PHYSICALREGISTERMODEL_H
#define SCHEDULER_PHYSICALREGISTERMODEL_H

#include <register.h>

static int notUsedReadPort = -1;
class PhysicalRegisterModel {
private:
  //  serdes::MikroInstruction &mi0;
  //  vector<vector<int>> usedPorts;
  //
  //  void calculatePhysicalTransitions();
  //  void initUsedPorts();
  //  void findPhysicalPort(int regs, int bank);
  //
  // public:
  //  PhysicalRegisterModel(serdes::MikroInstruction &mi0) : mi0(mi0) {
  //    initUsedPorts();
  //    calculatePhysicalTransitions();
  //  }
  //
  //  const vector<vector<int>> &getUsedPorts() const { return usedPorts; }
};

#endif // SCHEDULER_PHYSICALREGISTERMODEL_H
