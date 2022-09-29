// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_INSTRUCTIONMODEL_H
#define SCHEDULER_INSTRUCTIONMODEL_H

#include "Program.h"
#include <string>

class InstructionModel {
private:
  std::map<std::string, double> instructionEnergy[2];
  bool importSuccess = false;

  InstructionModel();
  InstructionModel(InstructionModel const &);
  void operator=(InstructionModel const &);

  void importModel();

public:
  double generateInstructionEnergy(Program *p);

  static InstructionModel &getInstance() {
    static InstructionModel instance;
    return instance;
  }
};

#endif // SCHEDULER_INSTRUCTIONMODEL_H
