// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "InstructionModel.h"
#include "CSVReader.h"
#include "MI.h"

InstructionModel::InstructionModel() { importModel(); }

void InstructionModel::importModel() {
  if (params.instructionModelFile != "") {
    auto import = readFile(params.instructionModelFile);
    for (int i = 1; i < import.size(); i++) {
      int issueslot = stoi(import[i][0]);
      string key = import[i][1] + "-" + import[i][2];
      string reverseKey = import[i][2] + "-" + import[i][1];
      double energy = stod(import[i][3]);
      double energyReverse = stod(import[i][4]);
      //      cout << key << " - " << reverseKey << endl;
      instructionEnergy[issueslot][key] = energy;
      instructionEnergy[issueslot][reverseKey] = energyReverse;
    }
    importSuccess = true;
  } else {
    importSuccess = false;
  }
}

std::string getBaseName(MI *mi, int issueSlot) {
  if (mi->getOperation(issueSlot)) {
    string base = mi->getOperation(issueSlot)->getOperation()->getBaseName();
    if (base == "MVI") {
      return "MV";
    } else {
      if (base.back() == 'I') {
        return base.substr(0, base.size() - 1);
      }
      return base;
    }
  } else {
    return "NOP";
  }
}

double InstructionModel::generateInstructionEnergy(Program *ins) {
  double totalEnergy = 0;
  if (not importSuccess) {
    return 0;
  }
  string issue0Operation, issue1Operation = "";
  for (int i = 0; i < ins->size(); i++) {
    MI *mi = ins->getMI(i);
    //    cout << mi->to_string() << endl;
    int issueSlots = mi->getNumberIssueSlots();
    string issue0 = getBaseName(mi, 0);
    string issue1 = getBaseName(mi, 1);
    if (issueSlots != 2) {
      throw "Instruction Model: unequal to 2 issue slots\n";
    }
    if (i > 0) {
      string key0 = issue0Operation + "-" + issue0;
      string key1 = issue1Operation + "-" + issue1;
      // todo: check if instruction in the model are missing
      if (issue0Operation != issue0 and
          instructionEnergy[0].find(key0) == instructionEnergy[0].end()) {
        throw logic_error("Connection " + key0 +
                          " missing in instruction Model-Issue 0");
      }
      if (issue1Operation != issue1 and
          instructionEnergy[1].find(key1) == instructionEnergy[1].end()) {
        throw logic_error("Connection " + key1 +
                          " missing in instruction Model-Issue 1");
      }
      totalEnergy += instructionEnergy[0][key0] + instructionEnergy[1][key1];
      //      std::cout << key0 << " " << instructionEnergy[0][key0]<< " ---- "
      //      << key1 << " " << instructionEnergy[1][key1] << endl;
    }
    issue0Operation = issue0;
    issue1Operation = issue1;
  }
  return totalEnergy;
}