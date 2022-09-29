// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#include "MI.h"
#include "../export/ScheduledMOCharacteristics.h"
#include "MO.h"
#include "global.h"
#include "register.h"
#include <iostream>
#include <sstream>

int MI::slotNumber = 0;
int MI::counter[MAX_THREAD_COUNT];

MI::MI() {
  if (slotNumber == 0) {
    std::cerr << "There are no issue slots defined. Exiting" << std::endl;
    EXIT_ERROR
  }
  id = counter[CURRENT_THREAD_NUM]++;
  // the root node has no Operations.
  mos = NULL;
  mos = (MO **)malloc(sizeof(MO *) * slotNumber);
  memset(mos, 0, sizeof(MO *) * slotNumber);
}

MI::MI(MI *old) {
  if (slotNumber == 0) {
    std::cerr << "There are no issue slots defined. Exiting" << std::endl;
    EXIT_ERROR
  }
  id = counter[CURRENT_THREAD_NUM]++;
  // the root node has no Operations.
  mos = NULL;
  mos = (MO **)malloc(sizeof(MO *) * slotNumber);
  for (int i = 0; i < slotNumber; i++)
    mos[i] = old->mos[i];
}

void MI::removeOperation(int slot) {
  if (slot >= slotNumber || mos[slot] == NULL)
    return;
  // we have to use a temporary variable, otherwise there is a segfault trying
  // to access the slot after deleting it.
  int length = mos[slot]->opLengthMultiplier();
  for (int i = 0; i < length; i++)
    mos[slot + i] = NULL;
}

bool MI::putOperation(MO *op, int slot) {
  if (slot >= slotNumber)
    return false;
  if (slot + op->opLengthMultiplier() > slotNumber)
    return false;
  int i;
  for (i = 0; i < op->opLengthMultiplier(); i++) {
    if (mos[slot + i] != NULL)
      return false;
  }

  for (i = 1; i < op->opLengthMultiplier(); i++) {
    mos[slot + i] = (MO *)-1;
  }
  mos[slot] = op;
  return true;
}

void MI::determineUsedWritePorts(int *usedWritePorts) {
  for (unsigned int i = 0; i < registers::getNumRegisterFiles(); ++i)
    usedWritePorts[i] = 0;
  for (unsigned int slot = 0; slot < this->getNumberIssueSlots();) {
    MO *mo = this->getOperations()[slot];
    if (mo) {
      char32_t *args = mo->getArguments();
      for (int argIdx = 0; argIdx < mo->getArgNumber(); ++argIdx) {
        if (mo->isPhysicalWriteArg(argIdx)) {
          int regfile = registers::getRegFile(args[argIdx]);
          usedWritePorts[regfile]++;
        }
        if (mo->isPhysicalWriteArg(argIdx + 4)) {
          int regfile = registers::getRegFile(args[argIdx + 4]);
          usedWritePorts[regfile]++;
        }
      }
      slot += mo->opLengthMultiplier();
    } else
      ++slot;
  }
}

bool MI::hasFreeSlot() const {
  for (int i = 0; i < slotNumber; ++i)
    if (!mos[i])
      return true;
  return false;
}

bool MI::isFree(int slot) {
  if (slot < slotNumber)
    return (mos[slot] == NULL);
  return false;
}

void MI::writeOutBin(ostream &out) {
  int i = 0;
  while (i < slotNumber) {
    if (mos[i] == NULL) {
      i++;
      cerr << "null in MI::writeOutBin() this should not have happend" << endl;
      EXIT_ERROR;
      continue;
    }
    mos[i]->writeOutBin(out);
    // if it spans multiply issue slots, we should jump over them.
    i += mos[i]->opLengthMultiplier();
  }
}

void MI::writeOutReadable(ostream &out, const VirtualRegisterMap *mapping,
                          bool printEnergy) const {
  int i = 0;
  while (i < slotNumber) {
    if (mos[i] == NULL) {
      i++;
      //            cerr << "null in MI::writeOutReadable() this should not have
      //            happend" << endl; EXIT_ERROR;
      out << "        0 NOP                                                    "
             "          ;";
      // todo: put back as found up
      continue;
    }
    mos[i]->writeOutReadable(out, mapping);
    // if it spans multiply issue slots, we should jump over them.
    i += mos[i]->opLengthMultiplier();
    out << " ; ";
  }
  if (printEnergy) {
    out << " TransitionEnergy [pJ] = " << transitionEnergy << "  ";
  }
  out << std::endl;
}

void MI::registerMO2AssemblerLine(int line) {
  int i = 0;
  while (i < slotNumber) {
    if (mos[i] == NULL) {
      i++;
      continue;
    }
    ScheduledMOCharacteristics::getInstance().setID(mos[i]->getID(), line);
    i += mos[i]->opLengthMultiplier();
  }
}

void MI::writeOutCompilable(ostream &out,
                            const VirtualRegisterMap *mapping) const {
  int i = 0;
  while (i < slotNumber) {
    if (mos[i] == NULL) {
      i++;
      continue;
    }
    if (i > 0 and mos[i]->getOperation()->getName() != "NOP") {
      out << "\n";
    }
    if (mos[i]->getOperation()->getName() != "NOP") {
      out << ":" << i << " ";
    }
    mos[i]->writeOutCompilable(out, mapping);
    // if it spans multiply issue slots, we should jump over them.
    i += mos[i]->opLengthMultiplier();
  }
  out << std::endl;
}

std::string MI::to_string(VirtualRegisterMap const *mapping) const {
  stringstream sstr;
  int i = 0;
  while (i < slotNumber) {
    if (!mos[i]) {
      ++i;
      sstr << " NOP ";
      continue;
    }
    sstr << mos[i]->to_string(mapping);
    i += mos[i]->opLengthMultiplier();
    sstr << " ; ";
  }
  if (!params.powerOptimizationFile.empty()) {
    sstr << " TransitionEnergy [pj] = " << transitionEnergy << "  ";
  }
  return sstr.str();
}

MI::~MI() {
  if (mos != NULL) {
    for (int i = 0; i < slotNumber; ++i)
      if (mos[i] && (mos[i] != (MO *)-1) && (mos[i]->getLineNumber() == 0))
        delete mos[i];
    free(mos);
    mos = NULL;
  }
}

Program::~Program() {
  for (auto it = _mis.begin(); it != _mis.end(); ++it)
    delete (*it);
  _mis.clear();
}

// void Program::printInstructions(const Program *prog) {
//  if (!prog)
//    return;
//  for (auto it = prog->begin(); it != prog->end(); ++it) {
//    auto mi = *it;
//    mi->writeOutReadable(cout, 0);
//  }
//}
//
// void Program::printInstructions(const Program *prog,
//                                const VirtualRegisterMap *mapping) {
//  if (!prog)
//    return;
//  for (auto it = prog->begin(); it != prog->end(); ++it) {
//    auto mi = *it;
//    mi->writeOutReadable(cout, mapping);
//  }
//}

int Program::getMergeCount() {
  int count = 0;
  for (auto mi : _mis) {
    unsigned int slot = 0;
    auto ops = mi->getOperations();
    while (slot < mi->getNumberIssueSlots()) {
      if (ops[slot]) {
        count += ops[slot]->isX2Operation() ? 1 : 0;
        slot += ops[slot]->opLengthMultiplier();
      } else
        slot += 1;
    }
  }
  return count;
}

string MI::getString(VirtualRegisterMap const *map) {
  stringstream ss;
  for (int i = 0; i < slotNumber; i++) {
    if (mos[i]) {
      ss << mos[i]->to_string(map);
    }
  }
  return ss.str();
}