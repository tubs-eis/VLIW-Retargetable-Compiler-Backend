// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "AssemblerParser/IssueSlotChecker.h"
#include <MI.h>

IssueSlotChecker::IssueSlotChecker(const char *codeLine) {
  this->codeLine = codeLine;
  run();
}

IssueSlotChecker::~IssueSlotChecker() {}

bool IssueSlotChecker::errorOccured() { return error; }

int IssueSlotChecker::getActiveIssueSlotBitVector() {
  unsigned long longValue = slots.to_ulong();
  int value = (int)longValue;
  return value;
}

bool IssueSlotChecker::containsPartingBit() { return partingBit; }

unsigned int IssueSlotChecker::getNextPosition() { return nextPosition; }

void IssueSlotChecker::run() {
  if (containsIssueSlot()) {
    extractIssueSlotInformation();
  }
}

void IssueSlotChecker::extractIssueSlotInformation() {
  nextPosition = 1;
  while (!isspace(codeLine[nextPosition])) {
    unsigned int number = getDigit(codeLine, nextPosition);
    extractPartingInformation(number);
    extractIssueSlots(number);
    nextPosition++;
  }
  findNextWordBeginning();
}

bool IssueSlotChecker::containsIssueSlot() {
  return codeLine[0] == ':' && isdigit(codeLine[1]);
}

void IssueSlotChecker::extractPartingInformation(unsigned int number) {
  partingBit = number == 9;
}

void IssueSlotChecker::extractIssueSlots(unsigned int number) {
  if (number >= MI::getNumberIssueSlots()) {
    LOG_OUTPUT(LOG_M_ALWAYS,
               "there are only %d issue slots. Constraint %d is not valid!\n",
               MI::getNumberIssueSlots(), number);
    error = true;
  } else {
    slots[number] = 1;
  }
}

void IssueSlotChecker::findNextWordBeginning() {
  while (isspace(codeLine[nextPosition])) {
    nextPosition++;
  }
}
