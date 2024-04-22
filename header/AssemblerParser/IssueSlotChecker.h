// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef MOAI_ASM_ISSUESLOTCHECKER_H
#define MOAI_ASM_ISSUESLOTCHECKER_H

#include <Utility/HexUtility.h>
#include <bitset>
#include <locale>

class IssueSlotChecker {

public:
  IssueSlotChecker(const char *codeLine);

  ~IssueSlotChecker();

  bool errorOccured();

  int getActiveIssueSlotBitVector();

  unsigned int getNextPosition();

  bool containsPartingBit();

private:
  const char *codeLine;
  bool partingBit = false;
  bool error = false;
  std::bitset<9> slots;
  unsigned int nextPosition = 0;

  void run();

  bool containsIssueSlot();

  void extractPartingInformation(unsigned int number);

  void extractIssueSlots(unsigned int number);

  void findNextWordBeginning();

  void extractIssueSlotInformation();
};

#endif // MOAI_ASM_ISSUESLOTCHECKER_H
