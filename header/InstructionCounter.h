// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_INSTRUCTIONCOUNTER_H
#define SCHEDULER_INSTRUCTIONCOUNTER_H

class InstructionCounter {

public:
  static InstructionCounter &getInstance() {
    static InstructionCounter instance; // Guaranteed to be destroyed.
    // Instantiated on first use.
    return instance;
  }

  void increment() { instructionCount += 1; }
  int getCount() { return instructionCount; }

private:
  InstructionCounter() {} // Constructor? (the {} brackets) are needed here.
  int instructionCount = 0;

  // C++ 11
  // =======
  // We can use the better technique of deleting the methods
  // we don't want.
public:
  InstructionCounter(InstructionCounter const &) = delete;
  void operator=(InstructionCounter const &) = delete;

  // Note: Scott Meyers mentions in his Effective Modern
  //       C++ book, that deleted functions should generally
  //       be public as it results in better error messages
  //       due to the compilers behavior to check accessibility
  //       before deleted status
};

#endif // SCHEDULER_INSTRUCTIONCOUNTER_H
