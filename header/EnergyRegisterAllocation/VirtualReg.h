// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_VIRTUALREG_H
#define SCHEDULER_VIRTUALREG_H

#include "virtual_reg.h"
#include <memory>
#include <sstream>

namespace portOptReg {
class VirtualRegs {

public:
  char32_t virtualRegister;
  int positionFirstCreated;
  int positionLastUsed;
  int temp;

  VirtualRegs(char32_t virtualRegs, int miStartPos, int MiEndPosition,
              int temp) {
    virtualRegister = virtualRegs;
    positionFirstCreated = miStartPos;
    positionLastUsed = MiEndPosition;
    this->temp = temp;
  }

  // Constructor for Physical Register
  VirtualRegs(char32_t physicalRegister)
      : virtualRegister(physicalRegister), positionFirstCreated(-1),
        positionLastUsed(-1), temp(-1) {}

  VirtualRegs(const VirtualRegs &virtRegs) {
    virtualRegister = virtRegs.virtualRegister;
    positionFirstCreated = virtRegs.positionFirstCreated;
    positionLastUsed = virtRegs.positionLastUsed;
    temp = virtRegs.temp;
  }

  VirtualRegs(const std::shared_ptr<VirtualRegs> original) {
    virtualRegister = original->virtualRegister;
    positionFirstCreated = original->positionFirstCreated;
    positionLastUsed = original->positionLastUsed;
    temp = original->temp;
  }

  int const getStart() const { return positionFirstCreated; }

  int const getEnd() const { return positionLastUsed; }

  const char32_t getVirtualRegister() const { return virtualRegister; }

  const int getTempVirtualRegister() const { return temp; }

  bool operator!=(const VirtualRegs &other) const {
    return getVirtualRegister() != other.getVirtualRegister();
  }

  bool overlap(std::shared_ptr<VirtualRegs> virtualReg) {
    if (registers::getName(getVirtualRegister()) == "VxR136" or
        registers::getName(getVirtualRegister()) == "VxR137") {
      if (registers::getName(virtualReg->getVirtualRegister()) == "VxR136" or
          registers::getName(virtualReg->getVirtualRegister()) == "VxR137") {
        int n = 3;
      }
    }
    // No usage of the register. therefore no overlap available
    if (virtualReg->positionFirstCreated == virtualReg->positionLastUsed) {
      return false;
    }
    if (positionFirstCreated == positionLastUsed) {
      return false;
    }

    bool startedBefore = virtualReg->getStart() < positionFirstCreated;
    if (startedBefore) {
      return virtualReg->getEnd() > //>=
             positionFirstCreated;  // and virtualReg->getEnd()
                                    // <=positionLastUsed ;
    }
    bool startedAfter = virtualReg->getStart() > positionLastUsed;
    if (startedAfter) {
      return false;
    }
    // if def and use in same MI, no overlap
    if (positionLastUsed == virtualReg->positionFirstCreated) {
      return false;
    }
    return true;
    //    return ((virtualReg->getStart() >= positionFirstCreated and
    //    virtualReg->getStart() <= positionLastUsed) or (virtualReg->getEnd()
    //    >= positionFirstCreated and virtualReg->getEnd()
    //    <= positionLastUsed )) or
    //        ((positionFirstCreated >= virtualReg->getStart()  and
    //        positionLastUsed  <= virtualReg->getStart()) or
    //        (positionFirstCreated  >= virtualReg->getEnd() and
    //        positionLastUsed  <=  virtualReg->getEnd()));
  }

  std::string toString() {
    std::stringstream ss;
    ss << temp << " (" << registers::getName(virtualRegister) << " )";
    return ss.str();
  }

  std::string toGraph() {
    std::stringstream ss;
    ss << graphID() << " [label=\"" << toString() << "\"];" << std::endl;
    return ss.str();
  }

  std::string graphID() {
    std::stringstream ss;
    ss << registers::getName(virtualRegister);
    return ss.str();
  }

  bool isPhysical() { return not registers::isVirtualReg(virtualRegister); }

  bool isVirtual() { return registers::isVirtualReg(virtualRegister); }

  bool operator==(const VirtualRegs &other) {
    return virtualRegister == other.virtualRegister;
  }
};
} // namespace portOptReg

#endif // SCHEDULER_VIRTUALREG_H
