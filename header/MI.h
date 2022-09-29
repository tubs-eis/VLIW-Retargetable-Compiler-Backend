// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#ifndef MI_H
#define MI_H
#include "MO.h"
#include "global.h"
#include "virtual_reg.h"
#include <array>
#include <ostream>

/** @brief Representation of a microinstruction with multiple MO to execute in
 * parallel
 *
 * MI are representet as a double linked tree structure, to represent the
 * multiple possibilities a List of MI can take.
 */
class MI {
private:
  MO **mos = 0;
  static int slotNumber;
  int id;
  double transitionEnergy = 0;

public:
  static int counter[MAX_THREAD_COUNT];
  /** @brief Creates a new empty MI.
   *
   * the upper MI is not informed about its new descendent. This has to be done
   * manually. Reason for this is, that sol you can easily try new MI's out and
   * check if they work, before you add them to the tree.
   * @note If no parameter is given, then the MI is initialised as a root, which
   * can not be printed out!
   *
   * @param up the Parent of this MI
   */
  MI();
  MI(MI *old);
  ~MI();
  int getID() { return id; }
  /** @brief adds an MO to this MI in the specified slot.
   *
   * tries to add a MO in the specified slot. If there is an Operation, or the
   * the slot is blocked by the multiple of a instruction in an previous slot,
   * then no Operation is performed and false is returned.
   * @return true On success
   */
  bool putOperation(MO *op, int slot);
  /** @brief Removes the Operation.
   *
   * if the given slot is emty, then nothing happens, otherwise the Operation is
   * removed from this MO.
   *
   * @param slot The slot where the Operation should be deleted.
   */
  void removeOperation(int slot);

  /** @brief Checks if an given slot is free
   * @param slot The slot to check for an operation
   * @return true If this is not blocked by any Operation
   */
  bool isFree(int slot);
  /** @brief Check, if there is a free issue slot in the MI.
   */
  bool hasFreeSlot() const;

  /** @brief Sets the total number of issue slots for all MI
   *
   * This should be done before any MI is initialized, since the Constructor
   * would fail if the number of slots is set to 0.
   *
   * @sa getNumberIssueSlots()
   * @param number The total number of issue slots for the Processor.
   */
  static void setNumberOfSlots(int number) { slotNumber = number; }

  /** @brief returns the total number of issue slots.
   *
   * @sa setNumberOfSlots();
   * @return The number of issue slots
   */
  static unsigned int getNumberIssueSlots() { return slotNumber; }

  void writeOutBin(std::ostream &out);
  void writeOutReadable(std::ostream &out,
                        const VirtualRegisterMap *mapping = 0,
                        bool printEnergy = false) const;
  void writeOutCompilable(std::ostream &out,
                          const VirtualRegisterMap *mapping = 0) const;
  std::string to_string(VirtualRegisterMap const *mapping = 0) const;

  MO **getOperations() { return mos; }
  MO **const getOperations() const { return mos; }

  MO const *getOperation(int index) const { return mos[index]; }
  MO *getOperation(int index) { return mos[index]; }

  void determineUsedWritePorts(int *usedWritePorts);

  std::string getString(VirtualRegisterMap const *map);

  /**
   *
   * @tparam T
   * @param map
   * @return physical read register in format issue{}_src{} = [ 0_0, 0_1, 1_0,
   * 1_1, 0_2, 0_3, 1_2, 1_3]
   */
  std::array<int, 8> getPhysicalRegister(const VirtualRegisterMap *map) const {

    std::array<int, 8> physicalRegs;
    for (int i = 0; i < 8; i++) {
      physicalRegs[i] = -1;
    }
    int i = 0;
    while (i < slotNumber) {
      if (!mos[i]) {
        ++i;
      } else {
        MO *mo = mos[i];
        int src0 = mo->getOperation()->getPhysicalReadPort(mo, 0, map);
        int src1 = mo->getOperation()->getPhysicalReadPort(mo, 1, map);
        int src2 = mo->getOperation()->getPhysicalReadPort(mo, 2, map);
        int src3 = mo->getOperation()->getPhysicalReadPort(mo, 3, map);
        // if not X2 or mac, src2 is read wrong (will be write port)
        if (src2 == -1 or src3 == -1) {
          src2 = -1;
        }
        //        cout << "src0 " << registers::getName(src0) << " src1 "
        //             << registers::getName(src1) << " src2 " <<
        //             registers::getName(src2)
        //             << " src3 " << registers::getName(src3) << "      ; ";
        if (not registers::isVirtualReg(src0) or
            not registers::isVirtualReg(src1)) {
          if (src0 > 64 or src1 > 64) {
            stringstream ss;
            ss << "VIRTUAL Register not found but Physical Register > 0: src0="
               << src0 << " src1=" << src1;
            throw runtime_error(ss.str());
          }
        }

        bool x2Operation = mo->isX2Operation();
        if (x2Operation)
          LOG_OUTPUT(LOG_M_SCHED,
                     "Warning! Tryping to predict Transition Energy "
                     "for an X2 operation.\n");
        bool dualWritePort = mo->getOperation()->isDoubleRegister(mo, 0);
        if (dualWritePort)
          LOG_OUTPUT(LOG_M_SCHED,
                     "Warning! Tryping to predict Transition Energy "
                     "for an dual Write operation (e.g. MAC).\n");

        physicalRegs[i * 2] = src0;
        physicalRegs[i * 2 + 1] = src1;
        physicalRegs[4 + i * 2] = src2;
        physicalRegs[4 + i * 2 + 1] = src3;

        i += mos[i]->opLengthMultiplier();
      }
    }
    //    std::cout << endl;
    return physicalRegs;
  }

  double getTransitionEnergy() const { return transitionEnergy; }

  void setTransitionEnergy(double energy) { transitionEnergy = energy; }

  /**
   * this function copies writeOutReadable functionality but registers MO IDs
   * with Assembler lines! If @writeOutReadable changes, this function has to
   * change too.
   * @param line Assembler Line.
   */
  void registerMO2AssemblerLine(int line);
};

#endif // MI_H
