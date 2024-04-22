// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef FUNCTIONALUNIT_H
#define FUNCTIONALUNIT_H 1

#include "MO.h"
#include "operation.h"
#include "rapidxml-1.13/rapidxml.hpp"
#include <set>
#include <vector>

void initFU(rapidxml::xml_node<> *start);
/** \brief deletes all functional Units.
 *
 * Since a single VectorUnit can contain multiple instances of the same
 * FunctionalUnit and different VectorUnit can also contain the same
 * FunctionalUnit. the destructor for all FU is put into a global function.
 */
void deleteFUs();

/** @brief Representation of a functional unit
 *
 *
 */
class FunctionalUnit {
private:
  static int counter;
  // define the default directions for operations
  char dir[MAXARGNUMBER];
  /** all the operations this FU can execute */
  std::vector<Operation *> OP;
  std::set<std::string> OPNames;
  char *UnitName;
  /** Extracts one kind of suffix out of the xml node and adds them to the
   * underlying parts */
  void getSuffixesHelper(rapidxml::xml_node<> *root, char *name, char *message);
  Operation *NOP;
  int id;

public:
  /** @brief Parses and initializes a given xml_node into a FunctionalUnit.
   *
   * @sa Assembler()
   */
  FunctionalUnit(rapidxml::xml_node<> *start);
  ~FunctionalUnit();
  /** @returns the NOP Operation if present
   *
   * searches for a NOP-Operation an returns the Pointer to it.
   *
   * @return Operation* if this FunctionalUnit can execute NOP
   * @return NULL otherwise
   */
  Operation *getNOPOperation();
  /** @brief test if this FunctionalUnit can execute the NOP Operation
   *
   * NOP is the only Operation every Processor must be able to execute, since it
   * is scheduled, if nothing else can be scheduled. It must always be possible
   * to execute NOP on an IssueSlot.
   * @return true If it is possible to execute NOP on this FunctionalUnit.
   */
  bool hasNOP();
  /** @brief Returns the printable Name of the FunctionalUnit.
   *
   */
  char *getFUname();
  /** @brief Checks if the given Operation can be executed by this
   * FunctionalUnit.
   *
   * @return true If it is possible to execute the Operation.
   */
  bool isExecuteable(MO *op);
  /** @brief Checks if the FunctionalUnit contains given Operation.
   *
   * @return true If the FunctionalUnit contains the Operation.
   */
  bool contains(string op);
  /** @brief All Operations this FunctionalUnit can execute
   *
   */
  std::vector<Operation *> *getAllOperations() { return &OP; }
  void getSuffixes(rapidxml::xml_node<> *root);
  void addSuffix(Operation *op);

  void extractContent(rapidxml::xml_node<> *start);

  friend ostream &operator<<(ostream &out, const FunctionalUnit &p) {
    out << "--->Functional Unit - " << p.UnitName << std::endl;
    //			for(std::vector<Operation*>::const_iterator u =
    //(p.OP).begin();u!=(p.OP).end();u++) {
    // ASS_DEBUG_COUT("---->"
    //<<
    //(**u) << std::endl);
    //			}
    return out;
  }

  int getID() const { return id; }

  static int getNumFU() { return counter; }

  int getLatency() const {
    if (OP.empty()) {
      throw std::runtime_error(
          "Empty Functional Unit detected. Please check your config file.");
    }
    return OP[0]->getLatency();
  }
};

/** @brief Returns the FunctionalUnit with the given Name
 *
 * searches on all the Functional Units for the given Name and returns it.
 * @return A Pointer to the searched FunctionalUnit
 * @return NULL if it can not be found
 */
FunctionalUnit *getFU(char *FUname);

void printUnits();

#endif
