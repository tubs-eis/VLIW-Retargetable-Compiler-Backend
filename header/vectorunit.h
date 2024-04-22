// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef VECTORUNIT_H
#define VECTORUNIT_H 1

#include "MI.h"
#include "MO.h"
#include "functionalunit.h"
#include "rapidxml-1.13/rapidxml.hpp"

/** @brief Representation of a Vector Unit
 *
 */
class VectorUnit {
private:
  /** bitcoded the registers this VectorUnit has access to
   *
   * 0b0010 means this VectorUnit has access to register 1 (count starts at 0)
   */
  char32_t regis;
  /** the number of issue slots it has to execute operations */
  int numIssueSlots;
  /** a vector of underlying parts */
  std::vector<FunctionalUnit *> Units;
  /** Extracts one kind of suffix out of the xml node and adds them to the
   * underlying parts */
  void getSuffixesHelper(rapidxml::xml_node<> *root, char *name, char *message);
  /** Extracts all kind of suffixes out of the xml node using getSuffixHelper
   * and adds them to the underlying parts */
  void getSuffixes(rapidxml::xml_node<> *root);
  int countInstructions(MO **ins);
  bool isExecuteableShort(MO **ins);
  int *functionalUnits;

public:
  /** @brief Initializes a VectorUnit with the XML-representation given by the
   * node. the XML node and all subnode are parsed and all information are
   * extracted.
   */
  VectorUnit(rapidxml::xml_node<> *start);
  ~VectorUnit() { free(functionalUnits); }
  /** @brief adds an suffix to all its operations and the operations of its
   * underlying parts when called, the given Operation is added as a possible
   * Suffix to all Operations of this VectorUnit if the OPCodes of the
   * Operations have differences, the suffix is not added.
   * @param [in] op an Suffix to add to all of its Operations.
   */
  void addSuffix(Operation *op);
  /** @brief checks if the VectorUnit is valid.
   * a VectorUnit is valid if it has as much FunctionalUnit with the possibility
   * to execute NOP as it has issue slots.
   * @return true if the criteria above are meet.
   * @return false if one is not meet.
   */
  bool isValid();
  /** @brief checks if it is possible to run the given Array of MO with this
   * VectorUnit checks if all the addressed registers can be accessed from this
   * VectorUnit checks for every MO if there is an free FunctionalUnit in this
   * VectorUnit
   *
   *
   * the size of the array must be the number of IssueSlots this VectorUnit has.
   * @sa getNumIssueSlots()
   * @note
   * makes @b NO checks if a mapping for virtual registers is possible
   *
   * it also does @b NOT check if the instructions have dependencies
   *
   * @param [in] An Array of MO which could have NULL entries.
   * @return @b true if it possible to execute this instruction
   * @return @b false if one criteria is not met
   */
  bool isExecuteable(MO **ins);
  /** @brief returns the number of issue slots.
   * @return the number of issue slots.
   */
  int getNumIssueSlots() const { return numIssueSlots; }
  /** @brief returns a list of all operations this VectorUnit can execute
   *
   * the list is generated from the FU everytime from the underlying
   * FunctionalUnit's
   * @return list of all operation for this vector unit.
   */
  vector<Operation *> *getAllOperations();
  /** for nice printing out */
  friend ostream &operator<<(ostream &out, const VectorUnit &p) {
    out << "->Vector unit" << std::endl;
    for (std::vector<FunctionalUnit *>::const_iterator u = (p.Units).begin();
         u != (p.Units).end(); u++) {
      out << (*u);
    }
    return out;
  }

  FunctionalUnit *getFUbyID(int id) {
    for (auto it = Units.begin(); it != Units.end(); ++it) {
      if ((*it)->getID() == id)
        return *it;
    }
    return 0;
  }

  // return Units
  const std::vector<FunctionalUnit *> *getUnits() const { return &Units; }
};

#endif
