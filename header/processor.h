// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef PROCESSOR_H
#define PROCESSOR_H 1

class VectorUnit;
class MI;
#include "operation.h"
#include "rapidxml-1.13/rapidxml.hpp"
#include <map>
#include <vector>

/** @brief Representation of a Processor
 *
 * A representation of a Processor as defined in the configuration file read in
 * by Assembler.
 *
 */
class Processor {
public:
  struct DMAAddrConfig {
    uint32_t startAddr;
    uint32_t endAddr;
    uint32_t CTRL_START_ADDR;
    uint32_t RESET_ADDR;

    bool isDMAStore(MO *op);
    bool isDMACtrlAddr(MO *op);
  };

private:
  /** The minimum number of bytes the binary file should have. */
  int binSize;
  string name;
  /** All VectorUnit this Processor has. */
  std::vector<VectorUnit *> Units;
  std::map<std::string, Operation *> *allOps;
  int issueSlots;
  bool X2Support;
  Operation *MV;
  Operation *OR;
  Operation *NOP;
  DMAAddrConfig *dmaAddrConfig;
  void initialize(char *configFile);
  void readDMAAddrRange(rapidxml::xml_node<> *node);

public:
  /** \brief Initializes an Processor with the configuration file given by the
   * filename
   *
   * The configuration file must be a valid XML file.
   *
   * @param [in] configFile The name of the configuration file.
   *
   */
  Processor(char *configFile);
  ~Processor();
  /** @brief checks if the Processor is valid.
   *
   * A Processor is valid if every VectorUnit is valid.
   * A VectorUnit is valid if it has as much FunctionalUnit with the possibility
   * to execute NOP as it has issue slots.
   * @return true if the criteria above are meet.
   * @return false if one is not meet.
   */
  bool isValid();
  /** @brief Returns the size of the binary file to produce
   * @return The binary file size in bytes.
   */
  int getBinSize() { return binSize; }
  /** @brief Checks if it is possible to run the given MI on this processor.
   *
   * Checks for read/write ports and if the functional units are available.
   * @note
   * makes @b NO checks if a mapping for virtual registers is possible
   *
   * it also does @b NOT check if the instructions have dependencies
   *
   * @param [in] ins A Microinstruction which could have empty spots.
   * @return @b true If it possible to execute this instruction
   * @return @b false If one criteria is not met
   */
  string getProcessorName() { return name; }
  bool isExecuteable(MI *ins) const;
  bool isNotParallelMemorySystemAccess(MI *ins) const;

  bool isNotParallelMOs(const MO *mo1, const MO *mo2) const;
  /** @brief Replace empty slots in the MI with NOP operations.
   *
   * NOP operations are generated from the VectorUnit's and put in for every
   * empty MO.
   *
   * @param [inOut] ins The MI where to replace the empty slots.
   */
  void replaceNull(MI *ins);
  /** @brief Returns the total number of issue slots for this Processor
   *
   * The number of issue slots of all vector units are summed up and returned.
   * @return The total number of issue slots.
   */
  int getIssueSlotNumber() const;
  /** @brief Returns a vector with all Operation's this Processor could execute.
   *
   * The vector is generatet on initialisation and not altered afterwards.
   * All possible suffixes are also in this vector.
   * @return A vector with all Operation
   */
  /** @brief Returns the Operation with the given name.
   *
   * Searches all available Operations to find the one witch matches the given
   * name.
   * @return The Operation with the given name.
   * @return NULL if no operaton ist found.
   */
  Operation *getOperation(char *Name) {
    return getOperation(std::string(Name));
  }
  Operation *getOperation(std::string Name);
  Operation *getMVOperation() { return MV; }
  Operation *getOROperation() { return OR; }

  DMAAddrConfig *getDMAConfig();

  /** @brief returns the version of the MAC instruction.
   *
   * @return true the mac uses two registers in the same file
   * @return false the mac uses two registers from two parallel registers
   */
  bool isX2Supported() { return X2Support; }
  /** for nice printing out*/
  friend ostream &operator<<(ostream &out, const Processor &p) {
    out << "tukuturi-processor" << std::endl;
    for (std::vector<VectorUnit *>::const_iterator u = (p.Units).begin();
         u != (p.Units).end(); u++) {
      out << (*u);
    }
    return out;
  }

  std::string convertFU2CSV() const;
};

#endif
