// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef REGISTER_H
#define REGISTER_H 1

#include "global.h"
#include "rapidxml-1.13/rapidxml.hpp"
#include "utility.h"
#include <string>
#include <vector>

#if CHECK_RA_TIMING
struct RA_Timings {
  util::Timer heuristicTimer;
  unsigned int heuristicCount;
  util::Timer geneticTimer;
  unsigned int geneticCount;
  util::Timer gen_CompleteTimer;
  unsigned int gen_CompleteCount;

  RA_Timings() : heuristicCount(0), geneticCount(0), gen_CompleteCount(0) {}
  void reset() {
    heuristicTimer.reset();
    heuristicCount = 0;
    geneticTimer.reset();
    geneticCount = 0;
    gen_CompleteTimer.reset();
    gen_CompleteCount = 0;
  }
};

extern RA_Timings ra_timings;

void printRATimings();
#endif

class MO;

/** \brief Namespace for registers.
 *
 *
 */
namespace registers {
// #define ass_reg_t char32_t
/**
 * Bit Vector containing blocked register.
 */
#define ass_reg_t long long unsigned int

/** \brief Initializes the registers.
 *
 * Needs a pointer to an XML-structure defining the registers. */
void initRegister(rapidxml::xml_node<> *start);

/** \brief Parses a register and returns a pointer to the field after the parsed
 * text.
 *
 * The register could be followed by any text. As soon as a register is parsed,
 * the resulting ID is generated and a pointer to the text after the register is
 * returned. This function is intended to be used in parsing lines. This way a
 * line can be parsed, by just calling the parser and if they parse anything,
 * their result is stored in the ID and the return text can be passed to the
 * next function.
 *
 * @param [in] line The name of the register to parse.
 * @param [out] ID The ID of the register.
 * @return Pointer to the text behind the register.
 * */
const char *parseRegister(const char *line, char32_t *ID);

/** \brief Returns the name of the register.
 *
 * Returns a printable register name
 * @param [in] ID The ID of a register.
 * @return A human readable register name.
 */
std::string getName(char32_t ID);

/** \brief Binary representation of the register
 *
 * The bits are in the lower bits. the other are filled with zeros.
 * @param [in] ID The ID of a register.
 * @return Binary representation of the register.
 */
char32_t getBinary(char32_t ID);

/** \brief Returns the register file number of the given register.
 *
 * For Special registers their register number or 0 is returned.
 * Virtual registers return -1
 * @param [in] ID The ID of a register.
 * @return The register file number or -1.
 */
int getRegFile(char32_t ID);
/** \brief Returns the register number of the given register.
 *
 * Virtual registers and special registers return -1.
 * Only hardware registers return their number.
 * V1R2 would return 2
 * @param [in] ID The ID of a register.
 * @return The register number
 */
int getRegNumber(char32_t ID);

int getFirRegNumber(char32_t ID);
/** \brief Returns the register number of the given virtual register.
 *
 * Special registers and hardware registers return a negative number.
 *
 * @param [in] ID The ID of a register.
 * @return The virtual register number.
 */
int getVirtualRegisterNumber(char32_t ID);
/** \brief Returns the number of read ports.
 *
 * Returns the number of read port for the given register file number.
 *
 * @param [in] regNumber Register file number.
 * @return Number of read port for the given register file.
 */
unsigned int getReadPorts(int regNumber);
/** \brief Returns the number of write ports.
 *
 * Returns the number of write port for the given register file number.
 *
 * @param [in] regNumber Register file number.
 * @return Number of write port for the given register file.
 */
unsigned int getWritePorts(int regNumber);

/** \brief Returns the number of register files. */
unsigned int getNumRegisterFiles();
/** \brief Returns the size of one register file */
unsigned int getNumRegister1RF();
/** \brief Creates an ID for a new register.
 *
 * Returns the ID of the register given by the parameters.
 * If the given register file is negative a virtual register is created.
 *
 * @param [in] regFile The register file number or -1.
 * @param [in] regNumber The number of the register in the given register file.
 * @return The ID of the given paramter.
 */
char32_t createRegister(int regFile, int regNumber);

/** \brief Returns the register file number of the given register.
 *
 * For Special registers their register number or 0 is returned.
 * Virtual registers return -1
 * @param [in] ID The ID of a register.
 * @return The register file number or -1.
 */
void getPhysicalRegister(char32_t ID, int *regFile, int *regNumber);

/** \brief Checks two register for equality.
 *
 * Since FIR-registers decode in their ID the art of addressing them,
 * a simple == is not sufficient and this function is a global solution
 * to check if two registers are equal.
 *
 * the Programmer first has to make sure, that the parameters are ID's for
 * registers and not labels or immediate.
 *
 * @param [in] first The first register
 * @param [in] second The second register
 * @return true if decode the same hardware resource.
 */
bool equals(char32_t first, char32_t second);

/** \brief Checks if the register is a Condition Select register */
bool isCondSel(char32_t ID);
/** \brief Checks if the register is a FIR register. */
bool isFirReg(char32_t ID);
/** \brief Checks if the register is a FIR register and addressed indirect. */
bool isIndFirReg(char32_t ID);

bool isOffsetReg(char32_t ID);

bool isFirOffsetReg(char32_t ID);

bool isFirDecReg(char32_t ID);

bool isFirIncReg(char32_t ID);

bool isFirWriteReg(char32_t ID);
bool isFirReadReg(char32_t ID);
bool directFir(char32_t ID);

bool isFlag(char32_t ID);
bool isSpecialRegNoFir(char32_t ID);

bool isVirtualReg(char32_t const ID);

bool isDummyRegister(char32_t ID);

int getLastDummyRegNumber();

int getDummyUsageTime();

int getFirWriteLatency();

void setFirWriteLatency(int latency);

int getParallelExecutableFirBlockSize();

int getNumberOfCondselRegister();

bool isX2Arg(MO *mo, int index);
bool isValidX2Pair(MO *mo, int index);

bool isPhysicalRegister(char32_t ID);

bool isBlockedRegister(const ass_reg_t *blocked);

int getDotID(const char32_t arg);
int getDotOffset();

} // namespace registers
extern unsigned int numRegisterFiles;
extern unsigned int numRegisters;
extern int *writePorts;
extern int *readPorts;

#endif
