// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "register.h"
#include "global.h"
#include "operation.h"
#include "rapidxml-1.13/rapidxml.hpp"

#include <cstddef>
#include <iostream>
#include <map>
#include <string.h>
#include <string>
#include <vector>

#include <algorithm>
#include <cctype>
#include <functional>
#include <locale>

#include "SLM.h"

// trim from start
static inline std::string &ltrim(std::string &s) {
  s.erase(s.begin(),
          std::find_if(s.begin(), s.end(),
                       std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
  s.erase(std::find_if(s.rbegin(), s.rend(),
                       std::not1(std::ptr_fun<int, int>(std::isspace)))
              .base(),
          s.end());
  return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) { return ltrim(rtrim(s)); }

#define SPECIALOFFSET (1 << 16)
#define BITCODE 0xffff

using namespace std;
using namespace rapidxml;

unsigned int numRegisterFiles = 0;
unsigned int numRegisters = 0;
char *possibleVirtual;
int *writePorts;
int *readPorts;

int numberOfCondselRegister = 1;

int lastDummyRegister = -1;
int dummyUsageTime = 0;
int parallelExecutableFirBlockSize = 8;

map<char32_t, string> specialRegister;

int VUbit = -1;

int firWriteLatency = 3;

bool usedOffsetX2register = false;

void registers::setFirWriteLatency(int latency) { firWriteLatency = latency; }

int registers::getFirWriteLatency() { return firWriteLatency; }

int registers::getLastDummyRegNumber() { return lastDummyRegister; }

int registers::getDummyUsageTime() { return dummyUsageTime; }

bool registers::isVirtualReg(char32_t const ID) {
  return registers::getVirtualRegisterNumber(ID) >= 0;
}

bool registers::isDummyRegister(char32_t ID) {
  int regNumber = getRegNumber(ID);
  if (getLastDummyRegNumber() > 0 && regNumber > 0)
    return regNumber <= getLastDummyRegNumber();
  return false;
}

int registers::getParallelExecutableFirBlockSize() {
  return parallelExecutableFirBlockSize;
}

bool registers::isCondSel(char32_t ID) {
  if (ID < (1 << 16))
    return false;
  ID = ID & 4095;
  string condsel("CONDSEL");
  string acondsel("ALLCONDSEL");
  std::map<char32_t, string>::const_iterator x = specialRegister.find(ID);
  if (x != specialRegister.end()) {
    return condsel.compare(x->second) == 0 || acondsel.compare(x->second) == 0;
  }
  return false;
}

bool registers::isFlag(char32_t ID) {
  if (ID < (1 << 16))
    return false;
  ID = ID & 4095;
  string flags("FLAGS");
  string aflags("ALLFLAGS");
  std::map<char32_t, string>::const_iterator x = specialRegister.find(ID);
  if (x != specialRegister.end()) {
    return flags.compare(x->second) == 0 || aflags.compare(x->second) == 0;
  }
  return false;
}

bool registers::isFirReg(char32_t ID) {
  return (ID >= SPECIALOFFSET && (ID & 1 << 12)); // bitmask of FIR-Registers
  //	return (ID >= SPECIALOFFSET && (ID & 0x60 || !(ID & 0x38) || (ID &
  // 0x50))); // bitmask of FIR-Registers
}

/** Indirect addressing through FIR register */
bool registers::isIndFirReg(char32_t ID) {
  //	return (isFirReg(ID) && (ID & 0x38));
  return (isFirReg(ID) && (ID & 0x40));
  //	return (ID >= SPECIALOFFSET && ((ID & 0x20)|| (ID & 0x50))); // bitmask
  // of FIR-Registers
}

bool registers::isOffsetReg(char32_t ID) {
  return (ID >= SPECIALOFFSET && ((ID & 0x78) >> 3) == 0xB);
}

bool registers::isFirOffsetReg(char32_t ID) {
  return (isFirReg(ID) && ((ID & 0x78) >> 3) == 0xA);
}

bool registers::isFirDecReg(char32_t ID) {
  return (isFirReg(ID) && ((ID & 0x78) >> 3) == 0xC);
}

bool registers::isFirIncReg(char32_t ID) {
  return (isFirReg(ID) && ((ID & 0x78) >> 3) == 0xE);
}

/** Changing the address of a FIR register while using it for indirect
 * addressing. */
bool registers::isFirWriteReg(char32_t ID) {
  return (isFirOffsetReg(ID) || isFirIncReg(ID) || isFirDecReg(ID));
}

/** Indirect FIR access, which does not change FIR content. */
bool registers::isFirReadReg(char32_t ID) {
  return isIndFirReg(ID) && ((ID & 0x30) >> 4 == 0);
}

bool registers::directFir(char32_t ID) {
  return isFirReg(ID) && ((ID & 0x40) >> 4 == 0);
}

unsigned int registers::getWritePorts(int numRegister) {
  return writePorts[numRegister];
}

unsigned int registers::getReadPorts(int numRegister) {
  return readPorts[numRegister];
}

unsigned int registers::getNumRegisterFiles() { return numRegisterFiles; }

unsigned int registers::getNumRegister1RF() { return numRegisters; }

bool registers::isPhysicalRegister(char32_t ID) {
  return (registers::getRegFile(ID) != -1);
}

char32_t registers::createRegister(int regFile, int regNumber) {
  if (regFile < 0) {
    return (numRegisterFiles + 1) * numRegisters + regNumber;
  }
  return (regFile * numRegisters + regNumber);
}

void registers::getPhysicalRegister(char32_t ID, int *regFile, int *regNumber) {
  if (ID >= SPECIALOFFSET) {
    *regFile = -1;
    *regNumber = ID & BITCODE;
    return;
  }
  if (ID >= ((numRegisterFiles + 1) * numRegisters)) {
    *regFile = -1;
    *regNumber = ID - ((numRegisterFiles + 1) * numRegisters);
    return;
  }
  *regFile = ID / numRegisters;
  *regNumber = ID % numRegisters;
}

int registers::getDotID(const char32_t arg) {
  int regFile, regNumber;
  registers::getPhysicalRegister(arg, &regFile, &regNumber);
  return 32000 + regFile * numRegisters + regNumber;
}

int registers::getDotOffset() { return 32000; }

void inserte(char32_t value, char *bin, const string *s, int size, bool found) {
  string final;
  char temp[32];
  switch (*bin) {
  case 'x':
  case 'X':
    inserte(value * 2, (bin + 1), s, size * 2, true);
    inserte(value * 2 + 1, (bin + 1), s, size * 2 + 1, true);
    break;
  case '1':
    inserte(value * 2 + 1, ++bin, s, size, found);
    break;
  case '0':
    inserte(value * 2, ++bin, s, size, found);
    break;
  case 0:
    if (found) {
      sprintf(temp, "%d", size);

      size_t f = s->find("*");
      if (f != string::npos) {
        final = *s;
        final.replace(f, std::string("*").length(), temp);
      } else {
        final = *s + temp;
      }
    } else
      final = *s;
    LOG_OUTPUT(LOG_M_INIT_PROC, "Added special register %20s with value %6d\n",
               final.c_str(), value);
    if (!strncmp(final.c_str(), "FIR", 3))
      value |= 1 << 12;
    if (!strncmp(final.c_str(), "OFFSET_X2", 9))
      usedOffsetX2register = true;
    specialRegister.insert(std::pair<const char32_t, string>(value, final));
    break;
  default:
    break;
  }
}

void registers::initRegister(xml_node<> *start) {
  if (start == nullptr)
    return;
  xml_node<> *tmp = start->first_node("num-reg-files");
  if (tmp) {
    numRegisterFiles = atoi(tmp->value());
    writePorts = (int *)calloc(sizeof(int), numRegisterFiles);
    readPorts = (int *)calloc(sizeof(int), numRegisterFiles);
    for (uint i = 0; i < numRegisterFiles; i++) {
      writePorts[i] = -1;
      readPorts[i] = -1;
    }
    tmp = start->first_node("reg-file-size");
    numRegisters = atoi(tmp->value());
    if (numRegisters > 32 && sizeof(char32_t) == sizeof(ass_reg_t)) {
      fprintf(stderr, "There might be a problem with the register allocation. "
                      "Compile again with a bigger type for ass_reg_t in "
                      "global.h. Preferably long long int.\n");
    }
    for (tmp = start->first_node("num-write-ports"); tmp;
         tmp = tmp->next_sibling()) {
      if (strcmp(tmp->name(), "num-write-ports"))
        continue;
      xml_attribute<> *reg_at = tmp->first_attribute("regFile");
      if (reg_at)
        writePorts[atoi(reg_at->value())] = atoi(tmp->value());
      else {
        for (uint i = 0; i < numRegisterFiles; i++) {
          if (writePorts[i] == -1)
            writePorts[i] = atoi(tmp->value());
        }
      }
    }
    for (tmp = start->first_node("num-read-ports"); tmp;
         tmp = tmp->next_sibling()) {
      if (strcmp(tmp->name(), "num-read-ports"))
        continue;
      auto reg_at = tmp->first_attribute("regFile");
      if (reg_at)
        readPorts[atoi(reg_at->value())] = atoi(tmp->value());
      else {
        for (uint i = 0; i < numRegisterFiles; i++) {
          if (readPorts[i] == -1)
            readPorts[i] = atoi(tmp->value());
        }
      }
    }
  }
  tmp = start->first_node("VUbit");
  if (tmp) {
    VUbit = atoi(tmp->value());
  }
  tmp = start->first_node("fir-write-latency");
  if (tmp) {
    firWriteLatency = atoi(tmp->value());
  }
  tmp = start->first_node("dummy-usage-time");
  if (tmp) {
    dummyUsageTime = atoi(tmp->value());
  }
  tmp = start->first_node("dummy-num-reg");
  if (tmp) {
    lastDummyRegister = atoi(tmp->value()) - 1;
  }
  tmp = start->first_node("parallel-executable-fir-block-size");
  if (tmp) {
    parallelExecutableFirBlockSize = atoi(tmp->value());
  }
  tmp = start->first_node("special-regs");
  if (tmp) {
    for (xml_node<> *special = tmp->first_node("sreg"); special;
         special = special->next_sibling()) {
      string ss(special->value());
      const string s = trim(ss);
      char *binex = special->first_attribute("binary")->value();
      inserte(0, binex, &s, 0, false);
    }
    tmp = tmp->first_node("num-condsel-regs");
    if (tmp) {
      numberOfCondselRegister = atoi(tmp->value());
    }
  }
}

const char *registers::parseRegister(const char *line, char32_t *ID) {
  int i = 0;
  unsigned int regFile = 0;
  LOG_OUTPUT(LOG_M_PARSE, "parsing register %s\n", line);
  if (line[0] != 'V') { // special register
    string reg(line);
    size_t found = reg.find_first_of(", \n\t");
    if (found == string::npos)
      found = reg.size();
    reg = reg.substr(0, found);
    int y = 0;
    for (map<char32_t, string>::iterator it = specialRegister.begin();
         it != specialRegister.end(); ++it) {
      if (!strcmp((it->second).c_str(), reg.c_str())) {
        *ID = (1 << 20) + it->first;
        LOG_OUTPUT(LOG_M_PARSE, "Found special register: %s\n",
                   it->second.c_str());
        return line + found;
      }
      y++;
    }
    //    std::cerr << "could not parse the register: " << line << std::endl;
    //	EXIT_ERROR
    return nullptr;
  }
  if (line[1] == 'x') { // virtual register
    *ID = (numRegisterFiles + 1) * numRegisters;
  } else {
    regFile = (line[1] - '0');
    if (regFile >= numRegisterFiles) {
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "we don't have enough registers (%d) to adress this: %s\n",
                 numRegisterFiles, line);
      EXIT_ERROR
    }
    *ID = regFile * numRegisters;
  }

  if (line[2] == 'R') // we have a standard register.
    i = 3;
  // else
  { // we have some other kind of register.

    string reg(line);
    size_t found = reg.find_first_of(", \n\t");
    if (found == string::npos)
      found = reg.size();

    reg = reg.substr(2, found - 2);
    int y = 0;
    for (map<char32_t, string>::iterator it = specialRegister.begin();
         it != specialRegister.end(); ++it) {
      if (!strcmp((it->second).c_str(), reg.c_str())) {
        *ID = ((regFile + 1) << 16) + it->first;
        LOG_OUTPUT(LOG_M_PARSE, "Found special register: %s\n",
                   it->second.c_str());
        return line + found;
      }
      y++;
    }

    if (i == 0) {

      LOG_OUTPUT(LOG_M_ALWAYS, "could not parse the register: %s\n", line);
      EXIT_ERROR
      return NULL;
    }
  }
  unsigned int tmp = 0;
  while (line[i] >= '0' && line[i] <= '9') {
    tmp *= 10;
    tmp += line[i] - '0';
    i++;
  }

  if (tmp >= numRegisters && line[1] != 'x') {
    std::cerr << "the register " << line << " is bigger than it should be! ("
              << tmp << ">" << numRegisters - 1 << ")" << std::endl;
    EXIT_ERROR
    return NULL;
  }
  *ID += tmp;
  if (isDummyRegister(*ID)) {
    cerr << "Parsed register is a dummy register: " << line << endl;
    EXIT_ERROR;
    return NULL;
  }
  if (line[i] == '+')
    return parseRegister(line + i + 1, ID + 4);
  return line + i;
}

bool registers::isSpecialRegNoFir(char32_t ID) {
  return (ID >= SPECIALOFFSET) && !isFirReg(ID);
}

std::string registers::getName(char32_t ID) {
  if (ID >= SPECIALOFFSET) {
    map<char32_t, string>::iterator it;
    it = specialRegister.find(ID & BITCODE);
    if (it == specialRegister.end()) {
      //			LOG_OUTPUT(LOG_M_ALWAYS, "<could not find the
      // special Register for ID %d>\n", ID);
      char buffer[64];
      sprintf(buffer, "<ID=%d>", ID);
      return string(buffer);
      //			EXIT_ERROR
    }
    return it->second;
  }
  if (ID >= ((numRegisterFiles + 1) * numRegisters)) {
    char buffer[64];
    //    stringstream ss;
    //    ss << ID - ((numRegisterFiles + 1) * numRegisters);
    //    return ss.str();
    //    cout << "VxR" << ID - ((numRegisterFiles + 1) * numRegisters) << endl;
    sprintf(buffer, "VxR%d", ID - ((numRegisterFiles + 1) * numRegisters));
    return std::string(buffer);
  }
  char buffer[64];
  int v = ID / numRegisters;
  int reg = ID % numRegisters;
  sprintf(buffer, "V%dR%d", v, reg);
  return std::string(buffer);
}

char32_t registers::getBinary(char32_t ID) {
  if (ID >= SPECIALOFFSET) {
    if (registers::isFirReg(ID))
      return (ID | 0x40) & 0xff;
    else
      return ID & 0xff;
  }
  if (ID >= ((numRegisterFiles + 1) * numRegisters)) {
    cerr << "Could not write out register " << registers::getName(ID) << ". ";
    cerr << "You must first convert virtual registers to real before writing "
            "out binary!"
         << endl;
    EXIT_ERROR;
    return 0;
  }
  char32_t ret = ID % numRegisters;
  ret |= (ID / numRegisters) % 2 << VUbit;
  return ret;
}

bool registers::equals(char32_t first, char32_t second) {
  if (first == second)
    return true;
  if (isFirReg(first) && isFirReg(second)) {
    return ((first >> 16) == (second >> 16) &&
            (first & 0x87) == (second & 0x87));
  }
  return false;
}

int registers::getRegFile(char32_t ID) {
  if (ID >= SPECIALOFFSET) {
    //		if ((ID >> 20) == 1)
    //			return -1;
    //		return ((ID >> 16) - 1); // ! (ID >> 16) - 1 ist nicht RegFile,
    // sondern Vector Unit (see parseRegister)
    return -1;
  }
  if (ID >= ((numRegisterFiles + 1) * numRegisters)) {
    return -1; // if we have virtual registers, they don't care.
  }
  return ID / numRegisters;
}

int registers::getRegNumber(char32_t ID) {
  if (ID >= SPECIALOFFSET) {
    return -1;
  }
  if (ID >= ((numRegisterFiles + 1) * numRegisters)) {
    return -1; // if we have virtual registers, they don't care.
  }
  return ID % numRegisters;
}

int registers::getFirRegNumber(char32_t ID) {
  // FIR reg number is composed of the 'X'-bits from the binary coding
  if (ID >= SPECIALOFFSET &&
      (ID & (1 << 12))) { // FIR registers have bit 12 set (see parseRegsiter)
    return ((ID & 0x80) >> 4) | (ID & 0x7);
  } else {
    EXIT_ERROR;
    return -1;
  }
}

int registers::getVirtualRegisterNumber(char32_t ID) {
  if (ID >= SPECIALOFFSET) {
    return -1;
  }
  return ID - ((numRegisterFiles + 1) * numRegisters);
}

bool registers::isX2Arg(MO *mo, int index) {
  OPtype *types = mo->getTypes();
  if (types[index] == REG && (index + 4) < MAXARGNUMBER) {
    return (types[index + 4] == REG);
  }
  return false;
}

bool registers::isValidX2Pair(MO *mo, int index) {
  OPtype *types = mo->getTypes();
  char32_t *args = mo->getArguments();
  if (types[index] == REG && (index + 4) < MAXARGNUMBER) {
    int regno1 = getRegNumber(args[index]);
    int regno2 = getRegNumber(args[index + 4]);
    if (regno1 == -1 && regno2 == -1)
      return true;
    int regfile1 = getRegFile(args[index]);
    int regfile2 = getRegFile(args[index + 4]);
    return ((regfile1 == regfile2 && (regno2 == (regno1 + 1))) ||
            usedOffsetX2register);
  }
  return true;
}

int registers::getNumberOfCondselRegister() { return numberOfCondselRegister; }

#if CHECK_RA_TIMING

void printRATimings() {
  LOG_OUTPUT(LOG_M_ALWAYS, "RA timing:\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "  Heuristic:\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "    Total time: %f s\n",
             ra_timings.heuristicTimer.seconds());
  LOG_OUTPUT(LOG_M_ALWAYS, "    Count     : %d\n", ra_timings.heuristicCount);
  if (ra_timings.heuristicCount > 0)
    LOG_OUTPUT(LOG_M_ALWAYS, "    Avg time  : %f\n",
               ra_timings.heuristicTimer.seconds() / ra_timings.heuristicCount);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Genetic:\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "    Total time: %f s\n",
             ra_timings.geneticTimer.seconds());
  LOG_OUTPUT(LOG_M_ALWAYS, "    Count     : %d\n", ra_timings.geneticCount);
  if (ra_timings.geneticCount > 0)
    LOG_OUTPUT(LOG_M_ALWAYS, "    Avg time  : %f\n",
               ra_timings.geneticTimer.seconds() / ra_timings.geneticCount);
  LOG_OUTPUT(LOG_M_ALWAYS, "    Whole algo: %f\n",
             ra_timings.gen_CompleteTimer.seconds());
  if (ra_timings.gen_CompleteCount > 0)
    LOG_OUTPUT(LOG_M_ALWAYS, "          Avg.: %f\n",
               ra_timings.gen_CompleteTimer.seconds() /
                   ra_timings.gen_CompleteCount);
}
#endif
