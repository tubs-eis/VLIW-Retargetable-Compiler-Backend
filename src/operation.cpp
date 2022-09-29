// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#include "operation.h"
#include "MO.h"
#include "functionalunit.h"
#include "label.h"
#include "processor.h"
#include "register.h"
#include "virtual_reg.h"
#include <InstructionCounter.h>
#include <map>
#include <string>
using namespace std;

static std::map<std::string, Operation *> allOP;

const int MAXVALUES = LabelAddr + 1;
const char *stateNames[MAXVALUES] = {"Error", "REG",   "Imm",
                                     "Imm32", "Label", "LabelAddr"};

static unsigned int instructionWidth = 32;
static bool multipleInstructions = false;
static int BUSWR = 0;
static int FORWARDING = 0;
static int stackRegLatency = 4;

void setInstructionWidth(int width) { instructionWidth = width; }

int getInstructionWidth() { return instructionWidth; }

void setMultipleInstruction(bool allow) { multipleInstructions = allow; }

int getBuswr() { return BUSWR; }

void setBuswr(int buswr) { BUSWR = buswr; }

int getForwarding() { return FORWARDING; }

void setForwarding(int forwarding) { FORWARDING = forwarding; }

int getStackRegLatency() { return stackRegLatency; }

void setStackRegLatency(int latency) { stackRegLatency = latency; }

// the length in chars of the output.
#define NAMELENGTH 16
#define OPERANDLENGTH 12

// static const int MultiplyDeBruijnBitPosition[32] =
//{
//  0, 9, 1, 10, 13, 21, 2, 29, 11, 14, 16, 18, 22, 25, 3, 30,
//  8, 12, 20, 28, 15, 17, 24, 7, 19, 27, 23, 6, 26, 5, 4, 31
//};

void Operation::writeOutBin(ostream &out, MO *m) {
  OPtype *typ = m->getTypes();
  char32_t *args = m->getArguments();
  unsigned int i, off, x;
  int counter[MAXARGNUMBER];
  char32_t bins[MAXARGNUMBER];

  for (x = 0; x < MAXARGNUMBER; x++) {
    counter[x] = -1;
    bins[x] = 0;
  }

  for (i = 0; i < strlen(opcode);
       i++) { // first we count the number of occurrences of each letter.
    switch (opcode[i]) {
    case '0':
    case '1':
    case 'X':
      break;
    default: // this could be ABCDEF, etc
      off = opcode[i] - 'A';
      if (off < 0) { // error handling
        LOG_OUTPUT(LOG_M_ALWAYS,
                   "[line %4d %4s] WARNING: I think there is a mistake with "
                   "OPcode '%d'\n",
                   m->getLineNumber(), name.c_str(), opcode[i]);
        EXIT_ERROR
      }
      if (off > MAXARGNUMBER) { // error handling
        LOG_OUTPUT(
            LOG_M_ALWAYS,
            "[line %4d %4s] WARNING: OPCode '%c' is not used. there is only",
            m->getLineNumber(), name.c_str(), opcode[i]);
        for (x = 0; x < MAXARGNUMBER; x++) {
          LOG_OUTPUT(LOG_M_ALWAYS, " '%c'", (char)('A' + x));
        }
        LOG_OUTPUT(LOG_M_ALWAYS, "\n");
      }

      counter[off]++; // what should be done here?
    }
  }

  // next we create for each operand the binary representation.
  for (x = 0; x < MAXARGNUMBER; x++) {
    if (counter[x] == -1)
      continue;
    switch (typ[x]) {
    case Immediate:
    case Imm32:
      bins[x] = args[x];
      break;
    case Label:
    case LabelAddr:
      bins[x] = label::getSLM(args[x])->getOffset();
      break;
    case REG:
      bins[x] = registers::getBinary(args[x]);
      break;
    default:
      LOG_OUTPUT(LOG_M_BASIC,
                 "[line %4u %4s] INFO: no type defined for the %u. operator. "
                 "Using zeros for all occurences.\n",
                 m->getLineNumber(), name.c_str(), (x + 1));
      bins[x] = 0;
      break;
    }
  }

  if (isX2Operation()) {
    if (!strncmp(basename, "MV", 2)) {
      if (registers::isFirReg(args[1])) {
        bins[2] |= 0b00100000;
      } else {
        bins[2] &= ~(0b00100000);
      }
      if (registers::isFirReg(args[0])) {
        bins[2] |= 0b00010000;
      } else {
        bins[2] &= ~(0b00010000);
      }
    }

    if (typ[5] == REG && typ[1] == REG) {
      if (args[5] != args[1]) {
        bins[1] |= 0b01000000;
      } else {
        bins[1] &= ~(0b01000000);
      }
    }

    if (typ[6] == REG && typ[2] == REG) {
      if (args[6] != args[2]) {
        bins[0] |= 0b01000000;
      } else {
        bins[0] &= ~(0b01000000);
      }
    }
  }

  //	// sanity check. Is the operand bigger than the possible writeout bits.
  //	for (x = 0; x < MAXARGNUMBER; x++) {
  //		if (counter[x] == -1)
  //			continue;
  //
  //		uint32_t v;
  //		if(int32_t(bins[x]) < 0){
  //			v = ~uint32_t(bins[x]) + uint32_t(1);
  //		} else {
  //			v = uint32_t(bins[x]);
  //		}
  //		int binary_digits;
  //		for (binary_digits = 0; v > 0; v >>= 1)
  //			binary_digits++;
  //
  //		if ((counter[x] + 1) < binary_digits) {
  //			cerr << "[line " << std::setw(4) << m->getLineNumber()
  //<<
  //"
  //"
  //					<< std::setw(4) << name << "] ERROR:
  // Operand
  //"
  //<<
  // x
  //					<< " is bigger than the possible
  // writeoutbits."
  //<<
  // endl; 			m->writeOutReadable(cerr);
  // cerr
  // << endl
  // <<
  // "[line
  // "
  // << std::setw(4) << m->getLineNumber() << " "
  //					<< std::setw(4) << name << "] " <<
  // binary_digits
  //<< " bit (required)" << " > " << counter[x] + 1 << " bit (available)"
  //					<< endl;
  //			EXIT_ERROR
  //		}
  //	}

  // the actual writeout
  // it is done blockwise with 8 bits each block (1 char)
  char writeout = 0;
  for (i = 0; i < strlen(opcode); i++) {
    switch (opcode[i]) {
    case 'X':
      if (defaultValues[i] == '1')
        writeout |= (1 << (7 - (i % 8)));
      break;
    case '0': // we can ignore the 0, because it is already set.
      break;
    case '1':
      writeout |= (1 << (7 - (i % 8)));
      break;
    default: // this should be ABCDEF, etc
      off = opcode[i] - 'A';
      char temp = (bins[off] >> counter[off]) &
                  1; // shift the bit to the right until the bit we want
      temp = temp << (7 - (i % 8));
      counter[off]--;
      writeout |= temp;
      break;
    }
    if (i % 8 == 7) {
      out << writeout;
      writeout = 0;
    }
  }
}

char32_t getSearchableRegisterChar(char32_t reg,
                                   VirtualRegisterMap const *mapping) {
  if (!mapping)
    return reg;
  else {
    auto it = mapping->find(reg);
    if (it != mapping->end()) {
      char32_t real = it->second->getReal();
      if (real != (char32_t)-1 && real != (char32_t)-2)
        return it->second->getReal();
      else
        return reg;
    } else
      return reg;
  }
}

std::string getMappedRegisterName(char32_t reg,
                                  VirtualRegisterMap const *mapping) {
  return registers::getName(getSearchableRegisterChar(reg, mapping));
}

/**
 * Get Register. If a virtual Mapping occurs, resolve it. This function ASSUMES
 * a monolithic RF!
 * @param reg
 * @param mapping
 * @return return Register (0-63)
 */
char32_t getMappedRegisterMonolith(char32_t reg,
                                   VirtualRegisterMap const *mapping) {
  auto physicalRegister = getSearchableRegisterChar(reg, mapping);
  return physicalRegister;
}

void Operation::writeOutReadable(ostream &out, const MO *m,
                                 const VirtualRegisterMap *mapping) {
  if (name != "NOP") {
    InstructionCounter::getInstance().increment();
  }
  const OPtype *typ = m->getTypes();
  const char32_t *args = m->getArguments();
  int i, f;
  char buffer[64];
  // if the Instruction is merged from two instructions, then print out both
  // line numbers.
  if ((int)m->getSecondLineNumber() > 0) {
    sprintf(buffer, "%d+%d", m->getLineNumber(), m->getSecondLineNumber());
    for (f = strlen(buffer); f < 9; f++)
      out << " ";
    out << buffer;
  } else {
    out << std::setw(9) << std::setfill(' ') << m->getLineNumber();
  }
  // space between line numbers and instruction name.
  out << " " << std::setw(0) << name << " ";
#if STORE_PARTINGS_AND_ALONE
  if (m->wasParting || m->wasAlone) {
    out << " (";
    if (m->wasParting)
      out << "p";
    if (m->wasAlone)
      out << "a";
    out << ")";
  }
#endif
  for (i = strlen(name.c_str()); i < NAMELENGTH - 1; i++) {
    out << " "; // print out the name and some spaces to make it look nicer.
  }
  char end[64];
  end[0] = 0;
  // we only iterate over the first 4 arguments. The rest is not printed out,
  // but for internal purposes.
  for (i = 0; i < 4; i++) {
    switch (typ[i]) {
    case Label:
      sprintf(buffer, "%s ", label::getLabelName(args[i]).c_str());
      out << buffer;
      for (f = strlen(buffer); f < OPERANDLENGTH; f++) {
        out << " ";
      }
      break;
    case REG:
      if (args[i + 4] != 0 && args[i + 4] != args[i])
        sprintf(buffer, "%s+%s ",
                getMappedRegisterName(args[i], mapping).c_str(),
                getMappedRegisterName(args[i + 4], mapping).c_str());
      else
        sprintf(buffer, "%s ", getMappedRegisterName(args[i], mapping).c_str());
      out << buffer;
      for (f = strlen(buffer); f < OPERANDLENGTH; f++) {
        out << " ";
      }
      break;
    case Immediate:
      sprintf(buffer, "0x%x ", (uint)args[i]);
      out << buffer;
      for (f = strlen(buffer); f < OPERANDLENGTH; f++)
        out << " ";
      break;
    case Imm32:
      sprintf(end, "0x%x ", (uint)args[i]);
      // break deleted, since default prints the space.
    case Error:
    default:
      out << "            ";
      break;
    }
  }
  if (end[0] != 0) {
    // the space where the line number would normally be.
    out << "          " << end;
    for (f = strlen(end); f < (NAMELENGTH + 4 * OPERANDLENGTH); f++)
      out << " ";
  }
}

void Operation::writeOutCompilable(ostream &out, const MO *m,
                                   const VirtualRegisterMap *mapping) {
  // nice clone of writeOutReadable

  if (name == "NOP") {
    return;
  }
  const OPtype *typ = m->getTypes();
  const char32_t *args = m->getArguments();
  int i; //, f;
  char buffer[64];
  out << name << " ";

#if STORE_PARTINGS_AND_ALONE
  if (m->wasParting || m->wasAlone) {
    out << " (";
    if (m->wasParting)
      out << "p";
    if (m->wasAlone)
      out << "a";
    out << ")";
  }
#endif
  //  for (i = strlen(name.c_str()); i < NAMELENGTH - 1; i++) {
  //    out << " "; // print out the name and some spaces to make it look nicer.
  //  }
  char end[64];
  end[0] = 0;
  // we only iterate over the first 4 arguments. The rest is not printed out,
  // but for internal purposes.
  for (i = 0; i < 4; i++) {
    if (i > 0 and i < 3 and
        (typ[i] == REG or typ[i] == Immediate or typ[i] == Imm32)) {
      out << ", ";
    }
    switch (typ[i]) {
    case Label:
      sprintf(buffer, "%s", label::getLabelName(args[i]).c_str());
      out << buffer;
      break;
    case REG:
      if (args[i + 4] != 0 && args[i + 4] != args[i])
        sprintf(buffer, "%s+%s",
                getMappedRegisterName(args[i], mapping).c_str(),
                getMappedRegisterName(args[i + 4], mapping).c_str());
      else
        sprintf(buffer, "%s", getMappedRegisterName(args[i], mapping).c_str());
      out << buffer;
      break;
    case Immediate:
      sprintf(buffer, "#0x%x", (uint)args[i]);
      out << buffer;
      break;
    case Imm32:
      sprintf(end, "#0x%x", (uint)args[i]);
      // break deleted, since default prints the space.
    case Error:
    default:
      out << "";
      break;
    }
  }
}

std::string Operation::to_string(MO *mo, VirtualRegisterMap const *mapping) {
  stringstream sstr;

  if ((int)mo->getSecondLineNumber() > 0)
    sstr << setw(4) << mo->getLineNumber() << "+" << setw(4)
         << mo->getSecondLineNumber();
  else
    sstr << setw(9) << mo->getLineNumber();
  sstr << " " << setw(NAMELENGTH) << left << name << " ";
  OPtype *type = mo->getTypes();
  char32_t *args = mo->getArguments();
  for (int i = 0; i < 4; ++i) {
    switch (type[i]) {
    case Label:
      sstr << setw(OPERANDLENGTH) << "-" << i << "-"
           << label::getLabelName(args[i]).c_str();
      break;
    case REG:
      if (isDoubleRegister(mo, i)) {
        stringstream ostr;
        ostr << getMappedRegisterName(args[i], mapping) << "+"
             << getMappedRegisterName(args[i + 4], mapping);
        sstr << setw(OPERANDLENGTH) << "-" << i << "-" << ostr.str();
      } else {
        sstr << setw(OPERANDLENGTH) << "-" << i << "-"
             << getMappedRegisterName(args[i], mapping);
        break;
      }
    case Immediate:
    case Imm32: {
      stringstream ostr;
      ostr << "0x" << hex << (uint)args[i];
      sstr << setw(OPERANDLENGTH) << "-" << i << "-" << ostr.str();
      break;
    }
    default:
      sstr << "            ";
      break;
    }
  }
  return sstr.str();
}

char32_t convertRegisterSet(char32_t reg, VirtualRegisterMap const *mapping) {
  // todo: correct convertRegister
  if (!mapping)
    return reg;
  else {
    auto it = mapping->find(reg);
    if (it != mapping->end()) {
      char32_t real = it->second->getReal();
      if (real != (char32_t)-1 && real != (char32_t)-2)
        return it->second->getReal();
      else
        return reg;
    } else
      return reg;
  }
}

OPtype getOPtype(char *c) {
  for (int i = 0; i < MAXVALUES; i++) {
    if (!strcmp(c, stateNames[i])) {
      return static_cast<OPtype>(i);
    }
  }
  return Error;
}

void showAllOperations() {
  for (map<std::string, Operation *>::iterator op = allOP.begin();
       op != allOP.end(); op++) {
    std::cout << "Operation: " << *(op->second) << std::endl;
  }
  std::cout << "size: " << allOP.size() << std::endl;
}

bool checkValidArguments(MO *mo, Processor *pro) {
  Operation *op = mo->getOperation();
  char32_t *args = mo->getArguments();
  int FIR_OFF_MASK =
      pro->isX2Supported() && mo->getOperation()->isX2Operation() ? 0xf : 0x3f;
  int FIR_OFF_LEN =
      pro->isX2Supported() && mo->getOperation()->isX2Operation() ? 4 : 6;
  if (mo->getOperation()->getBaseName() == "MV" &&
      (registers::isFirOffsetReg(args[0]) ||
       registers::isFirOffsetReg(args[1]))) {
    int imm = (((int)args[2] & ~FIR_OFF_MASK) >> FIR_OFF_LEN);
    int sign = ((int)args[2] >> (FIR_OFF_LEN - 1)) & 1;
    if (!((sign == 0 && imm == 0) || (sign == 1 && imm == -1) ||
          (pro->getProcessorName() != "EIS-VLIW" && args[2] == 64))) {
      LOG_OUTPUT(LOG_M_ALWAYS, "FIR offset value too big!\n");
      return false;
    }
  }
  for (int i = 0; i < mo->getArgNumber(); ++i) {
    if (registers::isX2Arg(mo, i)) {
      if (getFU((char *)"AA")->contains(op->getName().c_str()) &&
          strncmp(op->getName().c_str(), "MUL", 3)) {
        // MAC instruction is handled in moai_special in specialInit
      } else {
        if (!op->isX2Operation()) {
          LOG_OUTPUT(LOG_M_ALWAYS, "X2 argument for non-X2 operation\n");
          return false;
        }
        if (!registers::isValidX2Pair(mo, i)) {
          LOG_OUTPUT(LOG_M_ALWAYS,
                     "Invalid register pairing for X2 argument\n");
          return false;
        }
      }
    }
  }
  return true;
}

MO *checkline(const char *line, Processor *pro) {

  char *toSend = (char *)line;
  // remove the issueslots for the moment.
  int slot = 0;
  int i = 1;
  bool partOperation = false;
  if (toSend[0] == ':' && isdigit(toSend[1])) {
    while (!isspace(toSend[i])) {
      unsigned int number = (toSend[i] - '0');
      if (number == 9) {
        partOperation = true;
      } else if (number >= MI::getNumberIssueSlots()) {
        LOG_OUTPUT(
            LOG_M_ALWAYS,
            "there are only %d issue slots. Constraint %d is not valid!\n",
            MI::getNumberIssueSlots(), number);
        return NULL;
      } else
        slot |= (1 << number);
      i++;
    }
    toSend += i;
    while (isspace(*toSend++))
      ;
    toSend--; // because we are one step ahead
  }

  // check with all Operations.
  MO *tmp = NULL;
  std::string compare(toSend);
  compare = compare.substr(0, compare.find_first_of(" \t\n"));
  std::map<std::string, Operation *>::iterator op = allOP.find(compare);
  if (op != allOP.end()) {
    tmp = op->second->generateMO(toSend);
    if (tmp != NULL) {
      if (!checkValidArguments(tmp, pro)) {
        delete tmp;
        return NULL;
      }
      if (partOperation) {
        tmp->setParting(true);
      }
      tmp->setIssueSlotConstraint(slot);
      return tmp;
    }
  } else {
    LOG_OUTPUT(LOG_M_ALWAYS, "Operation not found!\n");
  }

  return NULL;
}

using namespace rapidxml;

Operation::Operation(rapidxml::xml_node<> *start, int FUID,
                     char *defaultDirections) {
  Imm[0] = 0;
  Cond[0] = 0;
  Suf[0] = 0;
  Size[0] = 0;
  memset(basename, 0, NAMELENGTH + 1);
  name[0] = 0;
  argnumber = MAXARGNUMBER;
  latency = 1;
  branches = false;
  CRBranch = false;
  def = false;
  FuId = FUID;
  selfdependent = false;
  pipelined = false;

  int i;
  for (i = 0; i < OPCODELENGTH; i++) {
    opcode[i] = 0;
    defaultValues[i] = 0;
  }
  for (i = 0; i < MAXARGNUMBER; i++) {
    types[i] = Error;
    dir[i] = READ;
  }
  if (defaultDirections != NULL)
    for (i = 0; i < MAXARGNUMBER; i++) {
      dir[i] = defaultDirections[i];
    }
  char *n = start->first_attribute("name")->value();
  xml_attribute<> *branch = start->first_attribute("branch");
  if (branch != NULL) {
    branches |= !strcmp(branch->value(), "true");
    branches |= !strcmp(branch->value(), "TRUE");
  }
  xml_attribute<> *condition = start->first_attribute("condition");
  if (condition != NULL) {
    CRBranch |= !strcmp(branch->value(), "true");
    CRBranch |= !strcmp(branch->value(), "TRUE");
  }
  xml_attribute<> *defa = start->first_attribute("default");
  if (defa != NULL) {
    def |= !strcmp(defa->value(), "true");
    def |= !strcmp(defa->value(), "TRUE");
  }
  if (!strcmp(start->name(),
              "opp")) { // if this is a operation, change the name.
    if (strlen(n) > NAMELENGTH) {
      LOG_OUTPUT(LOG_M_ALWAYS, " Name of operation %s exceeds %d characters.\n",
                 n, NAMELENGTH);
      EXIT_ERROR;
    }
    strcopy(basename, n);
    name = string(n);
  }
  if (!strcmp(start->name(), "imm")) { // else change accordingly
    strcopy(Imm, n);
  }
  if (!strcmp(start->name(), "cond")) {
    strcopy(Cond, n);
  }
  if (!strcmp(start->name(), "suf")) {
    strcopy(Suf, n);
  }
  if (!strcmp(start->name(), "size")) {
    strcopy(Size, n);
  }
  rename();

  char *o = trim(start->value(), 512);
  unsigned int x;
  i = 0;
  for (x = 0; x < strlen(o); x++) {
    opcode[i] = o[x];
    if (!isspace(o[x])) // so we can have spaces in the opcode for better human
                        // reading.
      i++;
  }
  opcode[i] = 0;
  if (multipleInstructions) {
    if (strlen(opcode) % instructionWidth != 0) {
      cerr << "the opcode has not the right length (" << strlen(opcode)
           << " is not a multiple of " << instructionWidth << ")" << endl;
      cerr << start->first_attribute("name")->value() << " - " << o << endl;
      EXIT_ERROR;
    }
  } else {
    if (strlen(opcode) != instructionWidth) {
      cerr << "the opcode has not the right length (" << strlen(opcode)
           << " != " << instructionWidth << ")" << endl;
      cerr << start->first_attribute("name")->value() << " - " << o << endl;
      EXIT_ERROR;
    }
  }
  if (strlen(opcode) == 0) {
    cerr << "the opcode is not present!" << endl;
    cerr << start->first_attribute("name")->value() << " - " << o << endl;
    EXIT_ERROR;
  }
  lengthMultiplier = strlen(opcode) / instructionWidth;
  LOG_OUTPUT(LOG_M_INIT_PROC, "Operation Name: %6s Bitstream: %s\n",
             name.c_str(), opcode);

  xml_node<> *latNode = start->first_node("lat");
  if (latNode != NULL) {
    latency = atoi(latNode->first_attribute("value")->value());
    LOG_OUTPUT(LOG_M_INIT_PROC, "New Latency: %d\n", latency);
  }

  xml_node<> *dependencyNode = start->first_node("selfdependent");
  if (dependencyNode != NULL) {
    selfdependent = atoi(dependencyNode->first_attribute("value")->value());
    xml_attribute<> *ppl_attr = dependencyNode->first_attribute("pipelined");
    if (ppl_attr) {
      pipelined = std::string(ppl_attr->value()) == "1";
    }
    LOG_OUTPUT(LOG_M_INIT_PROC, "Self-Dependent: %d\n", selfdependent);
  }

  char *tmp;
  for (xml_node<> *parameterNode = start->first_node("par"); parameterNode;
       parameterNode = parameterNode->next_sibling()) {
    if (strcmp(parameterNode->name(), "par"))
      continue; // otherwise all other siblings would be considert.

    tmp = parameterNode->first_attribute("pos")->value();
    if (tmp != NULL)
      argnumber = atoi(tmp);
    else
      argnumber++;
    if (argnumber > MAXARGNUMBER) {
      cerr << "The defined position (" << argnumber << ") is to high!" << endl;
      EXIT_ERROR
    }

    tmp = parameterNode->first_attribute("type")->value();
    OPtype type = getOPtype(tmp);
    if (type == Error) {
      cerr << "The Parameter type (" << tmp << ") is not known!";
      EXIT_ERROR
    }
    types[argnumber - 1] = type;
    LOG_OUTPUT(LOG_M_INIT_PROC, "Parameter %d is a %s\n", argnumber,
               stateNames[types[argnumber - 1]]);
  }
  for (xml_node<> *parameterNode = start->first_node("direction");
       parameterNode; parameterNode = parameterNode->next_sibling()) {
    if (strcmp(parameterNode->name(), "direction"))
      continue; // otherwise all other siblings would be considert.
    int i = -1;
    tmp = parameterNode->first_attribute("pos")->value();
    if (tmp != NULL)
      i = atoi(tmp);
    else {
      cerr << "You must define a position for the direction!" << endl;
      EXIT_ERROR;
    }
    if (i > MAXARGNUMBER) {
      cerr << "The defined position (" << i << ") is to high!" << endl;
      EXIT_ERROR
    }

    tmp = parameterNode->first_attribute("dir")->value();
    if (!strcmp(tmp, "read"))
      dir[i - 1] = READ;
    else if (!strcmp(tmp, "write"))
      dir[i - 1] = WRITE;
    else if (!strcmp(tmp, "readwrite"))
      dir[i - 1] = READWRITE;
    else {
      cerr << "The direction type (" << tmp << ") is not known!";
      EXIT_ERROR
    }
    LOG_OUTPUT(LOG_M_INIT_PROC, "Direction %d is %s\n", i, tmp);
  }

  if (!strcmp(
          start->name(),
          "opp")) // only consider this for allOp if it is an actual Operation.
    allOP.insert(std::pair<std::string, Operation *>(name, this));
}

// since strtol returns 0 if it fails, we have to extra-check if an operand is a
// representation of 0.
bool isNull(const char *test) {
  int i = 0;
  while (test[i] == '0')
    i++;
  if (test[i] == 'x' || test[i] == 'b') {
    i++;
    while (test[i] == '0')
      i++;
  }
  bool n;
  n = test[i] == ',';
  n |= isspace(test[i]);
  n |= test[i] == 0;
  if (n) {
    LOG_OUTPUT(LOG_M_INIT_PROC, "%s = 0\n", test);
  } else {
    LOG_OUTPUT(LOG_M_INIT_PROC, "%s != 0\n", test);
  }
  return n;
}

char *Operation::generateArgument(const char *line, char32_t *value,
                                  OPtype *type) {
  char *final;
  int i = 0;
  while ((isspace(line[i]) && line[i] != 0) || line[i] == ',')
    i++;
  if (line[i] == 0)
    return NULL;
  if ((*type == Immediate || *type == Imm32) && line[i] != '#') {
    cerr << "Invalid immediate value " << line + i << endl;
    EXIT_ERROR
    return NULL;
  }
  if (line[i] == '#') {
    if (*type == Error)
      *type = Immediate;
    if (line[i + 2] == 'b')
      *value = strtoul(line + (i + 3), &final, 2);
    else
      *value = strtoul(line + (i + 1), &final, 0);
    if (*value == 0L && !isNull(line + (i + 1))) {
      final = label::parseLabel((char *)line + (i + 1), value);
      *type = LabelAddr;
      //			if (*value == 0L) {
      //				EXIT_ERROR;
      //			}
    }
    return final;
  }
  *value = strtoul(line + i, &final, 0);
  if (*value != 0L || isNull(line + i)) {
    if (*type == Error)
      *type = Immediate;
    return final;
  }
  final = (char *)registers::parseRegister(line + i, value);
  if (final != NULL) {
    *type = REG;
    return final;
  }
  final = label::parseLabel((char *)line + i, value);
  if (final != NULL && value >= 0) {
    *type = LabelAddr;
    return final;
  }
  cerr << "could not parse " << line + i << endl;
  EXIT_ERROR
  return NULL;
}

char *Operation::parsePragma(MO *mo, char *line) {
  if (!strncmp(line, "final_lat=", 10)) {
    line += 10;
    // Here, we should check, if there are more pragmas...
    int lat = atoi(line);
    mo->setFinalLatency(lat);
  }
  return 0;
}

MO *Operation::generateMO(const char *asmline) {
  bool clear = true; // check whether the names are equal
  int i = 0, x = 0;
  do {
    clear &= tolower(asmline[i]) == tolower(name[i]);
    i++;
  } while (clear && !isspace(asmline[i]) && asmline[i] != 0);
  if (!clear || name[i] != 0)
    return NULL;
  LOG_OUTPUT(LOG_M_PARSE, "generate new instruction from %s\n", name.c_str());
  char32_t arguments[MAXARGNUMBER];
  char *blub = (char *)asmline + i;
  OPtype tmpType[MAXARGNUMBER];
  for (x = 0; x < MAXARGNUMBER; x++) {
    tmpType[x] = types[x];
    arguments[x] = 0;
  }
  for (x = 0; x < MAXARGNUMBER; x++) {
    if (types[x] == Label)
      blub = label::parseLabel(blub, arguments + x);
    else
      blub = generateArgument(blub, arguments + x, tmpType + x);
    if (blub == NULL)
      break;
    if (blub == (char *)-1) {
      return NULL;
    }
    while (blub[0] && isspace(blub[0]))
      ++blub;
    if (blub[0] && blub[0] == '-' && blub[1] && blub[1] == '-') {
      MO *mo = new MO(this, arguments, tmpType, dir);
      blub = parsePragma(mo, blub + 2);
      return mo;
    }
  }

  return new MO(this, arguments, tmpType, dir);
}

Operation *Operation::addSuffix(Operation *op) {
  if (Imm[0] != 0 && op->Imm[0] != 0) // we can only have one Immediate, one
                                      // condition flag and one Size set.
    return NULL;
  if (Cond[0] != 0 && op->Cond[0] != 0)
    return NULL;
  if (Size[0] != 0 && op->Size[0] != 0)
    return NULL;
  string s(Suf);
  string test(op->Suf);
  if (test.size() > 0 && s.size() > 0) {
    std::size_t found = s.find(test);
    if (found != std::string::npos)
      return NULL;
  }
  int i;
  char code[OPCODELENGTH + 1];
  for (i = 0; i < OPCODELENGTH; i++) {
    if (op->def) {
      if (defaultValues[i] == 0 || defaultValues[i] == 'X') {
        defaultValues[i] = op->opcode[i];
      } else if (defaultValues[i] != op->opcode[i] && op->opcode[i] != 'X') {
        cerr << "[" << name
             << "] ERROR: there are different defaultvalues trying to be set. "
             << endl
             << defaultValues << endl
             << op->opcode << endl;
        EXIT_ERROR;
      }
    }

    if (opcode[i] == op->opcode[i])
      code[i] = opcode[i];
    else {
      if (opcode[i] == 'X')
        code[i] = op->opcode[i];
      else if (op->opcode[i] == 'X')
        code[i] = opcode[i];
      else if (opcode[i] == 0)
        code[i] = op->opcode[i];
      else if (op->opcode[i] == 0)
        code[i] = opcode[i];
      else { // ERROR
        return NULL;
      }
    }
    if (op->opcode[i] == 0 && opcode[i] == 0)
      break;
  }
  code[i] = 0;
  Operation *nop = new Operation(*this);
  strcopy(nop->opcode, code, OPCODELENGTH);
  if (op->Imm[0] != 0)
    strcopy(nop->Imm, op->Imm);
  if (op->Cond[0] != 0)
    strcopy(nop->Cond, op->Cond);
  if (op->Size[0] != 0)
    strcopy(nop->Size, op->Size);
  if (op->Suf[0] != 0)
    strcat(nop->Suf, op->Suf);
  nop->rename();
  std::map<std::string, Operation *>::iterator it = allOP.find(nop->name);
  if (it != allOP.end()) {
    delete nop;
    return NULL;
  }
  allOP.insert(std::pair<std::string, Operation *>(nop->name, nop));
  return nop;
}

void Operation::rename() {
  string tmp(basename);
  tmp += Imm;
  tmp += Cond;
  tmp += Size;
  tmp += Suf;
  name = tmp;

  isX2 = name.find("_X2") != name.npos;

  bool explicitCR = name.find("CR") != name.npos;
  bool isaImplicitCR = false;
  if (tmp.find("ADDX") != tmp.npos or tmp.find("SUBX") != tmp.npos or
      tmp.find("ADDSUB") != tmp.npos or tmp.find("SUBADD") != tmp.npos) {
    isaImplicitCR = true;
  }
  isCR = explicitCR or isaImplicitCR;
  isCS = name.find("CS") != name.npos;
}

// template <class T>
// int getPhysicalReadPort(const MO *mo, const uint port, const T *map) {
//  return -1;
//}

int Operation::getPhysicalReadPort(const MO *mo, const uint port,
                                   const VirtualRegisterMap *map) const {
  if (port > 3) {
    throw "Trying to access read port through the wrong method! To check MAC "
          "read ports use different function!";
  }
  const char32_t *args = mo->getArguments();
  const OPtype *type = mo->getTypes();

  // write port offset
  uint actualArgumentPort = port + 1;
  // process MAC or X2 ports
  if (port == 2 or port == 3) {
    if (port == 2) {
      actualArgumentPort = 0;
    }
    if (port == 3) {
      actualArgumentPort = 4;
    }
  }
  if (type[actualArgumentPort] == REG) {
    if (registers::isVirtualReg(args[actualArgumentPort])) {
      return getMappedRegisterMonolith(args[actualArgumentPort], map);
    } else if (registers::isPhysicalRegister(args[actualArgumentPort])) {
      return args[actualArgumentPort];
    } else {
      return getEmptyPhysicalReadPort();
    }
  } else {
    return getEmptyPhysicalReadPort();
  }
}

int Operation::getPhysicalReadPort(const MO *mo, const uint port,
                                   const VirtualAllocation *regMapping) const {
  if (port > 1) {
    throw "Trying to access read port through the wrong method! To check MAC "
          "read ports use different function!";
  }
  const char32_t *args = mo->getArguments();
  const OPtype *type = mo->getTypes();
  uint actualArgumentPort = port + 1;
  if (type[actualArgumentPort] == REG) {
    auto argument = args[actualArgumentPort];
    if (regMapping->find(argument) != regMapping->end()) {
      return regMapping->at(argument);
    }
    { return argument; }
  } else {
    return getEmptyPhysicalReadPort();
  }
}

int getEmptyPhysicalReadPort() { return -1; }

bool Operation::isDoubleRegister(const MO *mo, const uint port) const {
  const char32_t *args = mo->getArguments();
  const OPtype *type = mo->getTypes();
  if (type[port] == REG) {
    return args[port + 4] != 0 && args[port + 4] != args[port];
  } else {
    return false;
  }
}
