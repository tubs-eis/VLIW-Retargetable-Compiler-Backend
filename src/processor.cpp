// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "processor.h"
#include "../header/memory.h"
#include "global.h"
#include "register.h"

#include "functionalunit.h"
#include "operation.h"
#include "readFile.h"
#include "vectorunit.h"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>

using namespace rapidxml;
using namespace std;

bool Processor::DMAAddrConfig::isDMAStore(MO *op) {
  if (strncmp(op->getOperation()->getName().c_str(), "STORE", 5))
    return false;

  char32_t a = op->getArguments()[0];
  return (a >= startAddr && a <= endAddr);
}

bool Processor::DMAAddrConfig::isDMACtrlAddr(MO *op) {
  if (!isDMAStore(op))
    return false;
  char32_t a = op->getArguments()[0];
  return a == CTRL_START_ADDR || a == RESET_ADDR;
}

void instructionHelper(rapidxml::xml_node<> *start) {
  if (start == NULL)
    return;
  xml_node<> *s = start->first_node("size");
  if (s != NULL) {
    int size = atoi(s->value());
    setInstructionWidth(size);
  }
  s = start->first_node("allowMultiple");
  if (s != NULL) {
    if (!strcmp("true", s->value()))
      setMultipleInstruction(true);
    else if (!strcmp("false", s->value()))
      setMultipleInstruction(false);
    else {
      cerr << "could not understand " << s->value()
           << ".\nit must be either 'true' or 'false'" << endl;
      EXIT_ERROR;
    }
  }
  xml_node<> *latfirwriteNode = start->first_node("firwritelat");
  if (latfirwriteNode != NULL) {
    int firwritelatency =
        atoi(latfirwriteNode->first_attribute("value")->value());
    registers::setFirWriteLatency(firwritelatency);
  }
  xml_node<> *stackRegLatNode = start->first_node("stackreglat");
  if (stackRegLatNode)
    setStackRegLatency(
        atoi(stackRegLatNode->first_attribute("value")->value()));
  xml_node<> *buswrNode = start->first_node("buswr");
  if (buswrNode)
    setBuswr(atoi(buswrNode->first_attribute("value")->value()));
  xml_node<> *fwdNode = start->first_node("forwarding");
  if (fwdNode)
    setForwarding(atoi(fwdNode->first_attribute("value")->value()));
}

void preProcessor(rapidxml::xml_node<> *start) {
  if (start == NULL)
    return;
  char *before, *after;
  for (xml_node<> *regex = start->first_node("regex"); regex;
       regex = regex->next_sibling()) {
    if (strcmp(regex->name(), "regex"))
      continue; // we just want to see the vector Units for the Moment.
    before = regex->first_attribute("before")->value();
    after = regex->first_attribute("after")->value();
    LOG_OUTPUT(LOG_M_INIT_PROC,
               "found a new regular expression\n\"%s\" -> \"%s\"\n", before,
               after);
    addRegEx(before, after);
  }
}

void Processor::readDMAAddrRange(rapidxml::xml_node<> *node) {
  if (dmaAddrConfig)
    delete dmaAddrConfig;
  dmaAddrConfig = new DMAAddrConfig;

  xml_node<> *n = node->first_node("start");
  if (n) {
    dmaAddrConfig->startAddr = stoi(n->value(), 0, 0);
  }
  n = node->first_node("end");
  if (n)
    dmaAddrConfig->endAddr = stoi(n->value(), 0, 0);
  n = node->first_node("ctrl-start");
  if (n)
    dmaAddrConfig->CTRL_START_ADDR = stoi(n->value(), 0, 0);
  n = node->first_node("reset");
  if (n)
    dmaAddrConfig->RESET_ADDR = stoi(n->value(), 0, 0);

  LOG_OUTPUT(LOG_M_ALWAYS, "DMA Address Range:\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "      start = 0x%x\n", dmaAddrConfig->startAddr);
  LOG_OUTPUT(LOG_M_ALWAYS, "        end = 0x%x\n", dmaAddrConfig->endAddr);
  LOG_OUTPUT(LOG_M_ALWAYS, " ctrl start = 0x%x\n",
             dmaAddrConfig->CTRL_START_ADDR);
}

void Processor::initialize(char *configFile) {
  LOG_OUTPUT(LOG_M_INIT_PROC, "Opening configuration file: %s\n", configFile);
  xml_document<> doc;
  xml_node<> *root_node;
  // Read the xml file into a vector
  ifstream theFile(configFile);
  if (!theFile.good()) {
    cerr << "File \"" << configFile << "\" is not accessible" << endl;
    theFile.close();
    EXIT_ERROR;
  }
  vector<char> buffer((istreambuf_iterator<char>(theFile)),
                      istreambuf_iterator<char>());
  buffer.push_back('\0');
  // Parse the buffer using the xml file parsing library into doc
  doc.parse<0>(&buffer[0]);
  // Find our root node
  root_node = doc.first_node("processor");
  if (root_node == NULL) {
    cerr << "could not read config file " << configFile << endl
         << "xml file must contain a root node called 'processor'" << endl;
    EXIT_ERROR;
  }

  for (xml_node<> *include_node = root_node->first_node("include");
       include_node; include_node = include_node->next_sibling()) {
    if (strcmp(include_node->name(), "include"))
      continue; // we just want to see the includes for the Moment.
    // redo the name!
    char nextname[256];
    strcopy(nextname, include_node->first_attribute("name")->value());
    if (nextname[0] != '/') {
      strcopy(nextname, configFile);
      int len = strlen(nextname) - 1;
      while (len >= 0 && nextname[len] != '/')
        len--;
      len++;
      strcopy(nextname + len, include_node->first_attribute("name")->value());
    }
    initialize(nextname);
  }
  instructionHelper(root_node->first_node("instruction"));
  preProcessor(root_node->first_node("preprocessor"));
  // find the definition of FU and init them.
  initFU(root_node->first_node("FU-define"));
  // Registers should be initialized, before the IssueSlots are initialized
  registers::initRegister(root_node->first_node("register-file"));
  memory::initMemory(root_node->first_node("memory"));
  xml_node<> *tmp = root_node->first_node("binary-size");
  if (tmp != NULL)
    binSize = atoi(tmp->value());
  tmp = root_node->first_node("dma");
  if (tmp)
    readDMAAddrRange(tmp);
  tmp = root_node->first_node("name");
  if (tmp != NULL)
    name = tmp->value();
  // Iterate over the Vector Units
  for (xml_node<> *vectorUnit_node = root_node->first_node("vectorUnit");
       vectorUnit_node; vectorUnit_node = vectorUnit_node->next_sibling()) {
    if (strcmp(vectorUnit_node->name(), "vectorUnit"))
      continue; // we just want to see the vector Units for the Moment.
    LOG_OUTPUT(LOG_M_INIT_PROC, "found a new Vector Unit\n");
    VectorUnit *unit = new VectorUnit(vectorUnit_node);
    Units.push_back(unit);
    issueSlots = unit->getNumIssueSlots();
    MI::setNumberOfSlots(issueSlots);
  }
  // creates the list of all operations.
  if (allOps != NULL)
    delete allOps;
  allOps = new std::map<std::string, Operation *>();
  for (std::vector<VectorUnit *>::iterator p = Units.begin(); p != Units.end();
       p++) {
    vector<Operation *> *fOps = (*p)->getAllOperations();
    for (std::vector<Operation *>::iterator o = fOps->begin(); o != fOps->end();
         o++) {
      allOps->insert(
          std::pair<const std::string, Operation *>((*o)->getName(), *o));
      if (!strcmp((*o)->getName().c_str(), "NOP"))
        NOP = (*o);
      if (!strcmp((*o)->getName().c_str(), "MV"))
        MV = (*o);
      if (!strcmp((*o)->getName().c_str(), "OR"))
        OR = (*o);
      if ((*o)->isX2Operation())
        X2Support = true;
    }
    delete fOps;
  }
}

Processor::Processor(char *configFile) {
  binSize = 0;
  name = "tukuturi";
  issueSlots = 0;
  X2Support = false;
  allOps = NULL;
  dmaAddrConfig = 0;
  initialize(configFile);
}

Processor::~Processor() {
  delete allOps;
  for (std::vector<VectorUnit *>::iterator p = Units.begin(); p != Units.end();
       p++) {
    delete *p;
  }
  if (dmaAddrConfig)
    delete dmaAddrConfig;
  deleteFUs();
}

Operation *Processor::getOperation(std::string name) {
  std::map<std::string, Operation *>::iterator res = allOps->find(name);
  if (res != allOps->end())
    return res->second;
  return NULL;
}

bool Processor::isValid() {
  bool valid = true;
  for (std::vector<VectorUnit *>::iterator p = Units.begin(); p != Units.end();
       p++) {
    valid &= (*p)->isValid();
  }
  return valid;
}

void Processor::replaceNull(MI *ins) {
  uint x = 0;
  MO **ops = ins->getOperations();
  while (x < MI::getNumberIssueSlots()) {
    MO *op = ops[x];
    if (op == NULL) { // this happens if it is not yet set.
      ops[x] = new MO(NOP);
      x++;
    } else
      x += op->opLengthMultiplier();
  }
}

Processor::DMAAddrConfig *Processor::getDMAConfig() { return dmaAddrConfig; }

bool Processor::isNotParallelMOs(const MO *mo1, const MO *mo2) const {
  return (mo1 == nullptr) || (mo1 == (MO *)-1) || (mo2 == nullptr) ||
         (mo2 == (MO *)-1);
}

bool Processor::isNotParallelMemorySystemAccess(MI *ins) const {
  if (ins->getNumberIssueSlots() == 1) {
    return true; // no parallel MOs
  }
  MO *mo1 = ins->getOperations()[0];
  MO *mo2 = ins->getOperations()[1];
  if (isNotParallelMOs(mo1, mo2)) {
    return true; // no parallel MOs
  } else {
    if (mo1->isMemStore(dmaAddrConfig) && mo2->isMemStore(dmaAddrConfig))
      return false;
    if ((mo1->isMemStore(dmaAddrConfig) && mo2->isWriteIndirect()) ||
        (mo1->isWriteIndirect() && mo2->isMemStore(dmaAddrConfig)))
      return false;
    if (dmaAddrConfig && dmaAddrConfig->isDMAStore(mo1) &&
        dmaAddrConfig->isDMAStore(mo2))
      return false;

    if (mo1->isMemLoad() && mo2->isMemLoad())
      return false;

    if ((mo1->isMemLoad() && mo2->isReadIndirect()) ||
        (mo1->isReadIndirect() && mo2->isMemLoad()))
      return false;

    return true;
  }
}

bool Processor::isExecuteable(MI *ins) const {
  // Initialize counters for port access in register file and memory
  auto readPorts =
      std::make_unique<char[]>(registers::getNumRegisterFiles() + 1);
  auto writePorts =
      std::make_unique<char[]>(registers::getNumRegisterFiles() + 1);
  for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
    writePorts[i] = registers::getWritePorts(i);
    readPorts[i] = registers::getReadPorts(i);
  }
  writePorts[registers::getNumRegisterFiles()] = memory::getWritePorts(0);
  readPorts[registers::getNumRegisterFiles()] = memory::getReadPorts(0);

  if (!isNotParallelMemorySystemAccess(ins))
    return false;

  int virtRead = 0, virtWrite = 0;
  int currentSlot = 0;
  while (currentSlot < getIssueSlotNumber()) {
    MO *op = ins->getOperations()[currentSlot];
    if (op == NULL) { // the operation is not (yet) set.
      currentSlot++;
      continue;
    }

    OPtype *types = op->getTypes();
    char32_t *arguments = op->getArguments();
    char *dir = op->getDirections();
    int maxArg = op->getArgNumber();

    if (op->isX2Operation()) {
      // suffix X2 implies two writes
      OPtype *types = op->getTypes();
      char *dir = op->getDirections();
      if (types[4] == REG) {
        dir[4] = WRITE;
      }

      if (types[1] == REG && types[6] == REG && types[2] == REG &&
          types[5] == REG) {
        if ((arguments[1] == arguments[6]) ||
            (arguments[2] == arguments[5])) { // example   SUB_32_X2 VxR0+VxR1,
                                              // VxR3+VxR4, VxR4+VxR3
          if (isLog(LOG_M_CHECK_EXEC)) {
            ins->writeOutReadable(cout);
            LOG_OUTPUT(LOG_M_CHECK_EXEC,
                       "Unable to use given X2 source registers\n");
          }
          return false;
        }
        if (((arguments[1] == arguments[2]) &&
             (arguments[5] != arguments[6])) ||
            ((arguments[1] != arguments[2]) &&
             (arguments[5] == arguments[6]))) { // example  SUB_32_X2 VxR0+VxR1,
                                                // VxR3+VxR4, VxR3+VxR5
          if (isLog(LOG_M_CHECK_EXEC)) {
            ins->writeOutReadable(cout);
            LOG_OUTPUT(LOG_M_CHECK_EXEC,
                       "Unable to use given X2 source registers\n");
          }
          return false;
        }
      }
    }

    for (int i = 0; i < maxArg; i++) {
      if (types[i] == REG) {
        if (i >= 4 &&
            arguments[i] == arguments[i - 4]) // non-X2 operand in X2 operation
                                              // (e.g., MIXL_32_X2 V0R26+V0R27
                                              // V1R2+V1R3   >> V1R21 <<)
          continue;
        int regfile = registers::getRegFile(arguments[i]);
        if (registers::isIndFirReg(arguments[i])) {
          // indirect fir registers are an indicator of memory access
          regfile = registers::getNumRegisterFiles();

          // Check following MOs for IndFirReg
          int j = currentSlot + op->opLengthMultiplier();
          while (j < getIssueSlotNumber()) {
            MO *op2 = ins->getOperations()[j];
            if (op2 == NULL) {
              ++j;
              continue;
            }

            OPtype *types2 = op2->getTypes();
            char32_t *arguments2 = op2->getArguments();
            int maxArg2 = op2->getArgNumber();

            if (!strncmp(op->getOperation()->getBaseName().c_str(), "SMV", 3) ||
                !strncmp(op2->getOperation()->getBaseName().c_str(), "SMV",
                         3)) {
              j += op2->opLengthMultiplier();
              continue;
            }

            for (int k = 0; k < maxArg2; ++k) {
              if (types2[k] == REG && registers::isIndFirReg(arguments2[k])) {
                int firreg1 = registers::getFirRegNumber(arguments[i]);
                int firreg2 = registers::getFirRegNumber(arguments2[k]);
                int grp1 =
                    firreg1 / registers::getParallelExecutableFirBlockSize();
                int grp2 =
                    firreg2 / registers::getParallelExecutableFirBlockSize();
                if (grp1 == grp2) {
                  LOG_OUTPUT(LOG_M_CHECK_EXEC,
                             "Parallel fir access not allowed in MOs %d+%d\n",
                             op->getLineNumber(), op2->getLineNumber());
                  return i != k;
                }
              }
            }
            j += op2->opLengthMultiplier();
          }
        }
        if (regfile < 0) {
          // Count access to virtual registers
          if (registers::getVirtualRegisterNumber(arguments[i]) >= 0) {
            if ((dir[i] & WRITE) && !(dir[i] & PSEUDOWRITE))
              virtWrite++;
            if ((dir[i] & READ) && !(dir[i] & PSEUDOREAD))
              virtRead++;
          }
          continue;
        }
        if ((dir[i] & WRITE) && !(dir[i] & PSEUDOWRITE) &&
            !registers::isDummyRegister(arguments[i])) {
          if (writePorts[regfile]-- == 0) {
            if (op->isReorderable()) {
              LOG_OUTPUT(LOG_M_CHECK_EXEC,
                         "[proc] trying to access to many write ports %d: \n",
                         regfile);
              if (isLog(LOG_M_CHECK_EXEC))
                ins->writeOutReadable(cout);
              return false;
            } else {
              LOG_OUTPUT(
                  LOG_M_ALWAYS,
                  "[proc] [line %d] trying to access too many write ports\n",
                  op->getLineNumber());
              EXIT_ERROR
            }
          }
        }
        if ((dir[i] & READ) && !(dir[i] & PSEUDOREAD) &&
            !registers::isDummyRegister(arguments[i])) {
          if (readPorts[regfile]-- == 0) {
            if (op->isReorderable()) {
              LOG_OUTPUT(LOG_M_CHECK_EXEC,
                         "trying to access to many read ports %d: ", regfile);
              if (isLog(LOG_M_CHECK_EXEC))
                ins->writeOutReadable(cout);
              return false;
            } else {
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[line %d] trying to access to many read ports\n",
                         op->getLineNumber());
              EXIT_ERROR
            }
          }
        }

        if ((i == 0 || i == 4) &&
            op->getOperation()->getName().compare(0, 2, "MV") == 0 &&
            !strncmp(op->getOperation()->getCond(), "CR", 2)) {
          if (readPorts[regfile]-- == 0) {
            if (op->isReorderable()) {
              LOG_OUTPUT(LOG_M_CHECK_EXEC,
                         "trying to access to many read ports %d: ", regfile);
              if (isLog(LOG_M_CHECK_EXEC))
                ins->writeOutReadable(cout);
              return false;
            } else {
              LOG_OUTPUT(LOG_M_ALWAYS,
                         "[line %d] trying to access to many read ports\n",
                         op->getLineNumber());
              EXIT_ERROR
            }
          }
        }
      }
      if (types[i] == Immediate && (dir[i] & WRITE) &&
          op->getOperation()->getName().compare(0, 5, "STORE") == 0) {
        if (writePorts[registers::getNumRegisterFiles()]-- == 0) {
          if (op->isReorderable()) {
            LOG_OUTPUT(LOG_M_CHECK_EXEC,
                       "trying to access to many write ports on memory\n");
            return false;
          } else {
            LOG_OUTPUT(
                LOG_M_ALWAYS,
                "[line %d] trying to access to many write ports on memory\n",
                op->getLineNumber());
            EXIT_ERROR
          }
        }
      }
      if (types[i] == Immediate && (dir[i] & READ) &&
          op->getOperation()->getName().compare(0, 4, "LOAD") == 0) {
        if (readPorts[registers::getNumRegisterFiles()]-- == 0) {
          if (op->isReorderable()) {
            LOG_OUTPUT(LOG_M_CHECK_EXEC,
                       "trying to access to many read ports on memory\n");
            return false;
          } else {
            LOG_OUTPUT(
                LOG_M_ALWAYS,
                "[line %d] trying to access to many read ports on memory\n",
                op->getLineNumber());
            EXIT_ERROR
          }
        }
      }
    }
    currentSlot += op->opLengthMultiplier();
  }
  for (uint i = 0; i < registers::getNumRegisterFiles(); i++) {
    virtWrite -= writePorts[i];
    virtRead -= readPorts[i];
  }
  if ((virtRead > 0 || virtWrite > 0) &&
      registers::getLastDummyRegNumber() < 0) {
    return false;
  }
  // let the vector units do their check.
  bool valid = true;
  int part = 0;
  MO **ops = ins->getOperations();
  for (auto p = Units.begin(); p != Units.end(); p++) {
    VectorUnit *i = (*p);
    valid &= i->isExecuteable(ops + part);
    part += i->getNumIssueSlots();
  }
  return valid;
}

int Processor::getIssueSlotNumber() const { return issueSlots; }

std::string Processor::convertFU2CSV() const {
  // map of FU name to count
  std::map<std::string, int> fuCount;
  // map fu name to latency
  std::map<std::string, int> fuLatency;
  if (Units.size() != 1) {
    throw std::runtime_error(
        "Only one VectorUnit is supported. Please check your config file.");
  }
  for (auto unit : *Units[0]->getUnits()) {
    auto op = unit->getAllOperations()->begin();
    if (fuCount.find(unit->getFUname()) == fuCount.end()) {
      fuCount[unit->getFUname()] = 1;
      fuLatency[unit->getFUname()] = unit->getLatency();
    } else {
      fuCount[unit->getFUname()]++;
    }
  }

  // iterate through fus and export FU;count;latency
  std::string csv = "FU;count;latency\n";
  for (auto fu : fuCount) {
    csv += fu.first + ";" + std::to_string(fu.second) + ";" +
           std::to_string(fuLatency[fu.first]) + "\n";
  }
  return csv;
}