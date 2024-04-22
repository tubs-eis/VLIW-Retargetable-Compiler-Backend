// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "moai_special.h"

#include "functionalunit.h"
#include "label.h"
#include "register.h"
#include <fstream>
#include <iostream>
#include <map>
#include <unordered_map>
#include <unordered_set>

void writeOutOps(SLM *slm, Processor *pro) {
  vector<MO *> *ops = slm->getOperations();

  std::ofstream out;
  if (slm->getID() == 1)
    out.open(params.precompiled, std::ofstream::out);
  else
    out.open(params.precompiled, std::ofstream::out | std::ofstream::app);

  out << "// ----- SLM " << slm->getID() << " -----" << endl;

  if (slm->getLabel() != (char32_t)-2)
    out << ":L_" << label::getLabelName(slm->getLabel()) << endl;
  for (vector<MO *>::iterator it = ops->begin(), end = ops->end(); it != end;
       it++) {
    (*it)->writeOutReadable(out);
    out << endl;
  }
  if (slm->getBranchOperation() != NULL) {
    slm->getBranchOperation()->writeOutReadable(out);
    out << endl;
  }
}

void MVcheck(MO *m, char *code, Processor *pro) {
  int length = strlen(code);
  Operation *op = m->getOperation();
  string name = op->getName();
  string vlgName = name.substr(0, length);
  char32_t *args = m->getArguments();
  OPtype *types = m->getTypes();
  if (vlgName == code) {
    char32_t cmp = args[2] & 0x7fff;
    if (args[2] == cmp && args[1] == 0xff) {
      vlgName = vlgName.substr(0, vlgName.length() - 1) + name.substr(length);
      Operation *sub = pro->getOperation(vlgName);
      if (sub == NULL) {
        cerr << "[shrinkImmediates] Could not find the operation: " << vlgName
             << endl;
        EXIT_ERROR;
        return;
      }
      args[1] = args[2];
      args[2] = 0;
      types[2] = Error;
      m->reSetOperation(sub);
      LOG_OUTPUT(LOG_M_ALWAYS, "[line %u] %s substitution.\n",
                 m->getLineNumber(), code);
    } else if (args[2] == cmp && args[1] == 0x0f &&
               registers::getVirtualRegisterNumber(args[0]) >= 0) {
      vlgName = vlgName.substr(0, vlgName.length() - 1) + name.substr(length);
      Operation *sub = pro->getOperation(vlgName);
      if (sub == NULL) {
        cerr << "[shrinkImmediates] Could not find the operation: " << vlgName
             << endl;
        EXIT_ERROR;
        return;
      }
      args[1] = args[2];
      args[2] = 0;
      types[2] = Error;
      m->reSetOperation(sub);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "[line %u] %s substitution. Because of a VIRTUAL register\n",
                 m->getLineNumber(), code);
    } else if (args[2] == cmp && args[1] == 0x0f) {
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "[line %u][MVcheck] is it really neccessary to use a %s\n",
                 m->getLineNumber(), code);
    }
  }
}

int maskImmediate(int immediate, int bitlen) {
  int bitmask = ((unsigned)-1) >> (32 - bitlen);
  return immediate & bitmask;
}

int signExtend(int value, int in_len, int out_len) {
  int sign = (value >> (in_len - 1) & 1);
  if (sign == 0)
    return value;
  else {
    int b = 1 << in_len;
    for (int i = 0; i < out_len - in_len; ++i) {
      value |= b;
      b <<= 1;
    }
    return value;
  }
}

/**
 * Check, if a given immediate value (signed/unsigned) can be encoded in the
 * given number of bits without losing information.
 */
bool immediateFits(char32_t immediate, int immlen, int data_width,
                   bool imm_signed, int line) {
  int v = (int)immediate >> data_width;
  if (v != -1 && v != 0)
    LOG_OUTPUT(LOG_M_ALWAYS,
               "WARNING: Immediate value has more than %d bits in line %d\n",
               data_width, line);
  int value = maskImmediate(immediate, data_width);
  int imm = maskImmediate(value, immlen);
  return (!imm_signed && value == imm) ||
         (imm_signed && value == signExtend(imm, immlen, data_width));
}

void ArithmeticCheck(MO *m, char *code, Processor *pro, int immlen, int param) {
  int length = strlen(code);
  Operation *op = m->getOperation();
  string name = op->getName();
  string vlgName = name.substr(0, length);
  char32_t *args = m->getArguments();
  if (vlgName == code) {
    bool op_signed = op->isSigned();
    int data_width = op->getDataWidth();
    if (immediateFits(args[param], immlen, data_width, op_signed,
                      m->getLineNumber())) {
      vlgName = vlgName.substr(0, vlgName.length() - 1) + name.substr(length);
      Operation *sub = pro->getOperation(vlgName);
      if (sub == NULL) {
        cerr << "[shrinkImmediates] Could not find the operation: " << vlgName
             << endl;
        EXIT_ERROR;
        return;
      }
      m->reSetOperation(sub);
      LOG_OUTPUT(LOG_M_ALWAYS, "[line %u] %s substitution\n",
                 m->getLineNumber(), code);
    }
  }
}

void shrinkImmediates(SLM *slm, Processor *pro) {
  vector<MO *> *ops = slm->getOperations();
  for (vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it) {
    MO *m = (*it);
    MVcheck(m, (char *)"MVIL", pro);
    MVcheck(m, (char *)"SMVIL", pro);
    ArithmeticCheck(m, (char *)"STOREIL", pro, 10, 1);
    ArithmeticCheck(m, (char *)"ADDIL", pro, 5, 2);
    ArithmeticCheck(m, (char *)"SUBIL", pro, 5, 2);
    ArithmeticCheck(m, (char *)"MULIL", pro, 5, 2);
    ArithmeticCheck(m, (char *)"MACIL", pro, 5, 2);
    ArithmeticCheck(m, (char *)"SADIL", pro, 5, 2);
    ArithmeticCheck(m, (char *)"MINIL", pro, 5, 2);
    ArithmeticCheck(m, (char *)"MAXIL", pro, 5, 2);
    ArithmeticCheck(m, (char *)"CLIPIL", pro, 5, 2);
    ArithmeticCheck(m, (char *)"ANDIL", pro, 5, 2);
    ArithmeticCheck(m, (char *)"ORIL", pro, 5, 2);
    ArithmeticCheck(m, (char *)"XORIL", pro, 5, 2);
    ArithmeticCheck(m, (char *)"XNORIL", pro, 5, 2);
  }
}

void MVOptimizer(SLM *slm, Processor *pro) {
  vector<MO *> *ops = slm->getOperations();
  std::map<char32_t, char32_t> maps;
  bool X2Operation = false;
  for (vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it) {
    MO *m = *it;
    X2Operation |= m->isX2Operation();
  }
  for (vector<MO *>::iterator it = ops->begin(), end = ops->end(); it != end;
       ++it) {
    MO *m = (*it);
    char32_t *args = m->getArguments();
    OPtype *types = m->getTypes();
    for (int i = 0; i < m->getArgNumber(); i++) {
      if (types[i] != REG)
        continue;
      std::map<char32_t, char32_t>::iterator result = maps.find(args[i]);
      if (result != maps.end()) {
        args[i] = result->second;
      }
    }
    Operation *op = m->getOperation();
    char *name = (char *)op->getName().c_str();
    if (strncmp(name, "MV", 2))
      continue;
    if (types[0] != REG || types[1] != REG)
      continue;
    char *cond = op->getCond();
    if (!strncmp(cond, "CR", 2))
      continue;
    if (X2Operation)
      continue;
    if (registers::getVirtualRegisterNumber(args[0]) >= 0 &&
        registers::getVirtualRegisterNumber(args[1]) >= 0) {
      cerr
          << "[line " << m->getLineNumber()
          << "] MV will be deleted from the code and the registers are renamed."
          << std::endl;
      maps.insert(std::pair<char32_t, char32_t>(args[0], args[1]));
      ops->erase(it);
      it--;
    }
  }
}

void X2Modification(MO *m) {
  if (m->isX2Operation()) {
    OPtype *types = m->getTypes();
    char *dir = m->getDirections();
    if (types[4] == REG) {
      dir[4] = dir[0];
    }
  }
}

void specialInit(SLM *slm, Processor *pro) {

  vector<MO *> *ops = slm->getOperations();
  for (vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it) {
    MO *m = (*it);

    // special treatment for the mac operation
    if (getFU((char *)"AA")->contains(m->getOperation()->getName().c_str()) &&
        strncmp(m->getOperation()->getName().c_str(), "MUL", 3)) {
      LOG_OUTPUT(LOG_M_SCHED_DETAIL, "[line %4d] Found MAC-like operation!\n",
                 m->getLineNumber());
      char32_t arg = m->getArguments()[0];
      if (pro->isX2Supported()) {
        if (registers::getRegNumber(arg) % 2 == 1) {
          cerr << "[line " << m->getLineNumber()
               << "] [MAC] the first register is uneven! "
               << registers::getName(arg) << endl;
          EXIT_ERROR;
        }
        if (m->getTypes()[4] == Error) {
          m->getArguments()[4] = arg + 1;
        } else {
          if (m->getArguments()[4] != arg + 1) {
            cerr << "[line " << m->getLineNumber()
                 << "] [MAC] the first target register is not equal the second "
                    "target register + 1! Expecting "
                 << registers::getName(arg) << "+"
                 << registers::getName(arg + 1) << endl;
            EXIT_ERROR;
          }
        }
      } else {
        if (registers::getRegFile(arg) % 2 == 1) {
          cerr << "[line " << m->getLineNumber()
               << "] [MAC] the register file is uneven! "
               << registers::getName(arg) << endl;
          EXIT_ERROR;
        }
        if (m->getTypes()[4] == Error) {
          if (registers::getVirtualRegisterNumber(arg) >= 0) {
            m->getArguments()[4] = arg + 1;
          } else
            m->getArguments()[4] = arg + registers::getNumRegister1RF();
        } else {
          if (registers::getVirtualRegisterNumber(arg) >= 0) {
          } else {
            if (m->getArguments()[4] != arg + registers::getNumRegister1RF()) {
              cerr << "[line " << m->getLineNumber()
                   << "] [MAC] the first and second target register must have "
                      "the same register number of different register banks! "
                      "Expecting "
                   << registers::getName(arg) << "+"
                   << registers::getName(arg + registers::getNumRegister1RF())
                   << endl;
              EXIT_ERROR;
            }
          }
        }
      }
      m->getTypes()[4] = REG;
      m->getDirections()[0] = READWRITE;
      m->getDirections()[4] = READWRITE;
      if (!strncmp(m->getOperation()->getName().c_str(), "MACPLZ", 6)) {
        m->getDirections()[0] = WRITE;
        m->getDirections()[4] = WRITE;
      }
      if (!strncmp(m->getOperation()->getName().c_str(), "CMUL", 4)) {
        m->getDirections()[0] = WRITE;
        m->getDirections()[4] = WRITE;
      }
      m->setArgNumber(5);
    }

    if (!strncmp(m->getOperation()->getName().c_str(), "STORERCU", 8)) {
      int lat = m->getArguments()[2];
      if (lat == 0) {
        cerr << "STORERCU latency of 0 not supported" << endl;
        EXIT_ERROR;
      }
      m->reSetLatency((int)m->getArguments()[2] + getBuswr() + getForwarding());
      m->getArguments()[2] += getBuswr() + getForwarding();
    }

    if (!strncmp(m->getOperation()->getName().c_str(), "SMV", 3)) {
      if (registers::isFirReg(m->getArguments()[0])) {
        m->reSetLatency(
            registers::getFirWriteLatency()); // on special move to a fir
                                              // register, the latency is 3
      } else if (registers::isSpecialRegNoFir(m->getArguments()[0])) {
        if (registers::isOffsetReg(m->getArguments()[0])) {
          m->reSetLatency(
              registers::getFirWriteLatency() +
              1); // on special move to a fir register, the latency is 3
        }
        if (!strncmp(registers::getName(m->getArguments()[0]).c_str(),
                     "CONDSEL", 7) ||
            !strncmp(registers::getName(m->getArguments()[0]).c_str(), "FLAGS",
                     5)) {
          m->setSMVtoSingleCondselFlag();
        }
        if (!strncmp(registers::getName(m->getArguments()[0]).c_str(),
                     "ALLCONDSEL", 7) ||
            !strncmp(registers::getName(m->getArguments()[0]).c_str(),
                     "ALLFLAGS", 5)) {
          m->setSMVtoAllCondselFlag();
        }
      } else if (registers::isPhysicalRegister(m->getArguments()[0]) ||
                 registers::isVirtualReg(m->getArguments()[0])) {
        m->reSetLatency(m->getLatency() + getForwarding());
      }
    }

    if (!strncmp(m->getOperation()->getName().c_str(), "MVIL", 4)) {
      if (m->getArguments()[1] != 0xff) {
        m->getDirections()[0] = READWRITEPSEUDOREAD;
      }
    }

    if (!strncmp(m->getOperation()->getCond(), "CS", 2)) {
      m->setCSOperation();
    }

    if (!strncmp(m->getOperation()->getCond(), "CR", 2)) {
      m->setCROperation();
      m->getDirections()[0] = READWRITEPSEUDOREAD;
      //			m->getDirections()[4] = READWRITEPSEUDOREAD;
    }

    if (!strncmp(m->getOperation()->getCond(), "CRS", 3)) {
      m->setCRSOperation();
    }

    // special treatment for the X2 instructions.
    X2Modification(m);
  }

  // bug in kavuaka chip vu_ex.vhd smv_wr
  if (pro->getProcessorName() == "kavuaka_chip") {
    for (vector<MO *>::size_type i = 0; i < ops->size(); ++i) {
      MO *m = (*ops)[i];
      if (m->getTypes()[2] == Imm32) {
        if (((m->getArguments()[2]) & (1 << (6))) &&
            (((m->getArguments()[2] >> 27) == 0x1) ||
             ((m->getArguments()[2] >> 27) == 0x0))) {
          if (!strncmp(m->getOperation()->getName().c_str(), "MVIL", 4)) {
            LOG_OUTPUT(LOG_M_ALWAYS,
                       "WARNING: [line %u] Invalid IL instruction replaced due "
                       "to chip hardware bug.\n",
                       m->getLineNumber());
            m->getArguments()[2] &= ~(0x1 << 6);
            string or_strg = "ORIL";
            or_strg.append(m->getOperation()->getSize());
            m->setOperation(pro->getOperation(or_strg.c_str()));

            string mv_strg = "MVI_64";
            MO *mv = new MO(pro->getOperation(mv_strg.c_str()));

            mv->getArguments()[0] = m->getArguments()[0];
            mv->getArguments()[1] = 0x40;
            mv->getTypes()[0] = REG;
            mv->getTypes()[1] = Immediate;
            mv->getDirections()[0] = WRITE;
            mv->getDirections()[1] = READ;
            mv->setArgNumber(2);
            mv->setLineNumber(m->getLineNumber());

            ops->insert(ops->begin() + i, mv);

          } else {
            cerr << "ERROR: [line " << m->getLineNumber()
                 << "] Invalid IL instruction due chip hardware bug. No "
                    "automatic fix possible."
                 << endl;
            EXIT_ERROR;
          }
        }
      }
    }
  }
}

void replaceHardWithVirtual(vector<SLM *> *slms, Processor *pro) {
  unordered_set<char32_t> notReplaceable;

  char32_t maxVirt =
      registers::createRegister(-1, 0); // create the minimal virtual register.

  for (std::vector<SLM *>::iterator ot = slms->begin(), send = slms->end();
       ot != send; ot++) {
    SLM *slm = *ot;
    unordered_set<char32_t> temp_able;
    vector<MO *> *ops = slm->getOperations();

    for (vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it) {
      MO *m = *it;
      char32_t *arguments = m->getArguments();
      OPtype *types = m->getTypes();
      char *dir = m->getDirections();
      for (int i = 0; i < m->getArgNumber(); i++) {
        if (types[i] == REG) {
          if (registers::getVirtualRegisterNumber(arguments[i]) >= 0) {
            // calculate the highest used virtual register.
            maxVirt = max(maxVirt, arguments[i]);
          }
          if (registers::getRegNumber(arguments[i]) >= 0) {
            if ((dir[i] & READ) && !(dir[i] & PSEUDOREAD)) {
              if (temp_able.find(arguments[i]) == temp_able.end()) {
                notReplaceable.insert(arguments[i]);
              }
            }
            if ((dir[i] & WRITE) && !(dir[i] & PSEUDOWRITE)) {
              temp_able.insert(arguments[i]);
            }
          }
        }
      }
    }
  }

  for (std::unordered_set<char32_t>::iterator it = notReplaceable.begin(),
                                              end = notReplaceable.end();
       it != end; it++) {
    LOG_OUTPUT(LOG_M_ALWAYS, "not replaceable: %s\n",
               registers::getName(*it).c_str());
  }

  maxVirt++;
  unordered_map<char32_t, char32_t> mapping;
  for (std::vector<SLM *>::iterator ot = slms->begin(), send = slms->end();
       ot != send; ot++) {
    SLM *slm = *ot;
    vector<MO *> *ops = slm->getOperations();
    for (vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it) {
      MO *m = *it;
      char32_t *arguments = m->getArguments();
      OPtype *types = m->getTypes();
      for (int i = 0; i < m->getArgNumber(); i++) {
        if (types[i] == REG && registers::getRegNumber(arguments[i]) >= 0) {
          if (notReplaceable.find(arguments[i]) == notReplaceable.end()) {
            std::unordered_map<char32_t, char32_t>::iterator find =
                mapping.find(arguments[i]);
            if (find == mapping.end()) {
              mapping.insert(
                  std::pair<char32_t, char32_t>(arguments[i], maxVirt));
              arguments[i] = maxVirt;
              maxVirt++;
            } else {
              arguments[i] = find->second;
            }
          }
        }
      }
    }
  }
}
