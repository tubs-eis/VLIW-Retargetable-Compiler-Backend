// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "vectorunit.h"
#include "global.h"

#include "functionalunit.h"
#include "rapidxml-1.13/rapidxml.hpp"
#include "register.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <string.h>

using namespace rapidxml;

VectorUnit::VectorUnit(rapidxml::xml_node<> *start) {
  this->numIssueSlots = 1;
  regis = 0;
  xml_attribute<> *reg = start->first_attribute("registers");
  if (reg != NULL) {
    string temp(reg->value());
    string item;
    stringstream ss(temp);
    int reg;
    while (getline(ss, item, ',')) {
      reg = atoi(item.c_str());
      regis |= (1 << reg);
    }
  }
  xml_attribute<> *nis = start->first_attribute("numIssueSlots");
  if (nis != NULL) {
    this->numIssueSlots = atoi(nis->value());
  }

  this->functionalUnits =
      (int *)malloc(sizeof(int) * FunctionalUnit::getNumFU());
  for (int i = 0; i < FunctionalUnit::getNumFU(); i++) {
    functionalUnits[i] = 0;
  }

  // Iterate over the Issue Slots
  for (xml_node<> *FU_node = start->first_node("FU"); FU_node;
       FU_node = FU_node->next_sibling()) {
    if (strcmp(FU_node->name(), "FU"))
      continue; // we just want to see the issueSlots for the Moment.
    LOG_OUTPUT(LOG_M_INIT_PROC, "Found a new functional unit: %s\n",
               FU_node->first_attribute("name")->value());
    FunctionalUnit *fu = getFU(FU_node->first_attribute("name")->value());
    //		FunctionalUnit* copy = new FunctionalUnit(fu); // we make a copy
    // of the functional unit, because maybe there are modification to be made.
    //		copy->getSuffixes(FU_node);
    //		Units.push_back(copy);
    fu->getSuffixes(FU_node);
    Units.push_back(fu);
    functionalUnits[fu->getID()]++;
  }
  getSuffixes(start);
}

void VectorUnit::addSuffix(Operation *op) {
  for (std::vector<FunctionalUnit *>::iterator p = Units.begin();
       p != Units.end(); p++) {
    (*p)->addSuffix(op);
  }
}

vector<Operation *> *VectorUnit::getAllOperations() {
  vector<Operation *> *allOps = new vector<Operation *>;
  for (std::vector<FunctionalUnit *>::iterator p = Units.begin();
       p != Units.end(); p++) {
    vector<Operation *> *fOps = (*p)->getAllOperations();
    for (std::vector<Operation *>::iterator o = fOps->begin(); o != fOps->end();
         o++) {
      allOps->push_back(*o);
    }
  }
  return allOps;
}

bool VectorUnit::isValid() {
  int numNull = 0;
  for (std::vector<FunctionalUnit *>::iterator p = Units.begin();
       p != Units.end(); p++) {
    if ((*p)->hasNOP())
      ++numNull;
  }
  return numNull >= numIssueSlots;
}

void VectorUnit::getSuffixesHelper(rapidxml::xml_node<> *root, char *name,
                                   char *Message) {
  for (xml_node<> *operation_node = root->first_node(name); operation_node;
       operation_node = operation_node->next_sibling()) {
    if (strcmp(operation_node->name(), name))
      continue; // otherwise all other siblings would be considered.
    LOG_OUTPUT(LOG_M_INIT_PROC, "found a new %s\n", Message);
    Operation suffix(operation_node, -1);
    addSuffix(&suffix);
  }
}

unsigned int countOnes(char32_t value) {
  int number = 0;
  while (value) {
    if (value & 1) {
      number++;
    }
    value = value >> 1;
  }
  return number;
}

bool VectorUnit::isExecuteable(MO **ins) {
  auto copy = std::make_unique<int[]>(FunctionalUnit::getNumFU());
  for (auto i = 0; i < FunctionalUnit::getNumFU(); ++i)
    copy[i] = 0;

  int slot = 0;
  while (slot < numIssueSlots) {
    MO *op = ins[slot];
    if (op == NULL) { // this happens if it is not yet set.
      slot++;
      continue;
    }
    int fu = op->getOperation()->getFuId();
    if (fu == -1) {
      LOG_OUTPUT(LOG_M_CHECK_EXEC,
                 "The operation %s has no allocated functional unit!",
                 op->getOperation()->getName().c_str());
      return false;
    }
    copy[fu]++;
    // if it is an X2 Operation, the functional unit has to be available two
    // times.
    if (op->isX2Operation()) {
      copy[fu]++;
    }

    OPtype *types = op->getTypes();
    char32_t *arguments = op->getArguments();
    // checks all the arguments of the instruction to see if all the registers
    // can be accessed
    for (int i = 0; i < op->getArgNumber(); i++) {
      if (types[i] == REG) {
        int regfile = registers::getRegFile(arguments[i]);
        if (regfile >= 0 && !((1 << regfile) & regis)) {
          LOG_OUTPUT(LOG_M_CHECK_EXEC,
                     "Issue slot %d cannot accesss register file %d (%d)\n",
                     slot, regfile, regis);
          return false;
        }
      }
    }
    slot += op->opLengthMultiplier();
  }

  for (int i = 0; i < FunctionalUnit::getNumFU(); i++) {
    if (copy[i] > functionalUnits[i]) {
      //		    if (isLog(LOG_M_CHECK_EXEC)) {
      //		        LOG_OUTPUT(LOG_M_CHECK_EXEC, "MI is not
      // executable. MOs:\n"); 		        for (int s = 0; s <
      // numIssueSlots; ++s) { ins[s]->writeOutReadable(cout);
      //		            LOG_OUTPUT(LOG_M_CHECK_EXEC, "\n");
      //		        }
      //		    }
      LOG_OUTPUT(LOG_M_CHECK_EXEC,
                 "Not enough instances of FU %s. Would need %d but have %d.\n",
                 getFUbyID(i)->getFUname(), copy[i], functionalUnits[i]);
      return false;
    }
  }
  return true;
}

void VectorUnit::getSuffixes(rapidxml::xml_node<> *root) {
  getSuffixesHelper(root, (char *)"suf", (char *)"Suffix");
  getSuffixesHelper(root, (char *)"imm", (char *)"Immediate");
  getSuffixesHelper(root, (char *)"cond", (char *)"Condition");
  getSuffixesHelper(root, (char *)"size", (char *)"Size Order");
}
