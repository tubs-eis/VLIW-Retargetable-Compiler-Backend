// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "functionalunit.h"
#include "global.h"

#include "rapidxml-1.13/rapidxml.hpp"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

using namespace rapidxml;
std::vector<FunctionalUnit *> allFU;

int FunctionalUnit::counter = 0;

void getSuffixesHelper(rapidxml::xml_node<> *root, char *name, char *Message) {
  for (xml_node<> *operation_node = root->first_node(name); operation_node;
       operation_node = operation_node->next_sibling()) {
    if (strcmp(operation_node->name(), name))
      continue; // otherwise all other siblings would be considered.
    LOG_OUTPUT(LOG_M_INIT_PROC, "Found a new %s\n", Message);
    Operation suffix(operation_node, -1);
    for (std::vector<FunctionalUnit *>::iterator fu = allFU.begin();
         fu != allFU.end(); fu++) {
      FunctionalUnit *f = *fu;
      f->addSuffix(&suffix);
    }
  }
}

void deleteFUs() {
  for (std::vector<FunctionalUnit *>::iterator fu = allFU.begin();
       fu != allFU.end(); fu++) {
    delete *fu;
  }
}

void initFU(rapidxml::xml_node<> *start) {
  if (start == NULL)
    return;
  for (xml_node<> *operation_node = start->first_node("FU"); operation_node;
       operation_node = operation_node->next_sibling()) {
    if (strcmp(operation_node->name(), "FU"))
      continue; // there should be only FU in here, but if not. ignore them.
    LOG_OUTPUT(LOG_M_INIT_PROC, "Found a new Functional Unit definition: %s\n",
               operation_node->first_attribute("name")->value());
    char *name = operation_node->first_attribute("name")->value();
    bool found = false;
    for (std::vector<FunctionalUnit *>::iterator fu = allFU.begin();
         fu != allFU.end(); fu++) {
      FunctionalUnit *f = *fu;
      if (!strcmp(f->getFUname(), name)) {
        found = true;
        f->extractContent(operation_node);
        break;
      }
    }
    if (!found)
      allFU.push_back(new FunctionalUnit(operation_node));
  }
  getSuffixesHelper(start, (char *)"suf", (char *)"Suffix");
  getSuffixesHelper(start, (char *)"imm", (char *)"Immediate");
  getSuffixesHelper(start, (char *)"cond", (char *)"Condition");
  getSuffixesHelper(start, (char *)"size", (char *)"Size Order");
  //  printUnits();
}

void printUnits() {
  cout << "printing FU unit stats" << endl;
  for (FunctionalUnit *fu : allFU) {
    cout << "FU: " << fu->getFUname() << endl;
    for (Operation *op : *fu->getAllOperations()) {
      cout << op->getBaseName() << endl;
    }
    cout << endl;
  }
}

bool FunctionalUnit::isExecuteable(MO *op) {
  std::string searching = op->getOperation()->getName();
  return OPNames.find(searching) != OPNames.end();
}

bool FunctionalUnit::contains(string op) {
  return OPNames.find(op) != OPNames.end();
}

Operation *FunctionalUnit::getNOPOperation() { return NOP; }

bool FunctionalUnit::hasNOP() { return getNOPOperation() != NULL; }

char *FunctionalUnit::getFUname() { return UnitName; }

FunctionalUnit *getFU(char *FUname) {
  for (std::vector<FunctionalUnit *>::iterator fu = allFU.begin();
       fu != allFU.end(); fu++) {
    FunctionalUnit *f = *fu;
    if (!strcmp(FUname, f->getFUname()))
      return f;
  }
  LOG_OUTPUT(LOG_M_ALWAYS, "Functional Unit %s could not be found.\n", FUname);
  for (std::vector<FunctionalUnit *>::iterator fu = allFU.begin();
       fu != allFU.end(); fu++) {
    LOG_OUTPUT(LOG_M_ALWAYS, "%s\n", (*fu)->getFUname());
  }
  EXIT_ERROR;
  return NULL;
}

FunctionalUnit::~FunctionalUnit() {
  for (std::vector<Operation *>::iterator it = OP.begin(), end = OP.end();
       it != end; it++)
    delete *it;
  free(UnitName);
}

FunctionalUnit::FunctionalUnit(rapidxml::xml_node<> *start) {
  id = counter++;
  NOP = NULL;
  char *temp = start->first_attribute("name")->value();
  if (strlen(temp) == 0) {
    LOG_OUTPUT(LOG_M_ALWAYS, "Functional Unit names must not be empty!\n");
    EXIT_ERROR;
  }
  UnitName = (char *)malloc(strlen(temp) + 1);
  strcopy(UnitName, temp);
  LOG_OUTPUT(LOG_M_INIT_PROC, "Adding new functional unit %s \n", UnitName);

  extractContent(start);
}

void FunctionalUnit::extractContent(rapidxml::xml_node<> *start) {
  for (int i = 0; i < MAXARGNUMBER; i++)
    dir[i] = READ;
  for (xml_node<> *parameterNode = start->first_node("direction");
       parameterNode; parameterNode = parameterNode->next_sibling()) {
    if (strcmp(parameterNode->name(), "direction"))
      continue; // otherwise all other siblings would be considert.
    int i = -1;
    char *tmp = parameterNode->first_attribute("pos")->value();
    if (tmp != NULL)
      i = atoi(tmp);
    else {
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "You must define a position for the direction!\n");
      EXIT_ERROR;
    }
    if (i > MAXARGNUMBER) {
      LOG_OUTPUT(LOG_M_ALWAYS, "The defined position (%d) is to high!\n", i);
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
      LOG_OUTPUT(LOG_M_ALWAYS, "The direction type (%d) is not known!\n", tmp);
      EXIT_ERROR
    }
    LOG_OUTPUT(LOG_M_INIT_PROC, "Direction %d is %s\n", i, tmp);
  }

  // Iterate over the Operations
  for (xml_node<> *operation_node = start->first_node("opp"); operation_node;
       operation_node = operation_node->next_sibling()) {
    if (strcmp(operation_node->name(), "opp"))
      continue; // we just want to see the Operations for the Moment.
    LOG_OUTPUT(LOG_M_INIT_PROC, "found a new Operation\n");
    OP.push_back(new Operation(operation_node, id, dir));
  }
  getSuffixes(start);

  xml_node<> *latNode = start->first_node("lat");
  if (latNode != NULL) {
    int lat = atoi(latNode->first_attribute("value")->value());
    for (std::vector<Operation *>::iterator p = OP.begin(); p != OP.end();
         p++) {
      Operation *tmp = (*p);
      tmp->setLatency(lat);
    }
  }

  for (std::vector<Operation *>::iterator p = OP.begin(); p != OP.end(); p++) {
    Operation *tmp = (*p);
    OPNames.insert(tmp->getName());
    if (!strcmp(tmp->getName().c_str(), "NOP"))
      NOP = tmp;
  }
}

void FunctionalUnit::addSuffix(Operation *op) {
  std::vector<Operation *> copy(OP);
  for (std::vector<Operation *>::iterator p = copy.begin(); p != copy.end();
       p++) {
    Operation *tmp = (*p)->addSuffix(op);
    if (tmp != NULL) {
      OP.push_back(tmp);
      OPNames.insert(tmp->getName());
    }
  }
}

void FunctionalUnit::getSuffixesHelper(rapidxml::xml_node<> *root, char *name,
                                       char *Message) {
  for (xml_node<> *operation_node = root->first_node(name); operation_node;
       operation_node = operation_node->next_sibling()) {
    if (strcmp(operation_node->name(), name))
      continue; // otherwise all other siblings would be considert.
    LOG_OUTPUT(LOG_M_INIT_PROC, "found a new %s\n", Message);
    Operation suffix(operation_node, -1);
    addSuffix(&suffix);
  }
}

void FunctionalUnit::getSuffixes(rapidxml::xml_node<> *root) {
  getSuffixesHelper(root, (char *)"suf", (char *)"Suffix");
  getSuffixesHelper(root, (char *)"imm", (char *)"Immediate");
  getSuffixesHelper(root, (char *)"cond", (char *)"Condition");
  getSuffixesHelper(root, (char *)"size", (char *)"Size Order");
}
