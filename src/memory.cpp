// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "../header/memory.h"

using namespace std;
using namespace rapidxml;

unsigned int numMemory = 0;
int *numWritePorts;
int *numReadPorts;

unsigned int memory::getWritePorts(int numMemory) {
  return numWritePorts[numMemory];
}

unsigned int memory::getReadPorts(int numMemory) {
  return numReadPorts[numMemory];
}

void memory::initMemory(xml_node<> *start) {
  if (start == NULL)
    return;
  xml_node<> *tmp = start->first_node("num-memories");
  if (tmp != NULL) {
    numMemory = atoi(tmp->value());
    numWritePorts = (int *)calloc(sizeof(int), numMemory);
    numReadPorts = (int *)calloc(sizeof(int), numMemory);
    for (uint i = 0; i < numMemory; i++) {
      numWritePorts[i] = -1;
      numReadPorts[i] = -1;
    }

    for (tmp = start->first_node("num-write-ports"); tmp;
         tmp = tmp->next_sibling()) {
      if (strcmp(tmp->name(), "num-write-ports"))
        continue;
      for (uint i = 0; i < numMemory; i++) {
        if (numWritePorts[i] == -1)
          numWritePorts[i] = atoi(tmp->value());
      }
    }
    for (tmp = start->first_node("num-read-ports"); tmp;
         tmp = tmp->next_sibling()) {
      if (strcmp(tmp->name(), "num-read-ports"))
        continue;
      for (uint i = 0; i < numMemory; i++) {
        if (numReadPorts[i] == -1)
          numReadPorts[i] = atoi(tmp->value());
      }
    }
  }
}
