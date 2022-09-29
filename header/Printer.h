// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_PRINTER_H
#define SCHEDULER_PRINTER_H

#include "../Processor/ProcessorConfiguration.h"
#include "assembler.h"
#include "listsearch.h"
#include "rdg.h"
#include "readFile.h"

class Printer {
private:
  std::string asmContent;
  unique_ptr<Processor> pro;
  std::vector<SLM *> slms;
  vector<unique_ptr<ListSearch>> listSearchs;
  vector<unique_ptr<Program>> programs;
  vector<int> notSchedulableCSCRs;
  vector<unique_ptr<rdg::RDG>> rdgs;
  int previousSLMCounter = 0;

  void generateListSearch();

public:
  Printer();
  void printGraph();
};

#endif // SCHEDULER_PRINTER_H
