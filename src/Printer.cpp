// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "Printer.h"
#include "CompilerContext.h"
#include "SLM.h"
#include "gen_sched.h"
#include "register.h"
#include "virtual_reg.h"
#include <fstream>

Printer::Printer() {
  Assembler ass(params.config);
  ass.setASM(params.asmfile);
  ass.preScheduling(virtual_init);
  asmContent = ass.getASM();
  pro = std::make_unique<Processor>(params.config);
  slms = ass.getSLMS();
  //  generateListSearch();
}

// void Printer::generateListSearch() {
//  for (int i = 0; i < slms.size(); i++) {
//    // generate List search
//    SLM *slm = slms[i];
//    slm->writeOutDot(pro.get(), "/home/stuckmann/slm.dot");
//    listSearchs.push_back(
//        std::make_unique<ListSearch>(slm->getOperations(), pro.get()));
//     ListSearch* listSearch = new ListSearch(slm->getOperations(), pro.get());
//    // calculate DDG
//    slm->calculateGraph(pro.get(), true);
//    listSearchs[0]->printSchedulable();
//    int n=3;
//  }
//
//  // Program *instructions = alg->scheduleMOs(processor, notScheduableCSCR);
//}

void Printer::printGraph() {
  std::ofstream dotGraphWeightFileStream("/home/stuckmann/slms.dot");
  //  std::ofstream dotGraphWeightFileStream(params.dotGraphWeightFile);
  dotGraphWeightFileStream << "digraph weights{\n";
  for (SLM *slm : slms) {
    std::vector<MO *> *mos = slm->getOperations();
    //    std::ofstream dotGraphWeightFile(params.dotGraphWeightFile,
    //                                     std::ofstream::out |
    //                                     std::ofstream::app);
    dotGraphWeightFileStream << "subgraph " << slm->getOriginalID() << " {\n";
    for (unsigned int i = 0; i < mos->size(); ++i) {
      auto mo = mos->at(i);
      mo->writeOutDot(dotGraphWeightFileStream);
    }
    dotGraphWeightFileStream << "\n}";
  }
  dotGraphWeightFileStream << "}\n";
  dotGraphWeightFileStream.close();
}