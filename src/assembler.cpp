// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#define SETDEBUG                                                               \
  1 // SETDEBUG must be set in ONE file before including global.h.

#include "assembler.h"
#include "../export/ScheduledMOCharacteristics.h"
#include "CompilerContext.h"
#include "gen_merging.h"
#include "gen_sched.h"
#include "label.h"
#include "portOptimalReg.h"
#include "processor.h"
#include "readFile.h"
#include "virtual_reg.h"
#include <fstream>
#include <iostream>
#include <opts.h>
using namespace std;

void Assembler::writeOutCompilableAssembler(std::ostream &out) const {
  for (auto slm : slms) {
    slm->writeOutCompilable(out);
    out << endl << endl;
  }
}

void Assembler::writeOutInstructionTransitions(std::ostream &out) const {
  out << "Total Instruction Transitions " << endl;
  for (auto slm : slms) {
    out << "SLM " << slm->getID() << " = "
        << slm->getShortestInstruction()->getInstructionTransitions() << endl;
  }
}

void Assembler::setASM(char *ASMFile) {
  std::ifstream t(ASMFile);

  if (!t.good()) {
    cerr << "File " << ASMFile << " is not accessible" << endl;
    t.close();
    EXIT_ERROR;
  }
  std::string str;

  t.seekg(0, std::ios::end);
  str.reserve(t.tellg());
  t.seekg(0, std::ios::beg);

  str.assign((std::istreambuf_iterator<char>(t)),
             std::istreambuf_iterator<char>());
  t.close();
  setASM(str);
}

void Assembler::writeCsvOutput(SLM *slm, bool error) {
  /// Schreib here
  std::ofstream outfile;
  std::string path(params.asmfile);
  std::string filebasename = path.substr(path.find_last_of("/\\") + 1);

  if (params.stats_file.length() == 0) {
    return;
  }

  outfile.open(params.stats_file, std::ios::out | std::ios::app);
  if (error) {
    outfile << filebasename << ";" << slm->getOriginalID() << ";" << -1 << ";"
            << slm->getOperations()->size() << "\n";
  } else {
    outfile << filebasename << ";" << slm->getOriginalID() << ";"
            << slm->getSize() << ";" << slm->getOperations()->size() << "\n";
  }

  outfile.close();
}

Assembler::Assembler(char *configFile) {
  // reset counter to dereference IDs
  int ID_SEP = std::numeric_limits<int>::max() / MAX_THREAD_COUNT;
  for (int i = 0; i < MAX_THREAD_COUNT; ++i) {
    MI::counter[i] = ID_SEP * i;
    MO::counter[i] = ID_SEP * i;
    SLM::counter[i] = ID_SEP * i;
  }
  scheduled = false;
  pro = new Processor(configFile);
  // ASS_DEBUG_COUT(*pro << std::endl);
  if (!pro->isValid()) {
    cerr << "this is not a valid representation of an processor." << std::endl;
    EXIT_ERROR
  }
}

Assembler::~Assembler() {
  for (auto it = slms.begin(); it != slms.end(); ++it) {
    delete *it;
  }
  delete pro;
}

void Assembler::setASM(string ASMcode) {
  std::string str = commentFree(ASMcode);
  this->asmFile = regExReplacement(str);
  slms.clear();
}

void Assembler::preScheduling(void (*f)(SLM *, Processor *)) {
  preCompile();
  for (vector<SLM *>::iterator it = slms.begin(); it != slms.end(); ++it) {
    (*f)(*it, pro);
  }
}

void Assembler::preScheduling(void (*f)(std::vector<SLM *> *, Processor *)) {
  preCompile();
  (*f)(&slms, pro);
}

void Assembler::writePrecompiled() {
  std::ofstream str(params.initialassembler);
  str << asmFile;
  str.close();
}

int simpleCompile(SLM *slm, Processor *pro, const Context &ctx) {
  slm->calculateGraph(pro, true);
  LOG_OUTPUT(LOG_M_SCHED,
             "%s Simple compile started for SLM %d (op count: %lu, minimal "
             "size: %d critical path: %d)\n",
             ctx.asString().c_str(), slm->getOriginalID(),
             slm->getOperations()->size(), slm->getMinimalSize(),
             slm->getCriticalPath());

  int opt = slm->getOptimizationLevel();
  opt = (params.optimization == 1 || opt == -1) ? params.optimization : opt;

  int f;
  if (opt <= 1) {
    int notScheduableCSCR;
    Program *instructions = gen_sched::scheduleSLM(
        0, slm, pro, notScheduableCSCR, Context::nextStage(ctx, "first"));
    if (instructions) {
      f = instructions->size();

      int failedRegs = 0;
      VirtualRegisterMap *map = NULL;
      RegisterCoupling *couplings = NULL;
      if (isPowerOptimization()) {
        // generate heuristic RA as baseline
        int failedRegsIntern = 0;
        VirtualRegisterMap *mapIntern = NULL;
        RegisterCoupling *couplingsIntern = NULL;
        gen_sched::registerAllocation(slm, pro, instructions, &mapIntern,
                                      &couplingsIntern, failedRegsIntern,
                                      Context::nextStage(ctx, "first"));
        // allocate failed, free memory
        if (failedRegsIntern != 0) {
          if (mapIntern) {
            releaseVirtualMap(&mapIntern);
          }
          mapIntern = nullptr;
        }

        rdg::RDG *rdg = nullptr;
        // does this consider previous SLMs?
        bool hasRegs = gen_sched::prepareRegisterAllocation(
            slm, pro, instructions, &map, &couplings,
            Context::nextStage(ctx, "reg_prep"));
        if (hasRegs) {
          geneticRegisterAllocationPower(slm, pro, instructions, couplings, map,
                                         &rdg, Context::nextStage(ctx, "first"),
                                         mapIntern);

          slm->setInstructions(&instructions, &map);
        } else {
          LOG_OUTPUT(
              LOG_M_ALWAYS,
              "There are no virtual Register Available for allocation! \n"
              "Energy Transitions is estimated for physical Register.\n");
          gen_sched::registerAllocation(slm, pro, instructions, &map,
                                        &couplings, failedRegs,
                                        Context::nextStage(ctx, "first"));
        }
        // manual memory cleaning
        if (couplings) {
          deleteCouplings(*couplings);
          delete couplings;
          couplings = nullptr;
        }
      } else {
        gen_sched::registerAllocation(slm, pro, instructions, &map, &couplings,
                                      failedRegs,
                                      Context::nextStage(ctx, "first"));
      }
      slm->setInstructions(&instructions, &map);
      if (failedRegs != 0)
        f += failedRegs * failedRegs + NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET;
      slm->collectInstructionsFromThreads();
    } else {
      f = NON_SCHEDULEABLE_OFFSET + notScheduableCSCR;
    }
    LOG_OUTPUT(LOG_M_SCHED, "%s Initial getFitness for SLM %d is %d\n",
               ctx.asString().c_str(), slm->getOriginalID(), f);

  } else {
    f = gen_sched::genetic_scheduling(slm, pro,
                                      Context::nextStage(ctx, "gen_sched"));

    //    slm->print();
  }
  int size;
  if (slm->getShortestInstruction())
    size = slm->getShortestInstruction()->size();
  else
    size = (slm->getBranchOperation()
                ? (slm->getBranchOperation()->isReorderable() ? 4 : 1)
                : 0);
  LOG_OUTPUT(LOG_M_SCHED,
             "%s Simple compile for SLM %d (op count: %lu, minimal size: %d "
             "critical path: %d) returned getFitness: %d, size: %d\n",
             ctx.asString().c_str(), slm->getOriginalID(),
             slm->getOperations()->size(), slm->getMinimalSize(),
             slm->getCriticalPath(), f, size);
  return f;
}

void Assembler::compileSLM(int id) {
  preCompile();

  if (id < 0) {
    int end = slms.size();

    for (int i = 0; i < end; i++)
      compileSLM(i);
    return;
  }
  struct timespec startTime, endTime, diff;
  SLM *slm = slms.at(id);
  if (slm->getOriginalID() < params.startSLM ||
      (params.stopSLM != -1 && slm->getOriginalID() > params.stopSLM))
    return;
  clock_gettime(CLOCK_REALTIME, &startTime);

  int opt = slm->getMergeLevel();
  opt = (opt == -1) ? params.mergeLevel : opt;
  int sched = slm->getOptimizationLevel();
  sched =
      (params.optimization == 1 || sched == -1) ? params.optimization : sched;
  slm->calculateGraph(pro, true);
  LOG_OUTPUT(LOG_M_BASIC,
             "\nStart compiling SLM %d (%s). (op count: %lu, minimal size: %d, "
             "critical path: %d) o: %d, x: %d\n",
             slm->getOriginalID(), label::getLabelName(slm->getLabel()).c_str(),
             slm->getOperations()->size(), slm->getMinimalSize(),
             slm->getCriticalPath(), sched, opt);

  if (params.printSchedPopulation) {
    gen_sched::failedRACount = 0;
    gen_sched::heuristicRACount = 0;
    gen_sched::geneticRACount = 0;
  }

  //  slm->print();
  if (opt != 0) {
    // genetic X2 Merging
    gen_X2::genetic(slm, pro, Context("merge"));
  } else {
    // do not use X2 Merging
    simpleCompile(slm, pro, Context("sched"));
  }

  if (slm->getShortestInstruction() == NULL &&
      slm->getOperations()->size() != 0) {
    stringstream ss;
    ss << "ERROR: could not schedule slm " << slm->getOriginalID()
       << " starting from line "
       << slm->getOperations()->front()->getLineNumber() << endl;
    slm->releaseRegisterMapping();
    writeCsvOutput(slm, true);
    throw runtime_error(ss.str());
    //    EXIT_ERROR
  }

  if (params.nonAllocReadable) {
    std::ofstream readable(params.nonAllocReadable, ios_base::app);
    readable << "----- [ SLM " << slm->getOriginalID()
             << "] ----------------------------------------" << endl;
    slm->writeOutReadable(readable);
    readable.close();
  }
  // replace virtual register with found allocation
  if (!virtual_schedule(slm, pro)) {
    cerr << "ERROR: Found no possible placement for the virtual registers!"
         << endl;
    cerr << "ERROR: SLM: " << slm->getID() << " starting from line "
         << slm->getOperations()->front()->getLineNumber() << endl;
    slm->releaseRegisterMapping();
    EXIT_ERROR
  }
  //  }
  clock_gettime(CLOCK_REALTIME, &endTime);
  diff.tv_sec = endTime.tv_sec - startTime.tv_sec;
  diff.tv_nsec = endTime.tv_nsec - startTime.tv_nsec;
  if (diff.tv_nsec < 0) {
    diff.tv_nsec += 1000000000;
    diff.tv_sec -= 1;
  }
  Program *instructions = slm->getShortestInstruction();
  if ((!instructions || instructions->size() == 0) &&
      slm->getOperations()->size() != 0) {
    cerr << "ERROR: could not schedule slm " << slm->getID() << endl
         << "starting from line "
         << slm->getOperations()->front()->getLineNumber() << endl;
    slm->releaseRegisterMapping();
    EXIT_ERROR
  } else {
    uint size = slm->getShortestInstruction()
                    ? slm->getShortestInstruction()->size()
                    : -1;
    LOG_OUTPUT(
        LOG_M_BASIC,
        "Finished compiling SLM %d. (op count: %3lu, compiled size: %3lu, "
        "minimal size: %3d, critical path: %3d, time: %ld.%03ld s)\n",
        slm->getID(), slm->getOperations()->size(), size, slm->getMinimalSize(),
        slm->getCriticalPath(), diff.tv_sec, diff.tv_nsec / 1000000);
  }

  if (slm->getShortestInstruction())
    for (auto it = instructions->begin(), end = instructions->end(); it != end;
         it++)
      pro->replaceNull(*it);
  if (isLog(LOG_M_RA_ROUNDS_HIST)) {
    LOG_OUTPUT(LOG_M_ALWAYS, "RA Histogram after SLM %d:\n", slm->getID());
    portOptReg::printRAHists();
    portOptReg::clearRAHists();
  }
  if (params.printSchedPopulation) {
    LOG_OUTPUT(LOG_M_ALWAYS, "RA Stats: failed %d heuristic %d genetic %d\n",
               gen_sched::failedRACount, gen_sched::heuristicRACount,
               gen_sched::geneticRACount);
  }
  writeCsvOutput(slm);
}

void Assembler::preCompile() {
  // this only has to be done once.
  if (slms.size() != 0)
    return;
  slms = convert(asmFile, pro);
  if (params.optimization) {
    float sum = 0.0;
    for (vector<SLM *>::iterator it = slms.begin(); it != slms.end(); ++it) {
      sum += (*it)->getBooster();
    }
    sum /= params.optimization;
    for (vector<SLM *>::iterator it = slms.begin(); it != slms.end(); ++it) {
      (*it)->setBooster((*it)->getBooster() / sum);
    }
  }
}

void Assembler::postCompile() {
  if (scheduled)
    return;
  // we have to calculate the offsets afterwards, otherwise the offset from the
  // previous might not be known.
  for (vector<SLM *>::iterator it = slms.begin(); it != slms.end(); ++it) {
    if (it !=
        slms.begin()) { // calculate the offset, so labels can be set right.
      vector<SLM *>::iterator old = it - 1;
      (*it)->setOffset((*old)->getOffset() + (*old)->getSize());
    }
  }
  scheduled = true;
}

void Assembler::writeOutReadable(ostream &out, bool printEnergy) {
  if (printEnergy) {
    calculateTransitionEnergy();
  }
  postCompile();
  for (vector<SLM *>::iterator it = slms.begin(); it != slms.end(); ++it) {
    SLM *slm = *it;
    int slm_id = slm->getOriginalID();
    if (slm_id >= params.startSLM &&
        (params.stopSLM == -1 || slm_id <= params.stopSLM)) {
      out << "----- [ SLM " << slm_id
          << "] ----------------------------------------" << endl;
      slm->writeOutReadable(out, printEnergy);
    }
  }
}

void Assembler::writeOutDot(ostream &out) {
  out << "digraph weight{" << endl;
  preCompile();
  for (vector<SLM *>::iterator it = slms.begin(); it != slms.end(); ++it) {
    (*it)->calculateGraph(pro, true);
    (*it)->writeOutDot(pro, out);
  }
  out << "}" << endl;
}

void Assembler::writeOutBin(ostream &out) {
  postCompile();
  int size = 0;
  for (vector<SLM *>::iterator it = slms.begin(); it != slms.end(); ++it) {
    SLM *slm = *it;
    int slm_id = slm->getOriginalID();
    if (slm_id >= params.startSLM &&
        (params.stopSLM == -1 || slm_id <= params.stopSLM)) {
      slm->writeOutBin(out);
      size += slm->getSize() * pro->getIssueSlotNumber() *
              getInstructionWidth() / 8;
    }
  }
  for (; size < pro->getBinSize(); size++)
    out << (char)0;
}

void Assembler::calculateTransitionEnergy() {
  postCompile();
  //  out << "Transition Power Estimation." << endl << endl;
  for (vector<SLM *>::iterator it = slms.begin(); it != slms.end(); ++it) {
    SLM *slm = *it;
    slm->calculateTransitionEnergy();
  }
}

void Assembler::writeOutTransitionPowerEstimate(std::ostream &out) {
  if (!params.powerOptimizationFile.empty()) {
    calculateTransitionEnergy();
  }
  writeOutCompilableAssembler(out);
}

void Assembler::writeOutTransitionPowerEstimateCSV(std::ostream &out) {
  postCompile();
  //  out << "Transition Power Estimation." << endl << endl;
  for (vector<SLM *>::iterator it = slms.begin(); it != slms.end(); ++it) {
    SLM *slm = *it;
    out << "SLM" << slm->getOriginalID() << std::endl;
    slm->writeOutTransitionPowerEstimate(out);
  }
}
