// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <time.h>

#include "assembler.h"
#include "moai_special.h"
#include "opts.h"
#include "register.h"
#include "sched_stats.h"
#include "virtual_reg.h"
#include <limits>

#if WITH_PROFILING
#include <gperftools/profiler.h>
#endif

#if defined(_OPENMP)
#include <omp.h>
#endif

#include "../export/ScheduledMOCharacteristics.h"
#include "Constants.h"
#include <InstructionCounter.h>
#include <signal.h>

void sigfunction(int sig) {
  if (sig == SIGINT) {
    if (killed)
      exit(EXIT_FAILURE);
    else {
      killed = true;
#if WITH_PROFILING
      ProfilerStop();
#endif
    }
  }
}

void printAssemblerSettings() {
  char tmp_str[200];
  LOG_OUTPUT(LOG_M_ALWAYS,
             "########################################################\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "MOAI4K2_ASM_2 Setting:\n");

  LOG_OUTPUT(LOG_M_ALWAYS, "INPUT:\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "  Input assembler code   : %s\n", params.asmfile);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Processor configuration: %s\n", params.config);

  LOG_OUTPUT(LOG_M_ALWAYS, "GENERAL:\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "  Threads    : %d\n", params.threads);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Config file: %s\n", params.config);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Start SLM  : %d\n", params.startSLM);
  if (params.stopSLM == -1)
    LOG_OUTPUT(LOG_M_ALWAYS, "  Scheduling all SLMs\n");
  else
    LOG_OUTPUT(LOG_M_ALWAYS, "  Stop SLM   : %d\n", params.stopSLM);

  LOG_OUTPUT(LOG_M_ALWAYS, "SCHEDULING:\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "  Scheduling level : %d\n", params.optimization);
  sprintf(tmp_str, "%d", params.schedPopulation);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Population size  : %s\n",
             params.schedPopulation == -1 ? "default" : tmp_str);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Tournament size  : %d\n",
             params.schedTournamentSize);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Rounds           : %d\n", params.schedRounds);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Trial rounds     : %d\n",
             params.schedTrialRounds);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Random ratio     : %f\n",
             params.schedRandomRatio);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Split probability: %f\n", params.partProb);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Alone probability: %f\n", params.aloneProb);
  LOG_OUTPUT(LOG_M_ALWAYS, "  DDG weight      : %d\n",
             params.includeDDGWeights);
  if (params.enableRASkip)
    LOG_OUTPUT(
        LOG_M_ALWAYS,
        "  Skipping register allocation for too big SLMs (offset = %d)\n",
        params.RASkipOffset);

  LOG_OUTPUT(LOG_M_ALWAYS, "MERGING:\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "  Merge level      : %d\n", params.mergeLevel);
  sprintf(tmp_str, "%d", params.mergePopulation);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Population size  : %s\n",
             params.mergePopulation == -1 ? "default" : tmp_str);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Popsize divider  : %d\n", params.mergePopsizeDiv);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Tournament size  : %d\n",
             params.mergeTournamentSize);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Rounds           : %d\n", params.mergeRounds);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Combine ratio    : %d\n", params.mergeCmbRatio);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Mutation rate    : %7.4f\n",
             params.mergeMutationRate);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Adaptive mutation: %s\n",
             params.mergeMutateAdaptively ? "enabled" : "disabled");
  LOG_OUTPUT(LOG_M_ALWAYS, "  Adaptive slope   : %7.4f\n",
             params.mergeMutateSlope);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Recursive merge  : %s\n",
             params.recursiveMerge ? "enabled" : "disabled");
  if (params.enableSchedSkip)
    LOG_OUTPUT(LOG_M_ALWAYS,
               "  Skipping scheduling for merged SLMs with too big minimal "
               "size (offset %d)\n",
               params.schedSkipOffset);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Merge fix+virtual: %s\n",
             params.mergeFixAndVirtual ? "enabled" : "disabled");
  if (params.noMergeProbability != -1)
    LOG_OUTPUT(LOG_M_ALWAYS, "  No merge prob    : %f\n",
               params.noMergeProbability);
  else
    LOG_OUTPUT(LOG_M_ALWAYS, "  No merge prob    : disabled\n");

  LOG_OUTPUT(LOG_M_ALWAYS, "REGISTER ALLOCATION:\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "  RA level     : %d\n", params.portReg);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Heuristic    : %s\n",
             params.heuristicReg ? "enabled" : "off");
  if (params.RAPruneCount == -1)
    LOG_OUTPUT(LOG_M_ALWAYS, "  Pruning count: disabled\n");
  else
    LOG_OUTPUT(LOG_M_ALWAYS, "  Pruning count: %d\n", params.RAPruneCount);

  LOG_OUTPUT(LOG_M_ALWAYS, "ENERGY AWARE GENETIC REGISTER ALLOCATION:\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "  Population Size             : %d\n",
             params.raRegPopSize);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Random Fraction             : %f\n",
             params.raRegRandom);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Elitism                     : %f\n",
             params.raRegCopy);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Combine Fraction            : %f\n",
             params.raRegCombine);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Combine Descend Hierachy    : %f\n",
             params.raRegHierarchyDescend);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Max Root Genes to combine   : %d\n",
             params.raRegMaxCombineRoots);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Mutate Fraction             : %f\n",
             params.raRegMutate);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Max Mutate number Regs Frac : %f\n",
             params.raRegMaxMutationFraction);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Mutate after Combine        : %f\n",
             params.raRegCombineMutate);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Tournament Size             : %d\n",
             params.raRegTournamentSize);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Tournament Abort            : %d\n",
             params.raRegTournamentAbort);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Minimum Register Rounds     : %d\n",
             params.raRegMinRounds);
  LOG_OUTPUT(LOG_M_ALWAYS, "  No Improvement Abort        : %d\n",
             params.raRegNoImprovementAbort);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Max Generations             : %d\n",
             params.raRegMaxGeneration);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Greedy Combine Repair       : %s\n",
             params.raRegRepairCombineGreedy ? "true" : "false");
  LOG_OUTPUT(LOG_M_ALWAYS, "  Greedy Combine              : %s\n",
             params.raRegCombineGreedy ? "true" : "false");
  LOG_OUTPUT(LOG_M_ALWAYS, "  Greedy Mutation             : %s\n",
             params.raRegMutateGreedy ? "true" : "false");
  if (CONSTANT::powerEqualThreshold >= 0) {
    LOG_OUTPUT(LOG_M_ALWAYS, "  Power Equal Instr Transition: %f\n",
               CONSTANT::powerEqualThreshold);
  } else {
    LOG_OUTPUT(LOG_M_ALWAYS, "  Power Equal Instr Transition: Disabled\n");
  }

  LOG_OUTPUT(LOG_M_ALWAYS, "REGISTER FILE:\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "  Number register files: %d\n", numRegisterFiles);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Number of registers  : %d\n", numRegisters);
  LOG_OUTPUT(LOG_M_ALWAYS, "  Read ports           :");
  for (unsigned int i = 0; i < numRegisterFiles; ++i)
    LOG_OUTPUT(LOG_M_ALWAYS, " %d", readPorts[i]);
  LOG_OUTPUT(LOG_M_ALWAYS, "\n");
  LOG_OUTPUT(LOG_M_ALWAYS, "  Write ports          :");
  for (unsigned int i = 0; i < numRegisterFiles; ++i)
    LOG_OUTPUT(LOG_M_ALWAYS, " %d", writePorts[i]);
  LOG_OUTPUT(LOG_M_ALWAYS, "\n");
  LOG_OUTPUT(LOG_M_ALWAYS,
             "########################################################\n");
}

int main(int argc, char **argv) {

#if WITH_PROFILING
  std::cout << "Running with profiling" << std::endl;
  ProfilerStart("moai4k2_o4_x2.prof");
#endif

#ifdef PRINTOUT_DISTANCES
  for (int i = 0; i < 2048; i++)
    distances[i] = 0;
#endif

  struct timespec startTime {
  }, endTime{}, diff{};
  clock_gettime(CLOCK_REALTIME, &startTime);
  signal(SIGINT, sigfunction);
  killed = false;
  /// INIT
  opts(argc, argv);
  Assembler ass(params.config);
  ass.setASM(params.asmfile);
  if (params.portReg == -1) {
    numRegisterFiles = 2;
    numRegisters = 1024;
    for (int i = 0; i < 2; ++i) {
      writePorts[i] = 4;
      readPorts[i] = 8;
    }
  }

  printAssemblerSettings();

  if (params.initialassembler) {
    ass.writePrecompiled();
  }

  if (params.sched_stats)
    SchedulingStats::init(params.sched_stats);

    /// THREADLIMIT
#ifdef _OPENMP
  omp_set_nested(0);
  // limit the number of threads in parallel
  if (params.threads > MAX_THREAD_COUNT) {
    LOG_OUTPUT(LOG_M_ALWAYS, "This program supports at most %d threads!\n",
               MAX_THREAD_COUNT);
    params.threads = MAX_THREAD_COUNT;
  }
  if (params.threads != 0) {
    omp_set_num_threads(params.threads);
  }
#endif
  ass.preScheduling(specialInit);

  ass.preScheduling(virtualRenaming);

  if (params.replace) {
    ass.preScheduling(shrinkImmediates);
  }
  if (params.mvOptimizer) {
    ass.preScheduling(MVOptimizer);
  }

  if (params.replaceHard) {
    ass.preScheduling(replaceHardWithVirtual);
  }

  //        ass.preScheduling(virtualRegisterGraph);

  if (params.precompiled) {
    ass.preScheduling(writeOutOps);
  }

  if (params.dot != nullptr) {
    std::ofstream dot;
    dot.open(params.dot);
    ass.writeOutDot(dot);
    dot.close();
  }

  // initialisation for the virtual register allocation.
  ass.preScheduling(virtual_init);

  if (params.nonAllocReadable) {
    std::ofstream readable(params.nonAllocReadable);
    readable.close();
  }
  int end = ass.getSlmCount();
  for (int i = 0; i < end; i++) {
    ass.compileSLM(i);
  }

  /// READABLE OUTPUT
  if (params.readable != nullptr) {
    std::ofstream read;
    read.open(params.readable);
    ass.writeOutReadable(read);
    read.close();
  } else {
    ass.writeOutReadable(std::cout);
  }

  /// BINARY OUTPUT
  if (params.binary != nullptr) {
    std::ofstream bin;
    bin.open(params.binary);
    ass.writeOutBin(bin);
    bin.close();
  } // */

  if (!params.compilableAssemblerFile.empty()) {
    std::ofstream compilableEnergyFile;
    compilableEnergyFile.open(params.compilableAssemblerFile);
    ass.writeOutTransitionPowerEstimate(compilableEnergyFile);
    compilableEnergyFile.close();
  }

  if (!params.printEstimatedTransitionPowerCSV.empty()) {
    std::ofstream bin;
    bin.open(params.printEstimatedTransitionPowerCSV);
    ass.writeOutTransitionPowerEstimateCSV(bin);
    bin.close();
  }

  if (!params.powerOptimizationFile.empty()) {
    stringstream ss;
    ass.writeOutReadable(ss, true);
    LOG_OUTPUT(LOG_M_ALWAYS, ss.str().c_str());
    if (!params.printEstimatedTransitionPower.empty()) {
      std::ofstream bin;
      bin.open(params.printEstimatedTransitionPower);
      bin << ss.str();
      bin.close();
    }
  }

  if (!params.dotAsmLineFile.empty()) {
    std::ofstream dotGraphWeightFileStream(params.dotAsmLineFile,
                                           std::ofstream::out);
    ScheduledMOCharacteristics::getInstance().setEnable(true);
    ass.writeOutDot(dotGraphWeightFileStream);
    dotGraphWeightFileStream.close();
  }
  clock_gettime(CLOCK_REALTIME, &endTime);
  diff.tv_sec = endTime.tv_sec - startTime.tv_sec;
  diff.tv_nsec = endTime.tv_nsec - startTime.tv_nsec;
  if (diff.tv_nsec < 0) {
    diff.tv_nsec += 1000000000;
    diff.tv_sec -= 1;
  }

  LOG_OUTPUT(LOG_M_ALWAYS, "Total Time: %8ld.%03ld sec\n", diff.tv_sec,
             diff.tv_nsec / 1000000);
  cout << "Total Time: " << diff.tv_sec << "." << diff.tv_nsec / 1000000
       << " sec\n";
  LOG_OUTPUT(LOG_M_ALWAYS, "succesful ended\n");

#ifdef PRINTOUT_DISTANCES
  for (int i = 0; i < 2048; i++) {
    if (distances[i - 1] != 0 || distances[i] != 0 || distances[i + 1] != 0)
      fprintf(params.logFile, "%d %d\n", i, distances[i]);
  }
#endif

  SchedulingStats::release();

  if (params.logFile != stdout)
    fclose(params.logFile);

#if WITH_PROFILING
  ProfilerStop();
#endif

#if CHECK_RA_TIMING
  printRATimings();
#endif

  cout << "successfully finished!" << endl;
  return 0;
}
