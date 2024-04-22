// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_ENERGYALLOCATION_H
#define SCHEDULER_ENERGYALLOCATION_H

#include "GenerationManager.h"
#include "PopulationEnergyAllocation.h"
#include "deserializer.h"

#include <fstream>

namespace portOptReg {

std::unordered_map<int, int> successful_RA_histLog, failed_RA_histLog;
static void finishedRALogs(int generation, int fitness) {
  if (fitness == 0)
    successful_RA_histLog[generation] += 1;
  else
    failed_RA_histLog[generation] += 1;
}

ga_stats::ChromosomeFitness
allocateEnergyRegister(size_t popSize, Program *ins, VirtualRegisterMap *map,
                       const rdg::RDG &rdg, ass_reg_t *blocked, Processor *pro,
                       const RegisterCoupling &couplings, size_t COPY,
                       size_t RANDOM, size_t COMBINE, size_t combineMutate,
                       size_t mutate, uint *seed, const Context &ctx,
                       VirtualRegisterMap *heuristicMap = nullptr) {

  RegisterLivelihood reg(ins, map, rdg, blocked);
  //  auto chromosome = reg.getNewRandomChromosome(seed);
  //  std::stringstream ss;
  //  ss << "Chromosome Representation:" << endl;
  //  ss << chromosome.gene2String(" ", true) << endl;

  //  cout << ss.str();

  //  LOG_OUTPUT(LOG_M_REG_ENERGY, ss.str().c_str());

  std::vector<double> doublications;

  GenerationManager generationManager(popSize, ins, map, rdg, blocked,
                                      couplings, pro, COPY, RANDOM, COMBINE,
                                      combineMutate, mutate, seed, ctx);

  int generation = 0;
  int lastBestGeneration = 0;
  int round = 0;
  int noImprovement = 0;
  vector<int> improvementSteps;
  int noMinImprovement = 0;

  generationManager.setHeuristicIndividual(heuristicMap);
  generationManager.evaluatePopulation();
  generationManager.printPopulation(generation, noImprovement = noImprovement);

  while (generation < params.raRegMaxGeneration &&
         (noImprovement < params.raRegNoImprovementAbort ||
          generation < abs(params.raRegMinRounds))) {
    bool newBest = false;
    ++generation;
    ++round;
    ++noImprovement;

    generationManager.nextGeneration();
    generationManager.performElitism();
    generationManager.mutationOperations();
    generationManager.evaluateNonCopyPopulation();
    doublications.push_back(
        (double)((double)(generationManager.getDoublications())) /
        ((double)(popSize)));
    //    generationManager.evaluatePopulation();

#if REG_GA_STATS
    for (size_t i = 0; i < COPY + COMBINE + RANDOM; ++i) {
      regGAStats[CURRENT_THREAD_NUM].add(pop->individual(COPY + i));
    }
#endif

    //    generationManager.sort();
    if (generationManager.findNewBest()) {
      newBest = true;
      noImprovement = 0;
      //      LOG_OUTPUT(LOG_M_ALWAYS, "\nFound better Individuum in Generation
      //      %d\n", generation);
    }
    noMinImprovement = generationManager.getGenerations4MinImprovement();
    LOG_OUTPUT(noMinImprovement < params.raRegNoMinImprovement,
               "Abort GA for failing minimum improvement steps of %f\n",
               params.raRegNoMinImprovement);

    generationManager.printGeneSimilarity();

    generationManager.printPopulation(generation, noImprovement);
    //    generationManager.evaluatePopulation();
    //    LOG_OUTPUT(LOG_M_ALWAYS, "New Evaluation \n");
    //      generationManager.printPopulation(generation);

    if (isLog(LOG_M_RA_SIZE_HIST) && newBest) {
      improvementSteps.push_back(generation - lastBestGeneration);
      lastBestGeneration = generation;
    }
  }

#if REG_GA_STATS
  cout << "Register allocation GA statistics for SLM " << slm_id << ": ";
  regGAStats[CURRENT_THREAD_NUM].printSummary(cout, "register allocations");
  cout << endl;
  //    regGAStats[CURRENT_THREAD_NUM].printStats(cout);
#endif

  int fitness = generationManager.getBestIndividual().getFitness().getFitness();
  // RA failed
  if (fitness != 0) {
    LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG,
               "Genetic RA could not find a Solution \n");
    return generationManager.getBestIndividual().getChromosomeFitness();
  }
  LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG, "Genetic RA ended with getFitness %d\n",
             fitness);
  if (isLog(LOG_M_REG_ENERGY_DEBUG)) {
    if (fitness == 0 && improvementSteps.empty())
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Genetic RA getFitness improved after 0 generations\n");
    for (auto &i : improvementSteps) {
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Genetic RA getFitness improved after %d generations\n", i);
    }
  }
  if (isLog(LOG_M_RA_ROUNDS_HIST))
    finishedRALogs(generation, fitness);
  flushLog();
  //  pop->releaseIndividuals();
  //  delete pop;

#if CHECK_RA_TIMING
  ra_timings.gen_CompleteTimer.stop();
  ra_timings.gen_CompleteCount += popSize * generation;
#endif

  //  delete & Transfer map;
  for (auto it = (map)->begin(); it != (map)->end(); ++it) {
    if (it->second)
      delete it->second;
  }
  map->clear();

  auto bestChild = generationManager.getBestChild();
  double childEnergy = bestChild.getTransitionEnergy();
  // transfer Registermap
  auto bestIndv = generationManager.getBestIndividual();
  double bestFitness = bestIndv.getTransitionEnergy();

  if (fabs(childEnergy - bestFitness) >
      std::numeric_limits<double>::epsilon()) {
    cout << "Child Energy is different than best Fitness" << endl;
    cout << "BestFitness: " << bestFitness << " == "
         << "Child[0]: " << childEnergy << endl;
    exit(11);
    //    throw runtime_error();
  }

  auto tempMapping =
      generationManager.getBestIndividual().getVirtualRegisterMap();

  //  ins->writeOutInstructions(cout, tempMapping, true);

  for (auto &it : *tempMapping) {
    map->insert(std::pair<const char32_t, VirtualRegisterMapping *>(it.first,
                                                                    it.second));
  }
  // delete tempMapping RegisterMap Pointer, content owner is now map
  delete tempMapping;

  std::vector<double> cycleEnergyConsumption = bestIndv.evaluateDetails(ins);
  for (int i = 0; i < ins->size(); i++) {
    ins->getMI(i)->setTransitionEnergy(cycleEnergyConsumption[i]);
  }
  ins->setCumulativeTransitionEnergy(bestFitness);

  LOG_OUTPUT(LOG_M_ALWAYS, "Power RA Generations %d\n", generation);
  if (isLog(LOG_M_REG_ENERGY_DEBUG)) {
    for (auto it : *ins) {
      LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG, "%s\n", it->to_string(map).c_str());
    }
    LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG,
               "Old Energy: %d New Calculated Energy: %d\n", bestFitness,
               bestIndv.getChromosomeFitness().getTransitionEnergy());
  }

  stringstream ss = stringstream();
  ss << "Doublications" << endl;
  for (auto doubles : doublications) {
    ss << doubles << endl;
  }
  LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG, "%s", ss.str().c_str());
  return bestIndv.getChromosomeFitness();
}

} // namespace portOptReg

#endif // SCHEDULER_ENERGYALLOCATION_H
