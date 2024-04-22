// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_GENERATIONMANAGER_H
#define SCHEDULER_GENERATIONMANAGER_H

#include "PopulationEnergyAllocation.h"

namespace portOptReg {
class GenerationManager {

private:
  const Program *_ins;
  const VirtualRegisterMap *_map;
  const rdg::RDG &_rdg;
  uint *_seed;
  const ass_reg_t *_blocked;
  const Processor *_pro;
  const RegisterCoupling &_couplings;

  size_t COPY;
  size_t RANDOM;
  size_t COMBINE;
  size_t COMBINEMUTATE;
  size_t MUTATE;

  //  int tournamentSize = params.raRegTournamentSize;
  //  float mutationProb = params.raMutationRate;
  const Context &_ctx;
  size_t _popSize;
  double minImprovmentStep;
  std::vector<double> improvements;

  unique_ptr<PopulationEnergyAllocation> parentGeneration;
  unique_ptr<PopulationEnergyAllocation> childGeneration;

  EnergyChromosome bestIndividual;

  shared_ptr<RegisterLivelihood> registerLivelihood;
  ga_stats::ChromosomesSet processed_chromosomes;

  ga_stats::ChromosomesSet processed_chromosomesGenerationManager;

public:
  GenerationManager(size_t popSize, const Program *ins,
                    const VirtualRegisterMap *map, const rdg::RDG &rdg,
                    const ass_reg_t *blocked, const RegisterCoupling &couplings,
                    const Processor *pro, size_t Copy, size_t Random,
                    size_t Combine, size_t combineMutate, size_t mutate,
                    uint *seed, const Context &ctx)
      : _popSize(popSize), _ins(ins), _map(map), _rdg(rdg), _blocked(blocked),
        COPY(Copy), RANDOM(Random), COMBINE(Combine),
        COMBINEMUTATE(combineMutate), MUTATE(mutate), _pro(pro),
        _couplings(couplings), _seed(seed), _ctx(ctx) {
    registerLivelihood =
        make_shared<RegisterLivelihood>(ins, map, rdg, blocked);
    childGeneration = make_unique<PopulationEnergyAllocation>(
        popSize, ins, map, rdg, blocked, couplings, pro, COPY, RANDOM, COMBINE,
        seed, ctx, registerLivelihood);
    parentGeneration = make_unique<PopulationEnergyAllocation>(
        popSize, ins, map, rdg, blocked, couplings, pro, COPY, RANDOM, COMBINE,
        seed, ctx, registerLivelihood);
    initPopulation();
    int n = 3;
  }

  ~GenerationManager() {}

  void initPopulation() {
    childGeneration->setOwnsIndividuals(false);
    childGeneration->evaluate();
    bestIndividual = childGeneration->getIndividual(0);
    minImprovmentStep =
        bestIndividual.getTransitionEnergy() * params.raRegMinImprovementFrac;
  }

  void nextGeneration() {
    parentGeneration = std::move(childGeneration);
    childGeneration = make_unique<PopulationEnergyAllocation>(
        _popSize, _ins, _map, _rdg, _blocked, _couplings, _pro, COPY, RANDOM,
        COMBINE, _seed, _ctx, registerLivelihood);
  }

  void performElitism() {
    // is this sort necessary? shouldn't it already be sorted?
    parentGeneration->sort();
    for (size_t i = 0; i < COPY; ++i) {
      EnergyChromosome copied = parentGeneration->getIndividual(i);

      childGeneration->setIndividual(copied, i);
      childGeneration->setCopyOrigin(i);
      ga_stats::add_processed(
          childGeneration->getIndividual(i).getNumberSequence(),
          childGeneration->getIndividual(i).getFitness(),
          processed_chromosomes);
    }
  }

  /**
   *
   * @return True if failedToFindIndividuals was below threshold, false if
   * mutation is useless
   */
  void mutationOperations() {

    int combineAbort = 0;
    std::vector<int> combineRepetitions;
    int combineMutateAbort = 0;
    std::vector<int> combineMutateRepetitions;
    int mutationAbort = 0;
    std::vector<int> mutationRepetitions;
    int randomAbort = 0;
    std::vector<int> randomRepetitions;
    //      LOG_OUTPUT(LOG_M_RA_DEBUG, "Mutation Operation %d: Child Print\n",
    //      generation); childGeneration->print();
    int indexCounter = COPY;

    // #pragma omp for nowait
    int combineMutateCounter = 0;
    for (size_t i = 0; i < COMBINE; ++i) {
      bool doublicate = false;
      int counter = 0;

      std::pair<bool, ga_stats::ChromosomeFitness> r;
      EnergyChromosome child;
      do {
        counter++;

        child = parentGeneration->combine(_ctx);
        if (combineMutateCounter < COMBINEMUTATE) {
          child.mutateGene(_blocked, _seed, CHROMOSOME::COMBMUT, _ins);
          child.appendCombineMutationNumberDifferentVirtRegs();
        }
        std::pair<bool, ga_stats::ChromosomeFitness> r =
            ga_stats::already_processed(child.getNumberSequence(),
                                        processed_chromosomes);
        doublicate = r.first;
        if (doublicate) {
          if (combineMutateCounter < COMBINEMUTATE) {
            combineMutateRepetitions.push_back(counter);
          } else {
            combineRepetitions.push_back(counter);
          }
        } else {
          ga_stats::add_processed(child.getNumberSequence(), child.getFitness(),
                                  processed_chromosomes);
        }

      } while (doublicate and counter < params.raRegDoublicateAbort);
      if (doublicate) {
        child = registerLivelihood->getNewRandomChromosome(_seed);
      }
      // track logs
      if (combineMutateCounter < COMBINEMUTATE) {
        combineMutateCounter++;
        if (doublicate) {
          combineMutateAbort++;
        }
      } else {
        if (doublicate) {
          combineAbort++;
        }
      }

      childGeneration->setIndividual(child, indexCounter);
      indexCounter++;
    }

    for (size_t i = 0; i < MUTATE; ++i) {
      bool doublicate = false;
      int counter = 0;
      EnergyChromosome child;

      do {
        counter++;

        auto parent = parentGeneration->tournamentSelect(
            params.raRegTournamentSize, _seed);
        child = registerLivelihood->getNewChromosome();
        child.copyPhysical(parent);
        int id = parentGeneration->getIndex(parent);
        child.mutateGene(_blocked, _seed, CHROMOSOME::MUTATE, _ins);
        child.setParent1(parent, id);
        child.setNumberDifferentVirtRegs(parent);

        // remove doublicates
        std::pair<bool, ga_stats::ChromosomeFitness> r =
            ga_stats::already_processed(child.getNumberSequence(),
                                        processed_chromosomes);
        doublicate = r.first;
        if (doublicate) {
          mutationRepetitions.push_back(counter);
        } else {
          ga_stats::add_processed(child.getNumberSequence(), r.second,
                                  processed_chromosomes);
        }

      } while (doublicate and counter < params.raRegDoublicateAbort);
      if (doublicate) {
        child = registerLivelihood->getNewRandomChromosome(_seed);
        mutationAbort++;
      }
      childGeneration->setIndividual(child, indexCounter);
      indexCounter++;
    }

    for (size_t i = 0; i < RANDOM; ++i) {
      bool doublicate = false;
      int counter = 0;
      EnergyChromosome child;

      do {
        counter++;

        child = registerLivelihood->getNewRandomChromosome(_seed);
        // remove doublicates
        std::pair<bool, ga_stats::ChromosomeFitness> r =
            ga_stats::already_processed(child.getNumberSequence(),
                                        processed_chromosomes);
        doublicate = r.first;
        if (doublicate) { // doublicate
          randomRepetitions.push_back(counter);
        } else { // new individuum
          ga_stats::add_processed(child.getNumberSequence(), r.second,
                                  processed_chromosomes);
        }

      } while (doublicate and counter < params.raRegDoublicateAbort);
      if (doublicate) {
        child = registerLivelihood->getNewRandomChromosome(_seed);
        randomAbort++;
      }
      childGeneration->setIndividual(child, indexCounter);
      indexCounter++;
    }
    // log repetitions!
    if (LOG_M_REG_ENERGY) {
      string s;
      s += "Combine Stats:\n";
      s += "Combine Aborts          = " + std::to_string(combineAbort) +
           " - PERCENT of Total = " +
           std::to_string(
               int((float(combineAbort) / (COMBINE - COMBINEMUTATE)) * 100)) +
           "\n";
      s += "Combine Mutation Aborts = " + std::to_string(combineMutateAbort) +
           " - PERCENT of Total = " +
           std::to_string(
               int((float(combineMutateAbort) / (COMBINEMUTATE)) * 100)) +
           "\n";
      s += "Muation Aborts          = " + std::to_string(mutationAbort) +
           " - PERCENT of Total = " +
           std::to_string(int((float(mutationAbort) / MUTATE) * 100)) + "\n";
      s += "Random Aborts           = " + std::to_string(randomAbort) +
           " - PERCENT of Total = " +
           std::to_string(int((float(randomAbort) / RANDOM) * 100)) + "\n";
      LOG_OUTPUT(LOG_M_REG_ENERGY, s.c_str());
    }
  }

  void evaluateNonCopyPopulation() {
    //    childGeneration->evaluateNonCopy();
    childGeneration->evaluate();
  }

  int getDoublications() {
    int doublicates = 0;
    for (size_t i = 0; i < childGeneration->size(); ++i) {
      //      cout << childGeneration->getIndividual(i)->hash() << endl;
      auto sequence = childGeneration->getIndividual(i).getNumberSequence();
      std::pair<bool, ga_stats::ChromosomeFitness> r =
          ga_stats::already_processed(
              childGeneration->getIndividual(i).getNumberSequence(),
              // ga_stats::NumberSequence(c->getGenes(), c->length()),
              processed_chromosomesGenerationManager);

      if (r.first) {
        doublicates++;
      } else {
        ga_stats::add_processed(
            childGeneration->getIndividual(i).getNumberSequence(), r.second,
            processed_chromosomesGenerationManager);
      }
    }
    return doublicates;
  }

  void evaluatePopulation() { childGeneration->evaluate(); }

  //  void printInitLogs() {
  //    LOG_OUTPUT(LOG_M_RA_DETAIL, "%s Fitness of initial population: %s\n",
  //               _ctx.asString().c_str(),
  //               bestIndividual.getFitness().toString().c_str());
  //
  //    if (params.printRegPopulation) {
  //      LOG_OUTPUT(LOG_M_ALWAYS, "\nGeneration 0\n");
  //      childGeneration->sort();
  //      printPopulation(0);
  //    }
  //  }

  void sort() { childGeneration->sort(); }

  bool findNewBest() {
    improvements.push_back(
        childGeneration->getIndividual(0).getTransitionEnergy());
    if (compareTransitionEnergy(
            childGeneration->getIndividual(0)
                .getFitness()
                .getTransitionEnergy(),
            bestIndividual.getFitness().getTransitionEnergy())) {
      bestIndividual = childGeneration->getIndividual(0);
      return true;
    } else {
      return false;
    }

    childGeneration->deleteRegisterMappings();
    parentGeneration->deleteNonCopyIndividuals();
  }

  int getGenerations4MinImprovement() {
    if (improvements.size() < params.raRegMinRounds) {
      return -1;
    } else {
      int lastElement = improvements.size() - 1;
      for (int i = improvements.size() - 2; i > 0; i--) {
        if (abs(improvements[lastElement] - improvements[i]) >
            minImprovmentStep) {
          return improvements.size() - i;
        }
      }
      return improvements.size();
    }
  }

  void printPopulation(int generation, int noImprovement,
                       bool fullChromosomes = false) {
    if (params.printRegPopulation) {
      LOG_OUTPUT(LOG_M_ALWAYS, "\nGeneration %d, no Improvement %d\n",
                 generation, noImprovement);
      childGeneration->print(fullChromosomes);
    }
    if (!params.raRegPopulationVerificationDir.empty()) {
      childGeneration->write2File(generation);
    }
  }

  void printParentPopulation(bool fullChromosomes = false) {
    LOG_OUTPUT(LOG_M_ALWAYS, "\nParent Population\n");
    parentGeneration->print(fullChromosomes);
  }

  void printGeneSimilarity() {
    double diff = 0.00001;
    childGeneration->printGeneSimilarity(bestIndividual);
  }

  void setHeuristicIndividual(VirtualRegisterMap *heuristicMap) {
    if (heuristicMap) {
      childGeneration->getIndividualRef(1).setVirtualRegisterMap(heuristicMap);
      childGeneration->getIndividualRef(1).setOrigin(
          CHROMOSOME::Origin::HEURISITIC);
    }
  }

  EnergyChromosome getBestIndividual() { return bestIndividual; }

  EnergyChromosome getBestChild() { return childGeneration->getIndividual(0); }
};
} // namespace portOptReg

#endif // SCHEDULER_GENERATIONMANAGER_H
