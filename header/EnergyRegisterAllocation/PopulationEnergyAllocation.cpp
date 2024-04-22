// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "PopulationEnergyAllocation.h"
#include "../powerEstimation/TransitionEnergyEstimator.h"
#include "EnergyChromosome.h"
#include "MI.h"
#include "deserializer.h"

namespace portOptReg {

void PopulationEnergyAllocation::setIndividual(EnergyChromosome chrm,
                                               size_t index) {
  _individuals[index] = chrm;
}

EnergyChromosome PopulationEnergyAllocation::getIndividual(size_t index) {
  return _individuals[index];
}

EnergyChromosome &PopulationEnergyAllocation::getIndividualRef(size_t index) {
  return _individuals[index];
}

void PopulationEnergyAllocation::sort() {
  std::sort(_individuals.begin(), _individuals.end(), portOptReg::smaller);
}

// void PopulationEnergyAllocation::randomizeIndividual(uint index) {
//   _individuals[index].randomize(_seed);
// }

void PopulationEnergyAllocation::evaluate() {

  // #pragma omp parallel for
  for (auto &ind : _individuals) {
    ind.evaluate(_ins);
  }

  sort();
}

void PopulationEnergyAllocation::evaluateNonCopy() {

  // #pragma omp parallel for
  for (size_t i = COPY; i < COPY + COMBINE + RANDOM; ++i) {
    EnergyChromosome &c = _individuals[i];
    if (true) {
      //      if (!c.isDuplicate()) {
      c.evaluate(_ins);
      //      }
    }
  }

  sort();
}

EnergyChromosome &PopulationEnergyAllocation::selectUniform(uint *seed) {
  bool onlyDuplicates = true;
  for (size_t i = 0; i < _popSize; ++i) {
    if (!_individuals[i].isDuplicate()) {
      onlyDuplicates = false;
      break;
    }
  }
  while (true) {
    int index = rand_r(seed) % _popSize;
    if (onlyDuplicates || !_individuals[index].isDuplicate()) {
      return _individuals[index];
    }
  }
}

EnergyChromosome &
PopulationEnergyAllocation::tournamentSelect(int tournamentSize, uint *seed,
                                             EnergyChromosome *parent1) {
  EnergyChromosome &chrm = selectUniform(seed);
  for (int i = 1; i < tournamentSize; ++i) {
    EnergyChromosome &c = selectUniform(seed);
    if (c.getFitness() < chrm.getFitness()) {
      chrm = c;
    } else {
      if (c.getFitness() == chrm.getFitness()) {
        if (parent1) {
          if (c.getAvailableGenesForCombination(*parent1).size() >
              chrm.getAvailableGenesForCombination(*parent1).size()) {
            chrm = c;
            LOG_OUTPUT(
                LOG_M_REG_ENERGY_DEBUG,
                "Tournament Select: Overriding Fitness in Favor of a more "
                "different Parent!\n");
          }
        }
      }
    }
  }
  return chrm;
}

EnergyChromosome PopulationEnergyAllocation::combine(const Context &ctx) {
  EnergyChromosome parent1 =
      tournamentSelect(params.raRegTournamentSize, _seed);
  EnergyChromosome parent2 =
      tournamentSelect(params.raRegTournamentSize, _seed);

  int searchCount = 0;
  while ((parent1.hash() == parent2.hash() or
          parent1.getAvailableGenesForCombination(parent2).size() == 0) &&
         searchCount < params.raRegTournamentAbort) {
    parent2 = tournamentSelect(params.raRegTournamentSize, _seed);
    if (searchCount > 0) {
      LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG, "TournamentSelect Retry: %d\n",
                 searchCount);
    }
    searchCount++;
  }

  if (searchCount == params.raRegTournamentAbort) {
    LOG_OUTPUT(
        LOG_M_REG_ENERGY_DEBUG,
        "%s Did not find different parents in Genetic PowerAware Register "
        "allocation\n",
        ctx.asString().c_str());
    LOG_OUTPUT(
        LOG_M_REG_ENERGY_DEBUG,
        "Combination: Tournament Selection did not find different parents!\n");
  }
  EnergyChromosome child =
      combine(parent1, parent2, _seed, params.raMutationRate);

  return child;
}

EnergyChromosome
PopulationEnergyAllocation::combine(const EnergyChromosome &parent1,
                                    const EnergyChromosome &parent2, uint *seed,
                                    float mutationRate) {

  EnergyChromosome child = registerLivelihood->getNewChromosome();
  //  stringstream ss;
  //  ss << "Combination of parent1: " << parent1.toString() << "\nParent2: " <<
  //  parent2.toString() << endl; LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG,
  //  ss.str().c_str());

  // copy parent1 to child
  child.copyPhysical(parent1);

  child.setParent1(parent1, getIndex(parent1));
  child.setParent2(parent2, getIndex(parent2));

  child.setOrigin(CHROMOSOME::COMBINE);
  child.setFitness(ga_stats::ChromosomeFitness());

  if (params.raRegCombineGreedy) {
    child.combineGreedy(parent2, seed, _ins);
  } else {
    child.combine(parent2, seed, _ins);
  }
  child.setNumberDifferentVirtRegs(parent1);
  return child;
}

int PopulationEnergyAllocation::getDoublicateCount() {
  int doublications = 0;
  ga_stats::ChromosomesSet processed_chromosomes;
  for (int i = 0; i < _individuals.size(); i++) {
    std::pair<bool, ga_stats::ChromosomeFitness> r =
        ga_stats::already_processed(
            _individuals[i].getNumberSequence(),
            // ga_stats::NumberSequence(c->getGenes(), c->length()),
            processed_chromosomes);
    if (r.first) {
      doublications++;
    }
    ga_stats::add_processed(_individuals[i].getNumberSequence(), r.second,
                            processed_chromosomes);
  }

  return doublications;
}

void PopulationEnergyAllocation::print(bool fullChromosomes) {
  stringstream ss;
  int failedCombRepair = 0;
  int combineRepairCount = 0;
  ss << "Doublicates " << getDoublicateCount() << " / " << _individuals.size()
     << std::endl;
  for (int i = 0; i < _individuals.size(); i++) {
    ss << "[EnergyAllocation] Nr. " << setw(numberResolution) << i << " "
       << _individuals[i].getChromosomeFitness().toString();
    //  if (_individuals[i].getOrigin() == CHROMOSOME::COMBINE or
    //  _individuals[i].getOrigin() == CHROMOSOME::COMBMUT or
    //  _individuals[i].getOrigin() == CHROMOSOME::COMREPAIR  ){

    //    ss << " pre: ";
    //    for (int k=0; k < _individuals[i].transitionEnergies.size(); k++){
    //      ss <<  _individuals[i].transitionEnergies[k] << " ";
    //    }

    ss << " ORIGIN " << _individuals[i].getOriginString();
    ss << _individuals[i].getDifferentNumberVirtualRegsString();
    if (fullChromosomes) { // fullChromosomes
      ss << " Genes:" << _individuals[i].toString();
    }
    if (_individuals[i].getOrigin() == CHROMOSOME::COMREPAIR) {
      combineRepairCount++;
      if (_individuals[i].getFitness().getFitness() != 0) {
        failedCombRepair++;
      }
    }
    ss << _individuals[i].printHierarchyDepth();
    ss << _individuals[i].getParentsString();
  }
  ss << endl;
  LOG_OUTPUT(LOG_M_REG_ENERGY, ss.str().c_str());
  ss = std::stringstream();
  ss << "Failed combine Repair: " << failedCombRepair << "/"
     << combineRepairCount << endl;
  if (_individuals[0].getTransitionEnergy() ==
      _individuals[1].getTransitionEnergy()) {
    ss << "Found Doublicate Transition Energy in Population" << std::endl;
  }
  //    // top 5 best
  //    // 5 random
  //    for (int i = 0; i < 1; i++) {
  //
  //      auto virtualmap = _individuals[i].getVirtualRegisterMap();
  //      bool validRegisterAllocation =
  //      _individuals[i].validRegisterAllocation(); if
  //      (validRegisterAllocation)
  //      {
  //
  //        try {
  //          TransitionEnergyEstimator estimator;
  //          for (auto it = _ins->begin(); it != _ins->end(); ++it) {
  //            auto mi = *it;
  //            estimator.push(*it, virtualmap);
  //            mi->setTransitionEnergy(estimator.getLastTransitionEnergy());
  //          }
  //          for (auto it = _ins->begin(); it != _ins->end(); ++it) {
  //            auto mi = *it;
  //            mi->writeOutReadable(ss, virtualmap, validRegisterAllocation);
  //          }
  //          if (validRegisterAllocation) {
  //            ss << "Total TransitionEnergy [pJ] = "
  //               << estimator.getTransitionEnergy() << endl;
  //          }
  //          ss << "------" << endl;
  //          releaseVirtualMap(&virtualmap);
  //        } catch (const exception &e) {
  //          cout << "EnergyChromosome::Print Population : has an error!\n";
  //          releaseVirtualMap(&virtualmap);
  //        }
  //      }
  //    }

  LOG_OUTPUT(LOG_M_REG_ENERGY_DEBUG, ss.str().c_str());
}

void PopulationEnergyAllocation::removeRegMapping(
    ga_stats::ChromosomesSet &processed_chromosomes) {
  for (EnergyChromosome c : _individuals) {
#if PORT_OPT_RA_DUP_CHRM_CHECK
    ga_stats::add_processed(
        c->getNumberSequence(),
        // ga_stats::NumberSequence(c->getGenes(), c->length()),
        ga_stats::ChromosomeFitness(c->getFitness(), c->getTransitionEnergy()),
        processed_chromosomes);
#endif
  }
  int n = 3;
}

void PopulationEnergyAllocation::deleteRegisterMappings() {}

void PopulationEnergyAllocation::deleteNonCopyIndividuals() {}

void PopulationEnergyAllocation::printGeneSimilarity(
    EnergyChromosome &bestIndividual) {
  double fitnessDiff = 0.00001;
  bool equalFitness = true;
  int totalGenes = 0;
  for (auto gene : bestIndividual.getGenes()) {
    if (gene.size() > 1) {
      totalGenes++;
    }
  }
  for (int i = 0; i < _individuals.size(); i++) {
    if (abs(bestIndividual.getTransitionEnergy() -
            _individuals[i].getTransitionEnergy()) > fitnessDiff) {
      return;
    }
    //      LOG_OUTPUT(LOG_M_ALWAYS, "\nIndividual %d: has %d/%d equal genes
    //      from best Individuum.\n", i,
    //      _individuals[i].numberEqualGenes(bestIndividual),totalGenes );
  }
}

const VirtualRegisterMap *
PopulationEnergyAllocation::getVirtualRegisterMap() const {
  return nullptr;
}
// PopulationEnergyAllocation::~PopulationEnergyAllocation() {
//  for(int i=0; i < _individuals.size(); i++) {
//    delete _individuals[i];
//  }
//}

} // namespace portOptReg
