// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#ifndef SCHEDULER_POPULATIONENERGYALLOCATION_H
#define SCHEDULER_POPULATIONENERGYALLOCATION_H

#include <fstream>
#include <memory>

#include "CompilerContext.h"
#include "global.h"

#include "CombineSelection.h"
#include "EnergyChromosome.h"
#include "RegisterLivelihood.h"

namespace portOptReg {
class PopulationEnergyAllocation {
private:
  vector<EnergyChromosome> _individuals;

  const Program *_ins;
  const VirtualRegisterMap *_map;
  const rdg::RDG &_rdg;
  uint *_seed;
  const ass_reg_t *_blocked;
  const Processor *_pro;
  const Context &_ctx;
  const RegisterCoupling &_couplings;

  bool ownsIndividuals;

  size_t _popSize;
  size_t COPY;
  size_t RANDOM;
  size_t COMBINE;

  ga_stats::ChromosomesSet _processed_chromosomes;

  shared_ptr<RegisterLivelihood> registerLivelihood;

  EnergyChromosome &selectUniform(uint *seed);

  EnergyChromosome combine(const EnergyChromosome &parent1,
                           const EnergyChromosome &parent2, uint *seed,
                           float mutationRate);

  int getDoublicateCount();

public:
  PopulationEnergyAllocation(size_t popSize, const Program *ins,
                             const VirtualRegisterMap *map, const rdg::RDG &rdg,
                             const ass_reg_t *blocked,
                             const RegisterCoupling &couplings,
                             const Processor *pro, size_t Copy, size_t Random,
                             size_t Combine, uint *seed, const Context &ctx,
                             shared_ptr<RegisterLivelihood> registerLivelihood)
      : _popSize(popSize), _ins(ins), _map(map), _rdg(rdg), _blocked(blocked),
        COPY(Copy), RANDOM(Random), COMBINE(Combine), _pro(pro),
        _couplings(couplings), _seed(seed), _ctx(ctx),
        registerLivelihood(registerLivelihood) {

    ga_stats::ChromosomesSet processed_chromosomes;
    for (uint i = 0; i < popSize; i++) {
      EnergyChromosome c = registerLivelihood->getNewRandomChromosome(_seed);
      _individuals.push_back(c);
    }
    int n = 3;
  }

  int size() { return _popSize; }
  void init() {
    setOwnsIndividuals(false);
    evaluate();
  }

  EnergyChromosome getIndividual(size_t index);
  EnergyChromosome &getIndividualRef(size_t index);
  /** \brief Set an individual at the given position in the population. */
  void setIndividual(EnergyChromosome chrm, size_t index);

  //  void randomizeIndividual(uint index);

  /**
   * \brief Evaluate all individuals with the given getFitness function.
   *
   */
  void evaluate();

  void evaluateNonCopy();

  /** \brief Sort individuals in population by getFitness.
   *
   * The fittest individual will be at individual(0) afterwards.
   */
  void sort();

  //  Chromosome *combine(CombineSelection selectionMethod,
  //                      const Context &ctx);
  /**
   * Select an individual by tournament selection.
   */
  EnergyChromosome &tournamentSelect(int tournamentSize, uint *seed,
                                     EnergyChromosome *parent1 = nullptr);

  void print(bool fullChromosomes = false);

  void setOwnsIndividuals(bool ownIndividuals) {
    ownsIndividuals = ownIndividuals;
  }
  const bool getOwnsIndividuals() const { return ownsIndividuals; }

  void setCopyOrigin(int index) {
    _individuals[index].setOrigin(CHROMOSOME::COPY);
  }

  /*****************************************************
   * Should be removed as soon as possible, because they point to wrong pointer
   * usage!
   *
   */
  void removeRegMapping(ga_stats::ChromosomesSet &processed_chromosomes);

  /*****************************************************
   * Should be removed as soon as possible, because they point to wrong pointer
   * usage!
   *
   */
  void deleteRegisterMappings();

  /*****************************************************
   * Should be removed as soon as possible, because they point to wrong pointer
   * usage!
   *
   */
  void deleteNonCopyIndividuals();

  void printGeneSimilarity(EnergyChromosome &bestIndividual);

  EnergyChromosome combine(const Context &ctx);

  const VirtualRegisterMap *getVirtualRegisterMap() const;

  bool isDoublicate(EnergyChromosome &child) {
    std::pair<bool, ga_stats::ChromosomeFitness> r =
        ga_stats::already_processed(
            child.getNumberSequence(),
            // ga_stats::NumberSequence(c->getGenes(), c->length()),
            _processed_chromosomes);
    return r.first;
  }

  int getIndex(const EnergyChromosome &chromosome) const {
    return std::distance(
        _individuals.begin(),
        std::find(_individuals.begin(), _individuals.end(), chromosome));
  }

  void write2File(int generation) {
    //    cout << "Generation: " << generation << endl;
    for (int i = 0; i < min(10, (int)_individuals.size()); i++) {
      if (_individuals[i].validRegisterAllocation()) {
        //      cout << "individual: " << i << " " <<
        //      _individuals[i].checkConflict() << endl;
        auto registerMap = _individuals[i].getVirtualRegisterMap();
        stringstream ss;
        ss << params.raRegPopulationVerificationDir + "/generation_";
        ss << generation;
        ss << "_individual_";
        ss << i;
        std::string origin = toStr(_individuals[i].getOrigin());
        std::replace(origin.begin(), origin.end(), ' ', '_');
        ss << origin;
        ss << ".asm";
        std::ofstream s;
        s.open(ss.str()); // + "_individual" + string(i));
        s << "----- [ SLM 1] ---------------------------------------- " << endl;
        _ins->writeOutInstructions(s, registerMap);
        s.close();

        releaseVirtualMap(&registerMap);
      }
    }
  }
};

} // namespace portOptReg

#endif // SCHEDULER_POPULATIONENERGYALLOCATION_H
