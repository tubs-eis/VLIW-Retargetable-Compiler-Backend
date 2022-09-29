// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#if defined(_OPENMP)
#include <omp.h>
#endif

#include "CompilerContext.h"
#include "SLM.h"
#include "assembler.h"
#include "ga_stats.h"
#include "gen_merging.h"
#include "processor.h"
#include <algorithm>
using namespace gen_X2;

int X2_chromosome::X2_CHROMOSOME_ID;

const int USE_HEURISTIC = -2;

#define NO_X2_FOUND 100000

std::string MergeGeneOrigin::toString(Origin origin) {
  switch (origin) {
  case RANDOM:
    return "randomPopulation";
  case CROSSOVER:
    return "crossover";
  case MUTATE:
    return "mutation";
  case COPY:
    return "copy";
  case COPY2:
    return "copy2";
  case KEEP:
    return "keep";
  default:
    return "<invalid>";
  }
}

X2_chromosome::X2_chromosome(int size, bool keep) {
  _ID = ++X2_CHROMOSOME_ID;
  _size = size;
  _genes = new MergeCandidates::CandidateList_t[size];
  _fitness = 0;
  _keep_it = keep;
  _origin = MergeGeneOrigin::RANDOM;
  slm_mi = 0;
  noMerged = 0;
  _minsize = 0;
  _rank = 0;
  _ranked = false;
  _p1_fit = _p2_fit = 0;
  _p1_ID = _p2_ID = 0;
  _mergedSLM = 0;
  _duplicateChrm = false;
  _duplicateMerge = false;
  _raWorked = 0;
  _registerHeuristicFit = -1;
  _registerGeneticFit = -1;
  _critical = 0;
  _schedSkipped = false;
  _mutation_rate = -1;
  _mutation_count = 0;
}

X2_chromosome::X2_chromosome(const X2_chromosome &gene) {
  _ID = ++X2_CHROMOSOME_ID;
  _size = gene._size;
  _genes = new MergeCandidates::CandidateList_t[_size];
  for (int i = 0; i < _size; ++i)
    _genes[i] = gene._genes[i];
  _fitness = gene._fitness;
  _keep_it = gene._keep_it;
  _origin = gene._origin;
  slm_mi = gene.slm_mi;
  noMerged = gene.noMerged;
  _minsize = 0;
  _rank = 0;
  _ranked = false;
  _p1_fit = _p2_fit = 0;
  _p1_ID = _p2_ID = 0;
  _mergedSLM = gene._mergedSLM;
  _duplicateChrm = gene.isDuplicateChrm();
  _duplicateMerge = gene.isDuplicateMerge();
  _raWorked = 0;
  _registerHeuristicFit = gene._registerHeuristicFit;
  _registerGeneticFit = gene._registerGeneticFit;
  _critical = 0;
  _schedSkipped = gene._schedSkipped;
  _mutation_rate = gene._mutation_rate;
  _mutation_count = gene._mutation_count;
}

X2_chromosome::X2_chromosome(MergeCandidates &mergeCandidates, uint *seed,
                             float noMergeProb, bool shuffle, bool noX) {
  _ID = ++X2_CHROMOSOME_ID;
  _size = mergeCandidates.size();
  _genes = 0;
  _origin = MergeGeneOrigin::RANDOM;
  slm_mi = 0;
  _fitness = 0;
  _keep_it = false;
  noMerged = 0;
  if (_size > 0) {
    _genes = new MergeCandidates::CandidateList_t[_size];
    for (int i = 0; i < _size; ++i) {
      _genes[i] = mergeCandidates.getCandidates(i);
      if (shuffle) {
        if (noMergeProb == -1 || util::uniRandom(seed) > noMergeProb) {
          if (noX) {
            util::FYshuffle(_genes[i], seed);
            _genes[i].push_back(-1);
          } else {
            _genes[i].push_back(-1);
            util::FYshuffle(_genes[i], seed);
          }
        } else {
          if (_genes[i].size() > 0)
            util::FYshuffle(_genes[i], seed);
          _genes[i].insert(_genes[i].cbegin(), -1);
        }
      } else {
        if (noX)
          _genes[i].push_back(-1);
        else
          _genes[i].insert(_genes[i].cbegin(), -1);
      }
    }
  }
  _minsize = 0;
  _rank = 0;
  _ranked = false;
  _p1_fit = _p2_fit = 0;
  _p1_ID = _p2_ID = 0;
  _mergedSLM = 0;
  _duplicateChrm = false;
  _duplicateMerge = false;
  _raWorked = 0;
  _registerHeuristicFit = -1;
  _registerGeneticFit = -1;
  _critical = 0;
  _schedSkipped = false;
  _mutation_rate = -1;
  _mutation_count = 0;
}

X2_chromosome &X2_chromosome::operator=(const X2_chromosome &g) {
  if (_size != g._size) {
    delete[] _genes;
    _size = g._size;
    _genes = new MergeCandidates::CandidateList_t[_size];
  }
  for (int i = 0; i < _size; ++i)
    _genes[i] = g._genes[i];
  _fitness = g._fitness;
  _keep_it = g._keep_it;
  _origin = g._origin;
  slm_mi = g.slm_mi;
  noMerged = g.noMerged;
  _minsize = g._minsize;
  _rank = g._rank;
  _ranked = g._ranked;
  _duplicateChrm = g._duplicateChrm;
  _duplicateMerge = g._duplicateMerge;
  _mergedSLM = g._mergedSLM;
  return *this;
}

X2_chromosome *X2_chromosome::random(MergeCandidates &mergeCandidates,
                                     uint *seed, float noMergeProb) {
  return new X2_chromosome(mergeCandidates, seed, noMergeProb, true);
}

X2_chromosome *X2_chromosome::no_merge(MergeCandidates &mergeCandidates,
                                       uint *seed) {
  return new X2_chromosome(mergeCandidates, seed, -1, false, false);
}

X2_chromosome *X2_chromosome::no_X(MergeCandidates &mergeCandidates,
                                   uint *seed) {
  return new X2_chromosome(mergeCandidates, seed, -1, true, true);
}

X2_chromosome *X2_chromosome::heuristic(MergeCandidates &mergeCandidates,
                                        uint *seed) {
  return new X2_chromosome(mergeCandidates, seed, -1, false, true);
}

void X2_chromosome::copySLM(SLM *slm, Processor *pro) {
  _mergedSLM = new SLM();
  _mergedSLM->setOriginalID(slm->getOriginalID());
  auto orig_ops = slm->getOperations();
  for (std::vector<MO *>::iterator it = orig_ops->begin();
       it != orig_ops->end(); ++it)
    _mergedSLM->addMO((*it)->copy());
  if (slm->getBranchOperation())
    _mergedSLM->addMO(slm->getBranchOperation()->copy());
  _mergedSLM->calculateGraph(pro, false);
  _minsize = _mergedSLM->getCriticalPath();
  _mergedSLM->calculateGraph(pro, true);
  _mergedSLM->setPartProb(slm->getPartProb());
  _mergedSLM->setAloneProb(slm->getAloneProb());
  _mergedSLM->setRAPruneCount(slm->getRAPruneCount());
}

X2_chromosome::~X2_chromosome() {
  if (_genes) {
    delete[] _genes;
    _genes = 0;
  }
  if (_mergedSLM) {
    delete _mergedSLM; // we can delete all included MOs, as they are individual
                       // copies for each SLM.
  }
}

void X2_chromosome::releaseAll() {
  delete[] _genes;
  _genes = 0;
}

void X2_chromosome::print(const X2_chromosome *chrm, std::vector<MO *> *ops) {
  for (int i = 0; i < chrm->size(); ++i) {
    LOG_OUTPUT(LOG_M_ALWAYS, "%3d -> ", getMONum(ops, i));
    const MergeCandidates::CandidateList_t &list = chrm->gene(i);
    printMergeCandidates(LOG_M_ALWAYS, list, ops);
    LOG_OUTPUT(LOG_M_ALWAYS, "\n");
  }
}

X2Population::X2Population(int size) {
  _size = size;
  _individuals = new X2_chromosome *[_size]();
  memset(_individuals, 0, _size * sizeof(X2_chromosome *));
}

X2Population::X2Population(const X2Population &pop) {
  _size = pop._size;
  _individuals = new X2_chromosome *[_size];
  memcpy(_individuals, pop._individuals, _size * sizeof(X2_chromosome *));
}

X2Population &X2Population::operator=(const X2Population &pop) {
  if (_size != pop._size) {
    delete _individuals;
    _size = pop._size;
    _individuals = new X2_chromosome *[_size];
  }
  memcpy(_individuals, pop._individuals, _size * sizeof(X2_chromosome *));
  return *this;
}

X2Population::~X2Population() { delete[] _individuals; }

void X2Population::releaseAll() {
  for (int i = 0; i < _size; ++i) {
    _individuals[i]->releaseAll();
    delete _individuals[i];
    _individuals[i] = 0;
  }
  delete[] _individuals;
  _individuals = 0;
}

void X2Population::releaseNonNeeded() {
  for (int i = 0; i < _size; i++) {
    X2_chromosome *g = _individuals[i];
    if (g && !g->hasKeep() && g->origin() != MergeGeneOrigin::COPY &&
        g->origin() != MergeGeneOrigin::COPY2) {
      g->releaseAll();
      _individuals[i] = 0;
      delete g;
    }
  }
}

bool gen_X2::getMatchingX2Reg(char32_t fixReg, REG_PAIR_POS fixPos,
                              bool concurrent, int &regNo, int &regFile) {
  uint fixNo = registers::getRegNumber(fixReg);
  if (fixNo < 0)
    return false;
  int fixFile = registers::getRegFile(fixReg);
  switch (fixPos) {
  case REG_PAIR_POS::FIRST:
    if ((concurrent &&
         (fixNo % 2 == 1 || fixNo >= registers::getNumRegister1RF())) ||
        (!concurrent && (fixFile != 0)))
      return false;
    if (concurrent) {
      regNo = fixNo + 1;
      regFile = fixFile;
    } else {
      regNo = fixNo;
      regFile = fixFile + 1;
    }
    return true;
  case REG_PAIR_POS::SECOND:
    if ((concurrent && (fixNo % 2 == 0 || fixNo == 0)) ||
        (!concurrent && (fixFile != 1)))
      return false;
    if (concurrent) {
      regNo = fixNo - 1;
      regFile = fixFile;
    } else {
      regNo = fixNo;
      regFile = fixFile - 1;
    }
    return true;
  }
  return false;
}

#define NO_VALID_SCHED_PENALTY 10000

float gen_X2::fitness(SLM *slm, Processor *pro, X2_chromosome *gene,
                      int bestSize, const Context &ctx) {
  SLM *better = gene->getMergedSLM();
  if (!better)
    return NO_X2_FOUND;
  LOG_OUTPUT(LOG_M_MERGE_DETAIL,
             "%s Minimal size of SLM %d after merging: %d, critical path: %d\n",
             ctx.asString().c_str(), slm->getOriginalID(),
             better->getMinimalSize(), better->getCriticalPath());
  if (params.enableSchedSkip && bestSize != -1)
    if (better->getMinimalSize() > bestSize + params.schedSkipOffset) {
      LOG_OUTPUT(LOG_M_SCHED_SKIP,
                 "%s Skipping scheduling for merged SLM %d because of minimal "
                 "size %d > %d + %d\n",
                 ctx.asString().c_str(), slm->getOriginalID(),
                 better->getMinimalSize(), bestSize, params.schedSkipOffset);
      gene->setMICount(better->getOperations()->size());
      size_t s = better->getOperations()->size() + NO_VALID_SCHED_PENALTY;
      delete better;
      gene->setMergedSLM(0);
      gene->schedSkipped() = true;
      return s;
    }
  //    int portReg = params.portReg;
  //    LOG_OUTPUT(LOG_M_ALWAYS, "RECOMPILE WITH GENETIC RA\n");
  int s = simpleCompile(better, pro, Context::nextStage(ctx, "sched"));
  //    params.portReg = 0;
  //    LOG_OUTPUT(LOG_M_ALWAYS, "RECOMPILE WITHOUT GENETIC RA\n");
  //    s = simpleCompile(better, pro, Context::nextStage(ctx, "sched"));
  //    params.portReg = portReg;

  Program *ins = better->getShortestInstruction();
  if (ins != NULL)
    s = ins->size();

  gene->registerHeuristicFit() = better->getHeuristicRegisterFitness();
  gene->registerGeneticFit() = better->getGeneticRegisterFitness();
  gene->raWorked() = (ins != NULL);
  slm->setAlternative(gene->getMergedSLM_ptr());
  gene->setMICount(s);
  if (!gene->raWorked())
    s += NO_VALID_SCHED_PENALTY;
  if (gene->registerGeneticFit() == -2) // impossible register allocation
    s += NO_VALID_SCHED_PENALTY;
  LOG_OUTPUT(LOG_M_MERGE_DETAIL, "%s Scheduled size for merged SLM %d: %d\n",
             ctx.asString().c_str(), slm->getOriginalID(), s);
  return s;
}

#define CROSSOVER_RATE 0.5

void gen_X2::combine(X2_chromosome *parent1, X2_chromosome *parent2,
                     X2_chromosome *child1, X2_chromosome *child2, uint *seed) {
  int size = parent1->size();
  for (int i = 0; i < size; ++i) {
    if (util::uniRandom(seed) < CROSSOVER_RATE) {
      child1->gene(i) = parent2->gene(i);
      child2->gene(i) = parent1->gene(i);
    } else {
      child1->gene(i) = parent1->gene(i);
      child2->gene(i) = parent2->gene(i);
    }
  }
  child1->p1ID() = parent1->ID();
  child1->p1Fit() = parent1->fitness();
  child1->p2ID() = parent2->ID();
  child1->p2Fit() = parent2->fitness();
  child2->p1ID() = parent1->ID();
  child2->p1Fit() = parent1->fitness();
  child2->p2ID() = parent2->ID();
  child2->p2Fit() = parent2->fitness();
}

void gen_X2::combine_default(X2_chromosome *parent1, X2_chromosome *parent2,
                             X2_chromosome *child1, X2_chromosome *child2,
                             uint *seed) {
  int size = parent1->size();
  int cut = rand_r(seed) % size;
  for (int i = 0; i < size; ++i) {
    if (i < cut) {
      child1->gene(i) = parent2->gene(i);
      child2->gene(i) = parent1->gene(i);
    } else {
      child1->gene(i) = parent1->gene(i);
      child2->gene(i) = parent2->gene(i);
    }
  }
  child1->p1ID() = parent1->ID();
  child1->p1Fit() = parent1->fitness();
  child1->p2ID() = parent2->ID();
  child1->p2Fit() = parent2->fitness();
  child2->p1ID() = parent1->ID();
  child2->p1Fit() = parent1->fitness();
  child2->p2ID() = parent2->ID();
  child2->p2Fit() = parent2->fitness();
}

void gen_X2::mutate(X2_chromosome *g, float chrm_fitness, float best_fitness,
                    int regGen, float &noMergeProb, uint *seed) {
  float mutation_rate = params.mergeMutationRate;
  if (params.mergeMutateAdaptively) {
    if (regGen == -2)
      mutation_rate =
          min(1.0f, (chrm_fitness - best_fitness) * params.mergeMutateSlope +
                        mutation_rate);
    else
      mutation_rate *=
          min(chrm_fitness - best_fitness, 10.0f) * params.mergeMutateSlope + 1;
  }
  g->mutationRate() = mutation_rate;
  int size = g->size();
  for (int i = 0; i < size; i++) {
    if (util::uniRandom(seed) < mutation_rate) {
      if (regGen == -2) {
        util::FYshuffle(g->gene(i), seed);
        if (util::uniRandom(seed) <
            noMergeProb) { // noMergeProb == -1 -> feature off; is included in
                           // this test!
          auto Xpos = MergeCandidates::Xpos(g->gene(i));
          g->gene(i).erase(g->gene(i).begin() + Xpos);
          g->gene(i).insert(g->gene(i).begin(), -1);
        }
      } else {
        auto Xpos_before = MergeCandidates::Xpos(g->gene(i));
        util::FYshuffle(g->gene(i), seed);
        if (util::uniRandom(seed) >= mutation_rate) {
          auto Xpos_after = MergeCandidates::Xpos(g->gene(i));
          if (Xpos_after < Xpos_before)
            util::swap(g->gene(i), Xpos_after, Xpos_before);
        }
      }
      g->origin() = MergeGeneOrigin::MUTATE;
      g->mutationCount()++;
    }
  }
}

X2_chromosome *gen_X2::tournamentSelect(X2Population &population,
                                        int tournamentSize, uint *seed) {
  int bestIndex = -1;
  int bestFitness = INT32_MAX;

  int popsize = population.size();
  for (int i = 0; i < tournamentSize; ++i) {
    int index = util::uniRandom(0, popsize - 1, seed);
    if (population[index]->fitness() < bestFitness ||
        (population[index]->fitness() == bestFitness && bestIndex != -1 &&
         population[index]->minsize() < population[bestIndex]->minsize())) {
      bestIndex = index;
      bestFitness = population[index]->fitness();
    }
  }

  return population[bestIndex];
}

int mergePopSize(const MergeCandidates &mergeCandidates) {
  int size = 0;
  for (int index = 0; index < mergeCandidates.size(); ++index)
    if (mergeCandidates.getCandidates(index).size() > 0)
      ++size;
  return size;
}

int getPopulationSize(SLM *slm, MergeCandidates &mergeCandidates) {
  int popsize;
  if (slm->getMergeLevel() != -1 || params.mergePopulation == -1) {
    popsize = mergePopSize(mergeCandidates);
    int mergeLvl = slm->getMergeLevel();
    mergeLvl = (mergeLvl == -1) ? params.mergeLevel : mergeLvl;
    if (popsize > 0 && mergeLvl == 1)
      return USE_HEURISTIC;
    else {
      for (int o = 2; o < mergeLvl; ++o)
        popsize *= 2;
    }
  } else {
    return params.mergePopulation;
  }
  return popsize > 0 ? max(3, popsize / params.mergePopsizeDiv) : 0;
}

void performAutomerge(X2_chromosome *chrm, SLM *slm, Processor *pro,
                      const Context &ctx) {
  int noMerged = 0;
  SLM *better = X2Automerge(slm, pro, chrm, noMerged, ctx);
  chrm->setMergedNum(noMerged);
  chrm->setMergedSLM(better);
  if (better == NULL)
    return;
  chrm->critical() = better->getCriticalPath();
  chrm->minsize() = better->getMinimalSize();
  better->calculateGraph(pro, true);
}

void initializePopulation(X2Population &population,
                          MergeCandidates &mergeCandidates, Processor *pro,
                          SLM *slm, int size, uint *seed, const Context &ctx) {
  int popsize = population.size();
#pragma omp parallel for
  for (int i = 0; i < popsize; i++) {
    if (killed)
      continue;
    X2_chromosome *g;
    if (i == 0)
      g = X2_chromosome::no_merge(mergeCandidates, seed);
    else if (i == 1)
      g = X2_chromosome::heuristic(mergeCandidates, seed);
    else
      g = X2_chromosome::random(mergeCandidates, seed,
                                params.noMergeProbability);

    g->origin() = MergeGeneOrigin::RANDOM;
    if (i > 0)
      performAutomerge(g, slm, pro, ctx);
    else
      g->copySLM(slm, pro);
    population.set(g, i);
  }
  for (int i = 0; i < popsize; ++i) {
    X2_chromosome *g = population.getIndividual(i);
    g->fitness() = gen_X2::fitness(slm, pro, g, -1, Context::Merge(ctx, 0, i));

    //        if (i == 0 && !g->raWorked()) {
    //            LOG_OUTPUT(LOG_M_ALWAYS, "NO SOLUTION IN FIRST
    //            INDIVIDUAL!\n"); exit(-1);
    //        }
  }
}

void printPopDuplicates(X2Population &population) {
  int count = population.size();
  for (int i = 0; i < count; ++i) {
    for (int j = i + 1; j < count; ++j) {
      if (population[i]->sameGenes(*population[j]))
        LOG_OUTPUT(LOG_M_MERGE, "(%d, %d)", i, j);
    }
  }
}

void printPopulation(X2Population &population, int round, int slmID,
                     std::vector<MO *> *ops, float noMergeProb,
                     const Context &ctx) {
  LOG_OUTPUT(LOG_M_ALWAYS,
             "%s Merge population generation %d population size %d. No merge "
             "prob = %5.4f\n",
             ctx.asString().c_str(), round, population.size(), noMergeProb);
  for (int n = 0; n < population.size(); ++n) {
    LOG_OUTPUT(LOG_M_ALWAYS, "  Indiv %3d: ", n);
    X2_chromosome *chrm = population[n];
    LOG_OUTPUT(LOG_M_ALWAYS, " size: %4d ", chrm->getMICount());
    LOG_OUTPUT(LOG_M_ALWAYS, " fit: %6.0f ", chrm->fitness());
    LOG_OUTPUT(LOG_M_ALWAYS, " (H: %3d G: %3d) | ",
               chrm->registerHeuristicFit(), chrm->registerGeneticFit());
    LOG_OUTPUT(LOG_M_ALWAYS, " merges: %3d ", chrm->getMergedNum());
    LOG_OUTPUT(LOG_M_ALWAYS, " minsize: %3d  critical: %3d | ", chrm->minsize(),
               chrm->critical());
    LOG_OUTPUT(LOG_M_ALWAYS, " origin: %9s",
               MergeGeneOrigin::toString(chrm->origin()).c_str());
    if (chrm->origin() == MergeGeneOrigin::CROSSOVER ||
        chrm->origin() == MergeGeneOrigin::MUTATE)
      LOG_OUTPUT(LOG_M_ALWAYS, " (%5d + %5d) | ", chrm->p1Fit(), chrm->p2Fit());
    else
      LOG_OUTPUT(LOG_M_ALWAYS, "                 | ");
    LOG_OUTPUT(LOG_M_ALWAYS, "MR: %7.4f  MC: %3d", chrm->mutationRate(),
               chrm->mutationCount());
    if (chrm->schedSkipped())
      LOG_OUTPUT(LOG_M_ALWAYS, "  sched skipped");
    LOG_OUTPUT(LOG_M_ALWAYS, "\n");

    //        X2_chromosome::print(chrm, ops);
  }
}

void evaluate_chromosome(X2_chromosome *chrm, unsigned int &dupChrmSkipped,
                         unsigned int &dupMergesSkipped, int bestSize,
                         const Context &ctx, int generation, int chrmIdx,
                         SLM *slm, Processor *pro,
                         ga_stats::ChromosomesSet &processed_chromosomes,
                         ga_stats::ChromosomesSet &processed_merges) {
  if (chrm->isDuplicateChrm()) {
    dupChrmSkipped++;
  } else {
    if (chrm->isDuplicateMerge()) {
      dupMergesSkipped++;
    } else {
      chrm->fitness() = fitness(slm, pro, chrm, bestSize,
                                Context::Merge(ctx, generation, chrmIdx));
#if MERGE_CHRM_DUPLICATE_CHECK
      ga_stats::add_processed(ga_stats::NumberSequence(chrm), chrm->fitness(),
                              processed_chromosomes);
#endif
#if MERGE_DUPLICATE_CHECK
      SLM *s = chrm->getMergedSLM();
      if (s) {
        ga_stats::NumberSequence seq(s->getOperations());
        ga_stats::add_processed(seq, chrm->fitness(), processed_merges);
      }
#endif
    }
  }
}

float initialNoMergeProb(const X2Population *pop) {
  int invalidRegGACount = pop->countInvalidRegGA();
  return static_cast<float>(invalidRegGACount) /
         static_cast<float>(pop->size());
}

#define INVALID_REG_THRESH .5

void adaptNoMergeProb(const X2Population *pop, float &noMergeProb) {
  if (pop->size() == 0)
    return;

  int invalidRegGACount = pop->countInvalidRegGA();
  float invalidRegRatio =
      static_cast<float>(invalidRegGACount) / static_cast<float>(pop->size());
  if (invalidRegRatio > INVALID_REG_THRESH) {
    noMergeProb += (1 - noMergeProb) * (invalidRegRatio - INVALID_REG_THRESH);
  } else if (invalidRegRatio < INVALID_REG_THRESH) {
    noMergeProb -= noMergeProb * (INVALID_REG_THRESH - invalidRegRatio);
  }
}

void gen_X2::genetic(SLM *slm, Processor *pro, const Context &ctx) {
  int size = slm->getOperations()->size();
  LOG_OUTPUT(LOG_M_MERGE, "%s Starting X2 merging for SLM %d (op count: %d)\n",
             ctx.asString().c_str(), slm->getOriginalID(), size);
  if (isLog(LOG_M_MERGE_DEBUG)) {
    LOG_OUTPUT(LOG_M_MERGE_DEBUG, "%s SLM %d blocked registers:\n",
               ctx.asString().c_str(), slm->getOriginalID());
    printBlockedRegisters(blockedRegsInSLM[slm->getOriginalID()]);
  }
  MergeCandidates mergeCandidates(slm, pro);
  if (isLog(LOG_M_MERGE_DEBUG)) {
    LOG_OUTPUT(LOG_M_MERGE_DEBUG, "Merge candidates for SLM %d:\n",
               slm->getOriginalID());
    MergeCandidates::print(LOG_M_MERGE_DEBUG, mergeCandidates,
                           slm->getOperations());
  }

  int mergeCombinationRatio = params.mergeCmbRatio;
  unsigned int dupChrmSkipped = 0;
  unsigned int dupMergesSkipped = 0;
  ga_stats::ChromosomesSet processed_merges;
  ga_stats::ChromosomesSet processed_chromosomes;

  uint seed = 500; //(unsigned int)time(0);
  int popsize = getPopulationSize(slm, mergeCandidates);
  if (popsize == USE_HEURISTIC) {
    LOG_OUTPUT(LOG_M_MERGE, "%s Merging using heuristic in SLM %d\n",
               ctx.asString().c_str(), slm->getOriginalID());
    auto g = X2_chromosome::heuristic(mergeCandidates, &seed);
    performAutomerge(g, slm, pro, ctx);
    gen_X2::fitness(slm, pro, g, -1, Context::Merge(ctx, 0, 0));
    delete g;
    return;
  }
  if (popsize == 0) {
    LOG_OUTPUT(LOG_M_MERGE, "%s No merging possible in SLM %d\n",
               ctx.asString().c_str(), slm->getOriginalID());
    auto g = X2_chromosome::no_merge(mergeCandidates, &seed);
    g->copySLM(slm, pro);
    gen_X2::fitness(slm, pro, g, -1, Context::Merge(ctx, 0, 0));
    delete g;
    return;
  }

  int bestMinimalSize = -1;

  /* Generate initial population */
  X2Population *population = new X2Population(popsize);
  int num_copy = popsize > 3 ? 3 : 0; // Elitism
  int num_combine = mergeCombinationRatio * (popsize - num_copy) / 100 /
                    2; // crossover (generates pairs, therefore / 2)
  int num_create;      // Random individuals

  if (num_copy < 0 || num_combine < 0)
    LOG_OUTPUT(
        LOG_M_ALWAYS,
        "Fatal error in genetic merging: num_copy = %d, num_combine = %d\n",
        num_copy, num_combine);

  LOG_OUTPUT(LOG_M_MERGE, "%s Merging SLM %d with population size %d\n",
             ctx.asString().c_str(), slm->getOriginalID(), popsize);

  initializePopulation(*population, mergeCandidates, pro, slm, size, &seed,
                       ctx);

#if MERGE_CHRM_DUPLICATE_CHECK
  for (int i = 0; i < popsize; ++i)
    add_processed(ga_stats::NumberSequence(population->getIndividual(i)),
                  population->getIndividual(i)->fitness(),
                  processed_chromosomes);
#endif

  X2_chromosome *fittest = population->getFittest();
  float best = fittest->fitness();
  int bestSize = (fittest->registerHeuristicFit() == 0 ||
                  fittest->registerGeneticFit() == 0)
                     ? fittest->getMICount()
                     : -1;
  LOG_OUTPUT(
      LOG_M_MERGE_DETAIL,
      "%s Random initialization of SLM %d. MI count: %lu, getFitness: %7.2f\n",
      ctx.asString().c_str(), slm->getOriginalID(), slm->getShortestSize(),
      best);

  int generation = 0;
  bool solutionFound = best != NO_X2_FOUND;
  int maxRounds =
      solutionFound ? abs(params.mergeRounds) : params.mergeTrialRounds;
  population->sort();

  if (params.printMergePopulation)
    printPopulation(*population, 0, slm->getOriginalID(), slm->getOperations(),
                    params.noMergeProbability, ctx);

  float noMergeProb = initialNoMergeProb(population);

  /***********************************
   * Start evolution
   */
  int dumb_rounds = 0; // number of rounds without any improvement
  while (!killed && dumb_rounds++ < maxRounds) {
    ++generation;
    LOG_OUTPUT(
        LOG_M_MERGE_DETAIL,
        "%s Starting merge generation %d for SLM %d (best minimal size: %d)\n",
        ctx.asString().c_str(), generation, slm->getOriginalID(),
        bestMinimalSize);
    X2Population *copy = population;
    for (int x = 0; x < copy->size();
         ++x) { // delete COPY origin, so that these individuals can be freed
                // later
      X2_chromosome *g = copy->getIndividual(x);
      g->origin() =
          g->hasKeep() ? MergeGeneOrigin::KEEP : MergeGeneOrigin::RANDOM;
    }
    population = new X2Population(popsize);

    /*****************************
     * Elitism
     */
    int copiedIndividuals = 0;
    for (int x = 0; x < num_copy; x++) {
      X2_chromosome *g = copy->getIndividual(x);
      if (!population->contains(g)) {
        population->set(g, copiedIndividuals);
        ++copiedIndividuals;
      }
      g->origin() =
          g->hasKeep() ? MergeGeneOrigin::KEEP : MergeGeneOrigin::COPY;
    }

    // Number of randomly generated individuals is computed to fill the
    // population up to popsize
    num_create = popsize - copiedIndividuals - 2 * num_combine;
    LOG_OUTPUT(LOG_M_MERGE_DETAIL,
               "%s Merging population: COPY = %d, COMBINE = %d, CREATE = %d\n",
               ctx.asString().c_str(), num_copy, num_combine, num_create);
    if (num_create < 0)
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Fatal error in genetic merging: num_create = %d\n",
                 num_create);

#pragma omp parallel
    {
      /***********************
       * Crossover & mutation
       */
#pragma omp for nowait
      for (int x = 0; x < num_combine; ++x) {
        X2_chromosome *p1 =
            tournamentSelect(*copy, params.mergeTournamentSize, &seed);
        X2_chromosome *p2 =
            tournamentSelect(*copy, params.mergeTournamentSize, &seed);
        int searchCount = 0;
        while (searchCount < 20 && p1 == p2) {
          p2 = tournamentSelect(*copy, params.mergeTournamentSize, &seed);
          searchCount++;
        }
        LOG_OUTPUT(searchCount == 10,
                   "%s Did not find different parents in operation merging\n",
                   ctx.asString().c_str());
        X2_chromosome *next1 = new X2_chromosome(p1->size());
        X2_chromosome *next2 = new X2_chromosome(p1->size());
        next1->origin() = MergeGeneOrigin::CROSSOVER;
        next2->origin() = MergeGeneOrigin::CROSSOVER;
        combine(p1, p2, next1, next2, &seed);
        float chrm_fitness = (p1->fitness() + p2->fitness()) / 2;
        int regG =
            (p1->registerGeneticFit() == -2 || p2->registerGeneticFit() == -2)
                ? -2
                : p1->registerGeneticFit();
        mutate(next1, chrm_fitness, best, regG, noMergeProb, &seed);
        mutate(next2, chrm_fitness, best, regG, noMergeProb, &seed);
        population->set(next1, copiedIndividuals + 2 * x);
        population->set(next2, copiedIndividuals + 2 * x + 1);
      }

      /**********************
       * Create randomPopulation individuals
       */
#pragma omp for
      for (int x = 0; x < num_create; ++x) {
        X2_chromosome *g =
            X2_chromosome::random(mergeCandidates, &seed, noMergeProb);
        g->origin() = MergeGeneOrigin::RANDOM;
        population->set(g, copiedIndividuals + 2 * num_combine + x);
      }

#if MERGE_CHRM_DUPLICATE_CHECK
#pragma omp for
      for (int x = 0; x < 2 * num_combine + num_create; ++x) {
        X2_chromosome *c = population->getIndividual(copiedIndividuals + x);
        std::pair<bool, ga_stats::ChromosomeFitness> found = already_processed(
            ga_stats::NumberSequence(c), processed_chromosomes);
        if (found.first) {
          c->setDuplicateChrm();
          c->fitness() = found.second.getFitness();
        } else
          performAutomerge(c, slm, pro, ctx);
      }
#else
#pragma omp for
      for (int x = 0; x < 2 * num_combine + num_create; ++x)
        performAutomerge(population->getIndividual(copiedIndividuals + x), slm,
                         pro, ctx);
#endif

#if MERGE_DUPLICATE_CHECK
#pragma omp for
      for (int x = 0; x < 2 * num_combine + num_create; ++x) {
        X2_chromosome *c = population->getIndividual(copiedIndividuals + x);
        if (c->isDuplicate())
          continue;
        SLM *s = c->getMergedSLM();
        if (s) {
          ga_stats::NumberSequence seq(s->getOperations());
          std::pair<bool, ga_stats::ChromosomeFitness> found =
              already_processed(seq, processed_merges);
          if (found.first) {
            c->setDuplicateMerge();
            c->fitness() = found.second.getFitness();
          }
        }
      }
#endif

    } // omp parallel
    /***********************
     * Evaluate Crossover & mutation
     */
    for (int x = 0; x < num_combine; ++x) {
      if (killed)
        break;
      X2_chromosome *next1 =
          population->getIndividual(copiedIndividuals + 2 * x);
      evaluate_chromosome(next1, dupChrmSkipped, dupMergesSkipped, bestSize,
                          ctx, generation, copiedIndividuals + 2 * x, slm, pro,
                          processed_chromosomes, processed_merges);
      X2_chromosome *next2 =
          population->getIndividual(copiedIndividuals + 2 * x + 1);
      evaluate_chromosome(next2, dupChrmSkipped, dupMergesSkipped, bestSize,
                          ctx, generation, copiedIndividuals + 2 * x + 1, slm,
                          pro, processed_chromosomes, processed_merges);
    }

    /**********************
     * Evaluate randomPopulation individuals
     */
    for (int x = 0; x < num_create; ++x) {
      if (killed)
        break;
      X2_chromosome *g =
          population->getIndividual(copiedIndividuals + 2 * num_combine + x);
      evaluate_chromosome(g, dupChrmSkipped, dupMergesSkipped, bestSize, ctx,
                          generation, copiedIndividuals + 2 * num_combine + x,
                          slm, pro, processed_chromosomes, processed_merges);
    }

    if (slm->getMinimalSize() < bestMinimalSize)
      bestMinimalSize = slm->getMinimalSize();
    population->sort();
    if (bestSize > population->getFittest()->getMICount())
      bestSize = population->getFittest()->getMICount();
    X2_chromosome *fittest = population->getFittest();
    float popFitness = fittest->fitness();
    if (!solutionFound && popFitness != NO_X2_FOUND) {
      solutionFound = true;
      maxRounds = abs(params.mergeRounds);
      generation = 0;
      dumb_rounds = 0;
    }
    if (best > popFitness) {
      best = popFitness;
      if (params.mergeRounds > 0) {
        LOG_OUTPUT(LOG_M_MERGE_DETAIL, "%s Reset dumb_rounds in round %d\n",
                   ctx.asString().c_str(), generation);
        dumb_rounds = 0;
      }
      uint size = slm->getShortestInstruction()
                      ? slm->getShortestInstruction()->size()
                      : (uint)-1;
      LOG_OUTPUT(LOG_M_MERGE_DETAIL,
                 "%s Improvement in merging in round %i for SLM %d to MI "
                 "count: %lu, getFitness: %7.2f\n",
                 ctx.asString().c_str(), generation, slm->getOriginalID(), size,
                 best);
    }
    uint size = slm->getShortestInstruction()
                    ? slm->getShortestInstruction()->size()
                    : (uint)-1;
    LOG_OUTPUT(LOG_M_MERGE_SIZE_HIST,
               "%s End of merge generation %d for SLM %d. MI count: %lu, "
               "getFitness: %7.2f, best minimal size: %d\n",
               ctx.asString().c_str(), generation, slm->getOriginalID(), size,
               popFitness, bestMinimalSize);

    if (params.printMergePopulation)
      printPopulation(*population, generation, slm->getOriginalID(),
                      slm->getOperations(), noMergeProb, ctx);
    copy->releaseNonNeeded();
    delete copy;

    adaptNoMergeProb(population, noMergeProb);
  } // end generations

#if MERGE_CHRM_DUPLICATE_CHECK
  LOG_OUTPUT(
      LOG_M_MERGE,
      "%s Merging skipped %d duplicate chromosomes (processed %lu) in SLM %d\n",
      ctx.asString().c_str(), dupChrmSkipped, processed_chromosomes.size(),
      slm->getOriginalID());
#endif
#if MERGE_DUPLICATE_CHECK
  LOG_OUTPUT(LOG_M_MERGE,
             "%s Merging skipped %d duplicate automerged (processed %lu) SLMs "
             "in SLM %d\n",
             ctx.asString().c_str(), dupMergesSkipped, processed_merges.size(),
             slm->getOriginalID());
#endif

  /**************
   * Cleanup
   */
  population->releaseAll();
  delete population;

  uint finalsize = slm->getShortestInstruction()
                       ? slm->getShortestInstruction()->size()
                       : (uint)-1;
  LOG_OUTPUT(LOG_M_MERGE, "%s Merging of SLM %d finished with MI count: %lu\n",
             ctx.asString().c_str(), slm->getOriginalID(), finalsize);
  LOG_OUTPUT(params.printMergePopulation,
             "Finished merging of SLM %d after %d generations\n",
             slm->getOriginalID(), generation);
}
