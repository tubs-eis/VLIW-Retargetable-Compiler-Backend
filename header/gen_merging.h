// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef HEADER_GEN_MERGING_H_
#define HEADER_GEN_MERGING_H_

#include "virtual_reg.h"
#include <algorithm>

class Context;

#define MERGE_CHRM_DUPLICATE_CHECK 1
#define MERGE_DUPLICATE_CHECK 1
#define DISPLAY_MERGES 0

namespace gen_X2 {

class MergeGeneOrigin {
public:
  enum Origin { RANDOM, CROSSOVER, MUTATE, COPY, KEEP, COPY2 };
  static std::string toString(Origin origin);
};

/** For each MO in a SLM, list all possible merge candidates. */
class MergeCandidates {
public:
  typedef std::vector<int> CandidateList_t;

private:
  CandidateList_t *_mergeCandidates;
  int _size;

  friend std::ostream &operator<<(std::ostream &os,
                                  const MergeCandidates &candidates);
  void copyFrom(const MergeCandidates &candidates);

public:
  MergeCandidates(int size);
  MergeCandidates(const MergeCandidates &candidates);
  MergeCandidates(SLM *slm, Processor *pro);
  ~MergeCandidates();
  MergeCandidates &operator=(const MergeCandidates &candidates);

  int size() const { return _size; }

  static uint Xpos(const CandidateList_t &candidates);

  int totalCandidateCount() const;

  void addCandidate(int firstMOIndex, int candidateIndex);
  const CandidateList_t &getCandidates(int MOIndex) const;

  static void print(LOG_MASK_T logMask, const MergeCandidates &candidates,
                    const std::vector<MO *> *ops);
};

void printMergeCandidates(LOG_MASK_T logMask,
                          const MergeCandidates::CandidateList_t &candidates,
                          const std::vector<MO *> *ops);

class X2_chromosome {
private:
  MergeCandidates::CandidateList_t *_genes;

  static int X2_CHROMOSOME_ID;

  int _ID;
  int _size;
  float _fitness;
  bool _keep_it;
  MergeGeneOrigin::Origin _origin;

  int slm_mi;
  int noMerged;
  int _minsize;
  int _critical;
  SLM *_mergedSLM;
  int _rank;
  bool _ranked;
  int _p1_fit, _p2_fit;
  int _p1_ID, _p2_ID;

  int _raWorked;

  bool _duplicateMerge;
  bool _duplicateChrm;

  bool _schedSkipped;
  float _mutation_rate;
  int _mutation_count;

  int _registerHeuristicFit, _registerGeneticFit;

#if DISPLAY_MERGES
  std::vector<std::pair<int, int>> _merges;
#endif
public:
  X2_chromosome(int size, bool keep = false);
  X2_chromosome(const X2_chromosome &gene);
  X2_chromosome(MergeCandidates &mergeCandidates, uint *seed,
                float noMergeProb = -1, bool shuffle = true, bool noX = false);
  X2_chromosome &operator=(const X2_chromosome &g);

  static X2_chromosome *random(MergeCandidates &mergeCandidates, uint *seed,
                               float noMergeProb = -1);
  static X2_chromosome *no_merge(MergeCandidates &mergeCandidates, uint *seed);
  static X2_chromosome *no_X(MergeCandidates &mergeCandidates, uint *seed);
  static X2_chromosome *heuristic(MergeCandidates &mergeCandidates, uint *seed);

  void copySLM(SLM *slm, Processor *pro);

  ~X2_chromosome();
  void releaseAll();

  MergeCandidates::CandidateList_t &gene(int index) { return _genes[index]; }

  const MergeCandidates::CandidateList_t &gene(int index) const {
    return _genes[index];
  }

  MergeCandidates::CandidateList_t *genes() { return _genes; }

  int size() const { return _size; }

  float &fitness() { return _fitness; }

  bool &schedSkipped() { return _schedSkipped; }

  float &mutationRate() { return _mutation_rate; }

  int &mutationCount() { return _mutation_count; }

  float fitness() const { return _fitness; }

  bool hasKeep() { return _keep_it; }

  MergeGeneOrigin::Origin &origin() { return _origin; }

  int getMICount() { return slm_mi; }

  void setMICount(int count) { slm_mi = count; }

  int getMergedNum() { return noMerged; }

  void setMergedNum(int num) { noMerged = num; }

  void shuffle(uint *seed) {
    for (int i = 0; i < _size; ++i)
      util::FYshuffle(_genes[i], seed);
  }

  int &rank() { return _rank; }

  int &minsize() { return _minsize; }

  int &critical() { return _critical; }

  bool &ranked() { return _ranked; }

  int &p1Fit() { return _p1_fit; }

  int &p2Fit() { return _p2_fit; }

  int ID() { return _ID; }

  int &p1ID() { return _p1_ID; }

  int &p2ID() { return _p2_ID; }

  int &registerHeuristicFit() { return _registerHeuristicFit; }

  int &registerGeneticFit() { return _registerGeneticFit; }

  int registerGeneticFit() const { return _registerGeneticFit; }

  int &raWorked() { return _raWorked; }

  static void print(const X2_chromosome *chrm, std::vector<MO *> *ops);

  bool operator<(const X2_chromosome &chrm) const {
    if (_fitness == chrm._fitness)
      return _minsize < chrm._minsize;
    else
      return _fitness < chrm._fitness;
  }

  static bool compare(const X2_chromosome *c1, const X2_chromosome *c2) {
    return (*c1) < (*c2);
  }

  void setMergedSLM(SLM *slm) { _mergedSLM = slm; }

  SLM *getMergedSLM() { return _mergedSLM; }

  SLM **getMergedSLM_ptr() { return &_mergedSLM; }

  const SLM *getMergedSLM() const { return _mergedSLM; }

  bool isDuplicate() const { return isDuplicateChrm() || isDuplicateMerge(); }

  bool isDuplicateChrm() const { return _duplicateChrm; }

  void setDuplicateChrm(bool dup = true) { _duplicateChrm = dup; }

  bool isDuplicateMerge() const { return _duplicateMerge; }

  void setDuplicateMerge(bool dup = true) { _duplicateMerge = dup; }

  bool sameGenes(const X2_chromosome &c) const {
    if (_size != c._size)
      return false;
    for (int i = 0; i < _size; ++i) {
      const MergeCandidates::CandidateList_t &myGene = gene(i);
      const MergeCandidates::CandidateList_t &cGene = c.gene(i);
      if (myGene.size() != cGene.size())
        return false;
      for (size_t j = 0; j < myGene.size(); ++j) {
        if (myGene[j] != cGene[j])
          return false;
      }
    }
    return true;
  }

#if DISPLAY_MERGES
  std::vector<std::pair<int, int>> &getMerges() { return _merges; }
  const std::vector<std::pair<int, int>> &getMerges() const { return _merges; }
#endif
};

class X2Population {
private:
  X2_chromosome **_individuals;
  int _size;

  friend std::ostream &operator<<(std::ostream &os, const X2_chromosome &chrm);

public:
  explicit X2Population(int size);
  X2Population(const X2Population &pop);
  X2Population &operator=(const X2Population &pop);

  ~X2Population();
  /**
   * Cleanup internal pointer structure:
   * Here _individuals pointer is deleted.
   */
  void releaseAll();
  void releaseNonNeeded();

  int size() const { return _size; }

  void set(X2_chromosome *chrm, int index) { _individuals[index] = chrm; }

  bool contains(X2_chromosome *chrm) {
    for (int i = 0; i < _size; ++i)
      if (_individuals[i] == chrm)
        return true;
    return false;
  }

  const X2_chromosome *getIndividual(int index) const {
    return _individuals[index];
  }

  X2_chromosome *&getIndividual(int index) { return _individuals[index]; }

  const X2_chromosome *operator[](int index) const {
    return _individuals[index];
  }

  X2_chromosome *&operator[](int index) { return _individuals[index]; }

  X2_chromosome *getFittest() {
    if (_size == 0)
      return 0;
    float fit = _individuals[0]->fitness();
    int fit_i = 0;
    for (int i = 1; i < _size; ++i) {
      if (_individuals[i]->fitness() < fit) {
        fit = _individuals[i]->fitness();
        fit_i = i;
      }
    }

    return _individuals[fit_i];
  }

  X2_chromosome *getSmallest() {
    if (_size == 0)
      return 0;
    int size = _individuals[0]->getMICount();
    int smallest = 0;
    for (int i = 1; i < _size; ++i) {
      if (_individuals[i]->getMICount() < size) {
        size = _individuals[i]->getMICount();
        smallest = i;
      }
    }
    return _individuals[smallest];
  }

  void sort() {
    std::sort(_individuals, _individuals + _size, X2_chromosome::compare);
  }

  int countInvalidRegGA() const {
    int invalidRegGACount = 0;
    for (int i = 0; i < _size; ++i)
      if (getIndividual(i)->registerGeneticFit() == -2)
        ++invalidRegGACount;
    return invalidRegGACount;
  }
};

void combine(X2_chromosome *parent1, X2_chromosome *parent2,
             X2_chromosome *child1, X2_chromosome *child2, uint *seed);
void combine_default(X2_chromosome *parent1, X2_chromosome *parent2,
                     X2_chromosome *child1, X2_chromosome *child2, uint *seed);
void mutate(X2_chromosome *next, float chrm_fitness, float best_fitness,
            int regGen, float &noMergeProb, uint *seed);
float fitness(SLM *slm, Processor *pro, X2_chromosome *gene, int bestSize,
              const Context &ctx);
void genetic(SLM *slm, Processor *pro, const Context &ctx);
X2_chromosome *tournamentSelect(X2Population &population, int tournamentSize,
                                uint *seed);

bool checkRegCouplings(const RegisterCoupling &coulings, char32_t arg1,
                       char32_t arg2);
bool isMergeable(MO *m, Processor *pro);
/**
 * \brief Check, if two MOs might be merged.
 *
 * Checks for dependencies between the MOs and the mergeability of their
 * arguments. This function DOES NOT CHECK, if a _X2-version of the operations
 * is available, as this is already checked by isMergeable!
 */
bool areMergeable(MO *first, MO *second, int slmId,
                  const RegisterCoupling &couplings, Processor *pro);
SLM *X2Automerge(SLM *slm, Processor *pro, gen_X2::X2_chromosome *schedulable,
                 int &noMerged, const Context &ctx);
enum class REG_PAIR_POS { FIRST, SECOND };
bool getMatchingX2Reg(char32_t fixReg, REG_PAIR_POS pos, bool concurrent,
                      int &regNo, int &regFile);

} // namespace gen_X2

#endif /* HEADER_GEN_MERGING_H_ */
