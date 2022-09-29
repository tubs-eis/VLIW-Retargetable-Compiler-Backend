// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef HEADER_GA_STATS_H_
#define HEADER_GA_STATS_H_

#include <ostream>
#include <set>
#include <unordered_map>
#include <vector>

#include "Constants.h"
#include "global.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

class MI;
class MO;
class Program;

namespace gen_X2 {
class X2_chromosome;
}

namespace ga_stats {

class NumberSequence {
private:
  std::vector<int> _values;
  bool _binary;

public:
  NumberSequence(const int *values, unsigned int count, bool binary = false);
  explicit NumberSequence(const std::vector<int> ints, bool binary = false);
  explicit NumberSequence(const gen_X2::X2_chromosome *chrm);
  explicit NumberSequence(const std::vector<MO *> *mos);
  explicit NumberSequence(const Program *instructions);
  NumberSequence(const NumberSequence &seq);

  NumberSequence &operator=(const NumberSequence &seq);

  unsigned int length() const { return _values.size(); }

  int value(int index) const { return _values[index]; }

  bool operator==(const NumberSequence &other) const {
    if (length() != other.length())
      return false;
    for (unsigned int i = 0; i < length(); ++i)
      if (value(i) != other.value(i))
        return false;
    return true;
  }

  bool operator<(const NumberSequence &other) const {
    return this->lessThan(other);
  }

  bool lessThan(const NumberSequence &other, bool debug = false) const {
    if (debug) {
      std::cout << "NumberSequence::operator<" << std::endl;
      std::cout << "   this.length() = " << length() << std::endl;
      std::cout << "  other.length() = " << other.length() << std::endl;
    }
    if (length() < other.length())
      return true;
    if (length() > other.length())
      return false;
    for (unsigned int i = 0; i < length(); ++i) {
      if (value(i) < other.value(i))
        return true;
      if (value(i) > other.value(i))
        return false;
    }
    return false; // this == equal
  }

  std::ostream &print(std::ostream &os) const;
};

std::ostream &operator<<(std::ostream &os, const NumberSequence &seq);

} // namespace ga_stats

namespace std {

template <> struct hash<ga_stats::NumberSequence> {
  static const int FACTORS[];
  static const int FACTOR_COUNT;

  static int GET_FACTOR(int i) { return FACTORS[i % FACTOR_COUNT]; }

  size_t operator()(const ga_stats::NumberSequence &seq) const {
    size_t h = 0x83f2a5b3;
    for (unsigned int i = 0; i < seq.length(); ++i)
      h += GET_FACTOR(i) * seq.value(i);
    return h;
  }
};

template <> struct less<ga_stats::NumberSequence> {
  bool operator()(const ga_stats::NumberSequence &x,
                  const ga_stats::NumberSequence &y) const {
    return x < y;
  }
};

} // namespace std

namespace ga_stats {

struct ChromosomeFitness {
  int primaryFitness;
  double transitionEnergy;
  ChromosomeFitness(int fitness = 0, double transitionEnergy = -1)
      : primaryFitness(fitness), transitionEnergy(transitionEnergy) {}

  ChromosomeFitness(const ChromosomeFitness &r) {
    primaryFitness = r.primaryFitness;
    transitionEnergy = r.transitionEnergy;
  }

  void setTransitionEnergy(double transitionEnergy) {
    this->transitionEnergy = transitionEnergy;
  }

  /**
   * Primary Fitness Metric.
   * in RA nonAllocatable Register.
   * in Scheduling Size of SLM.
   * @param fitness
   */
  void setFitness(int fitness) { this->primaryFitness = fitness; }

  /**
   * Primary Fitness Metric.
   * in RA nonAllocatable Register.
   * in Scheduling Size of SLM.
   * @param fitness
   */
  int getFitness() const { return primaryFitness; }

  double getTransitionEnergy() const { return transitionEnergy; }

  void incrementPrimary(int fitness) { primaryFitness += fitness; }

  bool operator<(const ChromosomeFitness &fit) const {
    if (primaryFitness < fit.primaryFitness) {
      return true;
    } else if (primaryFitness > fit.primaryFitness) {
      return false;
    } else { // _chromosomeFitness == fit.getFitness
      // todo:fixme with correct transitions
      bool fitness =
          compareTransitionEnergy(transitionEnergy, fit.transitionEnergy);
      return fitness;
    }
  }

  bool operator==(const ChromosomeFitness &fit) const {
    if (primaryFitness == fit.primaryFitness) {
      return fabs(transitionEnergy - fit.transitionEnergy) <
             params.raRegFitnessEqualIgnoreForSecondarySelection;
    }
    return false;
  }

  std::string toString() const {
    std::stringstream ss;
    ss.precision(numberResolution);
    ss << std::fixed;
    ss << "Fit: " << primaryFitness << " Energy [pJ]:" << transitionEnergy
       << " ";
    return ss.str();
  }
};

// typedef std::unordered_map<ga_stats::NumberSequence, ChromosomeFitness>
//    ChromosomesSet;
typedef std::unordered_map<ga_stats::NumberSequence,
                           ga_stats::ChromosomeFitness>
    ChromosomesSet;

/***
 *
 * @param seq Key to search for
 * @param set set contianing all chromosomes and fitnesses
 * @return True and Key if found, false and Empty Valu if not found
 */
std::pair<bool, ga_stats::ChromosomeFitness>
already_processed(const ga_stats::NumberSequence &seq,
                  const ChromosomesSet &set);
void add_processed(const ga_stats::NumberSequence &seq,
                   ga_stats::ChromosomeFitness fitness, ChromosomesSet &set);

} // namespace ga_stats

#endif /* HEADER_GA_STATS_H_ */
