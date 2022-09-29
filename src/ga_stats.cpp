// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "ga_stats.h"
#include "MI.h"
#include "MO.h"
#include "gen_merging.h"

namespace ga_stats {

NumberSequence::NumberSequence(const int *values, unsigned int count,
                               bool binary)
    : _binary(binary) {
  for (unsigned int i = 0; i < count; ++i)
    _values.push_back(values[i]);
}

NumberSequence::NumberSequence(const std::vector<int> ints, bool binary)
    : _values(ints), _binary(binary) {}

NumberSequence::NumberSequence(const gen_X2::X2_chromosome *chrm)
    : _binary(false) {
  for (int idx = 0; idx < chrm->size(); ++idx) {
    const gen_X2::MergeCandidates::CandidateList_t &gene = chrm->gene(idx);
    if (gene.size() > 1)
      for (size_t i = 0; i < gene.size(); ++i)
        _values.push_back(gene[i]);
  }
}

NumberSequence::NumberSequence(const std::vector<MO *> *mos) : _binary(false) {
  for (unsigned int i = 0; i < mos->size(); ++i) {
    MO *mo = mos->at(i);
    if (mo->getLineNumber()) {
      if (!mo->getSecondLineNumber())
        _values.push_back(mo->getLineNumber());
      else
        _values.push_back(mo->getLineNumber() * 1000 +
                          mo->getSecondLineNumber());
    }
  }
}

NumberSequence::NumberSequence(const Program *instructions) : _binary(false) {
  if (!instructions) {
    _values.push_back(-1);
    return;
  }
  for (auto it = instructions->begin(); it != instructions->end(); ++it) {
    MI *ins = *it;
    if (ins) {
      unsigned int slot = 0;
      MO **mos = ins->getOperations();
      if (mos) {
        while (slot < ins->getNumberIssueSlots()) {
          MO *mo = mos[slot];
          if (mo) {
            for (int i = 0; i < mo->opLengthMultiplier(); ++i)
              _values.push_back(mo->getLineNumber());
            slot += mo->opLengthMultiplier();
          } else {
            _values.push_back(0);
            slot += 1;
          }
        }
      } else {
        _values.push_back(-3);
        _values.push_back(-3);
      }
    } else {
      _values.push_back(-2);
      _values.push_back(-2);
    }
  }
}

NumberSequence::NumberSequence(const NumberSequence &seq)
    : _values(seq._values), _binary(seq._binary) {}

NumberSequence &NumberSequence::operator=(const NumberSequence &seq) {
  _values = seq._values;
  _binary = seq._binary;
  return *this;
}

std::ostream &NumberSequence::print(std::ostream &os) const {
  char fillchar = os.fill('0');
  int w = 4;
  if (_binary)
    w = 1;
  for (unsigned int i = 0; i < length(); ++i)
    os << std::hex << std::setw(w) << _values[i];
  os.fill(fillchar);
  os << dec;
  return os;
}

std::ostream &operator<<(std::ostream &os, const NumberSequence &seq) {
  seq.print(os);
  return os;
}

std::pair<bool, ga_stats::ChromosomeFitness>
already_processed(const ga_stats::NumberSequence &seq,
                  const ChromosomesSet &set) {
  ChromosomesSet::const_iterator it = set.find(seq);
  if (it == set.cend()) {
    return std::make_pair(false, ga_stats::ChromosomeFitness());
  } else {
    return std::make_pair(true, it->second);
  }
}

void add_processed(const ga_stats::NumberSequence &seq,
                   ga_stats::ChromosomeFitness fitness, ChromosomesSet &set) {
  ChromosomesSet::const_iterator it = set.find(seq);
  if (it == set.cend())
    set[seq] = fitness;
  else if (fitness < set[seq])
    set[seq] = fitness;
}

} // namespace ga_stats

namespace std {
const int hash<ga_stats::NumberSequence>::FACTORS[] = {11,  13,  7,  137,
                                                       257, 199, 269};
const int hash<ga_stats::NumberSequence>::FACTOR_COUNT = 7;
} // namespace std
