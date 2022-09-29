// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "DoubleRegister.h"
#include "register.h"
#include "virtual_reg.h"
#include <iostream>

std::pair<int, int> getConcurrentDoubleFreeRegisters(const ass_reg_t blocked) {
  ass_reg_t temp = 1;
  for (unsigned int i = 0; i < registers::getNumRegister1RF() - 1; i += 2)
    if (!(blocked & (temp << i)) && !(blocked & (temp << (i + 1))))
      return {i, i + 1};
  return {-1, -1};
}

std::pair<int, int> getParallelDoubleFreeRegisters(const ass_reg_t *blocked) {
  ass_reg_t temp = 1;
  for (unsigned int i = 0; i < registers::getNumRegister1RF(); i++)
    if (!(blocked[0] & (temp << i)) && !(blocked[1] & (temp << i)))
      return {i, i + numRegisters};
  return {-1, -1};
}

int getDoubleFreeRegisters(const ass_reg_t *blocked, int regFile,
                           bool concurrent) {
  return getDoubleFreeRegistersPair(blocked, regFile, concurrent).first;
}

std::pair<int, int> getDoubleFreeRegistersPair(const ass_reg_t *blocked,
                                               int regFile, bool concurrent) {
  if (concurrent)
    return getConcurrentDoubleFreeRegisters(blocked[regFile]);
  else
    return getParallelDoubleFreeRegisters(blocked);
}

std::pair<int, int> getConcurrentDoubleFreeRegisters(ass_reg_t *blocked,
                                                     uint *seed) {
  std::vector<std::pair<int, int>> candidates;
  ass_reg_t temp = 1;
  for (int j = 0; j < registers::getNumRegisterFiles(); j++) {
    for (unsigned int i = 0; i < registers::getNumRegister1RF() - 1; i += 2) {
      if (!(blocked[j] & (temp << i)) && !(blocked[j] & (temp << (i + 1)))) {
        if (j == 0) {
          candidates.push_back({i, i + 1});
        } else {
          candidates.push_back({i + numRegisters, i + numRegisters + 1});
        }
      }
    }
  }

  if (candidates.empty()) {
    return {-1, -1};
  } else {
    int index = rand_r(seed) % candidates.size();
    if (candidates[index].first < numRegisters) {
      // RF0
      blocked[0] |= ((ass_reg_t)1) << candidates[index].first;
      blocked[0] |= ((ass_reg_t)1) << candidates[index].second;
    } else {
      // RF1
      blocked[1] |= ((ass_reg_t)1) << (candidates[index].first - numRegisters);
      blocked[1] |= ((ass_reg_t)1) << (candidates[index].second - numRegisters);
    }
    return candidates[index];
  }
}

std::pair<int, int> getParallelDoubleFreeRegisters(ass_reg_t *blocked,
                                                   uint *seed) {
  std::vector<std::pair<int, int>> candidates;
  ass_reg_t temp = 1;
  for (unsigned int i = 0; i < registers::getNumRegister1RF(); i++) {
    if (!(blocked[0] & (temp << i)) && !(blocked[1] & (temp << i))) {
      candidates.push_back({i, i + numRegisters});
    }
  }
  if (candidates.empty()) {
    return {-1, -1};
  } else {
    int index = rand_r(seed) % candidates.size();
    blocked[0] |= ((ass_reg_t)1) << candidates[index].first;
    blocked[1] |= ((ass_reg_t)1) << candidates[index].first;
    return candidates[index];
  }
}

std::pair<int, int> getDoubleFreeRegistersPairRandom(ass_reg_t *blocked,
                                                     bool concurrent,
                                                     uint *seed) {
  if (concurrent) {
    std::pair<int, int> coupledBlocked =
        getConcurrentDoubleFreeRegisters(blocked, seed);
    if (coupledBlocked.first != -1 and coupledBlocked.second != -1) {
      if (coupledBlocked.second - 1 != coupledBlocked.first) {
        std::stringstream ss;
        ss << "Coupled Register Are not VxR{EVEN} + VxR{UNEVEN}, instead ";
        ss << coupledBlocked.first << "+" << coupledBlocked.second;
        throw std::runtime_error(ss.str());
      }
    }
    return coupledBlocked;
  } else {
    return getParallelDoubleFreeRegisters(blocked, seed);
  }
}
