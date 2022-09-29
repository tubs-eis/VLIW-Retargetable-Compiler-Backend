// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_COMBINERETURN_H
#define SCHEDULER_COMBINERETURN_H

#include <sstream>
#include <string>
#include <vector>

namespace portOptReg {
class COMBINE_RETURN {
public:
  bool combine = false;
  bool conflict = false;
  std::vector<int> hierarchyLevel;
  std::vector<int> maxHierarchyLevel;
  int virtualRegisterCount = 0;
  int virtualRegisterCountMax = 0;
  int numberCombineRoots = 1;
  COMBINE_RETURN() { combine = false; }
  COMBINE_RETURN(bool combine, std::vector<int> hierarchyLevel,
                 std::vector<int> maxHierarchyLevel) {
    this->combine = combine;
    this->hierarchyLevel = hierarchyLevel;
    this->maxHierarchyLevel = maxHierarchyLevel;
  }

  void append(COMBINE_RETURN &other) {
    if (other.combine) {
      combine = true;
      hierarchyLevel.insert(hierarchyLevel.end(), other.hierarchyLevel.begin(),
                            other.hierarchyLevel.end());
      maxHierarchyLevel.insert(maxHierarchyLevel.end(),
                               other.maxHierarchyLevel.begin(),
                               other.maxHierarchyLevel.end());
    }
    if (not conflict) {
      conflict = other.conflict;
    }
    virtualRegisterCount += other.virtualRegisterCount;
    virtualRegisterCountMax += other.virtualRegisterCountMax;
  }

  void add(int level, int maxLevel) {
    hierarchyLevel.push_back(level);
    maxHierarchyLevel.push_back(maxLevel);
  }

  std::string toString() const {
    std::stringstream ss;
    ss << " CombineHierarchy: ";
    double average = 0;
    for (int i = 0; i < hierarchyLevel.size(); i++) {
      ss << hierarchyLevel[i] << "/" << maxHierarchyLevel[i] << " ";
      average += maxHierarchyLevel[i] - hierarchyLevel[i];
    }
    average /= hierarchyLevel.size();
    ss << " avg depth: " << average;
    ss << " Number Register " << virtualRegisterCount;
    ss << "/ " << virtualRegisterCountMax;
    ss << " Number Roots " << numberCombineRoots;
    return ss.str();
  }
};
} // namespace portOptReg

#endif // SCHEDULER_COMBINERETURN_H
