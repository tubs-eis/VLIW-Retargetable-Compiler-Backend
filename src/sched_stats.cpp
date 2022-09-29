// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "sched_stats.h"
#include "MO.h"
#include "SLM.h"
#include <iomanip>
using namespace std;

SchedulingStats *SchedulingStats::_instance = 0;

void SchedulingStats::init(const char *path) {
  if (!SchedulingStats::_instance)
    SchedulingStats::_instance = new SchedulingStats(path);
}

SchedulingStats *SchedulingStats::instance() { return _instance; }

bool SchedulingStats::hasInstance() { return _instance != 0; }

void SchedulingStats::release() {
  delete SchedulingStats::_instance;
  SchedulingStats::_instance = 0;
}

SchedulingStats::SchedulingStats(const char *path) : _ostream(path) {}

SchedulingStats::~SchedulingStats() {
  if (_ostream.is_open())
    _ostream.close();
}

bool SchedulingStats::containsGeneratedMV(SLM *slm) {
  std::vector<MO *> *ops = slm->getOperations();
  for (std::vector<MO *>::iterator it = ops->begin(); it != ops->end(); ++it) {
    MO *mo = *it;
    if (mo->getLineNumber() != 0)
      continue;
    if (!strncmp(mo->getOperation()->getName().c_str(), "MV", 2))
      return true;
  }
  return false;
}

void SchedulingStats::printSchedulingStats(uint round, SLM *slm) {
#pragma omp critical
  {
    _ostream << "[SCHED] SLM " << setw(3) << slm->getOriginalID() << " round "
             << setw(4) << round << " size " << setw(3);
    if (slm->getSize() == 0)
      _ostream << (slm->getBranchOperation()
                       ? (slm->getBranchOperation()->isReorderable() ? 4 : 1)
                       : 0);
    else
      _ostream << slm->getSize();
    if (containsGeneratedMV(slm))
      _ostream << " (MV)";
    else
      _ostream << " (No MV)";
    _ostream << endl;
  }
}
