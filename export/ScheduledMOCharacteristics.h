// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_SCHEDULEDMOCHARACTERISTICS_H
#define SCHEDULER_SCHEDULEDMOCHARACTERISTICS_H

#include "gen_sched.h"
#include <map>

struct chromsomeSummary {
  int weight = -1;
  bool parting = false;
  bool alone = false;
};

class ScheduledMOCharacteristics {
public:
  static ScheduledMOCharacteristics &getInstance() {
    static ScheduledMOCharacteristics _instance;
    return _instance;
  }
  ~ScheduledMOCharacteristics() {}
  void reset() { moLineNumberMap.clear(); }
  void setID(int id, int lineNumber) {
    moLineNumberMap.insert(std::pair<int, int>(id, lineNumber));
  }
  void setChromosome(int id, int weight, bool parting, bool alone) {
    chromsomeSummary summary;
    if (weight < 0) {
      throw "weight is negative!";
    }
    summary.weight = weight;
    summary.parting = parting;
    summary.alone = alone;
    moChromosomes.insert(std::pair<int, chromsomeSummary>(id, summary));
  }
  int getLineNumber(int id) {
    if (moLineNumberMap.size() == 0) {
      throw std::runtime_error(
          "Trying to access ScheduledMOCharacteristic that has "
          "not been initialized. "
          "You have to print readable assembler to "
          "generate this Data structure before accessing it.");
    }
    return moLineNumberMap[id];
  }

  chromsomeSummary getChromosomeSummary(int id) {
    auto it = moChromosomes.find(id);
    if (it != moChromosomes.end()) {
      return moChromosomes[id];
    } else {
      chromsomeSummary s;
      return s;
    }
  }

  std::string getChromosome2String(int id) {
    auto summary = getChromosomeSummary(id);
    stringstream ss;
    ss << "chromosomeWeight=";
    ss << summary.weight;
    ss << ";";
    ss << "alone=";
    if (summary.alone) {
      ss << "1";
    } else {
      ss << "0";
    }
    ss << ";parting=";
    if (summary.parting) {
      ss << "1";
    } else {
      ss << "0";
    }
    ss << ";";
    return ss.str();
  }

  void setEnable(bool enable) { enabled = enable; }

  bool isEnabled() const { return enabled; }

private:
  std::map<int, int> moLineNumberMap;
  std::map<int, chromsomeSummary> moChromosomes;
  bool enabled = false;
  ScheduledMOCharacteristics() {
  } // verhindert, dass ein Objekt von außerhalb von N erzeugt wird.
  // protected, wenn man von der Klasse noch erben möchte
  ScheduledMOCharacteristics(const ScheduledMOCharacteristics &); /* verhindert,
dass eine weitere Instanz via Kopier-Konstruktor erstellt werden kann */
  ScheduledMOCharacteristics &
  operator=(const ScheduledMOCharacteristics
                &); // Verhindert weitere Instanz durch Kopie
};

#endif // SCHEDULER_SCHEDULEDMOCHARACTERISTICS_H
