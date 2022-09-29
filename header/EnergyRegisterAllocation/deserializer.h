// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_DESERIALIZER_H
#define SCHEDULER_DESERIALIZER_H

#include <fstream>
#include <string>

class EnergyChromosome;
class Deserializer {
public:
  static Deserializer &getInstance() {
    static Deserializer instance;
    return instance;
  }

private:
  std::vector<std::unordered_map<int, int>> chromosomeVirtualPhysicalMapping;
  Deserializer() {
    std::string line;

    std::ifstream myfile;
    myfile.open("/localtemp/stuckmann/tmp/generation/combine.txt");

    int i = 0;
    std::unordered_map<int, int> virtual2Physical;
    while (std::getline(myfile, line)) {
      if (line.empty()) {
        //        std::cout << "New Chromosome!" << std::endl;
        chromosomeVirtualPhysicalMapping.push_back(virtual2Physical);
        virtual2Physical.erase(virtual2Physical.begin(),
                               virtual2Physical.end());
      } else {
        int parting = line.find(" ");
        int virtualReg = stoi(line.substr(0, parting));
        int physReg = stoi(line.substr(parting + 1, line.size()));
        virtual2Physical.insert(std::pair<int, int>{virtualReg, physReg});
      }
      int n = 3;
      i++;
    }
    myfile.close();
    int n = 3;
  }

  Deserializer(Deserializer const &);
  void operator=(Deserializer const &);

public:
  std::unordered_map<int, int> deserialize(int individual) {
    return chromosomeVirtualPhysicalMapping[individual];
  }

  int getCombinationCount() { return chromosomeVirtualPhysicalMapping.size(); }
};

#endif // SCHEDULER_DESERIALIZER_H
