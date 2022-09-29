// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef EIS_OSC_ML_SCHEDULE_H
#define EIS_OSC_ML_SCHEDULE_H

#include "ctype.h"
#include <algorithm>
#include <fstream>
#include <string>

std::map<int, int> getMLWeights(std::vector<MO *> *mos, int slmID,
                                std::string model_filepath) {

  std::ifstream input(model_filepath);
  if (!input) {
    std::cerr << "Machine Learning Weights File does not exist "
              << model_filepath << endl;
    exit(-1);
  }

  bool correctSLM = false;
  std::string line;
  map<int, int> weightMap;
  while (std::getline(input, line)) {

    // check if correct SLM ID
    if (line.find("slm") != string::npos) {
      auto index = line.find(" ");
      int slmNumber = std::stoi(line.substr(index + 1));
      if (slmNumber == slmID) {
        correctSLM = true;
        continue;
      } else {
        correctSLM = false;
      }
    }
    if (correctSLM) {
      if (line.find("slm") == string::npos and
          !std::all_of(line.begin(), line.end(), [](char c) {
            return std::isspace(static_cast<unsigned char>(c));
          })) {
        auto index = line.find(" ");
        int id = std::stoi(line.substr(0, index));
        int weight = std::stoi(line.substr(index + 1));

        weightMap.insert(pair<int, int>(id, weight));
      }
    }
  }

  std::vector<int> result;
  for (auto mo : *mos) {
    int moID = mo->getID();
    int exists = 0;
    if (weightMap.find(moID) == weightMap.end()) {
      std::cout << "@@@@ MO id " << moID << "  \n";
    }
    //    std::cout << "Mo id "<< moID << " " << weightMap[moID] << std::endl;

    result.push_back(weightMap[moID]);
  }

  return weightMap;

  // return result;
}

#endif // EIS_OSC_ML_SCHEDULE_H
