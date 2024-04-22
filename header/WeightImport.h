// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_WEIGHTIMPORT_H
#define SCHEDULER_WEIGHTIMPORT_H

#include "MO.h"
#include "ctype.h"
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

std::string trim(const std::string &str) {
  auto begin = str.begin();
  auto end = str.end();

  // Find the first non-space character from the beginning
  begin = std::find_if(begin, end,
                       [](unsigned char ch) { return !std::isspace(ch); });

  // Find the first non-space character from the end
  end = std::find_if(str.rbegin(), std::string::const_reverse_iterator(begin),
                     [](unsigned char ch) { return !std::isspace(ch); })
            .base();

  return std::string(begin, end);
}

std::string to_lower(const std::string &s) {
  std::string lower_s = s;
  std::transform(lower_s.begin(), lower_s.end(), lower_s.begin(), ::tolower);
  return lower_s;
}

bool find_case_insensitive(const std::string &line, const std::string &target) {
  std::string lower_line = to_lower(line);
  std::string lower_target = to_lower(target);
  return lower_line.find(lower_target) != std::string::npos;
}

// std::vector<int> getMLWeights(std::vector<MO *> *mos, int slmID,
std::vector<int> importSLMWeights(std::vector<MO *> *mos, int slmID,
                                  std::string model_filepath,
                                  std::map<int, bool> &alone,
                                  std::map<int, bool> &parting) {

  std::istream *input;
  if (params.slmWeights != "") {
    input = new std::istringstream(params.slmWeights);
  } else {
    input = new std::ifstream(model_filepath);
    if (!input) {
      std::cerr << "Machine Learning Weights File does not exist "
                << model_filepath << endl;
      // exception
      throw std::runtime_error("Machine Learning Weights File does not exist");
    }
  }

  bool correctSLM = false;
  std::string line;
  map<int, int> weightMap;
  vector<int> moOrdering;
  while (std::getline(*input, line)) {

    // check if correct SLM ID
    if (find_case_insensitive(line, "slm")) {
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

        std::istringstream iss(line);
        int id;
        bool aloneBit, partingBit;

        if (iss >> id >> aloneBit >> partingBit) {
          moOrdering.push_back(id);
          alone.insert(pair<int, bool>(id, aloneBit));
          parting.insert(pair<int, bool>(id, partingBit));
        } else {
          std::cerr << "Error reading line: " << line << endl;
          throw std::runtime_error("Error reading line. MO_ID start with 0 at "
                                   "first MO. Should be FORMAT: "
                                   "MO_ID ALONE_BIT PARTING_BIT");
        }
      }
    }
  }

  delete input;
  input = nullptr;

  //  int maxWeight = moOrdering.size();
  //  for (auto mo : moOrdering){
  //    weightMap.insert(pair<int,int>(mo, maxWeight) );
  //    maxWeight--;
  //  }

  return moOrdering;

  // return result;
}

#endif // SCHEDULER_WEIGHTIMPORT_H
