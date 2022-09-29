// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#include "CSVReader.h"
#include <boost/token_functions.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

int main() {
  std::string dir = "/localtemp/stuckmann/tmp/InstructionModel.csv";

  auto v = readFile(dir);
  for (auto vec : v) {
    for (int i = 0; i < 5; i++) {
      std::cout << vec[i] << " ";
    }
    std::cout << std::endl;
  }
}