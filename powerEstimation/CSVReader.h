// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef TESTER_CSVREADER_H
#define TESTER_CSVREADER_H

#include <boost/token_functions.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <string>
#include <vector>

std::vector<std::vector<std::string>> readFile(std::string filename) {
  std::ifstream infile(filename);
  std::vector<std::vector<std::string>> result;

  std::string line;
  while (std::getline(infile, line)) {
    std::vector<std::string> vec;
    using namespace boost;
    tokenizer<escaped_list_separator<char>> tk(
        line, escaped_list_separator<char>('\\', ' ', '\"'));
    for (tokenizer<escaped_list_separator<char>>::iterator i(tk.begin());
         i != tk.end(); ++i) {
      vec.push_back(*i);
    }
    result.push_back(vec);
  }
  infile.close();
  return result;
}

#endif // TESTER_CSVREADER_H
