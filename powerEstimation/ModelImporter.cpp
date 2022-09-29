// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "ModelImporter.h"
#include "global.h"
#include <array>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <vector>

ModelImporter::ModelImporter() { //: weights(95,96), singleTermBoost(96){
  _file = params.powerOptimizationFile;
  importModel();
}

void ModelImporter::importModel() {
  std::ifstream csvread;
  csvread.open(_file);
  std::vector<std::array<std::string, 5>> fileContent;
  if (csvread) {
    // Datei bis Ende einlesen und bei ';' strings trennen
    std::string s = "";
    std::array<std::string, 5> line;
    int i = 0;
    while (getline(csvread, s)) {
      size_t pos = 0;
      std::string token;
      int i = 0;
      std::array<std::string, 5> line;
      boost::tokenizer<boost::escaped_list_separator<char>> tok(s);
      for (boost::tokenizer<boost::escaped_list_separator<char>>::iterator beg =
               tok.begin();
           beg != tok.end(); ++beg) {
        line[i] = *beg;
        i++;
      }
      fileContent.push_back(line);
    }
  } else {
    throw "Power Could not open " + _file;
  }
  csvread.close();
  // process fileContent
  intercept = stod(fileContent[1][1]);
  for (int i = 0; i < vectorSize; i++) {
    auto value = stod(fileContent[i + 2][1]);
    singleTerms[i] = value;
    //    singleTermBoost[i] = value;
  }
  // init interaction terms
  for (int i = 0; i < vectorSize - 1; i++) {
    for (int j = 0; j < vectorSize - 1; j++) {
      interactionTerms[i][j] = 0;
      //      weights(i,j) = 0;
    }
  }
  int counter = vectorSize + 2;
  for (int i = 0; i < vectorSize - 1; i++) {
    for (int j = i + 1; j < vectorSize; j++) {
      auto value = stod(fileContent[counter][1]);
      interactionTerms[i][j] = value;
      //      weights(i,j) = value;
      counter++;
    }
  }

  int n = 3;
}

double ModelImporter::predictTransitionEnergy(
    const std::array<int, 96> &inputVector) const {
  //  double result =0;
  // add intercept
  double result = intercept;
  //   //add single Term impacts
  for (int i = 0; i < vectorSize; i++) {
    if (inputVector[i] == 1) {
      result += singleTerms[i];
    }
  }
  // add interaction terms
  for (int i = 0; i < vectorSize - 1; i++) {
    if (inputVector[i] != 0) {
      for (int j = i + 1; j < vectorSize; j++) {
        if (inputVector[i] * inputVector[j] == 1) {
          result += interactionTerms[i][j];
        }
      }
    }
  }
  return result;
}

// double ModelImporter::predictTransitionEnergy(
//    boost::numeric::ublas::vector<double> &inputVector) const {
//
//  double temp = 0;
////  auto temp = boost::numeric::ublas::sum(element_prod(inputVector,
/// singleTermBoost)) + intercept; /  auto  result =
/// boost::numeric::ublas::prod(weights, inputVector) ; /  return temp;
//  auto inputMatrix = boost::numeric::ublas::outer_prod(inputVector,
//  inputVector); return boost::numeric::ublas::prod(weights, inputMatrix) +
//  temp;
//}

bool ModelImporter::goodTransition(int previousValue, int currentValue) const {
  return previousValue != getEmptyPhysicalReadPort() and
         currentValue != getEmptyPhysicalReadPort();
}

double ModelImporter::predictTransitionEnergy(
    const std::array<int, 8> &readPreviousRegister,
    const std::array<int, 8> &readCurrentRegister) const {
  //  boost::numeric::ublas::vector<double> bitCombinationBoost(96);
  //  for(int i=0;i < 96; i ++){
  //    bitCombinationBoost(i) =0;
  //  }
  std::array<int, 96> bitCombination{0};
  for (int i = 0; i < readPreviousRegister.size(); i++) {
    std::bitset<12> changingbits;
    auto currentReg = readCurrentRegister[i];
    if (readCurrentRegister[i] == getEmptyPhysicalReadPort()) {
      currentReg = readPreviousRegister[i];
    }
    if (goodTransition(readPreviousRegister[i], currentReg)) {
      auto previousRegisterBits = bitset<6>(readPreviousRegister[i]);

      auto currentRegisterBits = bitset<6>(currentReg);
      for (int k = 0; k < 6; k++) {
        // set deactivating bits
        if (previousRegisterBits[5 - k] == 1 and
            currentRegisterBits[5 - k] == 0) {
          changingbits[k] = 1;
          //          bitCombinationBoost[i * 12 +k] =1;
          changingbits[k + 6] = 0;
          //          bitCombinationBoost[i * 12 +k + 6] = 0;
        } else if (previousRegisterBits[5 - k] == 0 and
                   currentRegisterBits[5 - k] == 1) {
          changingbits[k] = 0;
          //          bitCombinationBoost[i * 12 +k] = 0;
          changingbits[k + 6] = 1;
          //          bitCombinationBoost[i * 12 +k + 6] = 1;
        } else {
          changingbits[k] = 0;
          //          bitCombinationBoost[i * 12 +k] = 0;
          changingbits[k + 6] = 0;
          //          bitCombinationBoost[i * 12 +k + 6] = 0;
        }
      }
    } else {
      changingbits = std::bitset<12>(0);
    }
    for (int k = 0; k < 12; k++) {
      bitCombination[i * 12 + k] = changingbits[k];
    }
  }
  //  auto oldResult = predictTransitionEnergy(bitCombination);
  //  auto newResult = predictTransitionEnergy(bitCombinationBoost);
  //  for(int i =0; i < 96; i++){
  //    if(bitCombination[i] != bitCombinationBoost[i]){
  //      throw runtime_error("Boost Implementation failed");
  //    }
  //  }
  //  if( fabs(oldResult - newResult) > 0.0001){
  //    throw runtime_error("Boost Implementation failed");
  //  }
  return predictTransitionEnergy(bitCombination);
}