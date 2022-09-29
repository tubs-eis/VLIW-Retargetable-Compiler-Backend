// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "ModelImporter.h"
#include <fstream>
#include <iostream>

void printTransition(std::ofstream &out, std::array<int, 8> &baseTransition,
                     std::array<int, 8> &testTransition) {
  for (int i = 0; i < 8; i++) {
    out << baseTransition[i] << ";";
  }
  for (int i = 0; i < 8; i++) {
    out << testTransition[i] << ";";
  }
}

int main() {
  string CSV_FILE =
      "/home/ipa/stuckmann/export/powerDistribution/powerDistribution.csv";
  params.powerOptimizationFile =
      "/localtemp2/stuckmann/tmp/sourceCode/config/modelmaccoefficient.txt";

  std::ofstream dot;
  dot.open(CSV_FILE);
  dot << "B-src0-issue0;B-src1-issue0;B-src2-issue0;B-src3-issue0;B-src0-"
         "issue1;B-src1-issue1;B-src2-issue1;B-src3-issue1;";
  dot << "T-src0-issue0;T-src1-issue0;T-src2-issue0;T-src3-issue0;T-src0-"
         "issue1;T-src1-issue1;T-src2-issue1;T-src3-issue1;power\n";

  std::vector<double> transitionEnergies;
  int error = 0;
  int counter = 0;
  for (int k = 0; k < 4; k++) {
    string CSV_FILESinglePort =
        "/home/ipa/stuckmann/export/powerDistribution/powerDistribution" +
        std::to_string(k) + ".csv";
    std::ofstream csvRes;
    csvRes.open(CSV_FILESinglePort);
    csvRes << "B-src0-issue0;B-src1-issue0;B-src2-issue0;B-src3-issue0;B-src0-"
              "issue1;B-src1-issue1;B-src2-issue1;B-src3-issue1;";
    csvRes << "T-src0-issue0;T-src1-issue0;T-src2-issue0;T-src3-issue0;T-src0-"
              "issue1;T-src1-issue1;T-src2-issue1;T-src3-issue1;power\n";

    for (int i = 0; i < 64; i++) {
      for (int j = 0; j < 64; j++) {
        //        if (i != j) {
        std::array<int, 8> baseTransition = {0};
        std::array<int, 8> testTransition = {0};
        baseTransition[k] = i;
        testTransition[k] = j;

        counter++;

        printTransition(dot, baseTransition, testTransition);
        printTransition(csvRes, baseTransition, testTransition);
        double energy = ModelImporter::getInstance().predictTransitionEnergy(
            baseTransition, testTransition);
        transitionEnergies.push_back(energy);
        ;
        if (energy < -1.0) {
          error++;
          ModelImporter::getInstance().predictTransitionEnergy(baseTransition,
                                                               testTransition);
        }
        dot << energy << "\n";
        csvRes << energy << "\n";
        //        }
      }
    }
    csvRes.close();
  }
  cout << "Single Transition " << counter << endl;

  //  // add MAC data
  //  for (int k = 0; k < 2; k++) {
  //    int index = 4+ k*2;
  //    for (int i = 0; i < 32; i++) {
  //      for (int j = 0; j < 32; j++) {
  ////        if (i != j) {
  //          std::array<int, 8> baseTransition = {0};
  //          std::array<int, 8> testTransition = {0};
  //          baseTransition[index] = 2*i;
  //          baseTransition[index+1] = 2*i + 1;
  //          testTransition[index] = 2*j;
  //          testTransition[index+1] = 2*j + 1;
  //
  //          printTransition(dot, baseTransition, testTransition);
  //           double energy =
  //           ModelImporter::getInstance().predictTransitionEnergy(
  //                     baseTransition, testTransition);
  //           if (energy < -1.0){
  //             error++;
  //           }
  //                  dot << energy   << "\n";
  ////        }
  //      }
  //    }
  //  }

  dot.close();

  std::cout << error << " Number of < -1 occured!" << std::endl;

  //  std::array<double, 48> v;
  //  v.fill(0);
  //  std::cout << "zero array:" << ModelImporter::predictTransitionEnergy(v)
  //            << std::endl;
  //  v[0] = 1;
  //  std::cout << "zero array + [0]=1:" << importer.predictTransitionEnergy(v)
  //            << std::endl;
  //  v[0] = 0;
  //  v[1] = 1;
  //  std::cout << "zero array + [1]=1:" << importer.predictTransitionEnergy(v)
  //            << std::endl;
  //  v[0] = 1;
  //  std::cout << "zero array + [0]=1; [1]=1:"
  //            << importer.predictTransitionEnergy(v) << std::endl;
}