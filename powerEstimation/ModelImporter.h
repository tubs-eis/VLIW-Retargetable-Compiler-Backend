// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_MODELIMPORTER_H
#define SCHEDULER_MODELIMPORTER_H

#include <MI.h>
#include <array>
#include <global.h>
#include <string>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

class ModelImporter {

private:
  std::string _file;
  std::string delimiter = ",";

  double intercept = 0;
  double singleTerms[96];
  double interactionTerms[95][96] = {0};
  //  boost::numeric::ublas::matrix<double> weights;
  //  boost::numeric::ublas::vector<double> singleTermBoost;

  int vectorSize = 96;

  ModelImporter();
  ModelImporter(ModelImporter const &);
  void operator=(ModelImporter const &);

  void importModel();

  bool goodTransition(int previousValue, int currentValue) const;

  std::array<double, 12> getBitActivationSequence(int previousRegister,
                                                  int currentRegister) const;

public:
  double predictTransitionEnergy(const std::array<int, 96> &inputVector) const;
  //  double predictTransitionEnergy(boost::numeric::ublas::vector<double>
  //  &inputVector) const;

  /***
   * Int Array Register Sorting:
   * Issue0_Src0, Issue0_Src1, Issue1_Src0, Issue1_Src1, Issue0_Src2,
   * Issue0_Src3, Issue1_Src2, Issue1_Src3
   * @param readPreviousRegister
   * @param readCurrentRegister
   * @return
   */
  double
  predictTransitionEnergy(const std::array<int, 8> &readPreviousRegister,
                          const std::array<int, 8> &readCurrentRegister) const;

public:
  static ModelImporter &getInstance() {
    static ModelImporter instance;
    return instance;
  }
};

#endif // SCHEDULER_MODELIMPORTER_H
