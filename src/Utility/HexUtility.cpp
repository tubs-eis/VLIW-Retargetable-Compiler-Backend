// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "Utility/HexUtility.h"

unsigned int getDigit(const char *codeLine, int position) {
  unsigned int number = (codeLine[position] - '0');
  return number;
}