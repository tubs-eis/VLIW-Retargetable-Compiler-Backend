// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "utility.h"
#include "operation.h"

namespace util {

float uniRandom(uint *seed) { return (float)rand_r(seed) / (float)RAND_MAX; }

int uniRandom(int low, int high, uint *seed) {
  int limit = high - low;
  int divisor = RAND_MAX / (limit + 1);
  int retval;

  do {
    retval = rand_r(seed) / divisor;
  } while (retval > limit);

  return low + retval;
}

void FYshuffle(int *list, int len, uint *seed) {
  for (int i = 0; i < len - 1; ++i) {
    int j = uniRandom(i, len - 1, seed);
    swap(list, i, j);
  }
}

void integerSequence(int *list, int len) {
  for (int i = 0; i < len; ++i)
    list[i] = i;
}

void randomPermutation(int *list, int len, uint *seed) {
  integerSequence(list, len);
  FYshuffle(list, len, seed);
}

bool strStartsWith(const std::string &str, const char *prefix) {
  std::string pref(prefix);
  return str.compare(0, pref.length(), pref) == 0;
}

bool isOpPair(Operation *op_i, Operation *op_j, const char *op1,
              const char *op2, bool exact) {
  if (exact) {
    bool res = op_i->getBaseName() == op1 && op_j->getBaseName() == op2;
    return res;
  } else {
    bool res = util::strStartsWith(op_i->getBaseName(), op1) &&
               util::strStartsWith(op_j->getBaseName(), op2);
    return res;
  }
}

} // namespace util
