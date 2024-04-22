// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef HEADER_UTILITY_H_
#define HEADER_UTILITY_H_

#include <chrono>
#include <cstdlib>
#include <string>
#include <vector>

#ifdef __APPLE__
typedef unsigned int uint;
#endif

class Operation;

namespace util {

float uniRandom(uint *seed);
/**
 * A randomPopulation number from [low, high], i.e., both inclusive!
 */
int uniRandom(int low, int high, uint *seed);

template <class T> void swap(T *list, int i1, int i2) {
  T tmp = list[i1];
  list[i1] = list[i2];
  list[i2] = tmp;
}

template <class T> void swap(std::vector<T> &vector, int i1, int i2) {
  T tmp = vector[i1];
  vector[i1] = vector[i2];
  vector[i2] = tmp;
}

template <class T> void swap(T &a, T &b) {
  T tmp = a;
  a = b;
  b = tmp;
}

/** Fisher-Yates shuffle.
 *  https://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle
 *
 *  Permutes the elements in the given list (of length len).
 */
void FYshuffle(int *list, int len, uint *seed);
template <class T> void FYshuffle(std::vector<T> &vector, uint *seed) {
  size_t len = vector.size();
  if (len > 1) {
    for (size_t i = 0; i < len - 1; ++i) {
      int j = uniRandom(i, len - 1, seed);
      swap(vector, i, j);
    }
  }
}

void integerSequence(int *list, int len);
/**
 * Generate a randomPopulation permutation of 0..(len-1)
 */
void randomPermutation(int *list, int len, uint *seed);

class Timer {
public:
  typedef std::chrono::time_point<std::chrono::high_resolution_clock>
      time_point;
  typedef std::chrono::duration<double> duration;

private:
  time_point _start;
  duration _elapsed;

public:
  Timer() : _start(std::chrono::high_resolution_clock::now()), _elapsed(0) {}

  void start() { _start = std::chrono::high_resolution_clock::now(); }

  void stop() {
    auto now = std::chrono::high_resolution_clock::now();
    _elapsed += now - _start;
    _start = std::chrono::high_resolution_clock::now();
  }

  double seconds() { return _elapsed.count(); }

  void reset() { _elapsed = duration::zero(); }
};

bool strStartsWith(const std::string &str, const char *prefix);
bool isOpPair(Operation *op_i, Operation *op_j, const char *op1,
              const char *op2, bool exact = true);
} // namespace util

#endif /* HEADER_UTILITY_H_ */
