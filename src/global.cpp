// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "global.h"

#include <cstdarg>
#include <fstream>
#include <iostream>
#include <string.h>
#include <vector>

/* If you add a new log section here, use a power of two for its value,
 * as the masks are used as bitmasks.
 * You also have to add an entry in the LOG_MASKS map below, so that the
 * log section can be enabled/disabled on the command line:
 *   --enableLog <NAME> looks for an entry with <NAME> and uses the
 *                      associated LOG_M_... mask.
 * To correctly set the 64-bit values, you can use DEF_LOG_M(shift) macro.
 */

#define DEF_LOG_M(shift) (((LOG_MASK_T)1) << (shift))

const LOG_MASK_T LOG_M_ALWAYS = 0;
const LOG_MASK_T LOG_M_BASIC = DEF_LOG_M(0);

const LOG_MASK_T LOG_M_MERGE = DEF_LOG_M(1);
const LOG_MASK_T LOG_M_MERGE_DETAIL = DEF_LOG_M(2);
const LOG_MASK_T LOG_M_MERGE_DEBUG = DEF_LOG_M(3);
const LOG_MASK_T LOG_M_SCHED_SKIP = DEF_LOG_M(4);

const LOG_MASK_T LOG_M_SCHED = DEF_LOG_M(5);
const LOG_MASK_T LOG_M_SCHED_DETAIL = DEF_LOG_M(6);
const LOG_MASK_T LOG_M_SCHED_DEBUG = DEF_LOG_M(7);

const LOG_MASK_T LOG_M_RA = DEF_LOG_M(8);
const LOG_MASK_T LOG_M_RA_DETAIL = DEF_LOG_M(9);
const LOG_MASK_T LOG_M_RA_DEBUG = DEF_LOG_M(10);

const LOG_MASK_T LOG_M_INIT_PROC = DEF_LOG_M(11);
const LOG_MASK_T LOG_M_PARSE = DEF_LOG_M(12);
const LOG_MASK_T LOG_M_CHECK_EXEC = DEF_LOG_M(13);

const LOG_MASK_T LOG_M_RDG = DEF_LOG_M(14);
const LOG_MASK_T LOG_M_RDG_DETAIL = DEF_LOG_M(15);
const LOG_MASK_T LOG_M_RDG_DEBUG = DEF_LOG_M(16);
const LOG_MASK_T LOG_M_RDG_STAT = DEF_LOG_M(17);

const LOG_MASK_T LOG_M_RA_CONFLICT = DEF_LOG_M(18);
const LOG_MASK_T LOG_M_RA_CONFLICT_DETAIL = DEF_LOG_M(19);

const LOG_MASK_T LOG_M_RA_PREP = DEF_LOG_M(20);
const LOG_MASK_T LOG_M_SCHED_SIZE_HIST = DEF_LOG_M(21);
const LOG_MASK_T LOG_M_MERGE_SIZE_HIST = DEF_LOG_M(22);
const LOG_MASK_T LOG_M_RA_ROUNDS_HIST = DEF_LOG_M(23);
const LOG_MASK_T LOG_M_RA_SIZE_HIST = DEF_LOG_M(24);
const LOG_MASK_T LOG_M_REG_ENERGY_DEBUG = DEF_LOG_M(25);
const LOG_MASK_T LOG_M_REG_DETAIL = DEF_LOG_M(26);
const LOG_MASK_T LOG_M_REG_ENERGY = DEF_LOG_M(27);

LOG_MAP_T LOG_MASKS = {
    {"BASIC", LOG_M_BASIC},
    {"MERGE", LOG_M_MERGE},
    {"MERGE_DETAIL", LOG_M_MERGE | LOG_M_MERGE_DETAIL},
    {"MERGE_DEBUG", LOG_M_MERGE | LOG_M_MERGE_DETAIL | LOG_M_MERGE_DEBUG},
    {"SCHEDSKIP", LOG_M_SCHED_SKIP},
    {"SCHED", LOG_M_SCHED},
    {"SCHED_DETAIL", LOG_M_SCHED | LOG_M_SCHED_DETAIL},
    {"SCHED_DEBUG", LOG_M_SCHED | LOG_M_SCHED_DETAIL | LOG_M_SCHED_DEBUG},
    {"RA", LOG_M_RA},
    {"RA_DETAIL", LOG_M_RA | LOG_M_RA_DETAIL},
    {"RA_DEBUG", LOG_M_RA | LOG_M_RA_DETAIL | LOG_M_RA_DEBUG},
    {"INIT_PROC", LOG_M_INIT_PROC},
    {"PARSE", LOG_M_PARSE},
    {"ISEXEC", LOG_M_CHECK_EXEC},
    {"RDG", LOG_M_RDG},
    {"RDG_DETAIL", LOG_M_RDG | LOG_M_RDG_DETAIL},
    {"RDG_DEBUG", LOG_M_RDG | LOG_M_RDG_DETAIL | LOG_M_RDG_DEBUG},
    {"RDG_STAT", LOG_M_RDG_STAT},
    {"RA_CONFLICT", LOG_M_RA_CONFLICT},
    {"RA_CONFLICT_DETAIL", LOG_M_RA_CONFLICT | LOG_M_RA_CONFLICT_DETAIL},
    {"RA_PREP", LOG_M_RA_PREP},
    {"SCHED_SIZE_HIST", LOG_M_SCHED_SIZE_HIST},
    {"MERGE_SIZE_HIST", LOG_M_MERGE_SIZE_HIST},
    {"RA_ROUNDS_HIST", LOG_M_RA_ROUNDS_HIST},
    {"RA_SIZE_HIST", LOG_M_RA_SIZE_HIST},
    {"RA_REG_DEBUG", LOG_M_REG_ENERGY | LOG_M_REG_ENERGY_DEBUG},
    //    {"RA_REG_DETAIL", LOG_M_REG_DETAIL},
    {"RA_REG", LOG_M_REG_ENERGY}};

bool isLog(LOG_MASK_T mask) { return mask == 0 || (params.logMask & mask); }

void LOG_OUTPUT(bool cond, ...) {
  va_list args;
  va_start(args, cond);
  char *fmt = va_arg(args, char *);
  if (cond) {
    vfprintf(params.logFile, fmt, args);
  }
  va_end(args);
}

void LOG_OUTPUT(LOG_MASK_T mask, ...) {
  va_list args;
  va_start(args, mask);
  char *fmt = va_arg(args, char *);
  if (isLog(mask)) {
    vfprintf(params.logFile, fmt, args);
  }
  va_end(args);
}

void flushLog() { fflush(params.logFile); }

char *trim(char *s, int length) {
  // sanity check, so that we don't do too much.
  int temp = strlen(s);
  if (temp < length)
    length = temp;
  else
    s[length - 1] = 0;        // set the last byte to zero, to have an ending.
  char *end = s + length - 1; // was 2
  while (*s && isspace(*s))   // remove the spaces from the beginning.
    *s++ = 0;
  while (end >= s && isspace(*end)) // remove the spaces from the end.
    *end-- = 0;
  return s;
}

void strcopy(char *to, char *from, int length) {
  int i = -1;
  if (length == -1) {
    do {
      i++;
      to[i] = from[i];
    } while (from[i] > 30 && from[i] < 128);
  } else {
    for (i = 0; i < length; i++)
      to[i] = from[i];
  }
}
