// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "global.h"
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdlib.h>
#ifndef GCCREGEX
#include <boost/regex.hpp>
#else
#include <regex>
#endif
#include "SLM.h"
#include "label.h"
#include "operation.h"
#include "processor.h"
#include "readFile.h"
#include "register.h"

#define BOOSTERNAME "--boost"

#define MINDISTANCEX2MERGINGNAME "--min_distance_x2_merging"
#define MAXDISTANCEX2MERGINGNAME "--max_distance_x2_merging"
#define SLM_OPT_LEVELNAME "--slm_opt_level"
#define SLM_MERGE_LEVELNAME "--slm_merge_level"
#define RA_PRUNE_COUNTNAME "--ra_prune_count"
#define PART_PROBNAME "--part_prob"
#define ALONE_PROBNAME "--alone_prob"

#define BUFFER_SIZE 512

using namespace std;

struct regex_replacement {
  string pattern;
  string replacement;
#ifdef __APPLE__
  boost::regex regex;
  regex_replacement(const char *before, const char *after)
      : pattern(before), replacement(after), regex(before) {}

#else
#ifndef GCCREGEX
  boost::regex regex regex_replacement(const char *before, const char *after)
      : pattern(before), replacement(after) {
    regex.assign(before);
  }
#else
  std::regex regex;
  regex_replacement(const char *before, const char *after)
      : pattern(before), replacement(after), regex(before) {}
#endif
#endif
};
#ifdef __APPLE__
vector<regex_replacement> replacements;
#else
#ifndef GCCREGEX
map<boost::regex, string> replacements;
#else
vector<regex_replacement> replacements;
#endif
#endif

std::vector<SLM *> convert(string ASM, Processor *pro) {
  istringstream lab(ASM);
  std::vector<SLM *> slms;
  char line[BUFFER_SIZE];
  char *tmp;
  while (lab.getline(line, BUFFER_SIZE)) {
    tmp = trim(line, BUFFER_SIZE);
    if (tmp[0] == ':' && tmp[1] == 'L') {
      // first, lets just index all the labels.
      label::addLabel(tmp);
    }
  };

  istringstream input(ASM);
  bool schedule = params.optimization != 0;
  SLM *actualSLM = new SLM();
  MO *tmpMO = NULL;
  int i = 0;
  while (input.getline(line, BUFFER_SIZE)) {
    i++;
    tmp = trim(line, BUFFER_SIZE);
    LOG_OUTPUT(LOG_M_PARSE, "Parsing: %s\n", tmp);
    if (tmp[0] == 0) // skip empty lines.
      continue;
    if (!strncmp(tmp, BOOSTERNAME, strlen(BOOSTERNAME))) {
      float b = atof(tmp + strlen(BOOSTERNAME) + 1);
      actualSLM->setBooster(b);
      continue;
    }
    if (!strncmp(tmp, MINDISTANCEX2MERGINGNAME,
                 strlen(MINDISTANCEX2MERGINGNAME))) {
      int min_distance_x2_merging =
          atoi(tmp + strlen(MINDISTANCEX2MERGINGNAME) + 1);
      actualSLM->setMinDistanceX2Merging(min_distance_x2_merging);
      continue;
    }
    if (!strncmp(tmp, MAXDISTANCEX2MERGINGNAME,
                 strlen(MAXDISTANCEX2MERGINGNAME))) {
      int max_distance_x2_merging =
          atoi(tmp + strlen(MAXDISTANCEX2MERGINGNAME) + 1);
      actualSLM->setMaxDistanceX2Merging(max_distance_x2_merging);
      continue;
    }
    if (!strncmp(tmp, SLM_OPT_LEVELNAME, strlen(SLM_OPT_LEVELNAME))) {
      int opt_level = atoi(tmp + strlen(SLM_OPT_LEVELNAME) + 1);
      actualSLM->setOptimizationLevel(opt_level);
      continue;
    }
    if (!strncmp(tmp, SLM_MERGE_LEVELNAME, strlen(SLM_MERGE_LEVELNAME))) {
      int merge_level = atoi(tmp + strlen(SLM_MERGE_LEVELNAME) + 1);
      actualSLM->setMergeLevel(merge_level);
      continue;
    }
    if (!strncmp(tmp, PART_PROBNAME, strlen(PART_PROBNAME))) {
      float prob = atof(tmp + strlen(PART_PROBNAME) + 1);
      actualSLM->setPartProb(prob);
      continue;
    }
    if (!strncmp(tmp, ALONE_PROBNAME, strlen(ALONE_PROBNAME))) {
      float prob = atof(tmp + strlen(ALONE_PROBNAME) + 1);
      actualSLM->setAloneProb(prob);
      continue;
    }
    if (!strncmp(tmp, RA_PRUNE_COUNTNAME, strlen(RA_PRUNE_COUNTNAME))) {
      int count = atoi(tmp + strlen(RA_PRUNE_COUNTNAME) + 1);
      actualSLM->setRAPruneCount(count);
      continue;
    }
    if (!strcmp(tmp, "--scheduling-off")) {
      slms.push_back(actualSLM);
      actualSLM = new SLM();
      actualSLM->setOptimizationLevel(0);
      actualSLM->setMergeLevel(0);
      schedule = false;
      continue;
    }
    if (!strcmp(tmp, "--scheduling-on")) {
      slms.push_back(actualSLM);
      actualSLM = new SLM();
      if (params.optimization != 0)
        schedule = true;
      if (tmpMO != NULL)
        tmpMO->setParting(true);
      continue;
    }
    if (!strncmp(tmp, "FREEREG", 7)) {
      char32_t ID;
      int y = 7;
      while (isspace(tmp[y])) {
        y++;
      }
      registers::parseRegister(tmp + y, &ID);
      // The idea is good, but if a register is read during a loop
      // and then changes their content in the next loop, this is not wanted.
      //			if (actualSLM->getOperations()->size() > 0) {
      actualSLM->addFreeReg(ID);
      //			} else {
      //				slms.back()->addFreeReg(ID);
      //			}
      continue;
    }
    if (!strncmp(tmp, "FIXREG", 6)) {
      char32_t ID;
      int y = 6;
      while (isspace(tmp[y]))
        y++;
      registers::parseRegister(tmp + y, &ID);
      if (registers::isPhysicalRegister(ID))
        actualSLM->addFixReg(ID);
      else {
        cerr << "Only physical registers can be FIXED in line " << i << endl;
        cerr << " -> " << tmp << endl;
        EXIT_ERROR
      }
      continue;
    }
    if (tmp[0] == ':' && tmp[1] == 'L') {
      // but if there is a Label, there needs to be a new SLM.
      if (actualSLM->getOperations()->size() > 0) {
        slms.push_back(actualSLM);
        actualSLM = new SLM();
      }
      try {
        char32_t LabelID;
        label::parseLabel(tmp, &LabelID);
        actualSLM->setLabel(LabelID);
      } catch (std::runtime_error &e) {
        cerr << e.what() << endl;
        cerr << "Could not register label " << tmp << " in line " << i << endl;
        cerr << "Be aware that the L_NOT_SCHEDULE comparison is limited to the "
                "substring 'NOT_SCHEDULE'."
             << endl;
        EXIT_ERROR
      }
      continue;
    }
    tmpMO =
        checkline(tmp, pro); // check whether it is a valid operation or not.
    if (tmpMO != NULL) {     // checkline is defined in operation.h
      tmpMO->setLineNumber(i);
      tmpMO->setReorderable(schedule);
      actualSLM->addMO(tmpMO);
      // Check if the processor can execute the MO.
      MI *m = new MI();
      m->putOperation(tmpMO, 0);
      if (!pro->isExecuteable(m)) {
        cerr << "Could not execute line " << i << endl;
        tmpMO->writeOutReadable(cerr);
        delete m;
        EXIT_ERROR
      }
      delete m;
      if (tmpMO->isBranchOperation()) {
        slms.push_back(actualSLM);
        actualSLM = new SLM();
      }
      // Check whether a register is written, which was freed (FREEREG) before
      // (ignore previous FREEREG if this is the case)
      char32_t *args_depend = tmpMO->getArguments();
      OPtype *types_depend = tmpMO->getTypes();
      char *dir_depend = tmpMO->getDirections();
      for (int e = 0, end = tmpMO->getArgNumber(); e < end; ++e) {
        if ((dir_depend[e] & WRITE) && types_depend[e] == REG &&
            !registers::isVirtualReg(args_depend[e])) {
          if (actualSLM->getFreeReg()->find(args_depend[e]) !=
              actualSLM->getFreeReg()->end()) {
            actualSLM->getFreeReg()->erase(args_depend[e]);
            LOG_OUTPUT(LOG_M_ALWAYS,
                       "WARNING: FREEREG for register %s is ignored!\n",
                       registers::getName(args_depend[e]).c_str());
          }
        }
      }
      continue;
    } // */
    cerr << "[line " << i << "] could not parse: ->" << tmp << "<-" << endl;
    EXIT_ERROR
  };
  if (actualSLM->getOperations()->size() > 0)
    slms.push_back(actualSLM);
  else
    delete actualSLM;
  return slms;
}

void addRegEx(char *before, char *after) {
  replacements.push_back(regex_replacement(before, after));
}

string regExReplacement(string ASM) {
  ostringstream os;
  char line[BUFFER_SIZE];
  string s;
  istringstream input(ASM);

  try {
#ifndef __APPLE__
#ifndef GCCREGEX
    for (std::map<boost::regex, string>::iterator it = replacements.begin();
         it != replacements.end(); ++it) {
      ASS_NOTE_COUT("'" << it->first << "' -> '" << it->second << "'" << endl);
    }
#endif
#endif
    while (input.getline(line, BUFFER_SIZE)) {
      s = string(line);
#ifdef __APPLE__
      for (std::vector<regex_replacement>::iterator it = replacements.begin();
           it != replacements.end(); ++it) {
#if REGEX_REPLACE_VERBOSE
        cout << "Attempting regex_replace:" << endl;
        cout << "  regex:        '" << it->pattern << "'" << endl;
        cout << "  replacement:  '" << it->replacement << "'" << endl;
        cout << "  input string: '" << s << "'" << endl;
#endif
        s = boost::regex_replace(s, it->regex, it->replacement);
#if REGEX_REPLACE_VERBOSE
        cout << "  replaced:     '" << s << "'" << endl;
#endif

#else
#ifndef GCCREGEX
      for (std::map<boost::regex, string>::iterator it = replacements.begin();
           it != replacements.end(); ++it) {
        std::ostringstream t(std::ios::out | std::ios::binary);
        std::ostream_iterator<char, char> oi(t);
        boost::regex_replace(oi, s.begin(), s.end(), it->first, it->second,
                             boost::match_default | boost::format_all);
        s = t.str();
#else
      for (std::vector<regex_replacement>::iterator it = replacements.begin();
           it != replacements.end(); ++it) {
#if REGEX_REPLACE_VERBOSE
        cout << "Attempting regex_replace:" << endl;
        cout << "  regex:        '" << it->pattern << "'" << endl;
        cout << "  replacement:  '" << it->replacement << "'" << endl;
        cout << "  input string: '" << s << "'" << endl;
#endif
        s = std::regex_replace(s, it->regex, it->replacement);
#if REGEX_REPLACE_VERBOSE
        cout << "  replaced:     '" << s << "'" << endl;
#endif
#endif
#endif
      }
      os << s << std::endl;
    }
  } catch (...) {
    ;
  };
  return os.str();
}

string commentFree(string original) {
  stringstream ss;
  const char *input = original.c_str();
  bool single = false, multi = false;

  for (uint i = 0; i < original.size(); i++) {
    if (!single && input[i] == '/' && input[i + 1] == '*') {
      i++;
      multi = true;
      continue;
    }
    if (!multi && input[i] == '/' && input[i + 1] == '/') {
      i++;
      single = true;
      continue;
    }
    if (multi && input[i] == '*' && input[i + 1] == '/') {
      multi = false;
      i++;
      continue;
    }
    if (single && (input[i] == 10 || input[i] == 13)) {
      single = false;
    }
    if (!single && !multi)
      ss.put(input[i]);
    else if (input[i] == 10 || input[i] == 13)
      ss.put(input[i]);
  }
  return ss.str();
}
