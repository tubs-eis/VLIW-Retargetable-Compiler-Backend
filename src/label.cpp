// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "label.h"
#include "global.h"
#include <iostream>
#include <map>
#include <string.h>

using namespace std;

namespace label {
/** \brief Mapping from Name to ID. */
map<std::string, char32_t> labels;
/** \brief Mapping from ID to SLM */
map<char32_t, SLM *> slms;
/* A mapping from Name to SLM can be done by using the features above.
 * Mapping from SLM to Name
 */

void registerSLM(char32_t labelID, SLM *slm) {
  map<char32_t, SLM *>::iterator it = slms.find(labelID);
  if (it != slms.end() && it->second == slm)
    return;
  if (it != slms.end()) {
    // not_schedule cuts off everything after the label, so we can't register
    throw std::runtime_error("You can't register a label twice!");
  }
  slms.insert(std::make_pair(labelID, slm));
}

SLM *getSLM(char32_t labelID) {
  map<char32_t, SLM *>::iterator it = slms.find(labelID);
  if (it == slms.end()) {
    LOG_OUTPUT(LOG_M_ALWAYS, "no SLM found for the label ID: %d\n", labelID);
    LOG_OUTPUT(LOG_M_ALWAYS, "labels ID's available:\n");

    for (map<char32_t, SLM *>::iterator it = slms.begin(); it != slms.end();
         ++it)
      LOG_OUTPUT(LOG_M_ALWAYS, "available: %u\n", it->first);
    EXIT_ERROR
    return NULL;
  }
  return (it->second);
}

void printAllLabels() {
  for (map<std::string, char32_t>::iterator it = labels.begin();
       it != labels.end(); ++it)
    LOG_OUTPUT(LOG_M_ALWAYS, "LabelName: %s\n", it->first.c_str());
  return;
}

void addLabel(const char *labelname) {
  std::string tmp = labelname;
  tmp = tmp.substr(3);
  for (unsigned int i = 0; i < tmp.length(); i++) {
    if (isspace(tmp[i]))
      tmp = tmp.substr(0, i);
  }
  if (tmp == "NOT_SCHEDULE") { // we want to ignore this one, because we never
                               // jump to it.
    return;
  }

  // find won't work, since we have different addresses of the char*, but the
  // same values.
  if (labels.find(tmp) != labels.end()) {
    LOG_OUTPUT(LOG_M_ALWAYS, "Label %s already existed.\n", labelname);
    EXIT_ERROR
  }
  labels.insert(std::make_pair(tmp, labels.size()));
  LOG_OUTPUT(LOG_M_PARSE, "Adding Label: %s\n", tmp.c_str());
}

char *parseLabel(char *labelname, char32_t *ID) {
  int offset = 0;
  while (isspace(labelname[offset]) || labelname[offset] == ',')
    offset++;
  if (labelname[offset] == ':')
    offset += 3;

  if (!strncmp(
          labelname + offset, "NOT_SCHEDULE",
          12)) { // we want to ignore this one, because we never jump to it.
    ID = NULL;
    return labelname + offset + 12;
  }

  char tmp[64];
  int i;
  for (i = 0; i < 64; i++) {
    tmp[i] = labelname[i + offset];
    if (isspace(tmp[i]))
      tmp[i] = 0;
    if (tmp[i] == ',')
      tmp[i] = 0;
    if (tmp[i] == 0)
      break;
  }
  auto it = labels.find(std::string(tmp));
  if (it != labels.end()) {
    *ID = it->second;
    return labelname + i + offset;
  }
  return (char *)-1;
}

std::string getLabelName(char32_t ID) {
  char32_t noschedule = -1;
  if (ID == noschedule)
    return (char *)"NOT_SCHEDULE";
  for (map<std::string, char32_t>::iterator it = labels.begin();
       it != labels.end(); ++it) {
    if (it->second == ID)
      return it->first;
  }
  return "";
}
} // namespace label
