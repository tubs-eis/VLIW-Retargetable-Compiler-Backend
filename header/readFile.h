// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#ifndef READFILE_H
#define READFILE_H 1

#include <string>
#include <vector>

class Processor;
class SLM;

/** formats a given ASM-File to get rid of Comments and unnessary spaces. */
void commentFreeFormat(char *inFile, char *outFile);
/** reads in an ASM-File and parses it
it has to be comment-free.
*/
void readFile(char *file, std::vector<SLM *> *slms);
std::vector<SLM *> convert(std::string ASM, Processor *pro);
std::string commentFree(std::string original);
void addRegEx(char *before, char *after);
void regExReplacement(char *inFile, char *outFile);
std::string regExReplacement(std::string before);

#endif
