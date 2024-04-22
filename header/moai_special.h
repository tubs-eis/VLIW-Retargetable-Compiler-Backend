// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef MOAI_SPECIAL_H
#define MOAI_SPECIAL_H

#include "SLM.h"
#include "processor.h"

void shrinkImmediates(SLM *slm, Processor *pro);

/***
 * X2 checks register usage as well as sets readwrite/write flags for
 * operaitons. Also sets CSR flags, and X2 optionality
 * @param slm
 * @param pro
 */
void specialInit(SLM *slm, Processor *pro);
void writeOutOps(SLM *slm, Processor *pro);
void MVOptimizer(SLM *slm, Processor *pro);

void replaceHardWithVirtual(vector<SLM *> *slms, Processor *pro);
#endif // MOAI_SPECIAL_H
