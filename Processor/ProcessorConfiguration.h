// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef MOAI_ASM_PROCESSORCONFIGURATION_H
#define MOAI_ASM_PROCESSORCONFIGURATION_H

#include "../header/processor.h"

class ProcessorConfiguration : public Processor {
public:
  static ProcessorConfiguration &getInstance() {
    static ProcessorConfiguration instance;
    return instance;
  }

private:
  ProcessorConfiguration() : Processor(params.config) {}

  ProcessorConfiguration(ProcessorConfiguration const &);
  void operator=(ProcessorConfiguration const &);
};

#endif // MOAI_ASM_PROCESSORCONFIGURATION_H
