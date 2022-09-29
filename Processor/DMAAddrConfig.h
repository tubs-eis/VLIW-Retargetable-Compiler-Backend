// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef MOAI_ASM_DMAADDRCONFIG_H
#define MOAI_ASM_DMAADDRCONFIG_H

#include "stdint.h"

struct DMAAddrConfig {
  uint32_t startAddr;
  uint32_t endAddr;
  uint32_t CTRL_START_ADDR;
  uint32_t RESET_ADDR;
};

#endif // MOAI_ASM_DMAADDRCONFIG_H
