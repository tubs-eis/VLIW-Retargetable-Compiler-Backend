// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef HEADER_MEMORY_H_
#define HEADER_MEMORY_H_

#include "rapidxml-1.13/rapidxml.hpp"
#include <string.h>

#ifdef __APPLE__
typedef unsigned int uint;
#endif

namespace memory {

unsigned int getWritePorts(int numMemory);

unsigned int getReadPorts(int numMemory);

void initMemory(rapidxml::xml_node<> *start);

} // namespace memory

#endif /* HEADER_MEMORY_H_ */
