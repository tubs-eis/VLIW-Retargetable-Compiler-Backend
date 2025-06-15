#!/bin/bash
# Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
#                    Technische Universitaet Braunschweig, Germany
#                    www.tu-braunschweig.de/en/eis
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT

mkdir build
cd build/
cmake ..
make -j 4

cd ..

# example assembly compilation
build/build/eis_osc -a conf/assembly.asm -c conf/processor-config.xml

#genetic scheduling enabled
# o 1 is list scheduling, o > 1 is genetic scheduling (better results)
build/build/eis_osc -a conf/assembly.asm -c conf/processor-config.xml -o 2

#operation merging (feature should be in cluded in the processor configuration (conf/processor-config.xml))
build/build/eis_osc -a conf/assembly.asm -c conf/processor-config.xml -x 2
