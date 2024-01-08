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

build/build/eis_osc -a conf/assembly.asm  -c conf/processor-config.xml


build/build/eis_osc -a conf/assembly.asm  -c conf/processor-config.xml  --StatsFile conf/stats.log --MLSchedulingWeights conf/mlweights.txt

#genetic scheduling enabled
# build/build/eis_osc -a conf/assembly.asm  -c conf/processor-config.xml  --StatsFile conf/stats.log --MLSchedulingWeights conf/mlweights.txt -o 2

#operation merging 
# build/build/eis_osc -a conf/assembly.asm  -c conf/processor-config.xml  --StatsFile conf/stats.log --MLSchedulingWeights conf/mlweights.txt -x 2

