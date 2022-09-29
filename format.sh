#!/bin/bash
# Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
#                    Technische Universitaet Braunschweig, Germany
#                    www.tu-braunschweig.de/en/eis
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT

find -iname *.h -o -iname *.cpp -o -iname *.c -o -iname *.hpp | xargs clang-format --assume-filename=.clang-format -i
