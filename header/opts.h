// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#ifndef OPTS_H
#define OPTS_H 1

/** prints out a help-message on how tu use this program. */
void printHelpOpts(char *name);
/** parses the operads given in the command line into the struct. */
void opts(int argc, char **argv);

/***
 *
 * @return Should power optimization be done?
 */
bool isPowerOptimization();

#endif
