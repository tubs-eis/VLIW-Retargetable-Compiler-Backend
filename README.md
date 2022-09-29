



# VLIW Retargetable Compiler Backend

This repository implements a Very Long Instruction Word (VLIW) retargetable compiler backend. 
The processor and instruction description can be modified with XML description files to adapt different VLIW processors. 

The backend contains code optimization, instruction scheduling and register allocation.
Each step contains a heuristic implementation as well as a genetic algorithm implementation.
The genetic algorithms increase the code compaction and thus increase performance
but can consume many computing resources while doing so.
Below is a figure with an overview of the compiler backend.

![Compiler Overview](CompilerOverview.png)


The VLIW retargetable compiler backend is described in the following paper. If you use this project, please cite it as follows:

> Florian Giesemann, Lukas Gerlach, and Guillermo Payá-Vayá. “Evolutionary Algorithms for
> Instruction Scheduling, Operation Merging, and Register Allocation in VLIW Compilers”. In:
> Journal of Signal Processing Systems 92.7 (2020), pp. 655–678. issn: 1939-8115. doi: 10.1007/s11265-019-01493-2



## Table of Contents

[Getting started](#Getting-started)

- [Installation](#Installation)
- [Configuration](#Configuration)
- [Compilation](#Compilation)
- [Formatting](#Formatting)
- [Documentation](#Documentation)


[Contributors](#Contributors)

[License](#License)

[Citation](#Citation)

## Getting started

### Installation
Clone the repository
```bash
git clone https://github.com/tubs-eis/VLIW-Retargetable-Compiler-Backend.git
```


### Configuration
In `conf` an exemplary, reduced target architecture is described.

The processor configuration, such as issue slots and register file configuration is described in `processor-config.xml`.

The assembly is described in `base.xml`. 
Instructions can be grouped into functional units and the instruction encoding is provided.

An example program is provided in `assembly.asm`. 
In the assembly virtual register will be allocated in the register allocation process.
Physical register are used to transfer information across basic block borders.

The `mlweights.txt` file describes an alternative scheduling for instruction scheduling.
Each Micro Operation (MO) is assigned a weight and the MO with the highest weights are scheduled first.



### Compile Process

To compile the project, please run `compile_example.sh` .


### Formatting
To format the source code according to LLVM-Formatting, please run `format.sh` .

### Documentation
The documentation can be generated with `generateDocumentation.sh` .




## Contributors

- Guillermo Payá Vayá (Technische Universität Braunschweig)
- Fabian Stuckmann (Technische Universität Braunschweig)
- Florian Giesemann 
- Lukas Gerlach

## License

This open-source project is distributed under the MIT license.

Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
Technische Universitaet Braunschweig, Germany
www.tu-braunschweig.de/en/eis

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE file or at
https://opensource.org/licenses/MIT

## Citation
The VLIW retargetable Compiler backend tool is described in the following paper. If you use this project , please cite it as follows:

> Florian Giesemann, Lukas Gerlach, and Guillermo Payá-Vayá. “Evolutionary Algorithms for
> Instruction Scheduling, Operation Merging, and Register Allocation in VLIW Compilers”. In:
> Journal of Signal Processing Systems 92.7 (2020), pp. 655–678. issn: 1939-8115. doi: 10.1007/s11265-019-01493-2
