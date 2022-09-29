// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_REGISTERLIVELIHOOD_H
#define SCHEDULER_REGISTERLIVELIHOOD_H

#include "EnergyRegisterAllocation/VirtualReg.h"
#include "Program.h"
#include "virtual_reg.h"

#include <EnergyRegisterAllocation/graph/Cluster.h>
#include <algorithm>
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <iostream>
#include <memory>
#include <sstream>

#include "EnergyChromosome.h"
#include "EnergyGene.h"

enum GraphGeneration { cuthillMckee, smallestLastVertex };

typedef boost::adjacency_list<
    boost::vecS, boost::vecS, boost::undirectedS,
    boost::property<boost::vertex_color_t, boost::default_color_type,
                    boost::property<boost::vertex_degree_t, int>>>
    Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor VertexDesc;

namespace portOptReg {
class RegisterLivelihood {

private:
  std::vector<std::shared_ptr<VirtualRegs>> virtualRegLivelihood;

  /***
   * Keeps track of the overlapping virtualRegs.
   * 1st Vector maps to maps to virtualRegLivelihood.
   * 2nd Vector are blocked Virtual Register.
   */
  std::vector<std::vector<int>> blockedVirtualRegisterIndices;

  /**
   * first Element index in VirtualRegLivelihood
   * second element index in VirtualRegLivelihood also concurrent boolean
   */
  std::unordered_map<int, std::pair<int, bool>> coupledIndexMap;
  //  /***
  //   * Collection of all the Virtual register, that read together
  //   * First Index has same index as coupled index map
  //   * Second Vector are all the register collisions!
  //   * - is a physical register -(physical Register) - offset [offset for V0R0
  //   * physical register]
  //   * + index of virtualRegLivelihood (virtual Register)
  //   */
  //  std::vector<std::vector<int>> coupledVirtReads;
  //  /***
  //   * Collection of all the Virtual register that write together
  //   * First Index has same index as coupled index map
  //   * Second Vector are all the register collisions! \n
  //   * - is a physical register -(physical Register) - offset [offset for V0R0
  //   * physical register] \n
  //   * + index of virtualRegLivelihood (virtual Register)
  //   */
  //  std::vector<std::vector<int>> coupledVirtWrites;
  //
  //  /***
  //   * Keeps track of which virtual register creates the coupling incident.
  //   * 1. Vector: Index of 1st is equal to the Indexing of CoupledVirtReads
  //   and
  //   * CoupledVirtWrites
  //   * 2. Vector: Virtual Register Indices causing the coupling incident.
  //   */
  //  std::vector<std::vector<int>> coupledIndices;

  /***
   * internal data structure to find indices of virtualRegister faster.
   */
  std::vector<char32_t> virtualRegister;

  //  /***
  //   * Maps
  //   */
  //  std::unordered_map<int , int> hierachyRelationship;

  Graph g;
  //  std::unordered_map<std::shared_ptr<graph::Node>, int> graph;
  //  std::unordered_map<std::shared_ptr<graph::Node>, int> tempGraph;

  GraphGeneration generationMethod;
  int maxDegree = 0;
  const Program *ins;
  const VirtualRegisterMap *map;
  const ass_reg_t *blocked;
  const rdg::RDG &rdg;

  std::vector<Cluster> clusters;

  //  std::vector<std::shared_ptr<graph::Node>> _getMinimalCut();

  //  Graph getGraph();

  std::vector<std::string> nodeNames;
  std::vector<int> degrees;
  std::vector<int> coupledVirtualRegister;

  std::string graphLocation;

  //  std::shared_ptr<graph::Node> _foundDependencyInUsedNodes(
  //      const std::vector<std::shared_ptr<graph::Node>> usedNodes,
  //      const graph::Node node) const;

  std::vector<std::shared_ptr<VirtualRegisterMapping>> addCoupledConstraints(
      std::vector<int> &allocationConstraintVector,
      std::unordered_map<int, std::shared_ptr<VirtualRegisterMapping>>
          &tempPhysicalRegister,
      std::vector<std::shared_ptr<VirtualRegisterMapping>> &virtualRegisters);

  void createVirtualRegisters();
  void processCouplings();
  void createGraph();

  EnergyGene createEnergyGene(
      const Cluster &cluster,
      std::vector<std::shared_ptr<VirtualRegisterMapping>> &virtualRegisters);

public:
  RegisterLivelihood(const Program *ins, const VirtualRegisterMap *map,
                     const rdg::RDG &rdg, const ass_reg_t *blocked,
                     std::string graphLocation = "/home/stuckmann/graph.dot");
  // RegisterLivelihood();

  ~RegisterLivelihood() {}

  /***
   *
   * @param nodes Indices of VirtualRegisters
   * @return
   */
  Cluster createSubGraph(std::vector<int> nodes);

  std::string toString();
  std::string graph2String();

  /**
   * Generate Chromosome without random initialization.
   * @return
   */
  EnergyChromosome getNewChromosome();

  /**
   * Generate Random Chromosome.
   * @param seed used to randomize physical Values of EnergyGenes.
   * @return
   */
  EnergyChromosome getNewRandomChromosome(uint *seed);

  const uint getSize() const { return virtualRegLivelihood.size(); }

  const uint getMaxCycles() const { return ins->size(); }

  const VirtualRegs &getLivelihood(int index) const {
    return virtualRegLivelihood[index];
  }

  const ass_reg_t *getBlocked() const { return blocked; }
  //  const bool isAlive(int index, int cycle) const {
  //    return virtualRegLivelihood[index].isAlive(cycle);
  //  }

  const int getVirtualRegister() const { return virtualRegLivelihood.size(); }
};

} // namespace portOptReg
#endif // SCHEDULER_REGISTERLIVELIHOOD_H
