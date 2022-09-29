// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "RegisterLivelihood.h"
#include <fstream>

#include "EnergyGene.h"
#include "MI.h"

#include <algorithm>
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/sequential_vertex_coloring.hpp>
#include <iostream>
#include <utility>
#include <vector>

#include <boost/graph/smallest_last_ordering.hpp>
#include <boost/property_map/shared_array_property_map.hpp>

#include <boost/graph/bandwidth.hpp>
#include <boost/graph/cuthill_mckee_ordering.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/pending/disjoint_sets.hpp>

#include <EnergyRegisterAllocation/graph/Cluster.h>
#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/incremental_components.hpp>

int OFFSET = 10;

bool legacy = false;

int convertPhysical2Index(char32_t physicalRegister) {

  int collisionIndex = 0;
  collisionIndex -= (int)(physicalRegister);
  collisionIndex -= OFFSET;
  return collisionIndex;
}

char32_t convertIndex2Physical(int index) {
  index += OFFSET;
  return (char32_t)(0 - index);
}

namespace portOptReg {

RegisterLivelihood::RegisterLivelihood(const Program *ins,
                                       const VirtualRegisterMap *map,
                                       const rdg::RDG &rdg,
                                       const ass_reg_t *blocked,
                                       std::string graphLocation)
    : ins(ins), map(map), rdg(rdg), blocked(blocked),
      generationMethod(cuthillMckee), graphLocation(graphLocation) {

  createVirtualRegisters();

  processCouplings();

  createGraph();
}

void RegisterLivelihood::createVirtualRegisters() {
  //  cout << "Scheduled Instruction to Register Allocate:" << std::endl;
  //  ins->printInstructions(cout, nullptr);
  //  ins[0].printInstructions(ins, nullptr);
  //  cout << endl;
  for (auto &key : *map) {
    auto firstOccurence = key.second->getFirstOccurrence();
    //    cout << registers::getName(key.first) << "First Occurence " <<
    //    firstOccurence.operator*()->to_string() << endl;

    auto entryIterator =
        std::find(ins->begin(), ins->end(), firstOccurence.operator*());
    auto exitIterator = std::find(ins->begin(), ins->end(),
                                  key.second->getLastOccurrence().operator*());

    //    cout << "Start at " << std::distance(ins->begin(), entryIterator) <<
    //    "---- End at " << std::distance(ins->begin(), exitIterator)<<endl;
    virtualRegLivelihood.push_back(make_shared<VirtualRegs>(
        key.first, std::distance(ins->begin(), entryIterator),
        std::distance(ins->begin(), exitIterator),
        virtualRegLivelihood.size() + 1));
  }
}

std::vector<std::shared_ptr<VirtualRegisterMapping>>
RegisterLivelihood::addCoupledConstraints(
    std::vector<int> &allocationConstraintVector,
    std::unordered_map<int, shared_ptr<VirtualRegisterMapping>>
        &tempPhysicalRegister,
    std::vector<std::shared_ptr<VirtualRegisterMapping>> &virtualRegisters) {
  std::vector<std::shared_ptr<VirtualRegisterMapping>> constraints;
  for (auto allocationConstraint : allocationConstraintVector) {
    int regs = allocationConstraint;
    if (allocationConstraint < 0) {
      // physical Register handling
      char32_t physicalRegister = convertIndex2Physical(allocationConstraint);
      auto element = tempPhysicalRegister.find((int)physicalRegister);
      if (element == tempPhysicalRegister.end()) {
        tempPhysicalRegister.insert(
            {(int)physicalRegister,
             std::make_shared<VirtualRegisterMapping>(physicalRegister)});
        element = tempPhysicalRegister.find((int)physicalRegister);
      }
      constraints.push_back(element->second);
    } else {
      shared_ptr<VirtualRegisterMapping> v =
          virtualRegisters[allocationConstraint];
      constraints.push_back(virtualRegisters[allocationConstraint]);
    }
  }
  return constraints;
}

EnergyChromosome RegisterLivelihood::getNewChromosome() {
  return getNewRandomChromosome(nullptr);
}

EnergyChromosome RegisterLivelihood::getNewRandomChromosome(uint *seed) {
  std::unordered_map<int, VirtualRegisterMapping *> tempPhysicalRegister;

  std::vector<VirtualRegisterMapping *> physicalRegister;

  std::vector<std::vector<VirtualRegisterMapping *>>
      coupledReadAllocationConstraints;
  std::vector<std::vector<VirtualRegisterMapping *>>
      coupledWriteAllocationConstraints;

  std::vector<std::shared_ptr<VirtualRegisterMapping>> virtualRegisters;
  for (int i = 0; i < virtualRegLivelihood.size(); i++) {
    VirtualRegisterMapping newMapping;
    if (virtualRegLivelihood[i]->isPhysical()) {
      newMapping =
          VirtualRegisterMapping(virtualRegLivelihood[i]->getVirtualRegister());
    } else {
      newMapping =
          *getMapping(map, virtualRegLivelihood[i]->getVirtualRegister());
    }
    virtualRegisters.push_back(
        std::make_shared<VirtualRegisterMapping>(newMapping));
  }

  for (auto it : coupledIndexMap) {
    virtualRegisters[it.first]->addCoupledVirtualRegisterMapping(
        virtualRegisters[it.second.first].get(), it.second.second);
    virtualRegisters[it.second.first]->addCoupledVirtualRegisterMapping(
        virtualRegisters[it.first].get(), it.second.second);
  }

  std::vector<EnergyGene> energyGenes;
  for (auto &cluster : clusters) {
    energyGenes.push_back(createEnergyGene(cluster, virtualRegisters));
  }

  CHROMOSOME::Origin origin = CHROMOSOME::NONE;
  EnergyChromosome chromosome(energyGenes, virtualRegisters, origin, blocked);
  if (seed) {
    chromosome.randomize(seed, blocked);
    chromosome.setOrigin(CHROMOSOME::RANDOM);
    for (int i = 0; i < chromosome.getGenes().size(); i++) {
      chromosome.getGenes()[i].repairConflicts(seed, blocked);
    }
    int n = 3;
  }
  // check chromosome for correct handling of couplings
  chromosome.checkCouplings();

  return chromosome;
}

EnergyGene RegisterLivelihood::createEnergyGene(
    const Cluster &cluster,
    std::vector<std::shared_ptr<VirtualRegisterMapping>> &virtualRegisters) {
  if (cluster.hasChildCluster()) {
    EnergyGene gene;
    for (auto &childCluster : cluster.getChildCluster()) {
      gene.addChildGene(createEnergyGene(childCluster, virtualRegisters));
    }
    gene.addBlockedVirtualRegister(getBlockedRegisters(blocked));
    gene.setLayer(cluster.getLayer());
    gene.setMaxLayer(cluster.getMaxHierarchyLevel());
    return gene;
  } else {
    EnergyGene gene;

    for (auto nodeID : cluster.getNodes()) {
      auto virtualRegister = virtualRegisters[nodeID];
      std::vector<VirtualRegisterMapping *> blockedRegister;
      for (auto blockedID : blockedVirtualRegisterIndices[nodeID]) {
        blockedRegister.push_back(virtualRegisters[blockedID].get());
      }
      gene.addVirtualRegister(virtualRegister.get(), blockedRegister);
    }

    gene.addBlockedVirtualRegister(getBlockedRegisters(blocked));
    gene.setLayer(cluster.getLayer());
    gene.setMaxLayer(cluster.getMaxHierarchyLevel());
    return gene;
  }
}

void RegisterLivelihood::processCouplings() {
  // create indexes
  for (auto reglive : virtualRegLivelihood) {
    virtualRegister.push_back(reglive->getVirtualRegister());
  }

  for (auto mi : *ins) {
    unsigned int issueSlot = 0;
    MO **ops = mi->getOperations();
    bool foundCoupled = false;

    std::vector<int> coupledVirtIndices;

    while (issueSlot < MI::getNumberIssueSlots()) {
      MO *op = ops[issueSlot];
      if (!op) { // MO is not set
        ++issueSlot;
        continue;
      }

      // go through the operands and track couplings
      OPtype *types = op->getTypes();
      char32_t *args = op->getArguments();
      char *dirs = op->getDirections();
      for (int argIdx = 0; argIdx < op->getArgNumber(); ++argIdx) {
        char32_t argument = args[argIdx];
        if (types[argIdx] == REG &&
            (dirs[argIdx] == WRITE ||
             dirs[argIdx] == READWRITEPSEUDOREAD)) { // only target registers
                                                     // are interesting

          const rdg::RDG::Entry *e = rdg.getEntry(argument);
          if (e) {
            if (e->couple) {
              const RegisterCouple *couple = e->couple;
              auto firstIndex = std::find(virtualRegister.begin(),
                                          virtualRegister.end(), couple->first);
              // if physical register create virtualregistermapping for
              // coupled tracking
              if (firstIndex == virtualRegister.end()) {
                virtualRegLivelihood.push_back(
                    std::make_shared<VirtualRegs>(couple->first));
                virtualRegister.push_back(couple->first);
              }
              auto secondIndex =
                  std::find(virtualRegister.begin(), virtualRegister.end(),
                            couple->second);
              // if physical register create virtualregistermapping for
              // coupled tracking
              if (secondIndex == virtualRegister.end()) {
                virtualRegLivelihood.push_back(
                    std::make_shared<VirtualRegs>(couple->second));
                virtualRegister.push_back(couple->second);
              }

              int firstElement =
                  std::distance(virtualRegister.begin(), firstIndex);
              int secondElement =
                  std::distance(virtualRegister.begin(), secondIndex);
              coupledIndexMap.insert(
                  {firstElement,
                   std::make_pair(secondElement, couple->concurrent)});
              coupledIndexMap.insert(
                  {secondElement,
                   std::make_pair(firstElement, couple->concurrent)});

              //              std::cout << "found Coupled VirtualRegister: " <<
              //              firstElement << "="
              //                        <<
              //                        registers::getName(virtualRegLivelihood[firstElement]
              //                                                  ->getVirtualRegisters())
              //                        << " + " << secondElement << "=" <<
              //                        registers::getName(virtualRegLivelihood[secondElement]
              //                                                                                ->getVirtualRegisters()) << std::endl;

              // only add unique indices
              if (coupledVirtIndices.end() == find(coupledVirtIndices.begin(),
                                                   coupledVirtIndices.end(),
                                                   firstElement)) {
                coupledVirtIndices.push_back(firstElement);
              }
              if (coupledVirtIndices.end() == find(coupledVirtIndices.begin(),
                                                   coupledVirtIndices.end(),
                                                   secondElement)) {
                coupledVirtIndices.push_back(secondElement);
              }
              foundCoupled = true;
            }
          }
        }
      }

      issueSlot += op->opLengthMultiplier();
    }
  }
  int n = 3;
}

void RegisterLivelihood::createGraph() {
  if (generationMethod == smallestLastVertex) {

    // https://stackoverflow.com/a/16264286
    cout << endl << "smallest_last_vertex_ordering" << endl;
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS>
        Graph;
    typedef boost::graph_traits<Graph>::vertex_descriptor VertexDesc;

    vector<VertexDesc> vertices;
    for (int i = 0; i < virtualRegLivelihood.size(); i++) {
      vertices.push_back(add_vertex(g));
    }

    int max = num_vertices(g);
    for (int i = 0; i < max; i++) {
      for (int j = i + 1; j < max; j++) {
        if (virtualRegLivelihood[i]->overlap(virtualRegLivelihood[j])) {
          add_edge(vertices[i], vertices[j], g);
        }
      }
    }

    typedef std::map<std::size_t, VertexDesc> OrderMap;
    OrderMap order;
    boost::associative_property_map<OrderMap> order_prop_map(order);

    typedef std::map<VertexDesc, std::size_t> Map;
    Map degreeMap;
    Map marker;
    boost::associative_property_map<Map> degree_prop_map(degreeMap);
    boost::associative_property_map<Map> marker_prop_map(marker);

    smallest_last_vertex_ordering(g, order_prop_map, degree_prop_map,
                                  marker_prop_map);

    std::string names[virtualRegLivelihood.size()];
    for (int i = 0; i < virtualRegLivelihood.size(); i++) {
      names[i] =
          std::to_string(virtualRegLivelihood[i]->getTempVirtualRegister());
      nodeNames.push_back(
          registers::getName(virtualRegLivelihood[i]->getVirtualRegister()));
    }

    maxDegree = 0;
    for (std::size_t index = 0; index < max; ++index) {
      if (degreeMap[order[index]] > maxDegree) {
        maxDegree = degreeMap[order[index]];
      }
      degrees.push_back(order[index]);
    }
    std::ofstream myfile;
    myfile.open("/localtemp/stuckmann/tmp/Smallest_Last_Vertex.dot");
    myfile << "graph {" << endl;
    myfile << "compound=true" << endl;
    for (int j = 0; j <= maxDegree; j++) {
      myfile << "subgraph cluster" << j << " {" << endl;
      myfile << "color=red" << endl;
      for (std::size_t index = 0; index < max; ++index) {
        if (j == degreeMap[order[index]])
          myfile << names[order[index]] << "[label=\""
                 << nodeNames[order[index]] << "(" << degreeMap[order[index]]
                 << ") \"];" << endl;
      }
      myfile << endl << "}" << endl;
    }
    for (int i = 0; i < num_vertices(g); i++) {
      for (int j = i + 1; j < max; j++) {
        if (virtualRegLivelihood[i]->overlap(virtualRegLivelihood[j])) {
          myfile << i + 1 << " -- " << j + 1 << ";" << endl;
          //        myfile << i+1 << " -- " << j+1 << "[ltail=cluster"
          //        <<make_degree_map(g)[i] << ",lhead=cluster"<<
          //        make_degree_map(g)[j]<<"]"<< ";" << endl;
        }
      }
    }
    myfile << "}" << endl;
    myfile << endl;
    myfile.close();

    for (std::size_t index = 0; index < max; ++index) {
      std::cout << nodeNames[order[index]] << "(" << degreeMap[order[index]]
                << ") ";
    }
    std::cout << std::endl;
  } else if (generationMethod == cuthillMckee) {
    // reverse cuthill_mckee_ordering
    using namespace boost;

    typedef graph_traits<Graph>::vertex_descriptor Vertex;
    vector<Vertex> vertices;
    for (int i = 0; i < virtualRegLivelihood.size(); i++) {

      vertices.push_back(add_vertex(g));
      nodeNames.push_back(
          registers::getName(virtualRegLivelihood[i]->getVirtualRegister()));
    }

    int max = num_vertices(g);
    for (int i = 0; i < max; i++) {
      for (int j = i + 1; j < max; j++) {
        if (virtualRegLivelihood[i]->overlap(virtualRegLivelihood[j]) !=
            virtualRegLivelihood[j]->overlap(virtualRegLivelihood[i])) {
          bool iOverlap =
              virtualRegLivelihood[i]->overlap(virtualRegLivelihood[j]);
          bool jOverlap =
              virtualRegLivelihood[j]->overlap(virtualRegLivelihood[i]);
          stringstream ss;
          ss << "Overlap is calulated Wrong on "
             << registers::getName(
                    virtualRegLivelihood[i]->getVirtualRegister())
             << " and "
             << registers::getName(
                    virtualRegLivelihood[j]->getVirtualRegister())
             << "!\n";
          throw runtime_error(ss.str());
        }
        if (virtualRegLivelihood[i]->overlap(virtualRegLivelihood[j])) {
          add_edge(vertices[i], vertices[j], g);
        }
      }
    }

    // create single blockedVirtualRegisterIndex
    for (int i = 0; i < virtualRegLivelihood.size(); i++) {
      std::vector<int> blocked;
      for (int j = 0; j < virtualRegLivelihood.size(); j++) {
        if (i != j) {
          if (virtualRegLivelihood[i]->overlap(virtualRegLivelihood[j])) {

            //          // dont add coupled register to blocked
            //          if (virtualRegisters[i]->getCoupled() !=
            //          virtualRegisters[j].get())
            blocked.push_back(j);
          }
        }
      }
      blockedVirtualRegisterIndices.push_back(blocked);
    }

    typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef graph_traits<Graph>::vertices_size_type size_type;
    std::vector<Vertex> inv_perm(num_vertices(g));
    std::vector<size_type> perm(num_vertices(g));
    cuthill_mckee_ordering(g, inv_perm.rbegin(),
                           boost::get(boost::vertex_color, g),
                           make_degree_map(g));

    //    cout << "Reverse Cuthill-McKee ordering:" << endl;
    //    cout << "  ";

    if (!graphLocation.empty()) {
      std::ofstream myfile;
      myfile.open(graphLocation + "/graph.dot");
      myfile << "graph {" << endl;
      myfile << "compound=true" << endl;
      myfile.close();
    }
    property_map<Graph, vertex_index_t>::type index_map = get(vertex_index, g);
    for (std::vector<Vertex>::const_iterator i = inv_perm.begin();
         i != inv_perm.end(); ++i) {
      if (make_degree_map(g)[*i] > maxDegree) {
        maxDegree = make_degree_map(g)[*i];
      }
      degrees.push_back(make_degree_map(g)[*i]);
    }

    if (!graphLocation.empty()) {
      //      std::ofstream degreeFile;

      std::ofstream myfile;
      //      cout << endl << "Graph Creating Stage" << endl;
      myfile.open(graphLocation + "/graph.dot", ios::app);
      for (int DEGREE = 0; DEGREE <= maxDegree; DEGREE++) {
        myfile << "subgraph cluster" << DEGREE << " {" << endl;
        myfile << "color=red" << endl;

        //        degreeFile << "subgraph cluster" << DEGREE << " {" << endl;
        //        degreeFile << "color=red" << endl;

        for (int j = 0; j < virtualRegLivelihood.size(); j++) {
          if (DEGREE == degrees[j]) {
            myfile << (j + 1) << ";"
                   << " ";
            myfile << (j + 1) << "[ label=\"" << nodeNames[j] << " ("
                   << degrees[j] << ")\"];"
                   << " ";
          }
        }

        //        cout << endl;
        myfile << endl << "}" << endl;
        //        degreeFile << endl << "}" << endl;
      }

      for (int i = 0; i < num_vertices(g); i++) {
        for (int j = i + 1; j < max; j++) {
          if (virtualRegLivelihood[i]->overlap(virtualRegLivelihood[j])) {
            myfile << (i + 1) << " -- " << (j + 1) << ";" << endl;
            //            myfile << (i + 1) << " -- " << (j + 1) <<
            //            "[ltail=cluster"
            //                       << degrees[i] << ",lhead=cluster"
            //                       << degrees[j] << "]"
            //                       << ";" << endl;
          }
        }
      }
      myfile << "}" << endl;
      myfile << endl;
      myfile.close();
    }
  }

  clusters = std::vector<Cluster>();
  for (int rank = 0; rank <= maxDegree; rank++) {

    //    cout << "Checking out Rank: " << rank << endl;

    std::vector<int> nodeInRank;
    //    cout << "Same Rank ";
    for (int node = 0; node < degrees.size(); node++) {
      if (rank == degrees[node]) {
        //        cout << nodeNames[rank] << ",";
        nodeInRank.push_back(node);
        // node is a coupled register, add coupled register to same rank
        // coupled register should be in same cluster, so that combination does
        // not by default create a Conflict.
        if (coupledIndexMap.find(node) != coupledIndexMap.end()) {
          nodeInRank.push_back(coupledIndexMap[node].first);
          degrees[coupledIndexMap[node].first] = rank;
        }
      }
    }
    //    cout << endl;
    if (nodeInRank.size() > 0) {
      //      std::cout << "Creating new Cluster" << endl;
      // remove doublicate cluster
      sort(nodeInRank.begin(), nodeInRank.end());
      nodeInRank.erase(unique(nodeInRank.begin(), nodeInRank.end()),
                       nodeInRank.end());
      int clusterNodeCount = nodeInRank.size();

      auto cluster = createSubGraph(nodeInRank);
      cluster.setDepth(0);
      clusters.push_back(cluster);
      auto temp = cluster.getNodeList();
      sort(temp.begin(), temp.end());
      int nodeList = temp.size();
      if (cluster.getNodeCount() != nodeInRank.size()) {
        throw runtime_error("Error in creating Subcluster!");
      }
    }
  }
  int n = 3;
}

Cluster RegisterLivelihood::createSubGraph(std::vector<int> nodes) {
  if (generationMethod == cuthillMckee) {
    // reverse cuthill_mckee_ordering
    using namespace boost;
    Graph g;

    vector<basic_string<char>> nodeNames;

    //    std::cout << "Creating new Subgraph" << endl;
    typedef graph_traits<Graph>::vertex_descriptor Vertex;
    vector<Vertex> vertices;
    for (int node : nodes) {

      vertices.push_back(add_vertex(g));
      nodeNames.push_back(
          registers::getName(virtualRegLivelihood[node]->getVirtualRegister()));
      //      std::cout << "   Added node: "
      //                << registers::getName(
      //                       virtualRegLivelihood[node]->getVirtualRegister())
      //                << endl;
    }

    //    cout << "   Overlap Analysis" << endl;
    int max = num_vertices(g);
    for (int i = 0; i < max; i++) {
      for (int j = i + 1; j < max; j++) {
        if (virtualRegLivelihood[i]->overlap(virtualRegLivelihood[j])) {
          add_edge(vertices[i], vertices[j], g);
          //          cout << nodeNames[i] << " ---- " << nodeNames[j] << endl;
        }
      }
    }

    typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef graph_traits<Graph>::vertices_size_type size_type;
    std::vector<Vertex> inv_perm(num_vertices(g));
    std::vector<size_type> perm(num_vertices(g));
    cuthill_mckee_ordering(g, inv_perm.rbegin(),
                           boost::get(boost::vertex_color, g),
                           make_degree_map(g));

    int maxDegree = 0;
    std::vector<int> degrees;
    std::unordered_map<int, int> differentDegrees;
    property_map<Graph, vertex_index_t>::type index_map = get(vertex_index, g);
    for (std::vector<Vertex>::const_iterator i = inv_perm.begin();
         i != inv_perm.end(); ++i) {
      int degree = make_degree_map(g)[*i];

      degrees.push_back(degree);
    }

    // MAC may move Degrees, so process coupled Register and recalculate degrees
    for (int nodeIndex = 0; nodeIndex < degrees.size(); nodeIndex++) {
      // nodeIndex is a coupled register, add coupled register to same
      // rank coupled register should be in same cluster, so that
      // combination does not by default create a Conflict.
      int firstVirtualRegister = nodes[nodeIndex];
      // add coupled register
      if (coupledIndexMap.find(firstVirtualRegister) != coupledIndexMap.end()) {
        // set rank of coupled register to same rank
        auto coupledRegister =
            std::find(nodes.begin(), nodes.end(),
                      coupledIndexMap[firstVirtualRegister].first);
        if (coupledRegister != nodes.end()) {
          int index = std::distance(nodes.begin(), coupledRegister);
          if (nodes[index] != coupledIndexMap[firstVirtualRegister].first) {
            throw logic_error("Setting Rank of wrong Node!");
          }
          // set coupled Register to the same rank
          degrees[index] = degrees[nodeIndex];
        }
      }
    }

    // calculate differentDegrees and max degree
    for (int i = 0; i < degrees.size(); i++) {
      if (degrees[i] > maxDegree) {
        maxDegree = degrees[i];
      }
      differentDegrees.insert(std::pair<int, int>(degrees[i], 1));
    }

    int differentRanks = differentDegrees.size();

    // abort criteria
    if (differentRanks == 1) {
      Cluster cluster;
      cluster.addNodes(nodes);
      return cluster;
    }

    std::vector<Cluster> clusters;

    for (int rank = 0; rank <= maxDegree; rank++) {

      std::vector<int> clusterNodes;
      for (int nodeIndex = 0; nodeIndex < degrees.size(); nodeIndex++) {

        if (rank == degrees[nodeIndex]) {
          clusterNodes.push_back(nodes[nodeIndex]);
          // nodeIndex is a coupled register, add coupled register to same
          // rank coupled register should be in same cluster, so that
          // combination does not by default create a Conflict.
          int firstVirtualRegister = nodes[nodeIndex];
          // add coupled register
          if (coupledIndexMap.find(firstVirtualRegister) !=
              coupledIndexMap.end()) {
            clusterNodes.push_back(coupledIndexMap[firstVirtualRegister].first);

            // set rank of coupled register to same rank
            auto coupledRegister =
                std::find(nodes.begin(), nodes.end(),
                          coupledIndexMap[firstVirtualRegister].first);
            if (coupledRegister != nodes.end()) {
              int index = std::distance(nodes.begin(), coupledRegister);
              if (nodes[index] != coupledIndexMap[firstVirtualRegister].first) {
                throw logic_error("Setting Rank of wrong Node!");
              }
              // set coupled Register to the same rank
              degrees[index] = rank;
            } else {
              throw logic_error("Coupled Register was not found!");
            }
          }
        }
      }
      // processed all ranks
      if (clusterNodes.size() > 0) {
        // remove doublicate cluster
        sort(clusterNodes.begin(), clusterNodes.end());
        clusterNodes.erase(unique(clusterNodes.begin(), clusterNodes.end()),
                           clusterNodes.end());

        // MAC can change degrees of

        clusters.push_back(createSubGraph(clusterNodes));
      }
    }
    Cluster cluster;
    cluster.addChildCluster(clusters);
    return cluster;

  } else {
    throw logic_error("Not implemented different graph ranking algorithm.");
  }
  int n = 3;
}

std::string RegisterLivelihood::toString() {
  std::stringstream ss;
  ss << "Print Livelihood Graph" << std::endl;
  // print header
  ss << " ";
  for (int vxR = 0; vxR < virtualRegLivelihood.size(); vxR++) {
    if (vxR + 1 > 9) {
      ss << vxR + 1;
    } else {
      ss << vxR + 1 << " ";
    }
    ss << " ";
    //    ss << registers::getName(virtualRegLivelihood[vxR].virtualRegister)
    //    << " ";
  }
  ss << std::endl;
  for (int mi = 0; mi < ins->size(); mi++) {
    // iterate over VirtualRegister
    for (int vxR = 0; vxR < virtualRegLivelihood.size(); vxR++) {
      bool firstCondition =
          virtualRegLivelihood[vxR]->positionFirstCreated <= mi;
      bool secondCondition = mi <= virtualRegLivelihood[vxR]->positionLastUsed;
      if (virtualRegLivelihood[vxR]->positionFirstCreated <= mi and
          mi <= virtualRegLivelihood[vxR]->positionLastUsed) {
        ss << " x ";
      } else {
        ss << " - ";
      }
    }
    ss << std::endl;
  }
  return ss.str();
}

std::string RegisterLivelihood::graph2String() {
  std::stringstream ss;
  boost::write_graphviz(ss, g);
  return ss.str();
}

// vector<std::shared_ptr<graph::Node>> RegisterLivelihood::_getMinimalCut() {
//
//  // find minimal cut
//  auto iterator = tempGraph.begin();
//  int minCuts = iterator->first->getEdgeCount();
//  for (; iterator != tempGraph.end(); iterator++) {
//    int currentEdges = iterator->first->getEdgeCount();
//    if (currentEdges < minCuts) {
//      minCuts = currentEdges;
//    }
//  }
//
//  cout << "MinCuts = " << minCuts << endl;
//  // accululate minimal cuts
//  vector<shared_ptr<graph::Node>> temp;
//  for (auto iterator = tempGraph.begin(); iterator != tempGraph.end();
//       iterator++) {
//    if (iterator->first->getEdgeCount() == minCuts) {
//      temp.push_back(iterator->first);
//      cout << iterator->first->getNode()->toString() << ", ";
//    }
//  }
//  cout << std::endl;
//
//  // remove minimal cuts
//  for (auto node : temp) {
//    tempGraph.erase(node);
//  }
//
//  // update node values
//  for (auto node : temp) {
//    for (auto tempNode : tempGraph) {
//      tempNode.first->removeTempEdge(node->getNode());
//    }
//  }
//
//  return temp;
//}

// Graph RegisterLivelihood::getGraph() {
//  Graph g;
//  vector<VertexDesc> vertices;
//  for (int i = 0; i < virtualRegLivelihood.size(); i++) {
//    vertices.push_back(add_vertex(g));
//  }
//
//  int max = num_vertices(g);
//  for (int i = 0; i < max; i++) {
//    for (int j = i + 1; j < max; j++) {
//      if (virtualRegLivelihood[i]->overlap(virtualRegLivelihood[j])) {
//        add_edge(vertices[i], vertices[j], g);
//      }
//    }
//  }
//  return g;
//}

} // namespace portOptReg
