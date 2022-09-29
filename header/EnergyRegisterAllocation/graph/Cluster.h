// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef SCHEDULER_CLUSTER_H
#define SCHEDULER_CLUSTER_H

#include <vector>
namespace portOptReg {
class Cluster {
private:
  std::vector<Cluster> childCluster;
  std::vector<int> nodes;
  int layer = 0;

public:
  Cluster() {}

  void addNodes(std::vector<int> nodes) { this->nodes = nodes; }

  void addChildCluster(std::vector<Cluster> childCluster) {
    this->childCluster = childCluster;
  }

  const std::vector<Cluster> getChildCluster() const { return childCluster; }

  const std::vector<int> getNodes() const { return nodes; }

  bool hasChildCluster() const { return not childCluster.empty(); }

  int getMaxHierarchyLevel() const {
    if (childCluster.empty()) {
      return layer;
    } else {
      int maxDepth = layer;
      for (int i = 0; i < childCluster.size(); i++) {
        int temp = childCluster[i].getMaxHierarchyLevel();
        if (temp > maxDepth) {
          maxDepth = temp;
        }
      }
      return maxDepth;
    }
  }

  int getLayer() const { return layer; }

  void setDepth(int level) {
    this->layer = level;
    if (childCluster.size() > 0) {
      for (int i = 0; i < childCluster.size(); i++) {
        childCluster[i].setDepth(level + 1);
      }
    }
  }

  int getNodeCount() {
    int count = 0;
    for (int i = 0; i < childCluster.size(); i++) {
      count += childCluster[i].getNodeCount();
    }
    count += nodes.size();
    return count;
  }

  std::vector<int> getNodeList() {
    std::vector<int> temp;
    for (int i = 0; i < childCluster.size(); i++) {
      auto childNodes = childCluster[i].getNodeList();
      temp.insert(temp.begin(), childNodes.begin(), childNodes.end());
    }
    for (int i = 0; i < nodes.size(); i++) {
      temp.push_back(nodes[i]);
    }
    return temp;
  }
};
} // namespace portOptReg

#endif // SCHEDULER_CLUSTER_H
