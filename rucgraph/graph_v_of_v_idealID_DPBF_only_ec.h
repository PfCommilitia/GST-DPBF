#pragma once

#include <vector>
#include <queue>
#include <unordered_set>
#include <utility>

#include <graph_hash_of_mixed_weighted/graph_hash_of_mixed_weighted.h>
#include <graph_hash_of_mixed_weighted/two_graphs_operations/graph_hash_of_mixed_weighted_to_graph_v_of_v_idealID.h>
#include <graph_hash_of_mixed_weighted_read_for_GSTP.h>

struct Tree {
  int root;
  int keywordFlag;
  int cost;

  Tree(int r, int k, int c) : root(r), keywordFlag(k), cost(c) {}

  bool operator>(const Tree& other) const {
    if (cost != other.cost) {
      return cost > other.cost;
    }
    if (root != other.root) {
      return root > other.root;
    }
    return keywordFlag > other.keywordFlag;
  }
};

int graph_v_of_v_idealID_DPBF_only_ec(
  graph_v_of_v_idealID& rawAdjacencyList,
  graph_v_of_v_idealID& generatedAdjacencyList,
  std::unordered_set<int>& groupIndexes
) {
  // 1. Declare containers
  int numGroup = groupIndexes.size();
  int maxFlag = (1 << numGroup) - 1;

  std::vector<std::vector<int>> dp(
    rawAdjacencyList.size(),
    std::vector<int>(1 << numGroup, std::numeric_limits<int>::max())
  );

  std::priority_queue<Tree, std::vector<Tree>, std::greater<Tree>> Qt;

  // 2. Initialization
  int flag = 1;
  for (const auto& groupIndex: groupIndexes) {
    for (const auto& [vertex, _]: generatedAdjacencyList[groupIndex]) {
      Tree tree(vertex, flag, 0);
      dp[vertex][flag] = 0;
      Qt.push(std::move(tree));
    }
    flag <<= 1;
  }

  // 3. Main Loop
  while (!Qt.empty()) {
    Tree tree = Qt.top();

    if (tree.keywordFlag == maxFlag) {
      return tree.cost;
    }

    Qt.pop();

    if (tree.cost != dp[tree.root][tree.keywordFlag]) {
      continue;
    }

    // 3.1 Tree Grow
    for (const auto& [neighbor, weight]: rawAdjacencyList[tree.root]) {
      int newCost = tree.cost + weight;
      if (newCost < dp[neighbor][tree.keywordFlag]) {
        Tree newTree(neighbor, tree.keywordFlag, newCost);
        dp[neighbor][tree.keywordFlag] = newCost;
        Qt.push(std::move(newTree));
      }
    }

    // 3.2 Tree Merge
    for (int flag = 1; flag < maxFlag; flag++) {
      if ((flag & tree.keywordFlag) || dp[tree.root][flag] == std::numeric_limits<int>::max()) {
        continue;
      }

      int mergedFlag = flag | tree.keywordFlag;
      int mergedCost = dp[tree.root][flag] + tree.cost;

      if (mergedCost < dp[tree.root][mergedFlag]) {
        Tree newTree(tree.root, mergedFlag, mergedCost);
        dp[tree.root][mergedFlag] = mergedCost;
        Qt.push(std::move(newTree));
      }
    }
  }

  return -1;
}
