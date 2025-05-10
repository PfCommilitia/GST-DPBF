#include <graph_hash_of_mixed_weighted/graph_hash_of_mixed_weighted.h>
#include <graph_hash_of_mixed_weighted/two_graphs_operations/graph_hash_of_mixed_weighted_to_graph_v_of_v_idealID.h>
#include <graph_hash_of_mixed_weighted_read_for_GSTP.h>

// Tree structure for DPBF algorithm
class Tree {
  public:
    int root;
    std::unordered_set<int> keywords;
    int cost;
    std::unordered_map<int, std::unordered_map<int, int>> edges;
    std::unordered_set<int> vertices;
    bool is_in_queue;

    Tree(int root_vertex, const std::unordered_set<int>& keywords_):
      root(root_vertex), keywords(keywords_), cost(0), is_in_queue(true) {
      vertices.insert(root_vertex);
    }

    Tree(const Tree& other):
      root(other.root),
      cost(other.cost),
      keywords(other.keywords),
      edges(other.edges),
      vertices(other.vertices),
      is_in_queue(true) {}

    void addEdge(int from, int to, int weight) {
      if (edges[from].find(to) != edges[from].end()) {
        return;
      }

      edges[from][to] = weight;
      edges[to][from] = weight;
      vertices.insert(from);
      vertices.insert(to);
      cost += weight;
    }

    void mergeTrees(const Tree& other) {
      if (other.root != root) {
        return;
      }

      keywords.insert(other.keywords.begin(), other.keywords.end());
      
      for (const auto& [vertex, neighbors]: other.edges) {
        for (const auto& [neighbor, weight]: neighbors) {
          addEdge(vertex, neighbor, weight);
        }
      }
    }
};

std::unordered_set<std::unordered_set<int>> generate_subsets(const std::unordered_set<int>& input_set) {
  if (input_set.empty()) {
    return {};
  }

  std::unordered_set<std::unordered_set<int>> result;
  const std::vector<int> elements(input_set.begin(), input_set.end());
  const int n = elements.size();

  for (int i = 1; i < (1 << n); i++) {
    std::unordered_set<int> subset;
    for (int j = 0; j < n; j++) {
      if (i & (1 << j)) {
        subset.insert(elements[j]);
      }
    }
    result.insert(std::move(subset));
  }

  return result;
}

std::unordered_set<std::unordered_set<int>> generate_disjoint_subsets(
  const std::unordered_set<int>& input_set,
  const std::unordered_set<int>& exclude_set
) {
  if (input_set.empty()) {
    return {};
  }

  if (exclude_set.empty()) {
    return generate_subsets(input_set);
  }

  std::vector<int> valid_elements;
  valid_elements.reserve(input_set.size());
  for (const int& elem: input_set) {
    if (exclude_set.find(elem) == exclude_set.end()) {
      valid_elements.push_back(elem);
    }
  }

  std::unordered_set<std::unordered_set<int>> result;
  const int n = valid_elements.size();

  for (int i = 1; i < (1 << n); i++) {
    std::unordered_set<int> subset;
    for (int j = 0; j < n; j++) {
      if (i & (1 << j)) {
        subset.insert(valid_elements[j]);
      }
    }
    result.insert(std::move(subset));
  }

  return result;
}

// 你需要在这里参考论文实现相应代码
int graph_v_of_v_idealID_DPBF_only_ec(
  graph_v_of_v_idealID& v_instance_graph,
  graph_v_of_v_idealID& v_generated_group_graph,
  std::unordered_set<int>& generated_group_vertices
) {
  // 0. Initialize T(v, p)
  std::unordered_map<std::pair<int, std::unordered_set<int>>, Tree> T;

  // 1. Initialize Qt (min heap)
  std::priority_queue<
    std::pair<int, Tree>,
    std::vector<std::pair<int, Tree>>, std::greater<std::pair<int, Tree>>
  > Qt;

  // 2. Generate reverse keywords
  std::vector<std::unordered_set<int>> reverse_keywords(v_instance_graph.size());

  for (const auto& index: generated_group_vertices) {
    for (const auto& edge: v_generated_group_graph[index]) {
      reverse_keywords[edge.first].insert(index);
    }
  }

  // 3. Enqueue primitive trees
  for (int i = 0; i < v_instance_graph.size(); i++) {
    for (const auto& keywords: generate_subsets(reverse_keywords[i])) {
      Tree tree(i, keywords);
      const auto key = std::make_pair(i, keywords);
      T.emplace(key, std::move(tree));
      Qt.emplace(0, T.at(key));
    }
  }

  // 4. Main loop
  while (!Qt.empty()) {
    const Tree tree = Qt.top().second;
    Qt.pop();

    if (!tree.is_in_queue) {
      continue;
    }

    if (tree.keywords == generated_group_vertices) {
      return tree.cost;
    }

    // 5. Expand tree
    for (const auto& edge: v_instance_graph[tree.root]) {
      Tree temp_tree(tree);
      temp_tree.addEdge(tree.root, edge.first, edge.second);

      const auto expand_key = std::make_pair(edge.first, tree.keywords);
      auto other_tree = T.find(expand_key);

      if (other_tree == T.end() || other_tree->second.cost > temp_tree.cost) {
        if (other_tree != T.end()) {
          other_tree->second.is_in_queue = false;
        }
        T[expand_key] = std::move(temp_tree);
        Qt.emplace(T[expand_key].cost, T[expand_key]);
      }
    }

    // 6. Merge trees
    for (const auto& keywords: generate_disjoint_subsets(generated_group_vertices, tree.keywords)) {
      Tree temp_tree(tree);
      const auto other_key = std::make_pair(tree.root, keywords);
      auto tree_2 = T.find(other_key);

      if (tree_2 == T.end()) {
        continue;
      }

      auto merged_subset = tree.keywords;
      merged_subset.insert(tree_2->second.keywords.begin(), tree_2->second.keywords.end());

      temp_tree.mergeTrees(tree_2->second);

      const auto merged_key = std::make_pair(tree.root, merged_subset);
      auto other_tree = T.find(merged_key);

      if (other_tree == T.end() || other_tree->second.cost > temp_tree.cost) {
        if (other_tree != T.end()) {
          other_tree->second.is_in_queue = false;
        }
        T[merged_key] = std::move(temp_tree);
        Qt.emplace(T[merged_key].cost, T[merged_key]);
      }
    }
  }

  return 0xffffffff;
}
