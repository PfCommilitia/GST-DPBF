#include <graph_hash_of_mixed_weighted/graph_hash_of_mixed_weighted.h>
#include <graph_hash_of_mixed_weighted/two_graphs_operations/graph_hash_of_mixed_weighted_to_graph_v_of_v_idealID.h>
#include <graph_hash_of_mixed_weighted_read_for_GSTP.h>

struct UnorderedSetHash {
  std::size_t operator()(const std::unordered_set<int>& set) const {
    std::size_t hash = 0;
    for (const auto& elem : set) {
      hash ^= std::hash<int>{}(elem) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
  }
};

class Tree {
  public:
    int root;
    std::unordered_set<int> keywords;
    int cost;
    std::unordered_map<int, std::unordered_map<int, int>> edges;
    std::unordered_set<int> vertices;
    bool is_in_queue;

    Tree() : root(-1), cost(0), is_in_queue(true) {}

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

    Tree clone() {
      Tree result(root, keywords);
      result.edges = edges;
      result.vertices = vertices;
      result.cost = cost;
      return result;
    }

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

    Tree mergeTrees(const Tree& other) {
      Tree result = clone();

      result.keywords.insert(other.keywords.begin(), other.keywords.end());

      for (const auto& [vertex, neighbors]: other.edges) {
        for (const auto& [neighbor, weight]: neighbors) {
          result.addEdge(vertex, neighbor, weight);
        }
      }

      return result;
    }

    Tree mergeEdge(int from, int to, int weight) {
      Tree result = clone();
      result.addEdge(from, to, weight);
      result.root = to;
      return result;
    }

    bool operator<(const Tree& other) const {
      return cost < other.cost;
    }
};

std::unordered_set<std::unordered_set<int>, UnorderedSetHash> generate_subsets(const std::unordered_set<int>& input_set) {
  if (input_set.empty()) {
    return {};
  }

  std::unordered_set<std::unordered_set<int>, UnorderedSetHash> result;
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

int graph_v_of_v_idealID_DPBF_only_ec(
  graph_v_of_v_idealID& v_instance_graph,
  graph_v_of_v_idealID& v_generated_group_graph,
  std::unordered_set<int>& generated_group_vertices
) {
  // 1. Initialize T(v, p) And Qt
  std::unordered_map<int, std::unordered_map<std::unordered_set<int>, Tree, UnorderedSetHash>> T;

  auto compare = [](const Tree& a, const Tree& b) { return a.cost > b.cost; };
  std::priority_queue<Tree, std::vector<Tree>, decltype(compare)> Qt(compare);

  // 2. Generate reverse keywords
  std::vector<std::unordered_set<int>> reverse_keywords(v_instance_graph.size());
  for (const auto& index: generated_group_vertices) {
    for (const auto& edge: v_generated_group_graph[index]) {
      reverse_keywords[edge.first].insert(index);
    }
  }

  // 3. Enqueue primitive trees
  for (int index = 0; index < v_instance_graph.size(); index++) {
    for (const auto& keywords: generate_subsets(reverse_keywords[index])) {
      Tree tree(index, keywords);
      T[index][keywords] = std::move(tree);
      Qt.emplace(T[index][keywords]);
    }
  }

  // 4. Main loop
  while (!Qt.empty()) {
    Tree tree = Qt.top();
    Qt.pop();

    if (!tree.is_in_queue) {
      continue;
    }

    if (tree.keywords == generated_group_vertices) {
      return tree.cost;
    }

    // 5. Expand tree
    for (const auto& [neighbor, weight]: v_instance_graph[tree.root]) {
      if (tree.vertices.find(neighbor) != tree.vertices.end()) {
        continue;
      }

      Tree expanded_tree = tree.mergeEdge(tree.root, neighbor, weight);

      auto other_tree = T[neighbor].find(expanded_tree.keywords);

      if (other_tree == T[neighbor].end() || other_tree->second.cost > expanded_tree.cost) {
        if (other_tree != T[neighbor].end()) {
          other_tree->second.is_in_queue = false;
        }

        T[neighbor][expanded_tree.keywords] = std::move(expanded_tree);
        Qt.emplace(T[neighbor][expanded_tree.keywords]);
      }
    }

    // 6. Merge trees
    for (const auto& [keywords, other_tree]: T[tree.root]) {
      if (keywords == tree.keywords) {
        continue;
      }

      if (
        std::any_of(
          keywords.begin(),
          keywords.end(),
          [&](int k) {
            return tree.keywords.count(k) > 0;
          }
        )
      ) {
        continue;
      }

      Tree merged_tree = tree.mergeTrees(other_tree);

      auto merged_tree_it = T[merged_tree.root].find(merged_tree.keywords);

      if (merged_tree_it == T[merged_tree.root].end() || merged_tree_it->second.cost > merged_tree.cost) {
        if (merged_tree_it != T[merged_tree.root].end()) {
          merged_tree_it->second.is_in_queue = false;
        }

        T[merged_tree.root][merged_tree.keywords] = std::move(merged_tree);
        Qt.emplace(T[merged_tree.root][merged_tree.keywords]);
      }
    }
  }

  return -1;
}
