
#include <filesystem>
#include <string>
#include <vector>

namespace cufem {

struct Node {
  int node_tag;
  double x;
  double y;
  double z;
};

struct Element {
  int element_tag;
  std::vector<int> vertices;
};

struct ElementBlock {
  int entity_dim;
  int entity_tag;
  int element_type;
  int num_elements_in_block;
  std::vector<Element> elements;
};

struct Elements {
  int num_blocks;
  int num_elements;
  int min_tag;
  int max_tag;
  std::vector<ElementBlock> blocks;
};

struct NodeBlock {
  int entity_dim;
  int entity_tag;
  int node_type;
  int num_nodes_in_block;
  std::vector<Node> nodes;
};

struct Nodes {
  int num_blocks;
  int num_nodes;
  int min_tag;
  int max_tag;
  std::vector<NodeBlock> blocks;
};

struct NodeData {
  std::string string_tag;
  double time = 0;
  int time_step = 0;
  int num_components = 0;
  int num_nodes = 0;
  std::vector<Node> nodes = std::vector<Node>();
};

struct MeshFormat {
  double version;
  int file_type;
  int data_size;
};

class Mesh {
public:
  MeshFormat format;
  Nodes nodes;
  Elements elements;
  NodeData node_data;

  Mesh();

  Mesh(std::filesystem::path path);

  std::string serialize();

  std::vector<Node> getAllNodes();

  void updateNodeData(std::string tag, double time, int iter,
                      std::vector<Node> motions);
};
} // namespace cufem
