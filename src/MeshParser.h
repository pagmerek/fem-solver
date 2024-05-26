
#include <filesystem>
#include <string>
#include <vector>

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
  double time;
  int time_step;
  int num_components;
  int num_nodes;
  std::vector<Node> nodes;
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

  Mesh(std::filesystem::path path);

  std::string serialize();
};
