#include "MeshParser.h"
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

enum class MshBlock { MeshFormat, Nodes, Elements, NodeData };

std::map<std::string, MshBlock> mapStringToMshBlock = {
    {"$MeshFormat", MshBlock::MeshFormat},
    {"$Nodes", MshBlock::Nodes},
    {"$Elements", MshBlock::Elements},
    {"$NodeData", MshBlock::NodeData},
};

std::map<MshBlock, std::string> mapMshBlockToString{
    {MshBlock::MeshFormat, "$MeshFormat"},
    {MshBlock::Nodes, "$Nodes"},
    {MshBlock::Elements, "$Elements"},
    {MshBlock::NodeData, "$NodeData"},
};

std::optional<MeshFormat> parseMeshFormat(std::ifstream &infile) {
  std::string line;
  std::getline(infile, line);
  std::istringstream iss(line);

  double version;
  int file_size, data_type;
  if (!(iss >> version >> data_type >> file_size)) {
    return {};
  }
  std::getline(infile, line);
  if (line.compare("$EndMeshFormat") != 0) {
    return {};
  }
  return MeshFormat{version, data_type, file_size};
}

std::optional<Nodes> parseNodes(std::ifstream &infile) {
  std::string line;
  std::getline(infile, line);
  std::istringstream iss(line);

  int num_blocks, num_nodes, min_tag, max_tag;
  if (!(iss >> num_blocks >> num_nodes >> min_tag >> max_tag)) {
    return {};
  }

  auto blocks = std::vector<NodeBlock>();
  for (int i = 0; i < num_blocks; i++) {
    std::getline(infile, line);
    std::istringstream iss(line);

    int entity_dim, entity_tag, node_type, num_nodes_in_block;
    if (!(iss >> entity_dim >> entity_tag >> node_type >> num_nodes_in_block)) {
      return {};
    }
    auto nodes = std::vector<Node>();
    for (int j = 0; j < num_nodes_in_block; j++) {
      std::getline(infile, line);
      std::istringstream iss(line);

      int node_tag;
      if (!(iss >> node_tag)) {
        return {};
      }
      nodes.push_back(Node{node_tag});
    }
    for (int j = 0; j < num_nodes_in_block; j++) {
      std::getline(infile, line);
      std::istringstream iss(line);

      double x, y, z;
      if (!(iss >> x >> y >> z)) {
        return {};
      }
      nodes[j].x = x;
      nodes[j].y = y;
      nodes[j].z = z;
    }
    blocks.push_back(NodeBlock{entity_dim, entity_tag, node_type,
                               num_nodes_in_block, nodes});
  }

  std::getline(infile, line);
  if (line.compare("$EndNodes") != 0) {
    return {};
  }
  return Nodes{num_blocks, num_nodes, min_tag, max_tag, blocks};
}

std::optional<Elements> parseElements(std::ifstream &infile) {
  std::string line;
  std::getline(infile, line);
  std::istringstream header(line);

  int num_blocks, num_elements, min_tag, max_tag;
  if (!(header >> num_blocks >> num_elements >> min_tag >> max_tag)) {
    return {};
  }

  auto blocks = std::vector<ElementBlock>();
  for (int i = 0; i < num_blocks; i++) {
    std::getline(infile, line);
    std::istringstream iss(line);

    int entity_dim, entity_tag, element_type, num_elements_in_block;
    if (!(iss >> entity_dim >> entity_tag >> element_type >>
          num_elements_in_block)) {
      return {};
    }
    auto elements = std::vector<Element>();
    for (int j = 0; j < num_elements_in_block; j++) {
      std::getline(infile, line);
      std::istringstream line_stream(line);
      std::string element_tag;

      std::getline(line_stream, element_tag, ' ');

      auto vertices = std::vector<int>();
      for (int k = 0; k < element_type + 1; k++) {
        std::string vertex;
        std::getline(line_stream, vertex, ' ');
        vertices.push_back(std::stoi(vertex));
      }
      elements.push_back(Element{std::stoi(element_tag), vertices});
    }
    blocks.push_back(ElementBlock{entity_dim, entity_tag, element_type,
                                  num_elements_in_block, elements});
  }
  std::getline(infile, line);

  if (line.compare("$EndElements") != 0) {
    return {};
  }

  return Elements{num_blocks, num_elements, min_tag, max_tag, blocks};
}

std::optional<NodeData> parseNodeData(std::ifstream &infile) {
  std::string line;
  std::getline(infile, line);
  std::getline(infile, line);

  std::string view_name = line;

  std::getline(infile, line);
  std::getline(infile, line);
  try {
    double time = std::stod(line);

    std::getline(infile, line);
    std::getline(infile, line);
    int time_step = std::stoi(line);

    std::getline(infile, line);
    int num_components = std::stoi(line);

    std::getline(infile, line);
    int num_nodes = std::stoi(line);

    auto nodes = std::vector<Node>();

    std::cout << num_nodes << std::endl;
    for (int i = 0; i < num_nodes; i++) {
      std::getline(infile, line);
      std::istringstream node_data(line);

      int node_tag;
      double x, y, z;
      if (!(node_data >> node_tag >> x >> y >> z)) {
        return {};
      }
      nodes.push_back(Node{node_tag, x, y, z});
    }

    std::getline(infile, line);
    if (line.compare("$EndNodeData") != 0) {
      return {};
    }
    return NodeData{view_name,      time,      time_step,
                    num_components, num_nodes, nodes};
  } catch (std::invalid_argument &e) {
    return {};
  }
}

Mesh::Mesh(std::filesystem::path path) {
  std::ifstream infile(path);
  if (!infile) {
    throw std::runtime_error("Unable to open file of path");
  }

  std::string line;
  while (std::getline(infile, line)) {
    if (line.starts_with("$")) {
      MshBlock block = mapStringToMshBlock[line];
      switch (block) {
      case MshBlock::MeshFormat: {
        auto mesh_format = parseMeshFormat(infile);
        if (mesh_format.has_value()) {
          this->format = mesh_format.value();
        }
        break;
      }
      case MshBlock::Nodes: {
        auto nodes = parseNodes(infile);
        if (nodes.has_value()) {
          this->nodes = nodes.value();
        }
        break;
      }
      case MshBlock::Elements: {
        auto elements = parseElements(infile);
        if (elements.has_value())
          this->elements = elements.value();
        break;
      }
      case MshBlock::NodeData: {
        auto node_data = parseNodeData(infile);
        if (node_data.has_value()) {
          this->node_data = node_data.value();
        }
        break;
      }
      }
    }
  }
  infile.close();
}

std::string Mesh::serialize() {
  std::ostringstream mesh;

  mesh << "$MeshFormat\n";
  mesh << format.version << ' ' << format.file_type << ' ' << format.data_size
       << "\n";
  mesh << "$EndMeshFormat\n";

  mesh << "$Nodes\n";
  mesh << nodes.num_blocks << ' ' << nodes.num_nodes << ' ' << nodes.min_tag
       << ' ' << nodes.max_tag << "\n";

  for (int i = 0; i < nodes.num_blocks; i++) {
    mesh << nodes.blocks[i].entity_dim << ' ' << nodes.blocks[i].entity_tag
         << ' ' << nodes.blocks[i].node_type << ' '
         << nodes.blocks[i].num_nodes_in_block << '\n';
    for (int j = 0; j < nodes.blocks[i].num_nodes_in_block; j++) {
      mesh << nodes.blocks[i].nodes[j].node_tag << '\n';
    }

    for (int j = 0; j < nodes.blocks[i].num_nodes_in_block; j++) {
      mesh << nodes.blocks[i].nodes[j].x << ' ' << nodes.blocks[i].nodes[j].y
           << ' ' << nodes.blocks[i].nodes[j].z << '\n';
    }
  }
  mesh << "$EndNodes\n";

  mesh << "$Elements\n";
  mesh << elements.num_blocks << ' ' << elements.num_elements << ' '
       << elements.min_tag << ' ' << elements.max_tag << "\n";
  for (int i = 0; i < elements.num_blocks; i++) {
    mesh << elements.blocks[i].entity_dim << ' '
         << elements.blocks[i].entity_tag << ' '
         << elements.blocks[i].element_type << ' '
         << elements.blocks[i].num_elements_in_block << '\n';

    for (int j = 0; j < elements.blocks[i].num_elements_in_block; j++) {
      mesh << elements.blocks[i].elements[j].element_tag << ' ';
      for (int k = 0; k < elements.blocks[i].element_type + 1; k++) {
        if (k != elements.blocks[i].element_type) {
          mesh << elements.blocks[i].elements[j].vertices[k] << ' ';
        } else {
          mesh << elements.blocks[i].elements[j].vertices[k] << '\n';
        }
      }
    }
  }
  mesh << "$EndElements\n";

  mesh << "$NodeData\n";
  mesh << "1\n";
  mesh << node_data.string_tag << '\n';
  mesh << "1\n";
  mesh << node_data.time << '\n';
  mesh << "3\n";
  mesh << node_data.time_step << '\n';
  mesh << node_data.num_components << '\n';
  mesh << node_data.num_nodes << '\n';

  for (int i = 0; i < node_data.num_nodes; i++) {
    mesh << node_data.nodes[i].node_tag << ' ' << node_data.nodes[i].x << ' '
         << node_data.nodes[i].y << ' ' << node_data.nodes[i].z << '\n';
  }
  mesh << "$EndNodeData\n";

  return mesh.str();
}
