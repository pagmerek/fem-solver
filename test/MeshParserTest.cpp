#include "../src/MeshParser.h"
#include "gtest/gtest.h"
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>

TEST(MeshParserTest, CorrectlyParsesMeshFormat) {
  auto mesh = Mesh("./assets/example_mesh.msh");

  EXPECT_EQ(mesh.format.version, 4.1);
  EXPECT_EQ(mesh.format.file_type, 0);
  EXPECT_EQ(mesh.format.data_size, 8);
}

TEST(MeshParserTest, CorrectlyParsesNodes) {
  auto mesh = Mesh("./assets/example_mesh.msh");

  EXPECT_EQ(mesh.nodes.num_blocks, 1);
  EXPECT_EQ(mesh.nodes.num_nodes, 6);
  EXPECT_EQ(mesh.nodes.min_tag, 1);
  EXPECT_EQ(mesh.nodes.max_tag, 6);

  EXPECT_EQ(mesh.nodes.blocks[0].entity_dim, 2);
  EXPECT_EQ(mesh.nodes.blocks[0].entity_tag, 1);
  EXPECT_EQ(mesh.nodes.blocks[0].node_type, 0);
  EXPECT_EQ(mesh.nodes.blocks[0].num_nodes_in_block, 6);

  auto nodes = std::vector<Node>{
      Node{1, 0, 0, 0}, Node{2, 1, 0, 0}, Node{3, 1, 1, 0},
      Node{4, 0, 1, 0}, Node{5, 2, 0, 0}, Node{6, 2, 1, 0},

  };

  for (int i = 0; i < mesh.nodes.blocks[0].num_nodes_in_block; i++) {
    EXPECT_EQ(mesh.nodes.blocks[0].nodes[i].node_tag, nodes[i].node_tag);

    EXPECT_EQ(mesh.nodes.blocks[0].nodes[i].x, nodes[i].x);
    EXPECT_EQ(mesh.nodes.blocks[0].nodes[i].y, nodes[i].y);
    EXPECT_EQ(mesh.nodes.blocks[0].nodes[i].z, nodes[i].z);
  }
}

TEST(MeshParserTest, CorrectlyParsesElements) {
  auto mesh = Mesh("./assets/example_mesh.msh");

  EXPECT_EQ(mesh.elements.num_blocks, 1);
  EXPECT_EQ(mesh.elements.num_elements, 2);
  EXPECT_EQ(mesh.elements.min_tag, 1);
  EXPECT_EQ(mesh.elements.max_tag, 2);

  EXPECT_EQ(mesh.elements.blocks[0].entity_dim, 2);
  EXPECT_EQ(mesh.elements.blocks[0].entity_tag, 1);
  EXPECT_EQ(mesh.elements.blocks[0].element_type, 3);
  EXPECT_EQ(mesh.elements.blocks[0].num_elements_in_block, 2);

  auto elements = std::vector<Element>{
      Element{1, std::vector<int>{1, 2, 3, 4}},
      Element{2, std::vector<int>{2, 5, 6, 3}},
  };
  EXPECT_EQ(mesh.elements.blocks[0].elements[0].element_tag,
            elements[0].element_tag);
  EXPECT_EQ(mesh.elements.blocks[0].elements[0].vertices, elements[0].vertices);

  EXPECT_EQ(mesh.elements.blocks[0].elements[1].element_tag,
            elements[1].element_tag);
  EXPECT_EQ(mesh.elements.blocks[0].elements[1].vertices, elements[1].vertices);
}

TEST(MeshParserTest, CorrectlyParsesNodeData) {
  auto mesh = Mesh("./assets/example_mesh.msh");

  EXPECT_EQ(mesh.node_data.string_tag, std::string("\"My view\""));
  EXPECT_DOUBLE_EQ(mesh.node_data.time, 0.0);
  EXPECT_EQ(mesh.node_data.time_step, 0);
  EXPECT_EQ(mesh.node_data.num_components, 3);
  EXPECT_EQ(mesh.node_data.num_nodes, 6);

  auto nodes = std::vector<Node>{
      Node{1, 0, 0, 0}, Node{2, 0.1, 0.1, 0.1}, Node{3, 0.2, 0.2, 0.2},
      Node{4, 0, 0, 0}, Node{5, 0.2, 0.2, 0.2}, Node{6, 0.4, 0.4, 0.4},

  };

  for (int i = 0; i < mesh.node_data.num_nodes; i++) {
    EXPECT_EQ(mesh.node_data.nodes[i].node_tag, nodes[i].node_tag);

    EXPECT_EQ(mesh.node_data.nodes[i].x, nodes[i].x);
    EXPECT_EQ(mesh.node_data.nodes[i].y, nodes[i].y);
    EXPECT_EQ(mesh.node_data.nodes[i].z, nodes[i].z);
  }
}

TEST(MeshParserTest, CorrectlySerializesMeshObj) {
  auto path = "../beam_initial.msh";
  auto mesh = Mesh(path);
  std::string serialized = mesh.serialize();

  std::ifstream infile(path);
  std::ostringstream buffer;
  buffer << infile.rdbuf();

  EXPECT_EQ(serialized, buffer.str());
}

TEST(MeshParserTest, CorrectlyReturnsAllNodes) {
  auto path = "./assets/example_mesh.msh";
  auto mesh = Mesh(path);

  auto all_nodes = mesh.getAllNodes();
  auto nodes = std::vector<Node>{
      Node{1, 0, 0, 0}, Node{2, 1, 0, 0}, Node{3, 1, 1, 0},
      Node{4, 0, 1, 0}, Node{5, 2, 0, 0}, Node{6, 2, 1, 0},

  };

  for (int i = 0; i < all_nodes.size(); i++) {
    EXPECT_EQ(all_nodes[i].node_tag, nodes[i].node_tag);

    EXPECT_EQ(all_nodes[i].x, nodes[i].x);
    EXPECT_EQ(all_nodes[i].y, nodes[i].y);
    EXPECT_EQ(all_nodes[i].z, nodes[i].z);
  }
}
