#include <cmath>
#include <cstdio>
#include <eigen3/Eigen/Sparse>
#include <filesystem>
#include <fstream>
#include <math.h>
#include <memory>
#include <onelab.h>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "solvers/SolverEigen.h"
#include "utils/Mesh.h"
#include "utils/OnelabTools.h"

std::unordered_map<int, float> getDirchletFromFile(std::filesystem::path path) {
  std::ifstream infile(path);
  if (!infile) {
    throw std::runtime_error("Unable to open Dirichlet file");
  }
  std::unordered_map<int, float> dirichlet;

  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream dirichlet_data(line);
    int vertex;
    float value;
    dirichlet_data >> vertex >> value;
    dirichlet[vertex] = value;
  }
  return dirichlet;
}

std::unordered_map<int, std::tuple<int, float, float>>
getNeumannFromFile(std::filesystem::path path) {
  std::ifstream infile(path);
  if (!infile) {
    throw std::runtime_error("Unable to open Neumann file");
  }
  std::unordered_map<int, std::tuple<int, float, float>> neumann;

  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream dirichlet_data(line);
    int vertex1, vertex2;
    float value_x, value_y;
    dirichlet_data >> vertex1 >> vertex2 >> value_x >> value_y;
    neumann[vertex1] = {vertex2, value_x, value_y};
  }
  return neumann;
}

int main(int argc, char **argv) {
  std::string name, address;
  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "-onelab" && i + 2 < argc) {
      name = std::string(argv[i + 1]);
      address = std::string(argv[i + 2]);
    }
  }

  if (name.empty() || address.empty())
    return 1;

  auto client = std::make_shared<onelab::remoteNetworkClient>(name, address);

  std::string action;
  std::vector<onelab::string> ns;
  client->get(ns, name + "/Action");
  if (ns.size())
    action = ns[0].getValue();

  // prevent automatic Gmsh model reload & meshing
  onelab::number n("IsMetamodel", 1);
  client->set(n);

  std::string path(argv[0]);
  int islash = (int)path.find_last_of("/\\");
  if (islash > 0)
    path = path.substr(0, islash + 1) + "../../";
  else
    path = "";

  double g = 9.8; // acceleration of gravity
  double m = 0.3; // mass of object

  std::map<std::string, std::string> attr;

  float young_modulus =
      cufem::defineOnelabNumber(client, "Young Modulus", 190e6, attr);

  float poisson_ratio =
      cufem::defineOnelabNumber(client, "Poisson Ratio", 0.33, attr);

  float material_density =
      cufem::defineOnelabNumber(client, "Material Density", 7850, attr);
  double time = cufem::defineOnelabNumber(client, "Time [s]", 0., attr);
  double dt = cufem::defineOnelabNumber(client, "Time step [s]", 0.01, attr);
  double tmax = cufem::defineOnelabNumber(client, "Max time [s]", 20, attr);
  double refresh =
      cufem::defineOnelabNumber(client, "Refresh interval [s]", 0.5, attr);
  attr["Highlight"] = "Pink";

  if (action != "compute") {
    return 0;
  }

  double m1 = m;
  double m2 = m;
  double refr = 0.0;
  int iter = 0;
  time = 0.0;

  auto dirichlet_conditions = getDirchletFromFile("./dirichlet");
  auto neumann_conditions = getNeumannFromFile("./neumann");

  while (time < tmax) {
    time += dt;
    refr += dt;

    if (refr >= refresh) {
      refr = 0;
      cufem::setOnelabNumber(client, name + "/Progress", time, 0, tmax, false);
      cufem::setOnelabNumber(client, "Dyna/time [s]", time);

      // ask Gmsh to refresh
      onelab::string s("Gmsh/Action", "refresh");
      client->set(s);

      // stop if we are asked to (by Gmsh)
      client->get(ns, name + "/Action");
      if (ns.size() && ns[0].getValue() == "stop")
        break;

      auto mesh = cufem::Mesh(path + "beam_initial.msh");
      auto motion_nodes = mesh.getAllNodes();
      auto coefficients =
          cufem::SolverEigen(mesh.getAllNodes(), mesh.getAllElements())
              .setYoungModulus(young_modulus)
              .setPoissonRatio(poisson_ratio)
              .setMaterialDensity(material_density)
              .setGravityForce(-9.81)
              .setDirichletConditions(dirichlet_conditions)
              .setNeumannConditions(neumann_conditions)
              .assembleStiffness()
              .calculateNodalForces()
              .solve();

      for (int i = 0; i < motion_nodes.size(); i++) {
        motion_nodes[i].x = coefficients[i];
        motion_nodes[i].y = coefficients[i + motion_nodes.size()];
        motion_nodes[i].z = 0;
      }
      mesh.updateNodes(motion_nodes);

      mesh.updateNodeData("\"motion\"", time, iter, motion_nodes);
      std::ofstream updated(path + "beam.msh");
      updated << mesh.serialize();
      updated.close();

      client->sendMergeFileRequest(path + "beam.msh");
      break;
      iter += 1;
    }
  }

  cufem::setOnelabNumber(client, name + "/Progress", 0, 0, tmax, false);

  return 0;
}
