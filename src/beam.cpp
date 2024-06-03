#include <cmath>
#include <eigen3/Eigen/Sparse>
#include <filesystem>
#include <fstream>
#include <math.h>
#include <memory>
#include <onelab.h>
#include <string>
#include <vector>

#include "utils/MeshParser.h"
#include "utils/OnelabTools.h"

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

  double time = cufem::defineOnelabNumber(client, "Time [s]", 0., attr);
  double dt = cufem::defineOnelabNumber(client, "Time step [s]", 0.01, attr);
  double tmax = cufem::defineOnelabNumber(client, "Max time [s]", 20, attr);
  double refresh =
      cufem::defineOnelabNumber(client, "Refresh interval [s]", 0.05, attr);
  attr["Highlight"] = "Pink";

  if (action != "compute") {
    return 0;
  }

  double m1 = m;
  double m2 = m;
  double refr = 0.0;
  int iter = 0;
  time = 0.0;

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
      for (auto &node : motion_nodes) {
        node.x = 0;
        node.y = 0;
        node.z = 0;
      }

      for (auto &node : motion_nodes) {
        if (node.node_tag > 18 && node.node_tag < 24) {
          node.x = 0;
          node.y = -abs(sin(time));
          node.z = 0;
        }
      }

      mesh.updateNodeData("\"motion\"", time, iter, motion_nodes);
      std::ofstream updated(path + "beam.msh");
      updated << mesh.serialize();
      updated.close();

      client->sendMergeFileRequest(path + "beam.msh");
      iter += 1;
    }
  }

  cufem::setOnelabNumber(client, name + "/Progress", 0, 0, tmax, false);

  return 0;
}
