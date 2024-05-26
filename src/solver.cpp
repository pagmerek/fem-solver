#include "onelab.h"
#include <eigen3/Eigen/Sparse>
#include <filesystem>
#include <iostream>
#include <math.h>
#include <memory>
#include <string>
#include <vector>

using OnelabClient = std::shared_ptr<onelab::client>;

void exportMsh(const std::string &path) {
  std::filesystem::copy_file(path + "beam_initial.msh",
                             path + "beam_modified.msh");
}

void exportMshOpt(const std::string &path) {
  FILE *optFile = fopen((path + "beam.msh.opt").c_str(), "w");
  if (!optFile)
    return;
  fprintf(optFile, "n = PostProcessing.NbViews - 1;\n");
  fprintf(optFile,
          "If(n >= 0)\nView[n].ShowScale = 0;\nView[n].VectorType = 5;\n");
  fprintf(optFile,
          "View[n].ExternalView = 0;\nView[n].DisplacementFactor = 1 ;\n");
  fprintf(optFile, "View[n].PointType = 1;\nView[n].PointSize = 5;\n");
  fprintf(optFile, "View[n].LineWidth = 2;\nEndIf\n");
  fclose(optFile);
}

void exportIter(const std::string &path, int iter, double t) {
  FILE *mshFile = fopen((path + "beam_modified.msh").c_str(), "a");
  if (!mshFile)
    return;
  fprintf(mshFile, "$NodeData\n\
          1\n\
          \"motion\"\n\
          1\n\
          \t%f\n\
          3\n\
          \t%d\n\
          3\n",
          t, iter);

  fprintf(mshFile, "\t3\n\
          \t1 0 0 0\n\
          \t2 0 0 0\n\
          \t3 0 0 0\n\
          $EndNodeData\n");
  fclose(mshFile);
}

double defineNumber(OnelabClient client, const std::string &name, double value,
                    const std::map<std::string, std::string> &attributes) {
  std::vector<onelab::number> ns;
  client->get(ns, name);
  if (ns.empty()) { // define new parameter
    onelab::number n(name, value);
    if (attributes.size())
      n.setAttributes(attributes);
    client->set(n);
    return value;
  }
  // return value from server
  return ns[0].getValue();
}

void setNumber(OnelabClient client, const std::string &name, double value,
               double min = 0, double max = 0, bool visible = true) {
  onelab::number n(name, value);
  n.setMin(min);
  n.setMax(max);
  n.setVisible(visible);
  client->set(n);
}

void addNumberChoice(OnelabClient client, const std::string &name,
                     double choice) {
  std::vector<onelab::number> ns;
  client->get(ns, name);
  if (ns.size()) {
    std::vector<double> choices = ns[0].getChoices();
    choices.push_back(choice);
    ns[0].setChoices(choices);
    client->set(ns[0]);
  }
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
    path = path.substr(0, islash + 1);
  else
    path = "";

  double g = 9.8; // acceleration of gravity
  double m = 0.3; // mass of object

  std::map<std::string, std::string> attr;

  double time = defineNumber(client, "Time [s]", 0., attr);
  double dt = defineNumber(client, "Time step [s]", 0.001, attr);
  double tmax = defineNumber(client, "Max time [s]", 20, attr);
  double refresh = defineNumber(client, "Refresh interval [s]", 0.05, attr);
  attr["Highlight"] = "Pink";

  // we're done if we are not in the compute phase
  if (action != "compute") {
    return 0;
  }

  double m1 = m;
  double m2 = m;
  double refr = 0.0;
  int iter = 0;
  time = 0.0;

  while (time < tmax) {

    exportMshOpt(path);
    time += dt;
    refr += dt;

    if (refr >= refresh) {
      refr = 0;
      setNumber(client, name + "/Progress", time, 0, tmax, false);
      setNumber(client, "Dyna/time [s]", time);

      // ask Gmsh to refresh
      onelab::string s("Gmsh/Action", "refresh");
      client->set(s);

      // stop if we are asked to (by Gmsh)
      client->get(ns, name + "/Action");
      if (ns.size() && ns[0].getValue() == "stop")
        break;

      exportMsh(path);
      exportIter(path, iter, time);
      client->sendMergeFileRequest(path + "beam.msh");
      iter += 1;
    }
  }

  setNumber(client, name + "/Progress", 0, 0, tmax, false);

  return 0;
}
