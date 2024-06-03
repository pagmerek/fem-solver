
#include "OnelabTools.h"

namespace cufem {

double
defineOnelabNumber(OnelabClient client, const std::string &name, double value,
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

void setOnelabNumber(OnelabClient client, const std::string &name, double value,
                     double min, double max, bool visible) {
  onelab::number n(name, value);
  n.setMin(min);
  n.setMax(max);
  n.setVisible(visible);
  client->set(n);
}

void addOnelabNumberChoice(OnelabClient client, const std::string &name,
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
} // namespace cufem
