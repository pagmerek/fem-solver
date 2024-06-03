#include <memory>
#include <onelab.h>

namespace cufem {

using OnelabClient = std::shared_ptr<onelab::client>;

double defineOnelabNumber(OnelabClient client, const std::string &namer,
                          double value,
                          const std::map<std::string, std::string> &attributes);

void setOnelabNumber(OnelabClient client, const std::string &name, double value,
                     double min = 0, double max = 0, bool visible = true);
void addOnelabNumberChoice(OnelabClient client, const std::string &name,
                           double choice);
} // namespace cufem
