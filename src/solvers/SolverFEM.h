#include <unordered_map>
#include <vector>
namespace cufem {

class SolverFEM {
public:
  SolverFEM(){};

  virtual SolverFEM &setYoungModulus(const float youngModulus) = 0;

  virtual SolverFEM &setPoissonRatio(const float poissonRatio) = 0;

  virtual SolverFEM &setMaterialDensity(const float density) = 0;

  virtual SolverFEM &setGravityForce(const float gravity) = 0;

  virtual SolverFEM &setDirichletConditions(
      const std::unordered_map<int, float> dirichletConditions) = 0;

  virtual SolverFEM &setNeumannConditions(
      const std::unordered_map<int, std::tuple<int, float, float>>
          neumannConditions) = 0;

  virtual SolverFEM &assembleStiffness() = 0;

  virtual SolverFEM &calculateNodalForces() = 0;

  virtual std::vector<float> solve() = 0;

  virtual ~SolverFEM() = default;
};

} // namespace cufem
