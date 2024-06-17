#include "../utils/Mesh.h"
#include "SolverFEM.h"
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/SparseCore/SparseMatrix.h>

namespace cufem {
class SolverEigen : SolverFEM {

public:
  SolverEigen(std::vector<Node> nodes, std::vector<Element> elements);

  SolverEigen &setYoungModulus(const float youngModulus) override;
  SolverEigen &setPoissonRatio(const float poissonRatio) override;
  SolverEigen &setMaterialDensity(const float density) override;
  SolverEigen &setGravityForce(const float gravity) override;
  SolverEigen &assembleStiffness() override;
  SolverEigen &calculateNodalForces() override;
  SolverEigen &setDirichletConditions(
      const std::unordered_map<int, float> dirichletConditions) override;

  SolverEigen &setNeumannConditions(
      const std::unordered_map<int, std::tuple<int, float, float>>) override;

  std::vector<float> solve() override;

  Eigen::SparseMatrix<float> stiffnessMatrix;
  Eigen::VectorXf nodalForces;

private:
  float youngModulus;
  float poissonRatio;
  float gravityForce;
  float materialDensity;

  std::unordered_map<int, float> dirichletConditions;
  std::unordered_map<int, std::tuple<int, float, float>> neumannConditions;
  Eigen::MatrixXf vertices;
  Eigen::MatrixXi elements;

  Eigen::MatrixXf get_stiffness_xx(const Eigen::MatrixXf &element,
                                   float elementArea);
  Eigen::MatrixXf get_stiffness_xy(const Eigen::MatrixXf &element,
                                   float elementArea);
  Eigen::MatrixXf get_stiffness_yy(const Eigen::MatrixXf &element,
                                   float elementArea);
};
} // namespace cufem
