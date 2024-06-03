#include "SolverFEM.h"
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/SparseCore/SparseMatrix.h>

namespace cufem::solvers {
class SolverEigen : SolverFEM {

public:
  void assemble_stiffness() override;
  void derive_weak() override;
  void solve() override;

private:
  Eigen::SparseMatrix<float> stiffnessMatrix;
  Eigen::Matrix2X<float> RHS;
};
} // namespace cufem::solvers
