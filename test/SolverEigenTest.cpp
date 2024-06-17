#include "../src/solvers/SolverEigen.h"
#include "../src/utils/Mesh.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/SparseCore/SparseMatrix.h>
#include <gtest/gtest.h>
#include <iostream>

TEST(SolverEigenTest, AssemblesSymmetricStiffnessMatrix) {
  auto mesh = cufem::Mesh("./assets/triangle.msh");

  auto stiffnessMatrix =
      cufem::SolverEigen(mesh.getAllNodes(), mesh.getAllElements())
          .setYoungModulus(110)
          .setPoissonRatio(0.33)
          .assembleStiffness()
          .stiffnessMatrix;

  EXPECT_TRUE(stiffnessMatrix.isApprox(stiffnessMatrix.transpose()));
}

TEST(SolverEigenTest, CalculatesNodalForces) {
  auto mesh = cufem::Mesh("./assets/triangle.msh");

  auto nodalForces =
      cufem::SolverEigen(mesh.getAllNodes(), mesh.getAllElements())
          .setMaterialDensity(500)
          .setGravityForce(-9.81)
          .calculateNodalForces()
          .nodalForces;

  EXPECT_TRUE(nodalForces.allFinite());
}

TEST(SolverEigenTest, AddsDirichletConditions) {
  auto mesh = cufem::Mesh("./assets/triangle.msh");

  std::unordered_map<int, float> dirichlet{{3, 0.}};
  auto solver = cufem::SolverEigen(mesh.getAllNodes(), mesh.getAllElements())
                    .setYoungModulus(110)
                    .setPoissonRatio(0.33)
                    .setMaterialDensity(7850)
                    .setGravityForce(-9.81)
                    .setDirichletConditions(dirichlet)
                    .assembleStiffness()
                    .calculateNodalForces();

  auto N = solver.stiffnessMatrix.rows();

  Eigen::MatrixXf stiffnessMatrix(solver.stiffnessMatrix);
  Eigen::MatrixXf identity =
      Eigen::MatrixXf::Identity(stiffnessMatrix.rows(), stiffnessMatrix.cols());

  EXPECT_TRUE(stiffnessMatrix.row(2) == identity.row(2));

  EXPECT_TRUE(stiffnessMatrix.row(2 + N / 2) == identity.row(2 + N / 2));

  EXPECT_TRUE(stiffnessMatrix.col(2) == identity.col(2));

  EXPECT_TRUE(stiffnessMatrix.col(2 + N / 2) == identity.col(2 + N / 2));

  EXPECT_TRUE(solver.nodalForces(2) == 0 && solver.nodalForces(2 + N / 2) == 0);
}

TEST(SolverEigenTest, AddsNeumannConditions) {
  auto mesh = cufem::Mesh("./assets/triangle.msh");

  std::unordered_map<int, std::tuple<int, float, float>> neumann{
      {3, {1, 50, 0}}, {2, {3, 0, -100}}};

  auto solver = cufem::SolverEigen(mesh.getAllNodes(), mesh.getAllElements())
                    .setYoungModulus(110)
                    .setPoissonRatio(0.33)
                    .setMaterialDensity(7850)
                    .setGravityForce(-9.81)
                    .setNeumannConditions(neumann)
                    .assembleStiffness()
                    .calculateNodalForces();
}

TEST(SolverEigenTest, SolvesSystem) {
  auto mesh = cufem::Mesh("./assets/triangle.msh");

  std::unordered_map<int, float> dirichlet{{3, 0.}};
  auto coefficients =
      cufem::SolverEigen(mesh.getAllNodes(), mesh.getAllElements())
          .setYoungModulus(190e6)
          .setPoissonRatio(0.30)
          .setMaterialDensity(7850)
          .setGravityForce(-9.81)
          .setDirichletConditions(dirichlet)
          .assembleStiffness()
          .calculateNodalForces()
          .solve();

  EXPECT_NO_THROW();
}
