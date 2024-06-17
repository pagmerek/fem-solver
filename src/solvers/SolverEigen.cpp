#include "SolverEigen.h"
#include <cmath>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <unordered_map>

namespace cufem {

SolverEigen::SolverEigen(std::vector<Node> nodes, std::vector<Element> elements)
    : SolverFEM() {
  int vertices_amount = nodes.size();
  int dimension = 2; // hardcoded for 2d beam
  Eigen::MatrixXf vertex_matrix = Eigen::MatrixXf(vertices_amount, dimension);

  int elements_amount = elements.size();
  int element_size = 3; // hardcoded for triangles only
  Eigen::MatrixXi element_matrix =
      Eigen::MatrixXi(elements_amount, element_size);

  for (int i = 0; i < vertices_amount; i++) {
    vertex_matrix(i, 0) = nodes[i].x;
    vertex_matrix(i, 1) = nodes[i].y;
  }

  this->vertices = vertex_matrix;

  for (int i = 0; i < elements_amount; i++) {
    for (int j = 0; j < element_size; j++) {
      element_matrix(i, j) = elements[i].vertices[j];
    }
  }
  this->elements = element_matrix;
}

SolverEigen &SolverEigen::setYoungModulus(const float youngModulus) {
  this->youngModulus = youngModulus;
  return *this;
}

SolverEigen &SolverEigen::setPoissonRatio(const float poissonRatio) {
  this->poissonRatio = poissonRatio;
  return *this;
}

SolverEigen &SolverEigen::setMaterialDensity(const float density) {
  this->materialDensity = density;
  return *this;
}

SolverEigen &SolverEigen::setGravityForce(const float gravity) {
  this->gravityForce = gravity;
  return *this;
}

SolverEigen &SolverEigen::setNeumannConditions(
    const std::unordered_map<int, std::tuple<int, float, float>>
        neumannConditions) {
  this->neumannConditions = neumannConditions;
  return *this;
}

SolverEigen &SolverEigen::setDirichletConditions(
    const std::unordered_map<int, float> dirichletConditions) {
  this->dirichletConditions = dirichletConditions;
  return *this;
}

float get_triangle_area(const Eigen::MatrixXf &coordinates) {

  Eigen::MatrixXf area_matrix = Eigen::MatrixXf::Ones(3, 3);
  area_matrix.col(0) = coordinates.col(0);
  area_matrix.col(1) = coordinates.col(1);

  return std::abs(area_matrix.determinant());
}

// TODO: Calculate local stiffness by matrix operations
//
// Eigen::MatrixXf SolverEigen::get_stiffness_xx(const Eigen::MatrixXf &element,
//                                               float element_area) {
//   auto lambda = youngModulus * poissonRatio / (1 - poissonRatio *
//   poissonRatio); auto v = youngModulus / (1 + poissonRatio);
//
//   return element;
// }
// Eigen::MatrixXf SolverEigen::get_stiffness_xy(const Eigen::MatrixXf &element,
//                                               float element_area) {
//   return element;
// }
//
// Eigen::MatrixXf SolverEigen::get_stiffness_yy(const Eigen::MatrixXf &element,
//                                               float element_area) {
//   return element;
// }

SolverEigen &SolverEigen::assembleStiffness() {
  auto N = vertices.rows();
  auto lambda = youngModulus * poissonRatio / (1 - poissonRatio * poissonRatio);
  auto v = youngModulus / (1 + poissonRatio);

  Eigen::MatrixXf stiffness_matrix = Eigen::SparseMatrix<float>(N * 2, N * 2);

  using StiffnessTriplet = Eigen::Triplet<float>;
  std::vector<StiffnessTriplet> tripletList;

  for (int i = 0; i < elements.rows(); i++) {
    auto element_vertices = vertices(elements.row(i), Eigen::all);
    auto element_area = get_triangle_area(element_vertices);

    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        if (dirichletConditions.contains(elements(i, j)) ||
            dirichletConditions.contains(elements(i, k))) {
          continue;
        }

        float stiffnessXX = (lambda + v) / (4 * element_area * element_area) *
                                (element_vertices((j + 1) % 3, 1) -
                                 element_vertices((j + 2) % 3, 1)) *
                                (element_vertices((k + 1) % 3, 1) -
                                 element_vertices((k + 2) % 3, 1)) +
                            v / (8 * element_area * element_area) *
                                (element_vertices((j + 2) % 3, 0) -
                                 element_vertices((j + 1) % 3, 0)) *
                                (element_vertices((k + 2) % 3, 0) -
                                 element_vertices((k + 1) % 3, 0));

        float stiffnessXY = (lambda + v) / (4 * element_area * element_area) *
                                (element_vertices((j + 2) % 3, 0) -
                                 element_vertices((j + 1) % 3, 0)) *
                                (element_vertices((k + 1) % 3, 1) -
                                 element_vertices((k + 2) % 3, 1)) +
                            v / (8 * element_area * element_area) *
                                (element_vertices((j + 1) % 3, 1) -
                                 element_vertices((j + 2) % 3, 1)) *
                                (element_vertices((k + 2) % 3, 0) -
                                 element_vertices((k + 1) % 3, 0));

        float stiffnessYX = (lambda + v) / (4 * element_area * element_area) *
                                (element_vertices((k + 2) % 3, 0) -
                                 element_vertices((k + 1) % 3, 0)) *
                                (element_vertices((j + 1) % 3, 1) -
                                 element_vertices((j + 2) % 3, 1)) +
                            v / (8 * element_area * element_area) *
                                (element_vertices((k + 1) % 3, 1) -
                                 element_vertices((k + 2) % 3, 1)) *
                                (element_vertices((j + 2) % 3, 0) -
                                 element_vertices((j + 1) % 3, 0));
        float stiffnessYY = (lambda + v) / (4 * element_area * element_area) *
                                (element_vertices((j + 2) % 3, 0) -
                                 element_vertices((j + 1) % 3, 0)) *
                                (element_vertices((k + 2) % 3, 0) -
                                 element_vertices((k + 1) % 3, 0)) +
                            v / (8 * element_area * element_area) *
                                (element_vertices((j + 1) % 3, 1) -
                                 element_vertices((j + 2) % 3, 1)) *
                                (element_vertices((k + 1) % 3, 1) -
                                 element_vertices((k + 2) % 3, 1));

        tripletList.push_back(StiffnessTriplet(
            elements(i, j) - 1, elements(i, k) - 1, stiffnessXX));

        tripletList.push_back(StiffnessTriplet(
            elements(i, j) - 1 + N, elements(i, k) - 1, stiffnessXY));

        tripletList.push_back(StiffnessTriplet(
            elements(i, j) - 1, elements(i, k) - 1 + N, stiffnessYX));

        tripletList.push_back(StiffnessTriplet(
            elements(i, j) - 1 + N, elements(i, k) - 1 + N, stiffnessYY));
      }
    }
  }

  for (auto &[vertex, value] : dirichletConditions) {
    tripletList.push_back(StiffnessTriplet(vertex - 1, vertex - 1, 1));

    tripletList.push_back(StiffnessTriplet(vertex - 1 + N, vertex - 1 + N, 1));
  }

  Eigen::SparseMatrix<float> stiffnessMatrix =
      Eigen::SparseMatrix<float>(2 * N, 2 * N);

  stiffnessMatrix.setFromTriplets(tripletList.begin(), tripletList.end());

  this->stiffnessMatrix = stiffnessMatrix;

  return *this;
}

SolverEigen &SolverEigen::calculateNodalForces() {
  auto N = this->vertices.rows();
  Eigen::VectorXf nodalForces = Eigen::VectorXf::Zero(2 * N);

  for (int i = 0; i < elements.rows(); i++) {
    auto element_vertices = vertices(elements.row(i), Eigen::all);
    auto element_area = get_triangle_area(element_vertices);

    for (int j = 0; j < element_vertices.rows(); j++) {
      if (dirichletConditions.contains(elements(i, j)))
        continue;

      if (neumannConditions.contains(elements(i, j))) {
        auto [second_vertex, value_x, value_y] =
            neumannConditions[elements(i, j)];

        Eigen::Vector2f normalVec{vertices(j, 0) - vertices(second_vertex, 0),
                                  vertices(j, 1) - vertices(second_vertex, 1)};
        normalVec = Eigen::Vector2f{normalVec[1], -normalVec[0]};
        normalVec = normalVec / normalVec.norm();

        nodalForces(elements(i, j) - 1) += normalVec(0) * value_x;

        nodalForces(elements(i, j) - 1 + N) += normalVec(1) * value_y;

        nodalForces(second_vertex - 1) += normalVec(0) * value_x;
        nodalForces(second_vertex - 1 + N) += normalVec(1) * value_y;
      }

      nodalForces(elements(i, j) - 1 + N) +=
          0.5 * element_area * materialDensity * gravityForce;
    }
  }

  this->nodalForces = nodalForces;
  return *this;
}
std::vector<float> SolverEigen::solve() {
  auto alpha = youngModulus / (1 - poissonRatio * poissonRatio);
  Eigen::SparseLU<Eigen::SparseMatrix<float>> solver;
  solver.analyzePattern(this->stiffnessMatrix);
  solver.factorize(this->stiffnessMatrix);
  auto result = solver.solve(this->nodalForces) / alpha;
  std::vector<float> coefficients;
  coefficients.resize(result.size());
  Eigen::VectorXf::Map(&coefficients[0], result.size()) = result;
  return coefficients;
}

} // namespace cufem
