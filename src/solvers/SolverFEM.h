

namespace cufem::solvers {

class SolverFEM {
  virtual void assemble_stiffness();
  virtual void derive_weak();
  virtual void solve();
};

} // namespace cufem::solvers
