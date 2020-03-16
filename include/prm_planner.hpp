#ifndef MPL_PRM_PLANNER_HPP
#define MPL_PRM_PLANNER_HPP
#include <subspace.hpp>

namespace mpl {
  template <class Scenario>
  class PRMPlanner {
    using State = typename Scenario::State;
    using Bound = typename Scenario::Bound;
    using Scalar = typename Scenario::Scalar;
    using Subspace_t = typename mpl::Subspace<Bound, State, Scalar>;

    public:
      PRMPlanner(Scenario scenario, Subspace_t local_subspace, Subspace_t global_subspace) {
        this->scenario = scenario;
        this->local_subspace = local_subspace;
        this->global_subspace = global_subspace;
      }
  };
}

#endif
