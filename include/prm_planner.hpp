#ifndef MPL_PRM_PLANNER_HPP
#define MPL_PRM_PLANNER_HPP
#include <subspace.hpp>
#include <vector>

namespace mpl {
    template <class Scenario, class Scalar>
    class PRMPlanner {
    private:
        using State = typename Scenario::State;
        using Bound = typename Scenario::Bound;
        using Subspace_t = typename mpl::Subspace<Bound, State, Scalar>;
        Scenario scenario;
        void* lambda; // TODO: extremely hacky, have to resolve the circular dependence here
        std::vector<void (*) (State, void*)> validSampleCallbacks;

    public:
        explicit PRMPlanner(Scenario scenario_, void* lambda_)
                : scenario(scenario_),
                  lambda(lambda_)
        {}
//      PRMPlanner(Scenario scenario, Subspace_t local_subspace, Subspace_t global_subspace) {
//        this->scenario = scenario;
//        this->local_subspace = local_subspace;
//        this->global_subspace = global_subspace;
//      }
        void addValidSampleCallback(void (*f)(State, void*)) {
            validSampleCallbacks.push_back(f);
        }

    };
}

#endif
