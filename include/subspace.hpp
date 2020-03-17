
#ifndef MPL_SUBSPACE_HPP
#define MPL_SUBSPACE_HPP

#include <vector>
#include <iostream>

namespace mpl {
    template <class Bound, class State, class Scalar>
    class Subspace {
    private:
        Bound lower;
        Bound upper;
        Subspace();
    public:
        Subspace(Bound lower, Bound upper) {
            this->lower = lower;
            this->upper = upper;
        }

        std::vector<Subspace> divide(std::vector<int> num_divisions) {

        }

        std::vector<Subspace> divide_until(int num_objects) {

        }

        std::vector<Subspace> get_neighbors(Subspace global_bounds) {

        }

        std::vector<Subspace> point_near_neighbors(State point, Subspace global_bounds, Scalar radius) {

        }
    };
}


template <class Bound, class State, class Scalar>
std::ostream &operator<<(std::ostream &os, mpl::Subspace<Bound, State, Scalar> const &subspace) {
    return os << "l:" << subspace.lower << "u:" << subspace.upper;
}

#endif
