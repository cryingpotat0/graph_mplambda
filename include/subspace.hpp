
#ifndef MPL_SUBSPACE_HPP
#define MPL_SUBSPACE_HPP

#include <vector>

namespace mpl {
  template <class Bound, class State, class Scalar>
  class Subspace {
    private:
      Subspace();
    public:
      Subspace(Bound lower, Bound upper) {

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
#endif
