
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
        Subspace(Bound lower_, Bound upper_) :
                lower(lower_),
                upper(upper_)
        { }

        Bound getLower() const {
            return lower;
        }

        Bound getUpper() const {
            return upper;
        }

        std::vector<Subspace> divide(std::vector<int> num_divisions) {
            // TODO
            return std::vector<Subspace>();
        }

        std::vector<Subspace> divide_until(int num_objects) {
            // TODO
            return std::vector<Subspace>();
        }

        std::vector<Subspace> get_neighbors(Subspace global_bounds) {
            // TODO
            return std::vector<Subspace>();
        }

        std::vector<Subspace> point_near_neighbors(State point, Subspace global_bounds, Scalar radius) {
            // TODO
            return std::vector<Subspace>();
        }

        bool operator==(Subspace const& other) const {
            return other.lower == lower && other.upper == upper;
        }
    };
}


template <class Bound, class State, class Scalar>
std::ostream &operator<<(std::ostream &os, const mpl::Subspace<Bound, State, Scalar> &subspace) {
    return os << "l:" << subspace.getLower() << ",u:" << subspace.getUpper();
}

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
    out << '[';
    for (auto it : v) {
        out << it << ";";
    }
    out << "]";
    return out;
}


namespace std {
    template <class Bound, class State, class Scalar>
    struct hash<mpl::Subspace<Bound, State, Scalar> > {
        // TODO
        size_t operator()(const mpl::Subspace<Bound, State, Scalar>& ) const noexcept {
            return 0;
        }
    };
}

#endif
