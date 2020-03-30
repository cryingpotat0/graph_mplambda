
#ifndef MPL_SUBSPACE_HPP
#define MPL_SUBSPACE_HPP

#include <vector>
#include <iostream>
#include <tree.hpp>

namespace mpl {
    template <class Bound, class State, class Scalar>
    class Subspace {
    private:
        Bound lower;
        Bound upper;
//        using Subspace_t = Subspace<Bound, State, Scalar>;

    public:
        Subspace(Bound lower_, Bound upper_) :
                lower(lower_),
                upper(upper_)
        {
            assert(lower.size() == upper.size());
        }

        Bound getLower() const {
            return lower;
        }

        Bound getUpper() const {
            return upper;
        }

        int dimension() const {
            return lower.size();
        };

        std::vector<Subspace> divide(const std::vector<int>& num_divisions) {
            // TODO
            std::vector<Subspace> subspaces;
            subspaces.emplace_back(Subspace(lower, upper));
            for (auto i=0; i < num_divisions.size(); ++i) {
                auto nd = num_divisions[i];
                std::vector<Subspace> new_subspaces;
                while (subspaces.size() > 0) {
                    auto curr_subspace = subspaces.back();
                    double increment = (curr_subspace.getUpper()[i] - curr_subspace.getLower()[i]) / (nd + 1);
                    for (int j=0; j <= nd; ++j) {
                        auto new_lower = curr_subspace.getLower();
                        auto new_upper = curr_subspace.getUpper();
                        new_lower[i] += increment * j;
                        new_upper[i] = new_lower[i] + increment;
                        new_subspaces.push_back({new_lower, new_upper});
                    }
                    subspaces.pop_back();
                }
                subspaces = new_subspaces;
            }
            return subspaces;
        }

        Tree<Subspace> layered_divide(const std::vector<int> &num_divisions) {
            Tree<Subspace> root{Subspace(lower, upper)};
            std::vector<Tree<Subspace> *> current_layer {&root};
            for (auto i=0; i < num_divisions.size(); ++i) {
                auto nd = num_divisions[i];
                std::vector<Tree<Subspace>*> new_layer;
                for (auto node : current_layer) {
                    // split each child num_division times
                    double increment = (node->getData().getUpper()[i] - node->getData().getLower()[i]) / (nd + 1);
                    for (int j=0; j <= nd; ++j) {
                        auto new_lower = node->getData().getLower();
                        auto new_upper = node->getData().getUpper();
                        new_lower[i] += increment * j;
                        new_upper[i] = new_lower[i] + increment;
                        Subspace child(new_lower, new_upper);
                        node->addChild(child);
                    }
                }
                for (auto node : current_layer) {
                    for (auto c : node->getChildren()) {
                        new_layer.push_back(c);
                        c->changed = true;
                        new_layer.size();
                    }
                }
                current_layer.clear();
                for (auto c : new_layer) { current_layer.emplace_back(c); } // TODO: I feel like I'm overcomplicating this too much
//                current_layer = new_layer;
            }
            return root;
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


namespace mpl::util {
    const static Eigen::IOFormat CommaInitFormat(
            Eigen::StreamPrecision, Eigen::DontAlignCols,
            ", ", ",",
            "", "", "", ";");

}

template <class Bound, class State, class Scalar>
std::ostream &operator<<(std::ostream &os, const mpl::Subspace<Bound, State, Scalar> &subspace) {
    return os << "l:" << subspace.getLower().format(mpl::util::CommaInitFormat)
            << "u:" << subspace.getUpper().format(mpl::util::CommaInitFormat);
}

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
    for (auto it : v) {
        out << it << std::endl;
    }
    return out;
}


namespace std {
    template <class Bound, class State, class Scalar>
    struct hash<mpl::Subspace<Bound, State, Scalar> > {
        size_t operator()(const mpl::Subspace<Bound, State, Scalar>& s) const noexcept {
            return 0; //std::hash<Bound>(s.getLower()) ^ std::hash<Bound>(s.getUpper()); // TODO: implement good hash
        }
    };
}

#endif
