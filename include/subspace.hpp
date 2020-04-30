
#ifndef MPL_SUBSPACE_HPP
#define MPL_SUBSPACE_HPP

#include <vector>
#include <iostream>
#include <util.hpp>

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

        const Bound& getLower() const {
            return lower;
        }

        const Bound& getUpper() const {
            return upper;
        }

        const int dimension() const {
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

        //std::pair<Tree<Subspace>, Tree<Subspace>*> layered_divide(const std::vector<int> &num_divisions, const Subspace* find=nullptr) {
        //    Tree<Subspace> root{Subspace(lower, upper)};
        //    std::vector<Tree<Subspace> *> current_layer {&root};
        //    Tree<Subspace> *found = nullptr;
        //    for (auto i=0; i < num_divisions.size(); ++i) {
        //        auto nd = num_divisions[i];
        //        std::vector<Tree<Subspace>*> new_layer;
        //        for (auto node : current_layer) {
        //            // split each child num_division times
        //            double increment = (node->getData().getUpper()[i] - node->getData().getLower()[i]) / (nd + 1);
        //            for (int j=0; j <= nd; ++j) {
        //                auto new_lower = node->getData().getLower();
        //                auto new_upper = node->getData().getUpper();
        //                new_lower[i] += increment * j;
        //                new_upper[i] = new_lower[i] + increment;
        //                Subspace child(new_lower, new_upper);
        //                node->addChild(child);
        //                if (i == num_divisions.size() - 1) JI_LOG(INFO) << child;
        //            }
        //        }
        //        for (auto node : current_layer) {
        //            for (auto c : node->getChildren()) {
        //                new_layer.push_back(c);
        //                if (c->getData() == (*find)) {
        //                    found = c;
        //                }
        //            }
        //        }
        //        current_layer.clear();
        //        for (auto c : new_layer) {
        //            current_layer.emplace_back(c);
        //        } // TODO: I feel like I'm overcomplicating this too much
//      //          current_layer = new_layer;
        //    }
        //    return std::make_pair(root, found);
        //}

        std::vector<Subspace> divide_until(int num_objects) {
            // TODO
            return std::vector<Subspace>();
        }

        std::vector<Subspace> get_neighbors(Subspace& global_bounds) {
            // TODO
            std::vector<Subspace> neighbors{*this};
            for (int i=0; i < dimension(); ++i) {
                Scalar increment = upper[i] - lower[i];
                std::vector<Subspace> new_neighbors;
                for (auto &n: neighbors) {
                    auto curr_lower = n.getLower();
                    auto curr_upper = n.getUpper();
                    curr_lower[i] += increment;
                    curr_upper[i] += increment;
                    Subspace incr(curr_lower, curr_upper);
                    if (global_bounds.contains(incr)) new_neighbors.push_back(std::move(incr));

                    curr_lower = n.getLower();
                    curr_upper = n.getUpper();
                    curr_lower[i] -= increment;
                    curr_upper[i] -= increment;
                    Subspace decr(curr_lower, curr_upper);
                    if (global_bounds.contains(decr)) new_neighbors.push_back(std::move(decr));

                    new_neighbors.push_back(std::move(n));
                }
                neighbors.clear();
                neighbors = std::move(new_neighbors);
            }
            return neighbors;
        }

        bool contains(const Subspace& other) const {
            assert(other.dimension() == dimension());
            auto other_lower = other.getLower();
            auto other_upper = other.getUpper();
            for (int i=0; i < dimension(); ++i) {
                if ((other_lower[i] < lower[i]) || (other_upper[i] > upper[i])) return false;
            }
            return true;
        }

        bool contains(const State& point) const {
            assert(point.size() == dimension());
            for (int i=0; i < dimension(); ++i) {
                if ((point[i] < lower[i]) || (point[i] > upper[i])) return false;
            }
            return true;
        }

        std::vector<Subspace> point_near_neighbors(const State& point,
                const Subspace& global_bounds, Scalar radius,
                const std::vector<Subspace>& neighbors) {
            // TODO
            return std::vector<Subspace>();
        }

        bool is_within(Scalar radius, const State& point) const {
            Scalar dist = 0;
            auto middle = (getLower() + getUpper()) / 2.0;
            auto shifted_upper = getUpper() - middle;
            auto shifted_point = point - middle;
            JI_LOG(INFO) << "point " << point << " upper " << getUpper() << " shifted_upper " << shifted_upper << " shifted_point " << shifted_point;

            for (int i=0; i < dimension(); ++i) {
                dist += pow(std::max(0.0, std::abs(shifted_point[i]) - shifted_upper[i]), 2);
                //dist += std::pow(shifted_point[i] , 2);
            }
            return dist <= pow(radius, 2);
        }

        bool operator==(Subspace const& other) const {
	    assert (other.dimension() == dimension());
        for (int i=0; i < dimension(); ++i) {
            if ((!double_equal(lower[i], other.lower[i])) || 
                    (!double_equal(upper[i], other.upper[i]))) {
            return false;
        }
	    }
	    return true;
            //return other.getLower() == lower && other.getUpper() == upper;
        }
	
	bool double_equal(const double& a, const double& b, const double TOLERANCE=0.01) const {
		return std::abs(a-b) < TOLERANCE;
	}

        bool operator!=(Subspace const& other) const {
            return !((*this) == other);
        }
    };

}


template <class Bound, class State, class Scalar>
std::ostream &operator<<(std::ostream &os, const mpl::Subspace<Bound, State, Scalar> &subspace) {
    return os << "l:" << subspace.getLower().format(mpl::util::CommaInitFormat)
            << "u:" << subspace.getUpper().format(mpl::util::CommaInitFormat);
}

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
    for (auto it : v) {
        out << it << ";";
    }
    return out;
}



//namespace std {
//    template <class Bound, class State, class Scalar>
//    struct hash<mpl::Subspace<Bound, State, Scalar> > {
//        size_t operator()(const mpl::Subspace<Bound, State, Scalar>& s) const noexcept {
//            return 0; //std::hash<Bound>(s.getLower()) ^ std::hash<Bound>(s.getUpper()); // TODO: implement good hash
//        }
//    };
//}

#endif
