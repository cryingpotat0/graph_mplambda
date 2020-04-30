//
// Created by Raghav Anand on 4/26/20.
//

#ifndef MPLAMBDA_MULTI_AGENT_PNG_2D_SCENARIO_HPP
#define MPLAMBDA_MULTI_AGENT_PNG_2D_SCENARIO_HPP

#include <png.h>
#include <Eigen/Dense>
#include <demo/shape_hierarchy.hpp>
#include <nigh/se3_space.hpp>
#include <vector>
#include <interpolate.hpp>
#include <randomize.hpp>
#include <jilog.hpp>

namespace mpl::demo {
    template <typename Scalar, int num_agents>
    class MultiAgentPNG2DScenario {
    public:
        using Space = unc::robotics::nigh::metric::L2Space<Scalar, num_agents * 2>; // x,y,vx,vy
        using State = typename Space::Type;
        using SingleAgentSpace = unc::robotics::nigh::metric::L2Space<Scalar, 2>;
        using SingleAgentState = typename SingleAgentSpace::Type;
        using Bound = State;
        using Distance = typename Space::Distance;

    private:
        const int width_;
        const int height_;
        Space space_;
        Bound min_;
        Bound max_;
        State goal_;
        std::vector<bool> isObstacle_;
        static constexpr Scalar PI = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620L;
        static constexpr Scalar agentRadius = 20; // In pixels

    public:
        MultiAgentPNG2DScenario(
                const int width,
                const int height,
                Bound min,
                Bound max,
                std::vector<bool> &isObstacle
        )
                : width_(width),
                  height_(height),
                  min_(min),
                  max_(max),
                  isObstacle_(isObstacle)
        {
        }

        const Distance maxSteering() const {
            return std::numeric_limits<Distance>::infinity();
        }

        bool isValidSingle(const SingleAgentState& q) const {
            int x = std::floor(q[0]);
            int y = std::floor(q[1]);

            return !isObstacle_[width_ * y + x];
        }

        bool isValid(const State &q) const
        {
            for (int i=0; i < num_agents; ++i) {
                auto curr = q.segment(2 * i, 2);
                //curr << q[2*i], q[2*i+1];
                if (!isValidSingle(curr)) return false;
            }
            for (int i=0; i < num_agents; ++i) {
                auto curr_i = q.segment(2*i, 2);
                for (int j=i+1; j < num_agents; ++j) {
                    auto curr_j = q.segment(2*j, 2);
                    if ((curr_j - curr_i).norm() < agentRadius * 2) return false;
                }
            }
            return true;
        }

        bool isValid(const State &a, const State &b) const
        {
            if(!isValid(a) || !isValid(b))
                return false;
            return validSegment(a, b);
        }

        static const int dimension() {
            return num_agents * 2;
        }

        const Space &space() const
        {
            return space_;
        }

        const Bound &min() const
        {
            return min_;
        }

        const Bound &max() const
        {
            return max_;
        }

        void setMin(const Bound &min) {
            min_ = min;
        }

        void setMax(const Bound &max) {
            max_ = max;
        }

        void setGoal(const State& q) {
            goal_ = q;
        }

        bool isGoal(const State& q) const {
            return (goal_ - q).isMuchSmallerThan(30, 1); // Tolerance of 5 pixels each way
        }

        const int width() const
        {
            return width_;
        }

        const int height() const
        {
            return height_;
        }

        static State scale(const State& q)
        {
            return q;
        }

        template <class RNG>
        State randomSample(RNG& rng)
        {
            State q;
            randomize(q, rng, min_, max_);
            return q;
        }

        Scalar prmRadius()
        {
            auto final_radius = 2 * pow(pow(width_ * height_, num_agents) / sphere_volume() * (1.0 + 1.0 / dimension()), 1.0 / dimension());
            return final_radius;
        }

    private:
        Scalar sphere_volume()
        {
            // Volume of a unit sphere
            double dim_over_2 = (double) dimension() / 2.0;
            return pow(PI, dim_over_2) / std::tgamma(dim_over_2 + 1);
        }

        bool eachSegmentValid(const State &a, const State &b) const
        {
            // uses the bisection method to verify links.
            // first check that each line is valid
            State mid = (a + b) / 2;
            Scalar distSquared = (b - a).squaredNorm();
            Scalar tolerance = 1;
            if (distSquared < tolerance * tolerance)
                return true;
            if (!isValid(mid))
                return false;
            if (!eachSegmentValid(a, mid)) // check the left half
                return false;
            return eachSegmentValid(mid, b); // check the right half
        }

        bool segmentsDontIntersect(const State& a, const State& b) const
        {
            for (int i=0; i < num_agents; ++i) {
                auto agent_i_start = a.segment(2 * i, 2);
                auto agent_i_end = b.segment(2 * i, 2);
                for (int j=i+1; j < num_agents; ++j) {
                    auto agent_j_start = a.segment(2 * j, 2);
                    auto agent_j_end = b.segment(2 * j, 2);
                    double intersection_x, intersection_y;
                    if (get_line_intersection(
                            agent_i_start[0],
                            agent_i_start[1],
                            agent_i_end[0],
                            agent_i_end[1],
                            agent_j_start[0],
                            agent_j_start[1],
                            agent_j_end[0],
                            agent_j_end[1],
                            &intersection_x,
                            &intersection_y
                            )) {
                        //JI_LOG(INFO) << "Intersection for " <<
                        //        agent_i_start[0] << "," <<
                        //        agent_i_start[1]<< ";" <<
                        //        agent_i_end[0]<< "," <<
                        //        agent_i_end[1]<< ";" <<
                        //        agent_j_start[0]<< "," <<
                        //        agent_j_start[1]<< ";" <<
                        //        agent_j_end[0]<< "," <<
                        //        agent_j_end[1]<< ";";
                        return false;
                    }
                    if (get_line_distance(
                            agent_i_start[0],
                            agent_i_start[1],
                            agent_i_end[0],
                            agent_i_end[1],
                            agent_j_start[0],
                            agent_j_start[1],
                            agent_j_end[0],
                            agent_j_end[1]
                            ) <= agentRadius * 2) {
                        return false;
                    }
                }
            }
            return true;
        }

        Scalar get_line_distance(Scalar p0_x, Scalar p0_y, Scalar p1_x, Scalar p1_y,
                                 Scalar p2_x, Scalar p2_y, Scalar p3_x, Scalar p3_y) const {
            return std::min(
                    std::min(get_line_point_distance(p0_x, p0_y, p2_x, p2_y, p3_x, p3_y), get_line_point_distance(p1_x, p1_y, p2_x, p2_y, p3_x, p3_y)),
                    std::min(get_line_point_distance(p2_x, p2_y, p0_x, p0_y, p1_x, p1_y), get_line_point_distance(p3_x, p3_y, p0_x, p0_y, p1_x, p1_y)));
        }

        Scalar get_line_point_distance(Scalar px, Scalar py, Scalar x1, Scalar y1, Scalar x2, Scalar y2) const {
            auto dx = x2-x1, dy=y2-y1;
            if (dx == 0 && dy == 0) {
                return std::hypot(px-x1, px-y1);
            }
            auto t = ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy);
            if (t < 0) {
                dx = px - x1;
                dy = py - y1;
            } else if (t > 1) {
                dx = px - x2;
                dy = py - y2;
            } else {
                dx = px - x1 + t * dx;
                dy = py - y1 + t * dy;
            }
            return std::hypot(dx, dy);
        }


        bool get_line_intersection(Scalar p0_x, Scalar p0_y, Scalar p1_x, Scalar p1_y,
                                   Scalar p2_x, Scalar p2_y, Scalar p3_x, Scalar p3_y, Scalar *i_x, Scalar *i_y) const
        {
            // From: https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
            Scalar s1_x, s1_y, s2_x, s2_y;
            s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
            s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

            Scalar s, t;
            s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
            t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

            if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
            {
                // Collision detected
                if (i_x != NULL)
                    *i_x = p0_x + (t * s1_x);
                if (i_y != NULL)
                    *i_y = p0_y + (t * s1_y);
                return true;
            }

            return false; // No collision

        }


        bool validSegment(const State &a, const State &b) const
        {
            return eachSegmentValid(a, b) && segmentsDontIntersect(a, b);
        }

    };
};

#endif //MPLAMBDA_MULTI_AGENT_PNG_2D_SCENARIO_HPP
