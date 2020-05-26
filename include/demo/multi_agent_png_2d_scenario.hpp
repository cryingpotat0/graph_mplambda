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
#include <set>
#include <interpolate.hpp>
#include <randomize.hpp>
#include <jilog.hpp>
#include <demo/app_options.hpp>
#include <prm_planner.hpp>
#include <interval_tree.hpp>

/*
namespace unc::robotics::nigh::metric {
    
    template<>
    struct is_metric<mpl::demo::ReedsSheppStateSpace> : std::true_type {};
    // Here is the template specialization declaration.  It starts
    // with 'template <...>' and is then followed by the template name
    // 'Space'.  Note that we make the specialization operate on
    // templated types and metric parameters.  For an example of this
    // see the test file 'custom_vector_test.cpp'.  In this demo,
    // we're not specializing for just our concrete Vec3 class and in
    // the LP<2> (Euclidean) metric.
    template <class Scalar, class dim>
    struct Space<Eigen::Matrix<Scalar, dim, 1>, mpl::demo::ReedsSheppStateSpace> {

        // These type aliases are required.  Type should be the first
        // template parameter.
        using Type = Eigen::Matrix<Scalar, dim, 1>;
        
        // This should be the second template parameter
        using Metric = mpl::demo::ReedsSheppStateSpace;//LP<2>;

        // Distance alias should be both the result of the distance
        // function and the coefficient type.
        using Distance = double;


        // Declare the dimensions of Vec3 as a constant.  For
        // customizing types that can be of arbitrary dimensions,
        // kDimensions can be set to -1.
        static constexpr int kDimensions = dim;


        // Required method that checks if the vector is valid.  This
        // can be minimally implemented as 'return true;', but adding
        // additional checks for validity will help find invalid data
        // sooner.
        static bool isValid(const Eigen::Matrix<Scalar, dim, 1>& v) {
          for (auto& elem : v) {
            if (!std::isfinite(elem)) return false;
          }
          return true;
        }

        // Required method that returns the coefficient using a
        // 0-based index.  In our example, (x, y, z) are in sequential
        // order, so we can do some pointer tricks to get the
        // zero-based index.  For other data layouts, if/else,
        // switch/case, etc... may be required.
        static double coeff(const Eigen::Matrix<Scalar, dim, 1>& v, std::size_t i) {
            return v[i];
        }

        // This constexpr method should return the number of
        // dimensions of the vector.  This method should match
        // kDimensions, except in the case where kDimensions is -1.
        // Note kDimensions is signed, while dimensions() is unsigned.
        constexpr unsigned dimensions() const {
            return dim;
        }

        // This required method computes and returns the distance
        // between two points.  The computation must match the metric
        // a specified by the second template parameter.  Note: Nigh
        // could implement this function based upon dimensions(), and
        // coeff(), however it is likely that a custom data type can
        // have a much faster implementation (e.g., based upon SIMD
        // instructions).

        static double distance(const Eigen::Matrix<Scalar, dim, 1>& a, const Eigen::Matrix<Scalar, dim, 1>& b) {
          
          mpl::demo::ReedsSheppStateSpace reedsSheppStateSpace(100);
          double length = 0;
          for (int i=0; i < a.size(); i += 3) {
            double q0[3] = {a[i], a[i+1], a[i+2]};
            double q1[3] = {b[i], b[i+1], b[i+2]};
            length += reedsSheppStateSpace.reedsShepp(q0, q1).length();
          }
          return length;
        }
    };
};
*/

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
        static constexpr Scalar agentRadius = 15; // In pixels

    private:
        const int width_;
        const int height_;
        Space space_;
        Bound min_;
        Bound max_;
        State goal_;
        std::vector<bool> isObstacle_;
        static constexpr Scalar PI = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620L;

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

        static bool segmentsDontIntersect(const State& a, const State& b) 
        {
            for (int i=0; i < num_agents; ++i) {
                auto agent_i_start = a.segment(2 * i, 2);
                auto agent_i_end = b.segment(2 * i, 2);
                for (int j=i+1; j < num_agents; ++j) {
                    auto agent_j_start = a.segment(2 * j, 2);
                    auto agent_j_end = b.segment(2 * j, 2);
                    double intersection_x, intersection_y;
                    //if (get_line_intersection(
                    //        agent_i_start[0],
                    //        agent_i_start[1],
                    //        agent_i_end[0],
                    //        agent_i_end[1],
                    //        agent_j_start[0],
                    //        agent_j_start[1],
                    //        agent_j_end[0],
                    //        agent_j_end[1],
                    //        &intersection_x,
                    //        &intersection_y
                    //        )) {
                    //    //JI_LOG(INFO) << "Intersection for " <<
                    //    //        agent_i_start[0] << "," <<
                    //    //        agent_i_start[1]<< ";" <<
                    //    //        agent_i_end[0]<< "," <<
                    //    //        agent_i_end[1]<< ";" <<
                    //    //        agent_j_start[0]<< "," <<
                    //    //        agent_j_start[1]<< ";" <<
                    //    //        agent_j_end[0]<< "," <<
                    //    //        agent_j_end[1]<< ";";
                    //    return false;
                    //}
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

    public:
        inline static std::pair<bool, std::pair<Scalar, Scalar>> get_intersection_time(const SingleAgentState& agent1_xinit, const SingleAgentState& agent1_xfinal, const SingleAgentState& agent2_xinit, const SingleAgentState& agent2_xfinal, 
                                 const Scalar& agent1_tStart, const Scalar& agent1_tEnd, const Scalar& agent2_tStart, const Scalar& agent2_tEnd, const Scalar& intersection_radius, const Scalar& agent_velocity) {
          // Return (true, intersection_time) if agent1 gets within intersection_radius

            auto d_ag1 = (agent1_xfinal - agent1_xinit).normalized();
            auto d_ag2 = (agent2_xfinal - agent2_xinit).normalized();
            auto c = (agent1_xinit - agent2_xinit) + agent_velocity * (-agent1_tStart * d_ag1 + agent2_tStart * d_ag2);
            auto k = agent_velocity * (d_ag1 - d_ag2);
            auto a1 = (k[0] * k[0] + k[1] * k[1]);
            auto a2 = 2 * (c[0] * k[0] + c[1] * k[1]);
            auto a3 = c[0] * c[0] + c[1] * c[1] - intersection_radius * intersection_radius;
            // solve quadratic inequality
            auto [root1, root2] = find_quadratic_roots(a1, a2, a3);
            //JI_LOG(INFO) << root1 << " " << root2;
            if (root1.second > 0 || root2.second > 0 || std::isnan(root1.first) || std::isnan(root2.first)) return std::make_pair(false, std::make_pair(0, 0)); // Imaginary roots mean no intersection
            auto min_val = std::min(root1.first, root2.first);
            auto max_val = std::max(root1.first, root2.first);
            if (min_val > agent1_tEnd || min_val > agent2_tEnd || max_val < agent1_tStart || max_val < agent2_tStart) return std::make_pair(false, std::make_pair(0, 0));
            return std::make_pair(true, std::make_pair(
                    std::max({min_val, agent1_tStart, agent2_tStart}),
                    std::min({max_val, agent1_tEnd, agent2_tEnd})
                  ));
        }

        inline static std::pair<std::pair<Scalar, Scalar>, std::pair<Scalar, Scalar>> find_quadratic_roots(const Scalar& a, const Scalar& b, const Scalar& c) {
          // Return ((root1_real, root1_imaginary), (root2_real, root2_imaginary))

          auto discriminant = b*b - 4*a*c;
          Scalar root1_real = 0, root1_imaginary = 0, root2_real = 0, root2_imaginary = 0;

          if (discriminant > 0) {
            root1_real = (-b + sqrt(discriminant)) / (2*a);
            root2_real = (-b - sqrt(discriminant)) / (2*a);
          }

          else if (discriminant == 0) {
            root1_real = -b / (2*a);
            root2_real = -b / (2*a);
          }

          else {
            root1_real = -b / (2*a);
            root2_real = -b / (2*a);
            root1_imaginary = sqrt(-discriminant)/(2*a);
            root2_imaginary = -sqrt(-discriminant)/(2*a);
          }
          return std::make_pair(
              std::make_pair(root1_real, root1_imaginary), 
              std::make_pair(root2_real, root2_imaginary));
        }

        inline static Scalar get_line_distance(const Scalar& p0_x, const Scalar& p0_y, const Scalar& p1_x, const Scalar& p1_y,
                                 const Scalar& p2_x, const Scalar& p2_y, const Scalar& p3_x, const Scalar& p3_y) {
            Scalar i_x, i_y;
            if (get_line_intersection(p0_x, p0_y, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, &i_x, &i_y)) return 0;
            return std::min(
                  std::min(get_line_point_distance(p0_x, p0_y, p2_x, p2_y, p3_x, p3_y), get_line_point_distance(p1_x, p1_y, p2_x, p2_y, p3_x, p3_y)),
                  std::min(get_line_point_distance(p2_x, p2_y, p0_x, p0_y, p1_x, p1_y), get_line_point_distance(p3_x, p3_y, p0_x, p0_y, p1_x, p1_y)));
        }

        inline static Scalar get_line_point_distance(const Scalar& px, const Scalar& py, const Scalar& x1, const Scalar& y1, const Scalar& x2, const Scalar& y2) {
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
                dx = px - x1 - t * dx;
                dy = py - y1 - t * dy;
            }
            return std::hypot(dx, dy);
        }


        inline static bool get_line_intersection(const Scalar& p0_x, const Scalar& p0_y, const Scalar& p1_x, const Scalar& p1_y,
                                   const Scalar& p2_x, const Scalar& p2_y, const Scalar& p3_x, const Scalar& p3_y, Scalar *i_x, Scalar *i_y) 
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


    private:
        bool validSegment(const State &a, const State &b) const
        {
            return eachSegmentValid(a, b) && segmentsDontIntersect(a, b);
        }

    };

    

    template <class Graph, class Scenario, class Vertex, class Edge, class MultiAgentScenario>
    decltype(auto) sequentialMultiAgentPlanning(Graph& graph, Scenario& scenario, AppOptions& app_options, std::uint64_t global_num_uniform_samples, std::vector<int> agentSequence) {
        //std::vector<int> agentSequence = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

        using State = typename Scenario::State;
        using VertexID = typename Vertex::ID;
        using Scalar = double;
        using MultiAgentState = typename MultiAgentScenario::State;
        Scalar agentVelocity = 100.0;
        std::vector<std::pair<
                std::pair<MultiAgentState, MultiAgentState>,
                std::vector<std::tuple<bool, std::vector<VertexID>, std::vector<Scalar>>> // For each agent: (found, path, velocity_along_edge). Velocity is from start vertex.
        >> pathsFromStartToGoals; // ((start, goal) -> (found, path))
        pathsFromStartToGoals.resize(app_options.getStartsAndGoals<MultiAgentState>().size());

        auto planner = mpl::PRMPlanner<Scenario, Scalar>(scenario, -1); // Use -1 as the standard prefix
        planner.clearVertices(); planner.clearEdges();
        planner.updatePrmRadius(global_num_uniform_samples);
        for (auto& [v_id, connections]: graph.getAdjacencyList()) {
            auto vertex = graph.getVertex(v_id);
            planner.addExistingVertex(vertex);
        }

        int index=0;
        for (auto& [start, goal] : app_options.getStartsAndGoals<MultiAgentState>()) {
            
            auto total_start_time = std::chrono::high_resolution_clock::now();
            auto& [identifier, paths] = pathsFromStartToGoals[index++];
            identifier.first = start;
            identifier.second = goal;
            paths.resize(agentSequence.size());
            //std::set<std::tuple<Scalar, Scalar, Edge>> timeSetOfEdges; // (start_time, end_time, edge) in sorted order
            //IntervalTree<Scalar, Edge> timeSetOfEdges;
            std::vector<Interval<Scalar, Edge>> timeSetOfEdges;
            std::vector<std::pair<Vertex, Vertex>> agentStartAndGoalVertices;
            agentStartAndGoalVertices.resize(agentSequence.size());
            for (auto& agentIndex : agentSequence) {

                State agentStart = start.segment(agentIndex * 2, 2);
                State agentGoal = goal.segment(agentIndex * 2, 2);
                
                // Add start and goal to graph

                auto start_time = std::chrono::high_resolution_clock::now();
                planner.addSample(agentStart);
                auto agentStartVertex = planner.getNewVertices()[0]; // should always exist
                graph.addVertex(agentStartVertex);
                for (auto& e: planner.getNewEdges()) graph.addEdge(e);
                planner.clearVertices(); planner.clearEdges();

                planner.addSample(agentGoal);
                auto agentGoalVertex = planner.getNewVertices()[0]; // should always exist
                graph.addVertex(agentGoalVertex);
                for (auto& e: planner.getNewEdges()) graph.addEdge(e);
                planner.clearVertices(); planner.clearEdges();

                auto end_time = std::chrono::high_resolution_clock::now();
                //JI_LOG(INFO) << "Time to add start " << agentStartVertex.id() << ": " << agentStart
                //    << "; goal " << agentGoalVertex.id() << ": " << agentGoal
                //    << " is " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

                timeSetOfEdges.push_back({-std::numeric_limits<Scalar>::infinity(), 0.0, Edge{0, agentStartVertex.id(), agentStartVertex.id()}});
                agentStartAndGoalVertices[agentIndex] = std::make_pair(agentStartVertex, agentGoalVertex);
            }

            for (auto& agentIndex : agentSequence) {

                //State agentStart = start.segment(agentIndex * 2, 2);
                //State agentGoal = goal.segment(agentIndex * 2, 2);
                //if (agentIndex > 6) continue;
                
                auto& [agentStartVertex, agentGoalVertex] = agentStartAndGoalVertices[agentIndex];

                // Djikstras
                auto start_time = std::chrono::high_resolution_clock::now();
                auto adjacency_list = graph.getAdjacencyList();
                using Distance = typename Edge::Distance_t;
                using DistVertexPair = typename std::pair<Distance, VertexID>;
                std::priority_queue<DistVertexPair, std::vector<DistVertexPair>, std::greater<DistVertexPair>> pq;
                std::unordered_map<VertexID, Distance> dists;
                std::unordered_map<VertexID, VertexID> prev;
                pq.push(std::make_pair(0, agentStartVertex.id()));
                dists[agentStartVertex.id()] = 0;

                IntervalTree<Scalar, Edge> timeIntervalOfEdges(timeSetOfEdges);
                std::vector<VertexID> current_fringe;
                while (!pq.empty()) {
                    auto [curr_dist, u] = pq.top();
                    pq.pop();
                    if (u == agentGoalVertex.id()) break;
                    // TODO: if we find the goal, we still need to ensure that the goal does not intersect with any of the edges for all time after
                    auto outgoing_edges = adjacency_list.find(u);
                    if (outgoing_edges == adjacency_list.end()) continue;
                    current_fringe.push_back(u);
                    for (auto& v : outgoing_edges->second) {
                        auto edge = graph.getEdge(u, v);
                        if (dists.find(v) == dists.end()) dists[v] = std::numeric_limits<Distance>::max();
                        auto agentStartedOnEdgeAt = dists[u] / agentVelocity;
                        

                        if (dists[v] > dists[u] + edge.distance()) { // Sometimes we will want to go back to the goal
                            auto agentEndedOnEdgeAt = (dists[u] + edge.distance()) / agentVelocity;
                            //JI_LOG(INFO) << "Start time " << agentStartedOnEdgeAt << " end time " << agentEndedOnEdgeAt;
                            bool intersects{false};
                            //timeIntervalOfEdges.visit_overlapping(agentStartedOnEdgeAt, agentEndedOnEdgeAt, [&intersects, &edge, &graph, &agentStartVertex, &agentGoalVertex, &agentStartedOn] (const Interval<Scalar, Edge>& intervalAndEdge) {
                            timeIntervalOfEdges.visit_overlapping(agentStartedOnEdgeAt, agentEndedOnEdgeAt, [&] (const Interval<Scalar, Edge>& intervalAndEdge) {
                                    if (intersects) return; // If it already intersects, nothing left to change
                                    // Otherwise check for intersection
                                    auto& other_edge = intervalAndEdge.value;
                                    auto& agent_i_start = graph.getVertex(other_edge.u()).state();
                                    auto& agent_i_end = graph.getVertex(other_edge.v()).state();
                                    auto& agent_j_start = graph.getVertex(u).state();
                                    auto& agent_j_end = graph.getVertex(v).state();
                                    //if (other_edge.distance() == 0) { JI_LOG(INFO) << "Checking intersection against start/ goal"; }
                                    if (other_edge.distance() == 0 && (other_edge.u() == agentStartVertex.id() || other_edge.u() == agentGoalVertex.id()) ) return; // Don't check for collision with my own starts/ goals
                                    //if (MultiAgentScenario::get_line_distance(
                                    //            agent_i_start[0],
                                    //            agent_i_start[1],
                                    //            agent_i_end[0],
                                    //            agent_i_end[1],
                                    //            agent_j_start[0],
                                    //            agent_j_start[1],
                                    //            agent_j_end[0],
                                    //            agent_j_end[1]
                                    //            ) <= MultiAgentScenario::agentRadius * 2) {
                                    //// If the lines intersect in space check for intersection in time

                                      auto [intersects_in_time, intersection_time_interval] = MultiAgentScenario::get_intersection_time(
                                                agent_i_start,
                                                agent_i_end,
                                                agent_j_start,
                                                agent_j_end,
                                                intervalAndEdge.start,
                                                intervalAndEdge.stop,
                                                agentStartedOnEdgeAt,
                                                agentEndedOnEdgeAt,
                                                MultiAgentScenario::agentRadius * 2,
                                                agentVelocity
                                        );
                                      //if (agentIndex == 6) {
                                      //  JI_LOG(INFO) 
                                      //    << "Testing edge intersection "
                                      //    << "agent_i_start " << agent_i_start
                                      //    << " agent_i_end " << agent_i_end
                                      //    << " agent_j_start " << agent_j_start
                                      //    << " agent_j_end " << agent_j_end
                                      //    << " agent_i_tStart " << intervalAndEdge.start
                                      //    << " agent_i_tEnd "  << intervalAndEdge.stop
                                      //    << " agent_i_tEnd "  << agentStartedOnEdgeAt
                                      //    << " agent_i_tEnd "  << agentEndedOnEdgeAt
                                      //    << " intersects " << intersects_in_time
                                      //    << " at " << intersection_time_interval;
                                      //}

                                      if (intersects_in_time) {
                                        intersects = true;
                                      }

                                    //}
                                });
                            //if (!intersects && agentStartedOnEdgeAt == 0.0) {
                            //if (!intersects) {
                                // If it is the first set of edges, check that it does not intersect a start vertex
                                //for (int ind=0; ind < agentStartAndGoalVertices.size(); ++ind) {
                                //    if (ind == agentIndex) continue;
                                //    auto& [otherStartVertex, otherGoalVertex] = agentStartAndGoalVertices[ind];
                                //    //if (otherStartVertex.id() == agentStartVertex.id()) continue;
                                //    
                                //    auto& agent_j_start = graph.getVertex(edge.u()).state();
                                //    auto& agent_j_end = graph.getVertex(edge.v()).state();
                                //    auto agent_i_start = otherStartVertex.state();
                                //    auto agent_i_end = otherGoalVertex.state();
                                //        //JI_LOG(INFO) << "Testing starts " << edge.u() << " " 
                                //        //    << "agent_i_start " << agent_i_start
                                //        //    << " agent_j_start " << agent_j_start
                                //        //    << " agent_j_end " << agent_j_end
                                //        //    << " distance " << MultiAgentScenario::get_line_point_distance(
                                //        //            agent_i_start[0],
                                //        //            agent_i_start[1],
                                //        //            agent_j_start[0],
                                //        //            agent_j_start[1],
                                //        //            agent_j_end[0],
                                //        //            agent_j_end[1]
                                //        //            );
                                //    if (MultiAgentScenario::get_line_point_distance(
                                //                agent_i_start[0],
                                //                agent_i_start[1],
                                //                agent_j_start[0],
                                //                agent_j_start[1],
                                //                agent_j_end[0],
                                //                agent_j_end[1]
                                //                ) <= MultiAgentScenario::agentRadius * 2) {
                                //        intersects = true;
                                //        //JI_LOG(INFO) << "Start interesects";
                                //        break;
                                //    } else {
                                //        //JI_LOG(INFO) << "Start doesnt interesect";
                                //    }
                                //      //JI_LOG(INFO) << "Testing goals " << edge.u() << " " 
                                //      //    << "agent_i_end " << agent_i_end
                                //      //    << " agent_j_start " << agent_j_start
                                //      //    << " agent_j_end " << agent_j_end
                                //      //    << " distance " << MultiAgentScenario::get_line_point_distance(
                                //      //            agent_i_end[0],
                                //      //            agent_i_end[1],
                                //      //            agent_j_start[0],
                                //      //            agent_j_start[1],
                                //      //            agent_j_end[0],
                                //      //            agent_j_end[1]
                                //      //            );

                                //    if (MultiAgentScenario::get_line_point_distance(
                                //                agent_i_end[0],
                                //                agent_i_end[1],
                                //                agent_j_start[0],
                                //                agent_j_start[1],
                                //                agent_j_end[0],
                                //                agent_j_end[1]
                                //                ) <= MultiAgentScenario::agentRadius * 2) {
                                //        intersects = true;
                                //        //JI_LOG(INFO) << "Goal interesects";
                                //        break;
                                //    } else {
                                //        //JI_LOG(INFO) << "Goal doesnt interesect";
                                //    }

                                //}
                            //}
                            if (!intersects)  {
                                dists[v] = dists[u] + edge.distance();
                                prev[v] = u;
                                pq.push(std::make_pair(dists[v], v));
                                current_fringe.clear();
                            }
                        }
                    }
                }
                if (prev.find(agentGoalVertex.id()) == prev.end()) {
                    paths[agentIndex] = std::make_tuple(false, current_fringe, std::vector<Scalar>());
                    //JI_LOG(INFO) << "No Path found for agent " <<  agentIndex << " with distance inf";
                } else {
                    auto curr = agentGoalVertex.id();
                    std::vector<VertexID> path;
                    std::vector<Scalar> velocities;
                    velocities.push_back(0); // At goal 0 velocity
                    while (curr != agentStartVertex.id()) {
                        velocities.push_back(agentVelocity);
                        path.push_back(curr);
                        curr = prev[curr];
                    }
                    path.push_back(agentStartVertex.id());
                    std::reverse(path.begin(), path.end());
                    std::reverse(velocities.begin(), velocities.end());

                    Scalar agentCurrentTime = 0.0;
                    Scalar pathDistance = 0.0;
                    for (int i=0; i < path.size() - 1; ++i) {
                        auto edge = graph.getEdge(path[i], path[i+1]);
                        pathDistance += edge.distance();
                        auto velocity = velocities[i];
                        Scalar agentEndTime = agentCurrentTime + edge.distance() / velocity;
                        timeSetOfEdges.push_back({agentCurrentTime, agentEndTime, edge});
                        agentCurrentTime = agentEndTime;
                    }
                    timeSetOfEdges.push_back({agentCurrentTime, std::numeric_limits<Scalar>::infinity(), Edge{0, agentGoalVertex.id(), agentGoalVertex.id()}});
                    paths[agentIndex] = std::make_tuple(true, path, velocities);
                    //JI_LOG(INFO) << "Path found for agent " <<  agentIndex << " with distance " << pathDistance;

                }
                auto end_time = std::chrono::high_resolution_clock::now();
                //JI_LOG(INFO) << "Time to do Djikstras for agent " << agentIndex 
                //    << " is " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            }
            auto total_end_time = std::chrono::high_resolution_clock::now();
            JI_LOG(INFO) << "Time to compute paths is " << std::chrono::duration_cast<std::chrono::milliseconds>(total_end_time - total_start_time);
        }
        return pathsFromStartToGoals;
    }


            /*
    template <typename Scalar, int num_agents>
    class MultiAgentDubinsPNG2DScenario {
    public:
        using Space = unc::robotics::nigh::metric::L2Space<Scalar, num_agents * 2>; // x,y,vx,vy
        using State = typename Space::Type;
        using SingleAgentSpace = unc::robotics::nigh::metric::L2Space<Scalar, 2>;
        using SingleAgentState = typename SingleAgentSpace::Type;
        using Bound = State;
        using Distance = typename Space::Distance;
        static constexpr Scalar agentRadius = 15; // In pixels

    private:
        const int width_;
        const int height_;
        Space space_;
        Bound min_;
        Bound max_;
        State goal_;
        std::vector<bool> isObstacle_;
        static constexpr Scalar PI = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620L;

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

        static bool segmentsDontIntersect(const State& a, const State& b) 
        {
            for (int i=0; i < num_agents; ++i) {
                auto agent_i_start = a.segment(2 * i, 2);
                auto agent_i_end = b.segment(2 * i, 2);
                for (int j=i+1; j < num_agents; ++j) {
                    auto agent_j_start = a.segment(2 * j, 2);
                    auto agent_j_end = b.segment(2 * j, 2);
                    double intersection_x, intersection_y;
                    //if (get_line_intersection(
                    //        agent_i_start[0],
                    //        agent_i_start[1],
                    //        agent_i_end[0],
                    //        agent_i_end[1],
                    //        agent_j_start[0],
                    //        agent_j_start[1],
                    //        agent_j_end[0],
                    //        agent_j_end[1],
                    //        &intersection_x,
                    //        &intersection_y
                    //        )) {
                    //    //JI_LOG(INFO) << "Intersection for " <<
                    //    //        agent_i_start[0] << "," <<
                    //    //        agent_i_start[1]<< ";" <<
                    //    //        agent_i_end[0]<< "," <<
                    //    //        agent_i_end[1]<< ";" <<
                    //    //        agent_j_start[0]<< "," <<
                    //    //        agent_j_start[1]<< ";" <<
                    //    //        agent_j_end[0]<< "," <<
                    //    //        agent_j_end[1]<< ";";
                    //    return false;
                    //}
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

    public:
        inline static std::pair<bool, std::pair<Scalar, Scalar>> get_intersection_time(const SingleAgentState& agent1_xinit, const SingleAgentState& agent1_xfinal, const SingleAgentState& agent2_xinit, const SingleAgentState& agent2_xfinal, 
                                 const Scalar& agent1_tStart, const Scalar& agent1_tEnd, const Scalar& agent2_tStart, const Scalar& agent2_tEnd, const Scalar& intersection_radius, const Scalar& agent_velocity) {
          // Return (true, intersection_time) if agent1 gets within intersection_radius

            auto d_ag1 = (agent1_xfinal - agent1_xinit).normalized();
            auto d_ag2 = (agent2_xfinal - agent2_xinit).normalized();
            auto c = (agent1_xinit - agent2_xinit) + agent_velocity * (-agent1_tStart * d_ag1 + agent2_tStart * d_ag2);
            auto k = agent_velocity * (d_ag1 - d_ag2);
            auto a1 = (k[0] * k[0] + k[1] * k[1]);
            auto a2 = 2 * (c[0] * k[0] + c[1] * k[1]);
            auto a3 = c[0] * c[0] + c[1] * c[1] - intersection_radius * intersection_radius;
            // solve quadratic inequality
            auto [root1, root2] = find_quadratic_roots(a1, a2, a3);
            //JI_LOG(INFO) << root1 << " " << root2;
            if (root1.second > 0 || root2.second > 0 || std::isnan(root1.first) || std::isnan(root2.first)) return std::make_pair(false, std::make_pair(0, 0)); // Imaginary roots mean no intersection
            auto min_val = std::min(root1.first, root2.first);
            auto max_val = std::max(root1.first, root2.first);
            if (min_val > agent1_tEnd || min_val > agent2_tEnd || max_val < agent1_tStart || max_val < agent2_tStart) return std::make_pair(false, std::make_pair(0, 0));
            return std::make_pair(true, std::make_pair(
                    std::max({min_val, agent1_tStart, agent2_tStart}),
                    std::min({max_val, agent1_tEnd, agent2_tEnd})
                  ));
        }

        inline static std::pair<std::pair<Scalar, Scalar>, std::pair<Scalar, Scalar>> find_quadratic_roots(const Scalar& a, const Scalar& b, const Scalar& c) {
          // Return ((root1_real, root1_imaginary), (root2_real, root2_imaginary))

          auto discriminant = b*b - 4*a*c;
          Scalar root1_real = 0, root1_imaginary = 0, root2_real = 0, root2_imaginary = 0;

          if (discriminant > 0) {
            root1_real = (-b + sqrt(discriminant)) / (2*a);
            root2_real = (-b - sqrt(discriminant)) / (2*a);
          }

          else if (discriminant == 0) {
            root1_real = -b / (2*a);
            root2_real = -b / (2*a);
          }

          else {
            root1_real = -b / (2*a);
            root2_real = -b / (2*a);
            root1_imaginary = sqrt(-discriminant)/(2*a);
            root2_imaginary = -sqrt(-discriminant)/(2*a);
          }
          return std::make_pair(
              std::make_pair(root1_real, root1_imaginary), 
              std::make_pair(root2_real, root2_imaginary));
        }

        inline static Scalar get_line_distance(const Scalar& p0_x, const Scalar& p0_y, const Scalar& p1_x, const Scalar& p1_y,
                                 const Scalar& p2_x, const Scalar& p2_y, const Scalar& p3_x, const Scalar& p3_y) {
            Scalar i_x, i_y;
            if (get_line_intersection(p0_x, p0_y, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, &i_x, &i_y)) return 0;
            return std::min(
                  std::min(get_line_point_distance(p0_x, p0_y, p2_x, p2_y, p3_x, p3_y), get_line_point_distance(p1_x, p1_y, p2_x, p2_y, p3_x, p3_y)),
                  std::min(get_line_point_distance(p2_x, p2_y, p0_x, p0_y, p1_x, p1_y), get_line_point_distance(p3_x, p3_y, p0_x, p0_y, p1_x, p1_y)));
        }

        inline static Scalar get_line_point_distance(const Scalar& px, const Scalar& py, const Scalar& x1, const Scalar& y1, const Scalar& x2, const Scalar& y2) {
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
                dx = px - x1 - t * dx;
                dy = py - y1 - t * dy;
            }
            return std::hypot(dx, dy);
        }


        inline static bool get_line_intersection(const Scalar& p0_x, const Scalar& p0_y, const Scalar& p1_x, const Scalar& p1_y,
                                   const Scalar& p2_x, const Scalar& p2_y, const Scalar& p3_x, const Scalar& p3_y, Scalar *i_x, Scalar *i_y) 
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


    private:
        bool validSegment(const State &a, const State &b) const
        {
            return eachSegmentValid(a, b) && segmentsDontIntersect(a, b);
        }

    };
    */

};

#endif //MPLAMBDA_MULTI_AGENT_PNG_2D_SCENARIO_HPP
