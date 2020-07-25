//
// Created by Raghav Anand on 2020-03-23.
//

#ifndef MPLAMBDA_APP_OPTIONS_HPP
#define MPLAMBDA_APP_OPTIONS_HPP

#include <string>
#include <optional>
#include <Eigen/Dense>
#include <iostream>
#include <getopt.h>
#include <jilog.hpp>

namespace mpl::demo {
    template <class T>
    struct OptionParser;

    template <>
    struct OptionParser<float> {
        static float parse(const std::string& name, const char *arg, char **endp)
        {
            if (!*arg)
                throw std::invalid_argument(name + " is required");

            float v = std::strtof(arg, endp);
            if (*endp == arg)
                throw std::invalid_argument("bad value for " + name + ": " + arg);
            return v;
        }
    };

    template <>
    struct OptionParser<const char *> {
        static const char * parse(const std::string& name, const char *arg, char **endp)
        {
            if (!*arg)
                throw std::invalid_argument(name + " is required");

            return arg;
        }
    };

    template <>
    struct OptionParser<double> {
        static double parse(const std::string& name, const char *arg, char **endp)
        {
            if (!*arg)
                throw std::invalid_argument(name + " is required");

            float v = std::strtod(arg, endp);
            if (*endp == arg)
                throw std::invalid_argument("bad value for " + name + ": " + arg);
            return v;
        }
    };

    template <class S, int rows>
    struct OptionParser<Eigen::Matrix<S, rows, 1>> {
        static Eigen::Matrix<S, rows, 1> parse(const std::string& name, const char *arg, char **endp) {
            if (!*arg)
                throw std::invalid_argument(name + " is required");

            Eigen::Matrix<S, rows, 1> q;
            q[0] = OptionParser<S>::parse(name, arg, endp);
            for (int i=1 ; i<rows ; ++i) {
                if (**endp != ',')
                    throw std::invalid_argument("expected comma");
                q[i] = OptionParser<S>::parse(name, *endp + 1, endp);
            }
            return q;
        }
    };

    template <class S>
    struct OptionParser<Eigen::Quaternion<S>> {
        static Eigen::Quaternion<S> parse(const std::string& name, const char *arg, char **endp) {
            auto v = OptionParser<Eigen::Matrix<S, 4, 1>>::parse(name, arg, endp);
            // Eigen::Quaternion<S> q;
            // q = Eigen::AngleAxis<S>{v[0], v.template tail<3>().normalized()};
            // return q;
            return Eigen::Quaternion<S>{v};
        }
    };

    template <class A, class B>
    struct OptionParser<std::tuple<A, B>> {
        static std::tuple<A, B> parse(const std::string& name, const char *arg, char **endp) {
            A a = OptionParser<A>::parse(name, arg, endp);
            if (**endp != ',')
                throw std::invalid_argument("expected comma");
            return { a, OptionParser<B>::parse(name, *endp + 1, endp) };
        }
    };

    template <class S, int mode>
    struct OptionParser<Eigen::Transform<S, 3, mode>> {
        using Result = Eigen::Transform<S, 3, mode>;
        static Result parse(const std::string& name, const char *arg, char **endp) {
            if (!*arg)
                throw std::invalid_argument("expected comma");
            std::vector<S> q;
            q.push_back(OptionParser<S>::parse(name, arg, endp));
            while (*(arg = *endp) != '\0') {
                if (*arg != ',')
                    throw std::invalid_argument("expected comma");
                q.push_back(OptionParser<S>::parse(name, arg+1, endp));
            }

            if (q.size() == 3) {
                Result t;
                t.setIdentity();
                Eigen::AngleAxis<S> aa(q[2], Eigen::Matrix<S, 3, 1>::UnitZ());
                t.linear() = aa.toRotationMatrix();
                t.translation() << q[0], q[1], 0;
                return t;
            } else if (q.size() == 6) {
                Eigen::Matrix<S, 3, 1> axis;
                Result t;
                t.setIdentity();
                axis << q[3], q[4], q[5];
                S angle = axis.norm();
                if (std::abs(angle) > 1e-6) {
                    Eigen::AngleAxis<S> aa(angle, axis / angle);
                    t.linear() = aa.toRotationMatrix();
                }
                t.translation() << q[0], q[1], q[2];
                return t;
            } else {
                throw std::invalid_argument(name + " only supports 3 or 6 arguments");
            }
        }
    };

    template <class T>
    struct OptionParser<std::optional<T>> {
        static std::optional<T> parse(const std::string& name, const char *arg, char **endp) {
            if (*arg)
                return OptionParser<T>::parse(name, arg, endp);
            else
                return {};
        }
    };

    class AppOptions {
    public:
        static constexpr unsigned long MAX_JOBS = 1000;

        std::string scenario_;
        std::string algorithm_;
        std::string coordinator_;
        //std::string communicator_;
        std::string lambdaType_;
        unsigned long jobs_{4};

        std::uint64_t num_samples_;
        std::uint64_t problemId_;
        std::uint64_t lambdaId_;
        std::uint64_t randomSeed_{0};

        std::string env_;
        std::string robot_;
        std::string envFrame_;

        std::string start_ = "";
        std::string goal_ = "";
        std::string correct_goal_ = "";
        std::vector<std::string> starts_;
        std::vector<std::string> goals_;

        std::string goalRadius_;

        std::string min_;
        std::string max_;
        std::string global_min_;
        std::string global_max_;
        std::string num_divisions_;
        std::string load_graph_ = "";


        double timeLimit_{std::numeric_limits<double>::infinity()};
        std::uint64_t graphSize_{0};
        double checkResolution_{0};

        bool singlePrecision_{false};

    private:
        static void usage(const char *argv0) {
            std::cerr << "Usage: " << argv0 << R"( [options]
                Options:
                  -S, --scenario=(png|se3|fetch)    Set the scenario to run
                  -a, --algorithm=(prm_fixed_graph|prm_variable_graph) Set the algorithm to run
                  -c, --coordinator=HOST:PORT   Specify the coordinator's host.  Port is optional.
                  -j, --jobs=COUNT              Specify the number of simultaneous lambdas to run
                  -I, --problem-id=ID           (this is for internal use only)
                  -t, --time-limit=TIME         Specify the time limit of each lambda in seconds
                  -n, --num-samples=N           Number of samples per lambda before communication
                  -l, --lambda_id=L_ID          Lambda ID
                  -L, --lambda_type=(local|aws) Lambda Type
                  -e, --env=MESH                The environment mesh to use (valid for se3 and fetch)
                  -E, --env-frame=X,Y,theta     The translation and rotation to apply to the environment (fetch only)
                  -r, --robot=MESH              The robot's mesh (se3 only)
                  -s, --start=W,I,J,K,X,Y,Z     The start configuration (se3 = rotation + translation, fetch = configuration)
                  -g, --goal=W,I,J,K,X,Y,Z      (may be in joint space or IK frame)
                  -G, --goal-radius=RADIUS      Specify the tolerances for the goal as either a scalar or twist (fetch only)
                  -m, --min=X,Y,Z               Workspace minimum (se3 only)
                  -M, --max=X,Y,Z               Workspace maximum (se3 only)
                  -d, --check-resolution=DIST   Collision checking resolution (0 means use default)
                  -f, --float                   Use single-precision math instead of double (not currently enabled)
            )";
        }

        template <class T>
        static T parse(const std::string& name, const std::string& value) {
            char *endp;
            T r = OptionParser<T>::parse(name, value.c_str(), &endp);
            if (*endp)
                throw std::invalid_argument("extra characters in --" + name);
            return r;
        }

    public:
        inline AppOptions() {}
        AppOptions(int argc, char* argv[]) {
            static struct option longopts[] = {
                    { "scenario", required_argument, NULL, 'S' },
                    { "algorithm", required_argument, NULL, 'a' },
                    { "coordinator", required_argument, NULL, 'c' },
                    //{ "communicator", required_argument, NULL, 'C' },
                    { "jobs", required_argument, NULL, 'j' },
                    { "env", required_argument, NULL, 'e' },
                    { "env-frame", required_argument, NULL, 'E' },
                    { "robot", required_argument, NULL, 'r' },
                    { "goal", required_argument, NULL, 'g' },
                    { "goal-radius", required_argument, NULL, 'G' },
                    { "start", required_argument, NULL, 's' },
                    { "lambda_id", required_argument, NULL, 'l' },
                    { "lambda_type", required_argument, NULL, 'L' },
                    { "min", required_argument, NULL, 'm' },
                    { "max", required_argument, NULL, 'M' },
                    { "num_divisions", required_argument, NULL, 'N' },
                    { "global_min", required_argument, NULL, 'o' },
                    { "global_max", required_argument, NULL, 'O' },
                    { "num_samples", required_argument, NULL, 'n' },
                    { "problem-id", required_argument, NULL, 'I' },
                    { "time-limit", required_argument, NULL, 't' },
                    { "graph-size", required_argument, NULL, 'T' },
                    { "check-resolution", required_argument, NULL, 'd' },
                    { "discretization", required_argument, NULL, 'd' }, // less-descriptive alieas
                    { "float", no_argument, NULL, 'f' },
                    { "load_graph", required_argument, NULL, 'R' },
                    { "random_seed", required_argument, NULL, 'Z' },
                    { NULL, 0, NULL, 0 }
            };

            for (int ch ; (ch = getopt_long(argc, argv, "S:a:c:C:j:e:E:r:g:G:s:m:M:I:t:T:d:f:R:Z", longopts, NULL)) != -1 ; ) {
                char *endp;

                switch (ch) {
                    case 'S':
                        scenario_ = optarg;
                        break;
                    case 'a':
                        algorithm_ = optarg;
                        break;
                    case 'c':
                        coordinator_ = optarg;
                        break;
                    //case 'C':
                    //    communicator_ = optarg;
                    //    break;
                    case 'j':
                        jobs_ = strtoul(optarg, &endp, 10);
                        if (endp == optarg || *endp || jobs_ == 0 || jobs_ > MAX_JOBS)
                            throw std::invalid_argument("bad value for --jobs");
                        break;
                    case 'e':
                        env_ = optarg;
                        break;
                    case 'E':
                        envFrame_ = optarg;
                        break;
                    case 'r':
                        robot_ = optarg;
                        break;
                    case 'g':
                        goal_ = optarg; 
                        goals_.push_back(optarg);
                        break;
                    case 'G':
                        goalRadius_ = optarg;
                        break;
                    case 's':
                        start_ = optarg; // Allow for multiple starts
                        starts_.push_back(optarg);
                        break;
                    case 'm':
                        min_ = optarg;
                        break;
                    case 'M':
                        max_ = optarg;
                        break;
                    case 'o':
                        global_min_ = optarg;
                        break;
                    case 'O':
                        global_max_ = optarg;
                        break;
                    case 'n':
                        num_samples_ = strtoull(optarg, &endp, 0);
                        break;
                    case 'N':
                        num_divisions_ = optarg;
                        break;
                    case 'l':
                        lambdaId_ = strtoull(optarg, &endp, 0);
                        break;
                    case 'L':
                        lambdaType_ = optarg;
                        break;
                    case 'I':
                        problemId_ = strtoull(optarg, &endp, 0);
                        if (endp == optarg || *endp)
                            throw std::invalid_argument("bad value for --problem-id");
                        break;
                    case 't':
                        timeLimit_ = std::strtod(optarg, &endp);
                        if (endp == optarg || *endp || timeLimit_ < 0)
                            throw std::invalid_argument("bad value for --time-limit");
                        break;
                    case 'T':
                        graphSize_ = std::strtoull(optarg, &endp, 0);
                        if (endp == optarg || *endp || graphSize_ < 0)
                            throw std::invalid_argument("bad value for --graph-size");
                        break;
                    case 'd':
                        checkResolution_ = std::strtod(optarg, &endp);
                        if (endp == optarg || *endp || checkResolution_ < 0)
                            throw std::invalid_argument("bad value for --check-resolution");
                        break;
                    case 'f':
                        singlePrecision_ = true;
                        break;
                    case 'R':
                        load_graph_ = optarg;
                        break;
                    case 'Z':
                        randomSeed_ = strtoull(optarg, &endp, 0);
                        break;
                    case 'z':
                        correct_goal_ = optarg;
                        break;
                    default:
                        usage(argv[0]);
                        throw std::invalid_argument("see above");
                }
            }
        }


        const std::string& scenario(bool required = true) const {
            if (required && scenario_.empty())
                throw std::invalid_argument("--scenario is required");
            return scenario_;
        }

        const std::string& algorithm(bool required = true) const {
            if (required && algorithm_.empty())
                throw std::invalid_argument("--algorithm is required");
            return algorithm_;
        }

        const std::string& coordinator(bool required = true) const {
            if (required && coordinator_.empty()) {
                throw std::invalid_argument("--coordinator is required");
            }
            return coordinator_;
        }

        //const std::string& communicator(bool required = true) const {
        //    if (required && communicator_.empty()) {
        //        throw std::invalid_argument("--communicator is required");
        //    }
        //    return communicator_;
        //}

        const std::string& lambdaType(bool required = true) const {
            if (required && lambdaType_.empty()) {
                throw std::invalid_argument("--lambda_type is required");
            }
            return lambdaType_;
        }

        const std::uint64_t problemId() const {
            return problemId_;
        }

        const std::string& loadGraph() const {
            return load_graph_;
        }

        const std::uint64_t lambdaId() const {
            return lambdaId_;
        }
        
        const std::uint64_t randomSeed() const {
            return randomSeed_;
        }

        const std::string& env(bool required = true) const {
            if (required && env_.empty()) {
                throw std::invalid_argument("--env is required");
            }
            return env_;
        }

        const unsigned long jobs(bool required = true) const {
            if (required && jobs_ == 0) {
                throw std::invalid_argument("--jobs is required");
            }
            return jobs_;
        }

        template <class T>
        T envFrame() const {
            return parse<T>("env-frame", envFrame_);
        }

        const std::string& robot(bool required = true) const {
            if (required && robot_.empty())
                throw std::invalid_argument("--robot is required");
            return robot_;
        }

        bool singlePrecision() const {
            return singlePrecision_;
        }

        template <class T>
        T start() const {
            return parse<T>("start", start_);
        }

        template <class T>
        T num_divisions() const {
            return parse<T>("num_divisions", num_divisions_);
        }

        template <class T>
        T goal() const {
            return parse<T>("goal", goal_);
        }


        template <class T>
        T correct_goal() const {
            return parse<T>("correct_goal", correct_goal_);
        }

        template <class T>
        T goalRadius() const {
            return parse<T>("goal-radius", goalRadius_);
        }

        template <class T>
        T min() const {
            return parse<T>("min", min_);
        }

        template <class T>
        T max() const {
            return parse<T>("max", max_);
        }

        template <class T>
        T globalMin() const {
            return parse<T>("global_min", global_min_);
        }

        template <class T>
        T globalMax() const {
            return parse<T>("global_max", global_max_);
        }

        template <class T>
        std::vector<T> goals() const {
            std::vector<T> goals;
            for (int i=0; i < goals_.size(); ++i) {
                goals.push_back(parse<T>("goal", goals_[i]));
            }
            return goals;
        }

        template <class T>
        std::vector<std::pair<T, T>> getStartsAndGoals() const {
            std::vector<std::pair<T, T>> starts_and_goals;
            if (starts_.size() == 1 && goals_.size() > 1) {
                for (int i=0; i < goals_.size(); ++i) {
                    starts_and_goals.push_back(std::make_pair(
                                parse<T>("start", starts_[0]),
                                parse<T>("goal", goals_[i])
                                ));
                }
            } else {
                assert(starts_.size() == goals_.size());
                for (int i=0; i < starts_.size(); ++i) {
                    starts_and_goals.push_back(std::make_pair(
                                parse<T>("start", starts_[i]),
                                parse<T>("goal", goals_[i])
                                ));
                }
            }
            return starts_and_goals;
        }

        double timeLimit() const {
            if (graphSize_ != 0 && timeLimit_ != std::numeric_limits<double>::infinity()) {
                JI_LOG(INFO) << "Time limit is " << timeLimit_ << " graph size is " << graphSize_;
                throw std::invalid_argument("--graph-size must be 0 if time-limit is provided");
            }
            return timeLimit_;
        }

        int graphSize() const {
            if (timeLimit_ != std::numeric_limits<double>::infinity()) {
                JI_LOG(INFO) << "Time limit is " << timeLimit_;
                throw std::invalid_argument("--time-limit cannot be provided if graph-size is provided");
            }
            return graphSize_;
        }

        int numSamples() const {
            return num_samples_;
        }

        double checkResolution(double defaultIfZero) const {
            return checkResolution_ <= 0 ? defaultIfZero : checkResolution_;
        }
    };
}

#endif //MPLAMBDA_APP_OPTIONS_HPP
