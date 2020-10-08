//
// Created by Raghav Anand on 2020-03-19.
//

#ifndef MPLAMBDA_UTIL_HPP
#define MPLAMBDA_UTIL_HPP

#include <string>
#include <iostream>
#include <sstream>
#include <Eigen/Dense>
#include <queue>

using namespace Eigen;
namespace mpl::util {
    const static Eigen::IOFormat CommaInitFormat(
            Eigen::StreamPrecision, Eigen::DontAlignCols,
            ", ", ",",
            "", "", "", "");

    const static Eigen::IOFormat FullPrecisionCommaInitFormat(
            Eigen::FullPrecision, Eigen::DontAlignCols,
            ", ", ",",
            "", "", "", "");

    template <typename T>
    static inline std::string ToString(T &tX) {
        std::ostringstream oStream;
        oStream << tX;
        return oStream.str();
    }

    template <typename T>
    static inline std::ostringstream ToOStream(T &tX) {
        std::ostringstream oStream;
        oStream << tX;
        return oStream;
    }

    template <typename T>
    static inline const char * ToCString(T &tX) {
        return ToString(tX).c_str();
    }

    void string_split(std::vector<std::string> &parts, std::string &s, std::string delimiter) {
        size_t pos = 0;
        parts.clear();
        std::string token;
        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            //std::cout << token << std::endl;
            parts.emplace_back(token);
            s.erase(0, pos + delimiter.length());
        }
        parts.emplace_back(s);
        //std::cout << s << std::endl;
    }

    template <typename T>
    inline std::string state_format(const T& state) {
        return ToString(state.format(FullPrecisionCommaInitFormat));
    }

    template <>
    inline std::string state_format(const std::tuple<Eigen::Quaternion<double, 0>, Eigen::Matrix<double, 3, 1, 0, 3, 1> > &state) {
        auto& [quat, pos] = state;
        std::ostringstream oStream;
        oStream << quat.w() << ","
                << quat.x() << ","
                << quat.y() << ","
                << quat.z() << ","
                << ToString(pos.format(FullPrecisionCommaInitFormat));
        return oStream.str();
    }

    std::queue<std::pair<std::uint64_t, std::uint64_t>>
        generateWorkQueueEqualWorkAmt(std::uint64_t graph_size, std::uint64_t num_lambdas,
                std::uint64_t num_samples) {
            std::queue<std::pair<std::uint64_t, std::uint64_t>> work_queue;

            double total_work = (graph_size - 1) * std::log(graph_size);
            auto work_per_lambda = total_work / num_lambdas;
            int start_pos = 0, end_pos = 0;
            while (end_pos < graph_size) {
                double current_work = 0;
                while (current_work < work_per_lambda && end_pos < graph_size) {
                    current_work += std::log(++end_pos);
                }
                work_queue.push({start_pos, end_pos});
                start_pos = end_pos;
            }

            return work_queue;
    }

    std::queue<std::pair<std::uint64_t, std::uint64_t>>
        generateWorkQueue(std::uint64_t graph_size, std::uint64_t num_lambdas,
                std::uint64_t num_samples) {
            std::queue<std::pair<std::uint64_t, std::uint64_t>> work_queue;
            // First sort out any edge cases with the initial packet
            auto initial_packet_size = num_lambdas * num_samples;
            initial_packet_size += graph_size % initial_packet_size; // put all the indivisible part here

            auto a = initial_packet_size / num_lambdas;
            auto c = initial_packet_size % num_lambdas;
            auto start_id = 0;
            JI_LOG(INFO) << " a " << a << " c " << c << " rem " << num_lambdas - c << " initial_packet_size " << initial_packet_size;
            for (int i=0; i < c; ++i) {
                work_queue.push({start_id, start_id + a + 1});
                start_id += a + 1;
            }

            for (int i=0; i < num_lambdas - c; ++i) {
                work_queue.push({start_id, start_id + a});
                start_id += a;
            }

            while (start_id < graph_size) {
                work_queue.push({start_id, start_id + num_samples});
                start_id += num_samples;
            }


            return work_queue;
    }


}

template <typename T1, typename T2>
std::ostream& operator<< (std::ostream& out, const std::pair<T1, T2>& v) {
    out << v.first << "," << v.second;
    return out;
}

#endif //MPLAMBDA_UTIL_HPP
