//
// Created by Raghav Anand on 2020-03-19.
//

#ifndef MPLAMBDA_UTIL_HPP
#define MPLAMBDA_UTIL_HPP

#include <string>
#include <iostream>
#include <sstream>
#include <Eigen/Dense>

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
    inline std::string state_format(T& state) {
        return ToString(state.format(FullPrecisionCommaInitFormat));
    }

    template <>
    inline std::string state_format(std::tuple<Eigen::Quaternion<double, 0>, Eigen::Matrix<double, 3, 1, 0, 3, 1> > &state) {
        auto& [quat, pos] = state;
        std::ostringstream oStream;
        oStream << quat.w() << ","
                << quat.x() << ","
                << quat.y() << ","
                << quat.z() << ","
                << ToString(pos.format(FullPrecisionCommaInitFormat));
        return oStream.str();
    }


}

template <typename T1, typename T2>
std::ostream& operator<< (std::ostream& out, const std::pair<T1, T2>& v) {
    out << v.first << "," << v.second;
    return out;
}

#endif //MPLAMBDA_UTIL_HPP
