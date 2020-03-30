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

}

#endif //MPLAMBDA_UTIL_HPP
