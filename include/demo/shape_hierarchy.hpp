// Software License Agreement (BSD-3-Clause)
//
// Copyright 2018 The University of North Carolina at Chapel Hill
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author William Lee

#ifndef SHAPE_HIERARCHY
#define SHAPE_HIERARCHY

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <Eigen/Dense>
#include <iostream>

namespace shape
{
    class Color
    {
    public:
        Color(const int r, const int g, const int b)
            : r(r), g(g), b(b) {}
        Color()
            : Color(255, 255, 255) {}
    private:
        const int r;
        const int g;
        const int b;
        friend std::ostream &operator<<(std::ostream &, const Color &);
    };

    inline std::string startTag(std::string tag)
    {
        return "<" + tag;
    }

    inline std::string closeTag()
    {
        return "/>\n";
    }

    inline std::string closeTag(std::string tag)
    {
        return "</" + tag + ">\n";
    }

    template <typename T>
    inline std::string addAttr(std::string name, T value, std::string unit = "")
    {
        std::stringstream ss;
        ss << ' ' << name << "=" <<  "\"" << value << unit << "\"";
        return ss.str();
    }

    inline void startSvg(std::ofstream &file, const int width, const int height)
    {
        file << "<?xml version=\"1.0\" standalone=\"no\" ?>" << std::endl
             << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">" << std::endl
             << startTag("svg")
             << addAttr("width", width, "px")
             << addAttr("height", height, "px")
             << addAttr("xmlns", "http://www.w3.org/2000/svg")
             << addAttr("xmlns:xlink", "http://www.w3.org/1999/xlink")
             << addAttr("version", "1.1")
             << ">\n";
    }

    inline void addBackgroundImg(std::ofstream &file, const std::string &path)
    {
        file << '\t' << startTag("image") << addAttr("xlink:href", path) << closeTag();
    }

    inline void endSvg(std::ofstream &file)
    {
        file << closeTag("svg");
        file.close();
    }

    inline void addEdge(std::ofstream &file, double x1, double y1, double x2, double y2, double width, Color c)
    {
        file << "\t"
             << startTag("line")
             << addAttr("x1", x1)
             << addAttr("y1", y1)
             << addAttr("x2", x2)
             << addAttr("y2", y2)
             << addAttr("stroke", c)
             << addAttr("stroke-width", width)
             << addAttr("stroke-linecap", "round")
             << closeTag();
    }
    
    inline void addText(std::ofstream &file, std::string text, double x, double y, Color c, int font_size) {
        file << "\t"
             << startTag("text")
             << addAttr("x", x)
             << addAttr("y", y)
             << addAttr("fill", c)
             << addAttr("font-size", font_size)
             << addAttr("font-weight", "bold")
	     << ">"
	     << text
             << closeTag("text");

    }

    inline void addSolutionEdge(std::ofstream &file, double x1, double y1, double x2, double y2, double width = 4.0, Color color = Color(250, 50, 50))
    {
        addEdge(file, x1, y1, x2, y2, width, color);
    }

    inline void addVisitedEdge(std::ofstream &file, double x1, double y1, double x2, double y2, double width = 3.0)
    {
        addEdge(file, x1, y1, x2, y2, width, Color(125, 125, 125));
//        addEdge(file, x1, y1, x2, y2, width, Color(255, 0, 0));

    }

    inline void addState(std::ofstream &file, double x, double y, double r, char c)
    {
        file << "\t"
             << startTag("circle")
             << addAttr("cx", x)
             << addAttr("cy", y)
             << addAttr("r", r)
             << addAttr("fill", "rgb(230, 230, 230)")
             << addAttr("stroke", "red")
             << addAttr("stroke-width", r / 5)
             << ">"
             << closeTag("circle");

        // add the label
        file << "\t"
             << startTag("text")
             << addAttr("x", x)
             << addAttr("y", y + r / 3.0)
             << addAttr("fill", "black")
             << addAttr("text-anchor", "middle")
             << addAttr("font-size", r)
             << ">"
             << ' ' << c << ' '
             << closeTag("text");
    }

    inline void addState(std::ofstream &file, double x, double y, double r, std::string c)
    {
        file << "\t"
             << startTag("circle")
             << addAttr("cx", x)
             << addAttr("cy", y)
             << addAttr("r", r)
             << addAttr("fill", "rgb(230, 230, 230)")
             << addAttr("stroke", "red")
             << addAttr("stroke-width", r / 5)
             << ">"
             << closeTag("circle");

        // add the label
        file << "\t"
             << startTag("text")
             << addAttr("x", x)
             << addAttr("y", y + r / 3.0)
             << addAttr("fill", "black")
             << addAttr("text-anchor", "middle")
             << addAttr("font-size", r)
             << ">"
             << ' ' << c << ' '
             << closeTag("text");
    }

    inline void addAnimatedStateWithVelocity(std::ofstream &file, double r, std::vector<std::pair<std::pair<double, double>, double>> path, std::string fill="rgb(230, 0, 0)") {
        std::string cx_values;
        std::string cy_values;
        std::string keyTimes;
        double current_time = 0;
        std::vector<double> keyTimeVector;
        for (int i=0; i < path.size(); ++i) {
            auto& [point, velocity] = path[i];
            
            cx_values += std::to_string(point.first);
            cy_values += std::to_string(point.second);
            keyTimeVector.push_back(current_time);
            if (i < path.size() - 1) {
                cx_values += ";";
                cy_values += ";";
                auto& [other_point, other_velocity] = path[i+1];
                current_time += std::hypot(other_point.first - point.first, other_point.second - point.second) / velocity;
            }
        }
        auto totalTime = keyTimeVector.back();

        for (int i=0; i < keyTimeVector.size(); ++i) {
            keyTimes += std::to_string(keyTimeVector[i] / totalTime);
            if (i < keyTimeVector.size() - 1) keyTimes += ";";
        }
        file << "\t"
             << startTag("circle")
             << addAttr("cx", path.back().first.first)
             << addAttr("cy", path.back().first.second)
             << addAttr("r", r)
             << addAttr("fill", fill)
             << ">"
             << startTag("animate")
             << addAttr("attributeName", "cx")
             << addAttr("dur", std::to_string(totalTime) + "s")
             << addAttr("repeatCount", "1")
             << addAttr("values", cx_values)
             << addAttr("keyTimes", keyTimes)
             << closeTag()
             << startTag("animate")
             << addAttr("attributeName", "cy")
             << addAttr("dur", std::to_string(totalTime) + "s")
             << addAttr("repeatCount", "1")
             << addAttr("values", cy_values)
             << addAttr("keyTimes", keyTimes)
             << closeTag()
             << closeTag("circle");
    }

    inline void addAnimatedState(std::ofstream &file, double x, double y, double r, std::vector<std::vector<double>> path) {
        // Assume we want to go through each part of the path in equal time
        std::string keyTimes;
        std::string cx_values;
        std::string cy_values;
        for (int i=0; i < path.size(); ++i) {
            keyTimes += std::to_string(i * 1.0 / (path.size() - 1));
            if (i < path.size() - 1) {
                keyTimes += ";";
            }
        }
        for (int i=0; i < path.size(); ++i) {
            auto point = path[i];
            cx_values += std::to_string(point[0]);
            cy_values += std::to_string(point[1]);
            if (i < path.size() - 1) {
                cx_values += ";";
                cy_values += ";";
            }
        }
        file << "\t"
             << startTag("circle")
             << addAttr("cx", x)
             << addAttr("cy", y)
             << addAttr("r", r)
             << addAttr("fill", "rgb(230, 0, 0)")
             << addAttr("stroke", "red")
             << addAttr("stroke-width", r / 5)
             << ">"
             << startTag("animate")
             << addAttr("attributeName", "cx")
             << addAttr("dur", "10s")
             << addAttr("repeatCount", "indefinite")
             << addAttr("values", cx_values)
             << addAttr("keyTimes", keyTimes)
             << closeTag()
             << startTag("animate")
             << addAttr("attributeName", "cy")
             << addAttr("dur", "10s")
             << addAttr("repeatCount", "indefinite")
             << addAttr("values", cy_values)
             << addAttr("keyTimes", keyTimes)
             << closeTag()
             << closeTag("circle");
    }

    inline void addStartState(std::ofstream &file, double x, double y, double r = 20)
    {
        addState(file, x, y, r, 'S');
    }

    inline void addGoalState(std::ofstream &file, double x, double y, double r = 20)
    {
        addState(file, x, y, r, 'G');
    }

    inline void addPath(std::ofstream &file, const std::vector<std::vector<double>>& path, double width = 2.0, Color color = Color(250, 50, 50)) {
        //file  << "\t"
        //  << startTag("path")
        //  << "d=\"";
        for (int i=0; i < path.size() - 1; ++i) {
            //file << "M" << point[0] << " " << point[1] << " ";
          auto point = path[i]; 
          auto other = path[i + 1];
          addEdge(file, point[0], point[1], other[0], other[1], width, color);
        }
        //file << "\"" << closeTag();
    }

    inline void addDubinsCar(std::ofstream &file, const std::vector<std::vector<double>>& path, double velocity, double width = 2.0, Color color = Color(250, 50, 50)) {
        std::string cx_values;
        std::string cy_values;
        std::string keyTimes;
        double current_time = 0;
        std::vector<double> keyTimeVector;
        for (int i=0; i < path.size(); ++i) {
            auto& point = path[i];
            
            cx_values += std::to_string(point[0]);
            cy_values += std::to_string(point[1]);
            keyTimeVector.push_back(current_time);
            if (i < path.size() - 1) {
                cx_values += ";";
                cy_values += ";";
                auto& other_point = path[i+1];
                current_time += std::hypot(other_point[0] - point[0], other_point[1] - point[1]) / velocity;
            }
        }
        auto totalTime = keyTimeVector.back();

        for (int i=0; i < keyTimeVector.size(); ++i) {
            keyTimes += std::to_string(keyTimeVector[i] / totalTime);
            if (i < keyTimeVector.size() - 1) keyTimes += ";";
        }
        file << "\t"
             << startTag("ellipse")
             << addAttr("cx", path.back()[0])
             << addAttr("cy", path.back()[1])
             << addAttr("rx", 15)
             << addAttr("ry", 10)
             << addAttr("fill", color)
             << ">"
             << startTag("animate")
             << addAttr("attributeName", "cx")
             << addAttr("dur", std::to_string(totalTime) + "s")
             << addAttr("repeatCount", "1")
             << addAttr("values", cx_values)
             << addAttr("keyTimes", keyTimes)
             << closeTag()
             << startTag("animate")
             << addAttr("attributeName", "cy")
             << addAttr("dur", std::to_string(totalTime) + "s")
             << addAttr("repeatCount", "1")
             << addAttr("values", cy_values)
             << addAttr("keyTimes", keyTimes)
             << closeTag();
             //<< startTag("animateTransform")
             //<< addAttr("attributeName", "transform")
             //<< addAttr("attributeType", "XML")
             //<< addAttr("dur", std::to_string(totalTime) + "s")
             //<< addAttr("repeatCount", "1")
             //<< addAttr("values", cy_values)
             //<< addAttr("keyTimes", keyTimes)
             //<< closeTag()

        current_time = 0.0;
        for (int i=0; i < path.size() - 1; ++i) {
            auto& point = path[i];
            auto& other_point = path[i+1];
            auto old_time = current_time;
            current_time += std::hypot(other_point[0] - point[0], other_point[1] - point[1]) / velocity;
            file 
             << startTag("animateTransform")
             << addAttr("attributeName", "transform")
             << addAttr("attributeType", "XML")
             << addAttr("type", "rotate")
             << addAttr("begin", std::to_string(old_time) + "s")
             << addAttr("dur", std::to_string(current_time) + "s")
             << addAttr("repeatCount", "1")
             << addAttr("from", std::to_string(point[2] * 180.0 / 3.14) + " " + std::to_string(point[0]) + " " + std::to_string(point[1]))
             << addAttr("to", std::to_string(other_point[2] * 180.0 / 3.14) + " " + std::to_string(other_point[0]) + " " + std::to_string(other_point[1]))
             << closeTag();
        }

        file << closeTag("ellipse");
    }

    template <typename Scalar>
    class Rect
    {
    public:
        using State = Eigen::Matrix<Scalar, 2, 1>;
        // TODO: check parameter in the constructor
        Rect(const Scalar x0, const Scalar y0, const Scalar x1, const Scalar y1, const Color color = Color(0, 0, 0))
            : p0_(State(x0, y0)), p1_(State(x1, y1)), color_(color) {}

        bool pointIsValid(const State &p) const
        {
            Scalar px = p[0];
            Scalar py = p[1];
            return !(px >= p0_[0] && px <= p1_[0] && py >= p0_[1] && py <= p1_[1]);
        }

        bool segmentIsValid(const State &a, const State &b) const
        {
            if (!pointIsValid(a) || !pointIsValid(b))
                return false;
            return bisectSegment(a, b);
        }

        bool bisectSegment(const State &a, const State &b) const
        {
            State mid = (a + b) / 2;
            Scalar distSquared = (b - a).squaredNorm();
            Scalar tolerance = 1;
            if (distSquared < tolerance * tolerance)
                return true;
            if (!pointIsValid(mid))
                return false;
            if (!bisectSegment(a, mid)) // check the left half
                return false;
            return bisectSegment(mid, b); // check the right half 
        }

    private:
        const State p0_;
        const State p1_;
        const Color color_;
        template <typename Scalar_>
        friend std::ostream &operator<<(std::ostream &, const Rect<Scalar_> &);
    };

    template <typename Scalar>
    class Circle
    {
    public:
        using State = Eigen::Matrix<Scalar, 2, 1>;

        Circle(Scalar rx, Scalar ry, const Scalar radius, const Color color = Color(0, 0, 0))
            : center_(State(rx, ry)), radius_(radius), color_(color) {}

        bool pointIsValid(const State &p) const
        {
            State dist = p - center_;
            return dist.squaredNorm() > radius_ * radius_;
        }

        bool segmentIsValid(const State &a, const State &b) const
        {
            return distPointSegmentSquared(center_, a, b) > radius_ * radius_;
        }

        // for robot with non trivial size (e.g. robot arm)
        bool segmentIsValid(const State &a, const State &b, Scalar robotRadius) const
        {
            return distPointSegmentSquared(center_, a, b) > (radius_ + robotRadius) * (radius_ + robotRadius);
        }

        Scalar cx() const
        {
            return center_[0];
        }

        Scalar cy() const
        {
            return center_[1];
        }

        Scalar r() const
        {
            return radius_;
        }

    private:
        const State center_;
        const Scalar radius_;
        const Color color_;

        Scalar distPointSegmentSquared (const State &pt, const State &s0, const State &s1) const
        {
            State v = s1 - s0;
            State w = pt - s0;
            Scalar c1 = v.dot(w);
            if (c1 <= 0)
                return w.squaredNorm();
            Scalar c2 = v.squaredNorm();
            if (c2 <= c1)
                return (pt - s1).squaredNorm();
            return (s0 - pt + v * (c1 / c2)).squaredNorm();
        }
        template <typename Scalar_>
        friend std::ostream &operator<<(std::ostream &, const Circle<Scalar_> &);
    };

    std::ostream &operator<<(std::ostream &strm, const Color &c)
    {
        return strm << "rgb(" << c.r << "," << c.g << "," << c.b << ")";
    }

    template <typename Scalar>
    std::ostream &operator<<(std::ostream &strm, const Circle<Scalar> &c)
    {
        return strm << '\t'
               << startTag("circle")
               << addAttr("cx", c.center_[0])
               << addAttr("cy", c.center_[1])
               << addAttr("r", c.radius_)
               << addAttr("fill", c.color_)
               << addAttr("stroke", "black")
               << addAttr("stroke-width", "0px") //TODO: find the right stroke width
               << closeTag();
    }

    template <typename Scalar>
    std::ostream &operator<<(std::ostream &strm, const Rect<Scalar> &r)
    {
        return strm << '\t'
               << startTag("rect")
               << addAttr("x", r.p0_[0])
               << addAttr("y", r.p0_[1])
               << addAttr("width", r.p1_[0] - r.p0_[0])
               << addAttr("height", r.p1_[1] - r.p0_[1])
//               << addAttr("fill", r.color_)
                << addAttr("fill-opacity", "0")
               << addAttr("stroke", r.color_)
               << addAttr("stroke-width", "6px")
               << closeTag();
    }
}
#endif
