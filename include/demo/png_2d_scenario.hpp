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

#ifndef PNG_2D_SCENARIO
#define PNG_2D_SCENARIO

//#include <mpt/lp_space.hpp>
//#include <mpt/box_bounds.hpp>
//#include <mpt/goal_state.hpp>
#include <png.h>
#include <Eigen/Dense>
#include <nigh/se3_space.hpp>
#include <vector>
#include <interpolate.hpp>
#include <randomize.hpp>
#include <jilog.hpp>

constexpr bool PRINT_FILTERED_IMAGE = false; // enable this to export a filtered png file.

namespace mpl::demo
{
    struct FilterColor
    {
        FilterColor(int r, int g, int b, int tol)
            : r_(r), g_(g), b_(b), tol_(tol)
        {
        }
        int r_;
        int g_;
        int b_;
        int tol_;

        bool isObstacle(int r, int g, int b) const
        {
            if ((r < r_ - tol_ || r > r_ + tol_) || (g < g_ - tol_ || g > g_ + tol_) || (b < b_ - tol_ || b > b_ + tol_))
            {
                return false;
            }
            return true;
        }
    };

    template <typename Scalar = double>
    class PNG2dScenario
    {
    public:

        using Space = unc::robotics::nigh::metric::L2Space<Scalar, 2>;
        using State = typename Space::Type;
        using Bound = Eigen::Matrix<Scalar, 2, 1>;
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

    public:
        PNG2dScenario(
            const int width,
            const int height,
            State min,
            State max,
            //State goalState,
            std::vector<bool> &isObstacle
        )
            : width_(width),
              height_(height),
              min_(min),
              max_(max),
              //goal_(goalState),
              isObstacle_(isObstacle)
        {
        }

        const Distance maxSteering() const {
            return std::numeric_limits<Distance>::infinity();
        }

        bool isValid(const State &q) const
        {
            int x = (int) (q[0] + 0.5);
            int y = (int) (q[1] + 0.5);

            return !isObstacle_[width_ * y + x];
        }

        bool isValid(const State &a, const State &b) const
        {
            if(!isValid(a) || !isValid(b))
                return false;
            return validSegment(a, b);
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

        //const State &goal() const
        //{
        //    return goal_;
        //}

        const int width() const
        {
            return width_;
        }

        const int height() const
        {
            return height_;
        }

        static State scale(const State& q) {
            return q;
        }

        template <class RNG>
        State randomSample(RNG& rng) {
            State q;
            randomize(q, rng, min_, max_);
            return q;
        }

        Scalar prmRadius() {
            return 2 * pow((width_ * height_ * 1. / PI) * (3.0 / 2.0), 0.5);
        }

    private:
        Bound makeMinBound()
        {
            Eigen::Matrix<Scalar, 2, 1> min;
            min.fill(0);
            //max << width_, height_;
            return min;
        }

        Bound makeMaxBound()
        {
            Eigen::Matrix<Scalar, 2, 1> max;
            max << width_, height_;
            return max;
        }

        bool validSegment(const State &a, const State &b) const
        {
            // uses the bisection method to verify links.
            State mid = (a + b) / 2;
            Scalar distSquared = (b - a).squaredNorm();
            Scalar tolerance = 1;
            if (distSquared < tolerance * tolerance)
                return true;
            if (!isValid(mid))
                return false;
            if (!validSegment(a, mid)) // check the left half
                return false;
            return validSegment(mid, b); // check the right half
        }
    };


    inline void writePngFile(png_bytep *rowPointers, int width, int height)
    {
        const std::string outputName = "png_planning_filtered.png";
        FILE *fp = fopen(outputName.c_str(), "wb");

        png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
        png_infop info = png_create_info_struct(png);
        if (setjmp(png_jmpbuf(png))) abort();

        png_init_io(png, fp);

        png_set_IHDR(
            png,
            info,
            width, height,
            8,
            PNG_COLOR_TYPE_RGB,
            PNG_INTERLACE_NONE,
            PNG_COMPRESSION_TYPE_DEFAULT,
            PNG_FILTER_TYPE_DEFAULT
        );
        png_write_info(png, info);

        png_write_image(png, rowPointers);
        png_write_end(png, NULL);
        fclose(fp);
    }


    inline std::tuple<std::vector<bool>, int, int> readAndFilterPng(std::vector<FilterColor> &filters, const std::string &inputName)
    {
        /*
         * Read png file
         */
        FILE *fp = std::fopen(inputName.c_str(), "rb");
        if (fp == NULL) {
		JI_LOG(ERROR) << "no file exists";
		exit(1);
	}
        png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
        png_infop info = png_create_info_struct(png);

        png_init_io(png, fp);
        png_read_info(png, info);
        png_byte color_type = png_get_color_type(png, info);
        png_byte bit_depth  = png_get_bit_depth(png, info);

        // resolve pallete img to rgb
        if (color_type == PNG_COLOR_TYPE_PALETTE)
            png_set_palette_to_rgb(png);

        // restrict 1 byte per pixel
        if (bit_depth == 16)
            png_set_strip_16(png);
        if (bit_depth < 8)
            png_set_packing(png);

        // strip alpha channel
        if (color_type & PNG_COLOR_MASK_ALPHA)
            png_set_strip_alpha(png);
        // update the changes
        png_read_update_info(png, info);

        /*
         * allocate the bitmap
         */
        int width = png_get_image_width(png, info);
        int height = png_get_image_height(png, info);
        int rowBytes = png_get_rowbytes(png, info);
        bit_depth  = png_get_bit_depth(png, info);

        std::vector<png_bytep> rowPointers(height);
        std::vector<png_byte> image(rowBytes * height);
        for (int y = 0 ; y < height ; ++y)
            rowPointers[y] = &image[y * rowBytes];
        png_read_image(png, rowPointers.data());
        fclose(fp);

        /*
         * filter the obstacle colors
         */
        std::vector<bool> obstacles;
        obstacles.reserve(width * height);
        const int tolerance = 15;
        for (int y = 0; y < height; y++)
        {
            png_bytep row = rowPointers[y];
            for (int x = 0; x < width; x++)
            {
                png_bytep px = &(row[x * 3]);
                bool isObstacle = false;

                for (auto const &c : filters)
                {
                    if (c.isObstacle(px[0], px[1], px[2]))
                    {
                        isObstacle = true;
                        break;
                    }
                }
                obstacles.push_back(isObstacle ? true : false);

                if (PRINT_FILTERED_IMAGE)
                {
                    px[0] = isObstacle ? 0 : 255;
                    px[1] = isObstacle ? 0 : 255;
                    px[2] = isObstacle ? 0 : 255;
                }
            }
        }

        if (PRINT_FILTERED_IMAGE)
            writePngFile(rowPointers.data(), width, height);

        png_destroy_read_struct(&png, &info, NULL);

        return std::make_tuple(obstacles, width, height);
    }
}
#endif
