#ifndef SPACES_REEDS_SHEPP_STATE_SPACE_
#define SPACES_REEDS_SHEPP_STATE_SPACE_

#include <cassert>
#include <math.h>
#include <jilog.hpp>

typedef int (*ReedsSheppPathSamplingCallback)(double q[3], void* user_data);
typedef int (*ReedsSheppPathTypeCallback)(int t, void* user_data);

namespace mpl::demo {

  class ReedsSheppStateSpace
  {
    public:

      /** \brief The Reeds-Shepp path segment types */
      enum ReedsSheppPathSegmentType { RS_NOP=0, RS_LEFT=1, RS_STRAIGHT=2, RS_RIGHT=3 };

      /** \brief Reeds-Shepp path types */
      static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];

      /** \brief Complete description of a ReedsShepp path */
      class ReedsSheppPath
      {
        public:
          ReedsSheppPath(const ReedsSheppPathSegmentType* type=reedsSheppPathType[0],
              double t=std::numeric_limits<double>::max(), double u=0., double v=0.,
              double w=0., double x=0.);

          double length() const { return totalLength_; }

          /** Path segment types */
          const ReedsSheppPathSegmentType* type_;
          /** Path segment lengths */
          double length_[5];
          /** Total length */
          double totalLength_;
      };

      ReedsSheppStateSpace(double turningRadius) : rho_(turningRadius) {}

      double distance(double q0[3], double q1[3]) const;

      void type(double q0[3], double q1[3], ReedsSheppPathTypeCallback cb, void* user_data) const;

      //void sample(double q0[3], double q1[3], double step_size, ReedsSheppPathSamplingCallback cb, void* user_data);

      /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
      ReedsSheppPath reedsShepp(double q0[3], double q1[3]) const;

      template <class Fn>
      void sample(double q0[3], double q1[3], double step_size, Fn cb) const
      {
        ReedsSheppPath path = reedsShepp(q0, q1);
        double dist = rho_ * path.length();

        for (double seg=0.0; seg<=dist; seg+=step_size){
          double qnew[3] = {};
          interpolate(q0, path, seg/rho_, qnew);
          if (cb(qnew)) {
            break;
          }
        }
        return;
      }

    protected:
      void interpolate(double q0[3], ReedsSheppPath &path, double seg, double q[3]) const;

      /** \brief Turning radius */
      double rho_;
  };
}

#endif
