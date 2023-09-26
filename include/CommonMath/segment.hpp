/*!
 * Rapid Collision Detection for Multicopter Trajectories
 *
 * Copyright 2019 by Nathan Bucki <nathan_bucki@berkeley.edu>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include <vector>
#include "CommonMath/Vec3.hpp"

namespace CommonMath {

//! Represents a triple integrator segment in 3D with constant jerk as input.
/*!
 *  The polynomial is a function of time and is of the form
 *  p = p0 + t * (v0 + t * (a0 / 2 + t * j / 6))
 *  v = v0 + t * (a0 + t * j / 2)
 *  a = a0 + t * j
 *  where each coefficient (p0, v0, a0, j) is a vector with three dimensions.
 *  The polynomial is only defined between given start and end times.
 */
class SegmentThirdOrder {
 public:

  //! Constructor.
  /*!
   * @param coeffs The six 3D coefficients defining the polynomial.
   * The coefficients are ordered such that coeffs[0] corresponds to t^5,
   * coeffs[1] to t^4 and so forth.
   * @param startTime The earliest time at which the polynomial is defined
   * @param endTime The latest time at which the polynomial is defined
   */
  SegmentThirdOrder(std::vector<Vec3> coeffs, double startTime, double endTime)
      : _coeffs(coeffs),
        _startTime(startTime),
        _endTime(endTime) {
    assert(coeffs.size() == 4);
    assert(startTime <= endTime);
  }

  //! Constructor.
  /*!
   * @param j/6   Coefficients for t^3
   * @param a0/2  Coefficients for t^2
   * @param v0    Coefficients for t
   * @param p0    Constant terms
   * @param startTime The earliest time at which the polynomial is defined
   * @param endTime The latest time at which the polynomial is defined
   */
  SegmentThirdOrder(Vec3 j, Vec3 a0, Vec3 v0, Vec3 p0,
             double startTime, double endTime)
      : _coeffs {j/6, a0/2, v0, p0},
        _startTime(startTime),
        _endTime(endTime) {
    assert(startTime <= endTime);
  }

  //! Returns the 3D position of the polynomial at a given time.
  /*!
   * @param t Time at which to evaluate the polynomial (must be between startTime and endTime)
   * @return Position of trajectory at time t
   */
  Vec3 GetValue(double t) const {
    assert(t >= _startTime);
    assert(t <= _endTime);
    return _coeffs[0] * t * t * t + _coeffs[1] * t * t + _coeffs[2] * t +
           _coeffs[3];
  }

  //! Returns the position of the polynomial along the given axis at a given time.
  /*!
   * @param i Axis of the trajectory to evaluate
   * @param t Time at which to evaluate the polynomial (must be between startTime and endTime)
   * @return Position of trajectory along axis i at time t
   */
  double GetAxisValue(int i, double t) const {
    assert(t >= _startTime);
    assert(t <= _endTime);
    return _coeffs[0][i] * t * t * t + _coeffs[1][i] * t * t +
           _coeffs[2][i] * t + _coeffs[3][i];
  }

  //! Return the 3D vector of coefficients
  std::vector<Vec3> GetCoeffs() {
    return _coeffs;
  }

  //! Return the start time of the trajectory
  double GetStartTime() const {
    return _startTime;
  }

  //! Return the end time of the trajectory
  double GetEndTime() const {
    return _endTime;
  }

  //! Returns the coefficient of the time derivative of the trajectory
  /*!
   * @return A 5D vector of 3D coefficients representing the time derivative
   * of the trajectory, where derivCoeffs[0] is a 3D vector of coefficients
   * corresponding to t^4, derivCoeffs[1] corresponds to t^3, etc.
   */
  std::vector<Vec3> GetDerivativeCoeffs() const {
    std::vector<Vec3> derivCoeffs;
    derivCoeffs.reserve(3);
    for (int i = 0; i < 3; i++) {
      derivCoeffs.push_back((3 - i) * _coeffs[i]);
    }
    return derivCoeffs;
  }

  //! Returns the difference between this trajectory and a given trajectory.
  /*!
   * The trajectory that is returned is only defined at times where both
   * trajectories are defined (both trajectories must overlap in time). The
   * returned trajectory is the relative position of the two trajectories in
   * time.
   */
  SegmentThirdOrder operator-(const SegmentThirdOrder rhs) {
    double startTime = std::max(_startTime, rhs.GetStartTime());
    double endTime = std::min(_endTime, rhs.GetEndTime());
    return SegmentThirdOrder(_coeffs[0] - rhs[0], _coeffs[1] - rhs[1],
                             _coeffs[2] - rhs[2], _coeffs[3] - rhs[3],
                             startTime, endTime);
  }

  //! Returns the i-th 3D vector of coefficients.
  inline Vec3 operator[](int i) const {
    switch (i) {
      case 0:
        return _coeffs[0];
      case 1:
        return _coeffs[1];
      case 2:
        return _coeffs[2];
      case 3:
        return _coeffs[3];
      default:
        // We should never get here (index out of bounds)
        assert(false);
        return Vec3();  // Returns vector of NaNs
    }
  }

 private:
  std::vector<Vec3> _coeffs;
  double _startTime, _endTime;
};

}
