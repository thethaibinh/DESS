/*!
 * Rectangular Pyramid Partitioning using Integrated Depth Sensors
 *
 * Copyright 2020 by Nathan Bucki <nathan_bucki@berkeley.edu>
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

#include "RectangularPyramidPlanner/steering_planner.hpp"

using namespace std::chrono;
using namespace CommonMath;
using namespace RectangularPyramidPlanner;

SteeringPlanner::SteeringPlanner(cv::Mat depthImage, double depthScale,
                                     double focalLength, double principalPointX,
                                     double principalPointY,
                                     double physicalVehicleRadius,
                                     double vehicleRadiusForPlanning,
                                     double minimumCollisionDistance)
    : _depthScale(depthScale),
      _focalLength(focalLength),
      _cx(principalPointX),
      _cy(principalPointY),
      _imageWidth(depthImage.cols),
      _imageHeight(depthImage.rows),
      _trueVehicleRadius(physicalVehicleRadius),
      _vehicleRadiusForPlanning(vehicleRadiusForPlanning),
      _minCheckingDist(minimumCollisionDistance),
      _minimumAllowedThrust(0),  // By default don't allow negative thrust
      _maximumAllowedThrust(30),  // By default limit maximum thrust to about 3g (30 m/s^2)
      _maximumAllowedAngularVelocity(20),  // By default limit maximum angular velocity to 20 rad/s
      _minimumSectionTimeDynamicFeas(0.02),  // By default restrict the dynamic feasibility check to a minimum section duration of 20ms
      _maxPyramidGenTime(1000),  // Don't limit pyramid generation time by default [seconds].
      _pyramidGenTimeNanoseconds(0),
      _maxNumPyramids(std::numeric_limits<int>::max()),  // Don't limit the number of generated pyramids by default
      _allocatedComputationTime(0),  // To be set when the planner is called
      _numTrajectoriesGenerated(0),
      _numCollisionChecks(0),
      _pyramidSearchPixelBuffer(2),  // A sample point must be more than 2 pixels away from the edge of a pyramid to use that pyramid for collision checking
      _steering_amount(0)
{
  _depthData = reinterpret_cast<const uint16_t*>(depthImage.data);
}

bool SteeringPlanner::FindFastestTrajRandomCandidates(
    int8_t sampling_mode,
    ruckig::InputParameter<3>& initial_state,
    ruckig::Trajectory<3>& opt_trajectory,
    double allocatedComputationTime, CommonMath::Vec3 explorationDirection) {

  ExplorationCost explorationCost(explorationDirection);
  return FindLowestCostTrajectoryRandomCandidates(
      sampling_mode,
      initial_state,
      opt_trajectory,
      allocatedComputationTime, &explorationCost,
      &ExplorationCost::direction_cost_wrapper);
}

bool SteeringPlanner::FindLowestCostTrajectoryRandomCandidates(
    int8_t sampling_mode,
    ruckig::InputParameter<3>& initial_state,
    ruckig::Trajectory<3>& opt_trajectory,
    double allocatedComputationTime,
    void* costFunctionDefinitionObject,
    double (*costFunctionWrapper)(
        void* costFunctionDefinitionObject,
        ruckig::Trajectory<3>&)) {

  RandomTrajectoryGenerator trajGenObj(this, sampling_mode);
  return FindLowestCostTrajectory(
      initial_state,
      opt_trajectory,
      allocatedComputationTime, costFunctionDefinitionObject,
      costFunctionWrapper, (void*) &trajGenObj,
      &RandomTrajectoryGenerator::get_next_time_optimal_trajectory_wrapper);
}

bool SteeringPlanner::FindLowestCostTrajectory(
    ruckig::InputParameter<3>& initial_state,
    ruckig::Trajectory<3>& opt_trajectory,
    double allocatedComputationTime,
    void* costFunctionDefinitionObject,
    double (*costFunctionWrapper)(
        void* costFunctionDefinitionObject,
        ruckig::Trajectory<3>&),
    void* trajectoryGeneratorObject,
    int (*trajectoryGeneratorWrapper)(
        void* trajectoryGeneratorObject,
        ruckig::InputParameter<3>& initial_state,
        ruckig::Trajectory<3>& nextTraj)) {

  // Start timing the planner
  _startTime = high_resolution_clock::now();
  _allocatedComputationTime = allocatedComputationTime;

  bool feasibleTrajFound = false;
  double bestCost = std::numeric_limits<double>::max();

  while (true) {

    if (duration_cast<microseconds>(high_resolution_clock::now() - _startTime)
        .count() > int(_allocatedComputationTime * 1e6)) {
      break;
    }

    // Get the next candidate trajectory to evaluate using the provided trajectory generator
    ruckig::Trajectory<3> candidateTraj;
    int returnVal = (*trajectoryGeneratorWrapper)(trajectoryGeneratorObject,
                                                  initial_state, candidateTraj);
    if (returnVal < 0) {
      // There are no more candidate trajectories to check. This case should only be reached if
      // the candidate trajectory generator is designed to only give a finite number of candidates
      // (e.g. when using a gridded approach instead of using random search)
      break;
    }
    _numTrajectoriesGenerated++;

    // Compute the cost of the trajectory using the provided cost function
    double cost = (*costFunctionWrapper)(costFunctionDefinitionObject, candidateTraj);
    if (cost < bestCost) {
      // The trajectory is a lower cost than lowest cost trajectory found so far
      // Check whether the trajectory collides with obstacles
      // bool isCollisionFree = IsCollisionFree(candidateTraj.GetTrajectory());
      // First split trajectory into possible 7 segment as described in the profile then check every one of them.
      std::vector<SegmentThirdOrder> segments = get_segments(candidateTraj);
      bool isCollisionFree = true;
      for (size_t i = 0; i < segments.size(); i++)
      {
        isCollisionFree &= is_segment_collision_free(segments[i]);
      }
      _numCollisionChecks++;
      if (isCollisionFree) {
        feasibleTrajFound = true;
        bestCost = cost;
        opt_trajectory = candidateTraj;
      }
    }
  }

  if (feasibleTrajFound) {
    return true;
  } else {
    _steering_amount = scan_depth();
    return false;
  }
}

int SteeringPlanner::scan_depth() {
  // We don't look at any pixels closer than this distance (e.g. if the
  // propellers are in the field of view)
  uint16_t ignoreDist = uint16_t(_trueVehicleRadius / _depthScale);
  // For reading the depth value stored in a given pixel.
  uint16_t pixDist;
  // Store the minimum depth pixel value of the expanded pyramid. The base plane
  // of the final pyramid will be this value minus the vehicle radius.
  uint16_t min_depth_value = std::numeric_limits<uint16_t>::max();
  int min_depth_x, min_depth_y;
  for (int y = 0; y < _imageHeight; y++) {
    for (int x = 0; x < _imageWidth; x++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist <= min_depth_value && pixDist > ignoreDist) {
        min_depth_value = std::min(min_depth_value, pixDist);
        min_depth_x = x;
        min_depth_y = y;
      }
    }
  }
  if (min_depth_x < _cx) {
    return 1;
  }

  else {
    return -1;
  }
}

std::vector<SegmentThirdOrder> SteeringPlanner::get_segments(
  const ruckig::Trajectory<3> traj) {
  std::vector<SegmentThirdOrder> segments;

  auto profile_array = traj.get_profiles();
  // We only consider one-target-waypoint trajectory
  assert(profile_array.size() == 1);
  auto profiles = profile_array[0];
  // Confirming there are 3 DOFs in a trajectory
  size_t num_dof = profiles.size();
  assert(num_dof == 3);

  // Adding two possible pre-trajectories (brake/accel) if there is any
  double brake_duration = profiles[0].brake.duration;
  if (brake_duration > 1e-6) {
    Vec3 j =
      Vec3(profiles[0].brake.j[0], profiles[1].j[0], profiles[2].brake.j[0]);
    Vec3 a0 = Vec3(profiles[0].brake.a[0], profiles[1].brake.a[0],
                   profiles[2].brake.a[0]);
    Vec3 v0 = Vec3(profiles[0].brake.v[0], profiles[1].brake.v[0],
                   profiles[2].brake.v[0]);
    Vec3 p0 = Vec3(profiles[0].brake.p[0], profiles[1].brake.p[0],
                   profiles[2].brake.p[0]);

    segments.push_back(SegmentThirdOrder(j, a0, v0, p0, 0.0, brake_duration));
  }
  double accel_duration = profiles[0].accel.duration;
  if (accel_duration > 1e-6) {
    Vec3 j =
      Vec3(profiles[0].accel.j[0], profiles[1].j[0], profiles[2].accel.j[0]);
    Vec3 a0 = Vec3(profiles[0].accel.a[0], profiles[1].accel.a[0],
                   profiles[2].accel.a[0]);
    Vec3 v0 = Vec3(profiles[0].accel.v[0], profiles[1].accel.v[0],
                   profiles[2].accel.v[0]);
    Vec3 p0 = Vec3(profiles[0].accel.p[0], profiles[1].accel.p[0],
                   profiles[2].accel.p[0]);
    segments.push_back(SegmentThirdOrder(j, a0, v0, p0, 0.0, accel_duration));
  }

  // Then adding possible 7 sections of the trajectory
  size_t num_sec = profiles[0].t.size();
  assert(num_sec == 7);
  for (size_t i = 0; i < num_sec; i++) {
    double endTime = profiles[0].t[i];
    if (fabs(endTime) < 1e-6) continue;
    Vec3 j = Vec3(profiles[0].j[i], profiles[1].j[i], profiles[2].j[i]);
    Vec3 a0 = Vec3(profiles[0].a[i], profiles[1].a[i], profiles[2].a[i]);
    Vec3 v0 = Vec3(profiles[0].v[i], profiles[1].v[i], profiles[2].v[i]);
    Vec3 p0 = Vec3(profiles[0].p[i], profiles[1].p[i], profiles[2].p[i]);

    segments.push_back(SegmentThirdOrder(j, a0, v0, p0, 0.0, endTime));
  }
  return segments;
}

bool SteeringPlanner::is_segment_collision_free(SegmentThirdOrder segment) {
  // Split segment into sections with monotonically changing depth
  std::vector<MonotonicSegment> monotonicSections =
    get_monotonic_segments(segment);
  while (monotonicSections.size() > 0) {
    // Check if we've used up all of our computation time
    if (duration_cast<microseconds>(high_resolution_clock::now() - _startTime)
          .count() > int(_allocatedComputationTime * 1e6)) {
      return false;
    }

    // Get a monotonic section to check
    MonotonicSegment monoTraj = monotonicSections.back();
    monotonicSections.pop_back();

    // Find the pixel corresponding to the endpoint of this section (deepest
    // depth)
    Vec3 startPoint, endPoint;
    if (monoTraj.increasingDepth) {
      startPoint = monoTraj.GetValue(monoTraj.GetStartTime());
      endPoint = monoTraj.GetValue(monoTraj.GetEndTime());
    } else {
      startPoint = monoTraj.GetValue(monoTraj.GetEndTime());
      endPoint = monoTraj.GetValue(monoTraj.GetStartTime());
    }

    // Ignore the segment section if it's closer than the minimum collision
    // checking distance
    if (startPoint.z < _minCheckingDist && endPoint.z < _minCheckingDist) {
      continue;
    }

    // Try to find pyramid that contains endPoint
    double endPointPixel[2];
    ProjectPointToPixel(endPoint, endPointPixel[0], endPointPixel[1]);
    Pyramid collisionCheckPyramid;
    bool pyramidFound = FindContainingPyramid(
      endPointPixel[0], endPointPixel[1], endPoint.z, collisionCheckPyramid);
    if (!pyramidFound) {
      // No pyramids containing endPoint were found, try to make a new pyramid
      if (_pyramids.size() >= _maxNumPyramids ||
          _pyramidGenTimeNanoseconds > _maxPyramidGenTime * 1e9) {
        // We've already exceeded the maximum number of allowed pyramids or
        // the maximum time allocated for pyramid generation.
        return false;
      }

      high_resolution_clock::time_point startInflate =
        high_resolution_clock::now();
      bool pyramidGenerated = InflatePyramid(endPointPixel[0], endPointPixel[1],
                                             endPoint.z, collisionCheckPyramid);
      _pyramidGenTimeNanoseconds +=
        duration_cast<nanoseconds>(high_resolution_clock::now() - startInflate)
          .count();

      if (pyramidGenerated) {
        // Insert the new pyramid into the list of pyramids found so far
        auto index = std::lower_bound(_pyramids.begin(), _pyramids.end(),
                                      collisionCheckPyramid.depth);
        _pyramids.insert(index, collisionCheckPyramid);
      } else {
        // No pyramid could be formed, so there must be a collision
        return false;
      }
    }

    // Check if/when the segment intersects a lateral face of the given pyramid.
    double collisionTime;
    bool collidesWithPyramid =
      FindDeepestCollisionTime(monoTraj, collisionCheckPyramid, collisionTime);

    if (collidesWithPyramid) {
      // The segment collides with at least lateral face of the pyramid. Split
      // the segment where it intersects, and add the section outside the
      // pyramid for further collision checking.
      if (monoTraj.increasingDepth) {
        monotonicSections.push_back(MonotonicSegment(
          monoTraj.GetCoeffs(), monoTraj.GetStartTime(), collisionTime));
      } else {
        monotonicSections.push_back(MonotonicSegment(
          monoTraj.GetCoeffs(), collisionTime, monoTraj.GetEndTime()));
      }
    }
  }
  return true;
}

std::vector<MonotonicSegment> SteeringPlanner::get_monotonic_segments(
  SegmentThirdOrder segment) {
  // This function exploits the property described in Section II.B of the
  // RAPPIDS paper

  // Compute the coefficients of \dot{d}_z(t)
  std::vector<Vec3> trajDerivativeCoeffs = segment.GetDerivativeCoeffs();
  double c[3] = {trajDerivativeCoeffs[0].z, trajDerivativeCoeffs[1].z,
                 trajDerivativeCoeffs[2].z};  // Just shortening the names

  // Compute the times at which the segment changes direction along the z-axis
  double roots[4];
  roots[0] = segment.GetStartTime();
  roots[1] = segment.GetEndTime();
  size_t rootCount;
  rootCount = RootFinder::solve_quadratic(c[2], c[1], c[0], roots + 2);
  std::sort(
    roots,
    roots + rootCount +
      2);  // Use rootCount + 2 because we include the start and end point

  std::vector<MonotonicSegment> monotonicSections;
  // We don't iterate until rootCount + 2 because we need to find pairs of roots
  for (unsigned i = 0; i < rootCount + 1; i++) {
    if (roots[i] < segment.GetStartTime()) {
      // Skip root if it's before start time
      continue;
    } else if (fabs(roots[i] - roots[i + 1]) < 1e-6) {
      // Skip root because it's a duplicate
      continue;
    } else if (roots[i] >= segment.GetEndTime()) {
      // We're done because the roots are in ascending order
      break;
    }
    // Add a section between the current root and the next root after checking
    // that the next root is valid We already know that roots[i+1] is greater
    // than the start time because roots[i] is greater than the start time and
    // roots is sorted
    if (roots[i + 1] <= segment.GetEndTime()) {
      monotonicSections.push_back(
        MonotonicSegment(segment.GetCoeffs(), roots[i], roots[i + 1]));
    } else {
      // We're done because the next section is out of the range
      break;
    }
  }
  std::sort(monotonicSections.begin(), monotonicSections.end());
  return monotonicSections;
}

bool SteeringPlanner::FindContainingPyramid(double pixelX, double pixelY,
                                            double depth, Pyramid& outPyramid) {
  // This function searches _pyramids for those with base planes at deeper
  // depths than endPoint.z
  auto firstPyramidIndex =
    std::lower_bound(_pyramids.begin(), _pyramids.end(), depth);
  if (firstPyramidIndex != _pyramids.end()) {
    // At least one pyramid exists that has a base plane deeper than endPoint.z
    for (std::vector<Pyramid>::iterator it = firstPyramidIndex;
         it != _pyramids.end(); ++it) {
      // Check whether endPoint is inside the pyramid
      // We need to use the _pyramidSearchPixelBuffer offset here because
      // otherwise we'll try to collision check with the pyramid we just exited
      // while checking the previous section
      if ((*it).leftPixBound + _pyramidSearchPixelBuffer < pixelX &&
          pixelX < (*it).rightPixBound - _pyramidSearchPixelBuffer &&
          (*it).topPixBound + _pyramidSearchPixelBuffer < pixelY &&
          pixelY < (*it).bottomPixBound - _pyramidSearchPixelBuffer) {
        outPyramid = *it;
        return true;
      }
    }
  }
  return false;
}

bool SteeringPlanner::FindDeepestCollisionTime(MonotonicSegment monoTraj,
                                               Pyramid pyramid,
                                               double& outCollisionTime) {
  // This function exploits the property described in Section II.C of the
  // RAPPIDS paper

  bool collidesWithPyramid = false;
  if (monoTraj.increasingDepth) {
    outCollisionTime = monoTraj.GetStartTime();
  } else {
    outCollisionTime = monoTraj.GetEndTime();
  }
  std::vector<Vec3> coeffs = monoTraj.GetCoeffs();
  for (Vec3 normal : pyramid.planeNormals) {
    // Compute the coefficients of d(t) (distance to the lateral face of the
    // pyramid)
    double c[4] = {0, 0, 0, 0};
    for (int dim = 0; dim < 3; dim++) {
      c[0] += normal[dim] * coeffs[0][dim];  // t^3
      c[1] += normal[dim] * coeffs[1][dim];  // t^2
      c[2] += normal[dim] * coeffs[2][dim];  // t
      c[3] += normal[dim] * coeffs[3][dim];  // constant
    }

    // Find the times at which the trajectory intersects the plane
    double roots[3];
    size_t rootCount;
    // reducing to finding quadratic roots due to root zero isolation
    if (fabs(c[0]) > 1e-6) {
      rootCount =
        RootFinder::solveP3(c[1] / c[0], c[2] / c[0], c[3] / c[0], roots);
    } else {
      rootCount = RootFinder::solve_quadratic(c[3], c[2], c[1], roots);
    }
    std::sort(roots, roots + rootCount);
    if (monoTraj.increasingDepth) {
      // Search backward in time (decreasing depth)
      for (int i = rootCount - 1; i >= 0; i--) {
        if (roots[i] > monoTraj.GetEndTime()) {
          continue;
        } else if (roots[i] > monoTraj.GetStartTime()) {
          if (roots[i] > outCollisionTime) {
            // This may seem unnecessary because we are searching an ordered
            // list, but this check is needed because we are checking multiple
            // lateral faces of the pyramid for collisions
            outCollisionTime = roots[i];
            collidesWithPyramid = true;
            break;
          }
        } else {
          break;
        }
      }
    } else {
      // Search forward in time (decreasing depth)
      for (int i = 0; i < int(rootCount); i++) {
        if (roots[i] < monoTraj.GetStartTime()) {
          continue;
        } else if (roots[i] < monoTraj.GetEndTime()) {
          if (roots[i] < outCollisionTime) {
            outCollisionTime = roots[i];
            collidesWithPyramid = true;
            break;
          }
        } else {
          break;
        }
      }
    }
  }
  return collidesWithPyramid;
}

bool SteeringPlanner::InflatePyramid(int x0, int y0, double minimumDepth,
                                     Pyramid& outPyramid) {
  // This function is briefly described by Section III.A. of the RAPPIDS paper

  // First check if the sample point violates the field of view constraints
  int imageEdgeOffset = _focalLength * _trueVehicleRadius / _minCheckingDist;
  if (x0 <= imageEdgeOffset + _pyramidSearchPixelBuffer + 1
      || x0 > _imageWidth - imageEdgeOffset - _pyramidSearchPixelBuffer - 1
      || y0 <= imageEdgeOffset + _pyramidSearchPixelBuffer + 1
      || y0 > _imageHeight - imageEdgeOffset - _pyramidSearchPixelBuffer - 1) {
    // Sample point could be in collision with something outside the FOV
    return false;
  }

  // The base plane of the pyramid must be deeper than this depth (written in pixel depth units)
  uint16_t minimumPyramidDepth = uint16_t(
      (minimumDepth + _vehicleRadiusForPlanning) / _depthScale);

  // This is the minimum "radius" (really the width/height divided by two) of a valid pyramid
  int initPixSearchRadius = _focalLength * _vehicleRadiusForPlanning
      / (_depthScale * minimumPyramidDepth);

  if (2 * initPixSearchRadius
      >= std::min(_imageWidth, _imageHeight) - 2 * imageEdgeOffset) {
    // The minimum size of the pyramid is larger than the maximum pyramid size
    return false;
  }

  // These edges are the edges of the expanded pyramid before it is shrunk to the final size
  int leftEdge, topEdge, rightEdge, bottomEdge;
  if (y0 - initPixSearchRadius < imageEdgeOffset) {
    topEdge = imageEdgeOffset;
    bottomEdge = topEdge + 2 * initPixSearchRadius;
  } else {
    bottomEdge = std::min(_imageHeight - imageEdgeOffset - 1,
                          y0 + initPixSearchRadius);
    topEdge = bottomEdge - 2 * initPixSearchRadius;
  }
  if (x0 - initPixSearchRadius < imageEdgeOffset) {
    leftEdge = imageEdgeOffset;
    rightEdge = leftEdge + 2 * initPixSearchRadius;
  } else {
    rightEdge = std::min(_imageWidth - imageEdgeOffset - 1,
                         x0 + initPixSearchRadius);
    leftEdge = rightEdge - 2 * initPixSearchRadius;
  }

  // We don't look at any pixels closer than this distance (e.g. if the propellers are in the field of view)
  uint16_t ignoreDist = uint16_t(_trueVehicleRadius / _depthScale);
  // For reading the depth value stored in a given pixel.
  uint16_t pixDist;

  for (int y = topEdge; y < bottomEdge; y++) {
    for (int x = leftEdge; x < rightEdge; x++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist <= minimumPyramidDepth && pixDist > ignoreDist) {
        // We are unable to inflate a rectangle that will meet the minimum size requirements
        return false;
      }
    }
  }

  // Store the minimum depth pixel value of the expanded pyramid. The base plane of the final pyramid will be
  // this value minus the vehicle radius.
  uint16_t maxDepthExpandedPyramid = std::numeric_limits<uint16_t>::max();

  // We search each edge of the rectangle until we hit a pixel value closer than minimumPyramidDepth
  // This creates a spiral search pattern around the initial sample point
  // Once all four sides of the pyramid hit either the FOV constraint or a pixel closer than
  // minimumPyramidDepth, we will shrink the pyramid based on the vehicle radius.
  bool rightFree = true, topFree = true, leftFree = true, bottomFree = true;
  while (rightFree || topFree || leftFree || bottomFree) {
    if (rightFree) {
      if (rightEdge < _imageWidth - imageEdgeOffset - 1) {
        for (int y = topEdge; y <= bottomEdge; y++) {
          pixDist = _depthData[y * _imageWidth + rightEdge + 1];
          if (pixDist > ignoreDist) {
            if (pixDist < minimumPyramidDepth) {
              rightFree = false;
              rightEdge--;  // Negate the ++ after breaking loop
              break;
            }
            maxDepthExpandedPyramid = std::min(maxDepthExpandedPyramid,
                                               pixDist);
          }
        }
        rightEdge++;
      } else {
        rightFree = false;
      }
    }
    if (topFree) {
      if (topEdge > imageEdgeOffset) {
        for (int x = leftEdge; x <= rightEdge; x++) {
          pixDist = _depthData[(topEdge - 1) * _imageWidth + x];
          if (pixDist > ignoreDist) {
            if (pixDist < minimumPyramidDepth) {
              topFree = false;
              topEdge++;  // Negate the -- after breaking loop
              break;
            }
            maxDepthExpandedPyramid = std::min(maxDepthExpandedPyramid,
                                               pixDist);
          }
        }
        topEdge--;
      } else {
        topFree = false;
      }
    }
    if (leftFree) {
      if (leftEdge > imageEdgeOffset) {
        for (int y = topEdge; y <= bottomEdge; y++) {
          pixDist = _depthData[y * _imageWidth + leftEdge - 1];
          if (pixDist > ignoreDist) {
            if (pixDist < minimumPyramidDepth) {
              leftFree = false;
              leftEdge++;  // Negate the -- after breaking loop
              break;
            }
            maxDepthExpandedPyramid = std::min(maxDepthExpandedPyramid,
                                               pixDist);
          }
        }
        leftEdge--;
      } else {
        leftFree = false;
      }
    }
    if (bottomFree) {
      if (bottomEdge < _imageHeight - imageEdgeOffset - 1) {
        for (int x = leftEdge; x <= rightEdge; x++) {
          pixDist = _depthData[(bottomEdge + 1) * _imageWidth + x];
          if (pixDist > ignoreDist) {
            if (pixDist < minimumPyramidDepth) {
              bottomFree = false;
              bottomEdge--;  // Negate the ++ after breaking loop
              break;
            }
            maxDepthExpandedPyramid = std::min(maxDepthExpandedPyramid,
                                               pixDist);
          }
        }
        bottomEdge++;
      } else {
        bottomFree = false;
      }
    }
  }

  // Next, shrink the pyramid according to the vehicle radius
  // Number of pixels to shrink final pyramid. Found by searching outside the boundaries of the expanded pyramid.
  // These edges will be the edges of the final pyramid.
  int rightEdgeShrunk = _imageWidth - 1 - imageEdgeOffset;
  int leftEdgeShrunk = imageEdgeOffset;
  int topEdgeShrunk = imageEdgeOffset;
  int bottomEdgeShrunk = _imageHeight - 1 - imageEdgeOffset;
  int numerator = _focalLength * _vehicleRadiusForPlanning / _depthScale;

  // First check the area between each edge and the edge of the image
  // Check right side
  for (int x = rightEdge; x < _imageWidth; x++) {
    for (int y = topEdge; y <= bottomEdge; y++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        // The pixel is farther away than the minimum checking distance
        if (numerator > (x - rightEdgeShrunk) * pixDist) {
          int rightShrinkTemp = x - int(numerator / pixDist);
          if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking from right will make pyramid invalid
            // Can we shrink from top or bottom instead?
            int topShrinkTemp = y + int(numerator / pixDist);
            int bottomShrinkTemp = y - int(numerator / pixDist);
            if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer
                && y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
              // We can't shrink either edge
              return false;
            } else if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
              // We can't shrink the upper edge, so shrink the lower edge
              bottomEdgeShrunk = bottomShrinkTemp;
            } else if (y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
              // We can't shrink the lower edge, so shrink the upper edge
              topEdgeShrunk = topShrinkTemp;
            } else {
              // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
              int uShrinkLostArea = (topShrinkTemp - topEdgeShrunk);
              int dShrinkLostArea = (bottomEdgeShrunk - bottomShrinkTemp);
              if (dShrinkLostArea > uShrinkLostArea) {
                // We lose more area shrinking the bottom side, so shrink the top side
                topEdgeShrunk = topShrinkTemp;
              } else {
                // We lose more area shrinking the top side, so shrink the bottom side
                rightEdgeShrunk = bottomShrinkTemp;
              }
            }
          } else {
            rightEdgeShrunk = rightShrinkTemp;
          }
        }
      }
    }
  }
  // Check left side
  for (int x = leftEdge; x >= 0; x--) {
    for (int y = topEdge; y <= bottomEdge; y++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        if ((leftEdgeShrunk - x) * pixDist < numerator) {
          int leftShrinkTemp = x + int(numerator / pixDist);
          if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking from left will make pyramid invalid
            // Can we shrink from top or bottom instead?
            int topShrinkTemp = y + int(numerator / pixDist);
            int bottomShrinkTemp = y - int(numerator / pixDist);
            if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer
                && y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
              // We can't shrink either edge
              return false;
            } else if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
              // We can't shrink the upper edge, so shrink the lower edge
              bottomEdgeShrunk = bottomShrinkTemp;
            } else if (y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
              // We can't shrink the lower edge, so shrink the upper edge
              topEdgeShrunk = topShrinkTemp;
            } else {
              // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
              int uShrinkLostArea = (topShrinkTemp - topEdgeShrunk);
              int dShrinkLostArea = (bottomEdgeShrunk - bottomShrinkTemp);
              if (dShrinkLostArea > uShrinkLostArea) {
                // We lose more area shrinking the bottom side, so shrink the top side
                topEdgeShrunk = topShrinkTemp;
              } else {
                // We lose more area shrinking the top side, so shrink the bottom side
                bottomEdgeShrunk = bottomShrinkTemp;
              }
            }
          } else {
            leftEdgeShrunk = leftShrinkTemp;
          }
        }
      }
    }
  }
  if (leftEdgeShrunk + _pyramidSearchPixelBuffer
      > rightEdgeShrunk - _pyramidSearchPixelBuffer) {
    // We shrunk the left and right sides so much that the pyramid is too small!
    return false;
  }

  // Check top side
  for (int y = topEdge; y >= 0; y--) {
    for (int x = leftEdge; x <= rightEdge; x++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        if ((topEdgeShrunk - y) * pixDist < numerator) {
          int topShrinkTemp = y + int(numerator / pixDist);
          if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking from top will make pyramid invalid
            // Can we shrink from left or right instead?
            int rightShrinkTemp = x - int(numerator / pixDist);
            int leftShrinkTemp = x + int(numerator / pixDist);
            if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer
                && x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
              // We can't shrink either edge
              return false;
            } else if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer) {
              // We can't shrink the upper right, so shrink the left edge
              leftEdgeShrunk = leftShrinkTemp;
            } else if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
              // We can't shrink the left edge, so shrink the right edge
              rightEdgeShrunk = rightShrinkTemp;
            } else {
              // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
              int rShrinkLostArea = (rightEdgeShrunk - rightShrinkTemp);
              int lShrinkLostArea = (leftShrinkTemp - leftEdgeShrunk);
              if (rShrinkLostArea > lShrinkLostArea) {
                // We lose more area shrinking the right side, so shrink the left side
                leftEdgeShrunk = leftShrinkTemp;
              } else {
                // We lose more area shrinking the left side, so shrink the right side
                rightEdgeShrunk = rightShrinkTemp;
              }
            }
          } else {
            topEdgeShrunk = topShrinkTemp;
          }
        }
      }
    }
  }
  // Check bottom side
  for (int y = bottomEdge; y < _imageHeight; y++) {
    for (int x = leftEdge; x <= rightEdge; x++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        // The pixel is farther away than the minimum checking distance
        if (numerator > (y - bottomEdgeShrunk) * pixDist) {
          int bottomShrinkTemp = y - int(numerator / pixDist);
          if (y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking from top will make pyramid invalid
            // Can we shrink from left or right instead?
            int rightShrinkTemp = x - int(numerator / pixDist);
            int leftShrinkTemp = x + int(numerator / pixDist);
            if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer
                && x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
              // We can't shrink either edge
              return false;
            } else if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer) {
              // We can't shrink the upper right, so shrink the left edge
              leftEdgeShrunk = leftShrinkTemp;
            } else if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
              // We can't shrink the left edge, so shrink the right edge
              rightEdgeShrunk = rightShrinkTemp;
            } else {
              // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
              int rShrinkLostArea = (rightEdgeShrunk - rightShrinkTemp);
              int lShrinkLostArea = (leftShrinkTemp - leftEdgeShrunk);
              if (rShrinkLostArea > lShrinkLostArea) {
                // We lose more area shrinking the right side, so shrink the left side
                leftEdgeShrunk = leftShrinkTemp;
              } else {
                // We lose more area shrinking the left side, so shrink the right side
                rightEdgeShrunk = rightShrinkTemp;
              }
            }
          } else {
            bottomEdgeShrunk = bottomShrinkTemp;
          }
        }
      }
    }
  }
  if (topEdgeShrunk + _pyramidSearchPixelBuffer
      > bottomEdgeShrunk - _pyramidSearchPixelBuffer) {
    // We shrunk the top and bottom sides so much that the pyramid has no volume!
    return false;
  }

  // Next, check the corners that we ignored before
  // Check top right corner
  for (int y = topEdge; y >= 0; y--) {
    for (int x = rightEdge; x < _imageWidth; x++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        if (numerator > (x - rightEdgeShrunk) * pixDist
            && (topEdgeShrunk - y) * pixDist < numerator) {
          // Both right and top edges could shrink
          int rightShrinkTemp = x - int(numerator / pixDist);
          int topShrinkTemp = y + int(numerator / pixDist);
          if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer
              && y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking either edge makes the pyramid exclude the starting point
            return false;
          } else if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking right edge makes pyramid exclude the starting point, so shrink the top edge
            topEdgeShrunk = topShrinkTemp;
          } else if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking top edge makes pyramid exclude the starting point, so shrink the right edge
            rightEdgeShrunk = rightShrinkTemp;
          } else {
            // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
            int rShrinkLostArea = (rightEdgeShrunk - rightShrinkTemp)
                * (bottomEdgeShrunk - topEdgeShrunk);
            int uShrinkLostArea = (topShrinkTemp - topEdgeShrunk)
                * (rightEdgeShrunk - leftEdgeShrunk);
            if (rShrinkLostArea > uShrinkLostArea) {
              // We lose more area shrinking the right side, so shrink the top side
              topEdgeShrunk = topShrinkTemp;
            } else {
              // We lose more area shrinking the top side, so shrink the right side
              rightEdgeShrunk = rightShrinkTemp;
            }
          }
        }
      }
    }
  }
  // Check bottom right corner
  for (int y = bottomEdge; y < _imageHeight; y++) {
    for (int x = rightEdge; x < _imageWidth; x++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        if (numerator > (x - rightEdgeShrunk) * pixDist
            && numerator > (y - bottomEdgeShrunk) * pixDist) {
          // Both right and bottom edges could shrink
          int rightShrinkTemp = x - int(numerator / pixDist);
          int bottomShrinkTemp = y - int(numerator / pixDist);
          if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer
              && y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking either edge makes the pyramid exclude the starting point
            return false;
          } else if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking right edge makes pyramid exclude the starting point, so shrink the bottom edge
            bottomEdgeShrunk = bottomShrinkTemp;
          } else if (y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking bottom edge makes pyramid exclude the starting point, so shrink the right edge
            rightEdgeShrunk = rightShrinkTemp;
          } else {
            // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
            int rShrinkLostArea = (rightEdgeShrunk - rightShrinkTemp)
                * (bottomEdgeShrunk - topEdgeShrunk);
            int dShrinkLostArea = (bottomEdgeShrunk - bottomShrinkTemp)
                * (rightEdgeShrunk - leftEdgeShrunk);
            if (rShrinkLostArea > dShrinkLostArea) {
              // We lose more area shrinking the right side, so shrink the bottom side
              bottomEdgeShrunk = bottomShrinkTemp;
            } else {
              // We lose more area shrinking the bottom side, so shrink the right side
              rightEdgeShrunk = rightShrinkTemp;
            }
          }
        }
      }
    }
  }
  // Check top left corner
  for (int y = topEdge; y >= 0; y--) {
    for (int x = leftEdge; x >= 0; x--) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        if ((leftEdgeShrunk - x) * pixDist < numerator
            && (topEdgeShrunk - y) * pixDist < numerator) {
          // Both left and top edges could shrink
          int leftShrinkTemp = x + int(numerator / pixDist);
          int topShrinkTemp = y + int(numerator / pixDist);
          if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer
              && y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking either edge makes the pyramid exclude the starting point
            return false;
          } else if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking left edge makes pyramid exclude the starting point, so shrink the top edge
            topEdgeShrunk = topShrinkTemp;
          } else if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking top edge makes pyramid exclude the starting point, so shrink the left edge
            leftEdgeShrunk = leftShrinkTemp;
          } else {
            // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
            int lShrinkLostArea = (leftShrinkTemp - leftEdgeShrunk)
                * (bottomEdgeShrunk - topEdgeShrunk);
            int uShrinkLostArea = (topShrinkTemp - topEdgeShrunk)
                * (rightEdgeShrunk - leftEdgeShrunk);
            if (lShrinkLostArea > uShrinkLostArea) {
              // We lose more area shrinking the left side, so shrink the top side
              topEdgeShrunk = topShrinkTemp;
            } else {
              // We lose more area shrinking the top side, so shrink the left side
              leftEdgeShrunk = leftShrinkTemp;
            }
          }
        }
      }
    }
  }
  // Check bottom left corner
  for (int y = bottomEdge; y < _imageHeight; y++) {
    for (int x = leftEdge; x >= 0; x--) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        if ((leftEdgeShrunk - x) * pixDist < numerator
            && numerator > (y - bottomEdgeShrunk) * pixDist) {
          // Both left and bottom edges could shrink
          int leftShrinkTemp = x + int(numerator / pixDist);
          int bottomShrinkTemp = y - int(numerator / pixDist);
          if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer
              && y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking either edge makes the pyramid exclude the starting point
            return false;
          } else if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking left edge makes pyramid exclude the starting point, so shrink the bottom edge
            bottomEdgeShrunk = bottomShrinkTemp;
          } else if (y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking bottom edge makes pyramid exclude the starting point, so shrink the left edge
            leftEdgeShrunk = leftShrinkTemp;
          } else {
            // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
            int lShrinkLostArea = (leftShrinkTemp - leftEdgeShrunk)
                * (bottomEdgeShrunk - topEdgeShrunk);
            int dShrinkLostArea = (bottomEdgeShrunk - bottomShrinkTemp)
                * (rightEdgeShrunk - leftEdgeShrunk);
            if (lShrinkLostArea > dShrinkLostArea) {
              // We lose more area shrinking the left side, so shrink the bottom side
              bottomEdgeShrunk = bottomShrinkTemp;
            } else {
              // We lose more area shrinking the bottom side, so shrink the left side
              leftEdgeShrunk = leftShrinkTemp;
            }
          }
        }
      }
    }
  }

  int edgesFinal[4] = { rightEdgeShrunk, topEdgeShrunk, leftEdgeShrunk,
      bottomEdgeShrunk };
  double depth = maxDepthExpandedPyramid * _depthScale
      - _vehicleRadiusForPlanning;

  // Create a new pyramid
  Vec3 corners[4];
  // Top right
  DeprojectPixelToPoint(double(edgesFinal[0]), double(edgesFinal[1]), depth,
                        corners[0]);
  // Top left
  DeprojectPixelToPoint(double(edgesFinal[2]), double(edgesFinal[1]), depth,
                        corners[1]);
  // Bottom left
  DeprojectPixelToPoint(double(edgesFinal[2]), double(edgesFinal[3]), depth,
                        corners[2]);
  // Bottom right
  DeprojectPixelToPoint(double(edgesFinal[0]), double(edgesFinal[3]), depth,
                        corners[3]);
  outPyramid = Pyramid(depth, edgesFinal, corners);

  return true;
}
