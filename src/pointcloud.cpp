/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * https://octomap.github.io/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * Copyright (c) 2023 Davide Faconti
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "bonxai_map/pointcloud.hpp"
#include <eigen3/Eigen/Core>

namespace Bonxai
{
bool ComputeRay(const Eigen::Vector3d &origin,
                const Eigen::Vector3d &end,
                const double resolution,
                std::vector<CoordT> &ray)
{
  const double inv_resolution = 1.0/resolution;
  CoordT key_origin = PosToCoord(ToPoint3D(origin), inv_resolution);
  CoordT key_end = PosToCoord(ToPoint3D(end), inv_resolution);

  ray.clear();

  if (key_origin == key_end)
  {
    return true; // same tree cell, we're done.
  }

  ray.push_back( key_origin );

  // Initialization phase

  Eigen::Vector3d direction =  end - origin;
  const double length = direction.norm();
  direction /= length; // normalize vector

  auto ToPos =[&resolution](double coord) {
    return (coord - 0.5) * resolution;
  };

  //------- Initialization phase -------

  Eigen::Vector3i step;
  Eigen::Vector3d tMax;
  Eigen::Vector3d tDelta;

  CoordT current_key = key_origin;

  for(unsigned int i=0; i < 3; ++i) {
    // compute step direction
    if (direction(i) > 0.0) {
      step[i] =  1;
    }
    else if (direction(i) < 0.0) {
      step[i] = -1;
    }
    else {
      step[i] = 0;
    }

    // compute tMax, tDelta
    if (step[i] != 0) {
      // corner point of voxel (in direction of ray)
      double voxelBorder = ToPos(current_key[i]);
      voxelBorder += (step[i] * resolution * 0.5);

      tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
      tDelta[i] = resolution / fabs( direction(i) );
    }
    else {
      tMax[i] =  std::numeric_limits<double>::max( );
      tDelta[i] = std::numeric_limits<double>::max( );
    }
  }

  //------- Incremental phase  -------
  bool done = false;
  while (!done)
  {
    unsigned dim;

    // find minimum tMax:
    if (tMax[0] < tMax[1]){
      if (tMax[0] < tMax[2]){ dim = 0;}
      else                  { dim = 2;}
    }
    else {
      if (tMax[1] < tMax[2]){ dim = 1;}
      else                  { dim = 2;}
    }

    // advance in direction "dim"
    current_key[dim] += step[dim];
    tMax[dim] += tDelta[dim];

    assert (current_key[dim] < 2*this->tree_max_val);

            // reached endpoint, key equv?
    if (current_key == key_end) {
      done = true;
      break;
    }
    else {

      // reached endpoint world coords?
      // dist_from_origin now contains the length of the ray
      // when traveled until the border of the current voxel
      double dist_from_origin = std::min(std::min(tMax[0], tMax[1]), tMax[2]);
      // if this is longer than the expected ray length, we should have
      // already hit the voxel containing the end point with the code above (key_end).
      // However, we did not hit it due to accumulating discretization errors,
      // so this is the point here to stop the ray as we would never reach the voxel key_end
      if (dist_from_origin > length) {
        break;
      }

      else {  // continue to add freespace cells
        ray.push_back(current_key);
      }
    }
  } // end while

  return true;
}

}

