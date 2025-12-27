// Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>. Modified by Lihan Chen
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

#include "common_lib.hpp"
#include "memory"

#ifndef ORIGIN_AT_CORNER
#ifndef ORIGIN_AT_CENTER
#error "Please define either ORIGIN_AT_CORNER or ORIGIN_AT_CENTER, but not both."
#endif
#endif

#ifdef ORIGIN_AT_CORNER
#ifdef ORIGIN_AT_CENTER
#error \
  "Cannot use both ORIGIN_AT_CENTER and ORIGIN_AT_CENTER at the same time. Please define only one."
#endif
#endif

namespace rog_map
{

namespace raycaster
{
template <typename T>
int signum(T val)
{
  return (T(0) < val) - (val < T(0));
}

class RayCaster
{
public:
  typedef std::shared_ptr<RayCaster> Ptr;

  explicit RayCaster() = default;

  void setResolution(const double & resolution);

  RayCaster(const double & resolution);

  ~RayCaster() = default;

  void posToIndex(const double d, int & id) const;

  void indexToPos(const int & id, double & d) const;

  bool setInput(const Eigen::Vector3d & start, const Eigen::Vector3d & end);

  bool step(Eigen::Vector3d & ray_pt);

private:
  double resolution_{-1};
  bool first_point{true};
  // progress variables
  double start_x_d_, start_y_d_, start_z_d_;
  double end_x_d_, end_y_d_, end_z_d_;
  double t_to_bound_x_, t_to_bound_y_, t_to_bound_z_;
  int expand_dir_x_, expand_dir_y_, expand_dir_z_;
  int end_x_i_, end_y_i_, end_z_i_;
  int start_x_i_, start_y_i_, start_z_i_;
  int cur_ray_pt_id_x_, cur_ray_pt_id_y_, cur_ray_pt_id_z_;
  double t_when_step_x_, t_when_step_y_, t_when_step_z_;
  int step_num_{0};
};
}  // namespace raycaster
}  // namespace rog_map
