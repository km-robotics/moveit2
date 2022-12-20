/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Jack Center, Wyatt Rees, Andy Zelenak, Stephanie Eng */

#pragma once

#include <Eigen/Core>
#include <list>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ruckig/ruckig.hpp>

namespace trajectory_processing
{
class RuckigGenerator
{
public:
  /**
   * \brief Generate a trajectory between two arbitrary states
   */
  static bool generate(robot_trajectory::RobotTrajectory& trajectory,
      double sampling_time,
      double max_velocity_scaling_factor = 1.0,
      double max_acceleration_scaling_factor = 1.0);

private:
  /**
   * \brief Initialize Ruckig position/vel/accel. This initializes ruckig_input and ruckig_output to the same values
   * \param trajectory      Robot trajectory
   * \param rucking_input   Input parameters to Ruckig.
   * \param ruckig_output   Output from the Ruckig algorithm.
   */
  static void prepareRuckig(const robot_trajectory::RobotTrajectory& trajectory,
      double max_velocity_scaling_factor,
      double max_acceleration_scaling_factor,
      ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input,
      ruckig::OutputParameter<ruckig::DynamicDOFs>& ruckig_output);

  /**
   * \brief Feed previous output back as input for next iteration. Get next target state from the next waypoint.
   * \param current_waypoint    The nominal current state
   * \param next_waypoint       The nominal, desired state at the next waypoint
   * \param joint_group         The MoveIt JointModelGroup of interest
   * \param ruckig_input        Output. The Rucking parameters for the next iteration
   */
  static void getNextRuckigInput(const moveit::core::RobotStateConstPtr& current_waypoint,
                                 const moveit::core::RobotStateConstPtr& next_waypoint,
                                 const moveit::core::JointModelGroup* joint_group,
                                 ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input);

  /**
   * \brief Initialize Ruckig position/vel/accel. This initializes ruckig_input and ruckig_output to the same values
   * \param rucking_input   Input parameters to Ruckig. Initialized here.
   * \param ruckig_output   Output from the Ruckig algorithm. Initialized here.
   * \param first_waypoint  The Ruckig input/output parameters are initialized to the values at this waypoint
   * \param joint_group     The MoveIt JointModelGroup of interest
   */
  static void initializeRuckigState(ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input,
                                    ruckig::OutputParameter<ruckig::DynamicDOFs>& ruckig_output,
                                    const moveit::core::RobotState& first_waypoint,
                                    const moveit::core::JointModelGroup* joint_group);

  static void addWayPoint(robot_trajectory::RobotTrajectory& trajectory,
                          const ruckig::OutputParameter<ruckig::DynamicDOFs>& ruckig_output,
                          double time_previous_wp);

  /**
   * \brief Compute how long it takes to go from `previous` to `current` with the velocity of `previous`.
   */
  static double getWayPointDurationFromPrevious(const moveit::core::RobotStateConstPtr& previous,
                                                const moveit::core::RobotStateConstPtr& current,
                                                double min_duration);
};

}  // namespace trajectory_processing
