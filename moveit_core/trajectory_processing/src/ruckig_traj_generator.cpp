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

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <limits>
#include <moveit/trajectory_processing/ruckig_traj_generator.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace trajectory_processing
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_trajectory_processing.ruckig_traj_generator");
constexpr double DEFAULT_MAX_VELOCITY = 5;       // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 10;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = 1000;        // rad/s^3

}  // namespace

bool RuckigGenerator::generate(robot_trajectory::RobotTrajectory& trajectory,
    double sampling_time,
    double max_velocity_scaling_factor,
    double max_acceleration_scaling_factor)
{
  moveit::core::JointModelGroup const* const group = trajectory.getGroup();
  if (!group)
  {
    RCLCPP_ERROR(LOGGER, "It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  const size_t num_waypoints = trajectory.getWayPointCount();
  if (num_waypoints < 2)
  {
    RCLCPP_WARN(LOGGER,
                "Trajectory does not have enough points to generate with Ruckig. Returning an unmodified trajectory.");
    return true;
  }

  if (num_waypoints != 2)
  {
    RCLCPP_FATAL(LOGGER, "Trajectory with intermediate points not supported yet");
    return false;
  }

  // Cache the trajectory in case we need to reset it
  robot_trajectory::RobotTrajectory original_trajectory =
      robot_trajectory::RobotTrajectory(trajectory, true /* deep copy */);

  const std::size_t dof = group->getVariableCount();

  // This lib does not actually work properly when angles wrap around, so we need to unwind the path first
  trajectory.unwind();

  // Instantiate the smoother
  std::unique_ptr<ruckig::Ruckig<ruckig::DynamicDOFs>> ruckig_ptr;
  ruckig_ptr = std::make_unique<ruckig::Ruckig<ruckig::DynamicDOFs>>(dof, sampling_time);
  ruckig::InputParameter<ruckig::DynamicDOFs> ruckig_input{dof};
  ruckig::OutputParameter<ruckig::DynamicDOFs> ruckig_output{dof};

  // Initialize the smoother
  prepareRuckig(trajectory, max_velocity_scaling_factor, max_acceleration_scaling_factor, ruckig_input, ruckig_output);
  getNextRuckigInput(trajectory.getFirstWayPointPtr(), trajectory.getLastWayPointPtr(), group, ruckig_input);

  // Reset the trajectory that will be replaced by the generated one.
  const auto first_waypoint = trajectory.getFirstWayPointPtr();
  const auto first_dt = trajectory.getWayPointDurationFromPrevious(0);
  trajectory.clear();
  trajectory.addSuffixWayPoint(first_waypoint, first_dt);

  double time_previous_wp = 0.0;
  ruckig::Result update_result;
  while ((update_result = ruckig_ptr->update(ruckig_input, ruckig_output)) == ruckig::Result::Working)
  {
    addWayPoint(trajectory, ruckig_output, time_previous_wp);
    ruckig_output.pass_to_input(ruckig_input);
    time_previous_wp = ruckig_output.time;
  }

  if (not((update_result == ruckig::Result::Working)
          or (update_result == ruckig::Result::Finished)))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Ruckig trajectory smoothing failed. Ruckig error: " << update_result);
    return false;
  }

  // The target point is not part of the trajectory generated by Ruckig, add it.
  const double dt = getWayPointDurationFromPrevious(trajectory.getLastWayPointPtr(), original_trajectory.getLastWayPointPtr(), 0.001);
  if (dt > 0.0)
  {
    trajectory.addSuffixWayPoint(original_trajectory.getLastWayPointPtr(), dt);
  }
  return true;
}

void RuckigGenerator::initializeRuckigState(ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input,
                                            ruckig::OutputParameter<ruckig::DynamicDOFs>& ruckig_output,
                                            const moveit::core::RobotState& first_waypoint,
                                            const moveit::core::JointModelGroup* joint_group)
{
  const std::size_t dof = joint_group->getVariableCount();
  const std::vector<int>& idx = joint_group->getVariableIndexList();

  std::vector<double> current_positions_vector(dof);
  std::vector<double> current_velocities_vector(dof);
  std::vector<double> current_accelerations_vector(dof);

  for (std::size_t j = 0; j < dof; ++j)
  {
    current_positions_vector.at(j) = first_waypoint.getVariablePosition(idx.at(j));
    current_velocities_vector.at(j) = first_waypoint.getVariableVelocity(idx.at(j));
    current_accelerations_vector.at(j) = first_waypoint.getVariableAcceleration(idx.at(j));
    // Clamp velocities/accelerations in case they exceed the limit due to small numerical errors
    current_velocities_vector.at(j) =
        std::clamp(current_velocities_vector.at(j), -ruckig_input.max_velocity.at(j), ruckig_input.max_velocity.at(j));
    current_accelerations_vector.at(j) = std::clamp(
        current_accelerations_vector.at(j), -ruckig_input.max_acceleration.at(j), ruckig_input.max_acceleration.at(j));
  }
  std::copy_n(current_positions_vector.begin(), dof, ruckig_input.current_position.begin());
  std::copy_n(current_velocities_vector.begin(), dof, ruckig_input.current_velocity.begin());
  std::copy_n(current_accelerations_vector.begin(), dof, ruckig_input.current_acceleration.begin());
  // Initialize output data struct
  ruckig_output.new_position = ruckig_input.current_position;
  ruckig_output.new_velocity = ruckig_input.current_velocity;
  ruckig_output.new_acceleration = ruckig_input.current_acceleration;
}

void RuckigGenerator::prepareRuckig(const robot_trajectory::RobotTrajectory& trajectory,
      double max_velocity_scaling_factor,
      double max_acceleration_scaling_factor,
      ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input,
      ruckig::OutputParameter<ruckig::DynamicDOFs>& ruckig_output)
{
  moveit::core::JointModelGroup const* const group = trajectory.getGroup();
  const std::size_t dof = group->getVariableCount();

  // kinematic limits (vel/accel/jerk)
  const std::vector<std::string>& vars = group->getVariableNames();
  const moveit::core::RobotModel& rmodel = group->getParentModel();
  // const std::vector<int>& move_group_idx = group->getvariableindexlist();
  for (size_t i = 0; i < dof; ++i)
  {
    const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars.at(i));

    // this assumes min/max bounds are symmetric
    if (bounds.velocity_bounded_)
    {
      ruckig_input.max_velocity.at(i) = max_velocity_scaling_factor * bounds.max_velocity_;
      RCLCPP_DEBUG_STREAM(LOGGER, "ruckig_input.max_velocity.at(" << i << "): " << ruckig_input.max_velocity.at(i));
    }
    else
    {
      RCLCPP_WARN_STREAM_ONCE(LOGGER,
                              "Joint velocity limits are not defined. Using the default "
                                  << DEFAULT_MAX_VELOCITY
                                  << " rad/s. You can define velocity limits in the URDF or joint_limits.yaml.");
      ruckig_input.max_velocity.at(i) = max_velocity_scaling_factor * DEFAULT_MAX_VELOCITY;
    }
    if (bounds.acceleration_bounded_)
    {
      ruckig_input.max_acceleration.at(i) = max_acceleration_scaling_factor * bounds.max_acceleration_;
      RCLCPP_DEBUG_STREAM(LOGGER, "ruckig_input.max_acceleration.at(" << i << "): " << ruckig_input.max_acceleration.at(i));
    }
    else
    {
      RCLCPP_WARN_STREAM_ONCE(LOGGER,
                              "Joint acceleration limits are not defined. Using the default "
                                  << DEFAULT_MAX_ACCELERATION
                                  << " rad/s^2. You can define acceleration limits in the URDF or joint_limits.yaml.");
      ruckig_input.max_acceleration.at(i) = max_acceleration_scaling_factor * DEFAULT_MAX_ACCELERATION;
    }
    ruckig_input.max_jerk.at(i) = bounds.jerk_bounded_ ? bounds.max_jerk_ : DEFAULT_MAX_JERK;
    if (bounds.jerk_bounded_)
    {
      ruckig_input.max_jerk.at(i) = bounds.max_jerk_;
    }
    else
    {
      RCLCPP_WARN_STREAM_ONCE(LOGGER, "Joint jerk limits are not defined. Using the default "
                                          << DEFAULT_MAX_JERK
                                          << " rad/s^3. You can define jerk limits in joint_limits.yaml.");
      ruckig_input.max_jerk.at(i) = DEFAULT_MAX_JERK;
    }
  }

  // TODO: add intermediate waypoints.
  initializeRuckigState(ruckig_input, ruckig_output, trajectory.getFirstWayPoint(), group);
}

void RuckigGenerator::getNextRuckigInput(const moveit::core::RobotStateConstPtr& current_waypoint,
                                         const moveit::core::RobotStateConstPtr& next_waypoint,
                                         const moveit::core::JointModelGroup* joint_group,
                                         ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input)
{
  const std::size_t dof = joint_group->getVariableCount();
  const std::vector<int>& idx = joint_group->getVariableIndexList();

  for (size_t joint = 0; joint < dof; ++joint)
  {
    ruckig_input.current_position.at(joint) = current_waypoint->getVariablePosition(idx.at(joint));
    ruckig_input.current_velocity.at(joint) = current_waypoint->getVariableVelocity(idx.at(joint));
    ruckig_input.current_acceleration.at(joint) = current_waypoint->getVariableAcceleration(idx.at(joint));

    // Target state is the next waypoint
    ruckig_input.target_position.at(joint) = next_waypoint->getVariablePosition(idx.at(joint));
    ruckig_input.target_velocity.at(joint) = next_waypoint->getVariableVelocity(idx.at(joint));
    ruckig_input.target_acceleration.at(joint) = next_waypoint->getVariableAcceleration(idx.at(joint));

    // Clamp velocities/accelerations in case they exceed the limit due to small numerical errors
    ruckig_input.current_velocity.at(joint) =
        std::clamp(ruckig_input.current_velocity.at(joint), -ruckig_input.max_velocity.at(joint),
                   ruckig_input.max_velocity.at(joint));
    ruckig_input.current_acceleration.at(joint) =
        std::clamp(ruckig_input.current_acceleration.at(joint), -ruckig_input.max_acceleration.at(joint),
                   ruckig_input.max_acceleration.at(joint));
    ruckig_input.target_velocity.at(joint) =
        std::clamp(ruckig_input.target_velocity.at(joint), -ruckig_input.max_velocity.at(joint),
                   ruckig_input.max_velocity.at(joint));
    ruckig_input.target_acceleration.at(joint) =
        std::clamp(ruckig_input.target_acceleration.at(joint), -ruckig_input.max_acceleration.at(joint),
                   ruckig_input.max_acceleration.at(joint));
  }
}

void RuckigGenerator::addWayPoint(robot_trajectory::RobotTrajectory& trajectory,
                                  const ruckig::OutputParameter<ruckig::DynamicDOFs>& ruckig_output,
                                  double time_previous_wp)
{
  auto wp = moveit::core::RobotState{trajectory.getRobotModel()};

  const std::vector<int>& move_group_idx = trajectory.getGroup()->getVariableIndexList();
  for (std::size_t j = 0; j < move_group_idx.size(); ++j)
  {
    wp.setVariablePosition(move_group_idx.at(j), ruckig_output.new_position.at(j));
    wp.setVariableVelocity(move_group_idx.at(j), ruckig_output.new_velocity.at(j));
    wp.setVariableAcceleration(move_group_idx.at(j), ruckig_output.new_acceleration.at(j));
  }

  trajectory.addSuffixWayPoint(wp, ruckig_output.time - time_previous_wp);
}

double RuckigGenerator::getWayPointDurationFromPrevious(const moveit::core::RobotStateConstPtr& previous,
      const moveit::core::RobotStateConstPtr& current,
      double min_duration)
{
  const std::size_t dof = previous->getVariableCount();
  double sum_dt = 0.0;
  unsigned int count_dt = 0;
  for (std::size_t j = 0; j < dof; ++j)
  {
    const double dq = current->getVariablePosition(j) - previous->getVariablePosition(j);
    if ((std::abs(dq) < 1e-3) or (std::abs(previous->getVariableVelocity(j)) < 1e-3))
    {
      // Too small change, do not take into account.
      continue;
    }
    sum_dt += dq / previous->getVariableVelocity(j);
    count_dt++;
  }

  if (count_dt > 0)
  {
    const double mean_dt = sum_dt / static_cast<double>(count_dt);
    if (mean_dt > min_duration)
    {
      return mean_dt;
    }
  }

  return 0.0;
}

}  // namespace trajectory_processing
