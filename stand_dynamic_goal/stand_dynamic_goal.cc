/**
 * @file stand_dynamic_goal.cc
 * @brief Implementation of the dynamic goal humanoid stand task in MuJoCo MPC.
 *
 * This task is an extension of the standard humanoid stand task where the goal 
 * position dynamically alternates between two predefined points (left and right) 
 * every 5 seconds. The humanoid must remain balanced while shifting between these 
 * goals, making it a more challenging control problem.
 */

#include "mjpc/tasks/humanoid/stand_dynamic_goal/stand_dynamic_goal.h"

#include <cmath>
#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {
namespace humanoid_dynamic_goal {

/**
 * @brief Destructor for StandDynamicGoal.
 *
 * Ensures proper cleanup of dynamically allocated resources if necessary.
 */
StandDynamicGoal::~StandDynamicGoal() = default;

/**
 * @brief Returns the XML file path for the task configuration.
 *
 * @return std::string - Path to the XML configuration file defining the humanoid model and environment.
 */
std::string StandDynamicGoal::XmlPath() const {
  return GetModelPath("humanoid/stand_dynamic_goal/task.xml");
}

/**
 * @brief Returns the name of the task.
 *
 * @return std::string - Task name used for identification in the MuJoCo MPC framework.
 */
std::string StandDynamicGoal::Name() const { 
  return "Humanoid Stand Dynamic Goal"; 
}

/**
 * @brief Computes the residuals for the dynamic goal humanoid stand task.
 *
 * Residuals quantify deviations from the task objectives. These residuals are used 
 * by the MuJoCo MPC optimization framework to compute control actions.
 *
 * Residuals in this task:
 *  - Residual (0): Height error (desired vs actual)
 *  - Residual (1): Balance error (COM xy deviation from feet)
 *  - Residual (2,3): COM velocity residual (should be zero)
 *  - Residual (4..NV-6): Joint velocity residual (should be minimized)
 *  - Residual (NV-6..NV+NU): Control effort residual (should be minimized)
 *  - Residual (last): Distance to dynamic goal (newly added)
 *
 * @param model Pointer to the MuJoCo model.
 * @param data Pointer to the MuJoCo data structure (current simulation state).
 * @param residual Pointer to an array where residual values will be stored.
 */
void StandDynamicGoal::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                            double* residual) const {
  int counter = 0;  // Residual index tracker

  // 🔹 Access the task instance to modify goal-tracking variables
  auto* humanoid_task = const_cast<StandDynamicGoal*>(static_cast<const StandDynamicGoal*>(task_));

  // 🔹 Retrieve the current simulation time
  double current_time = data->time;

  // 🔹 Define left and right goal positions in x-direction
  double left_goal_x = -0.2;   // Left position
  double right_goal_x = 0.2;   // Right position

  // 🔹 Switch goal every 5 seconds
  if (current_time - humanoid_task->elapsed_time_ > 5.0) {
    humanoid_task->elapsed_time_ = current_time;
    humanoid_task->goal_state_ = 1 - humanoid_task->goal_state_;  // Toggle between 0 (left) and 1 (right)
  }

  // 🔹 Determine the current target position
  double dynamic_goal_x = (humanoid_task->goal_state_ == 0) ? left_goal_x : right_goal_x;

  // ------------------ Compute Residuals ------------------

  // 🔹 Height residual: head-to-feet vertical error
  double* f1_position = SensorByName(model, data, "sp0");
  double* f2_position = SensorByName(model, data, "sp1");
  double* f3_position = SensorByName(model, data, "sp2");
  double* f4_position = SensorByName(model, data, "sp3");
  double* head_position = SensorByName(model, data, "head_position");

  double head_feet_error =
      head_position[2] - 0.25 * (f1_position[2] + f2_position[2] +
                                 f3_position[2] + f4_position[2]);
  
  residual[counter++] = head_feet_error - parameters_[0];  // Desired height residual

  // 🔹 Balance residual: Center of mass (COM) deviation from feet
  double* com_position = SensorByName(model, data, "torso_subtreecom");
  double* com_velocity = SensorByName(model, data, "torso_subtreelinvel");

  // Compute average foot position in xy-plane
  double fxy_avg[2] = {0.0};
  mju_addTo(fxy_avg, f1_position, 2);
  mju_addTo(fxy_avg, f2_position, 2);
  mju_addTo(fxy_avg, f3_position, 2);
  mju_addTo(fxy_avg, f4_position, 2);
  mju_scl(fxy_avg, fxy_avg, 0.25, 2);

  // Compute balance residual (COM vs feet average position)
  mju_subFrom(fxy_avg, com_position, 2);
  double com_feet_distance = mju_norm(fxy_avg, 2);
  residual[counter++] = com_feet_distance;

  // 🔹 COM velocity residual: should be zero
  mju_copy(&residual[counter], com_velocity, 2);
  counter += 2;

  // 🔹 Joint velocity residual: minimize joint motion
  mju_copy(residual + counter, data->qvel + 6, model->nv - 6);
  counter += model->nv - 6;

  // 🔹 Control effort residual: minimize applied forces
  mju_copy(&residual[counter], data->ctrl, model->nu);
  counter += model->nu;

  // ------------------ New Residual for Dynamic Goal ------------------

  // 🔹 Residual: Distance to dynamically changing goal position
  residual[counter++] = com_position[0] - dynamic_goal_x;
}

}  // namespace humanoid_dynamic_goal
}  // namespace mjpc


