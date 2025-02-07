#ifndef MJPC_TASKS_HUMANOID_STAND_DYNAMIC_GOAL_H_
#define MJPC_TASKS_HUMANOID_STAND_DYNAMIC_GOAL_H_

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/tasks/humanoid/stand/stand.h"

namespace mjpc {
namespace humanoid_dynamic_goal {

class StandDynamicGoal : public humanoid::Stand {  // 🔹 Explicitly reference humanoid::Stand
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  class ResidualFn : public humanoid::Stand::ResidualFn {  // 🔹 Explicitly reference humanoid::Stand::ResidualFn
   public:
    explicit ResidualFn(const StandDynamicGoal* task) : humanoid::Stand::ResidualFn(task) {}
    void Residual(const mjModel* model, const mjData* data, double* residual) const override;
  };

  StandDynamicGoal() : residual_(this) {}

  ~StandDynamicGoal() override;  // 🔹 Explicit destructor

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(this);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  ResidualFn residual_;

  // 🔹 Track elapsed time for goal switching
  mutable double elapsed_time_ = 0.0;

  // 🔹 Track which goal the humanoid is currently moving towards (0 = Left, 1 = Right)
  mutable int goal_state_ = 0;
};

}  // namespace humanoid_dynamic_goal
}  // namespace mjpc

#endif  // MJPC_TASKS_HUMANOID_STAND_DYNAMIC_GOAL_H_

