// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef MJPC_TASKS_HUMANOID_INTERACT_TASK_H_
#define MJPC_TASKS_HUMANOID_INTERACT_TASK_H_

#include <cmath>
#include <string>
#include <vector>
#include <cassert>
#include "mjpc/task.h"
#include "mjpc/utilities.h"
#include "contact_keyframe.h"
#include "motion_strategy.h"

namespace mjpc {
namespace humanoid {

// ---------- Constants ----------------- //
constexpr int NOT_FOUND = -1;
constexpr int RESIDUAL_TERMS_COUNT = 2;
constexpr int CONTACT_POINT_PARAM_INDEX = RESIDUAL_TERMS_COUNT + CONTACT_PAIR_COUNT;

constexpr int HEAD_HEIGHT_PARAM_INDEX = 0;
constexpr int TORSO_HEIGHT_PARAM_INDEX = 1;
constexpr int DIST_TOLERANCE_INDEX = 9;
constexpr int TIME_LIMIT_INDEX = 10;
constexpr int SUSTAIN_TIME = 11;

constexpr int SCENE_FREE_JOINT_COUNT = 0;

// ---------- Enums ----------------- //
enum TaskMode: int {
  kSitting = 0,
  kStanding = 1,
  kRelaxing = 2,
  kStraightLegs = 3,
  kLieDown = 4
};

enum ContactMode: int {
  kNoContact = 0,
  kContact = 1
};

// ----------- Default weights for the residual terms ----------------- //
const std::vector<std::vector<double>> default_weights = {{10,10,5,5,0,20,30,0,0,0,0.01,.1,80.}, // to sit
                                                          {10,0,1,1,80,0,0,100,0,0,0.01,0.025,0.}, // to stand
                                                          {0,0,0,0,0,0,0,0,0,0,0.01,.8,80.}, // relax
                                                          {0,0,0,0,0,0,0,0,0,50,20,.025,80.}, // stay still
};

class Interact : public Task {
 public:
  class ResidualFn : public mjpc::BaseResidualFn {
   public:
    explicit ResidualFn(const Interact* task, 
                        const ContactKeyframe& kf = ContactKeyframe(),
                        int current_mode = kSitting,
                        int current_contact_mode = kNoContact)
        : mjpc::BaseResidualFn(task),
          residual_keyframe_(kf),
          current_mode_((TaskMode)current_mode),
          current_contact_mode_((ContactMode)current_contact_mode) {
    }

    // ------------------ Residuals for interaction task ------------
    void Residual(const mjModel* model, const mjData* data,
                double* residual) const override;

   protected:
    ContactKeyframe residual_keyframe_;

   private:
    friend class Interact;
    
    TaskMode current_mode_;
    ContactMode current_contact_mode_;

    void UpResidual(const mjModel* model, const mjData* data,
                  double* residual, std::string&& name, int& counter) const;
                  
    void HeadHeightResidual(const mjModel* model, const mjData* data,
                  double* residual, int& counter) const;

    void TorsoHeightResidual(const mjModel* model, const mjData* data,
                  double* residual, int& counter) const;

    void KneeFeetXYResidual(const mjModel* model, const mjData* data,
                  double* residual, int& counter) const;

    void COMFeetXYResidual(const mjModel* model, const mjData* data,
                  double* residual, int& counter) const;

    void TorsoTargetResidual(const mjModel* model, const mjData* data,
                  double* residual, int& counter) const;

    void FacingDirResidual(const mjModel* model, const mjData* data,
                  double* residual, int& counter) const;

    void ContactResidual(const mjModel* model, const mjData* data,
                  double* residual, int& counter) const;
  };

  Interact() : residual_(this) {}

  void TransitionLocked(mjModel* model, mjData* data) override;

  std::string Name() const override;
  std::string XmlPath() const override;


  // ------------------ Handling keyframes ------------ 
  void AddKeyframe();
  void NextKeyframe();
  void RemoveKeyframe();
  void ClearKeyframes();
  void EditKeyframe();
  void SaveKeyframe();
  void LoadKeyframe();
  void SaveKeyframeSequence();
  void LoadKeyframeSequence();

  void SyncWeightsToKeyframe(ContactKeyframe& kf) const;
  void SyncWeightsFromKeyframe(const ContactKeyframe& kf);

  void SetSelectedPoint(const mjtNum* selpos, const mjtNum selbody,
                        const mjtNum selgeom);
  void SetContactKeyframes(const std::vector<ContactKeyframe>& keyframes, const int current_index);
  
  double CalculatePushForce(const mjModel* model, const mjData* data, const int geom1, const int geom2) const;
  double CalculateKeyframeError(const ContactKeyframe& keyframe, mjData* data) const;

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(this, 
                                        residual_.residual_keyframe_,
                                        residual_.current_mode_,
                                        residual_.current_contact_mode_);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  ResidualFn residual_;

  // draw task-related geometry in the scene
  void ModifyScene(const mjModel* model, const mjData* data,
                   mjvScene* scene) const override;
  double GetKeyframeDuration(const mjData* data) const { return data->time - kf_start_time; }
  double GetSuccessSustainTime(const mjData* data) const { return data->time - first_success_time; }
                   
  bool kf_index_updated;

  double kf_start_time = 0.; 
  double first_success_time = 0.;
  double kf_distance_error = 0.;
};

}  // namespace humanoid
}  // namespace mjpc

#endif  // MJPC_TASKS_HUMANOID_INTERACT_TASK_H_
