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

#include "mjpc/tasks/humanoid/interact/interact.h"

#include <cstddef>
#include <iostream>
#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc::humanoid {

std::string Interact::XmlPath() const {
  return GetModelPath("humanoid/interact/task.xml");
  // return SOURCE_DIR "/mjpc/tasks/humanoid/interact/task.xml";
}
std::string Interact::Name() const { return "Humanoid Interact"; }

// ------------------ Residuals for humanoid contact keyframes task ------------
void Interact::ResidualFn::UpResidual(const mjModel* model, const mjData* data,
              double* residual, std::string&& name, int& counter) const {
  name.append("_up");
  double* up_vector = SensorByName(model, data, name);
  residual[counter++] = mju_abs(up_vector[2] - 1.0);
}

void Interact::ResidualFn::HeadHeightResidual(const mjModel* model, const mjData* data,
              double* residual, int& counter) const {
  double head_height = SensorByName(model, data, "head_position")[2];
  residual[counter++] = mju_abs(head_height - parameters_[HEAD_HEIGHT_PARAM_INDEX]);
}

void Interact::ResidualFn::TorsoHeightResidual(const mjModel* model, const mjData* data,
              double* residual, int& counter) const {
  double torso_height = SensorByName(model, data, "torso_position")[2];
  residual[counter++] = mju_abs(torso_height - parameters_[TORSO_HEIGHT_PARAM_INDEX]);
}

void Interact::ResidualFn::KneeFeetXYResidual(const mjModel* model, const mjData* data,
              double* residual, int& counter) const {
  double* knee_right = SensorByName(model, data, "knee_right");
  double* knee_left = SensorByName(model, data, "knee_left");
  double* foot_right = SensorByName(model, data, "foot_right");
  double* foot_left = SensorByName(model, data, "foot_left");
  
  double knee_xy_avg[2] = {0.0};
  mju_addTo(knee_xy_avg, knee_left, 2);
  mju_addTo(knee_xy_avg, knee_right, 2);
  mju_scl(knee_xy_avg, knee_xy_avg, 0.5, 2);

  double foot_xy_avg[2] = {0.0};
  mju_addTo(foot_xy_avg, foot_left, 2);
  mju_addTo(foot_xy_avg, foot_right, 2);
  mju_scl(foot_xy_avg, foot_xy_avg, 0.5, 2);

  mju_subFrom(knee_xy_avg, foot_xy_avg, 2);
  residual[counter++] = mju_norm(knee_xy_avg, 2);
}

void Interact::ResidualFn::COMFeetXYResidual(const mjModel* model, const mjData* data,
              double* residual, int& counter) const {
  double* foot_right = SensorByName(model, data, "foot_right");
  double* foot_left = SensorByName(model, data, "foot_left");
  double* com_position = SensorByName(model, data, "torso_subtreecom");
  
  double foot_xy_avg[2] = {0.0};
  mju_addTo(foot_xy_avg, foot_left, 2);
  mju_addTo(foot_xy_avg, foot_right, 2);
  mju_scl(foot_xy_avg, foot_xy_avg, 0.5, 2);

  mju_subFrom(com_position, foot_xy_avg, 2);
  residual[counter++] = mju_norm(com_position, 2);
}

void Interact::ResidualFn::FacingDirResidual(const mjModel* model, const mjData* data,
              double* residual, int& counter) const {
  if (residual_keyframe_.facing_target.empty()) {
    residual[counter++] = 0;
    return;
  }

  double* torso_forward = SensorByName(model, data, "torso_forward");
  double* torso_position = mjpc::SensorByName(model, data, "torso_position");
  double target[2] = {0.}; 

  mju_sub(target, residual_keyframe_.facing_target.data(), torso_position, 2);
  mju_normalize(target, 2);
  mju_subFrom(target, torso_forward, 2);
  residual[counter++] = mju_norm(target, 2);
}

void Interact::ResidualFn::ContactResidual(const mjModel* model, const mjData* data,
                double* residual, int& counter) const {
  for (int i=0; i<CONTACT_PAIR_COUNT; i++) {
    const ContactPair& contact = residual_keyframe_.contact_pairs[i];
    if (contact.body1 != NOT_SELECTED && contact.body2 != NOT_SELECTED) {
      // if part != -1 then check if the body id is in the body part's ids, 
      // if yes proceed and if not then set this residual to 0 and continue

      mjtNum tmp1[3];
      mjtNum selected_global_pos1[3];
      mju_mulMatVec(tmp1, 
                    data->xmat+9*contact.body1, 
                    contact.local_pos1, 3, 3);
      mju_add3(selected_global_pos1, tmp1, data->xpos+3*contact.body1);

      mjtNum tmp2[3];
      mjtNum selected_global_pos2[3];
      mju_mulMatVec(tmp2, 
                    data->xmat+9*contact.body2, 
                    contact.local_pos2, 3, 3);
      mju_add3(selected_global_pos2, tmp2, data->xpos+3*contact.body2);

      double dist[3] = {0., 0., 0.};
      mju_addTo(dist, selected_global_pos1, 3);
      mju_subFrom(dist, selected_global_pos2, 3);
      residual[counter++] = mju_abs(dist[0]);
      residual[counter++] = mju_abs(dist[1]);
      residual[counter++] = mju_abs(dist[2]);
    }
    else {
      for (int j=0; j<3; j++)
        residual[counter++] = 0;
    }
  } 
}

// ------------------ Residuals for humanoid interaction task ------------
//   Number of residuals: 13
//     Residual (0): Torso up
//     Residual (1): Pelvis up
//     Residual (2): Right foot up
//     Residual (3): Left foot up
//     Residual (4): Head height
//     Residual (5): Torso height
//     Residual (6): Knee feet xy-plane distance
//     Residual (7): COM feet xy-plane distance
//     Residual (8): Facing direction target xy-distance
//     Residual (9): Com Vel: should be 0 and equal feet average vel
//     Residual (10): Control: minimise control
//     Residual (11): Joint vel: minimise joint velocity
//     Residual (12): Contact: minimise distance between contact pairs
//   Number of parameters: 2
//     Parameter (0): head_height_goal
//     Parameter (1): torso_height_goal
// ----------------------------------------------------------------
void Interact::ResidualFn::Residual(const mjModel* model,
                               const mjData* data, double* residual) const {
  int counter = 0;

  double* com_velocity = SensorByName(model, data, "torso_subtreelinvel");

  // ----- task-specific residual terms ----- //
  UpResidual(model, data, residual, "torso", counter);
  UpResidual(model, data, residual, "pelvis", counter);
  UpResidual(model, data, residual, "foot_right", counter);
  UpResidual(model, data, residual, "foot_left", counter);
  HeadHeightResidual(model, data, residual, counter);
  TorsoHeightResidual(model, data, residual, counter);
  KneeFeetXYResidual(model, data, residual, counter);
  COMFeetXYResidual(model, data, residual, counter);
  FacingDirResidual(model, data, residual, counter);

  // ----- COM xy velocity should be 0 ----- //
  mju_copy(&residual[counter], com_velocity, 2);
  counter += 2;

  // ----- joint velocity ----- //
  mju_copy(residual + counter, data->qvel + 6, model->nv - (6 + SCENE_FREE_JOINT_COUNT * 6));
  counter += model->nv - (6 + SCENE_FREE_JOINT_COUNT * 6);

  // ----- action ----- //
  mju_copy(&residual[counter], data->ctrl, model->nu);
  counter += model->nu;

  // ----- contact ----- //
  if (current_contact_mode_ == kContact) {
    ContactResidual(model, data, residual, counter);
  }
  else {
    for (int i=0; i<CONTACT_PAIR_COUNT*3;i++)
      residual[counter++] = 0;
  }

  CheckSensorDim(model, counter);
}

// -------- Transition for interaction task --------
//   If humanoid is done with one objective switch to the next
// ---------------------------------------------
void Interact::TransitionLocked(mjModel* model, mjData* data) {

  ContactKeyframe& current_keyframe = motion_strategy_.GetCurrentKeyframe();

  // ---------------------------- Task mode handling -------------------------------------
  if (residual_.current_mode_ != mode) {
    residual_.current_mode_ = (TaskMode)mode;     
    if (kf_weights && !current_keyframe.weight.empty()) {
      SyncWeightsFromKeyframe(current_keyframe);
    }
    else {
      weight = default_weights[residual_.current_mode_];
    }
  }

  // ---------------------------- Contact mode handling -----------------------------------
  if (residual_.current_contact_mode_ != contact_mode) {
    residual_.current_contact_mode_ = (ContactMode)contact_mode;
    kf_index_updated = true;
    kf_index = 0;
  }

  // --------------------------- Keyframe sequence handling -------------------------------
  if (!kf_weights) {
      SyncWeightsToKeyframe(current_keyframe);
  }

  if (motion_strategy_.HasKeyframes()) {
    if ((ContactMode)contact_mode == kContact) {
      kf_distance_error = CalculateKeyframeError(current_keyframe, data);
       
      // the case where the episode fails due to time out, we restart the simulation and go to next episode
      if (GetPhaseDuration(data) > current_keyframe.time_limit &&
              kf_distance_error >= current_keyframe.target_distance_tolerance) {
        kf_index = 0;
        kf_index_updated = true;
      }
      // the case where the episode succeeds, we restart the simulation and go to next episode
      else if (kf_index == motion_strategy_.GetKeyframesCount() - 1 &&
          kf_distance_error <= current_keyframe.target_distance_tolerance &&
          GetSuccessSustainTime(data) >= current_keyframe.success_sustain_time) {
          kf_index = 0;
          kf_index_updated = true;
          first_success_time = data->time;
      }
      // the case where the current keyframe succeeds, we move on to the next keyframe within the same episode
      else if (kf_distance_error <= current_keyframe.target_distance_tolerance &&
          GetSuccessSustainTime(data) >= current_keyframe.success_sustain_time) {
        kf_index++;

        kf_index_updated = true;
        phase_start_time = data->time;
        first_success_time = data->time;
      }
      // the case where the distance error is more than the tolerance, we reset success timer
      else if (kf_distance_error > current_keyframe.target_distance_tolerance) {
        // reset success timer
        first_success_time = data->time;
      }
    }

    assert(kf_index < motion_strategy_.GetKeyframesCount());

    // --------------------------- Updating keyframe -------------------------------------------
    if (kf_index_updated) {
      // current_keyframe_ = contact_keyframes_.at(kf_index);
      motion_strategy_.UpdateCurrentKeyframe(kf_index);
      ContactKeyframe current_keyframe = motion_strategy_.GetCurrentKeyframe();
      residual_.residual_keyframe_ = current_keyframe;
      if (kf_weights && !current_keyframe.weight.empty()) {
        SyncWeightsFromKeyframe(current_keyframe);
      } else {
        SyncWeightsToKeyframe(current_keyframe);
      }
      kf_index_updated = false;
    }
  }
}

void Interact::AddKeyframe() {
  SyncWeightsToKeyframe(motion_strategy_.GetCurrentKeyframe());
  motion_strategy_.AddCurrentKeyframe(kf_name);
  kf_index_updated = true;
}

void Interact::NextKeyframe() {
  kf_index = motion_strategy_.NextKeyframe();
  // SyncWeightsFromKeyframe(motion_strategy_.GetCurrentKeyframe());
  kf_index_updated = true;
}

void Interact::RemoveKeyframe() {
  if (motion_strategy_.RemoveCurrentKeyframe()) {
    SyncWeightsFromKeyframe(motion_strategy_.GetCurrentKeyframe());
    kf_index = motion_strategy_.GetCurrentKeyframeIndex();
    kf_index_updated = true;
  }
}

void Interact::ClearKeyframes() {
  motion_strategy_.ClearKeyframes();
  kf_index = motion_strategy_.GetCurrentKeyframeIndex();
  residual_.residual_keyframe_.Reset();
  weight = default_weights[residual_.current_mode_];
}

void Interact::EditKeyframe() {
  if (motion_strategy_.EditCurrentKeyframe(kf_name)) {
    SyncWeightsToKeyframe(motion_strategy_.GetCurrentKeyframe());
    kf_index_updated = true;
  }
}

void humanoid::Interact::SaveKeyframe() {
  if (kf_index >= motion_strategy_.GetKeyframesCount()) {
    std::printf("Keyframe index %d out of range, add keyframe first", kf_index);
    return;
  }
  SyncWeightsToKeyframe(motion_strategy_.GetCurrentKeyframe());
  motion_strategy_.SaveCurrentKeyframe();
  kf_index = motion_strategy_.GetCurrentKeyframeIndex();
}

void humanoid::Interact::LoadKeyframe() {
  if (motion_strategy_.LoadKeyframe(kf_name)) {
    SyncWeightsFromKeyframe(motion_strategy_.GetCurrentKeyframe());
    kf_index = motion_strategy_.GetCurrentKeyframeIndex();
  }
}

void humanoid::Interact::SaveKeyframeSequence() {
  SyncWeightsToKeyframe(motion_strategy_.GetCurrentKeyframe());
  motion_strategy_.UpdateCurrentKeyframe(kf_index);

  motion_strategy_.SaveStrategy(kf_name, 
                                CONTACT_KEYFRAME_SEQUENCE_FILENAME_PREFIX);
}

void humanoid::Interact::LoadKeyframeSequence() {
  if (motion_strategy_.LoadStrategy(kf_name, 
                                    CONTACT_KEYFRAME_SEQUENCE_FILENAME_PREFIX)) {
    SyncWeightsFromKeyframe(motion_strategy_.GetCurrentKeyframe());
    kf_index = motion_strategy_.GetCurrentKeyframeIndex();
  }
}

void Interact::SyncWeightsToKeyframe(ContactKeyframe& kf) const {
  for (int i=0; i<weight_names.size(); i++) {
    kf.weight[weight_names[i]] = weight[i];
  }
  kf.target_distance_tolerance = parameters[DIST_TOLERANCE_INDEX];
  kf.time_limit = parameters[TIME_LIMIT_INDEX];
  kf.success_sustain_time = parameters[SUSTAIN_TIME];
}

void Interact::SyncWeightsFromKeyframe(const ContactKeyframe& kf) {
  weight.clear();
  int i = 0;
  for (auto& w: weight_names) {
    if (kf.weight.find(w) != kf.weight.end())
      weight.push_back(kf.weight.at(w));
    else {
      double default_weight = default_weights[residual_.current_mode_][i];
      std::printf("Keyframe %s does not have weight for %s, set default %.1f.\n", kf.name.c_str(), w.c_str(), default_weight);
      weight.push_back(default_weight);
    }
    i++;
  }
  strcpy(kf_name, kf.name.c_str());
  parameters[DIST_TOLERANCE_INDEX] = kf.target_distance_tolerance;
  parameters[TIME_LIMIT_INDEX] = kf.time_limit;
  parameters[SUSTAIN_TIME] = kf.success_sustain_time;
}

void Interact::SetSelectedPoint(const mjtNum* selpos, const mjtNum selbody,
                        const mjtNum selgeom) {
  ContactKeyframe& current_keyframe = motion_strategy_.GetCurrentKeyframe();

  // select Contact process
  int contact_pair_index = ReinterpretAsInt(parameters[CONTACT_POINT_PARAM_INDEX]) - 1;

  // set facing target
  if (contact_pair_index < 0) {
    int facing_target = ReinterpretAsInt(parameters[CONTACT_POINT_PARAM_INDEX+1]);
    if (facing_target > 0) {
      current_keyframe.facing_target.clear();
      for (int j=0; j<2; j++)
        current_keyframe.facing_target.push_back(selpos[j]);
    }
    else {
      current_keyframe.facing_target.clear();
    }
    return;
  }
 
  // set contact pair
  int contact_point = ReinterpretAsInt(parameters[RESIDUAL_TERMS_COUNT+(contact_pair_index)]); 
  
  switch (contact_point) {
    case 0: // none
      current_keyframe.contact_pairs[contact_pair_index].body1 = NOT_SELECTED;
      current_keyframe.contact_pairs[contact_pair_index].body2 = NOT_SELECTED;
      current_keyframe.contact_pairs[contact_pair_index].geom1 = NOT_SELECTED;
      current_keyframe.contact_pairs[contact_pair_index].geom2 = NOT_SELECTED;
      return;
    case 1: // point1
      for (int j=0; j<3; j++)
        current_keyframe.contact_pairs[contact_pair_index].local_pos1[j] = selpos[j];
      current_keyframe.contact_pairs[contact_pair_index].body1 = selbody;
      current_keyframe.contact_pairs[contact_pair_index].geom1 = selgeom;
      break;
    case 2: // point2
      for (int j=0; j<3; j++)
        current_keyframe.contact_pairs[contact_pair_index].local_pos2[j] = selpos[j];
      current_keyframe.contact_pairs[contact_pair_index].body2 = selbody;
      current_keyframe.contact_pairs[contact_pair_index].geom2 = selgeom;
      break;
    default:
      break;
  }

  current_keyframe.contact_pairs[contact_pair_index].contact_type = static_cast<ContactType>(0);
}

void Interact::SetContactKeyframes(const std::vector<ContactKeyframe>& keyframes, const int current_index) {
  motion_strategy_.SetContactKeyframes(keyframes);
  if (current_index >= 0 && current_index < motion_strategy_.GetKeyframesCount())
    motion_strategy_.UpdateCurrentKeyframe(current_index);
}

double Interact::CalculatePushForce(const mjModel* model, const mjData* data, const int geom1, const int geom2) const {
  mjContact* con;
  mjtNum mat[9], tmp[9], vec[3], frc[3], confrc[6];

  for (int i=0; i < data->ncon; i++) {
    // get pointer
    con = data->contact + i;

    if ((con->geom2 == geom1  && con->geom1 == geom2) ||
        (con->geom1 == geom1  && con->geom2 == geom2))
    {
      mju_copy(tmp, con->frame+3, 6);
      mju_copy(tmp+6, con->frame, 3);
      mju_transpose(mat, tmp, 3, 3);

      // mat = contact frame rotation matrix (normal along x)
      mju_transpose(mat, con->frame, 3, 3);

      // get contact force:torque in contact frame
      mj_contactForce(model, data, i, confrc);

      mju_zero3(frc);
      mju_copy(frc, confrc, mjMIN(3, con->dim));
    }
  }
  vec[0] = mat[0]*frc[0];
  vec[1] = mat[3]*frc[0];
  vec[2] = mat[6]*frc[0];

  return mju_norm3(vec);
}

double Interact::CalculateKeyframeError(const ContactKeyframe& keyframe, mjData* data) const {
  double error[CONTACT_PAIR_COUNT];
  // iterate through all pairs
  for (int i=0; i<CONTACT_PAIR_COUNT; i++) {
    const ContactPair& contact = keyframe.contact_pairs[i];
    if (contact.body1 != NOT_SELECTED && contact.body2 != NOT_SELECTED) {
      // convert from local to global coordinates
      mjtNum tmp1[3];
      mjtNum selected_global_pos1[3];
      mju_mulMatVec(tmp1, data->xmat+9*contact.body1, contact.local_pos1, 3, 3);
      mju_add3(selected_global_pos1, tmp1, data->xpos+3*contact.body1);

      mjtNum tmp2[3];
      mjtNum selected_global_pos2[3];
      mju_mulMatVec(tmp2, data->xmat+9*contact.body2, contact.local_pos2, 3, 3);
      mju_add3(selected_global_pos2, tmp2, data->xpos+3*contact.body2);

      // calculate the distance in global coordinate frame
      double dist[3] = {0., 0., 0.};
      mju_addTo(dist, selected_global_pos1, 3);
      mju_subFrom(dist, selected_global_pos2, 3);
      error[i] = mju_norm3(dist);
    }
    else {
      error[i] = 0;
    }
  }
  return mju_norm(error, CONTACT_PAIR_COUNT);
}

// draw task-related geometry in the scene
void Interact::ModifyScene(const mjModel* model, const mjData* data,
                           mjvScene* scene) const {
  const ContactKeyframe& current_keyframe = motion_strategy_.GetCurrentKeyframe();

  const float rgba[CONTACT_PAIR_COUNT][4] = {
    // {1, 0, 1, 0.3}, // purple
    // {0, 1, 1, 0.3}, // cyan
    // {1, 1, 0, 0.3},  // yellow
    // {0.5, 0.5, 1, 0.3}, //purple
    // {1, 0.5, 0.5, 0.3}, //pink
    {0, 0, 1, 0.8}, // purple
    {0, 1, 0, 0.8}, // cyan
    {0, 1, 1, 0.8}, // yellow
    {1, 0, 0, 0.8}, //purple
    {1, 0, 1, 0.8}, //pink
  };

  // add visuals to the contact points
  for (int i=0; i<CONTACT_PAIR_COUNT; i++) {
    double sz[3] = {0.03,0.03,0.03};
    const ContactPair contact = current_keyframe.contact_pairs[i];
    if (contact.body1 != NOT_SELECTED) {
      mjtNum global[3];
      mju_mulMatVec(global, data->xmat+9*contact.body1, contact.local_pos1, 3, 3);
      mju_addTo(global, data->xpos+3*contact.body1, 3);
      AddGeom(scene, mjGEOM_SPHERE, sz, global, nullptr, rgba[i]);
    }
    if (contact.body2 != NOT_SELECTED) {
      mjtNum global[3];
      mju_mulMatVec(global, data->xmat+9*contact.body2, contact.local_pos2, 3, 3);
      mju_addTo(global, data->xpos+3*contact.body2, 3);
      AddGeom(scene, mjGEOM_SPHERE, sz, global, nullptr, rgba[i]);
    }
  }

  // add visuals for the facing direction
  if (!current_keyframe.facing_target.empty()) {
    double sz[3] = {0.02,0.02,0.02};
    mjtNum pos[3] = {current_keyframe.facing_target[0], current_keyframe.facing_target[1], 0};
    const float rgba[] = {1, 1, 1, 0.5};
    AddGeom(scene, mjGEOM_SPHERE, sz, pos, nullptr, rgba);
  }
}


}  // namespace mjpc
