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

#include "mjpc/motion_phase.h"

#include "mjpc/utilities.h"

namespace mjpc {

void MotionPhase::Reset() {
  start_state.Reset();
  qposes.clear();
  qvels.clear();       
  total_objective_values.clear();
  partwise_log.clear();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
  success = false;
  fixed = false;
}

void MotionPhase::AddState(const SimState& state) { 
  current_state = state; 
  qposes.push_back(state.qpos); 
  qvels.push_back(state.qvel);
  total_objective_values.push_back(state.total_cost_value);
}

void to_json(json& j, const MotionPhase& phase) {
  j = json{{"phase_number", phase.phase_number},
           {"start_state", phase.start_state},
           {"end_state", phase.last_state},
           {"qposes", phase.qposes},
           {"qvels", phase.qvels},
           {"total_objective_values", phase.total_objective_values},
           {"partwise_log", phase.partwise_log},
           {"length", phase.GetPhaseLength()},
           {"success", phase.success}};
}

void from_json(const json& j, MotionPhase& phase) {
  j.at("phase_number").get_to(phase.phase_number);
  j.at("start_state").get_to(phase.start_state);
  j.at("end_state").get_to(phase.last_state);
  j.at("qposes").get_to(phase.qposes);
  j.at("qvels").get_to(phase.qvels); 
  j.at("total_objective_values").get_to(phase.total_objective_values);
  j.at("success").get_to(phase.success);
}

}  // namespace mjpc
