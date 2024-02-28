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

#include "sim_state.h"

namespace mjpc {

void SimState::Reset() {
  std::fill(qpos.begin(), qpos.end(), 0.);
  std::fill(qvel.begin(), qvel.end(), 0.);
  std::fill(act.begin(), act.end(), 0.);
  std::fill(ctrl.begin(), ctrl.end(), 0.);
  total_cost_value = 0.;
  time = 0.;
}

// serialization
void to_json(json& j, const SimState& state) {
  j = json{{"qpos", state.qpos},
           {"qvel", state.qvel},
           {"act", state.act},
           {"ctrl", state.ctrl},
           {"total_cost_value", state.total_cost_value},
           {"time", state.time}};
}

void from_json(const json& j, SimState& state) {
  j.at("qpos").get_to(state.qpos);
  j.at("qvel").get_to(state.qvel);
  j.at("act").get_to(state.act);
  j.at("ctrl").get_to(state.ctrl);
  j.at("total_cost_value").get_to(state.total_cost_value);
  j.at("time").get_to(state.time);
}

}  // namespace mjpc
