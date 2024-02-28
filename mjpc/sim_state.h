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

#ifndef MJPC_SIM_STATE_H_
#define MJPC_SIM_STATE_H_

#include <mujoco/mujoco.h>

#include <filesystem>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace mjpc {

class SimState {
  public:
  
  SimState() = default;
  ~SimState() = default;

  SimState(std::vector<double>& qpos, 
           std::vector<double>& qvel, 
           std::vector<double>& act, 
           std::vector<double>& ctrl,
           double total_cost_value,
           double time) : qpos(qpos),
                          qvel(qvel),
                          act(act),
                          ctrl(ctrl),
                          total_cost_value(total_cost_value),
                          time(time) {}

  SimState(int nq, int nv, int na, int nu) : qpos(nq, 0.),
                                             qvel(nv, 0.),
                                             act(na, 0.),
                                             ctrl(nu, 0.),
                                             total_cost_value(0.),
                                             time(0.) {}
  
  void Reset();

  std::vector<double> qpos;
  std::vector<double> qvel;
  std::vector<double> act;
  std::vector<double> ctrl;
  double total_cost_value;
  double time;
};
// serialization
void to_json(json& j, const SimState& state);
void from_json(const json& j, SimState& state);

}  // namespace mjpc

#endif  // MJPC_SIM_STATE_H_