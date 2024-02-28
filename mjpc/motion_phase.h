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

#ifndef MJPC_MOTION_PHASE_H_
#define MJPC_MOTION_PHASE_H_

#include <mujoco/mujoco.h>
#include <mjpc/tasks/humanoid/interact/contact_keyframe.h>
#include <mjpc/sim_state.h>

#include <filesystem>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace mjpc {

/*
This class holds all the information needed for a motion phase/chunk.
It contains the information for this particular phase,
including the start state and target.
It keeps track of the simulation state when the phase is played.
*/
class MotionPhase {
  public:
    MotionPhase() = default;
    ~MotionPhase() = default;

    constexpr static int kMaxLength = 5000;

    MotionPhase(const SimState& start_state) :
                phase_number(0),
                start_state(start_state),
                last_state(SimState()),
                current_state(SimState()),
                qposes(kMaxLength),
                qvels(kMaxLength),
                total_objective_values(kMaxLength),
                partwise_log(kMaxLength),
                success(false),
                fixed(false) {}
    
    void Reset();

    void SetStartState(const SimState& start_state) { this->start_state = start_state; }
    void SetLastState(const SimState& last_state) { this->last_state = last_state; }
    SimState& GetStartState() { return start_state; }
    SimState& GetLastState() { return last_state;}
    
    void AddState(const SimState& state);
    void AddPartwiseLog(const int log) { partwise_log.push_back(log); }
    
    // void GetStates(std::vector<SimState>& states) const { states = sim_states; }
    void GetQPoses(std::vector<std::vector<double>>& qposes) const;
    int GetPhaseLength() const { return qposes.size(); }
    bool GetSuccess() const { return success; }

    double GetStartTime() const { return start_state.time; }
    void SetStartTime(const double time) { start_state.time = time; }

    int phase_number;

    // The start state of the phase
    SimState start_state;
    SimState last_state;
    SimState current_state;

    std::vector<std::vector<double>> qposes;
    std::vector<std::vector<double>> qvels;
    std::vector<double> total_objective_values;

    std::vector<int> partwise_log;

    bool success;
    bool fixed;
};

void to_json(json& j, const MotionPhase& phase);
void from_json(const json& j, MotionPhase& phase);

}  // namespace mjpc

#endif  // MJPC_MOTION_PHASE_H_