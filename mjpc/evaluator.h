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

#ifndef MJPC_EVALUATOR_H_
#define MJPC_EVALUATOR_H_

#include <mujoco/mujoco.h>
#include "mjpc/motion_phase.h"
#include "mjpc/tasks/humanoid/interact/contact_keyframe.h"
#include "mjpc/tasks/humanoid/interact/motion_strategy.h"
#include "mjpc/sim_state.h"

#include <filesystem>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace mjpc {

class Episode {
 public:
  Episode() = default;
  ~Episode() = default;

  void SaveEpisode(const std::string& task_name, std::string name = "tasks", bool modified = false);
  void Reset();

  MotionPhase& GetCurrentPhase() { 
    if (current_phase_index >= motion_phases.size())
      current_phase_index = motion_phases.size() - 1;
    if (current_phase_index < 0)
      current_phase_index = 0;
    return motion_phases[current_phase_index]; 
  }
  const int GetEpisodeLength() const { 
    int length = 0;
    for (const auto& phase : motion_phases) {
      length += phase.GetPhaseLength();
    }
    return length;
  }

  void AddPhase() { 
    MotionPhase phase;
    motion_phases.push_back(phase);
    current_phase_index = motion_phases.size() - 1;
    motion_phases.back().phase_number = current_phase_index;
  }

  int episode_number;
  bool successful;

  std::vector<MotionPhase> motion_phases;
  humanoid::MotionStrategy motion_strategy;
  int current_phase_index;
};

class Evaluator {
  friend class Task;

 public:
  Evaluator() = default;
  virtual ~Evaluator() = default;

  void Start(const std::string& task_name = "tasks");
  void Reset(std::string name = "");
  void SaveData();

  double GetPhaseDuration(const mjData* data) const { return reset_sim_required ? 0 : data->time - phase_start_time; }
  double GetSuccessSustainTime(const mjData* data) const { return data->time - first_success_time; }
 
  void GoToNextEpisode(bool success, bool evaluation_mode = false);

  // episode info
  void AddEpisodeState(std::vector<double>& qpos, 
                      std::vector<double>& qvel, 
                      std::vector<double>& act, 
                      std::vector<double>& ctrl,
                      double objective_value,
                      double time);

  void ReplayEpisode();
  void ReplayPhase();
  void Reoptimize();
  void SaveNewEpisode();
  void SaveNew();

  char name[mjMAXUITEXT] = "";
  // int episode;
  Episode episode;
  double phase_start_time = 0.; 
  double first_success_time = 0.;
  double kf_distance_error = 0.;
  int max_num_episodes = 10;
  int episode_count;
  int success_count;
  int failure_count;

  std::string task_name;
  std::string save_path;
  int replay_episode_number;
  int replay_frame_number;
  bool is_replaying = false;
  bool is_replaying_phase = false;
  bool is_reoptimizing = false;

  bool reset_sim_required = false;
  SimState reset_state;

};

// serialization
void to_json(json& j, const Episode& episode);
void from_json(const json& j, Episode& episode);
void to_json(json& j, const Evaluator& evaluator);

}  // namespace mjpc

#endif  // MJPC_EVALUATOR_H_