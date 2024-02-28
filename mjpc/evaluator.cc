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

#include "mjpc/evaluator.h"

#include "mjpc/utilities.h"

#include <fstream>

namespace mjpc {

void Episode::Reset() {
    episode_number = 0;
    successful = false;
    current_phase_index = 0;
    // motion_phases.clear();
    for (auto& phase: motion_phases)
      phase.Reset();
}

// void Episode::ReplayPhase(int phase_index) {
//   MotionPhase& phase = motion_phases[current_phase_index];
//   if (replay_frame_number >= phase.GetPhaseLength()) {
//     std::printf("\nReplayPhase: No more frames to replay\n");
//     return;
//   }
//   SimState& state = phase.qposes[replay_frame_number];
// }

void Episode::SaveEpisode(const std::string& task_name, std::string name, bool modified) {
  json j;
  to_json(j, *this);
  std::string data_kf = j.dump();
  std::string file_name = "episode_" + std::to_string(episode_number);
  if (modified) {
    file_name = "modified_" + file_name;
  }
  std::string filename(SOURCE_DIR "/mjpc/tasks/humanoid/eval/" + task_name + "/" + std::string(name) + "/" + file_name + "_state.json");
  std::ofstream of(filename);
  if (!of.is_open()) {
    std::printf("Failed to open file %s\n", filename.c_str());
    return;
  }
  of << j << std::endl;
  of.close();
  std::printf("\nEvaluation data saved as %s\n", filename.c_str());
}

void Evaluator::Start(const std::string& task_name) {
  this->episode_count = 0;
  this->phase_start_time = 0.;
  this->success_count = 0;
  this->failure_count = 0;
  this->task_name = std::string(task_name);
  this->reset_sim_required = true;
  episode.Reset();
  this->save_path = "/mjpc/tasks/humanoid/eval/" + task_name + "/" + std::string(name);
  std::filesystem::create_directory(SOURCE_DIR "" + this->save_path);
}

void Evaluator::Reset(std::string name) {
  this->episode_count = 0;
  this->phase_start_time = 0.;
  this->kf_distance_error = 0.;
  this->success_count = 0;
  this->failure_count = 0;
  this->task_name = name;
  this->reset_sim_required = true;
  this->is_replaying = false;
  this->is_replaying_phase = false;
  this->is_reoptimizing = false;
  this->replay_frame_number = 0;
  episode.Reset();
}

void Evaluator::SaveData() {
  json j;
  to_json(j, *this);
  std::string data_kf = j.dump();
  std::string file_name = this->name;
  if (file_name.empty()) {
    file_name = task_name;
  }
  std::replace(file_name.begin(), file_name.end(), ' ', '_');
  std::string filename(SOURCE_DIR "" + save_path + "/" + file_name + "_evaluation_data.json");
  std::ofstream of(filename);
  if (!of.is_open()) {
    std::printf("Failed to open file %s\n", filename.c_str());
    return;
  }
  of << j << std::endl;
  of.close();
  std::printf("\nEvaluation data saved as %s\n", filename.c_str());
}

void Evaluator::GoToNextEpisode(bool success, bool evaluation_mode) {
  if (evaluation_mode) {
    if (success) {
      success_count++;
      episode.successful = true;
    } else {
      failure_count++;
      episode.successful = false;
    }
    episode.SaveEpisode(task_name, name);
  }
  episode.Reset();
  episode.episode_number = ++episode_count;
  phase_start_time = 0.;
}

void Evaluator::AddEpisodeState(std::vector<double>& qpos, 
                                std::vector<double>& qvel, 
                                std::vector<double>& act, 
                                std::vector<double>& ctrl,
                                double objective_value,
                                double time) {
  SimState state (qpos, qvel, act, ctrl, objective_value, time);
  if (episode.motion_phases.empty()) {
    episode.AddPhase();
  }
  episode.GetCurrentPhase().AddState(state);
}

void Evaluator::ReplayEpisode() {
  // load saved episode
  std::string file_name = "episode_" + std::to_string(replay_episode_number);
  std::string filename(SOURCE_DIR "/mjpc/tasks/humanoid/eval/" + task_name + "/" + std::string(name) + "/" + file_name + "_state.json");

  try {
    std::ifstream f(filename);
    if (!f.good()) {
      std::printf("\nReplay Episode: File %s does not exist\n", filename.c_str());
      return;
    }
    json j_array = json::parse(f);
    from_json(j_array, episode);
    std::printf("\nEpisode loaded from %s\n", filename.c_str());
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
  } catch (...) {
    std::cerr << "\nUnknown error loading episode from " << filename << '\n';
  }

  // Set up the parameters
  reset_sim_required = true;
  replay_frame_number = 0;
  is_replaying = true;
  is_replaying_phase = false;
  is_reoptimizing = false;
}
 
void Evaluator::ReplayPhase() {
  // Set up the parameters
  reset_sim_required = true;
  replay_frame_number = 0;
  is_replaying = false;
  is_replaying_phase = true;
  is_reoptimizing = false;

  if (episode.motion_phases[episode.current_phase_index].qposes.empty()) {
    std::printf("\nReplay Phase: No saved states\n");
    is_replaying_phase = false;
    return;
  }
}

void Evaluator::Reoptimize() {
  // Set up the parameters
  reset_sim_required = true;
  replay_frame_number = 0;
  is_replaying = false;
  is_replaying_phase = false;
  is_reoptimizing = true;

  auto backup = episode.motion_phases[episode.current_phase_index];

  for(int i=episode.current_phase_index; i<episode.motion_phases.size(); i++) {
    episode.motion_phases[i].Reset();
    episode.motion_phases[i].fixed = false;
  }

  if (episode.current_phase_index == 0) {
    episode.GetCurrentPhase().SetStartState(episode.motion_strategy.GetHomeState());
    episode.GetCurrentPhase().qposes.push_back(episode.motion_phases[0].start_state.qpos);
    episode.GetCurrentPhase().qvels.push_back(episode.motion_phases[0].start_state.qvel);
  }
  else { 
    episode.GetCurrentPhase().SetStartState(backup.GetStartState());
    episode.GetCurrentPhase().qposes.push_back(backup.qposes[0]);
    episode.GetCurrentPhase().qvels.push_back(backup.qvels[0]);
  }
}

void Evaluator::SaveNew() {
  episode.SaveEpisode(task_name, name, false);
  for (auto& phase: episode.motion_phases)
    phase.fixed = true;
}

// serialization

void to_json(json& j, const Episode& episode) {
  j = json{{"episode_number", episode.episode_number},
           {"successful", episode.successful},
           {"episode_length", episode.GetEpisodeLength()},
           {"motion_phases", episode.motion_phases},
           {"motion_strategy", episode.motion_strategy}};
}

void from_json(const json& j, Episode& episode) {
  j.at("episode_number").get_to(episode.episode_number);
  j.at("successful").get_to(episode.successful);
  j.at("motion_phases").get_to(episode.motion_phases);
  j.at("motion_strategy").get_to(episode.motion_strategy);
}

void to_json(json& j, const Evaluator& evaluator) {
  j = json{{"episode", evaluator.episode_count},
           {"success_count", evaluator.success_count},
           {"failure_count", evaluator.failure_count},
           {"success_rate", 100.0*evaluator.success_count/evaluator.episode_count},
           {"failure_rate", 100.0*evaluator.failure_count/evaluator.episode_count}};
}

}  // namespace mjpc
