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

#ifndef MJPC_MOTION_STRATEGY_H_
#define MJPC_MOTION_STRATEGY_H_

#include <mujoco/mujoco.h>
#include "contact_keyframe.h"

#include <filesystem>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace mjpc::humanoid {

constexpr char CONTACT_KEYFRAME_FILENAME_PREFIX[] = SOURCE_DIR "/mjpc/tasks/humanoid/interact/keyframes/kf-";
constexpr char CONTACT_KEYFRAME_SEQUENCE_FILENAME_PREFIX[] = SOURCE_DIR "/mjpc/tasks/humanoid/interact/keyframes/kfs-";

/*
This class holds the motion strategy, e.g. a sequence of keyframes, and the initial state.
It manages any changes to the strategy.
*/
class MotionStrategy {
  public:
    MotionStrategy() = default;
    ~MotionStrategy() = default;

    MotionStrategy(const std::vector<ContactKeyframe>& keyframes) :
                   contact_keyframes_(keyframes),
                   current_keyframe_(ContactKeyframe()),
                   current_keyframe_index_(0) {}

    void Reset();

    bool HasKeyframes() const { return !contact_keyframes_.empty(); }
    ContactKeyframe& GetCurrentKeyframe() {
      return current_keyframe_; // contact_keyframes_[current_keyframe_index_];
    }
    const ContactKeyframe& GetCurrentKeyframe() const {
      return current_keyframe_;
    }

    const int GetCurrentKeyframeIndex() const { return current_keyframe_index_; }
    std::vector<ContactKeyframe>& GetContactKeyframes() { return contact_keyframes_; }
    const std::vector<ContactKeyframe>& GetContactKeyframes() const { return contact_keyframes_; }
    const int GetKeyframesCount() const { return contact_keyframes_.size(); }

    // Sync weights
    // Sync kf_name and kf_index
    // void SetIndex(const int index) { current_keyframe_index_ = index; } // should I guard this here
    void UpdateCurrentKeyframe(const int index) { 
      current_keyframe_index_ = index; 
      current_keyframe_ = contact_keyframes_[current_keyframe_index_]; 
    }
    void SetContactKeyframes(const std::vector<ContactKeyframe>& keyframes) { contact_keyframes_ = keyframes; }

    // Keyframe sequence commands: Add, Edit, Save, Delete
    bool AddCurrentKeyframe(const std::string& kf_name = "");
    int NextKeyframe();
    bool RemoveCurrentKeyframe();
    void ClearKeyframes();
    bool EditCurrentKeyframe(const std::string& kf_name = "");
    bool SaveCurrentKeyframe();
    bool LoadKeyframe(const std::string& kf_name);
    bool SaveStrategy(const std::string& name, const std::string& path = CONTACT_KEYFRAME_SEQUENCE_FILENAME_PREFIX);
    bool LoadStrategy(const std::string& name, const std::string& path = CONTACT_KEYFRAME_SEQUENCE_FILENAME_PREFIX);

  private:
    std::vector<ContactKeyframe> contact_keyframes_;
    ContactKeyframe current_keyframe_;
    int current_keyframe_index_;
};

void to_json(json& j, const MotionStrategy& strategy);
void from_json(const json& j, MotionStrategy& strategy);

}  // namespace mjpc

#endif  // MJPC_MOTION_STRATEGY_H_
