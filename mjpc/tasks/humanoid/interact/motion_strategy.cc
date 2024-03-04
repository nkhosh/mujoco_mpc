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

#include "motion_strategy.h"
#include "mjpc/utilities.h"
  
#include <fstream>

namespace mjpc::humanoid {

void MotionStrategy::Reset() {
  contact_keyframes_.clear();
  current_keyframe_.Reset();
  current_keyframe_index_ = 0;
}

bool MotionStrategy::AddCurrentKeyframe(const std::string& kf_name) {
  current_keyframe_.name = std::string(kf_name).empty() ? std::to_string(current_keyframe_index_) : std::string(kf_name);
  contact_keyframes_.push_back(current_keyframe_);
  current_keyframe_index_ = contact_keyframes_.size() - 1;
  std::printf("\nAdded keyframe %lu", contact_keyframes_.size());
  return true;
}

int MotionStrategy::NextKeyframe() {
  current_keyframe_index_ = (current_keyframe_index_ + 1) % contact_keyframes_.size();

  std::printf("\nCurrent keyframe: %d", current_keyframe_index_);
  current_keyframe_ = contact_keyframes_[current_keyframe_index_];

  return current_keyframe_index_;
}

bool MotionStrategy::RemoveCurrentKeyframe() {
  if (contact_keyframes_.empty())
    return false;
  
  contact_keyframes_.erase(contact_keyframes_.begin() + current_keyframe_index_);

  if (current_keyframe_index_ >= contact_keyframes_.size())
    current_keyframe_index_ = contact_keyframes_.size() - 1;

  current_keyframe_ = contact_keyframes_[current_keyframe_index_];
  std::printf("\nRemoved keyframe %lu", contact_keyframes_.size());
  return true;
}

void MotionStrategy::ClearKeyframes() {
  contact_keyframes_.clear();
  current_keyframe_.Reset();
  current_keyframe_index_ = 0;
  std::printf("\nCleared keyframes, current length: %lu", contact_keyframes_.size());
}

bool MotionStrategy::EditCurrentKeyframe(const std::string& kf_name) {
  if (current_keyframe_index_ > contact_keyframes_.size()) 
    return false;

  current_keyframe_.name = std::string(kf_name).empty() ? std::to_string(current_keyframe_index_) : std::string(kf_name);
  contact_keyframes_[current_keyframe_index_] = current_keyframe_;

  std::printf("\nEdited keyframe %d", current_keyframe_index_);
  return true;
}

bool MotionStrategy::SaveCurrentKeyframe() {
  // TODO IN CALLING
    if (current_keyframe_index_ >= contact_keyframes_.size() || current_keyframe_index_ < 0) {
      std::printf("\nKeyframe index %d out of range, add keyframe first", current_keyframe_index_);
      return false;
    }

    contact_keyframes_[current_keyframe_index_] = current_keyframe_; // TODO Do we need this?

    json j_kf;
    to_json(j_kf, current_keyframe_);
    std::string data_kf = j_kf.dump();
    std::string filename_kf(CONTACT_KEYFRAME_FILENAME_PREFIX);
    filename_kf.append(current_keyframe_.name.c_str());
    filename_kf.append(".json");
    std::ofstream of(filename_kf);
    if (!of.is_open()) {
      std::printf("\nFailed to open file %s", filename_kf.c_str());
      return false;
    }
    of << std::setw(4) << j_kf << std::endl;
    of.close();
    std::printf("\nKeyframe saved as %s", filename_kf.c_str());
    return true;
}

bool MotionStrategy::LoadKeyframe(const std::string& kf_name) { // TODO pass path
  std::string filename_kf(CONTACT_KEYFRAME_FILENAME_PREFIX);
  filename_kf.append(kf_name);
  filename_kf.append(".json");
  std::ifstream f(filename_kf);

  // Check if file exists
  if (!f.good()) {
    std::printf("\nFile %s does not exist", filename_kf.c_str());
    return false;
  }
  json data = json::parse(f);
  try {
    from_json(data, current_keyframe_);
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return false;
  } catch (...) {
    std::cerr << "\nUnknown error loading keyframe from " << filename_kf << '\n';
    return false;
  }
  std::printf("\nKeyframe loaded from %s", filename_kf.c_str());
  return true;
}

bool MotionStrategy::SaveStrategy(const std::string& name, const std::string& path) {
  // TODO: home keyframe field
  json j_array = json::array();

  // TODO: calling function
  // SyncWeightsToKeyframe(current_keyframe_); // Sync current keyframe weights in case user did not explicitly save it
  // SyncWeightsToKeyframe(contact_keyframes_[kf_index]);

  for (auto& kf: contact_keyframes_) {
    json j_kf;
    to_json(j_kf, kf);
    j_array.push_back(j_kf);
  }
  std::string data = j_array.dump();
  std::string filename(path);
  filename.append(name);
  filename.append(".json");
  std::ofstream of(filename);
  if (!of.is_open()) {
    std::printf("\nFailed to open file %s", filename.c_str());
    return false;
  }
  of << std::setw(4) << j_array << std::endl;
  of.close();
  std::printf("\nKeyframe sequence saved as %s", filename.c_str());
  return true;
}

bool MotionStrategy::LoadStrategy(const std::string& name, const std::string& path) {
  std::string filename(path);
  filename.append(name);
  filename.append(".json");
  try {
    std::ifstream f(filename);
    if (!f.good()) {
      std::printf("\nFile %s does not exist", filename.c_str());
      return false;
    }
    json j_array = json::parse(f);
    contact_keyframes_.clear();
    for (auto& j_kf : j_array) {
      ContactKeyframe kf;
      from_json(j_kf, kf);
      contact_keyframes_.push_back(kf);
    }

    if (!contact_keyframes_.empty()) {
      current_keyframe_ = contact_keyframes_[0];
    }

    current_keyframe_index_ = 0;
    std::printf("\nKeyframe sequence loaded from %s", filename.c_str());
    // SyncWeightsFromKeyframe(current_keyframe_); // TODO
    return true;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return false;
  } catch (...) {
    std::cerr << "\nUnknown error loading keyframe sequence from " << filename << '\n';
    return false;
  }
}

void to_json(json& j, const MotionStrategy& strategy) {
  j = json{{"contact_keyframes", strategy.GetContactKeyframes()}};
}

void from_json(const json& j, MotionStrategy& strategy) {
  strategy.SetContactKeyframes(j.at("contact_keyframes").get<std::vector<ContactKeyframe>>());
}

}  // namespace mjpc
