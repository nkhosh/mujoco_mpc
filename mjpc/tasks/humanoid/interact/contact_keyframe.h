#ifndef CONTACT_KEYFRAME_H
#define CONTACT_KEYFRAME_H

#include <mujoco/mujoco.h>
#include <vector>
#include <map>
#include <string>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace mjpc::humanoid {

// ---------- Constants ----------------- //
constexpr int NOT_SELECTED = -1;
constexpr int CONTACT_PAIR_COUNT = 5;
constexpr int WEIGHT_COUNT = 14;

enum ContactType: int {
  kTouch = 0,
  kPush = 1,
  kGrasp = 2
};

class ContactPair {
public:
    int body1, body2, geom1, geom2;
    ContactType contact_type;
    mjtNum local_pos1[3], local_pos2[3];

    ContactPair() : body1(NOT_SELECTED),
                body2(NOT_SELECTED),
                geom1(NOT_SELECTED),
                geom2(NOT_SELECTED),
                contact_type(kTouch),
                local_pos1{0., 0., 0.},
                local_pos2{0., 0., 0.} {}
    
    void Reset();
};

class ContactKeyframe {
public:
    std::string name;
    ContactPair contact_pairs[CONTACT_PAIR_COUNT]; 
    mjtNum time_limit; // the maximum time allowed for attempting a single keyframe before resetting
    mjtNum success_sustain_time; // the minimum time that the objective needs to be satisfied within the distance threshold to consider the keyframe successful
    mjtNum target_distance_tolerance; // the proximity to the keyframe objective that needs to be maintained for a certain time
    std::vector<mjtNum> facing_target;
    std::map<std::string, mjtNum> weight;

    ContactKeyframe() : name(""),
              contact_pairs{},
              time_limit(5.),
              success_sustain_time{2.},
              target_distance_tolerance{0.1},
              facing_target(),
              weight() {}

    void Reset();
};


void to_json(json& j, const ContactPair& contact_pair);
void from_json(const json& j, ContactPair& contact_pair);
void to_json(json& j, const ContactKeyframe& keyframe);
void from_json(const json& j, ContactKeyframe& keyframe);

}  // namespace mjpc::humanoid

#endif  // CONTACT_KEYFRAME_H