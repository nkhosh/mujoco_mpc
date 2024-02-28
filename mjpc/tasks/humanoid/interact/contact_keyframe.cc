#include "contact_keyframe.h"

namespace mjpc::humanoid {
    void ContactPair::Reset() {
        body1 = NOT_SELECTED;
        body2 = NOT_SELECTED;
        geom1 = NOT_SELECTED;
        geom2 = NOT_SELECTED;
        contact_type = kTouch;
        for (int i = 0; i < 3; ++i) {
            local_pos1[i] = 0.;
            local_pos2[i] = 0.;
        }
    }

    void ContactKeyframe::Reset() {
        name.clear();
        for (auto& contact_pair : contact_pairs)
            contact_pair.Reset();
        // torso_target.clear();
        facing_target.clear();
        time_limit = 5.;
        success_sustain_time = 2.;
        target_distance_tolerance = 0.1;
        weight.clear();
    }

    void to_json(json& j, const ContactPair& contact_pair) {
        j = json{{"body1", contact_pair.body1},
                 {"body2", contact_pair.body2},
                 {"geom1", contact_pair.geom1},
                 {"geom2", contact_pair.geom2},
                 {"contact_type", contact_pair.contact_type},
                 {"local_pos1", contact_pair.local_pos1},
                 {"local_pos2", contact_pair.local_pos2}};
    }

    void from_json(const json& j, ContactPair& contact_pair) {
        j.at("body1").get_to(contact_pair.body1);
        j.at("body2").get_to(contact_pair.body2);
        j.at("geom1").get_to(contact_pair.geom1);
        j.at("geom2").get_to(contact_pair.geom2);
        j.at("contact_type").get_to(contact_pair.contact_type);
        j.at("local_pos1").get_to(contact_pair.local_pos1);
        j.at("local_pos2").get_to(contact_pair.local_pos2);
    }

    void to_json(json& j, const ContactKeyframe& keyframe) {
        j = json{{"name", keyframe.name},
                 {"contacts", keyframe.contact_pairs},
                 {"facing_target", keyframe.facing_target},
                 {"time_limit", keyframe.time_limit},
                 {"success_sustain_time", keyframe.success_sustain_time},
                 {"target_distance_tolerance", keyframe.target_distance_tolerance},
                 {"weight", keyframe.weight}};
    }

    void from_json(const json& j, ContactKeyframe& keyframe) {
        j.at("name").get_to(keyframe.name);
        j.at("contacts").get_to(keyframe.contact_pairs);
        // j.at("torso_target").get_to(keyframe.torso_target);
        j.at("facing_target").get_to(keyframe.facing_target);
        if (j.contains("pause")) {
            mjtNum pause;
            j.at("pause").get_to(pause);
            keyframe.success_sustain_time = pause;
        } else {
            j.at("time_limit").get_to(keyframe.time_limit);
        }
        if (j.contains("threshold")) {
            std::vector<mjtNum> threshold;
            j.at("threshold").get_to(threshold);
            keyframe.time_limit = threshold[1];
            keyframe.target_distance_tolerance = threshold[0];
        } else {
            j.at("target_distance_tolerance").get_to(keyframe.target_distance_tolerance);
            j.at("success_sustain_time").get_to(keyframe.success_sustain_time);
        }
        j.at("weight").get_to(keyframe.weight);
    }
}