#ifndef FCSIM_SHARED_T2_JOINTINDEX_HPP
#define FCSIM_SHARED_T2_JOINTINDEX_HPP

#include <array>
#include <cstddef>

namespace k1sim::t2 {
    // Booster Deploy T2_31DOF_CFG.joint_names, which is the SDK serial order.
    inline constexpr std::size_t JOINT_COUNT = 31;
    inline constexpr std::size_t LEFT_HIP_PITCH = 19;
    inline constexpr std::array<const char*, JOINT_COUNT> JOINT_NAMES = {
        "aa_head_yaw_joint", "head_pitch_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_pitch_joint", "left_elbow_yaw_joint",
        "left_wrist_pitch_joint", "left_wrist_yaw_joint", "left_wrist_roll_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_pitch_joint", "right_elbow_yaw_joint",
        "right_wrist_pitch_joint", "right_wrist_yaw_joint", "right_wrist_roll_joint",
        "waist_pitch_joint", "waist_roll_joint", "waist_yaw_joint",
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    };
}  // namespace k1sim::t2

#endif
