"""Shared robot constants for standing and walking (Locomotion & Movement Unit)."""

ANKLE_H = 0.06   # ankle axis -> sole (m)
FOOT_X = 0.025   # foot centre ahead of the ankle axis (m)
FOOT_LEN = 0.22  # sole length (m)
FOOT_WID = 0.11  # sole width (m)
HIP_HALF_WIDTH = 0.09  # lateral offset of each hip from the pelvis centre (m)

BODY_JOINTS = [
    "waist_yaw_joint",
    "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
    "neck_yaw_joint", "head_pitch_joint",
]

LEG_JOINTS = {
    s: [f"{s}_{j}_joint" for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")]
    for s in ("left", "right")
}

# "ready" stance: knees bent for compliance, arms slightly out
READY = {
    # ankle/hip chosen with Pinocchio so the CoM sits over the support centre with a level pelvis
    "left_hip_pitch_joint": -0.25, "left_knee_joint": 0.60, "left_ankle_pitch_joint": -0.35,
    "right_hip_pitch_joint": -0.25, "right_knee_joint": 0.60, "right_ankle_pitch_joint": -0.35,
    "left_shoulder_roll_joint": 0.12, "right_shoulder_roll_joint": -0.12,
    "left_elbow_joint": -0.30, "right_elbow_joint": -0.30,
}
