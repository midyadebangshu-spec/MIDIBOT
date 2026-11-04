"""Single source of truth for MIDIBOT's kinematic dimensions (metres).

Used by both the CAD scripts (structure.py) and the URDF generator
(scripts/generate_urdf.py). All link frames are aligned with the world frame
in the zero pose: X forward, Y left, Z up, arms hanging straight down.
"""

# ---------------- legs ----------------
HIP_Y = 0.090          # lateral offset of hip yaw axis from pelvis centre
HIP_YAW_Z = -0.045     # hip yaw joint below pelvis origin
HIP_ROLL_Z = -0.065    # hip roll/pitch joint below hip yaw joint
THIGH = 0.300          # hip pitch -> knee
SHANK = 0.300          # knee -> ankle
ANKLE_H = 0.060        # ankle axis -> sole
FOOT_L, FOOT_W, FOOT_T = 0.220, 0.110, 0.025
FOOT_X = 0.025         # foot centre forward of the ankle axis

# ---------------- torso / head ----------------
WAIST_Z = 0.085        # waist yaw joint above pelvis origin
SHOULDER_Y = 0.175     # shoulder pitch axis lateral offset (from waist frame)
SHOULDER_Z = 0.270     # shoulder pitch axis height (from waist frame)
NECK_Z = 0.320         # neck yaw joint height (from waist frame)
HEAD_PITCH_Z = 0.030   # head pitch joint above neck yaw joint

# ---------------- arms ----------------
SHOULDER_YAW_Z = -0.100   # shoulder roll -> shoulder yaw
ELBOW_Z = -0.120          # shoulder yaw -> elbow   (upper arm = 0.22)
WRIST_YAW_Z = -0.090      # elbow -> wrist yaw
WRIST_PITCH_Z = -0.110    # wrist yaw -> wrist pitch/roll (forearm = 0.20)
HAND_MOUNT_Z = -0.035     # wrist roll -> hand base flange

# pelvis-origin height above the ground when standing straight
PELVIS_HEIGHT = ANKLE_H + SHANK + THIGH + (-HIP_ROLL_Z) + (-HIP_YAW_Z)
