# Body skeleton from Meta's body tracking API (OVRPlugin.GetBodyState4),
# sent by the Quest app in the same pose stream as QuestHmd/QuestController.
# With fidelity HIGH this is Inside-Out Body Tracking (IOBT): the headset's
# side cameras track the torso, shoulders, elbows and wrists. With LOW the
# arms are inferred from the head and controller poses only.

# Header.stamp = ROS clock at receive on the PC, identical to the QuestHmd and
# QuestController messages from the same source frame.
# Header.frame_id = the bridge's parent_frame (quest_origin).
std_msgs/Header header

# Same meaning as in QuestHmd: Quest monotonic clock (ns) of the source frame,
# PC monotonic clock (ns) at receive, sequence number and device id.
uint64 device_time_ns
uint64 pc_monotonic_ns
uint32 seq
string quest_id

# Time of this body sample on the headset (OVRPlugin BodyState.Time, seconds
# on the OpenXR clock). It can differ from device_time_ns because body
# tracking runs at its own rate and is predicted to the display time.
float64 body_time_s

# 0 = no confidence, 1 = full confidence.
float32 confidence

uint8 FIDELITY_LOW=1
uint8 FIDELITY_HIGH=2
uint8 fidelity

uint8 CALIBRATION_VALID=1
uint8 CALIBRATION_CALIBRATING=2
uint8 CALIBRATION_INVALID=3
uint8 calibration_status

# Increments when the runtime changes the skeleton (e.g. after recalibrating
# body proportions).
uint32 skeleton_changed_count

# One entry per joint, in OVRPlugin.BoneId order (70 for the upper-body joint
# set). Names are the BoneId names, e.g. Body_LeftArmUpper = shoulder,
# Body_LeftArmLower = elbow, Body_LeftHandWrist = wrist.
string[] joint_names

# Joint poses in header.frame_id, ROS axes (X forward, Y left, Z up). The
# bridge converts from Unity with the same mapping as the HMD and controllers.
geometry_msgs/Pose[] poses

# XrSpaceLocationFlags per joint: 1 orientation valid, 2 position valid,
# 4 orientation tracked, 8 position tracked. Ignore a pose whose valid bits
# are not set.
uint8 LOCATION_ORIENTATION_VALID=1
uint8 LOCATION_POSITION_VALID=2
uint8 LOCATION_ORIENTATION_TRACKED=4
uint8 LOCATION_POSITION_TRACKED=8
uint8[] location_flags
