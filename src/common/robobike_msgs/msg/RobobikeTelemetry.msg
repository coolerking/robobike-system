# One control-loop sample (250 Hz) read from ROBOBIKE GET /get_acc.
# stamp: TIME_MS mapped to ROS time; frame_id: bridge parameter frame_id.
std_msgs/Header header
# HEADER column (normally "a").
string record_type
# TIME_MS: milliseconds since ROBOBIKE boot (raw value).
uint32 time_ms
# SV_DRV [%]
float32 sv_drv
# SV_STR [deg], + = right
float32 sv_str
# SV_STD [deg]
float32 sv_std
# GY_ROLL [deg/s]
float32 gy_roll
# GY_YAW [deg/s]
float32 gy_yaw
# GY_PITCH [deg/s]
float32 gy_pitch
# Only in the 13-column format (firmware v1033+); NaN otherwise.
float32 swp_sg
# SV_POS [deg]
float32 sv_pos
# ACC_X/Y/Z [m/s^2]
float32 acc_x
float32 acc_y
float32 acc_z
