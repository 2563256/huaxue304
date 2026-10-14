# Live state of one robot in the fleet. Published under <ns>/robot_state and
# aggregated by the scheduler into FleetStatus.

string   id                       # namespace / robot id, e.g. "light_1"
string   type                     # light_diff_amr | heavy_diff_amr | omni_mecanum_amr | g1_humanoid

geometry_msgs/Pose2D pose         # current pose in the map frame
string   status                   # IDLE|ASSIGNED|TO_PICKUP|LOADING|TO_DROPOFF|CHARGING|FAILED

float32  battery_pct              # 0..100
float32  battery_capacity_wh
float32  payload_capacity_kg
float32  current_payload_kg
float32  max_speed                # m/s

string[] capabilities             # e.g. ["light_payload", "narrow_aisle"]
float32  estimated_idle_time_sec  # 0 when idle; remaining busy time otherwise

string   current_task_id          # "" when idle
float32  odom_distance_m          # accumulated travelled distance
float32  energy_consumed_wh       # accumulated energy
int32    tasks_completed
