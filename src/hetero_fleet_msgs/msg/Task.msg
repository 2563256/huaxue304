# Unified transport task model for the heterogeneous fleet.
# A task is a pickup -> dropoff job, optionally constrained by payload,
# required robot capability, priority and a deadline.

string   id                    # unique task id, e.g. "task_001"
string   type                  # transport | charge | inspect

geometry_msgs/Pose2D pickup    # where to collect the payload
geometry_msgs/Pose2D dropoff   # where to deliver the payload

float32  payload_kg            # payload mass in kg (0.0 for non-transport)
string   required_capability   # light_payload | heavy_payload | precise_docking | omnidirectional | ""
int32    priority              # 1 (low) .. 5 (high)
float32  deadline_sec          # soft deadline, seconds from task arrival

# --- runtime bookkeeping (filled in by the scheduler) ---
float32  created_at_sec        # sim time when the task arrived
string   assigned_robot        # "" while unassigned
string   state                 # PENDING|ASSIGNED|TO_PICKUP|LOADING|TO_DROPOFF|COMPLETED|FAILED|REQUEUED
