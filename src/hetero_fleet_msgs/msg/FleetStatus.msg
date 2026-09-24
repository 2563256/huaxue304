# Aggregate snapshot of the whole fleet, published by fleet_manager.
# This is what the RViz panel and the metrics logger consume.

std_msgs/Header header

RobotState[] robots               # one entry per robot
Task[]       queue                # tasks still awaiting assignment
int32        queue_length

int32        tasks_pending
int32        tasks_active
int32        tasks_completed
int32        tasks_failed

float32      makespan_sec         # elapsed since the first task arrived
int32        reschedule_count     # how many times the scheduler re-ran

float32      total_distance_m
float32      total_energy_wh
