uint16 ANALYTIC_HOLD = 0
uint16 ANALYTIC_CIRCLE = 1
uint16 ANALYTIC_CIRCLE_ENTRY = 3
uint16 ANALYTIC_FIGURE_EIGHT = 4

std_msgs/Header header
uint32 request_id
uint32 trajectory_id
uint32 revision
uint16 analytic_type
uint32 flags
time start_time
float64 duration
geometry_msgs/Pose origin
float64[] params
