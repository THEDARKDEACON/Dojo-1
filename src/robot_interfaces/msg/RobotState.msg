# Overall robot state message

std_msgs/Header header

# Robot operational states
uint8 UNKNOWN = 0
uint8 INITIALIZING = 1
uint8 IDLE = 2
uint8 MANUAL_CONTROL = 3
uint8 AUTONOMOUS = 4
uint8 ERROR = 5
uint8 EMERGENCY_STOP = 6
uint8 SHUTDOWN = 7

uint8 current_state
uint8 previous_state
builtin_interfaces/Time state_change_time

# System health
bool hardware_ok
bool software_ok
bool communication_ok
bool sensors_ok

# Current capabilities
bool can_move
bool can_navigate
bool can_perceive

# Error information
string[] active_errors
string[] warnings

# Performance metrics
float64 cpu_usage
float64 memory_usage
float64 network_usage