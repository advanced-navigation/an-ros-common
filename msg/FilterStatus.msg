# Filter Status bits of ANPP packet 20
#
# Specified using the Advanced Navigation Packet Protocol

# This field contains 16 bits that indicate the status of the filters.
# These are boolean fields with a zero indicating false and one indicating true.

bool orientation_filter_initialised
bool ins_filter_initialised
bool heading_initialised
bool utc_time_initialised

# GNSS Fix Status information.
adnav_interfaces/GNSSFixStatus gnss_fix_status

bool event1_flag
bool event2_flag
bool internal_gnss_enabled
bool dual_antenna_heading_active
bool velocity_heading_enabled
bool atmospheric_altitude_enabled
bool external_position_active
bool external_velocity_active
bool external_heading_active