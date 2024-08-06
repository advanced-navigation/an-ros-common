# System Status bits of ANPP packet 20
#
# Specified using the Advanced Navigation Packet Protocol

# This field contains 16 bits that indicate problems with the system.
# These are boolean fields with a zero indicating false and one indicating true.
bool system_failure
bool accelerometer_sensor_failure
bool gyroscope_sensor_failure
bool magnetometer_sensor_failure
bool pressure_sensor_failure
bool gnss_failure
bool accelerometer_over_range
bool gyroscope_over_range
bool magnetometer_over_range
bool pressure_over_range
bool minimum_temperature_alarm
bool maximum_temperature_alarm
bool internal_data_logging_error
bool high_voltage_alarm
bool gnss_antenna_fault
bool serial_port_overflow_alarm