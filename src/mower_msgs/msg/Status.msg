uint8 MOWER_STATUS_INITIALIZING=0
uint8 MOWER_STATUS_OK=255



time stamp
# Low level stuff
# Initialized or not
uint8 mower_status
bool raspberry_pi_power
bool charging_allowed
bool esc_power
bool rain_detected
bool uss_timeout
bool imu_timeout
bool battery_empty

float32[5] uss_ranges
uint16[5] uss_age_ms
bool emergency
float32 v_charge
float32 v_battery
float32 charge_current

# ESC stuff
ESCStatus left_esc_status
ESCStatus right_esc_status
ESCStatus mow_esc_status

