uint8 MOWER_STATUS_INITIALIZING=0
uint8 MOWER_STATUS_OK=255

time stamp
# Low level stuff
# Initialized or not
uint8 mower_status
bool raspberry_pi_power
bool charging
bool esc_power
bool rain_detected
bool uss_timeout
bool imu_timeout
bool bms_timeout
bool ll_timeout
#internal status from LL
bool battery_empty

float32[5] uss_ranges
uint16[5] uss_age_ms
bool emergency
bool temporary_emergency
#percent 0-100
uint8 battery_soc
float32 v_charge
float32 v_battery
#charging positive, discharging negative
float32 battery_current
bool mow_enabled

# ESC stuff
ESCStatus rear_left_esc_status
ESCStatus rear_right_esc_status
ESCStatus front_left_esc_status
ESCStatus front_right_esc_status
ESCStatus mow_esc_status
