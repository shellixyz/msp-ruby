
class MSP

  USER_CONTROL_MODE = %i[ atti cruise ]
  BATTERY_STATE = %i[ ok warning critical not_present ]
  SENSOR_STATUS = %i[ acc baro mag GPS rangefinder opflow pitot temp ]
  RTH_ALLOW_LANDING = %i[ never always fs_only ]
  RTH_ALT_CONTROL_MODE = %i[ current extra fixed max at_least at_least_linear_descent ]
  CURRENT_SENSOR_TYPE = %i[ none ADC virtual ]
  SERIALRX_PROVIDER = %i[ spektrum1024 spektrum2048 sbus sumd sumh xbus_mode_b xbus_mode_b_rj01 ibus jetiexbus crsf fport ]
  RECEIVER_TYPE = %i[ none pwm ppm serial msp spi uib ]
  ADJUSTMENT_FUNCTION = %i[ none rc_rate rc_expo throttle_expo pitch_roll_rate yaw_rate pitch_roll_p pitch_roll_i pitch_roll_d yaw_p yaw_i yaw_d rate_profile pitch_rate roll_rate pitch_p pitch_i pitch_d roll_p roll_i roll_d rc_yaw_expo manual_rc_expo manual_rc_yaw_expo manual_pitch_roll_rate manual_roll_rate manual_pitch_rate manual_yaw_rate nav_fw_cruise_thr nav_fw_pitch2thr roll_board_alignment pitch_board_alignment level_p level_i level_d pos_xy_p pos_xy_i pos_xy_d pos_z_p pos_z_i pos_z_d heading_p vel_xy_p vel_xy_i vel_xy_d vel_z_p vel_z_i vel_z_d fw_min_throttle_down_pitch_angle vtx_power_level profile ]
  BAUD_RATE = [ :auto, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 460800, 921600, 1000000, 1500000, 2000000, 2470000 ]
  FAILSAFE_PROCEDURE = %i[ set_thr drop rth none ]
  SDCARD_STATE = %i[ not_present fatal card_init FS_init ready ]
  FAT_FS_ERROR = %i[ none generic bad_MBR bad_FS_header ]
  VIDEO_SYSTEM = %i[ auto PAL NTSC ]
  OSD_UNITS = %i[ imperial metric UK ]
  VTX_DEVICE_TYPE = %i[ RTC6705 reserved smartaudio tramp FFPV ]
  MOTOR_OUTPUT_PROTOCOL = %i[ standard oneshot125 oneshot42 multishot brushed dshot150 dshot300 dshot600 dshot1200 serialshot ]
  HARDWARE = {
    acc: %i[ none autodetect ADXL345 MPU6050 MMA8452 BMA280 LSM303DLHC MPU6000 MPU6500 MPU9250 BMI160 ICM20689 fake ],
    baro: %i[ none autodetect BMP085 MS5611 BMP280 MS5607 LPS25H fake ],
    mag: %i[ none autodetect HMC5883 AK8975 GPS MAG3110 AK8963 IST8310 QMC5883 MPU9250 IST8308 LIS3MDL fake ],
    pitot: %i[ none autodetect MS4525 ADC virtual fake ],
    rangefinder: %i[ none HCSR04 SRF10 HCSR04I2C VL53L0X MSP UIB BENEWAKE ],
    opflow: %i[ none PMW3901 CXOF MSP fake ]
  }
  RSSI_SOURCE = %i[ none ADC rx_channel rx_protocol MSP ]
  GPS_FIX_TYPE = %i[ none _2D _3D ]
  GPS_PROVIDER = %i[ NMEA ublox I2CNAV naza ublox7plus MTK ]
  GPS_SBAS_MODE = %i[ auto EGNOS WAAS MSAS GAGAN none ]
  PID_ITEMS = %i[ roll pitch yaw pos_z pos_xy vel_xy surface level heading vel_z ]
  WP_ACTION = { waypoint: 0x01, RTH: 0x04 }
  WP_FLAGS = { last: 0xA5 }
  NAV_SYSTEM_MODE = { none: 0, hold: 1, RTH: 2, nav: 3, emerg: 15 }
  NAV_SYSTEM_STATE = %i[ none RTH_start RTH_enroute hold_infinit hold_timed WP_enroute process_next do_jump land_start land_in_progress landed land_settle land_start_descent hover_above_home emergency_landing ]
  NAV_SYSTEM_ERROR = %i[ none toofar spoiled_GPS WP_CRC finish timewait invalid_jump invalid_data wait_for_RTH_alt GPS_fix_lost disarmed landing ]
  NAV_SYSTEM_FLAGS = { adjusting_position: 1<<0, adjusting_altitude: 1<<1 }
  SENSOR_ALIGNMENT = %i[ default CW_0_deg CW_90_deg CW_180_deg CW_270_deg CW_0_deg_flip CW_90_deg_flip CW_180_deg_flip CW_270_deg_flip ]
  HARDWARE_STATUS = %i[ none OK unavailable unhealthy ]
  SERVO_MIXER_INPUT = %i[ stabilized_roll stabilized_pitch stabilized_yaw stabilized_throttle RC_roll RC_pitch RC_yaw RC_throttle RC_ch5 RC_ch6 RC_ch7 RC_ch8 gimbal_pitch gimbal_roll feature_flaps RC_ch9 RC_ch10 RC_ch11 RC_ch12 RC_ch13 RC_ch14 RC_ch15 RC_ch16 stabilized_roll_plus stabilized_roll_minus stabilized_pitch_plus stabilized_pitch_minus stabilized_yaw_plus stabilized_yaw_minus logic_one ]
  SETTING_TYPE = %i[ uint8 int8 uint16 int16 uint32 float string ]
  SETTING_SECTION = %i[ master profile control_rate_profile battery_profile ]
  SETTING_MODE = %i[ direct lookup ]
  BATTERY_CAPACITY_UNIT = %i[ mAh mWh ]
  VOLTAGE_SOURCE = %i[ raw sag_compensated ]
  PLATFORM_TYPE = %i[ multirotor airplane helicopter tricopter rover boat other ]
  OSD_CROSSHAIR_STYLE = %i[ default aircraft type3 type4 type5 type6 type7 ]
  OSD_SIDEBAR_SCROLL = %i[ none altitude ground_speed home_distance ]
  BLACKBOX_DEVICE = %i[ serial flash SDcard ]
  TEMPERATURE_SENSOR_TYPE = %i[ none LM75 DS18B20 ]
  LOGIC_CONDITION_OPERATION = %i[ true equal greater_than lower_than low mid high and or xor nand nor not ]
  LOGIC_CONDITION_OPERAND_TYPE = %i[ value RC_channel flight LC last ]
  LOGIC_CONDITION_FLIGHT_OPERAND = %i[ arm_timer home_distance trip_distance RSSI vbat cell_voltage current mAh_drawn GPS_sats, groud_speed 3D_speed air_speed altitude vertical_speed trottle_pos attitude_roll attitude_pitch ]

  LOGIC_CONDITION_FLAGS = {
    latch: 1<<0
  }

  YAW_MOTOR_DIRECTION = {
    standard: 1,
    reversed: -1
  }

  TIMER_USAGE = {
    any: 0,
    PPM: (1 << 0),
    PWM: (1 << 1),
    MC_motor: (1 << 2),
    MC_servo: (1 << 3),
    MC_chnfw: (1 << 4),
    FW_motor: (1 << 5),
    FW_servo: (1 << 6),
    LED: (1 << 24),
    beeper: (1 << 25)
  }

  SERIAL_PORT_FUNCTIONS = {
    msp: (1 << 0),
    gps: (1 << 1),
    telemetry_frsky: (1 << 2),
    telemetry_hott: (1 << 3),
    telemetry_ltm: (1 << 4),
    telemetry_smartport: (1 << 5),
    rx_serial: (1 << 6),
    blackbox: (1 << 7),
    telemetry_mavlink: (1 << 8),
    telemetry_ibus: (1 << 9),
    rcdevice: (1 << 10),
    vtx_smartaudio: (1 << 11),
    vtx_tramp: (1 << 12),
    uav_interconnect: (1 << 13),
    optical_flow: (1 << 14),
    log: (1 << 15),
    rangefinder: (1 << 16),
    vtx_ffpv: (1 << 17),
    serialshot: (1 << 18),
    telemetry_sim: (1 << 19)
  }

  ARMING_FLAGS = {
    armed: 1<<2,
    was_ever_armed: 1<<3,
    disabled_failsafe_system: 1<<7,
    disabled_not_level: 1<<8,
    disabled_sensors_calibrating: 1<<9,
    disabled_system_overloaded: 1<<10,
    disabled_navigation_unsafe: 1<<11,
    disabled_compass_not_calibrated: 1<<12,
    disabled_accelerometer_not_calibrated: 1<<13,
    disabled_arm_switch: 1<<14,
    disabled_hardware_failure: 1<<15,
    disabled_box_failsafe: 1<<16,
    disabled_box_killswitch: 1<<17,
    disabled_rc_link: 1<<18,
    disabled_throttle: 1<<19,
    disabled_CLI: 1<<20,
    disabled_CMS_menu: 1<<21,
    disabled_OSD_menu: 1<<22,
    disabled_rollpitch_not_centered: 1<<23,
    disabled_servo_autotrim: 1<<24,
    disabled_oom: 1<<25,
    disabled_invalid_setting: 1<<26,
    disabled_pwm_output_error: 1<<27
  }

  FEATURES = {
    :thr_vbat_comp => 1<<0,
    :vbat => 1<<1,
    :tx_prof_sel => 1<<2,
    :bat_profile_autoswitch => 1<<3,
    :motor_stop => 1<<4,
    :softserial => 1<<6,
    :gps => 1<<7,
    :telemetry => 1<<10,
    :current_meter => 1<<11,
    :_3D => 1<<12,
    :rssi_adc => 1<<15,
    :led_strip => 1<<16,
    :dashboard => 1<<17,
    :blackbox => 1<<19,
    :transponder => 1<<21,
    :airmode => 1<<22,
    :superexpo_rates => 1<<23,
    :vtx => 1<<24,
    :pwm_servo_driver => 1<<27,
    :pwm_output_enable => 1<<28,
    :osd => 1<<29,
    :fw_launch => 1<<30
  }

end
