class MSP

  module V1

    COMMANDS = {
      1 => :api_version,
      2 => :fc_variant,
      3 => :fc_version,
      4 => :board_info,
      5 => :build_info,
      6 => :inav_pid,
      7 => :set_inav_pid,
      10 => :name,
      11 => :set_name,
      12 => :nav_poshold,
      13 => :set_nav_poshold,
      14 => :calibration_data,
      15 => :set_calibration_data,
      16 => :position_estimation_config,
      17 => :set_position_estimation_config,
      18 => :wp_mission_load,
      19 => :wp_mission_save,
      20 => :wp_getinfo,
      21 => :rth_and_land_config,
      22 => :set_rth_and_land_config,
      23 => :fw_config,
      24 => :set_fw_config,
      34 => :mode_ranges,
      35 => :set_mode_range,
      36 => :feature,
      37 => :set_feature,
      38 => :board_alignment,
      39 => :set_board_alignment,
      40 => :current_meter_config,
      41 => :set_current_meter_config,
      44 => :rx_config,
      45 => :set_rx_config,
      46 => :led_colors,
      47 => :set_led_colors,
      48 => :led_strip_config,
      49 => :set_led_strip_config,
      50 => :rssi_config,
      51 => :set_rssi_config,
      52 => :adjustment_ranges,
      53 => :set_adjustment_range,
      54 => :cf_serial_config,
      55 => :set_cf_serial_config,
      56 => :voltage_meter_config,
      57 => :set_voltage_meter_config,
      58 => :sonar_altitude,
      61 => :arming_config,
      62 => :set_arming_config,
      64 => :rx_map,
      65 => :set_rx_map,
      68 => :reboot,
      69 => :BF_build_info,
      70 => :dataflash_summary,
      71 => :dataflash_read,
      72 => :dataflash_erase,
      73 => :loop_time,
      74 => :set_loop_time,
      75 => :failsafe_config,
      76 => :set_failsafe_config,
      79 => :sdcard_summary,
      84 => :osd_config,
      85 => :set_osd_config,
      87 => :osd_char_write,
      88 => :vtx_config,
      89 => :set_vtx_config,
      90 => :advanced_config,
      91 => :set_advanced_config,
      92 => :filter_config,
      93 => :set_filter_config,
      94 => :pid_advanced,
      95 => :set_pid_advanced,
      96 => :sensor_config,
      97 => :set_sensor_config,
      101 => :status,
      102 => :raw_imu,
      103 => :servo,
      104 => :motor,
      105 => :rc,
      106 => :raw_gps,
      107 => :comp_gps,
      108 => :attitude,
      109 => :altitude,
      110 => :analog,
      111 => :rc_tuning,
      112 => :pid,
      113 => :activeboxes,
      114 => :misc,
      116 => :boxnames,
      117 => :pidnames,
      118 => :wp,
      119 => :boxids,
      120 => :servo_configurations,
      121 => :nav_status,
      124 => :_3D,
      125 => :rc_deadband,
      126 => :sensor_alignment,
      127 => :led_strip_modecolor,
      150 => :status_ex,
      151 => :sensor_status,
      160 => :uid,
      164 => :gpssvinfo,
      166 => :gpsstatistics,
      186 => :set_tx_info,
      187 => :tx_info,
      200 => :set_raw_rc,
      201 => :set_raw_gps,
      202 => :set_pid,
      204 => :set_rc_tuning,
      205 => :acc_calibration,
      206 => :mag_calibration,
      207 => :set_misc,
      208 => :reset_conf,
      209 => :set_wp,
      210 => :select_setting,
      211 => :set_head,
      212 => :set_servo_configuration,
      214 => :set_motor,
      217 => :set_3D,
      218 => :set_rc_deadband,
      219 => :set_reset_curr_pid,
      220 => :set_sensor_alignment,
      221 => :set_led_strip_modecolor,
      241 => :servo_mix_rules,
      242 => :set_servo_mix_rule,
      246 => :rtc,
      247 => :set_rtc,
      250 => :eeprom_write,
      254 => :debug,
    }

  end # V1

  module V2

    COMMANDS = {
      0x1001 => :common_tz,
      0x1002 => :common_set_tz,
      0x1003 => :common_setting,
      0x1004 => :common_set_setting,
      0x1005 => :common_motor_mixer,
      0x1006 => :common_set_motor_mixer,
      0x1007 => :common_setting_info,
      0x1008 => :common_pg_list,
      0x1009 => :common_serial_config,
      0x100A => :common_set_serial_config,
      0x100B => :common_set_radar_pos,
      0x2000 => :inav_status,
      0x2001 => :inav_optical_flow,
      0x2002 => :inav_analog,
      0x2003 => :inav_misc,
      0x2004 => :inav_set_misc,
      0x2005 => :inav_battery_config,
      0x2006 => :inav_set_battery_config,
      0x2007 => :inav_rate_profile,
      0x2008 => :inav_set_rate_profile,
      0x2009 => :inav_air_speed,
      0x200A => :inav_output_mapping,
      0x200B => :inav_mc_braking,
      0x200C => :inav_set_mc_braking,
      0x2010 => :inav_mixer,
      0x2011 => :inav_set_mixer,
      0x2012 => :inav_osd_layouts,
      0x2013 => :inav_osd_set_layout_item,
      0x2014 => :inav_osd_alarms,
      0x2015 => :inav_osd_set_alarms,
      0x2016 => :inav_osd_preferences,
      0x2017 => :inav_osd_set_preferences,
      0x2018 => :inav_select_battery_profile,
      0x2019 => :inav_debug,
      0x201A => :blackbox_config,
      0x201B => :set_blackbox_config,
      0x201C => :inav_temp_sensor_config,
      0x201D => :inav_set_temp_sensor_config,
      0x201E => :inav_temperatures,
      0x2020 => :inav_servo_mixer,
      0x2021 => :inav_set_servo_mixer,
      0x2022 => :inav_logic_conditions,
      0x2023 => :inav_set_logic_conditions,
      0x2030 => :pid,
      0x2031 => :set_pid,
      0x2032 => :inav_opflow_calibration,
    }

  end # V2

end # MSP
