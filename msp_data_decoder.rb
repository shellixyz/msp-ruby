
require_relative 'msp_enums'

class MSP

  def self.decode_value packed_value, type
    unpack_string =
      case type
      when :uint8 then ?C
      when :int8 then ?c
      when :uint16 then ?S
      when :int16 then ?s
      when :uint32 then ?L
      when :float then ?f
      when :string then ?Z
      else raise ArgumentError, "unknown type: #{type}, valid: #{SETTING_TYPE}"
      end
    packed_value.unpack1 unpack_string
  end

  class APIVersion < Struct.new(:msp_protocol_version, :api_version_major, :api_version_minor)

      include Comparable

      def <=> other
          mpv_comp = msp_protocol_version <=> other.msp_protocol_version
          return mpv_comp if mpv_comp != 0
          avm_comp = api_version_major <=> other.api_version_major
          return avm_comp if avm_comp != 0
          api_version_minor <=> other.api_version_minor
      end

      def to_s
          "#{msp_protocol_version}.#{api_version_major}.#{api_version_minor}"
      end

  end

  module V1

    module DataDecoder

      class << self

	def api_version data
	  Struct.new(:msp_protocol_version, :api_version_major, :api_version_minor).new *data.unpack('CCC')
	  APIVersion.new *data.unpack('CCC')
	end

	def fc_variant data
	  data
	end

	def fc_version data
	  Struct.new(:major, :minor, :patch).new *data.unpack('CCC')
	end

	def board_info data
	  values = data.unpack 'a4SCCx'
	  has_osd = values[2] == 2
	  Struct.new(:board_identifier, :hardware_revision, :has_osd?, :comm_capabilities, :target_name).new *values[0..1], has_osd, values[3], data[9..-1]
	end

	def build_info data
	  values = data.unpack 'a11a8a8'
	  datetime = DateTime.parse values[0..1].join(' ')
	  Struct.new(:build_datetime, :git_short_rev).new datetime, values[2]
	end

	def inav_pid data
	  Struct.new(:heading_hold_rate_limit, :heading_hold_error_lpf_freq, :yaw_jump_prevention_limit, :gyro_lpf, :acc_lpf).new *data.unpack('x5CCSCC')
	end

	def name data
	  data
	end

	def nav_poshold data
	  values = data.unpack('CS4C2S')
	  user_control_mode = USER_CONTROL_MODE[values[0]]
	  use_thr_mid_for_althold = values[6] != 0
	  Struct.new(:user_control_mode, :max_auto_speed, :max_auto_climb_rate, :max_manual_speed, :max_manual_climb_rate, :max_mc_bank_angle, :use_thr_mid_for_althold?, :mc_hover_throttle).new user_control_mode, *values[1..5], use_thr_mid_for_althold, values[7]
	end

	def calibration_data data
	  values = data.unpack 'Cs10'
	  values[10] /= 256.0
	  field_names = %w[ acc_zero acc_gain mag_zero ].flat_map { |prefix| %w[ x y z ].map { |axis| prefix + '_' + axis } }
	  Struct.new(:calibration_axis_flags, *field_names, :opflow_scale).new *values
	end

	def position_estimation_config data
	  values = data.unpack 'S5C2'
	  (0..4).each { |index| values[index] /= 100.0 }
	  values[6] = values[6] != 0
	  Struct.new(:w_z_baro_p, :w_z_gps_p, :w_z_gps_v, :w_xy_gps_p, :w_xy_gps_v, :min_sats, :use_velned?).new *values
	end

	def wp_getinfo data
	  struct = Struct.new :max_waypoints, :mission_valid?, :waypoint_count
	  data = data.unpack 'xCCC'
	  struct.new data[0], (data[1] != 0), data[2]
	end

	def rth_and_land_config data
	  values = data.unpack 'SC5S6'
	  [ 0, 6, 7 ].each { |index| values[index] /= 100.0 }
	  (1..3).each { |index| values[index] = values[index] != 0 }
	  values[4] = RTH_ALLOW_LANDING[values[4]]
	  values[5] = RTH_ALT_CONTROL_MODE[values[5]]
	  Struct.new(:min_rth_distance, :rth_climb_first?, :rth_climb_ignore_emerg?, :rth_tail_first?, :rth_allow_landing, :rth_alt_control_mode, :rth_abort_threshold, :rth_altitude, :land_descent_rate, :land_slowdown_minalt, :land_slowdown_maxalt, :emerg_descent_rate).new *values
	end

	def fw_config data
	  values = data.unpack 'S3C4S'
	  values[7] /= 100.0
	  Struct.new(:cruise_throttle, :min_throttle, :max_throttle, :max_bank_angle, :max_climb_angle, :max_dive_angle, :pitch_to_throttle, :loiter_radius).new *values
	end

	def mode_ranges data
	  rule_count = data.length / 4
	  rule_count.times.map do |index|
	    rule_values = data.slice(index * 4, 4).unpack('C4')
	    Struct.new(:mode_range_index, :box_id, :aux_channel_index, :start_step, :end_step).new index, *rule_values
          end.find_all { |rule| rule.start_step != rule.end_step }
	end

	def feature data
	  feature_mask = data.unpack1 'L'
	  FEATURES.reduce([]) { |enabled, (name, value)| enabled.tap { enabled.push(name) if (feature_mask & value) != 0 } }
	end

	def board_alignment data
	  values = data.unpack('s3').map { |v| v / 100.0 }
	  Struct.new(:roll_align, :pitch_align, :yaw_align).new *values
	end

	def current_meter_config data
	  values = data.unpack 'SsCS'
	  values[2] = CURRENT_SENSOR_TYPE[values[2]]
	  Struct.new(:current_sensor_scale, :current_sensor_offset, :current_sensor_type, :battery_capacity).new *values
	end

	def rx_config data
	  values = data.unpack 'CS3CS2x4CLCxC'
	  values[0] = SERIALRX_PROVIDER[values[0]]
	  values[10] = RECEIVER_TYPE[values[10]]
	  Struct.new(:serialrx_provider, :max_check, :pwm_range_middle, :min_check, :spektrum_sat_bind, :rx_min_usec, :rx_max_usec, :rx_spi_protocol, :rx_spi_id, :rx_spi_rf_channel_count, :receiver_type).new *values
	end

	def led_colors data
	  led_color_count = data.length / 4;
	  led_color_count.times.map do |index|
	    color_values = data.slice(index * 4, 4).unpack 'SCC'
	    Struct.new(:hue, :saturation, :value).new *color_values
	  end
	end

	def led_strip_config data
	  led_count = data.length / 4;
	  led_count.times.map { |index| data.slice(index * 4, 4).unpack1 'L' }
	end

	def rssi_config data
	  Struct.new(:rssi_channel).new data.unpack1('C')
	end

	def adjustment_ranges data
	  adj_range_count = data.length / 6
	  adj_range_count.times.map do |index|
	    adj_range_values = data.slice(index * 6, 6).unpack 'C6'
	    adj_range_values[4] = ADJUSTMENT_FUNCTION[adj_range_values[4]]
	    Struct.new(:adj_index, :aux_channel_index, :start_step, :end_step, :adjustment_function, :aux_switch_channel_index).new *adj_range_values
	  end
	end

	def cf_serial_config data
	  serial_port_count = data.length / 7
	  serial_port_count.times.map do |index|
	    serial_port_values = data.slice(index * 7, 7).unpack 'CSC4'
	    serial_port_values[0] =
	      case
	      when serial_port_values[0] == 20 then :vcp
	      when serial_port_values[0] >= 40 then "softserial#{serial_port_values[0] - 40}".intern
	      else "uart#{serial_port_values[0]}".intern if serial_port_values[0] < 20
	      end
	    serial_port_values[1] = SERIAL_PORT_FUNCTIONS.reduce([]) { |functions, (name, value)| functions.tap { functions.push(name) if (serial_port_values[1] & value) != 0 } }
	    (2..5).each { |index| serial_port_values[index] = BAUD_RATE[serial_port_values[index]] }
	    Struct.new(:identifier, :function_mask, :msp_baudrate, :gps_baudrate, :telemetry_baudrate, :peripheral_baudrate).new *serial_port_values
	  end
	end

	def voltage_meter_config data
	  values = data.unpack('C4')
	  (1..3).each { |index| values[index] /= 10.0 }
	  Struct.new(:voltage_scale, :cell_min, :cell_max, :cell_warning).new *values
	end

	def sonar_altitude data
	  altitude = data.unpack1 'l'
	  case altitude
	  when -1 then raise Rangefinder::OutOfRange
	  when -2 then raise Rangefinder::HardwareFailure
	  when -3 then raise Rangefinder::StaleData
	  else altitude / 100.0
	  end
	end

	def arming_config data
	  disarm_kill_switch = data.unpack1('xC') != 0
	  Struct.new(:disarm_kill_switch?).new disarm_kill_switch
	end

	def rx_map data
	  channels = data.unpack('C4').map { |v| v + 1 }
	  Struct.new(:aileron, :elevator, :rudder, :throttle).new *channels
	end

	def dataflash_summary data
	  values = data.unpack 'CLLL'
	  values[0] = values[0] != 0
	  Struct.new(:flashfs_ready?, :sectors, :total_size, :flashfs_offset).new *values
	end

	def dataflash_read data
	  data
	end

	def loop_time data
	  data.unpack1 'S'
	end

	def failsafe_config data
	  values = data.unpack('C2SxSC2s3S2C')
	  [0, 1, 3, 5, 6, 7].each { |index| values[index] /= 10.0 }
	  values[4] = FAILSAFE_PROCEDURE[values[4]]
	  values[10] /= 100.0
	  values[11] = FAILSAFE_PROCEDURE[values[11]]
	  Struct.new(:delay, :off_delay, :throttle, :throttle_low_delay, :procedure, :recovery_delay, :fw_roll_angle, :fw_pitch_angle, :fw_yaw_rate, :stick_motion_threshold, :min_distance, :min_distance_procedure).new *values
	end

	def sdcard_summary data
	  values = data.unpack 'C3L2'
	  supported = values[0] != 0
	  raise SDCardNotSupported unless supported
	  values[1] = SDCARD_STATE[values[1]]
	  values[2] = FAT_FS_ERROR[values[2]]
	  Struct.new(:sdcard_state, :FAT_FS_error, :contiguous_free_space, :blocks).new values[1..-1]
	end

	def osd_config data
	  values = data.unpack 'C4S*'
	  supported = values[0] != 0
	  raise OSDNotSupported unless supported
	  config_values = values[1..8]
	  config_values[0] = VIDEO_SYSTEM[config_values[0]]
	  config_values[1] = OSD_UNITS[config_values[1]]
	  item_positions = values[9..-1].map { |item_bitmask| MSP.osd_item_bitmask_decode item_bitmask }
	  Struct.new(:video_system, :units, :rssi_alarm, :battery_capacity_warning, :time_alarm, :altitude_alarm, :distance_alarm, :negative_altitude_alarm, :items_positions).new *config_values, item_positions
	end

	def osd_char_write data
	  # TODO
	end

	def vtx_config data
	  values = data.unpack 'C7'
	  values[0] = VTX_DEVICE_TYPE[values[0]]
	  raise VTXNotConfigured if values[0].nil?
	  (4..6).each { |index| values[index] = values[index] != 0 }
	  Struct.new(:device_type, :band, :channel, :power, :pit_mode?, :ready?, :low_power_disarm?).new *values
	end

	def advanced_config data
	  values = data.unpack 'x3CS2C'
	  values[0] = MOTOR_OUTPUT_PROTOCOL[values[0]]
	  values[3] = values[3] != 0
	  Struct.new(:motor_output_protocol, :motor_pwm_rate, :servo_pwm_rate, :gyro_sync).new *values
	end

	def filter_config data
	  Struct.new(:gyro_soft_lpf_hz, :dterm_lpf_hz, :yaw_lpf_hz, :gyro_soft_notch_hz_1, :gyro_soft_notch_cutoff_1, :dterm_soft_notch_hz, :dterm_soft_notch_cutoff, :gyro_soft_notch_hz_2, :gyro_soft_notch_cutoff_2, :acc_notch_hz, :acc_notch_cutoff, :gyro_stage2_lowpass_hz).new *data.unpack('CS11')
	end

	def pid_advanced data
	  values = data.unpack 'x4Sx3CSxS2'
	  values[1] /= 100.0
	  [3, 4].each { |index| values[index] *= 10 }
	  Struct.new(:yaw_p_limit, :dterm_setpoint_weight, :pid_sum_limit, :acceleration_limit_rollpitch, :acceleration_limit_yaw).new *values
	end

	def sensor_config data
	  values = data.unpack 'C6'
	  values.map!.with_index { |v, index| HARDWARE.values[index][v] }
	  Struct.new(:acc, :baro, :mag, :pitot, :rangefinder, :opflow).new *values
	end

	def tx_info data
	  values = data.unpack 'C2'
	  values[0] = RSSI_SOURCE[values[0]]
	  values[1] = values[1] != 0
	  Struct.new(:rssi_source, :rtc_datetime_set?).new *values
	end

	def status data
	  values = data.unpack 'S3LC'
	  sensor_status = SENSOR_STATUS.each.with_index.reduce([]) { |enabled, (sensor, index)| enabled.tap { enabled.push(sensor) if (values[2] & (1 << index)) != 0 } }
	  config_profile = values[4] & 0b1111
	  Struct.new(:cycle_time, :i2c_errors, :enabled_sensors, :mode_flags, :config_profile).new(*values[0..1], sensor_status, values[3], config_profile)
	end

	def status_ex data
	  values = data.slice(11, 5).unpack 'S2C'
	  arming_flags = ARMING_FLAGS.reduce([]) { |arming_flags, (name, value)| arming_flags.tap { arming_flags.push(name) if (values[1] & value) != 0 } }
	  Struct.new(:cycle_time, :i2c_errors, :enabled_sensors, :mode_flags, :config_profile, :system_load, :arming_flags, :calibration_axis_flags).new *status(data), values[0], arming_flags, values[2]
	end

	def raw_imu data
	  values = data.unpack 's9'
	  (0..2).each { |index| values[index] /= 512.0 }
	  Struct.new(:acc, :gyro, :mag).new values[0..2], values[3..5], values[6..8]
	end

	def servo data
	  data.unpack 'S*'
	end

	def motor data
	  data.unpack 'S*'
	end

	def rc data
	  data.unpack 'S*'
	end

	def raw_gps data
	  values = data.unpack 'C2l2S4'
	  values[0] = GPS_FIX_TYPE[values[0]]
	  Struct.new(:fix_type, :sat_count, :lat, :lon, :alt, :ground_speed, :ground_course, :hdop).new *values
	end

	def comp_gps data
	  values = data.unpack 'S2C'
	  values[2] = values[2] != 0
	  Struct.new(:distance_to_home, :direction_to_home, :heartbeat?).new *values
	end

	def attitude data
	  values = data.unpack 's2S'
	  [0, 1].each { |index| values[index] /= 10.0 }
	  Struct.new(:roll, :pitch, :yaw).new *values
	end

	def altitude data
	  values = data.unpack 'lsl'
	  values.map! { |v| v / 100.0 }
	  Struct.new(:estimated_z_position, :estimated_z_velocity, :baro_altitude).new *values
	end

	def analog data
	  values = data.unpack 'CS2s'
	  values[0] /= 100.0
	  values[2] = values[2] * 100 / 1024.0
	  values[3] /= 100.0
	  Struct.new(:battery_voltage, :mah_drawn, :rssi, :current).new *values
	end

	def rc_tuning data
	  values = data.unpack 'xC7SC'
	  Struct.new(:rc_expo, :roll_rate, :pitch_rate, :yaw_rate, :tpa_rate, :thr_mid, :thr_expo, :tpa_breakpoint, :yaw_expo).new *values
	end

	def pid data
	  values = data.unpack 'C*'
	  pids = PID_ITEMS.map.with_index { |item, index| index *= 3; Struct.new(:p, :i, :d).new *values.slice(index, 3) }
	  Struct.new(*PID_ITEMS).new *pids
	end

	def activeboxes data
	  data.unpack1 'Q'
	end

	def misc data
	  values = data.unpack 'x2S4CxCxCxsC4'
	  values[4] = GPS_PROVIDER[values[4]]
	  values[5] = GPS_SBAS_MODE[values[5]]
	  [7, 9, 10, 11].each { |index| values[index] /= 10.0 }
	  Struct.new(:min_throttle, :max_throttle, :min_command, :failsafe_throttle, :gps_provider, :gps_sbas_mode, :rssi_channel, :mag_declination, :voltage_scale, :battery_cell_min, :battery_cell_max, :battery_cell_warning).new *values
	end

	def boxnames data
	  data.split(';').map { |name| name.gsub(' ', '_').downcase.intern }
	end

	def pidnames data
	  data.split ';'
	end

	def wp data
	  values = data.unpack 'C2l3s3C'
	  values[1] = WP_ACTION.invert[values[1]]
	  values[8] = WP_FLAGS.reduce([]) { |flags, (name, value)| flags.tap { flags.push(name) if (values[8] & value) != 0 } }
	  Struct.new(:wp_index, :action, :lat, :lon, :altitude, :p1, :p2, :p3, :flags).new *values
	end

	def boxids data
	  data.unpack 'C*'
	end

	def servo_configurations data
	  servo_count = data.length / 14
	  servo_count.times.map do |index|
	    Struct.new(:min, :max, :middle, :rate).new *data.slice(index * 14, 14).unpack('S3cx7')
	  end
	end

	def nav_status data
	  values = data.unpack 'C5s'
	  values[0] = NAV_SYSTEM_MODE.invert[values[0]]
	  values[1] = NAV_SYSTEM_STATE[values[1]]
	  values[2] = WP_ACTION.invert[values[2]]
	  values[4] = NAV_SYSTEM_ERROR[values[4]]
	  Struct.new(:nav_mode, :nav_state, :wp_action, :wp_index, :nav_error, :heading_hold_target).new *values
	end

	def _3D data
	  Struct.new(:deadband_low, :deadband_high, :neutral).new *data.unpack('S3')
	end

	def rc_deadband data
	  Struct.new(:deadband, :yaw_deadband, :alt_hold_deadband, :throttle_3D_deadband).new *data.unpack('C3S')
	end

	def sensor_alignment data
	  values = data.unpack 'C4'
	  values.map! { |v| SENSOR_ALIGNMENT[v] }
	  Struct.new(:gyro_align, :acc_align, :mag_align, :opflow_align).new *values
	end

	def led_strip_modecolor data
	  led_mode_colors = Array.new(6) { [] }
	  6.times do |led_mode_index|
	    6.times do |led_direction_index|
	      led_mode_colors[led_mode_index] << data.getbyte(led_mode_index * led_direction_index * 3)
	    end
	  end
	  special_colors_offset = 6 * 6
	  special_colors = 11.times.map { |special_color_index| data.getbyte(special_colors_offset + special_color_index) }
	  Struct.new(:led_mode_colors, :special_colors).new led_mode_colors, special_colors
	end

	def debug data
	  data.unpack 's4'
	end

	def sensor_status data
	  values = data.unpack 'C9'
	  values[0] = values[0] != 0
	  (1..8).each { |index| values[index] = HARDWARE_STATUS[values[index]] }
	  Struct.new(:hardware_healthy?, :gyro_status, :acc_status, :compass_status, :baro_status, :GPS_status, :rangefinder_status, :pitot_status, :opflow_status).new *values
	end

	def uid data
	  data.unpack 'L3'
	end

	def gpssvinfo data
	  Struct.new(:hdop).new data.unpack1('x3Cx')
	end

	def gpsstatistics data
	  values = data.unpack 'SL3S3'
	  Struct.new(:last_message_dt, :errors, :timeouts, :packet_count, :hdop, :eph, :epv).new *values
	end

	def servo_mix_rules data
	  rules_count = data.length / 8
	  rules_count.times.map do |index|
	    values = data.slice(index * 8, 8).unpack 'C2sCx3'
	    values[1] = SERVO_MIXER_INPUT[values[1]]
	    Struct.new(:rule_index, :output, :input, :rate, :speed).new *[ index, *values ]
	  end.find_all { |mr| mr.rate != 0 }
	end

	def rtc data
	  Struct.new(:seconds, :millisecons).new *data.unpack('LS')
	end

      end

    end

  end # V1

  module V2

    module DataDecoder

      class << self

	def common_tz data
	  data.unpack1 's'
	end

	def common_setting data
	  data
	end

	def common_motor_mixer data
	  rule_count = data.length / 8
	  rule_count.times.map do |index|
	    rule_values = data.slice(index * 8, 8).unpack('SSSS').map { |v| v / 1000.0 }
	    (1..3).each { |index| rule_values[index] -= 2 }
	    Struct.new(:rule_index, :throttle, :roll, :pitch, :yaw).new index, *rule_values
	  end.find_all { |rule| rule.throttle != 0 }
	end

	def common_setting_info data
          name_end = data.index "\0"
          name = data.byteslice 0...name_end
          data = data.byteslice (name_end+1)..-1
	  values = data.unpack 'SC3L2SC2'
	  type = values[1] = SETTING_TYPE[values[1]]
	  section = values[2] = SETTING_SECTION[values[2] >> 4]
	  mode = values[3] = SETTING_MODE[values[3] >> 6]
	  d_end = data.slice 17..-1
	  table = nil
	  value_index =
	    if mode == :lookup and type != :string
              table_data = d_end.match(/\A(?:[\w-]+\x00)+/).to_s
              table = table_data.split "\x00"
              table_data.length
	    else
	      0
	    end
	  value_packed = d_end.slice value_index..-1
	  value = MSP.decode_value value_packed, type
	  Struct.new(:name, :pg_id, :type, :section, :mode, :min, :max, :abs_index, :config_profile, :profile_count, :table, :value).new name, *values, table, value
	end

	def common_pg_list data
	  pg_count = data.length / 6
	  pg_count.times.map do |index|
	    Struct.new(:id, :start, :end).new *data.slice(index * 6, 6).unpack('S3')
	  end
	end

	def common_serial_config data
	  serial_port_count = data.length / 9
	  serial_port_count.times.map do |index|
	    serial_port_values = data.slice(index * 9, 9).unpack 'CLC4'
	    serial_port_values[0] =
	      case
	      when serial_port_values[0] == 20 then :vcp
	      when serial_port_values[0] >= 40 then "softserial#{serial_port_values[0] - 40}".intern
	      else "uart#{serial_port_values[0]}".intern if serial_port_values[0] < 20
	      end
	    serial_port_values[1] = SERIAL_PORT_FUNCTIONS.reduce([]) { |functions, (name, value)| functions.tap { functions.push(name) if (serial_port_values[1] & value) != 0 } }
	    (2..5).each { |index| serial_port_values[index] = BAUD_RATE[serial_port_values[index]] }
	    Struct.new(:identifier, :function_mask, :msp_baudrate, :gps_baudrate, :telemetry_baudrate, :peripheral_baudrate).new *serial_port_values
	  end
	end

	def inav_status data
	  values = data.unpack 'S4CLQ'
	  sensor_status = SENSOR_STATUS.each.with_index.reduce([]) { |enabled, (sensor, index)| enabled.tap { enabled.push(sensor) if (values[2] & (1 << index)) != 0 } }
	  config_profile = values[4] & 0b1111
	  battery_profile = values[4] >> 4
	  arming_flags = ARMING_FLAGS.reduce([]) { |arming_flags, (name, value)| arming_flags.tap { arming_flags.push(name) if (values[5] & value) != 0 } }
	  Struct.new(:cycle_time, :i2c_errors, :enabled_sensors, :system_load, :config_profile, :battery_profile, :arming_flags, :mode_flags).new(*values[0..1], sensor_status, values[3], config_profile, battery_profile, arming_flags, values[6])
	end

	def inav_optical_flow data
	  values = data.unpack 'Cs4'
	  Struct.new(:raw_quality, :flow_rate_x, :flow_rate_y, :body_rate_x, :body_rate_y).new *values
	end

	def inav_analog data
	  values = data.unpack 'CSslllLCS'
	  full = values[0] & 1 != 0
	  uses_cap_thresh = values[0] & 2 != 0
	  battery_state = BATTERY_STATE[(values[0] >> 2) & 0b11]
	  cell_count = values[0] >> 4
	  voltage = values[1] * 0.01
	  current = values[2] * 0.01
	  power = values[3] * 0.01
	  Struct.new(:battery_full_when_plugged_in?, :battery_uses_capacity_thresholds?, :battery_state, :battery_cell_count, :battery_voltage, :current, :power, :mah_drawn, :mwh_drawn, :battery_remaining_capacity, :battery_percentage, :rssi).new(full, uses_cap_thresh, battery_state, cell_count, voltage, current, power, *values[4..-1])
	end

	def inav_misc data
	  values = data.unpack 'S5CxC2sSC2S4L3C'
	  values[5] = GPS_PROVIDER[values[5]]
	  values[6] = GPS_SBAS_MODE[values[6]]
	  values[8] /= 10.0
	  values[10] = VOLTAGE_SOURCE[values[10]]
	  [ *(12..15).to_a, 17, 18 ].each { |index| values[index] /= 100.0 }
	  values[19] = BATTERY_CAPACITY_UNIT[values[19]]
	  Struct.new(:pwm_range_middle, :min_throttle, :max_throttle, :min_command, :failsafe_throttle, :gps_provider, :gps_sbas_mode, :rssi_channel, :mag_declination, :voltage_scale, :voltage_source, :battery_cell_count, :cell_detect_voltage, :cell_min_voltage, :cell_max_voltage, :cell_warning_voltage, :battery_capacity, :battery_capacity_warning, :battery_capacity_critical, :battery_capacity_unit).new *values
	end

	def inav_battery_config data
	  values = data.unpack 'SC2S4sSL3C'
	  values[0] /= 100.0
	  values[1] = VOLTAGE_SOURCE[values[1]]
	  values[12] = BATTERY_CAPACITY_UNIT[values[12]]
	  (3..6).each { |index| values[index] /= 100.0 }
	  [7, 8].each { |index| values[index] /= 10000.0}
	  Struct.new(:voltage_scale, :voltage_source, :battery_cell_count, :cell_detect_voltage, :cell_min_voltage, :cell_max_voltage, :cell_warning_voltage, :current_sensor_offset, :current_sensor_scale, :battery_capacity, :battery_capacity_warning, :battery_capacity_critical, :battery_capacity_unit).new *values
	end

	def inav_rate_profile data
	  Struct.new(:throttle_mid, :throttle_expo, :tpa_rate, :tpa_breakpoint, :stabilized_expo, :stabilized_yaw_expo, :stabilized_roll_rate, :stabilized_pitch_rate, :stabilized_yaw_rate, :manual_expo, :manual_yaw_expo, :manual_roll_rate, :manual_pitch_rate, :manual_yaw_rate).new *data.unpack('C3SC10')
	end

	def inav_air_speed data
	  data.unpack 'L'
	end

	def inav_output_mapping data
	  data.unpack('C*').map do |usage_bitmask|
	    TIMER_USAGE.reduce([]) { |usage, (name, value)| usage.tap { usage.push(name) if (usage_bitmask & value) != 0 } }
	  end
	end

	def inav_mc_braking data
	  values = data.unpack 'S3CS3C'
	  [2, 4].each { |index| values[index] /= 1000.0 }
	  Struct.new(:speed_threshold, :disengage_speed, :timeout, :boost_factor, :boost_timeout, :boost_speed_threshold, :boost_disengage_speed, :bank_angle).new *values
	end

	def inav_mixer data
	  values = data.unpack 'CSC2SC2'
	  values[0] = YAW_MOTOR_DIRECTION.invert[values[0]]
	  values[2] = PLATFORM_TYPE[values[2]]
	  values[3] = values[3] != 0
	  Struct.new(:yaw_motor_direction, :yaw_jump_prevention_limit, :platform_type, :has_flaps?, :applied_mixer_preset, :max_motors, :max_servos).new *values
	end

	def inav_osd_layouts data, command_parms
	  case
	  when %i[ layout_index item_index ].all? { |parm| command_parms.include? parm }
	    MSP.osd_item_bitmask_decode data.unpack1('S')
	  when command_parms.include?(:layout_index)
	    data.unpack('S*').map { |item_bitmask| MSP.osd_item_bitmask_decode item_bitmask }
	  else
	    Struct.new(:layout_count, :item_count).new *data.unpack('C2')
	  end
	end

	def inav_osd_alarms data
	  Struct.new(:rssi, :time, :altitude, :distance, :negative_altitude, :gforce, :gforce_axis_min, :gforce_axis_max, :current, :imu_temp_min, :imu_temp_max, :baro_temp_min, :baro_temp_max).new *data.unpack('CS5s2Cs4')
	end

	def inav_osd_preferences data
	  values = data.unpack 'C9'
	  values[0] = VIDEO_SYSTEM[values[0]]
	  values[2] = values[2] != 0
	  values[3] = OSD_CROSSHAIR_STYLE[values[3]]
	  [4, 5].each { |index| values[index] = OSD_SIDEBAR_SCROLL[values[index]] }
	  values[6] = values[6] != 0
	  values[7] = OSD_UNITS[values[7]]
	  values[8] = BATTERY_CAPACITY_UNIT[values[8]]
	  Struct.new(:video_system, :main_voltage_decimals, :ahi_reverse_roll?, :crosshairs_style, :left_sidebar_scroll, :right_sidebar_scroll, :sidebar_scroll_arrows?, :units, :stats_energy_unit).new *values
	end

	def inav_debug data
	  data.unpack 'l*'
	end

	def blackbox_config data
	  supported, *values = data.unpack 'C2S2'
	  raise BackboxNotSupported if supported != 1
	  values[0] = BLACKBOX_DEVICE[values[0]]
	  Struct.new(:device_type, :rate_num, :rate_denom).new *values
	end

	def inav_temp_sensor_config data
	  sensor_count = data.length / 18
	  sensor_count.times.map do |index|
	    values = data.slice(index * 18, 18).unpack 'CQs2CZ4'
	    values[0] = TEMPERATURE_SENSOR_TYPE[values[0]]
	    [2, 3].each { |index| values[index] /= 10.0 }
	    Struct.new(:type, :address, :alarm_min, :alarm_max, :osd_symbol, :label).new *values
	  end.find_all { |sensor| sensor.type != :none }
	end

	def inav_temperatures data
	  data.unpack('s*').map { |temp| temp == -1000 ? :invalid : temp / 10.0 }
	end

	def inav_servo_mixer data
	  rules_count = data.length / 6
	  rules_count.times.map do |index|
	    values = data.slice(index * 6, 6).unpack 'C2sCc'
	    values[1] = SERVO_MIXER_INPUT[values[1]]
	    Struct.new(:rule_index, :output, :input, :rate, :speed, :condition_id).new *[ index, *values ]
	  end.find_all { |mr| mr.rate != 0 }
	end

	def inav_logic_conditions data
	  conditions_count = data.length / 13
	  conditions_count.times.map do |index|
	    values = data.slice(index * 13, 13).unpack 'C3lClC'
	    values[0] = values[0] != 0
	    values[1] = LOGIC_CONDITION_OPERATION[values[1]]
	    [2, 4].each { |index| values[index] = LOGIC_CONDITION_OPERAND_TYPE[values[index]] }
	    [3, 5].each { |index| values[index] = LOGIC_CONDITION_FLIGHT_OPERAND[values[index]] if values[index-1] == :flight }
	    values[6] = LOGIC_CONDITION_FLAGS.reduce([]) { |flags, (name, value)| flags.tap { flags.push(name) if (values[6] & value) != 0 } }
	    Struct.new(:enabled?, :operation, :operand_a_type, :operand_a_value, :operand_b_type, :operand_b_value, :flags).new *values
	  end
	end

	def pid data
	  values = data.unpack 'C*'
	  pids = PID_ITEMS.map.with_index { |item, index| index *= 4; Struct.new(:p, :i, :d, :ff).new *values.slice(index, 4) }
	  Struct.new(*PID_ITEMS).new *pids
	end

      end

    end # DataDecoder

  end # V2

end # MSP
