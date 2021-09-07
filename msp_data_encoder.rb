
require_relative 'msp_enums'

class MSP

  module V1

    module DataEncoder

      class << self

	def set_inav_pid heading_hold_rate_limit, yaw_jump_prevention_limit, gyro_lpf, acc_lpf
	  [ 0, 0, 0, heading_hold_rate_limit, 0, yaw_jump_prevention_limit, gyro_lpf, acc_lpf, 0, 0, 0, 0 ].pack('CSSCCSCCC4')
	end

	def set_nav_poshold user_control_mode, max_auto_speed, max_auto_climb_rate, max_manual_speed, max_manual_climb_rate, max_mc_bank_angle, use_thr_mid_for_althold, mc_hover_throttle
	  #[ user_control_mode, max_auto_speed, max_auto_climb_rate, max_manual_speed, max_manual_climb_rate, max_mc_bank_angle, use_thr_mid_for_althold, mc_hover_throttle ].pack 'CS4C2S'
	  user_control_mode_id = USER_CONTROL_MODE.index user_control_mode
	  raise ArgumentError, "invalid user control mode: #{user_control_mode}, valid values are: #{USER_CONTROL_MODE.join(', ')}" if user_control_mode_id.nil?
	  use_thr_mid_for_althold = use_thr_mid_for_althold ? 1 : 0
	  [ user_control_mode_id, *method(__method__).parameters[1..-1].map { |_, name| binding.local_variable_get name } ].pack 'CS4C2S'
	end

	def set_calibration_data *values
	  #raise ArgumentError, "wrong number of arguments (given #{values.count}, expected 10)" if values.count != 10
	  MSP.argument_count_check values.count, 10
	  values[9] = (values[9] * 256).round
	  values.pack 's10'
	end

	def set_position_estimation_config *values
	  #raise ArgumentError, "wrong number of arguments (given #{values.count}, expected 7)" if values.count != 7
	  MSP.argument_count_check values.count, 7
	  (0..4).each { |index| values[index] = (values[index] * 100).round }
	  values[6] = values[6] ? 1 : 0
	  values.pack 'S5C2'
	end

	def set_name name
	  name
	end

	def wp_mission_save
	  "\x00"
	end

        def set_rth_and_land_config *values
	  MSP.argument_count_check values.count, 12
	  [ 0, 6, 7 ].each { |index| values[index] = (values[index] * 100).round }
	  (1..3).each { |index| values[index] = values[index] ? 1 : 0 }
	  values[4] = RTH_ALLOW_LANDING.index values[4]
	  values[5] = RTH_ALT_CONTROL_MODE.index values[5]
	  values.pack 'SC5S6'
	end

	def set_fw_config *values
	  MSP.argument_count_check values.count, 8
	  values[7] = (values[7] * 100).round
	  values.pack 'S3C4S'
	end

	def set_mode_range range_id, box_id, aux_channel_index, start_step, end_step
	  method(__method__).parameters.map { |_, name| binding.local_variable_get name }.pack 'C5'
	end

	def set_feature *features
          invalid_features = features - FEATURES.keys
          raise "invalid features: #{invalid_features.join(', ')}" unless invalid_features.empty?
          feature_mask = features.uniq.sum { |name| FEATURES[name] }
          [ feature_mask ].pack 'L'
	end

	def set_board_alignment roll, pitch, yaw
	  [ roll, pitch, yaw ].map { |v| (v * 100).round }.pack 's3'
	end

	def set_current_meter_config current_sensor_scale, current_sensor_offset, current_sensor_type, battery_capacity
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  values[2] = CURRENT_SENSOR_TYPE.index values[2]
	  raise ArgumentError, "inavlid current sensor type: #{values[2]}, valid: #{CURRENT_SENSOR_TYPE.join(', ')}" if values[2].nil?
	  values.pack 'SsCS'
	end

	def set_rx_config *values
	  MSP.argument_count_check values.count, 11
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  serialrx_provider = SERIALRX_PROVIDER.index values[0]
	  raise ArgumentError, "invalid serialrx provider: #{values[0]}, valid: #{SERIALRX_PROVIDER.join(', ')}" if serialrx_provider.nil?
	  receiver_type = RECEIVER_TYPE.index values[10]
	  raise ArgumentError, "invalid receiver type: #{values[10]}, valid: #{RECEIVER_TYPE.join(', ')}" if receiver_type.nil?
	  [ serialrx_provider, *values[1..9], receiver_type ].pack 'CS3CS2x4CLCxC'
	end

	def set_led_colors *led_colors
	  MSP.argument_count_check led_colors.count, 16
	  led_colors.map(&:to_a).pack('SCC' * 16)
	end

	def set_led_strip_config led_index, led_config
	  [ led_index, led_config ].pack 'CL'
	end

	def set_rssi_config rssi_channel
	  [ rssi_channel ].pack 'C'
	end

	def set_adjustment_ranges range_index, adj_index, aux_channel_index, start_step, end_step, adjustment_function, aux_switch_channel_index
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  values[5] = ADJUSTMENT_FUNCTION.index values[5]
	  raise ArgumentError, "invalid adjustment function: #{adjustment_function}, valid: #{ADJUSTMENT_FUNCTION.join(', ')}" if values[5].nil?
	  values.pack 'C7'
	end

	def set_cf_serial_config *serial_configs
	  serial_configs.map do |serial_config|
	    config_values = serial_config.to_a
	    config_values[0] =
	      case config_values[0]
	      when :vcp
		20
	      when /\Asoftserial(\d+)\Z/
		$1.to_i + 40
	      when /\Auart(\d+)\Z/
		$1.to_i
	      else
		raise ArgumentError, "invalid serial port identifier, valid identifiers start with uart or softserial or can be :vcp"
	      end
	    config_values[1] = config_values[1].sum { |name| SERIAL_PORT_FUNCTIONS[name] }
	    (2..5).each do |index|
	      baud_rate_id = BAUD_RATE.index config_values[index]
	      raise ArgumentError, "invalid baud rate: #{config_values[index]}, valid: #{BAUD_RATE.join(', ')}" if baud_rate_id.nil?
	      config_values[index] = baud_rate_id
	    end
	    config_values.pack 'CSC4'
	  end.join
	end

	def set_voltage_meter_config voltage_scale, cell_min, cell_max, cell_warning
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  (1..3).each { |index| values[index] = (values[index] * 10).round }
	  values.pack 'C4'
	end

	def set_arming_config disarm_kill_switch
	  [ 0, (disarm_kill_switch ? 1 : 0) ].pack 'CC'
	end

	def set_rx_map map
	  map.to_a.map { |v| v - 1 }.pack 'C4'
	end

	def dataflash_read address, size
	  [ address, size ].pack 'LS'
	end

	def set_loop_time value
	  [ value ].pack 'S'
	end

	def set_failsafe_config *values
	  MSP.argument_count_check values.count, 12
	  [0, 1, 3, 5, 6, 7].each { |index| values[index] = (values[index] * 10).round }
	  procedure = FAILSAFE_PROCEDURE.index values[4]
	  raise ArgumentError, "invalid procedure: #{values[4]}, valid: #{FAILSAFE_PROCEDURE.join(', ')}" if procedure.nil?
	  values[4] = procedure
	  values[10] = (values[10] * 100).round
	  min_distance_procedure = FAILSAFE_PROCEDURE.index values[11]
	  raise ArgumentError, "invalid min distance procedure: #{values[11]}, valid: #{FAILSAFE_PROCEDURE.join(', ')}" if min_distance_procedure.nil?
	  values[11] = min_distance_procedure
	  values.pack 'C2SxSC2s3S2C'
	end

	def set_osd_config
	  # TODO
	end

	def set_vtx_config frequency: nil, band: nil, channel: nil, power: nil, pit_mode: nil, low_power_disarm: nil # XXX: to test
	  raise ArgumentError, "frequency or band and channel must be specified" unless frequency or (band and channel)
	  raise ArgumentError, "frequency can't be specified at the same time as band and channel" if frequency and (band or channel)
	  raise ArgumentError, "both power and pit_mode must be specified when one is specified" if (power or pit_mode) and not (power and pit_mode)
	  raise ArgumentError, "power and pit_mode must be specified when low_power_disarm is specified" if low_power_disarm and not (power and pit_mode)
	  frequency = (band - 1) * 8 + (channel - 1) if band and channel # XXX TO CHECK
	  payload = [ frequency ].pack 'S'
	  if power and pit_mode
	    pit_mode = pit_mode ? 1 : 0
	    payload += [ power, pit_mode ].pack 'CC'
	  end
	  if low_power_disarm
	    low_power_disarm = low_power_disarm ? 1 : 0
	    payload += [ low_power_disarm ].pack 'C'
	  end
	  payload
	end

	def set_advanced_config motor_output_protocol, motor_pwm_rate, servo_pwm_rate, gyro_sync
	  motor_output_protocol_value = MOTOR_OUTPUT_PROTOCOL.index motor_output_protocol
	  raise ArgumentError, "invalid motor output protocol: #{motor_output_protocol}, valid: #{MOTOR_OUTPUT_PROTOCOL.join(', ')}" if motor_output_protocol_value.nil?
	  gyro_sync = gyro_sync ? 1 : 0
	  [ 0, 0, 0, motor_output_protocol_value, motor_pwm_rate, servo_pwm_rate, gyro_sync ].pack 'C3CS2C'
	end

	def set_filter_config gyro_soft_lpf_hz, dterm_lpf_hz, yaw_lpf_hz, gyro_soft_notch_hz_1, gyro_soft_notch_cutoff_1, dterm_soft_notch_hz, dterm_soft_notch_cutoff, gyro_soft_notch_hz_2, gyro_soft_notch_cutoff_2, acc_notch_hz, acc_notch_cutoff, gyro_stage2_lowpass_hz
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  values.pack 'CS11'
	end

	def set_pid_advanced yaw_p_limit, dterm_setpoint_weight, pid_sum_limit, acceleration_limit_rollpitch, acceleration_limit_yaw
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  [ 0, 0, yaw_p_limit, 0, 0, 0, dterm_setpoint_weight, pid_sum_limit, 0, acceleration_limit_rollpitch, acceleration_limit_yaw ].pack 'S3C4SCS2'
	end

	def set_sensor_config acc, baro, mag, pitot, rangefinder, opflow
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  values.map!.with_index do |v, index|
	    HARDWARE.values[index].index(v).tap do |value_id|
	      raise ArgumentError, "invalid #{HARDWARE.keys[index]} hardware type: #{v}, valid: #{HARDWARE.values[index].join(', ')}" if value_id.nil?
	    end
	  end
	  values.pack 'C6'
	end

	def set_tx_info rssi
	  [ rssi ].pack 'C'
	end

	def wp index
	  [ index ].pack 'C'
	end

	def set_raw_rc channel_values
	  channel_values.pack 'S*'
	end

	def set_raw_gps has_fix, sat_count, lat, lon, alt, ground_speed
	  has_fix = has_fix ? 1 : 0
	  [ has_fix, sat_count, lat, lon, alt, ground_speed ].pack 'C2l2sS'
	end

	def set_pid pids
	  pids.to_a.flat_map { |pid_section| pid_section.to_a }.pack 'C*'
	end

	def set_rc_tuning rc_expo, roll_rate, pitch_rate, yaw_rate, tpa_rate, throttle_mid, throttle_expo, tpa_breakpoint, yaw_expo
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  [ 0, *values ].pack 'C8SC'
	end

	def set_misc min_throttle, max_throttle, min_command, failsafe_throttle, gps_provider, gps_sbas_mode, rssi_channel, mag_declination, voltage_scale, battery_cell_min, battery_cell_max, battery_cell_warning
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  values[4] = GPS_PROVIDER.index values[4]
	  values[5] = GPS_SBAS_MODE.index values[5]
	  (7..11).each { |index| values[index] = (values[index] * 10).round }
	  [ 0, *values[0..4], 0, values[5], 0, values[6], 0, *values[7..11] ].pack 'S5C6SC4'
	end

	def set_wp wp_index, action, lat, lon, alt, p1, p2, p3, flag
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  values[1] = WP_ACTION[values[1]]
	  values[8] = flag.sum { |flag| WP_FLAGS[flag] }
	  values.pack 'C2l3s3C'
	end

	def select_setting profile_index
	  [ profile_index ].pack 'C'
	end

	def set_head heading
	  [ heading ].pack 'S'
	end

	def set_servo_configuration servo_index, min, max, middle, rate
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  [ *values, *[0] * 7 ].pack 'CS3cx7'
	end

	def set_motor motor_commands
	  MSP.argument_count_check motor_commands.count, 8
	  motor_commands.pack 'S*'
	end

	def set_3D deadband_low, deadband_high, neutral
	  [ deadband_low, deadband_high, neutral ].pack 'S3'
	end

	def set_rc_deadband deadband, yaw_deadband, alt_hold_deadband, deadband_3D_throttle
	  [ deadband, yaw_deadband, alt_hold_deadband, deadband_3D_throttle ].pack 'C3S'
	end

	def set_sensor_alignment gyro_align, acc_align, mag_align, opflow_align
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  values.map! do |v|
	    SENSOR_ALIGNMENT.index(v).tap do |align_index|
	      raise ArgumentError, "invalid alignment setting: #{v}, valid: #{SENSOR_ALIGNMENT.join(', ')}" if align_index.nil?
	    end
	  end
	  values.pack 'C4'
	end

	def set_led_strip_modecolor modecolor_index, function_index, color
	  [ modecolor_index, function_index, color ].pack 'C3'
	end

	def set_servo_mix_rule rule_index, output, input, rate, speed
	  input_id = SERVO_MIXER_INPUT.index input
	  raise ArgumentError, "invalid input: #{input}, valid: #{SERVO_MIXER_INPUT.join(', ')}" if input_id.nil?
	  [ rule_index, output, input_id, rate, speed, 0, 0 ].pack 'C3sCSC'
	end

	def set_rtc seconds, milliseconds
	  [ seconds, milliseconds ].pack 'LS'
	end

      end

    end # DataEncoder

  end # V1

  module V2

    module DataEncoder

      class << self

	def common_set_tz tz
	  [ tz ].pack 's'
	end

	def common_setting setting_selector
	  case
	  when setting_selector.is_a?(Integer)
	    [ 0, setting_selector ].pack 'CS'
	  when setting_selector.is_a?(Symbol) || setting_selector.is_a?(String)
	    setting_selector.to_s + "\x00"
	  else
	    raise ArgumentError, "unsupported setting selector type: #{setting_selector.class}, valid types: Integer, Symbol, String"
	  end
	end

	def common_set_setting setting_selector, value
	  selector = common_setting setting_selector
	end

	def common_set_motor_mixer mixer_index, throttle, roll, pitch, yaw
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  values[1] = (values[1] * 1000).round
	  (2..4).each { |index| values[index] = ((values[index] + 2) * 1000).round}
	  values.pack 'CSSSS'
	end

	alias common_setting_info common_setting

	def common_pg_list last_pg_id = nil
	  [ last_pg_id ].pack 'S' if last_pg_id
	end

	def common_set_serial_config *serial_configs
	  serial_configs.map do |serial_config|
	    config_values = serial_config.to_a
	    config_values[0] =
	      case config_values[0]
	      when :vcp
		20
	      when /\Asoftserial(\d+)\Z/
		$1.to_i + 40
	      when /\Auart(\d+)\Z/
		$1.to_i
	      else
		raise ArgumentError, "invalid serial port identifier, valid identifiers start with uart or softserial or can be :vcp"
	      end
	    config_values[1] = config_values[1].sum { |name| SERIAL_PORT_FUNCTIONS[name] }
	    (2..5).each do |index|
	      baud_rate_id = BAUD_RATE.index config_values[index]
	      raise ArgumentError, "invalid baud rate: #{config_values[index]}, valid: #{BAUD_RATE.join(', ')}" if baud_rate_id.nil?
	      config_values[index] = baud_rate_id
	    end
	    config_values.pack 'CLC4'
	  end.join
	end

	def common_set_radar_pos pos_index, state, lat, lon, alt, heading, speed, lq
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  values.pack 'C2l2S2C'
	end

	def inav_set_misc *values
	  MSP.argument_count_check values.count, 19
	  values[5] = GPS_PROVIDER.index values[5]
	  values[6] = GPS_SBAS_MODE.index values[6]
	  values[8] = (values[8] * 10).round
	  values[10] = VOLTAGE_SOURCE.index values[10]
	  [ *(12..15).to_a, 17, 18 ].each { |index| values[index] = (values[index] * 100).round }
	  values[19] = BATTERY_CAPACITY_UNIT.index values[19]
	  values.pack 'S5CxC2sSC2S4L3C'
	end

	def inav_set_battery_config *values
	  MSP.argument_count_check values.count, 13
	  values[0] = (values[0] * 100).round
	  values[1] = VOLTAGE_SOURCE.index values[1]
	  values[12] = BATTERY_CAPACITY_UNIT.index values[12]
	  (3..6).each { |index| values[index] = (values[index] * 100).round }
	  [7, 8].each { |index| values[index] = (values[index] * 10000).round }
	  values.pack 'SC2S4sSL3C'
	end

	def inav_set_rate_profile *values
	  MSP.argument_count_check values.count, 14
	  values.pack 'C3SC10'
	end

	def inav_set_mc_braking *values
	  MSP.argument_count_check values.count, 8
	  values.pack 'S3CS3C'
	end

	def inav_set_mixer yaw_motor_direction, yaw_jump_prevention_limit, platform_type, has_flaps, applied_mixer_preset
	  values = method(__method__).parameters.map { |_, name| binding.local_variable_get name }
	  values[0] = YAW_MOTOR_DIRECTION[values[0]]
	  values[2] = PLATFORM_TYPE.index values[2]
	  values[3] = values[3] ? 1 : 0
	  values.pack 'CSC2S'
	end

	def inav_osd_layouts layout_index: nil, item_index: nil
	  raise ArgumentError, 'the layout index must be speficied along with item index' if item_index and not layout_index
	  [ layout_index, *item_index ].pack 'C*' if layout_index
	end

	def inav_osd_set_layout_item layout_index, item_index, visible, x, y
	  [ layout_index, item_index, MSP.osd_item_bitmask_encode(visible, x, y) ].pack 'C2S'
	end

	def inav_osd_set_alarms *values
	  MSP.argument_count_check values.count, 13
	  values.pack 'CS7CS4'
	end

	def inav_osd_set_preferences *values
	  MSP.argument_count_check values.count, 9
	  values[0] = VIDEO_SYSTEM.index values[0]
	  values[2] = values[2] ? 1 : 0
	  values[3] = OSD_CROSSHAIR_STYLE.index values[3]
	  [4, 5].each { |index| values[index] = OSD_SIDEBAR_SCROLL.index values[index] }
	  values[6] = values[6] ? 1 : 0
	  values[7] = OSD_UNITS.index values[7]
	  values[8] = BATTERY_CAPACITY_UNIT.index values[8]
	  values.pack 'C9'
	end

	def inav_select_battery_profile profile_index
	  [ profile_index ].pack 'C'
	end

	def set_blackbox_config device_type, rate_num, rate_denom
	  [ device_type, rate_num, rate_denom ].pack 'CS2'
	end

	def inav_set_temp_sensor_config *sensors_config
	  MSP.argument_count_check sensors_config.count, 8
	  sensors_config.map do |sensor_config|
	    type_id = TEMPERATURE_SENSOR_TYPE.index sensor_config.type
	    alarm_min, alarm_max = sensor_config[2..3].map { |v| (v * 10).round }
	    raise ArgumentError, "invalid sensor type: #{sensor_config.type}, valid: #{TEMPERATURE_SENSOR_TYPE.join(', ')}" if type_id.nil?
	    [ type_id, sensor_config.address, alarm_min, alarm_max, sensor_config.osd_symbol, sensor_config.label ].pack 'CQs2CZ4'
	  end.join
	end

	def inav_set_servo_mixer rule_index, output, input, rate, speed, condition_id
	  input_id = SERVO_MIXER_INPUT.index input
	  raise ArgumentError, "invalid input: #{input}, valid: #{SERVO_MIXER_INPUT.join(', ')}" if input_id.nil?
	  [ rule_index, output, input_id, rate, speed, condition_id ].pack 'C3sCc'
	end

	def inav_set_logic_conditions *values
	  MSP.argument_count_check values.count, 7
	  values[0] = values[0] ? 1 : 0
	  values[1] = LOGIC_CONDITION_OPERATION.index values[1]
	  [3, 5].each { |index| values[index] = LOGIC_CONDITION_FLIGHT_OPERAND.index values[index] if values[index-1] == :flight }
	  [2, 4].each { |index| values[index] = LOGIC_CONDITION_OPERAND_TYPE.index values[index] }
	  values[6] = values[6].sum { |flag| LOGIC_CONDITION_FLAGS[flag] }
	  values.pack 'C4lClC'
	end

	def set_pid pids
	  pids.to_a.flat_map { |pid_section| pid_section.to_a }.pack 'C*'
	end

      end

    end # DataEncoder

  end # V2

end
