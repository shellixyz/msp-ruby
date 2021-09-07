
require 'serialport'
require 'crc'
require 'date'
require_relative 'msp_data_decoder'
require_relative 'msp_data_encoder'
require_relative 'msp_command_ids'
#require_relative 'msp_common_setting_interface'

class MSP

  V2_FRAME_ID = 255

  class ProtoError < StandardError
    class SyncFailed < ProtoError; end
    class UnrecognizedResultChar < ProtoError; end
    class ChecksumMismatch < ProtoError; end
    class ReadTimeout < ProtoError; end
    class UnknownCommandID < ProtoError; end
  end

  class SDCardNotSupported < StandardError; end
  class OSDNotSupported < StandardError; end
  class VTXNotConfigured < StandardError; end
  class BlackboxNotSupported < StandardError; end
  class CommandFailed < StandardError; end

  module Rangefinder

    class OutOfRange < StandardError; end
    class HardwareFailure < StandardError; end
    class StaleData < StandardError; end

  end

  SummedData = Struct.new :data, :cksum

  def self.osd_item_bitmask_decode bitmask
    visible = (bitmask & 0x800) != 0
    x, y = (bitmask & 0x1F), ((bitmask >> 5) & 0x1F)
    Struct.new(:visible?, :x, :y).new visible, x, y
  end

  def self.osd_item_bitmask_encode visible, x, y
    (visible ? 0x800 : 0) | x | (y << 5)
  end

  def self.argument_count_check given, expected
      raise ArgumentError, "wrong number of arguments (given #{given}, expected #{expected})", caller[1..-1] unless given == expected
  end

  def self.read_checksum serial_port
    serial_port.readbyte
  rescue EOFError
    raise ProtoError::ReadTimeout, 'failed to read checksum'
  end

  def self.sync! serial_port, str
    Timeout.timeout(serial_port.read_timeout / 1000.0) do
        buf = ''
        loop do
            rcv = serial_port.read 1
            raise Timeout::Error if rcv.nil?
            buf = buf.byteslice(1, str.bytesize - 1) if buf.bytesize == str.bytesize
            buf += rcv
            return nil if buf == str
        end
    end
  rescue Timeout::Error
      raise ProtoError::ReadTimeout, 'failed to read sync data'
  end

  module V1

    def self.cksum data, cksum = 0
      enum_method = data.is_a?(String) ? :each_byte : :each
      data.send(enum_method).reduce(cksum) { |v, cksum| cksum ^ v }
    end

    def self.cksum_packed data, cksum = 0
      [ cksum(data, cksum) ].pack 'C'
    end

    def self.pack_command cmd, *data
      cmd_no = COMMANDS.invert[cmd]
      raise ArgumentError, "unknown command: #{cmd}" if cmd_no.nil?
      #data_packed = encode cmd, *data
      data_packed = DataEncoder.respond_to?(cmd) ? DataEncoder.send(cmd, *data) || '' : ''
      header = Header.pack cmd_no, data_packed.length
      cksum = cksum_packed data_packed, header.cksum
      '$M<' + header.data + data_packed + cksum
    end

    def self.read_response serial_port
      MSP.sync! serial_port, '$M'
      result_char = serial_port.readchar
      raise ProtoError::UnrecognizedResultChar, "unrecognized result char: #{result_char}" unless %w[ > ! ].include? result_char
      header = Header.read serial_port
      #pp header
      #header = Header.new 1, 2, 3
      #binding.pry
      calc_cksum = header.cksum
      data_size = nil
      if header.size == 255
	size_data = serial_port.read(2)
	raise ProtoError::ReadTimeout, 'failed to read jumbo frame header' if size_data.nil?
	data_size = size_data.unpack1('S')
	calc_cksum = cksum size_data, calc_cksum
      else
	data_size = header.size
      end
      data = serial_port.read data_size
      raise ProtoError::ReadTimeout, 'failed to read payload' if data.nil?
      #serial_port.readbyte
      #data = serial_port.read 19
      received_cksum = MSP.read_checksum serial_port
      calc_cksum = cksum data, calc_cksum
      raise ProtoError::ChecksumMismatch, "bad checksum: received #{received_cksum.inspect}, calculated #{calc_cksum.inspect}" unless received_cksum == calc_cksum
      raise CommandFailed, 'the command returned an error' if result_char == ?!

      #header.cmd = 21

      command = COMMANDS[header.cmd]
      raise ProtoError::UnknownCommandID, "received unknown command id: #{header.cmd}" if command.nil?
      DataDecoder.send(command, data) if DataDecoder.respond_to? command
    end

    def self.command_noack serial_port, cmd, *data
      serial_port.write pack_command(cmd, *data)
    end

    def self.command serial_port, cmd, *data
      command_noack serial_port, cmd, *data
      #if @a
      #sleep 1
      #serial_port.readpartial 1000
      #else
      #@a = 1
      read_response serial_port
      #end
    end

    class Header < Struct.new :size, :cmd, :cksum

      def self.pack cmd, size
	data = [ size, cmd ].pack 'CC'
	cksum = V1.cksum data
	SummedData.new data, cksum
      end

      def self.read serial_port
	data = serial_port.read 2
	raise ProtoError::ReadTimeout, 'failed to read header' if data.nil?
	new *data.unpack('CC'), V1.cksum(data)
      end

    end

  end

  module V2

    def self.cksum data, cksum = 0
      CRC::crc8_dvb_s2 data, cksum
    end

    def self.cksum_packed data, cksum = 0
      [ cksum(data, cksum) ].pack 'C'
    end

    def self.pack_command cmd, *data
      cmd_no = COMMANDS.invert[cmd]
      raise ArgumentError, "unknown command: #{cmd}" if cmd_no.nil?
      #packed_data = encode cmd, *data
      data_packed = DataEncoder.respond_to?(cmd) ? DataEncoder.send(cmd, *data) || '' : ''
      header = Header.pack 0, cmd_no, data_packed.length
      cksum = cksum_packed data_packed, header.cksum
      '$X<' + header.data + data_packed + cksum
    end

    def self.read_response serial_port, *command_data
      MSP.sync! serial_port, '$X'
      result_char = serial_port.readchar
      raise ProtoError::UnrecognizedResultChar, "unrecognized result char: #{result_char}" unless %w[ > ! ].include? result_char

      #MSP.sync! serial_port, '$X>'
      header = Header.read serial_port
      data = serial_port.read header.size
      raise ProtoError::ReadTimeout, 'failed to read payload' if data.nil?
      cksum = MSP.read_checksum serial_port
      calc_cksum = cksum data, header.cksum
      raise ProtoError::ChecksumMismatch, "bad checksum: received #{cksum.inspect}, calculated #{calc_cksum.inspect}" unless cksum == calc_cksum
      #decode header.cmd, data
      raise CommandFailed, 'the command returned an error' if result_char == ?!
      command = COMMANDS[header.cmd]
      raise ProtoError::UnknownCommandID, "received unknown command id: #{header.cmd}" if command.nil?
      if DataDecoder.respond_to? command
	parms = [ data ]
	parms << command_data if DataDecoder.method(command).parameters.count > 1
	DataDecoder.send command, *parms
      end
    end

    def self.command_noack serial_port, cmd, *data
      serial_port.write pack_command(cmd, *data)
    end

    def self.command serial_port, cmd, *data
      command_noack serial_port, cmd, *data
      read_response serial_port, *data
      #serial_port.readpartial 1000
    end

    class Header < Struct.new :flags, :cmd, :size, :cksum

      def self.pack flags, cmd, size
	data = [ flags, cmd, size ].pack 'CSS'
	cksum = V2.cksum data
	SummedData.new data, cksum
      end

      def self.read serial_port
	data = serial_port.read 5
	raise ProtoError::ReadTimeout, 'failed to read header' if data.nil?
	new *data.unpack('CSS'), V2.cksum(data)
      end

    end

  end

  def self.command_version cmd
    if cmd.is_a? Integer
      cmd & 0x1000 == 0 ? V1 : V2
    else
      return V2 if V2::COMMANDS.has_value? cmd
      return V1 if V1::COMMANDS.has_value? cmd
      raise ArgumentError, "unknown command: #{cmd}"
    end
  end

  def flush_input
      flash_start = Time.now
      while Time.now - flash_start < 1
          serial.readbyte
      end
      nil
  rescue EOFError
      nil
  end

  def initialize port_dev_path, speed = 115200
    @port_dev_path = port_dev_path
    @port_speed = speed
    @state = :idle
    open_serial_port
    flush_input
  end

  def close
    serial.close
  end

  def command_noack cmd, *data, version: nil
    version = version ? self.class.const_get(version.upcase) : self.class.command_version(cmd)
    version.command_noack serial, cmd, *data
  end

  def command cmd, *data, version: nil
    version = version ? self.class.const_get(version.upcase) : self.class.command_version(cmd)
    version.command serial, cmd, *data
  end

  def reboot_fc resume: true
      command_noack :reboot
      close
      if resume
        sleep 5
        open_serial_port
      end
  end

  def features
      command :feature
  end

  def feature_enabled? feature
      features.include? feature
  end

  def features= features
      command :set_feature, *features
  end

  def enable_feature feature
      self.features = features | [ feature ]
  end

  def disable_feature feature
      new_features = features
      new_features.delete feature
      self.features = new_features
  end

  def save_settings
      command :eeprom_write
  end

  def decode_mode_flags mode_flags
    command(:boxnames).each.with_index.reduce([]) { |enabled, (name, index)| enabled.tap { enabled.push(name) if (mode_flags & (1 << index)) != 0 } }
  end

  def settings
      @settings_interface ||= CommonSettingInterface.new(self)
  end

  def last_setting_id
      command(:common_pg_list).map(&:end).max
  end

  def process
    loop do
      begin
	case state

	when :idle
	  serial.find ?$

	when :header_start
	  char = serial.readbyte
	  case char
	  when ?M
	    @state = :header_mspv1_start
	    @checksum1 = 0
	    @checksum2 = CRC::CRC8_DVB_S2.new
	  when ?X
	    @state = :header_mspv2_start
	    @checksum2 = 0
	  else
	    raise ProtoError, "Expected M or X, got #{char}"
	  end

	when :header_mspv1_start
	  wait_for ?<
	  header = read_v1_header
	  raise ProtoError, "size(#{header.size}) > 192" if header.size > 192
	  if header.cmd == V2_FRAME_ID
	    raise ProtoError, "v2 and payload size(#{header.size}) < 6" if header.size < 6
	    @msp_version = :v2_over_v1
	  else
	    @msp_version = :v1
	    @msp_cmd = header.cmd
	    @msp_data_size = header.size
	    @state = header.size == 0 ? :checksum_mspv1 : :payload_mspv1
	  end

	when :header_mspv2_start
	  wait_for ?<
	  header = read_v2_header

	when :payload_mspv1
	  data = serial.read @msp_data_size
	  @state = :checksum_mspv1

	when :checksum_mspv1
	  @state = :command_received

	when :command_received

	end
      rescue ProtoError
	STDERR.puts "Proto error: #{error}"
	@state = :idle
      end
    end
  end

  private

  def open_serial_port
    @serial = SerialPort.new @port_dev_path, baud: @port_speed
    serial.flow_control = SerialPort::NONE
    serial.read_timeout = 1000
    serial.flush_input
  end

  attr_reader :serial, :state

end

if $0 == __FILE__
  require 'pry'
  port = ARGV[0] or '/dev/ttyACM0'
  baud = ARGV[1] or 115200
  msp = MSP.new port, baud
  msp.pry
end

