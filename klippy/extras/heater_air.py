# Support for a heated air
#
# Copyright (C) 2024  Makerbase@Wangchong
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import os, logging, threading

#####################################
#  QD使用的风冷器
#  1、配置pwm控制引脚, 配置使能引脚
#  2、引入当前寄出头的
#  3、增加指令宏
#####################################

AIR_KELVIN_TO_CELSIUS = -273.15
AIR_MAX_HEAT_TIME = 5.0
AIR_AMBIENT_TEMP = 25.
AIR_PID_PARAM_BASE = 255.

class Heater_air:
    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.short_name = short_name = self.name.split()[-1]

        # 配置传感器
        self.sensor = sensor
        self.min_temp = config.getfloat('min_temp', minval=AIR_KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        self.pwm_delay = self.sensor.get_report_time_delta()

        self.mini_air_temp = 5      # 配置触发制冷的温度
        self.max_power = 1.
        self.smooth_time = 1

        self.lock = threading.Lock()
        self.last_temp = self.smoothed_temp = self.target_temp = 0.
        self.last_temp_time = 0.

        # pwm caching
        self.next_pwm_time = 0.
        self.last_pwm_value = 0.

        # Setup control algorithm sub-class
        self.control = AIR_ControlBangBang(self, config)

        # Setup output air pin and en pin
        air_pin = config.get('air_pin')
        out_en_pin = config.get("en_pin")
        ppins = self.printer.lookup_object('pins')
        self.mcu_pwm = ppins.setup_pin('pwm', air_pin)
        pwm_cycle_time = config.getfloat('pwm_cycle_time', 0.100, above=0.,
                                         maxval=self.pwm_delay)
        self.mcu_pwm.setup_cycle_time(pwm_cycle_time)
        self.mcu_pwm.setup_max_duration(AIR_MAX_HEAT_TIME)

        # self.en_pin = ppins.setup_pin('digital_out', out_en_pin)
        self.en_pin = ppins.setup_pin('static_out', out_en_pin)


        # gcode = self.printer.lookup_object("gcode")
        # gcode.register_mux_command("SET_HEATER_TEMPERATURE", "HEATER",
        #                            short_name, self.cmd_SET_HEATER_TEMPERATURE,
        #                            desc=self.cmd_SET_HEATER_TEMPERATURE_help)

    def set_en(self, print_time, value, is_resend=False):
        # if not is_resend:
        #     return
        logging.info("MKS_DEBUG_AIR:set en val:%s" % value)
        # self.en_pin.set_digital(print_time, value)
        self.en_pin.set_static_digital(print_time, value)


    def set_pwm(self, read_time, value):
        if self.target_temp <= 0. or self.is_shutdown:
            value = 0.
        if ((read_time < self.next_pwm_time or not self.last_pwm_value)
            and abs(value - self.last_pwm_value) < 0.05):
            # No significant change in value - can suppress update
            return
        pwm_time = read_time + self.pwm_delay
        self.next_pwm_time = pwm_time + 0.75 * AIR_MAX_HEAT_TIME
        self.last_pwm_value = value
        self.mcu_pwm.set_pwm(pwm_time, value)
    def temperature_callback(self, read_time, temp):
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            self.control.temperature_update(read_time, temp, self.target_temp)
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_temp += temp_diff * adj_time
            self.can_extrude = (self.smoothed_temp >= self.min_extrude_temp)
    def _handle_shutdown(self):
        pass
        # self.is_shutdown = True
    def get_name(self):
        return self.name
    def get_pwm_delay(self):
        return self.pwm_delay
    def get_max_power(self):
        return self.max_power
    def get_smooth_time(self):
        return self.smooth_time
    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp))
        with self.lock:
            self.target_temp = degrees
    def get_temp(self, eventtime):
        print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime) - 5.
        with self.lock:
            if self.last_temp_time < print_time:
                return 0., self.target_temp
            return self.smoothed_temp, self.target_temp
    def check_busy(self, eventtime):
        with self.lock:
            return self.control.check_busy(
                eventtime, self.smoothed_temp, self.target_temp)
    def set_control(self, control):
        with self.lock:
            old_control = self.control
            self.control = control
            self.target_temp = 0.
        return old_control
    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.target_temp = target_temp
    def stats(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            last_temp = self.last_temp
            last_pwm_value = self.last_pwm_value
        is_active = target_temp or last_temp > 50.
        return is_active, '%s: target=%.0f temp=%.1f pwm=%.3f' % ( 
            self.short_name, target_temp, last_temp, last_pwm_value)
    def get_status(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            smoothed_temp = self.smoothed_temp
            last_pwm_value = self.last_pwm_value
        return {'temperature': round(smoothed_temp, 2), 'target': target_temp,
                'power': last_pwm_value}
    cmd_SET_AIR_TEMPERATURE_help = "Sets a air temperature"
    def cmd_SET_AIR_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float('TARGET', 0.)
        pheaters = self.printer.lookup_object('heater_air')
        pheaters.set_temperature(self, temp)

######################################################################
# Bang-bang control algo
######################################################################
class AIR_ControlBangBang:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.max_delta = 2.0 # config.getfloat('max_delta', 2.0, above=0.)
        self.heating = False
    def temperature_update(self, read_time, temp, target_temp):
        if self.heating and temp >= target_temp+self.max_delta:
            self.heating = True
            # self.heating = False
        elif not self.heating and temp <= target_temp-self.max_delta:
            self.heating = False
            # self.heating = True
        if self.heating:
            self.heater.set_pwm(read_time, self.heater_max_power)
        else:
            self.heater.set_pwm(read_time, 0.)
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return smoothed_temp < target_temp-self.max_delta

class PrinterAirHeater:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.sensor_factories = {}
        self.heaters = {}
        self.gcode_id_to_sensor = {}
        self.available_heaters = []
        self.available_sensors = []
        self.available_monitors = []    
        self.has_started = self.have_load_sensors = False   
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("gcode:request_restart",
                                            self.turn_off_all_heaters)
    def load_config(self, config):
        self.have_load_sensors = True
        # Load default temperature sensors
        pconfig = self.printer.lookup_object('configfile')
        dir_name = os.path.dirname(__file__)
        filename = os.path.join(dir_name, 'temperature_sensors.cfg')
        try:
            dconfig = pconfig.read_config(filename)
        except Exception:
            raise config.config_error("Cannot load config '%s'" % (filename,))
        for c in dconfig.get_prefix_sections(''):
            self.printer.load_object(dconfig, c.get_name())
    def add_sensor_factory(self, sensor_type, sensor_factory):
        self.sensor_factories[sensor_type] = sensor_factory
    def setup_heater(self, config, gcode_id=None):
        heater_name = config.get_name().split()[-1]
        if heater_name in self.heaters:
            raise config.error("Heater %s already registered" % (heater_name,))
        # Setup sensor
        sensor = self.setup_sensor(config)
        # Create heater
        self.heaters[heater_name] = heater = Heater_air(config, sensor)
        self.register_sensor(config, heater, gcode_id)
        self.available_heaters.append(config.get_name())
        return heater
    def get_all_heaters(self):
        return self.available_heaters
    def lookup_heater(self, heater_name):
        if heater_name not in self.heaters:
            raise self.printer.config_error(
                "Unknown heater '%s'" % (heater_name,))
        return self.heaters[heater_name]
    def setup_sensor(self, config):
        if not self.have_load_sensors:
            self.load_config(config)
        sensor_type = config.get('sensor_type')
        if sensor_type not in self.sensor_factories:
            raise self.printer.config_error(
                "Unknown temperature sensor '%s'" % (sensor_type,))
        return self.sensor_factories[sensor_type](config)
    def register_sensor(self, config, psensor, gcode_id=None):
        self.available_sensors.append(config.get_name())
        if gcode_id is None:
            gcode_id = config.get('gcode_id', None)
            if gcode_id is None:
                return
        if gcode_id in self.gcode_id_to_sensor:
            raise self.printer.config_error(
                "G-Code sensor id %s already registered" % (gcode_id,))
        self.gcode_id_to_sensor[gcode_id] = psensor
    def register_monitor(self, config):
        self.available_monitors.append(config.get_name())
    def get_status(self, eventtime):
        return {'available_heaters': self.available_heaters,
                'available_sensors': self.available_sensors,
                'available_monitors': self.available_monitors}
    def turn_off_all_heaters(self, print_time=0.):
        for heater in self.heaters.values():
            heater.set_temp(0.)
    cmd_TURN_OFF_HEATERS_help = "Turn off all heaters"
    def cmd_TURN_OFF_HEATERS(self, gcmd):
        self.turn_off_all_heaters()
    # G-Code M105 temperature reporting
    def _handle_ready(self):
        self.has_started = True
    def _get_temp(self, eventtime):
        # Tn:XXX /YYY B:XXX /YYY
        out = []
        if self.has_started:
            for gcode_id, sensor in sorted(self.gcode_id_to_sensor.items()):
                cur, target = sensor.get_temp(eventtime)
                out.append("%s:%.1f /%.1f" % (gcode_id, cur, target))
        if not out:
            return "T:0"
        return " ".join(out)
    # def cmd_M105(self, gcmd):
    #     # Get Extruder Temperature
    #     reactor = self.printer.get_reactor()
    #     msg = self._get_temp(reactor.monotonic())
    #     did_ack = gcmd.ack(msg)
    #     if not did_ack:
    #         gcmd.respond_raw(msg)
    def _wait_for_temperature(self, heater):
        # Helper to wait on heater.check_busy() and report M105 temperatures
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        toolhead = self.printer.lookup_object("toolhead")
        gcode = self.printer.lookup_object("gcode")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        while not self.printer.is_shutdown() and heater.check_busy(eventtime):
            print_time = toolhead.get_last_move_time()
            gcode.respond_raw(self._get_temp(eventtime))
            eventtime = reactor.pause(eventtime + 1.)
    def set_temperature(self, heater, temp, wait=False):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt: None))
        heater.set_temp(temp)
        if wait and temp:
            self._wait_for_temperature(heater)
    def set_air_output(self, heater, value):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(0.100)
        printer_time = toolhead.get_last_move_time()
        toolhead.register_lookahead_callback(
            lambda printer_time: heater.set_en(printer_time, value))

def load_config(config):
    return PrinterAirHeater(config)