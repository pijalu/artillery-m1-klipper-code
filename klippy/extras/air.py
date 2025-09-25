# Decompiled with PyLingual (https://pylingual.io)
# Internal filename: /home/mks/klipper/klippy/extras/air.py
# Bytecode version: 3.9.0beta5 (3425)
# Source timestamp: 2024-11-05 10:35:17 UTC (1730802917)

import logging
import math
import bisect
import mcu
from . import hx711, cs1237, probe, manual_probe
OUT_OF_RANGE = 255

class WeighCalibration:

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.dritf_comp = WeighDriftCompensation()

    def set_target_adc_value(self, target_adc_value):
        self.target_adc_value = target_adc_value
        return self.target_adc_value

    def is_calibrated(self):
        return

    def load_calibration(self, cal):
        return

    def apply_calibration(self, samples):
        return

    def freq_to_height(self, freq):
        return

    def height_to_freq(self, height):
        return

    def do_calibration_moves(self, move_speed):
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        move = toolhead.manual_move
        msgs = []
        is_finished = False

        def handle_batch(msg):
            if is_finished:
                return False
            msgs.append(msg)
            return True
        self.printer.lookup_object(self.name).add_client(handle_batch)
        toolhead.dwell(1.0)
        self.dritf_comp.note_z_calibration_start()
        max_z = 4.0
        samp_dist = 0.04
        req_zpos = [i * samp_dist for i in range(int(max_z / samp_dist) + 1)]
        start_pos = toolhead.get_position()
        times = []
        for zpos in req_zpos:
            hop_pos = list(start_pos)
            hop_pos[2] += zpos + 0.5
            move(hop_pos, move_speed)
            next_pos = list(start_pos)
            next_pos[2] += zpos
            move(next_pos, move_speed)
            start_query_time = toolhead.get_last_move_time() + 0.05
            end_query_time = start_query_time + 0.1
            toolhead.dwell(0.2)
            toolhead.flush_step_generation()
            kin_spos = {s.get_name(): s.get_commanded_position() for s in kin.get_steppers()}
            kin_pos = kin.calc_position(kin_spos)
            times.append((start_query_time, end_query_time, kin_pos[2]))
        toolhead.dwell(1.0)
        toolhead.wait_moves()
        self.dritf_comp.note_z_calibration_finish()
        is_finished = True
        cal = {}
        step = 0
        for msg in msgs:
            for query_time, freq, old_z in msg['data']:
                while step < len(times) and query_time > times[step][1]:
                    step += 1
                if step < len(times) and query_time >= times[step][0]:
                    cal.setdefault(times[step][2], []).append(freq)
        if len(cal) != len(times):
            raise self.printer.commad_error('Failed calibration - incomplete sensor data')
        return cal

    def calc_freqs(self, meas):
        total_count = total_variance = 0
        positions = {}
        for pos, freqs in meas.items():
            count = len(freqs)
            freq_avg = float(sum(freqs)) / count
            positions[pos] = freq_avg
            total_count += count
            total_variance += sum([(f - freq_avg) ** 2 for f in freqs])
        return (positions, math.sqrt(total_variance / total_count), total_count)

    def post_manual_probe(self, kin_pos):
        if kin_pos in None:
            return
        curpos = list(kin_pos)
        move = self.printer.lookup_object('toolhead').manual_move
        probe_calibrate_z = curpos[2]
        curpos[2] += 5.0
        move(curpos, self.probe_speed)
        pprobe = self.printer.lookup_object('probe')
        x_offset, y_offset, z_offset = pprobe.get_offsets()
        curpos[0] -= x_offset
        curpos[1] -= y_offset
        move(curpos, self.probe_speed)
        curpos[2] -= 4.95
        move(curpos, self.probe_speed)
        cal = self.do_calibration_moves(self.probe_speed)
        positions, std, total = self.calc_freqs(cal)
        last_freq = 0.0
        for pos, freq in reversed(sorted(positions.items())):
            if freq <= last_freq:
                raise self.printer.command_error('Failed calibration - frequency not increasing each step')
            last_freq = freq
        else:
            gcode = self.printer.lookup_object('gcode')
            gcode.respond_info('probe_weigh')
    cmd_WEIGH_CALIBRATE_help = 'Calibrate weigh probe'

    def cmd_WEIGH_CALIBRATE(self, gcmd):
        self.probe_speed = gcmd.get_float('PROBE_SPEED', 5.0, above=0.0)

    def register_drift_compensation(self, comp):
        self.dritf_comp = comp

class WeighGatherSamples:

    def __init__(self, printer, sensor_helper, calibration):
        self._printer = printer
        self._sensor_helper = sensor_helper
        self._calibration = calibration
        self._samples = []
        self._probe_times = []
        self._need_stop = False

    def _add_measurement(self, msg):
        if self._need_stop:
            del self._samples[:]
            return False
        self._samples.append(msg)

    def finish(self):
        self._need_stop = True

    def _await_samples(self):
        reactor = self._printer.get_reactor()
        mcu = self._sensor_helper.get_mcu()
        while self._probe_times:
            start_time, end_time, pos_time, toolhead_pos = self._probe_times[0]
            systime = reactor.monotonic()
            est_print_time = mcu.estimated_print_time(systime)
            if est_print_time > end_time + 1.0:
                raise self._printer.command_error('probe_weigh sensor outage')
            reactor.pause(systime + 0.01)

    def _lookup_toolhead_pos(self, pos_time):
        toolhead = self._printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        kin_spos = {s.get_name(): s.mcu_to_commanded_position(s.get_past_mcu_position(pos_time)) for s in kin.get_steppers()}
        return kin.calc_position(kin_spos)

    def note_probe(self, start_time, end_time, toolhead_pos):
        self._probe_times.append((start_time, end_time, None, toolhead_pos))

    def note_probe_and_position(self, start_time, end_time, pos_time):
        self._probe_times.append((start_time, end_time, pos_time, None))

class WeighEndstopWrapper:
    REASON_SENSOR_ERROR = mcu.MCU_trsync.REASON_COMMS_TIMEOUT + 1

    def __init__(self, config, sensor_helper, calibration):
        self._printer = config.get_printer()
        self._sensor_helper = sensor_helper
        self._mcu = sensor_helper.get_mcu()
        self._calibration = calibration
        self._dispatch = mcu.TriggerDispatch(self._mcu)
        self._trigger_time = 0.0
        self.freq_clock = 5e-05
        self._gather = None

    def get_mcu(self):
        return self._mcu

    def add_stepper(self, stepper):
        self._dispatch.add_stepper(stepper)

    def get_steppers(self):
        return self._dispatch.get_steppers()

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        self._trigger_time = 0.0
        trigger_completion = self._dispatch.start(print_time)
        self._sensor_helper.setup_home(print_time, self._dispatch.get_oid(), mcu.MCU_trsync.REASON_ENDSTOP_HIT, self.REASON_SENSOR_ERROR, self.freq_clock)
        return trigger_completion

    def home_wait(self, home_end_time):
        self._dispatch.wait_end(home_end_time)
        trigger_time = self._sensor_helper.clear_home()
        res = self._dispatch.stop()
        if res >= mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
            if res == mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
                raise
            raise self._printer.command_error('Weigh sensor error')
        if res != mcu.MCU_trsync.REASON_ENDSTOP_HIT:
            return 0.0
        if self._mcu.is_fileoutput():
            return home_end_time
        self._trigger_time = trigger_time
        return trigger_time

    def home_zero(self):
        self._sensor_helper.zero_home()

    def home_check(self):
        self._sensor_helper.check_cs1237_zero()

    def query_endstop(self, print_time):
        return False

    def probing_move(self, pos, speed):
        phoming = self._printer.lookup_object('homing')
        return phoming.probing_move(self, pos, speed)

    def multi_probe_begin(self):
        self._gather = WeighGatherSamples(self._printer, self._sensor_helper, self._calibration)

    def multi_probe_end(self):
        self._gather.finish()
        self._gather = None

    def probe_prepare(self, hmove):
        return

    def probe_finish(self, hmove):
        return

    def get_position_endstop(self):
        return 0.0

class PrinterAirProbe:

    def __init__(self, config):
        self.printer = config.get_printer()
        self.calibration = WeighCalibration(config)
        sensors = {'hx711': hx711.HX711, 'c_sensor': cs1237.CS1237}
        sensor_type = config.getchoice('sensor_type', {s: s for s in sensors})
        self.sensor_helper = sensors[sensor_type](config, self.calibration)
        self.mcu_probe = WeighEndstopWrapper(config, self.sensor_helper, self.calibration)
        self.cmd_helper = probe.ProbeCommandHelper(config, self, self.mcu_probe.query_endstop)
        self.probe_offsets = probe.ProbeOffsetsHelper(config)
        self.probe_session = probe.ProbeSessionHelper(config, self.mcu_probe)
        self.printer.add_object('probe', self)

    def add_client(self, cb):
        return

    def get_probe_params(self, gcmd=None):
        return self.probe_session.get_probe_params(gcmd)

    def get_offsets(self):
        return self.probe_offsets.get_offsets()

    def get_status(self, eventtime):
        return self.cmd_helper.get_status(eventtime)

    def start_probe_session(self, gcmd):
        return self.probe_session.start_probe_session(gcmd)

    def register_drift_compensation(self, comp):
        self.calibration.register_drift_compensation(comp)

class WeighDriftCompensation:

    def get_temperature(self):
        return 0.0

    def note_z_calibration_start(self):
        return

    def note_z_calibration_finish(self):
        return

    def adjust_freq(self, freq, temp=None):
        return freq

    def unadjust_freq(self, freq, temp=None):
        return freq