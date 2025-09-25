# Decompiled with PyLingual (https://pylingual.io)
# Internal filename: /home/mks/klipper/klippy/extras/hx711.py
# Bytecode version: 3.9.0beta5 (3425)
# Source timestamp: 2024-11-05 10:35:17 UTC (1730802917)

from . import bus, bulk_sensor
BATCH_UPDATES = 0.1
HX711_FREQ = 64000000
BYTES_PER_SAMPLE = 4
MAX_SAMPLES_PER_BLOCK = 52 // BYTES_PER_SAMPLE
UPDATE_INTERVAL = 0.1
MAX_CHIPS = 1
MAX_BULK_MSG_SIZE = 4

class HX711Command:

    def __init__(self, config, chip):
        self.chip = chip
        self.config = config
        self.printer = config.get_printer()
        self.register_commands()
        self.last_val = 0

    def register_commands(self):
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('WEIGHTING_DEBUG_QUERY', self.cmd_WEIGHTING_DEBUG_QUERY, desc=self.cmd_WEIGHTING_DEBUG_QUERY_help)
        gcode.register_command('WEIGHTING_START_QUERY', self.cmd_WEIGHTING_START_QUERY, desc=self.cmd_WEIGHTING_DEBUG_QUERY_help)
        gcode.register_command('WEIGHTING_END_QUERY', self.cmd_WEIGHTING_END_QUERY, desc=self.cmd_WEIGHTING_END_QUERY_help)
        gcode.register_command('WEIGHT_TARGET', self.cmd_WEIGHT_TARGET, desc=self.cmd_WEIGHTING_DEBUG_QUERY_help)

    def cmd_WEIGHT_TARGET(self, gcmd):
        cb = self.chip.query_hx711_status.send([self.chip.oid])
        status = cb['status']
        gcmd.respond_info('HX711_status:%d' % status)

    def cmd_WEIGHTING_DEBUG_QUERY(self, gcmd):
        cb = self.chip.query_hx711_read_data.send([self.chip.oid, 0, 0])
        Lv_Bo = 0.02
        d = cb['data']
        hx711_data = d[0] & 255 | (d[1] & 255) << 8 | (d[2] & 255) << 16 | (d[3] & 255) << 24
        hx711_data_out = hx711_data
        if hx711_data_out < 100:
            gcmd.respond_info('HX711_Info:Data error!')
        else:
            weight_mg = hx711_data_out * 0.01671875 / 8388607
            gcmd.respond_info('HX711_Info: %d/0x%x(origin) / %.6f(ENOB)' % (hx711_data_out, hx711_data_out, weight_mg))
    cmd_WEIGHTING_DEBUG_QUERY_help = 'Obtain HX711 measurement values'

    def cmd_WEIGHTING_START_QUERY(self, gcmd):
        self.chip.start_hx711(1)
        gcmd.respond_info('hx711 test start!')

    def cmd_WEIGHTING_END_QUERY(self, gcmd):
        self.chip.start_hx711(0)
        gcmd.respond_info('hx711 test end!')
    cmd_WEIGHTING_END_QUERY_help = 'HX711 end query!'

class HX711:

    def __init__(self, config, calibration=None):
        self.printer = config.get_printer()
        self.config = config
        HX711Command(config, self)
        self.name = config.get_name().split()[-1]
        self.calibration = calibration
        dout_pin = config.get('dout_pin')
        sclk_pin = config.get('sclk_pin')
        self.voltage = config.getfloat('voltage', 4.95, minval=4.93)
        self.delta_v = config.getfloat('delta_v', 0.1, minval=0.0001)
        self.adc_voltage_diff = int(self.delta_v * 16777215 / self.voltage)
        ppins = self.printer.lookup_object('pins')
        self.dout_params = ppins.lookup_pin(dout_pin)
        self.sck_params = ppins.lookup_pin(sclk_pin)
        self.mcu = self.dout_params['chip']
        self.oid = self.mcu.create_oid()
        self.query_hx711_status = None
        self.query_hx711_cmd = None
        self.hx711_setup_home_cmd = None
        self.query_hx711_home_state_cmd = None
        self.mcu.register_config_callback(self._build_config)
        self.bytes_per_block = BYTES_PER_SAMPLE * 1
        self.blocks_per_msg = bulk_sensor.MAX_BULK_MSG_SIZE // self.bytes_per_block
        self.bulk_queue = bulk_sensor.BulkDataQueue(self.mcu, 'hx711_data', self.oid)
        self.mcu.add_config_cmd('query_hx711 oid=%d rest_ticks=0' % self.oid, on_restart=True)

    def _build_config(self):
        cmdqueue = self.mcu.alloc_command_queue()
        self.mcu.add_config_cmd('config_hx711 oid=%d dout_pin=%s sclk_pin=%s' % (self.oid, self.dout_params['pin'], self.sck_params['pin']))
        self.query_hx711_cmd = self.mcu.lookup_command('query_hx711 oid=%c rest_ticks=%u', cq=cmdqueue)
        self.hx711_setup_home_cmd = self.mcu.lookup_command('hx711_setup_home oid=%c clock=%u threshold=%u trsync_oid=%c trigger_reason=%c error_reason=%c', cq=cmdqueue)
        self.query_hx711_home_state_cmd = self.mcu.lookup_query_command('query_hx711_home_state oid=%c', 'hx711_home_state oid=%c homing=%c trigger_clock=%u', oid=self.oid, cq=cmdqueue)
        self.query_hx711_read_data = self.mcu.lookup_query_command('query_hx711_read oid=%c', 'query_hx711_data oid=%c data=%*s', oid=self.oid, cq=cmdqueue)
        self.query_hx711_update_cmd = self.mcu.lookup_query_command('query_hx711_zero oid=%c', 'query_hx711_zero_read oid=%c', oid=self.oid, cq=cmdqueue)

    def get_mcu(self):
        return self.mcu

    def setup_home(self, print_time, trsync_oid, hit_reason, err_reason, rest_time):
        clock = self.mcu.print_time_to_clock(print_time)
        rest_ticks = self.mcu.print_time_to_clock(print_time + rest_time) - clock
        self.query_hx711_cmd.send([self.oid, rest_ticks])
        self.hx711_setup_home_cmd.send([self.oid, clock, self.adc_voltage_diff, trsync_oid, hit_reason, err_reason])

    def clear_home(self):
        self.hx711_setup_home_cmd.send([self.oid, 0, 0, 0, 0, 0])
        if self.mcu.is_fileoutput():
            return 0.0
        params = self.query_hx711_home_state_cmd.send([self.oid])
        tclock = self.mcu.clock32_to_clock64(params['trigger_clock'])
        self.query_hx711_cmd.send([self.oid, 0])
        return self.mcu.clock_to_print_time(tclock)

    def zero_home(self):
        self.query_hx711_update_cmd.send([self.oid])