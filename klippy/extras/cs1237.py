# Decompiled with PyLingual (https://pylingual.io)
# Internal filename: /home/mks/klipper/klippy/extras/cs1237.py
# Bytecode version: 3.9.0beta5 (3425)
# Source timestamp: 2024-11-05 10:35:17 UTC (1730802917)

import logging
from . import bus, bulk_sensor
BATCH_UPDATES = 0.1
BYTES_PER_SAMPLE = 4
MAX_SAMPLES_PER_BLOCK = 52 // BYTES_PER_SAMPLE
UPDATE_INTERVAL = 0.1
MAX_CHIPS = 1
MAX_BULK_MSG_SIZE = 4
USE_SPEED = 1280
CS1237_QUERY_RATES = {40: 28, 640: 44, 1280: 60}

class CS1237Command:

    def __init__(self, config, chip):
        self.chip = chip
        self.config = config
        self.printer = config.get_printer()
        self.register_commands()
        self.last_val = 0

    def register_commands(self):
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('WEIGHTING_DEBUG_QUERY', self.cmd_WEIGHTING_DEBUG_QUERY, desc=self.cmd_WEIGHTING_DEBUG_QUERY_help)
        gcode.register_command('CS_WEIGHT_BEGIN', self.cmd_CS1237_WEIGHT_BEGIN, desc=self.cmd_WEIGHTING_DEBUG_QUERY_help)

    def cmd_WEIGHTING_DEBUG_QUERY(self, gcmd):
        cb = self.chip.query_cs1237_end_cmd.send([self.chip.oid, 0, 0])
        d = cb['data']
        adc_bit = 8388607
        Vref = 5.0
        GAIN = 128
        cs1237_data = d[0] & 255 | (d[1] & 255) << 8 | (d[2] & 255) << 16 | (d[3] & 255) << 24
        if cs1237_data & 8388608:
            zhengfu = 1
            read_data = ~cs1237_data
            read_data = -(read_data + 1 & 16777215)
        else:
            zhengfu = 0
            read_data = cs1237_data
            read_data = cs1237_data & 268435455
        cs1237_v_128 = read_data * (0.5 * Vref / GAIN) / adc_bit
        if zhengfu:
            gcmd.respond_info('传感器数据(-): %d / %d / %.6f(mV)' % (cs1237_data, read_data, cs1237_v_128 * 1000))
        else:
            gcmd.respond_info('传感器数据(+): %d / %d / %.6f(mV)' % (cs1237_data, read_data, cs1237_v_128 * 1000))
    cmd_WEIGHTING_DEBUG_QUERY_help = 'Obtain CS1237 measurement values'

    def cmd_CS1237_WEIGHT_BEGIN(self, gcmd):
        cb = self.chip.query_cs1237_begin_cmd.send([self.chip.oid, CS1237_QUERY_RATES[USE_SPEED]])
        config = cb['config']
        if config != CS1237_QUERY_RATES[USE_SPEED]:
            cb = self.chip.query_cs1237_begin_cmd.send([self.chip.oid, CS1237_QUERY_RATES[USE_SPEED]])
            config = cb['config']
        if config != CS1237_QUERY_RATES[USE_SPEED]:
            gcmd.respond_info('传感器配置失败: 0x%x' % config)
        else:
            gcmd.respond_info('传感器配置成功: 0x%x' % config)

class CS1237:

    def __init__(self, config, calibration=None):
        self.printer = config.get_printer()
        self.config = config
        CS1237Command(config, self)
        self.name = config.get_name().split()[-1]
        self.calibration = calibration
        dout_pin = config.get('dout_pin')
        sclk_pin = config.get('sclk_pin')
        self.voltage = config.getfloat('voltage', 4.95, minval=4.93)
        self.delta_v = config.getfloat('delta_v', 0.03, minval=0.0001)
        self.adc_voltage_diff = int(self.delta_v * 16777215 / self.voltage)
        ppins = self.printer.lookup_object('pins')
        self.dout_params = ppins.lookup_pin(dout_pin)
        self.sck_params = ppins.lookup_pin(sclk_pin)
        self.mcu = self.dout_params['chip']
        self.oid = self.mcu.create_oid()
        self.query_cs1237_status = None
        self.query_cs1237_cmd = None
        self.cs1237_setup_home_cmd = self.query_cs1237_home_state_cmd = None
        self.bytes_per_block = BYTES_PER_SAMPLE * 1
        self.blocks_per_msg = bulk_sensor.MAX_BULK_MSG_SIZE // self.bytes_per_block
        self.bulk_queue = bulk_sensor.BulkDataQueue(self.mcu, 'cs1237_data', self.oid)
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        cmdqueue = self.mcu.alloc_command_queue()
        self.mcu.add_config_cmd('config_cs1237 oid=%d dout_pin=%s sclk_pin=%s' % (self.oid, self.dout_params['pin'], self.sck_params['pin']))
        self.query_cs1237_cmd = self.mcu.lookup_command('query_cs1237 oid=%c rest_ticks=%u', cq=cmdqueue)
        self.cs1237_setup_home_cmd = self.mcu.lookup_command('cs1237_setup_home oid=%c clock=%u threshold=%u trsync_oid=%c trigger_reason=%c error_reason=%c', cq=cmdqueue)
        self.query_cs1237_home_state_cmd = self.mcu.lookup_query_command('query_cs1237_home_state oid=%c', 'cs1237_home_state oid=%c homing=%c trigger_clock=%u', oid=self.oid, cq=cmdqueue)
        self.query_cs1237_end_cmd = self.mcu.lookup_query_command('query_cs1237_read oid=%c reg=%u read_len=%u', 'query_cs1237_data oid=%c data=%*s', self.oid)
        self.query_cs1237_begin_cmd = self.mcu.lookup_query_command('query_cs1237_begin oid=%c config=%u', 'query_cs1237_begin_read oid=%c config=%u', self.oid)
        self.query_cs1237_update_cmd = self.mcu.lookup_query_command('query_cs1237_zero oid=%c', 'query_cs1237_zero_read oid=%c', self.oid)
        self.query_cs1237_config_read_cmd = self.mcu.lookup_query_command('query_cs1237_config_r oid=%c', 'query_cs1237_zero_config_read oid=%c config=%u', self.oid)
        self.query_cs1237_zero_read_cmd = self.mcu.lookup_query_command('query_cs1237_zero_read_only oid=%c', 'query_cs1237_zero_read_o oid=%c data=%*s', self.oid)

    def get_mcu(self):
        return self.mcu

    def setup_home(self, print_time, trsync_oid, hit_reason, err_reason, rest_time):
        clock = self.mcu.print_time_to_clock(print_time)
        rest_ticks = self.mcu.print_time_to_clock(print_time + rest_time) - clock
        self.query_cs1237_cmd.send([self.oid, rest_ticks])
        self.cs1237_setup_home_cmd.send([self.oid, clock, self.adc_voltage_diff, trsync_oid, hit_reason, err_reason])

    def clear_home(self):
        self.cs1237_setup_home_cmd.send([self.oid, 0, 0, 0, 0, 0])
        if self.mcu.is_fileoutput():
            return 0.0
        params = self.query_cs1237_home_state_cmd.send([self.oid])
        tclock = self.mcu.clock32_to_clock64(params['trigger_clock'])
        self.query_cs1237_cmd.send([self.oid, 0])
        return self.mcu.clock_to_print_time(tclock)

    def zero_home(self):
        cb = self.query_cs1237_config_read_cmd.send([self.oid])
        config = cb['config']
        if config != CS1237_QUERY_RATES[USE_SPEED]:
            logging.info('WEIGHT:CS1237-RESTART')
            cb = self.query_cs1237_begin_cmd.send([self.oid, CS1237_QUERY_RATES[USE_SPEED]])
        if config != CS1237_QUERY_RATES[USE_SPEED]:
            logging.info('WEIGHT:传感器IC故障，检查转接板是否正常: 0x%x' % config)
        self.query_cs1237_update_cmd.send([self.oid])
        self.check_cs1237_zero()

    def check_cs1237_zero(self):
        cb = self.query_cs1237_config_read_cmd.send([self.oid])
        config = cb['config']
        if config != CS1237_QUERY_RATES[USE_SPEED]:
            logging.info('WEIGHT:CS1237-RESTART')
            cb = self.query_cs1237_begin_cmd.send([self.oid, CS1237_QUERY_RATES[USE_SPEED]])
        if config != CS1237_QUERY_RATES[USE_SPEED]:
            logging.info('WEIGHT:传感器IC故障，检查转接板是否正常: 0x%x' % config)
        bk = self.query_cs1237_zero_read_cmd.send([self.oid])
        d = bk['data']
        cs1237_zero = d[0] & 255 | (d[1] & 255) << 8 | (d[2] & 255) << 16 | (d[3] & 255) << 24
        cb1 = self.query_cs1237_end_cmd.send([self.oid, 0, 0])
        b = cb1['data']
        cs1237_data = b[0] & 255 | (b[1] & 255) << 8 | (b[2] & 255) << 16 | (b[3] & 255) << 24
        res = 0
        if cs1237_data > cs1237_zero:
            res = cs1237_data - cs1237_zero
        else:
            res = cs1237_zero - cs1237_data
        if res > 0.05:
            self.query_cs1237_update_cmd.send([self.oid])
            logging.info('WEIGHT:需要清零')