# coding=utf-8
# Support for Honeywell ABP series digital pressure sensors with I2C interface
#
# Copyright (C) 2021  Simon Kühling <mail@simonkuehling.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus

# TODO
# - documentation
# - ABP error handling, maybe? ('status' in _sample_pressure())

HONEYWELL_ABP_STATUS = {
    0: 'STATUS_NOERROR',
    1: 'STATUS_COMMANDMODE',
    2: 'STATUS_STALEDATA',
    3: 'STATUS_DIAGNOSTIC'
}
REPORT_TIME = .3
MAX_RETRIES = 3

class HoneywellABP:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.sensor_name = config.get_name().split()[-1]
        self.i2c = bus.MCU_I2C_from_config(config, default_addr=0x28)
        self.p_min = config.getfloat('p_min')
        self.p_max = config.getfloat('p_max')
        units = {'mbar': 'mbar',
                 'bar': 'bar',
                 'kPa': 'kPa',
                 'MPa': 'MPa',
                 'psi': 'psi'}
        self.unit = config.getchoice('unit', units)
        self.last_pressure = 0.
        self._output_min = 0x0666; # 10% of 2^14
        self._output_max = 0x399A; # 90% of 2^14
        self.timer_handler = None
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('QUERY_ABP_PRESSURE', 'ID',
                                   self.sensor_name,
                                   self.cmd_QUERY_ABP_PRESSURE,
                                   desc=self.cmd_QUERY_ABP_PRESSURE_help)
        # DEBUG
        logging.info("INITIALIZED HONEYWELL ABP SENSOR '%s'" % self.sensor_name)

    def _handle_ready(self):
        waketime = self.reactor.monotonic() + REPORT_TIME
        self.timer_handler = self.reactor.register_timer(
            self._timer_event, waketime)

    def _timer_event(self, eventtime):
        if not self.printer.is_shutdown():
            for attempt in range(MAX_RETRIES):
                try:
                    self._sample_pressure()
                except self.printer.command_error as e:
                    pass
                    #logging.info("honeywell_abp pressure sampling failed on sensor '%s': %s" % (self.sensor_name, str(e)))
                else:
                    break
            else:
                # we failed all the attempts - deal with the consequences.
                logging.info("honeywell_abp pressure sampling failed on sensor '%s' %d retries in a row." % (self.sensor_name, MAX_RETRIES))
        nextwake = self.reactor.monotonic() + REPORT_TIME
        return nextwake

    def _sample_pressure(self):
        params = self.i2c.i2c_read([], 2)
        response = bytearray(params['response'])
        status = HONEYWELL_ABP_STATUS[response[0] >> 6]
        bridge_data = (response[0] << 8 | response[1]) & 0x3FFF
        self.last_pressure = self._convert_to_bar(self._raw_to_pressure(bridge_data))

    def _raw_to_pressure(self, output):
        return float(min(self._output_max, max(self._output_min, output)) - self._output_min) \
            * (self.p_max - self.p_min) / (self._output_max - self._output_min) + self.p_min

    def _convert_to_bar(self, val):
        conversion = {
            'mbar': 0.001,
            'kPa': 0.01,
            'MPa': 10,
            'psi': 0.0689476,
        }
        return val * conversion.get(self.unit, 1)

    def get_pressure(self):
        return self.last_pressure

    cmd_QUERY_ABP_PRESSURE_help = "Queries a pressure reading from a Honeywell ABP series sensor"
    def cmd_QUERY_ABP_PRESSURE(self, gcmd):
        gcmd.respond_raw("Last pressure on '{0}' was {1:.2f} bar.".format(self.sensor_name, self.last_pressure))

    def get_status(self, eventtime):
        return {'last_pressure': self.last_pressure}

def load_config_prefix(config):
    return HoneywellABP(config)
