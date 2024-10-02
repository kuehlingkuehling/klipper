# coding=utf-8
# A pressure watchdog, executing gcode on configured pressure thresholds
#
# Copyright (C) 2021  Simon Kühling <mail@simonkuehling.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

# TODO
# - documentation

class PressureWatchdog:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        gcode_macro = self.printer.load_object(config, 'gcode_macro')

        self.name = config.get_name().split()[1]
        self.interval = config.getfloat('interval', 1.0, minval=0.1)
        self.max_pressure = config.getfloat('max_pressure', 8.0, minval=-1.0, maxval=10.0)
        self.min_pressure = config.getfloat('min_pressure', 4.0, minval=-1.0, maxval=10.0)
        self.hysteresis = config.getfloat('hysteresis', 0.1, minval=0.05, maxval=1.0)
        self.sensor = self.printer.lookup_object(config.get('pressure_sensor').strip())
        self.max_pressure_gcode = gcode_macro.load_template(config, 'max_pressure_gcode')
        self.min_pressure_gcode = gcode_macro.load_template(config, 'min_pressure_gcode')

        self.last_pressure = 0
        self.max_pressure_triggered = False
        self.min_pressure_triggered = False
        self.timer_handler = None
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        self.last_pressure = self.sensor.get_pressure()
        waketime = self.reactor.monotonic() + self.interval
        self.timer_handler = self.reactor.register_timer(
            self._timer_event, waketime)

    def _timer_event(self, eventtime):
        if not self.printer.is_shutdown():
            cur_pressure = self.sensor.get_pressure()

            # pressure too high!
            if (cur_pressure > self.max_pressure) and not self.max_pressure_triggered:
                self.max_pressure_triggered = True
                try:
                    self.gcode.run_script(self.max_pressure_gcode.render())
                except:
                    logging.exception("Script running error")

            # pressure too low!
            if (cur_pressure < self.min_pressure) and not self.min_pressure_triggered:
                self.min_pressure_triggered = True
                try:
                    self.gcode.run_script(self.min_pressure_gcode.render())
                except:
                    logging.exception("Script running error")

            # pressure in range (with hysteresis check)
            if (cur_pressure >= (self.min_pressure + self.hysteresis)) and (cur_pressure <= (self.max_pressure - self.hysteresis)):
                self.max_pressure_triggered = False
                self.min_pressure_triggered = False

        nextwake = self.reactor.monotonic() + self.interval
        return nextwake

    def get_status(self, eventtime):
            return {'triggered': (self.max_pressure_triggered or self.min_pressure_triggered),
                    'max_pressure_triggered': self.max_pressure_triggered,
                    'min_pressure_triggered': self.min_pressure_triggered}

def load_config_prefix(config):
    return PressureWatchdog(config)
