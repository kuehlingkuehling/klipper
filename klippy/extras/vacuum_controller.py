# coding=utf-8
# A vacuum pressure controller (for vacuum print beds and such)
#
# Copyright (C) 2021  Simon Kühling <mail@simonkuehling.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

# TODO
# - documentation
# - implement configurable evacuation timeout (execute gcode macro block if exceeded)

PIN_MIN_TIME = 0.100

class VacuumController:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        self.reactor = self.printer.get_reactor()
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.gcode = self.printer.lookup_object('gcode')

        # load config
        self.name = config.get_name().split()[1]
        self.interval = config.getfloat('interval', 0.1, minval=0.1)
        self.pressure = config.getfloat('pressure', -0.7, minval=-1.0, maxval=-0.1)
        self.threshold = config.getfloat('threshold', -0.5, minval=-1.0, maxval=-0.1)
        self.timeout = config.getfloat('low_pressure_timeout', 30, minval=0.1)
        self.low_pressure_gcode = gcode_macro.load_template(config, 'low_pressure_gcode')
        self.sensor = self.printer.lookup_object(config.get('pressure_sensor').strip())

        # setup pins
        self.evacuation_pin = ppins.setup_pin('digital_out', config.get('evacuation_pin'))
        self.venting_pin = ppins.setup_pin('digital_out', config.get('venting_pin'))
        self.evacuation_pin.setup_max_duration(0.)
        self.venting_pin.setup_max_duration(0.)

        # runtime variables
        self.is_running = 0
        self.is_evacuating = 0
        self.is_venting = 0
        self.timer_handler = None
        self.last_time_pressure_ok = 0
        self.vacuum_ready = False

        # register callbacks and commands
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.gcode.register_mux_command(
            "SET_VACUUM", "ID", self.name,
            self.cmd_SET_VACUUM,
            desc=self.cmd_SET_VACUUM_help)
        self.gcode.register_mux_command(
            "VACUUM_WAIT", "ID", self.name,
            self.cmd_VACUUM_WAIT,
            desc=self.cmd_VACUUM_WAIT_help)

    def _handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        waketime = self.reactor.monotonic() + self.interval
        self.timer_handler = self.reactor.register_timer(
            self._timer_event, waketime)

    def _timer_event(self, eventtime):
        if not self.printer.is_shutdown():
            cur_pressure = self.sensor.get_pressure()
            if self.is_running:
                # vacuum control logic
                if self.is_venting:
                    self._set_venting(0)
                if not self.is_evacuating and (cur_pressure > self.threshold):
                    self._set_evacuation(1)
                elif self.is_evacuating and (cur_pressure < self.pressure):
                    self._set_evacuation(0)
                    self.vacuum_ready = True

                # low pressure timeout logic
                if cur_pressure < self.threshold:
                    self.last_time_pressure_ok = eventtime
                elif self.last_time_pressure_ok and ((eventtime - self.last_time_pressure_ok) > self.timeout):
                    try:
                        self.gcode.run_script(self.low_pressure_gcode.render())
                    except:
                        logging.exception("Script running error")
                    self.is_running = 0
                    self.vacuum_ready = False
            else:
                # venting logic
                if self.is_evacuating:
                    self._set_evacuation(0)
                if (cur_pressure < -0.05) and not self.is_venting:
                    self._set_venting(1)
                elif (cur_pressure >= -0.02) and self.is_venting:
                    self._set_venting(0)
        nextwake = self.reactor.monotonic() + self.interval
        return nextwake

    def _set_evacuation(self, value):
        if value != self.is_evacuating:
            curtime = self.reactor.monotonic()
            print_time = self.evacuation_pin.get_mcu().estimated_print_time(curtime) + PIN_MIN_TIME
            self.evacuation_pin.set_digital(print_time, value)
            self.is_evacuating = value

    def _set_venting(self, value):
        if value != self.is_venting:
            curtime = self.reactor.monotonic()
            print_time = self.venting_pin.get_mcu().estimated_print_time(curtime) + PIN_MIN_TIME
            self.venting_pin.set_digital(print_time, value)
            self.is_venting = value

    cmd_SET_VACUUM_help = "Update the parameters of a vacuum controller"
    def cmd_SET_VACUUM(self, gcmd):
        self.last_time_pressure_ok = 0
        self.vacuum_ready = False
        self.is_running = gcmd.get_int('ENABLE', minval=0, maxval=1)

    cmd_VACUUM_WAIT_help = "Wait for vacuum controller to reach target pressure"
    def cmd_VACUUM_WAIT(self, gcmd):
        wait_timeout = gcmd.get_float('TIMEOUT', minval=1.)

        reactor = self.printer.get_reactor()
        start_time = eventtime = reactor.monotonic()
        if self.is_running:
            while not self.printer.is_shutdown() and ((reactor.monotonic() - start_time) < wait_timeout) and self.is_running:
                if self.vacuum_ready:
                    return
                eventtime = reactor.pause(eventtime + 1.)
            raise self.gcode.error("VACUUM_WAIT: Target pressure could not be reached within timeout period!")
        else:
            raise self.gcode.error("VACUUM_WAIT: Vacuum is not running!")

def load_config_prefix(config):
    return VacuumController(config)
