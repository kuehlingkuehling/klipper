# coding=utf-8
# Support proportional solenoid valves as fans that are controlled by gcode
#
# Copyright (C) 2021  Simon Kühling <mail@simonkuehling.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

# TODO
# - documentation

FAN_MIN_TIME = 0.100

class PrinterFanProportional:
    cmd_SET_FAN_SPEED_help = "Sets the speed of a fan"
    def __init__(self, config):
        self.printer = config.get_printer()
        self.fan_name = config.get_name().split()[-1]

        # Read config
        self.min_pwm = config.getfloat('min_pwm', 0., above=0., maxval=1.)
        self.max_pwm = config.getfloat('max_pwm', 0., above=0., maxval=1.)
        self.kick_start_time = config.getfloat('kick_start_time', 0.1,
                                               minval=0.)
        cycle_time = config.getfloat('cycle_time', 0.0005, above=0.)
        hardware_pwm = config.getboolean('hardware_pwm', False)
        shutdown_speed = config.getfloat(
            'shutdown_speed', 0., minval=0., maxval=1.)

        # Setup pwm object
        ppins = self.printer.lookup_object('pins')
        self.fan_pin = ppins.setup_pin('pwm', config.get('pin'))
        self.fan_pin.setup_max_duration(0.)
        self.fan_pin.setup_cycle_time(cycle_time, hardware_pwm)
        shutdown_power = max(0., min(self.max_pwm, shutdown_speed))
        self.fan_pin.setup_start_value(0., shutdown_power)

        # Runtime variables
        self.last_fan_value = 0.
        self.last_fan_time = 0.

        # Register callbacks
        self.printer.register_event_handler("gcode:request_restart",
                                            self._handle_request_restart)

        # Register gcode command
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SET_FAN_SPEED", "FAN",
                                   self.fan_name,
                                   self.cmd_SET_FAN_SPEED,
                                   desc=self.cmd_SET_FAN_SPEED_help)

    def get_mcu(self):
        return self.fan_pin.get_mcu()
    def set_speed(self, print_time, value):
        # limit value to 0..1
        value = max(0., min(1., value))
        if value == self.last_fan_value:
            return

        if value == 0:
            pwm = 0
        else:
            # scale value to output pwm range
            pwm = self.min_pwm + (value * (self.max_pwm - self.min_pwm))

        print_time = max(self.last_fan_time + FAN_MIN_TIME, print_time)
        if value and self.kick_start_time:
            # Bump proportional valve at full power for specified kick_start_time
            # to break static friction of the valve piston
            self.fan_pin.set_pwm(print_time, 1.0)
            print_time += self.kick_start_time
        self.fan_pin.set_pwm(print_time, pwm)
        self.last_fan_time = print_time
        self.last_fan_value = value
    def set_speed_from_command(self, value):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt:
                                              self.set_speed(pt, value)))
    def _handle_request_restart(self, print_time):
        self.set_speed(print_time, 0.)

    def get_status(self, eventtime):
        return { 'speed': self.last_fan_value }
    def cmd_SET_FAN_SPEED(self, gcmd):
        speed = gcmd.get_float('SPEED', 0.)
        self.set_speed_from_command(speed)

def load_config_prefix(config):
    return PrinterFanProportional(config)
