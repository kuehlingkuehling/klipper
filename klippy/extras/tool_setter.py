# Tool Setter for Nozzle alignment module for 3d kinematic probes.
#
# This module has been adapted by
#     Simon Kühling <mail@simonkuehling.de>
# from code written by
#     https://github.com/ben5459/Klipper_ToolChanger/blob/master/probe_multi_axis.py
#         Kevin O'Connor <kevin@koconnor.net>
#         Martin Hierholzer <martin@hierholzer.info>
#     https://github.com/viesturz/klipper-toolchanger/blob/main/klipper/extras/tools_calibrate.py
#         Viesturs Zariņš <viesturz@gmail.com>


import logging
import numpy as np
import math

# defining normal vectors of each probing direction
direction_types = {'A+': [0.866, 0.5, 0.0],
                   'A-': [-0.866, -0.5, 0.0],
                   'B+': [-0.866, 0.5, 0.0],
                   'B-': [0.866, -0.5, 0.0],
                   'C+': [0.0, -1.0, 0.0],
                   'C-': [0.0, 1.0, 0.0],
                   'Z+': [0.0, 0.0, 1.0],
                   'Z-': [0.0, 0.0, -1.0]}


HINT_TIMEOUT = """
If the probe did not move far enough to trigger, then
consider reducing/increasing the axis minimum/maximum
position so the probe can travel further (the minimum
position can be negative).
"""


class ToolSetter:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.gcode_move = self.printer.load_object(config, "gcode_move")
        self.probe_multi_axis = PrinterProbeMultiAxis(config,
                                                      ProbeEndstopWrapper(
                                                          config))
        self.probe_name = config.get('probe', 'probe')
        self.travel_speed = config.getfloat('travel_speed', 10.0, above=0.)
        self.xy_spread = config.getfloat('xy_spread', 5.0)
        self.z_lower = config.getfloat('z_lower', 0.5)
        self.z_lift = config.getfloat('z_lift', 1.0)
        self.retract_speed = config.getfloat('retract_speed',
                                          self.probe_multi_axis.retract_speed)
        self.final_z_lift = config.getfloat('final_z_lift', 4.0)
        self.sensor_location = None
        self.last_result = [0., 0., 0.]
        self.probe_triggered = False

        # Register commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('TOOL_SETTER_LOCATE_SENSOR',
                                    self.cmd_TOOL_SETTER_LOCATE_SENSOR,
                                    desc=self.cmd_TOOL_SETTER_LOCATE_SENSOR_help)
        self.gcode.register_command('TOOL_SETTER_CALIBRATE_TOOL_OFFSET',
                                    self.cmd_TOOL_SETTER_CALIBRATE_TOOL_OFFSET,
                                    desc=self.cmd_TOOL_SETTER_CALIBRATE_TOOL_OFFSET_help)
        self.gcode.register_command('TOOL_SETTER_QUERY_PROBE',
                                    self.cmd_TOOL_SETTER_QUERY_PROBE,
                                    desc=self.cmd_TOOL_SETTER_QUERY_PROBE_help)

    def probe_xy(self, toolhead, top_pos, direction, gcmd, samples=None):
        vector = direction_types[direction]
        start_pos = list(top_pos)
        # calculate probing start positions from xy_spread and direction vector
        start_pos[0] = start_pos[0] - self.xy_spread * vector[0]
        start_pos[1] = start_pos[1] - self.xy_spread * vector[1]
        # move to start position
        toolhead.manual_move([None, None, top_pos[2] + self.z_lift],
                             self.retract_speed)
        toolhead.manual_move([start_pos[0], start_pos[1], None],
                             self.travel_speed)
        toolhead.manual_move([None, None, top_pos[2] - self.z_lower],
                             self.retract_speed)
        pos_now = self.printer.lookup_object('toolhead').get_position()
        logging.info("toolhead position is now = [ %.3f, %.3f, %.3f ]" % ( pos_now[0], pos_now[1], pos_now[2] ))
        return self.probe_multi_axis.run_probe(direction, gcmd, samples=samples,
                                               max_distance=self.xy_spread * 1.8)

    def calibrate_xy(self, toolhead, top_pos, gcmd, samples=None):
        probe_a = self.probe_xy(toolhead, top_pos, 'A-', gcmd, samples=samples)
        probe_b = self.probe_xy(toolhead, top_pos, 'B-', gcmd, samples=samples)
        probe_c = self.probe_xy(toolhead, top_pos, 'C-', gcmd, samples=samples)
        
        # use the three probe points as coordinates of a triangle
        x1, y1, *_ = probe_a
        x2, y2, *_ = probe_b
        x3, y3, *_ = probe_c

        # Calculate the determinant
        D = 2 * (x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2))

        if D == 0:
            raise gcmd.error(
                "The points are colinear or do not form a valid triangle.")

        # Calculate the coordinates of the circumcenter
        Ux = ((x1**2 + y1**2)*(y2 - y3) + (x2**2 + y2**2)*(y3 - y1) + (x3**2 + y3**2)*(y1 - y2)) / D
        Uy = ((x1**2 + y1**2)*(x3 - x2) + (x2**2 + y2**2)*(x1 - x3) + (x3**2 + y3**2)*(x2 - x1)) / D

        return [Ux, Uy]

    def locate_sensor(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        position = toolhead.get_position()
        downPos = self.probe_multi_axis.run_probe("Z-", gcmd, samples=1)
        logging.info("rough z top position is = %.3f" % ( downPos[2] ))
        center_x, center_y = self.calibrate_xy(toolhead, downPos, gcmd,
                                               samples=1)

        toolhead.manual_move([None, None, downPos[2] + self.z_lift],
                             self.retract_speed)
        toolhead.manual_move([center_x, center_y, None], self.travel_speed)
        center_z = self.probe_multi_axis.run_probe("Z-", gcmd, speed_ratio=0.5)[
            2]
        logging.info("exact z top position is = %.3f" % ( center_z ))
        # Now redo X and Y, since we have a more accurate center.
        center_x, center_y = self.calibrate_xy(toolhead,
                                               [center_x, center_y, center_z],
                                               gcmd)

        # rest above center
        position[0] = center_x
        position[1] = center_y
        position[2] = center_z + self.final_z_lift
        toolhead.manual_move([None, None, position[2]], self.retract_speed)
        toolhead.manual_move([position[0], position[1], None],
                             self.travel_speed)
        toolhead.set_position(position)
        return [center_x, center_y, center_z]

    cmd_TOOL_SETTER_LOCATE_SENSOR_help = ("Locate the tool setter sensor, "
                                   "use with probe tool.")

    def cmd_TOOL_SETTER_LOCATE_SENSOR(self, gcmd):
        self.last_result = self.locate_sensor(gcmd)
        self.sensor_location = self.last_result
        self.gcode.respond_info("Sensor location at %.6f,%.6f,%.6f"
                                % (self.last_result[0], self.last_result[1],
                                   self.last_result[2]))

    cmd_TOOL_SETTER_CALIBRATE_TOOL_OFFSET_help = "Calibrate current tool offset relative to probe tool"

    def cmd_TOOL_SETTER_CALIBRATE_TOOL_OFFSET(self, gcmd):
        if not self.sensor_location:
            raise gcmd.error(
                "No recorded sensor location, please run TOOL_SETTER_LOCATE_SENSOR first")
        location = self.locate_sensor(gcmd)
        self.last_result = [location[i] - self.sensor_location[i] for i in
                            range(3)]
        self.gcode.respond_info("Tool offset is %.6f,%.6f,%.6f"
                                % (self.last_result[0], self.last_result[1],
                                   self.last_result[2]))

    def get_status(self, eventtime):
        return {'last_result': self.last_result,
                'probe_triggered': self.probe_triggered,
                'last_x_result': self.last_result[0],
                'last_y_result': self.last_result[1],
                'last_z_result': self.last_result[2]}

    cmd_TOOL_SETTER_QUERY_PROBE_help = "Return the state of calibration probe"
    def cmd_TOOL_SETTER_QUERY_PROBE(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        endstop_state = self.probe_multi_axis.mcu_probe.query_endstop(print_time) # Check the state of probe
        self.probe_triggerend = not endstop_state
        gcmd.respond_info("Tool Setter Probe: %s" % (["open", "TRIGGERED"][endstop_state]))

class PrinterProbeMultiAxis:
    def __init__(self, config, mcu_probe):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.mcu_probe = mcu_probe
        self.probe_speed = config.getfloat('probe_speed', 5.0, above=0.)
        self.retract_speed = config.getfloat('retract_speed', self.probe_speed, above=0.)
        self.last_state = False
        self.last_result = [0., 0., 0.]
        self.last_x_result = 0.
        self.last_y_result = 0.
        self.last_z_result = 0.
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_move = self.printer.load_object(config, "gcode_move")

        # Multi-sample support (for improved accuracy)
        self.sample_count = config.getint('samples', 1, minval=1)
        self.sample_retract_dist = config.getfloat('sample_retract_dist', 2.,
                                                   above=0.)
        atypes = {'median': 'median', 'average': 'average'}
        self.samples_result = config.getchoice('samples_result', atypes,
                                               'average')
        self.samples_tolerance = config.getfloat('samples_tolerance', 0.100,
                                                 minval=0.)
        self.samples_retries = config.getint('samples_tolerance_retries', 0,
                                             minval=0)
        # Register xyz_virtual_endstop pin
        self.printer.lookup_object('pins').register_chip('probe_multi_axis',
                                                         self)

    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'xy_virtual_endstop':
            raise pins.error("Probe virtual endstop only useful as endstop pin")
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error("Can not pullup/invert probe virtual endstop")
        return self.mcu_probe

    def get_retract_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("RETRACT_SPEED", self.retract_speed, above=0.)
        return self.retract_speed

    def _probe(self, speed, direction, max_distance):
        phoming = self.printer.lookup_object('homing')
        pos = self._get_target_position(direction, max_distance)
        try:
            epos = phoming.probing_move(self.mcu_probe, pos, speed)
        except self.printer.command_error as e:
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += HINT_TIMEOUT
            raise self.printer.command_error(reason)
        # self.gcode.respond_info("probe at %.3f,%.3f is z=%.6f"
        self.gcode.respond_info("Probe made contact at %.6f,%.6f,%.6f"
                                % (epos[0], epos[1], epos[2]))
        return epos[:3]

    def _get_target_position(self, direction, max_distance):
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'x' not in toolhead.get_status(curtime)['homed_axes'] or \
                'y' not in toolhead.get_status(curtime)['homed_axes'] or \
                'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before probe")
        pos = toolhead.get_position()
        kin_status = toolhead.get_kinematics().get_status(curtime)
        if 'axis_minimum' not in kin_status or 'axis_minimum' not in kin_status:
            raise self.gcode.error(
                "Tools calibrate only works with cartesian kinematics")
        vector = direction_types[direction]
        pos[0] = pos[0] + max_distance * vector[0]
        pos[1] = pos[1] + max_distance * vector[1]
        pos[2] = pos[2] + max_distance * vector[2]
        return pos

    def _move(self, coord, speed):
        self.printer.lookup_object('toolhead').manual_move(coord, speed)

    def _calc_mean(self, positions):
        points = np.array(positions)

        # Arithmetic mean (average of each coordinate)
        return np.mean(points, axis=0)

    def _calc_median(self, point_list, eps=1e-5):
        points = np.array(point_list)
        median = np.mean(points, axis=0)  # Initial value: Arithmetic mean

        # Find geometric median by iterative algorithm
        while True:
            distances = np.linalg.norm(points - median, axis=1)
            # Avoid division by zero
            non_zero = distances > eps
            if not np.any(non_zero):
                break
            weights = 1 / np.maximum(distances, eps)
            new_median = np.sum(points * weights[:, np.newaxis], axis=0) / np.sum(weights)
            movement = np.linalg.norm(new_median - median)
            if movement < eps:
                break
            median = new_median

        return median
    
    def _calc_tolerance_of_points(self, reference_point, measurement_points):
        distances = []

        x0 = reference_point[0]
        y0 = reference_point[1]
        z0 = reference_point[2]

        for idx, point in enumerate(measurement_points):
            x, y, z = point
            # Calculate Euclidean distance
            distance = math.sqrt((x - x0)**2 + (y - y0)**2 + (z - z0)**2)
            distances.append(distance)

        max_distance = max(distances)
        min_distance = min(distances)
        tolerance_span = max_distance - min_distance
        return tolerance_span

    def run_probe(self, direction, gcmd, speed_ratio=1.0, samples=None,
                  max_distance=100.0):
        speed = gcmd.get_float("PROBE_SPEED", self.probe_speed,
                               above=0.) * speed_ratio
        
        vector = direction_types[direction]

        retract_speed = self.get_retract_speed(gcmd)
        sample_count = gcmd.get_int("SAMPLES",
                                    samples if samples else self.sample_count,
                                    minval=1)
        sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST",
                                             self.sample_retract_dist, above=0.)
        samples_tolerance = gcmd.get_float("SAMPLES_TOLERANCE",
                                           self.samples_tolerance, minval=0.)
        samples_retries = gcmd.get_int("SAMPLES_TOLERANCE_RETRIES",
                                       self.samples_retries, minval=0)
        samples_result = gcmd.get("SAMPLES_RESULT", self.samples_result)

        probe_start = self.printer.lookup_object('toolhead').get_position()
        retries = 0
        positions = []
        while len(positions) < sample_count:
            # Probe position
            pos = self._probe(speed, direction, max_distance)
            positions.append([pos[0],pos[1],pos[2]])
            # Check samples tolerance
            tolerance = self._calc_tolerance_of_points(probe_start, positions)
            if tolerance > samples_tolerance:
                if retries >= samples_retries:
                    raise gcmd.error("Probe samples exceed samples_tolerance")
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...")
                retries += 1
                positions = []
            # Retract
            if len(positions) < sample_count:
                retract_pos = pos
                # calculate retract target pos from direction vector 
                retract_pos[0] = retract_pos[0] - sample_retract_dist * vector[0]
                retract_pos[1] = retract_pos[1] - sample_retract_dist * vector[1]
                retract_pos[2] = retract_pos[2] - sample_retract_dist * vector[2]
                self._move(retract_pos, retract_speed)

        # Calculate and return result
        if samples_result == 'median':
            return self._calc_median(positions)
        return self._calc_mean(positions)


# Endstop wrapper that enables probe specific features
class ProbeEndstopWrapper:
    def __init__(self, config):
        self.printer = config.get_printer()
        # Create an "endstop" object to handle the probe pin
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        ppins.allow_multi_use_pin(pin.replace('^', '').replace('!', ''))
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params['chip']
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self._get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop

    def _get_steppers(self):
        return self.mcu_endstop.get_steppers()

    def _handle_mcu_identify(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            self.add_stepper(stepper)

    def get_position_endstop(self):
        return 0.


def load_config(config):
    return ToolSetter(config)
