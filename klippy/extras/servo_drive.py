# Framework for adding servo drives to Klipper
#
# Copyright (C) 2025 Jules
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import threading
import queue
import time
from klippy import chelper

class ServoStepper:
    def __init__(self, config, units_in_radians=False):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.units_in_radians_flag = units_in_radians

        # Load protocol
        protocol_name = config.get('servo_protocol', 'simple')
        self.protocol = self.printer.load_object(
            config, 'servo_protocol_' + protocol_name, self.name)

        # Motion parameters
        # We use standard rotation_distance / full_steps_per_rotation logic
        # but for servos it might just be 1.0 if they handle units directly.
        self.rotation_dist = config.getfloat(
            'rotation_distance', 1.0, above=0.0)
        self.steps_per_rotation = config.getfloat(
            'full_steps_per_rotation', 1.0, above=0.0)
        self.step_dist = self.rotation_dist / self.steps_per_rotation

        # Iterative solver setup
        self.ffi_main, self.ffi_lib = chelper.get_ffi()
        self.sk = self.ffi_main.gc(
            self.ffi_lib.cartesian_stepper_alloc(b'x'), self.ffi_lib.free)
        self.trapq = self.ffi_main.NULL

        # Communication queue and thread
        self.cmd_queue = queue.Queue()
        self.update_rate = config.getfloat('update_rate', 100.0, above=0.0)
        self.update_interval = 1.0 / self.update_rate
        self.last_pos = None
        self.last_flush_time = 0.0

        self.active_callbacks = []
        self.status_info = {}

        # Start background thread
        self.thread = threading.Thread(target=self._thread_main)
        self.thread.daemon = True
        self.thread.name = "ServoDrive:%s" % (self.name,)
        self.thread.start()

        # Register with helper modules
        for mname in ["stepper_enable", "force_move", "motion_report"]:
            m = self.printer.load_object(config, mname)
            m.register_stepper(config, self)

        # Register G-code commands
        gcode = self.printer.lookup_object('gcode')
        short_name = self.get_name(short=True)
        gcode.register_mux_command("SERVO_DRIVE_WRITE", "SERVO", short_name,
                                   self.cmd_SERVO_DRIVE_WRITE)

    def get_name(self, short=False):
        if short and self.name.startswith("stepper_"):
            return self.name[8:]
        return self.name

    def units_in_radians(self):
        return self.units_in_radians_flag

    def get_mcu(self):
        # Return the primary MCU for timing reference
        return self.printer.lookup_object('mcu')

    def setup_itersolve(self, alloc_func, *params):
        self.sk = self.ffi_main.gc(
            getattr(self.ffi_lib, alloc_func)(*params), self.ffi_lib.free)

    def set_trapq(self, tq):
        self.trapq = tq
        if tq is None:
            tq = self.ffi_main.NULL
        self.ffi_lib.itersolve_set_trapq(self.sk, tq)

    def generate_steps(self, flush_time):
        if self.trapq == self.ffi_main.NULL:
            return

        # Check for activity callbacks
        if self.active_callbacks:
            ret = self.ffi_lib.itersolve_check_active(self.sk, flush_time)
            if ret:
                cbs = self.active_callbacks
                self.active_callbacks = []
                for cb in cbs:
                    cb(ret)

        # Update iterative solver to current time
        self.ffi_lib.itersolve_generate_steps(self.sk, flush_time)
        pos = self.ffi_lib.itersolve_get_commanded_pos(self.sk)

        if pos != self.last_pos:
            self.cmd_queue.put(('pos', (flush_time, pos)))
            self.last_pos = pos
        self.last_flush_time = flush_time

    def set_position(self, coord):
        # itersolve_set_position updates the internal sk commanded_pos
        self.ffi_lib.itersolve_set_position(
            self.sk, coord[0], coord[1], coord[2])
        self.last_pos = self.ffi_lib.itersolve_get_commanded_pos(self.sk)
        self.cmd_queue.put(('pos', (self.last_flush_time, self.last_pos)))

    def get_commanded_position(self):
        return self.ffi_lib.itersolve_get_commanded_pos(self.sk)

    def get_step_dist(self):
        return self.step_dist

    def add_active_callback(self, cb):
        self.active_callbacks.append(cb)

    def note_homing_end(self):
        pass

    def get_past_mcu_position(self, print_time):
        return int(self.get_commanded_position() / self.step_dist)

    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos * self.step_dist

    def get_tmc_current_helper(self):
        return None

    def get_stepper_kinematics(self):
        return self.sk

    def motor_enable(self, print_time):
        self.cmd_queue.put(('enable', (print_time, True)))

    def motor_disable(self, print_time):
        self.cmd_queue.put(('enable', (print_time, False)))

    def _thread_main(self):
        logging.info("ServoDrive thread started for %s", self.name)
        last_poll_time = 0.0
        while True:
            try:
                # Wait for next command or timeout for polling
                try:
                    msg = self.cmd_queue.get(timeout=self.update_interval)
                    mtype, mdata = msg
                    if mtype == 'pos':
                        self.protocol.handle_set_position(mdata[1], mdata[0])
                    elif mtype == 'enable':
                        self.protocol.handle_enable(mdata[0], mdata[1])
                    elif mtype == 'write':
                        self.protocol.handle_write(mdata)
                except queue.Empty:
                    pass

                # Periodic status poll
                now = time.time()
                if now > last_poll_time + 0.5: # 2Hz poll for status
                    status = self.protocol.handle_get_status()
                    if status:
                        self.status_info.update(status)
                    last_poll_time = now

            except Exception:
                logging.exception("Error in ServoDrive thread %s", self.name)
                time.sleep(1.0)

    def get_status(self, eventtime):
        status = dict(self.status_info)
        status['commanded_position'] = self.get_commanded_position()
        return status

    def cmd_SERVO_DRIVE_WRITE(self, gcmd):
        val = gcmd.get('VALUE')
        self.cmd_queue.put(('write', val))

def load_config(config, units_in_radians=False):
    return ServoStepper(config, units_in_radians)

class ServoProtocol:
    def __init__(self, config, name):
        pass
    def handle_set_position(self, position, print_time):
        pass
    def handle_enable(self, print_time, enable):
        pass
    def handle_write(self, value):
        pass
    def handle_get_status(self):
        return {}
