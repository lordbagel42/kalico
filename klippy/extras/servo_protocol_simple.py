# Simple reference protocol for ServoDrive framework
#
# Copyright (C) 2025 Jules
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

class ServoProtocolSimple:
    def __init__(self, config, name):
        self.printer = config.get_printer()
        self.name = name
        logging.info("ServoProtocolSimple initialized for %s", name)

    def handle_set_position(self, position, print_time):
        # In a real implementation, send position to drive via
        # serial/ethercat/etc.
        pass

    def handle_enable(self, print_time, enable):
        logging.info("Servo %s: enable=%s at %.4f",
                     self.name, enable, print_time)

    def handle_write(self, value):
        logging.info("Servo %s: manual write value=%s", self.name, value)

    def handle_get_status(self):
        # Return random status info as requested
        return {
            "status": "ready",
            "delta_position": 0.0,
            "encoder_info": "ok"
        }

def load_config(config, name):
    return ServoProtocolSimple(config, name)
