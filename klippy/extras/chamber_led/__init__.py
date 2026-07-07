# Particle Boron based "chamber_led" USB-serial RGB LED controller
#
# See docs/Chamber_LED_Implementation_Spec.md for the wire protocol and
# design rationale. Wraps klippy/extras/led.py's LEDHelper the same way
# klippy/extras/neopixel.py does, so SET_LED/SET_LED_TEMPLATE work with
# a chamber_led the same as any other LED strip. Also exposes a couple
# of direct gcode commands, and optional automatic color changes driven
# by print_stats state transitions.
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os

from .. import led
from .transport import ChamberLedTransport

# print_stats (klippy/extras/print_stats.py) has no dedicated "idle"
# event, but note_cancel()'s "cancelled_printing" is the transition back
# to an idle chamber (print stopped, nothing queued), so color_idle
# rides on that event.
PRINT_STATS_COLOR_EVENTS = (
    ("color_printing", "print_stats:start_printing"),
    ("color_paused", "print_stats:paused_printing"),
    ("color_complete", "print_stats:complete_printing"),
    ("color_error", "print_stats:error_printing"),
    ("color_idle", "print_stats:cancelled_printing"),
)

STATUS_FIELDS = (
    "mode",
    "r",
    "g",
    "b",
    "brightness",
    "uptime_ms",
    "free_mem",
)
STATUS_INT_FIELDS = ("r", "g", "b", "brightness", "uptime_ms", "free_mem")


def _parse_color_option(config, option):
    values = config.getintlist(option, None)
    if values is None:
        return None
    if len(values) not in (3, 4):
        raise config.error(
            "Option '%s' in section '%s' must be 'R,G,B' or"
            " 'R,G,B,BRIGHTNESS'" % (option, config.get_name())
        )
    for v in values:
        if v < 0 or v > 255:
            raise config.error(
                "Option '%s' in section '%s' must have values between"
                " 0 and 255" % (option, config.get_name())
            )
    brightness = values[3] if len(values) == 4 else 255
    return (values[0], values[1], values[2], brightness)


def parse_status_reply(reply):
    # "STATUS mode=<off|solid> r=<n> g=<n> b=<n> brightness=<n>
    #  uptime_ms=<n> free_mem=<n>"
    parts = reply.split()
    if not parts or parts[0] != "STATUS":
        return None
    status = {}
    for part in parts[1:]:
        key, sep, value = part.partition("=")
        if not sep:
            return None
        status[key] = value
    if any(field not in status for field in STATUS_FIELDS):
        return None
    for field in STATUS_INT_FIELDS:
        try:
            status[field] = int(status[field])
        except ValueError:
            return None
    return status


class ChamberLED:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.full_name = config.get_name()
        self.name = self.full_name.split()[-1]
        self.serial_path = config.get("serial")
        self.baud = config.getint("baud", 115200)

        self.transport = ChamberLedTransport(
            printer, self.full_name, on_io_error=self._handle_io_error
        )
        self.connected = False
        self.last_status = {}

        self.auto_colors = {}
        for option, event in PRINT_STATS_COLOR_EVENTS:
            color = _parse_color_option(config, option)
            if color is not None:
                self.auto_colors[event] = color
        if self.auto_colors:
            # Only load print_stats (and register for its events) if the
            # user actually configured at least one automatic color --
            # a user who only wants manual SET_LED control shouldn't pay
            # for or depend on print_stats at all.
            printer.load_object(config, "print_stats")
            for event in self.auto_colors:
                printer.register_event_handler(
                    event, self._make_auto_color_handler(event)
                )

        # A chamber_led is always a single logical zone: the wire
        # protocol has no addressing, just one COLOR for the whole
        # strip.
        self.led_helper = led.LEDHelper(config, self._update_leds, 1)

        printer.register_event_handler("klippy:ready", self._handle_ready)
        printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        printer.register_event_handler(
            "klippy:disconnect", self._handle_shutdown
        )

        gcode = printer.lookup_object("gcode")
        gcode.register_mux_command(
            "CHAMBER_LED_OFF",
            "CHAMBER_LED",
            self.name,
            self.cmd_CHAMBER_LED_OFF,
            desc=self.cmd_CHAMBER_LED_OFF_help,
        )
        gcode.register_mux_command(
            "CHAMBER_LED_STATUS",
            "CHAMBER_LED",
            self.name,
            self.cmd_CHAMBER_LED_STATUS,
            desc=self.cmd_CHAMBER_LED_STATUS_help,
        )

    def get_status(self, eventtime=None):
        status = self.led_helper.get_status(eventtime)
        status["connected"] = self.connected
        return status

    # Color handling
    def _make_auto_color_handler(self, event):
        def handler():
            r, g, b, brightness = self.auto_colors[event]
            self._set_rgb_brightness(r, g, b, brightness)

        return handler

    def _set_rgb_brightness(self, r, g, b, brightness):
        # Route through LEDHelper so manual SET_LED calls and automatic
        # print_stats-driven changes share one source of truth for the
        # LED's current color (used by get_status/color_data and by
        # SET_LED's own change-diffing). Brightness has no direct
        # LEDHelper equivalent, so it is folded into the RGB floats
        # here; _update_leds below undoes this by always sending the
        # wire protocol's brightness field as 255.
        scale = brightness / 255.0
        color = (
            (r / 255.0) * scale,
            (g / 255.0) * scale,
            (b / 255.0) * scale,
            0.0,
        )
        self.led_helper._set_color(None, color)
        self.led_helper._check_transmit(None)

    def _update_leds(self, led_state, print_time):
        red, green, blue, white = led_state[0]
        # The physical LEDs are WS2812 RGB, not RGBW -- there is no
        # white sub-pixel to drive. Rather than silently dropping WHITE
        # if a user (or a template) sets it, fold it additively into
        # each of R/G/B, clamped to the channel's maximum.
        r = min(1.0, red + white)
        g = min(1.0, green + white)
        b = min(1.0, blue + white)
        self._send_color(
            int(r * 255.0 + 0.5), int(g * 255.0 + 0.5), int(b * 255.0 + 0.5)
        )

    def _send_color(self, r, g, b, brightness=255):
        if not self.connected:
            return

        def cb(reply):
            if reply != "OK":
                logging.warning(
                    "chamber_led %s: unexpected COLOR reply: %s",
                    self.full_name,
                    reply,
                )

        self.transport.query(
            "COLOR %d %d %d %d" % (r, g, b, brightness), cb, timeout=1.0
        )

    # Connection lifecycle
    def _handle_ready(self):
        try:
            self._try_connect()
        except Exception as e:
            logging.warning(
                "chamber_led %s: could not connect on startup: %s",
                self.full_name,
                e,
            )

    def _handle_shutdown(self):
        self.transport.close()
        self.connected = False

    def _try_connect(self):
        if not os.path.exists(self.serial_path):
            raise self.printer.command_error(
                "chamber_led serial device '%s' does not exist"
                % (self.serial_path,)
            )
        self.transport.open(self.serial_path, self.baud)
        reply = self.transport.query_sync("PING", timeout=1.0)
        if reply != "PONG":
            self.transport.close()
            raise self.printer.command_error(
                "chamber_led %s did not respond to PING on %s"
                % (self.full_name, self.serial_path)
            )
        self.connected = True
        logging.info(
            "chamber_led %s: connected on %s",
            self.full_name,
            self.serial_path,
        )
        # Push Klipper's current idea of the LED color (initial_RED/etc,
        # or anything set by gcode before klippy:ready ran) so the
        # physical LED matches it instead of whatever the firmware
        # powered on with.
        self.led_helper.need_transmit = True
        self.led_helper._check_transmit(None)

    def _handle_io_error(self, reason):
        logging.warning(
            "chamber_led %s: transport error: %s", self.full_name, reason
        )
        self.transport.close()
        self.connected = False

    # GCode commands
    cmd_CHAMBER_LED_OFF_help = "Turn off the chamber LED"

    def cmd_CHAMBER_LED_OFF(self, gcmd):
        if not self.connected:
            raise gcmd.error(
                "chamber_led %s is not connected" % (self.full_name,)
            )
        reply = self.transport.query_sync("OFF", timeout=1.0)
        if reply != "OK":
            raise gcmd.error(
                "chamber_led %s: OFF failed (reply=%s)"
                % (self.full_name, reply)
            )
        self.led_helper._set_color(None, (0.0, 0.0, 0.0, 0.0))
        gcmd.respond_info("chamber_led %s: off" % (self.full_name,))

    cmd_CHAMBER_LED_STATUS_help = (
        "Query the chamber LED controller's raw STATUS reply"
    )

    def cmd_CHAMBER_LED_STATUS(self, gcmd):
        if not self.connected:
            raise gcmd.error(
                "chamber_led %s is not connected" % (self.full_name,)
            )
        reply = self.transport.query_sync("STATUS", timeout=1.0)
        if reply is None:
            raise gcmd.error(
                "chamber_led %s: no response to STATUS" % (self.full_name,)
            )
        status = parse_status_reply(reply)
        if status is None:
            raise gcmd.error(
                "chamber_led %s: unexpected STATUS reply: %s"
                % (self.full_name, reply)
            )
        self.last_status = status
        gcmd.respond_info(
            "chamber_led %s: mode=%s r=%d g=%d b=%d brightness=%d"
            " uptime_ms=%d free_mem=%d"
            % (
                self.full_name,
                status["mode"],
                status["r"],
                status["g"],
                status["b"],
                status["brightness"],
                status["uptime_ms"],
                status["free_mem"],
            )
        )


def load_config_prefix(config):
    return ChamberLED(config)
