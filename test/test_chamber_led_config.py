"""Config-parsing and STATUS-reply-parsing tests for
klippy/extras/chamber_led/__init__.py.

Exercises the module's pure helper functions (_parse_color_option,
parse_status_reply) against a real klippy.configfile.ConfigWrapper
backed by a real configparser, mirroring this repo's existing
"no full Klippy printer object graph" test philosophy (see
test_odrive_board.py, test_odrive_axis.py). These two functions don't
need a Printer at all, so no fakes are required here -- constructing
the full ChamberLED object (which also builds a led.LEDHelper, in turn
needing gcode_macro/display/configfile machinery) is left to the
device-less klippy.test harness (test/klippy/chamber_led.test) and the
wire-protocol tests against the simulated serial responder
(test_chamber_led_mock.py).
"""

from __future__ import annotations

import configparser

import pytest

from klippy import configfile
from klippy.extras import chamber_led as chamber_led_mod


def make_config(section, options):
    parser = configparser.ConfigParser()
    parser.add_section(section)
    for key, value in options.items():
        parser.set(section, key, str(value))
    return configfile.ConfigWrapper(None, parser, {}, section)


def test_color_option_missing_returns_none():
    config = make_config("chamber_led chamber", {})
    assert chamber_led_mod._parse_color_option(config, "color_printing") is None


def test_color_option_rgb_defaults_brightness_to_255():
    config = make_config("chamber_led chamber", {"color_printing": "10,20,30"})
    assert chamber_led_mod._parse_color_option(config, "color_printing") == (
        10,
        20,
        30,
        255,
    )


def test_color_option_rgb_brightness():
    config = make_config(
        "chamber_led chamber", {"color_complete": "0,0,255,128"}
    )
    assert chamber_led_mod._parse_color_option(config, "color_complete") == (
        0,
        0,
        255,
        128,
    )


def test_color_option_wrong_element_count_rejected():
    config = make_config("chamber_led chamber", {"color_error": "255,0"})
    with pytest.raises(configfile.error):
        chamber_led_mod._parse_color_option(config, "color_error")


def test_color_option_out_of_range_rejected():
    config = make_config("chamber_led chamber", {"color_error": "256,0,0"})
    with pytest.raises(configfile.error):
        chamber_led_mod._parse_color_option(config, "color_error")


def test_color_option_negative_rejected():
    config = make_config("chamber_led chamber", {"color_idle": "-1,0,0"})
    with pytest.raises(configfile.error):
        chamber_led_mod._parse_color_option(config, "color_idle")


def test_parse_status_reply_well_formed():
    reply = (
        "STATUS mode=solid r=10 g=20 b=30 brightness=255"
        " uptime_ms=123456 free_mem=65536"
    )
    status = chamber_led_mod.parse_status_reply(reply)
    assert status == {
        "mode": "solid",
        "r": 10,
        "g": 20,
        "b": 30,
        "brightness": 255,
        "uptime_ms": 123456,
        "free_mem": 65536,
    }


def test_parse_status_reply_missing_field_rejected():
    reply = "STATUS mode=off r=0 g=0 b=0 brightness=0 uptime_ms=1"
    assert chamber_led_mod.parse_status_reply(reply) is None


def test_parse_status_reply_wrong_prefix_rejected():
    assert chamber_led_mod.parse_status_reply("ERR bad command") is None


def test_parse_status_reply_non_integer_field_rejected():
    reply = (
        "STATUS mode=solid r=x g=20 b=30 brightness=255 uptime_ms=1 free_mem=1"
    )
    assert chamber_led_mod.parse_status_reply(reply) is None
