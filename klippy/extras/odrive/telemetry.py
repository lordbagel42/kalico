# ODrive v3.6 webhooks telemetry/property surface
#
# See docs/ODrive_Implementation_Spec.md, "Status and webhooks
# surface". Registers a per-axis telemetry endpoint and a per-board
# raw-property-read endpoint so a future Mainsail panel can populate
# live readouts/forms without a gcode round-trip per field. This is a
# pull-style snapshot rather than the push-style bulk_sensor.
# BatchBulkHelper stream described in the spec -- a scope-reduced first
# cut; per-axis get_status() (polled every 0.25s by Moonraker's
# printer.objects.subscribe) already covers routine status display.
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.


def register_axis_endpoint(axis):
    wh = axis.printer.lookup_object("webhooks")
    wh.register_mux_endpoint(
        "odrive/telemetry", "axis", axis.name, axis_telemetry_handler(axis)
    )


def axis_telemetry_handler(axis):
    def handler(web_request):
        axis.poll_errors_and_telemetry()
        web_request.send(axis.get_status())

    return handler


def register_board_endpoint(board):
    wh = board.printer.lookup_object("webhooks")
    wh.register_mux_endpoint(
        "odrive/property_read",
        "odrive",
        board.name,
        property_read_handler(board),
    )


def property_read_handler(board):
    def handler(web_request):
        prop = web_request.get_str("property")
        if not board.connected:
            web_request.send(
                {"property": prop, "value": None, "connected": False}
            )
            return
        value = board.transport.read_property_sync(prop, timeout=2.0)
        web_request.send({"property": prop, "value": value, "connected": True})

    return handler
