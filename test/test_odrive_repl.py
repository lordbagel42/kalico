"""Behavior tests for klippy/extras/odrive/repl.py (Phase M5 REPL).

Pure logic tests, independent of the reactor/serial machinery, per this
module's "Device-less test coverage" testing philosophy (see
docs/ODrive_Implementation_Spec.md). Fakes stand in for ODriveBoard,
Printer, and webhooks so these run with no live ODrive and no full
Klippy printer object graph.
"""

from __future__ import annotations

from klippy.extras.odrive import repl


class FakeTransport:
    def __init__(self, connected=True):
        self.connected = connected
        self.reads = []
        self.writes = []

    def read_property_sync(self, path, timeout=2.0):
        self.reads.append((path, timeout))
        if not self.connected:
            return None
        return "42"

    def write_property_sync(self, path, value, verify=False, timeout=2.0):
        self.writes.append((path, value, verify, timeout))
        return self.connected


class FakeAxis:
    def __init__(self, name, axis_index):
        self.name = name
        self.axis_index = axis_index


class FakeWebhooks:
    def __init__(self):
        self.endpoints = {}

    def register_mux_endpoint(self, path, key, value, callback):
        self.endpoints[(path, key, value)] = callback


class FakePrinter:
    def __init__(self):
        self.webhooks = FakeWebhooks()

    def lookup_object(self, name):
        assert name == "webhooks"
        return self.webhooks


class FakeBoard:
    def __init__(self, name="drive0", axes=None, connected=True):
        self.name = name
        self.printer = FakePrinter()
        self.transport = FakeTransport(connected=connected)
        self.axes = axes or {}
        self.connected = connected


class FakeWebRequest:
    def __init__(self, params=None):
        self.params = params or {}
        self.response = None

    def get_str(self, item, default=None):
        return self.params.get(item, default)

    def send(self, data):
        self.response = data


def _exec_endpoint(board):
    return board.printer.webhooks.endpoints[
        ("odrive/repl_exec", "odrive", board.name)
    ]


def _reset_endpoint(board):
    return board.printer.webhooks.endpoints[
        ("odrive/repl_reset", "odrive", board.name)
    ]


def _push(board, line):
    handler = _exec_endpoint(board)
    web_request = FakeWebRequest({"line": line})
    handler(web_request)
    return web_request.response


def test_register_board_endpoint_registers_both_mux_endpoints():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    assert board._repl is None
    endpoints = board.printer.webhooks.endpoints
    assert ("odrive/repl_exec", "odrive", "drive0") in endpoints
    assert ("odrive/repl_reset", "odrive", "drive0") in endpoints


def test_repl_exec_evaluates_expression():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    response = _push(board, "1+1")

    assert response == {"output": "2\n", "more": False}


def test_repl_exec_incomplete_statement_requests_continuation():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    response = _push(board, "if True:")

    assert response == {"output": "", "more": True}


def test_repl_exec_multiline_block_completes_on_blank_line():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    assert _push(board, "if True:")["more"] is True
    assert _push(board, "    x = 3")["more"] is True
    response = _push(board, "")

    assert response["more"] is False
    # The namespace mutation from the completed block is visible on the
    # next line, proving the console's locals persist across push()
    # calls exactly like a real shell.
    assert _push(board, "x") == {"output": "3\n", "more": False}


def test_repl_exec_captures_print_output():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    response = _push(board, "print('hello')")

    assert response == {"output": "hello\n", "more": False}


def test_repl_exec_captures_traceback_output_via_redirect():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    response = _push(board, "undefined_name")

    assert response["more"] is False
    assert "NameError" in response["output"]
    assert "undefined_name" in response["output"]


def test_repl_exec_namespace_exposes_board_axes_and_printer():
    axis0 = FakeAxis("x_motor", 0)
    board = FakeBoard(axes={0: axis0})
    repl.register_board_endpoint(board)

    assert _push(board, "print(board.name)") == {
        "output": "drive0\n",
        "more": False,
    }
    assert _push(board, "print(sorted(axes))") == {
        "output": "['x_motor']\n",
        "more": False,
    }
    assert _push(board, "print(axis0 is axes['x_motor'])") == {
        "output": "True\n",
        "more": False,
    }
    assert _push(board, "print(printer is board.printer)") == {
        "output": "True\n",
        "more": False,
    }


def test_repl_exec_only_binds_axis1_when_second_axis_present():
    axis0 = FakeAxis("x_motor", 0)
    board = FakeBoard(axes={0: axis0})
    repl.register_board_endpoint(board)

    response = _push(board, "axis1")

    assert response["more"] is False
    assert "NameError" in response["output"]


def test_repl_exec_binds_axis1_when_second_axis_configured():
    axis0 = FakeAxis("x_motor", 0)
    axis1 = FakeAxis("y_motor", 1)
    board = FakeBoard(axes={0: axis0, 1: axis1})
    repl.register_board_endpoint(board)

    response = _push(board, "print(axis1 is axes['y_motor'])")

    assert response == {"output": "True\n", "more": False}


def test_repl_exec_read_and_write_call_transport_with_expected_args():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    read_response = _push(board, "print(read('axis0.pos_estimate'))")
    assert read_response == {"output": "42\n", "more": False}
    assert board.transport.reads == [("axis0.pos_estimate", 2.0)]

    write_response = _push(board, "print(write('axis0.pos_estimate', 1.0))")
    assert write_response == {"output": "True\n", "more": False}
    assert board.transport.writes == [("axis0.pos_estimate", 1.0, True, 2.0)]


def test_repl_exec_state_persists_across_calls():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    assert _push(board, "x = 5") == {"output": "", "more": False}
    assert _push(board, "x") == {"output": "5\n", "more": False}


def test_repl_reset_discards_console_and_namespace():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    _push(board, "x = 5")
    old_console = board._repl

    _reset_endpoint(board)(FakeWebRequest())

    assert board._repl is None
    response = _push(board, "x")
    assert "NameError" in response["output"]
    assert board._repl is not old_console


def test_repl_exec_odrv0_bare_read_returns_repr_of_transport_value():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    response = _push(board, "odrv0.axis0.motor.config.pole_pairs")

    assert response == {"output": "'42'\n", "more": False}
    assert board.transport.reads == [("axis0.motor.config.pole_pairs", 2.0)]


def test_repl_exec_odrv0_assignment_calls_write_property_sync():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    response = _push(board, "odrv0.axis0.motor.config.pole_pairs = 8")

    assert response == {"output": "", "more": False}
    assert board.transport.writes == [
        ("axis0.motor.config.pole_pairs", 8, True, 2.0)
    ]


def test_repl_exec_odrv0_chaining_builds_expected_dotted_path():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    _push(board, "odrv0.axis0.motor.config.pole_pairs")
    _push(board, "odrv0.vbus_voltage")
    _push(board, "odrv0.axis1.encoder.config.cpr")

    assert board.transport.reads == [
        ("axis0.motor.config.pole_pairs", 2.0),
        ("vbus_voltage", 2.0),
        ("axis1.encoder.config.cpr", 2.0),
    ]


def test_repl_exec_odrv0_internal_attributes_do_not_leak_into_device_path():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    proxy = repl.OdrivePropertyProxy(board.transport)
    child = proxy.axis0

    # Internal bookkeeping attributes are real Python attributes on the
    # proxy (via __slots__/object.__setattr__), not device paths --
    # accessing them must not touch the transport at all.
    assert child._path == "axis0"
    assert child._transport is board.transport
    assert board.transport.reads == []
    assert board.transport.writes == []


def test_repl_exec_odrv0_axis0_shortcut_works_without_special_casing():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    # odrv0.axis0 is not special-cased anywhere in repl.py; it falls
    # out of plain __getattr__ chaining, same as any other attribute.
    response = _push(board, "odrv0.axis0.pos_estimate")

    assert response == {"output": "'42'\n", "more": False}
    assert board.transport.reads == [("axis0.pos_estimate", 2.0)]


def test_repl_exec_odrv0_calling_a_proxy_raises_type_error():
    board = FakeBoard()
    repl.register_board_endpoint(board)

    # The ASCII protocol only supports property read/write, never
    # function calls -- attempting to call a proxy attribute (e.g. the
    # odrivetool-style odrv0.reboot()) must fail with a plain, honest
    # TypeError, not something silently wrong.
    response = _push(board, "odrv0.reboot()")

    assert response["more"] is False
    assert "TypeError" in response["output"]
    assert "not callable" in response["output"]
    # No transport traffic: the call fails before any read/write.
    assert board.transport.reads == []
    assert board.transport.writes == []


def test_repl_exec_odrv0_intermediate_node_read_returns_transport_result():
    # No recursive subtree dump (documented limitation): evaluating an
    # intermediate node just does a raw read of that path and returns
    # exactly what the transport gives back -- same as read() already
    # does for a non-leaf/invalid path.
    board = FakeBoard()
    repl.register_board_endpoint(board)

    response = _push(board, "odrv0.axis0.motor")

    assert response == {"output": "'42'\n", "more": False}
    assert board.transport.reads == [("axis0.motor", 2.0)]


def test_repl_works_while_board_disconnected():
    # Plain Python introspection must keep working even without a live
    # device -- only read()/write() calls should fail, and only because
    # the underlying transport methods naturally return falsy/None when
    # disconnected, not because repl.py special-cases it.
    board = FakeBoard(connected=False)
    repl.register_board_endpoint(board)

    response = _push(board, "print(board.connected)")
    assert response == {"output": "False\n", "more": False}

    response = _push(board, "print(read('axis0.pos_estimate'))")
    assert response == {"output": "None\n", "more": False}

    response = _push(board, "odrv0.axis0.pos_estimate")
    assert response == {"output": "None\n", "more": False}
