# ODrive v3.6 in-browser Python REPL
#
# See docs/ODrive_Mainsail_Integration_Plan.md, "Phase M5 -- In-browser
# Python REPL", for the full design rationale. Deliberately
# unsandboxed: attempting to sandbox arbitrary Python execution is a
# well-known losing battle, and Kalico already runs this exact trust
# model elsewhere -- klippy/extras/gcode_macro.py calls raw exec() on
# macro bodies with no sandbox. Whoever can edit printer.cfg or reach
# Moonraker's webhooks API can already run arbitrary Python in the
# Klippy process; this module doesn't introduce a new trust boundary,
# it just exposes the one that already exists through a more
# convenient interface. Treat it exactly like SSH access to the host.
#
# Unlike every other ODrive webhooks endpoint, this one is deliberately
# NOT gcode-only -- see the "Deliberate scope note" in the Mainsail
# integration plan for why a write() call issued from here doesn't show
# up in the gcode console history the way ODRIVE_WRITE would.
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import code
import contextlib
import io


class BoardRepl:
    # A real code.InteractiveConsole rather than a hand-rolled
    # expression parser, so multi-line-statement buffering,
    # expression-vs-statement handling, and traceback formatting are
    # the same machinery behind the standard python3 REPL -- not
    # something this module has to get right itself. One instance lives
    # for the lifetime of the board's console (see board._repl below);
    # state persists across push() calls exactly like a real shell.
    def __init__(self, board):
        self.board = board
        self.console = code.InteractiveConsole(self._build_namespace())

    def _build_namespace(self):
        board = self.board
        axes = {axis.name: axis for axis in board.axes.values()}
        namespace = {
            "board": board,
            "axes": axes,
            "read": lambda prop: board.transport.read_property_sync(
                prop, timeout=2.0
            ),
            "write": lambda prop, value: board.transport.write_property_sync(
                prop, value, verify=True, timeout=2.0
            ),
            "printer": board.printer,
        }
        # axis0/axis1 shortcuts mirror odrivetool's odrv0.axis0
        # ergonomics; axis1 is only bound if a second axis is actually
        # configured on this board.
        for axis in axes.values():
            if axis.axis_index == 0:
                namespace["axis0"] = axis
            elif axis.axis_index == 1:
                namespace["axis1"] = axis
        return namespace

    def push(self, line):
        # Both stdout and stderr redirect into the same buffer so
        # ordinary print() output and traceback output interleave in
        # the order they actually occurred, exactly as they would in a
        # real terminal -- capturing only InteractiveInterpreter's own
        # traceback writes (e.g. by overriding .write()) would silently
        # drop anything the executed code printed itself.
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            more = self.console.push(line)
        return buf.getvalue(), more


def _get_repl(board):
    # Lazily created on first use and kept alive across requests so
    # the namespace (and any variables the user assigned) persists
    # between REPL lines, like a real shell.
    if board._repl is None:
        board._repl = BoardRepl(board)
    return board._repl


def register_board_endpoint(board):
    board._repl = None
    wh = board.printer.lookup_object("webhooks")
    wh.register_mux_endpoint(
        "odrive/repl_exec", "odrive", board.name, repl_exec_handler(board)
    )
    wh.register_mux_endpoint(
        "odrive/repl_reset", "odrive", board.name, repl_reset_handler(board)
    )


def repl_exec_handler(board):
    def handler(web_request):
        line = web_request.get_str("line")
        repl = _get_repl(board)
        output, more = repl.push(line)
        web_request.send({"output": output, "more": more})

    return handler


def repl_reset_handler(board):
    def handler(web_request):
        # Discard the console/namespace; the next repl_exec call
        # lazily creates a fresh one via _get_repl().
        board._repl = None
        web_request.send({})

    return handler
