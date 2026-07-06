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


class OdrivePropertyProxy:
    """Attribute-chain proxy giving the REPL odrivetool-style ergonomics.

    Lets a REPL user write a bare dotted attribute chain --
    ``odrv0.axis0.motor.config.pole_pairs`` to read,
    ``odrv0.axis0.motor.config.pole_pairs = 8`` to write -- instead of
    the quoted-string ``read('axis0.motor.config.pole_pairs')`` /
    ``write('axis0.motor.config.pole_pairs', 8)`` forms this module's
    namespace also binds (see ``_build_namespace``; both forms remain
    available side by side).

    Each ``.foo`` access (``__getattr__``) returns a *brand new* child
    proxy with ``foo`` appended to the accumulated dotted path -- it
    does not talk to the device. Nothing is sent over the wire until
    the chain is actually resolved: a bare expression statement at the
    REPL prompt (whose ``repr()`` is what ``code.InteractiveConsole``
    prints) triggers ``__repr__``, which issues a real
    ``read_property_sync``; assigning to the chain triggers
    ``__setattr__``, which issues a real ``write_property_sync``.

    Internal state (``_transport``, ``_path``) is kept in
    ``__slots__`` and set via ``object.__setattr__`` in ``__init__`` so
    it is never mistaken for a property name by the overridden
    ``__getattr__``/``__setattr__`` below -- the classic gotcha with
    this pattern is either infinite recursion (an internal attribute
    access re-entering the override) or, worse, silently issuing a
    device write for what was supposed to be local bookkeeping.

    Two real limitations of this proxy, deliberate and NOT bugs:

    1. Read/write-only -- never function calls. The ASCII wire
       protocol this module speaks (see
       docs/ODrive_Implementation_Spec.md, "Protocol choice: ASCII
       over USB-CDC") supports exactly two operations, ``r <property>``
       and ``w <property> <value>``; there is no ASCII equivalent of a
       Fibre/native-protocol RPC. The native protocol that *would*
       support arbitrary calls was deliberately rejected elsewhere in
       this codebase for firmware-version-tolerance reasons -- see
       that same doc section. Consequently things a real odrivetool
       user might try, like ``odrv0.reboot()`` or
       ``odrv0.axis0.motor.calibrate()``, do not work through this
       proxy: a proxy attribute is itself just another
       ``OdrivePropertyProxy`` instance, not a bound method, so calling
       it raises a plain, honest Python ``TypeError`` ("... object is
       not callable"). That equivalent functionality already exists as
       gcode commands elsewhere (``ODRIVE_REBOOT``, ``ODRIVE_CALIBRATE``).
    2. No recursive subtree dump. Real odrivetool can pretty-print a
       whole nested subtree when you evaluate an intermediate node
       alone (e.g. ``odrv0.axis0.motor``) because it downloads the
       device's full JSON property-tree schema at connect time. This
       module deliberately never does that (same firmware-tolerance
       reasoning as above), so evaluating an intermediate node here
       just attempts a raw property read of that (non-leaf) path and
       will typically get back the device's own "invalid property"
       error string -- exactly what ``read()`` already returns for the
       same case today. That's expected, not a bug.
    """

    __slots__ = ("_transport", "_path")

    def __init__(self, transport, path=""):
        object.__setattr__(self, "_transport", transport)
        object.__setattr__(self, "_path", path)

    def _child_path(self, name):
        return "%s.%s" % (self._path, name) if self._path else name

    def __getattr__(self, name):
        # Only called when normal (slot-based) lookup already failed,
        # so _transport/_path themselves never reach this branch.
        if name.startswith("_"):
            raise AttributeError(name)
        return OdrivePropertyProxy(self._transport, self._child_path(name))

    def __setattr__(self, name, value):
        if name.startswith("_"):
            object.__setattr__(self, name, value)
            return
        self._transport.write_property_sync(
            self._child_path(name), value, verify=True, timeout=2.0
        )

    def __repr__(self):
        return repr(self._transport.read_property_sync(self._path, timeout=2.0))


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
            # odrivetool-style bare attribute-chain access, e.g.
            # `odrv0.axis0.motor.config.pole_pairs` to read or
            # `odrv0.axis0.motor.config.pole_pairs = 8` to write --
            # see OdrivePropertyProxy's docstring for exactly what this
            # can and can't do. `odrv0.axis0`/`odrv0.axis1` need no
            # special-casing here: they fall out of plain attribute
            # chaining through __getattr__ below.
            "odrv0": OdrivePropertyProxy(board.transport),
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
