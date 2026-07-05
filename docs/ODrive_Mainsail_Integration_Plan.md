# ODrive – Mainsail GUI Integration Plan

This document plans how the ODrive support described in
`ODrive_Implementation_Spec.md` surfaces in the Mainsail web GUI. It is a
separate plan because Mainsail is a distinct upstream project
(`mainsail-crew/mainsail`) with its own release process, and because
useful browser-based configuration is achievable **before** any
Mainsail-side code exists at all — the entire Kalico-side command set is
designed to work from Mainsail's built-in gcode console from day one.

## Constraint: Mainsail has no runtime plugin system

Mainsail is a compiled Vue 2/3 + Vuetify single-page application, built
with Vite and shipped as a static bundle
(`mainsail-crew/mainsail`). There is no plugin loader, no
extension-point API, and no mechanism to inject a third-party panel into
a running Mainsail install at runtime. Every device-specific panel that
exists today in Mainsail (TMC driver status, Spoolman, timelapse,
exclude-object) got there by being **merged into the Mainsail
repository** as a normal PR — panels are not something a device's own
project distributes and users install independently.

This has two consequences for the plan:

1. A first-class "ODrive panel" requires a PR (realistically a short
   series of PRs, phased below) submitted to `mainsail-crew/mainsail`.
   Fallback if upstreaming doesn't happen in a useful timeframe: maintain
   a patched Mainsail build/fork, or ship the equivalent functionality as
   gcode macros plus Mainsail's generic "macro" UI — strictly worse UX
   but zero upstream dependency.
2. **Everything must work without the panel.** Because every ODrive
   action in the implementation spec is a plain gcode command and every
   piece of ODrive state is a plain Klipper status object, a user gets
   full functionality — connect, calibrate, tune, home, diagnose — from
   Mainsail's existing console and macro UI the moment the Kalico-side
   module lands, with zero Mainsail changes. The panel work below is
   additive polish, not a blocking dependency for the feature to be
   usable from a browser.

## Data flow

Mainsail never talks to Klipper directly; it talks to **Moonraker** over
a WebSocket JSON-RPC connection, and Moonraker relays to/from Klippy over
its own unix-socket API (`klippy/webhooks.py`). Three channels apply here,
all already specified by the "Status and webhooks surface" section of
the implementation spec:

1. **State — no Moonraker component needed.** Because
   `[odrive drive0]` and `[odrive_axis x_motor]` are ordinary Klipper
   printer objects with a `get_status(eventtime)` method, they are
   automatically visible to `printer.objects.list` and subscribable via
   `printer.objects.subscribe`, exactly like TMC driver objects or
   `exclude_object` today. Moonraker pushes `notify_status_update`
   diffs (only changed fields) roughly every 0.25 s (Klippy's
   subscription refresh interval) with no ODrive-specific Moonraker code
   required at all.
2. **Actions — plain gcode.** Every `ODRIVE_*` command is invoked via
   Moonraker's `printer.gcode.script` RPC, the same path Mainsail already
   uses for every other gcode-driven UI action (bed mesh calibration,
   PID tuning, etc.).
3. **High-rate telemetry — the bulk webhooks endpoint.** The
   `odrive/telemetry` endpoint (a `bulk_sensor.BatchBulkHelper` mux
   endpoint) is reached the same way Mainsail/Moonraker already reach
   `motion_report/dump_trapq` or ADXL345 accelerometer dumps: through
   Klippy's webhooks passthrough, which Moonraker exposes over its own
   websocket. No custom Moonraker component is required for this either
   — it is a straight pass-through of an existing Klippy mechanism.

**No `moonraker/components/odrive.py` is planned.** The precedent cases
that *do* need a custom Moonraker component — Spoolman, timelapse — need
one because they manage state or files that live outside Klipper
entirely (a spool database, video encoding). Nothing in the ODrive
design has that shape: all state lives in Klipper printer objects, and
all actions are gcode. A Moonraker component would only be justified
later if a future feature needs something a printer object/gcode pairing
genuinely cannot express (for example, persisting calibration
history across firmware reflashes) — not by anything in the current
implementation spec.

## Panel design (phased Mainsail PRs)

Two distinct surfaces, not one:

- **Dashboard card** — a compact but informative summary widget among
  the user's other dashboard panels: board name, one connection-state
  badge, a single line of firmware/hardware version and bus voltage, and
  one line per configured axis (name, a small state indicator, and an
  error-count badge if the axis has active errors). High-level and
  glanceable — a handful of short lines per board, not a redesign that
  makes the card sprawl — but enough that a quick look at the dashboard
  answers "is everything OK?" without a click-through. What it
  deliberately omits: per-axis numeric telemetry (position/velocity/
  current), decoded error *names* (just a count), gains, and
  temperatures — those, plus calibration/tuning/diagnostics, are what
  the ODrive page is for.
- **Dedicated ODrive page** — a permanent left-sidebar nav entry (not a
  dashboard-only, opt-in widget) that hosts everything with real detail:
  full per-axis status, calibration, tuning, and diagnostics. This is
  where Phase M1's full detail view and all of M2-M4 live.

This split exists because a dashboard is a multi-widget, glanceable
surface — cramming per-axis temperatures, error decode, and (eventually)
a calibration wizard into one card among several competes for space and
attention with everything else on the dashboard. A permanent nav page has
the room, and unlike an opt-in dashboard widget it's always there without
the user having to know to add it.

Each phase below is still an independently mergeable Mainsail PR,
mirroring how Mainsail actually shipped past device-specific features
(TMC status and Spoolman integration landed as separate, incremental PRs
rather than one large change).

### Phase M1 — Status surfaces (shipped)

Split across two PRs:

- The **dashboard card** shows a per-board summary: board name, a
  single color-coded connection-state badge (`ready` / `configuring` /
  `error` / `disconnected`, etc.), a compact fw/hw version + bus-voltage
  line, and one line per axis (name, small state indicator, error-count
  badge if nonzero). No per-axis numeric telemetry, no decoded error
  names, no temperatures, no gains — those belong on the ODrive page.
  (The dashboard card briefly shipped with full per-axis detail, then
  with *only* name+badge before settling here — this level balances
  "detailed enough to be useful at a glance" against "doesn't sprawl.")
- The **ODrive page** (`/odrive` in the left nav) shows the full detail
  previously crammed into the dashboard card: per-board connection state,
  firmware/hardware version, bus voltage, per-axis state (idle /
  closed-loop / error) with color-coded badges, decoded error names (not
  raw bitmasks — the Kalico module already does this decoding
  server-side via `ODRIVE_ERRORS`/`get_status`), and temperature
  readouts.

**Visibility**: both surfaces are gated on the presence of `odrive*`
objects in `printer.objects.list` — the same feature-detection mechanism
Mainsail already uses to decide whether to show TMC panels, the
exclude-object UI, or the Spoolman card. No configuration flag is needed
on the Mainsail side; neither surface renders for printers without an
`[odrive]` section.

**Data source**: `printer.objects.subscribe` only, for both surfaces. No
new Moonraker endpoints.

### Phase M2 — Calibration wizard (on the ODrive page)

A guided, stepper-style UI, added to the ODrive page rather than the
dashboard card, that drives `ODRIVE_CALIBRATE` and displays its live
progress. Because the wizard's entire state machine already exists
server-side (the Kalico module emits progress via `respond_info` and
exposes calibration flags — `calibrated`, `pre_calibrated_motor`,
`pre_calibrated_encoder`, `index_found` — via `get_status`), the Mainsail
side is primarily a thin state-reflecting UI: a "Start Calibration"
button issues `ODRIVE_CALIBRATE AXIS=... TYPE=full`, and the wizard's
displayed step advances purely by watching the subscribed status object
change, with the raw gcode console output available as a fallback detail
view for anyone who wants it.

On successful calibration, this phase should reuse Mainsail's **existing
`save_config_pending` UI** (the same banner/prompt shown after
`PROBE_CALIBRATE` or any other `SAVE_CONFIG`-eligible action) rather than
inventing a new save-prompt component — the Kalico module already stages
the calibration breadcrumb through `configfile.set` for exactly this
reason.

### Phase M3 — Tuning view (on the ODrive page)

Live-adjustable controls (sliders/number inputs), added to the ODrive
page, for `pos_gain`, `vel_gain`, `vel_integrator_gain`,
`filter_bandwidth`, `current_lim`, and `vel_limit`, each issuing
`ODRIVE_TUNE AXIS=... <FIELD>=<value>` on change (debounced), with a
`SAVE=1` action wired to a "persist" button that also surfaces the same
`save_config_pending` banner as M2.

This phase also adds a live scope/chart (reusing Mainsail's existing
temperature-graph charting components, which already handle rolling
time-series display) fed by the `odrive/telemetry` webhooks endpoint,
plotting commanded position vs. estimated position and measured current
— useful both for tuning feel and for diagnosing a following-error
shutdown after the fact.

### Phase M4 — Diagnostics (on the ODrive page)

An error-history view and a raw property browser, added to the ODrive
page (`ODRIVE_READ` issued per row, populated in bulk via the
`odrive/property_read` endpoint to avoid one gcode round-trip per field),
aimed at advanced users and at clone boards where something in the
firmware-tolerance layer needed to guess — the property browser gives a
way to manually verify a value the automatic path couldn't confirm.

**Possible future extension (not yet scoped as its own phase):** an
in-browser Python REPL on the ODrive page for ad hoc diagnostics —
useful for advanced users who want to script against `odrivetool`-style
property access or prototype a check before it's worth turning into a
proper `ODRIVE_*` gcode command. This would need its own design pass
(most plausibly a `klippy`-side webhooks endpoint exposing a sandboxed
eval loop, surfaced through a terminal-style Mainsail component) rather
than being folded into the property browser above.

## Upstreaming requirements

Each phase, submitted as a Mainsail PR, needs:

- A Vuex (or Pinia, depending on Mainsail's state-management version at
  time of implementation) store module under `src/store/printer/...`
  mirroring the shape of the `get_status` dictionaries specified in the
  implementation spec, so the store layer needs no bespoke parsing logic
  beyond what Mainsail's existing generic printer-object subscription
  handling already provides.
- English-locale i18n strings for every new label, consistent with
  Mainsail's existing localization file structure (translations for
  other languages are a community process the PR does not need to
  include).
- Standard Vuetify components matching Mainsail's established visual
  language (cards, chips for status badges, sliders for gain fields) —
  no new design system elements.
- Feature-flag gating on printer-object presence (Phase M1), matching
  the pattern already used by every other optional Mainsail feature, so
  the panel is inert and invisible for any printer without an
  `[odrive]` section.

## Fallback if upstreaming stalls

Because the entire feature works from the console and macros
independently of any Mainsail change (see "Constraint" above), a stalled
or rejected upstream PR does not block usability — it only blocks the
convenience layer. If upstreaming proves impractical, the fallback is
either (a) a maintained fork of Mainsail carrying the panel PRs
indefinitely, rebased periodically against upstream, or (b) a
console-macro-based UX: a handful of `gcode_macro`-defined shortcut
commands plus Mainsail's built-in macro panel, giving one-click access to
the most common actions (connect, calibrate, arm/disarm, clear errors)
without any custom UI code at all.
