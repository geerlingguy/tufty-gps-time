# gps_time - GPS-synced clock for Badgeware (Tufty 2350)
#
# Shows UTC (large, ISO-style with trailing "Z") and local time below it,
# sourced from the internal RTC (kept accurate by periodic syncs from an
# Adafruit PA1010D GPS module over QWIIC/I2C). Owner name is shown just
# above the UTC clock, same size/style as the local time line, with a
# small WOPR-style scanner block bouncing left-right between the name and
# the UTC clock, phase-locked to the displayed second (WarGames nod).
#
# Top-left   : fix status - "NO FIX" (red), "2D FIX" (orange), or "3D FIX"
#              (green), always shown (same logic as GPS Info 1)
# Top-middle : "N sats" - satellites in view (GGA); colour matches the
#              fix status label at top-left, not a sat-count threshold
# Top-right  : battery gauge (level + charging indicator)
# UTC clock  : orange = no fix, yellow = fix but RTC not yet synced,
#              white = fix and RTC synced at least once
#
# Four screens, paged with UP/DOWN (small arrow icons hint at direction).
# "info1/2/3" below are positions, not on-screen titles - the on-screen
# titles are "Sky View", "GPS Info 1", and "GPS Info 2" respectively.
#   clock -> DOWN -> info1: sky plot of satellites in view, from GSV ("Sky View")
#   info1 -> DOWN -> info2: fix status, sats, fix quality, HDOP, altitude ("GPS Info 1")
#   info2 -> DOWN -> info3: RTC sync, GGA/RMC seen, lat/lon, light level ("GPS Info 2")
#   info1 -> UP   -> clock
#   info2 -> UP   -> info1
#   info3 -> UP   -> info2
#
# A fifth screen, Settings, is reached with button B from the clock (B
# again to return). C cycles which field UP/DOWN adjusts:
#   - Local UTC offset, 0.5h steps, clamped to -12.0..+14.0 (real-world
#     range). Starts from the LOCAL_OFFSET constant but is runtime-only -
#     it resets to that default on every boot, it isn't saved to flash.
#   - Time format, 12H (with AM/PM) or 24H, for the local time line only -
#     the UTC line is always 24H/ISO-style.
#   - GPS I2C Address, cycles gps_i2c_addr between PA1010D (0x10) and
#     u-blox (0x42) - runtime-only like the other two fields. Switching to
#     0x42 sends a UBX-CFG-VALSET enabling NMEA output on that module (see
#     _configure_gps_ubx()) - confirmed working with a MAX-M10S over Qwiic.
#
# Button A on the clock screen syncs the RTC from GPS (if we have a fix)
# and returns to the badge's home menu via machine.reset() - see _go_home()
# for why a full reset rather than something more graceful.
#
# Bottom row of the clock screen: home icon (A, left), cog icon (B,
# middle), down arrow (info page, right) - home.png/cog.png if they
# loaded, hand-drawn fallbacks otherwise. Both PNGs must be copied onto
# the badge alongside __init__.py, not just the script itself.
#
# Rear case LEDs (mono/white only - not RGB): a held-then-fading flash on
# the NO FIX -> fix transition. Uses the documented badge.caselights(level)
# call.
#
# Ambient dimming: badge.light_level() drives real backlight brightness
# via set_brightness(), smoothly ramped over AMBIENT_TRANSITION_MS, with
# 3 levels and hysteresis so it doesn't flicker between levels right at a
# threshold.
#
# Sky View: centre = zenith, edge = horizon, azimuth clockwise from N at
# top - a standard polar sky-view chart. Circles are GPS satellites,
# triangles are other constellations (e.g. GLONASS). Green = used in the
# current fix solution (from GSA), grey = visible but unused.
#
# GPS Info 1: fix status shown as NO FIX / 2D / 3D (from GSA mode2,
# combined with the same fix-valid/timeout check as the top-left warning).
# HDOP colour-coded green (<=2), orange (2-5), red (>5).
#
# Location logging: appends timestamp/lat/lon/fix quality/HDOP/sats in
# use/altitude to a CSV. A reading is sampled every 5s
# (GPS_LOG_SAMPLE_INTERVAL) into a RAM buffer, but that buffer is only
# written to flash every 60s (GPS_LOG_WRITE_INTERVAL) as one batched write
# - see _maybe_log_gps(). A new file is started fresh each power-on -
# gps_log_0001.csv, gps_log_0002.csv, etc. (see _next_gps_log_path()) -
# rather than one file capped/rotated over time, on the assumption the
# badge gets powered off nightly. Written under GPS_LOG_DIR = "/state" -
# per Pimoroni's forum-documented State API
# (forums.pimoroni.com/t/is-the-tufty-file-system-really-read-only/28743/4),
# an app's own /apps/<name> directory is not writable at runtime; /state is
# the one location regular open()/write() calls are guaranteed to work
# against. (Two earlier attempts - a relative filename, then a hardcoded
# /apps/gps_time path - both silently failed against that read-only area.)
# _go_home() also flushes any buffered-but-unwritten rows before
# resetting, since that's the one "leaving the app" moment the code gets
# to catch.

import machine
import time
import math
import os

# Real backlight control - found via a Pimoroni forum post, not documented
# in badge.md, so its exact value range/behaviour is unconfirmed. Guarded
# so a missing/renamed function on this firmware build doesn't crash the
# whole app at import time - see AMBIENT_BRIGHTNESS_LEVELS below for how
# it's used; if this fails, dimming is just skipped entirely.
try:
    from badgeware import set_brightness
except ImportError:
    set_brightness = None

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Hours to add to UTC to get local time, e.g. -7 for US Pacific (PDT).
# Half-hour offsets (e.g. 5.5) are fine too. This is only the *startup
# default* - it's adjustable at runtime from the Settings screen (see
# local_offset below), and resets back to this value on every boot rather
# than persisting, since there's no flash/EEPROM storage wired up for it.
LOCAL_OFFSET = -7

# Displayed just above the UTC clock, same size as the local time line.
OWNER_NAME = "Jeff Geerling"

# How often (in seconds) to push GPS time into the RTC once we have a fix.
# Frequent enough that RTC crystal drift between syncs stays negligible
# (a few ms even for a mediocre ~20-50ppm crystal over 5 minutes, vs. up to
# ~180ms/hour if left the old default of once an hour), without writing to
# the RTC constantly.
RTC_SYNC_INTERVAL = 300  # once every 5 minutes

# How long (seconds) to wait after first acquiring a fix before trusting it
# enough for the *first* RTC sync of this session. A freshly-acquired fix
# can still be settling (time/position solution refining as more satellites
# lock in) - this avoids syncing the RTC to that transient. Only applies to
# the first sync; every sync after that follows RTC_SYNC_INTERVAL normally.
FIRST_SYNC_DELAY = 20

# How long (seconds) we'll keep showing a fix as "valid" after the last
# good RMC sentence before falling back to "NO FIX".
GPS_FIX_TIMEOUT = 10

# Rear case LEDs: mono/white on Tufty ("four onboard white LEDs on the
# back of the board" per badge.md), not RGB, so no colour is possible here
# regardless of API. Controlled via the documented badge.caselights(level)
# call (level is a float 0-1, applied to all four LEDs at once) - this
# replaced an earlier attempt using machine.PWM() on a guessed GPIO pin,
# which hung the whole app on launch. caselights() is Badgeware's own
# documented API for exactly this, so it doesn't carry that same risk.
LEDS_ENABLED = True

# Ambient-light dimming, via badge.light_level() (Tufty only - a raw u16
# from the front-mounted light sensor) and real backlight control via
# set_brightness() (see the import above) - found via a Pimoroni forum
# post, not in badge.md, so its value range is a guess (0.0-1.0, matching
# caselights()'s convention elsewhere in this same API). If set_brightness
# isn't available or throws, dimming is simply skipped rather than falling
# back to anything on-screen.
#
# Three levels (not just on/off), each transition using a *different*
# threshold depending on direction - e.g. light has to climb back above
# AMBIENT_DIM_EXIT (not just above AMBIENT_DIM_ENTER) to leave DIM for
# BRIGHT. That gap is the hysteresis: it stops the level flickering back
# and forth when ambient light sits right at one boundary.
#
# The raw sensor range is unconfirmed/untested - these thresholds are a
# starting point. GPS Info 2 could show the live light_level() reading if
# you want to calibrate against your actual environment; ask if useful.
AMBIENT_DIM_ENTER = 400     # BRIGHT -> DIM when light_level() drops below this
AMBIENT_DIM_EXIT = 600      # DIM -> BRIGHT when light_level() rises above this
AMBIENT_DARK_ENTER = 150    # DIM -> DARK when light_level() drops below this
AMBIENT_DARK_EXIT = 200     # DARK -> DIM when light_level() rises above this

# Real backlight brightness per level (0.0-1.0, unconfirmed range - see
# comment above).
AMBIENT_BRIGHTNESS_LEVELS = {"bright": 1.0, "dim": 0.75, "dark": 0.5}

# Default GPS module I2C address at boot - PA1010D is 0x10. Adjustable at
# runtime from Settings (gps_i2c_addr below) to swap modules, e.g. a
# u-blox module at 0x42, without reflashing. Resets to this default on
# every boot, like local_offset/time_format_24h.
GPS_I2C_ADDR = 0x10

# Addresses cycled through by the Settings screen's "GPS I2C Address"
# field: PA1010D (0x10), then u-blox (0x42 - MAX-M10S and similar).
_GPS_I2C_ADDR_CHOICES = (0x10, 0x42)

# Location logging: appends a row of lat/lon/fix/HDOP/sats/altitude to a
# CSV file. Sampling and writing are decoupled: a GPS reading is taken
# every GPS_LOG_SAMPLE_INTERVAL seconds and held in RAM, but rows are only
# flushed to flash every GPS_LOG_WRITE_INTERVAL seconds (as one batched
# write of everything sampled since the last flush) - this keeps the log's
# time resolution fine while writing to flash far less often. A fresh file
# is started each time the badge powers on (see _next_gps_log_path())
# rather than one ever-growing log, so a day's outing is its own file -
# GPS_LOG_PREFIX/GPS_LOG_SUFFIX name each one gps_log_0001.csv,
# gps_log_0002.csv, etc., picking up wherever the highest existing number
# left off. GPS_LOG_DIR is "/state" - the Tufty's app data directories
# (e.g. /apps/gps_time, where this file lives) aren't writable at runtime;
# /state is the documented writable area for exactly this kind of thing
# (see forums.pimoroni.com/t/is-the-tufty-file-system-really-read-only/
# 28743/4). Two earlier attempts here - a relative filename, then a
# hardcoded /apps/gps_time path - both silently failed for that reason.
GPS_LOG_DIR = "/state"
GPS_LOG_PREFIX = "gps_log_"
GPS_LOG_SUFFIX = ".csv"
GPS_LOG_SAMPLE_INTERVAL = 5   # seconds between readings taken (in RAM)
GPS_LOG_WRITE_INTERVAL = 60   # seconds between batched writes to flash

# QWIIC/I2C bus setup. Pimoroni's most common Qw/ST pin mapping is GP4/GP5,
# but this isn't documented for Tufty 2350 specifically - if the GPS never
# shows a fix, check the startup console output (it prints an i2c.scan())
# and adjust these to match your board.
I2C_ID = 0
I2C_SDA_PIN = 4
I2C_SCL_PIN = 5
I2C_FREQ = 100000

# Cap on 32-byte reads drained per _poll_gps() call. One second of GGA+RMC
# output is roughly 140 bytes (~5 chunks); this gives headroom to catch up
# after a slow frame without risking an unbounded read loop on a bad one.
_I2C_MAX_READS_PER_POLL = 12

# If we see this many consecutive I2C read errors (e.g. from hot-plugging
# the GPS module), attempt a bus recovery/reinit rather than spinning
# forever on a wedged bus. See _i2c_recover() for what this can and can't
# fix - it helps with a slave holding SDA low, not a true blocking hang.
_I2C_ERROR_RECOVERY_THRESHOLD = 5
_I2C_RECOVER_COOLDOWN = 5000  # ms between recovery attempts

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

_i2c = None
_nmea_buffer = ""

rtc_available = False

gps_num_sats = 0
gps_fix_valid = False
gps_last_fix_ticks = None
gps_datetime = None          # (year, month, day, hour, minute, second, dow)
gps_last_sync_ticks = None
gps_hdop = None               # horizontal dilution of precision, from GGA
gps_altitude = None           # metres above sea level, from GGA
gps_fix_quality = 0           # GGA fix quality: 0=none, 1=GPS, 2=DGPS, ...
gps_fix_type = 1              # GSA mode2: 1=no fix, 2=2D fix, 3=3D fix
gps_lat = None                 # decimal degrees, +N/-S, best of GGA/RMC (see gps_lat/lon note below)
gps_lon = None                 # decimal degrees, +E/-W, best of GGA/RMC

# Some MTK-based receivers (incl. the PA1010D) emit lat/lon with more
# fractional digits in one sentence type than the other - which one varies
# by firmware, so rather than hardcoding a choice, _handle_nmea_sentence()
# parses lat/lon from *both* GGA and RMC and keeps whichever raw NMEA
# field string had more digits after the decimal point. This tracks the
# digit count of whichever reading is currently stored in gps_lat/gps_lon,
# so a later sentence only overwrites it if it's equal-or-more precise.
_gps_coord_precision = -1

# Per-satellite sky-plot data, from GSV:
# {(talker, prn): (elevation_deg, azimuth_deg, snr, last_seen_ticks)}.
# talker is e.g. "GP" (GPS) or "GL" (GLONASS), kept alongside prn since PRN
# numbering isn't guaranteed unique across constellations.
#
# Updated incrementally, one satellite at a time, as each GSV message is
# parsed - not gated on a full multi-message cycle completing. An earlier
# version waited for the last message in a cycle before committing
# anything, on the theory that it'd avoid showing a half-updated list -
# but a single dropped/corrupted sentence (routine on this I2C link, same
# as the odd missed GGA/RMC) meant that satellite would vanish from the
# plot for a whole second instead of just keeping its last known position.
# Continuous merging plus the staleness prune in _update_fix_status()
# (SAT_STALE_TIMEOUT) gives smoother, more honest behaviour: satellites
# only disappear once they've actually been unreported for a while.
gps_sats = {}

# How long (seconds) a satellite can go unreported in GSV before we drop
# it from gps_sats. Longer than GPS_FIX_TIMEOUT since a satellite briefly
# missing one GSV cycle isn't the same signal as losing the fix entirely.
SAT_STALE_TIMEOUT = 15

# Separate, much shorter timeout for gps_sats_used specifically. This only
# needs to be long enough to bridge one GPGSA/GLGSA alternation (~1-2s) -
# the *set* of satellites actually contributing to the fix can genuinely
# change every few seconds, so reusing SAT_STALE_TIMEOUT's 15s window here
# made "used" satellites linger long after they'd actually dropped out of
# the solution, showing far more green dots than were really in use.
SAT_USED_STALE_TIMEOUT = 3

# PRNs (numbers only, no talker - GSA doesn't repeat it per-satellite the
# way GSV does) actually used in the current fix solution, from GSA fields
# 3-14: {prn: last_seen_ticks}. Satellites in gps_sats but not a key here
# are visible but unused.
#
# A dict with per-satellite timestamps, not a plain set that gets
# replaced wholesale by each GSA sentence - this receiver (GPS+GLONASS)
# appears to send separate per-constellation GSA sentences ($GPGSA, then
# $GLGSA) rather than one combined one, and replacing the whole set on
# each one meant each sentence wiped out the other's contribution -
# whichever arrived last in a given cycle "won", so GPS satellites only
# ever showed as used in the brief window before the next $GLGSA
# overwrote them. Merging by satellite (like gps_sats already does) lets
# both constellations' contributions coexist; SAT_STALE_TIMEOUT-based
# pruning (see _update_fix_status()) clears one out once it's genuinely
# stopped being reported, same as gps_sats.
gps_sats_used = {}

# Ticks of the last time we processed *any* NMEA sentence, valid or not.
# Distinct from gps_last_fix_ticks (which only updates on a successful RMC
# fix) - this is how we tell "module unplugged, no data at all" apart from
# "module present but still searching for a fix".
_last_nmea_activity_ticks = None

# Ticks of the first time we ever saw a valid RMC fix this session. Used
# only to gate the first RTC sync behind FIRST_SYNC_DELAY - never reset,
# even if the fix is later lost and reacquired.
_first_valid_fix_ticks = None

_i2c_consecutive_errors = 0
_i2c_last_recover_ticks = None
_i2c_recover_count = 0

# Location logging state. _gps_log_path is decided once at boot (see
# _init_gps_log()/_next_gps_log_path()) and used for every row this
# session - there's no cap to track since each power-on gets its own file.
# _gps_log_buffer holds sampled-but-not-yet-written rows (see
# GPS_LOG_SAMPLE_INTERVAL vs GPS_LOG_WRITE_INTERVAL above).
_gps_log_path = None
_gps_log_buffer = []
_gps_last_sample_ticks = None
_gps_last_write_ticks = None

# Simple non-blocking fade envelope: hold at full brightness for
# _led_hold_ms, then linearly fade to off over _led_fade_ms. Both a quick
# sync blink and the longer fix-acquired flash use this same mechanism,
# just with different durations - see _led_flash().
_led_effect_start_ticks = None
_led_hold_ms = 0
_led_fade_ms = 0

# Current ambient dimming level: "bright", "dim", or "dark". See the
# AMBIENT_* constants above for the hysteresis thresholds.
_ambient_level = "bright"

# True if set_brightness() is available and hasn't failed yet - flips to
# False permanently (for this session) the first time it throws, after
# which dimming is simply skipped.
_backlight_working = set_brightness is not None
_last_applied_ambient_level = None

# Smooth transition state: ramps from _ambient_transition_start_value to
# _ambient_transition_target over AMBIENT_TRANSITION_MS, rather than
# jumping straight to the new level. _ambient_current_brightness is
# whatever value was last actually sent to set_brightness() - used as the
# starting point if a new transition begins before the current one
# finishes, so interrupting a fade restarts smoothly from where it
# actually is rather than jumping.
AMBIENT_TRANSITION_MS = 250
_ambient_current_brightness = 1.0
_ambient_transition_start_value = 1.0
_ambient_transition_target = 1.0
_ambient_transition_start_ticks = None

# Which screen update() is currently drawing: "clock", "info1", "info2",
# "info3", or "settings".
_screen = "clock"

# User-adjustable settings. All start from their code defaults and are
# only adjustable at runtime from the Settings screen (button B from the
# clock) - none persist across a reboot.
local_offset = LOCAL_OFFSET  # hours added to UTC for local time display
time_format_24h = False       # False = 12H with AM/PM, True = 24H
gps_i2c_addr = GPS_I2C_ADDR   # which GPS module address _poll_gps()/_configure_gps_for_address() talk to
gps_logging_enabled = True    # whether _maybe_log_gps() samples/writes rows at all

_SETTINGS_FIELD_COUNT = 4
_settings_selected_index = 0  # which field UP/DOWN currently adjusts

# Debug/diagnostic counters - shown on screen since Thonny's console won't
# catch prints from init() unless you're already attached before the app
# launches from the menu.
DEBUG_OVERLAY = False         # set False once everything's working
i2c_scan_result = None
i2c_read_errors = 0
i2c_bytes_read = 0
nmea_lines_seen = 0          # any line starting with '$', valid or not
gga_count = 0
rmc_count = 0
gsa_count = 0
last_nmea_line = ""

_time_font = rom_font.ignore      # 17px, colossal - the biggest built-in pixel font
_local_font = rom_font.absolute   # 10px - used for the local time line
_label_font = rom_font.nope       # 8px - corner labels (NO FIX / sats)
_debug_font = rom_font.ark        # 6px - tiny debug overlay text

# rom_font.ignore is already the largest built-in pixel font, so to go
# bigger we render the UTC text at native size into a small offscreen
# buffer, then blit that buffer scaled up. It'll look chunkier/blockier
# at larger scales (this is a raster scale-up, not a bigger font) - drop
# this if it gets too pixelated, or reduce it if "HH:MM:SSZ" no longer
# fits your screen width at this scale.
UTC_SCALE = 1.4

_utc_buf = None
_utc_buf_size = None  # (w, h) of the buffer at native font size

LOCAL_ALPHA = 204  # 0-255; 204 = ~80% opaque ("20% transparent")

# Battery icon (top-right) - same layout constants as the badger2350 menu's
# draw_header() battery indicator (firmware/apps/menu/ui.py).
_BATTERY_W = 16
_BATTERY_H = 8
_BATTERY_NUB_W = 1


# ---------------------------------------------------------------------------
# Date/time helpers
# ---------------------------------------------------------------------------

def _is_leap(year):
    return (year % 4 == 0 and year % 100 != 0) or (year % 400 == 0)


def _days_in_month(year, month):
    days = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    if month == 2 and _is_leap(year):
        return 29
    return days[month - 1]


def _day_of_week(year, month, day):
    # Sakamoto's algorithm. Returns 0=Sunday .. 6=Saturday.
    # NOTE: the exact convention rtc.datetime() expects for `dow` isn't
    # documented - adjust here if the display ever shows the wrong day.
    t = [0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4]
    y = year
    if month < 3:
        y -= 1
    return (y + y // 4 - y // 100 + y // 400 + t[month - 1] + day) % 7


def _add_seconds(year, month, day, hour, minute, second, delta_seconds):
    """Apply a whole-second offset (positive or negative) to a datetime,
    handling minute/hour/day/month/year rollover."""
    total_seconds = hour * 3600 + minute * 60 + second + delta_seconds

    day_delta = 0
    while total_seconds < 0:
        total_seconds += 24 * 3600
        day_delta -= 1
    while total_seconds >= 24 * 3600:
        total_seconds -= 24 * 3600
        day_delta += 1

    hour = total_seconds // 3600
    minute = (total_seconds % 3600) // 60
    second = total_seconds % 60
    day += day_delta

    while day < 1:
        month -= 1
        if month < 1:
            month = 12
            year -= 1
        day += _days_in_month(year, month)

    while day > _days_in_month(year, month):
        day -= _days_in_month(year, month)
        month += 1
        if month > 12:
            month = 1
            year += 1

    return year, month, day, hour, minute, second


def _add_offset(year, month, day, hour, minute, second, offset_hours):
    """Apply an hour offset (may be fractional, e.g. 5.5) to a datetime."""
    return _add_seconds(year, month, day, hour, minute, second, int(round(offset_hours * 3600)))


def _format_utc(hour, minute, second):
    return "{:02d}:{:02d}:{:02d}Z".format(hour, minute, second)


def _format_12h(hour, minute, second):
    period = "AM" if hour < 12 else "PM"
    h12 = hour % 12
    if h12 == 0:
        h12 = 12
    return "{:d}:{:02d}:{:02d} {}".format(h12, minute, second, period)


def _format_24h(hour, minute, second):
    return "{:02d}:{:02d}:{:02d}".format(hour, minute, second)


def _format_compact_count(n):
    """Abbreviates large counters so they don't run off the info screen:
    1244 -> '1.2K', 3456789 -> '3.5M'. Anything under 1000 is shown as-is."""
    if n < 1000:
        return str(n)
    if n < 1000000:
        text = "{:.1f}K".format(n / 1000)
        # n like 999,600 rounds to "1000.0K" rather than rolling over to M.
        if text == "1000.0K":
            return "1.0M"
        return text
    return "{:.1f}M".format(n / 1000000)


# ---------------------------------------------------------------------------
# Battery indicator
# ---------------------------------------------------------------------------

def _draw_battery_icon(x, y):
    """Battery gauge with its top-left corner at (x, y), using the same
    vector-shape technique as badger2350's menu (firmware/apps/menu/ui.py,
    draw_header()): an outline rectangle plus nub, hollowed out with an
    inset rectangle, then a proportional fill rectangle drawn back in on
    top - and the same animated "charging" fill (a sawtooth driven by
    badge.ticks) rather than a static level while plugged in.

    Their version is black-on-white for the menu's light header bar; ours
    swaps that to white-on-navy so it reads correctly against our dark
    background - same drawing technique, different palette.

    Returns the total width drawn, so callers can lay out neighbouring UI.
    """
    if badge.is_charging():
        battery_level = (badge.ticks / 20) % 100
    else:
        try:
            battery_level = badge.battery_level()
        except Exception:
            battery_level = 0

    size = (_BATTERY_W, _BATTERY_H)
    pos = (x, y)

    # outline + nub, filled solid
    screen.pen = color.white
    screen.shape(shape.rectangle(pos[0], pos[1], size[0], size[1]))
    screen.shape(shape.rectangle(pos[0] + size[0], pos[1] + 2, _BATTERY_NUB_W, 4))

    # hollow out the middle back to the background colour, leaving an outline
    screen.pen = color.black
    screen.shape(shape.rectangle(pos[0] + 1, pos[1] + 1, size[0] - 2, size[1] - 2))

    # fill level
    fill_w = ((size[0] - 4) / 100) * battery_level
    screen.pen = color.white
    screen.shape(shape.rectangle(pos[0] + 2, pos[1] + 2, fill_w, size[1] - 4))

    return _BATTERY_W + _BATTERY_NUB_W


# ---------------------------------------------------------------------------
# GPS / NMEA handling
# ---------------------------------------------------------------------------

def _coord_decimal_digits(value_str):
    """Number of digits after the decimal point in a raw NMEA coordinate
    field (e.g. "3732.8824" -> 4). Used to figure out, per-sentence, which
    of GGA/RMC this receiver happens to report with more precision -
    see the _gps_coord_precision note above gps_lat/gps_lon."""
    if not value_str or "." not in value_str:
        return 0
    return len(value_str.split(".", 1)[1])


def _parse_nmea_coord(value_str, hemisphere):
    """Converts an NMEA ddmm.mmmm / dddmm.mmmm coordinate field plus its
    hemisphere letter (N/S/E/W) into signed decimal degrees. Works for both
    latitude (2-digit degrees) and longitude (3-digit degrees) - dividing by
    100 always splits off the last two digits as minutes regardless of how
    many degree digits come before them."""
    if not value_str or not hemisphere:
        return None
    try:
        raw = float(value_str)
    except ValueError:
        return None

    degrees = int(raw / 100)
    minutes = raw - degrees * 100
    decimal = degrees + minutes / 60

    if hemisphere in ("S", "W"):
        decimal = -decimal

    return decimal


def _handle_nmea_sentence(line):
    global gps_num_sats, gps_fix_valid, gps_last_fix_ticks, gps_datetime
    global gga_count, rmc_count, gsa_count, last_nmea_line
    global gps_hdop, gps_altitude, gps_fix_quality, gps_fix_type, gps_lat, gps_lon
    global _gps_coord_precision
    global gps_sats, gps_sats_used
    global _last_nmea_activity_ticks, _first_valid_fix_ticks

    last_nmea_line = line
    _last_nmea_activity_ticks = badge.ticks

    body = line.split("*")[0]
    fields = body.split(",")
    if not fields:
        return

    sentence_id = fields[0]

    if sentence_id.endswith("GGA"):
        gga_count += 1
        # $--GGA,time,lat,NS,lon,EW,fixquality,numSV,HDOP,alt,...
        if len(fields) > 5 and fields[2] and fields[4]:
            prec = min(_coord_decimal_digits(fields[2]), _coord_decimal_digits(fields[4]))
            if prec >= _gps_coord_precision:
                new_lat = _parse_nmea_coord(fields[2], fields[3])
                new_lon = _parse_nmea_coord(fields[4], fields[5])
                if new_lat is not None and new_lon is not None:
                    gps_lat = new_lat
                    gps_lon = new_lon
                    _gps_coord_precision = prec
        if len(fields) > 6 and fields[6]:
            try:
                gps_fix_quality = int(fields[6])
            except ValueError:
                pass
        if len(fields) > 7 and fields[7]:
            try:
                gps_num_sats = int(fields[7])
            except ValueError:
                pass
        if len(fields) > 8 and fields[8]:
            try:
                gps_hdop = float(fields[8])
            except ValueError:
                pass
        if len(fields) > 9 and fields[9]:
            try:
                gps_altitude = float(fields[9])
            except ValueError:
                pass

    elif sentence_id.endswith("GSA"):
        gsa_count += 1
        # $--GSA,mode1,mode2,sat1..sat12,PDOP,HDOP,VDOP
        # mode2: 1=no fix, 2=2D fix, 3=3D fix
        if len(fields) > 2 and fields[2]:
            try:
                gps_fix_type = int(fields[2])
            except ValueError:
                pass

        # Merge this sentence's satellites in by timestamp rather than
        # replacing gps_sats_used wholesale - see the comment on
        # gps_sats_used above for why (this receiver appears to send
        # separate per-constellation GSA sentences that would otherwise
        # overwrite each other every cycle).
        for i in range(3, min(15, len(fields))):
            if fields[i]:
                try:
                    gps_sats_used[int(fields[i])] = badge.ticks
                except ValueError:
                    pass

    elif sentence_id.endswith("GSV"):
        # $--GSV,total_msgs,msg_num,total_sats,[prn,elev,az,snr]x1-4
        # Talker (e.g. "GP"/"GL") tells apart satellites from different
        # constellations that could otherwise share a PRN number.
        talker = sentence_id[1:3]

        if len(fields) < 4:
            return

        # Each group of 4 fields starting at index 4 is one satellite.
        # The last message in a cycle is often padded with empty groups.
        # Each satellite is merged straight into gps_sats as it's parsed -
        # see the comment on gps_sats above for why this isn't gated on
        # the whole multi-message cycle completing.
        i = 4
        while i + 3 < len(fields):
            prn_str = fields[i]
            if prn_str:
                try:
                    prn = int(prn_str)
                    elevation = int(fields[i + 1]) if fields[i + 1] else None
                    azimuth = int(fields[i + 2]) if fields[i + 2] else None
                    snr = int(fields[i + 3]) if fields[i + 3] else None
                    if elevation is not None and azimuth is not None:
                        gps_sats[(talker, prn)] = (elevation, azimuth, snr, badge.ticks)
                except ValueError:
                    pass
            i += 4

    elif sentence_id.endswith("RMC"):
        rmc_count += 1
        # $--RMC,time,status,lat,NS,lon,EW,speed,course,date,...
        if len(fields) < 10:
            return
        status = fields[2]
        time_str = fields[1]
        date_str = fields[9]

        if status == "A" and len(time_str) >= 6 and len(date_str) == 6:
            try:
                hour = int(time_str[0:2])
                minute = int(time_str[2:4])
                second = int(time_str[4:6])
                day = int(date_str[0:2])
                month = int(date_str[2:4])
                year = 2000 + int(date_str[4:6])

                candidate = (year, month, day, hour, minute, second)

                # A corrupted read during a bus glitch (e.g. hot-plugging
                # the module) can still parse as structurally valid digits
                # while being nonsense - don't let that update the clock.
                if _is_plausible_datetime(candidate):
                    dow = _day_of_week(year, month, day)

                    if not gps_fix_valid:
                        # Edge-triggered: only fires on the NO FIX -> fix
                        # transition, not every frame the fix stays valid.
                        _led_fix_acquired_flash()

                    gps_datetime = (year, month, day, hour, minute, second, dow)
                    gps_fix_valid = True
                    gps_last_fix_ticks = badge.ticks

                    if _first_valid_fix_ticks is None:
                        _first_valid_fix_ticks = badge.ticks

                    if len(fields) > 6:
                        prec = min(_coord_decimal_digits(fields[3]), _coord_decimal_digits(fields[5]))
                        if prec >= _gps_coord_precision:
                            new_lat = _parse_nmea_coord(fields[3], fields[4])
                            new_lon = _parse_nmea_coord(fields[5], fields[6])
                            if new_lat is not None and new_lon is not None:
                                gps_lat = new_lat
                                gps_lon = new_lon
                                _gps_coord_precision = prec
            except ValueError:
                pass


def _poll_gps():
    global _nmea_buffer, i2c_read_errors, i2c_bytes_read, nmea_lines_seen
    global _i2c_consecutive_errors, _i2c_last_recover_ticks

    if _i2c is None:
        return

    # Drain everything currently waiting instead of reading one fixed-size
    # chunk. The module streams continuously (now ~11.5KB/sec at the
    # 115200 baud we configure it for in _configure_gps_pmtk(), up from 9600
    # default) regardless of how often we poll - if update() runs slower
    # than that (e.g. because rendering the scaled clock takes a while),
    # reading only one 32-byte chunk per frame falls further behind every
    # second, showing up as a growing display lag rather than a fixed
    # latency. Stop once a chunk comes back as all filler (nothing left to
    # read) or we hit the cap, so we don't spend forever draining on a bad
    # frame.
    for _ in range(_I2C_MAX_READS_PER_POLL):
        try:
            chunk = _i2c.readfrom(gps_i2c_addr, 32)
        except OSError:
            i2c_read_errors += 1
            _i2c_consecutive_errors += 1

            # Sustained errors usually mean the bus got wedged, e.g. by
            # hot-plugging the GPS module mid-transaction. Try to recover
            # rather than failing every frame forever - see _i2c_recover().
            if _i2c_consecutive_errors >= _I2C_ERROR_RECOVERY_THRESHOLD:
                now = badge.ticks
                if (_i2c_last_recover_ticks is None or
                        (now - _i2c_last_recover_ticks) >= _I2C_RECOVER_COOLDOWN):
                    _i2c_recover()
                    _i2c_last_recover_ticks = now
                    _i2c_consecutive_errors = 0
            return

        _i2c_consecutive_errors = 0
        i2c_bytes_read += len(chunk)

        has_data = False
        for b in chunk:
            if b == 0x0A:
                has_data = True
                line = _nmea_buffer.strip()
                _nmea_buffer = ""
                if line.startswith("$"):
                    nmea_lines_seen += 1
                    _handle_nmea_sentence(line)
            elif b == 0x0D:
                has_data = True
                continue
            elif 0x20 <= b <= 0x7E:
                # printable ASCII - part of a sentence
                has_data = True
                _nmea_buffer += chr(b)
                if len(_nmea_buffer) > 100:
                    # runaway/corrupt line - drop it and resync on the next \n
                    _nmea_buffer = ""
            else:
                # filler byte the module sends when no data is ready
                # (commonly 0x0A, but some firmware uses 0xFF/0x00 instead) -
                # ignore it rather than corrupting the buffer.
                continue

        if not has_data:
            # This chunk was entirely filler - nothing more is queued up
            # right now, so stop draining for this frame.
            break


def _update_fix_status():
    global gps_fix_valid
    global gps_num_sats, gps_hdop, gps_altitude, gps_fix_quality, gps_fix_type, gps_lat, gps_lon
    global gps_sats, gps_sats_used

    if gps_fix_valid and gps_last_fix_ticks is not None:
        if (badge.ticks - gps_last_fix_ticks) > GPS_FIX_TIMEOUT * 1000:
            gps_fix_valid = False

    # If we haven't processed *any* NMEA sentence in a while (e.g. the
    # module was unplugged), the satellite count/HDOP/altitude/position we
    # last saw are stale, not current - reset them rather than leaving old
    # numbers on screen forever. This is deliberately separate from the
    # gps_fix_valid check above, which only tracks RMC-based time fixes:
    # a module that's still present but hasn't resolved a fix yet keeps
    # sending GGA with a live (possibly climbing) satellite count, and we
    # don't want to zero that out just because RMC hasn't gone valid yet.
    module_silent = (
        _last_nmea_activity_ticks is None or
        (badge.ticks - _last_nmea_activity_ticks) > GPS_FIX_TIMEOUT * 1000
    )
    if module_silent:
        gps_num_sats = 0
        gps_hdop = None
        gps_altitude = None
        gps_fix_quality = 0
        gps_fix_type = 1
        gps_lat = None
        gps_lon = None
        gps_sats = {}
        gps_sats_used = {}
    else:
        # Drop individual satellites that haven't shown up in a GSV
        # message in a while (genuinely out of view now), separate from
        # the module_silent wipe above - this runs even while the module
        # is otherwise healthy and chatty.
        now = badge.ticks
        stale = [
            key for key, (_e, _a, _s, last_seen) in gps_sats.items()
            if (now - last_seen) > SAT_STALE_TIMEOUT * 1000
        ]
        for key in stale:
            del gps_sats[key]

        # Same idea for gps_sats_used - a satellite that hasn't appeared
        # in any GSA sentence in a while has genuinely stopped being used,
        # not just been overwritten by the other constellation's sentence
        # this cycle (that's what the per-satellite timestamps prevent).
        used_stale = [
            prn for prn, last_seen in gps_sats_used.items()
            if (now - last_seen) > SAT_USED_STALE_TIMEOUT * 1000
        ]
        for prn in used_stale:
            del gps_sats_used[prn]


def _led_flash(hold_ms, fade_ms):
    """Starts (or restarts) the LED fade envelope: full brightness for
    hold_ms, then a linear fade to off over fade_ms. Non-blocking -
    _update_leds() advances it a little each frame."""
    global _led_effect_start_ticks, _led_hold_ms, _led_fade_ms
    _led_effect_start_ticks = badge.ticks
    _led_hold_ms = hold_ms
    _led_fade_ms = fade_ms


def _led_fix_acquired_flash():
    """Brief full-brightness hold, then fades out - for the NO FIX -> fix
    transition specifically."""
    _led_flash(hold_ms=100, fade_ms=800)


def _update_leds():
    """Advances the current fade envelope (if any) and writes the
    resulting brightness to all four rear LEDs via badge.caselights().
    Called every frame."""
    global _led_effect_start_ticks

    if not LEDS_ENABLED or _led_effect_start_ticks is None:
        return

    elapsed = badge.ticks - _led_effect_start_ticks

    if elapsed < _led_hold_ms:
        level = 1.0
    elif elapsed < _led_hold_ms + _led_fade_ms:
        level = 1.0 - (elapsed - _led_hold_ms) / _led_fade_ms
        level = max(0.0, level)
    else:
        level = 0.0
        _led_effect_start_ticks = None

    try:
        badge.caselights(level)
    except Exception as e:
        print("gps_time: caselights() failed:", e)


def _update_ambient_dimming():
    """Reads the ambient light sensor and updates _ambient_level using
    hysteresis (see the AMBIENT_* constants). Called once per frame.
    badge.light_level() is Tufty-only and its exact behaviour on this
    board is unconfirmed - if the call fails or doesn't exist, this just
    leaves the current level unchanged rather than erroring."""
    global _ambient_level

    try:
        raw = badge.light_level()
    except Exception:
        return

    if _ambient_level == "bright":
        if raw < AMBIENT_DIM_ENTER:
            _ambient_level = "dim"
    elif _ambient_level == "dim":
        if raw < AMBIENT_DARK_ENTER:
            _ambient_level = "dark"
        elif raw > AMBIENT_DIM_EXIT:
            _ambient_level = "bright"
    else:  # "dark"
        if raw > AMBIENT_DARK_EXIT:
            _ambient_level = "dim"


def _apply_ambient_level():
    """Applies the current _ambient_level via real backlight control, if
    set_brightness() is available and hasn't failed yet. Smoothly ramps
    from the current brightness to the target over AMBIENT_TRANSITION_MS
    rather than jumping straight there - runs every frame while a
    transition is in progress, then goes idle once it arrives. On
    failure, flips _backlight_working off for the rest of the session,
    after which dimming is simply skipped rather than falling back to
    anything on-screen."""
    global _backlight_working, _last_applied_ambient_level
    global _ambient_current_brightness, _ambient_transition_start_value
    global _ambient_transition_target, _ambient_transition_start_ticks

    if not _backlight_working:
        return

    if _ambient_level != _last_applied_ambient_level:
        # Level just changed - start a new ramp from wherever brightness
        # actually is right now (which might itself be mid-ramp) to the
        # new target, rather than restarting from the old target.
        _last_applied_ambient_level = _ambient_level
        _ambient_transition_start_value = _ambient_current_brightness
        _ambient_transition_target = AMBIENT_BRIGHTNESS_LEVELS[_ambient_level]
        _ambient_transition_start_ticks = badge.ticks

    if _ambient_transition_start_ticks is None:
        return  # already sitting at the target, nothing to do this frame

    elapsed = badge.ticks - _ambient_transition_start_ticks
    if elapsed >= AMBIENT_TRANSITION_MS:
        _ambient_current_brightness = _ambient_transition_target
        _ambient_transition_start_ticks = None
    else:
        frac = elapsed / AMBIENT_TRANSITION_MS
        span = _ambient_transition_target - _ambient_transition_start_value
        _ambient_current_brightness = _ambient_transition_start_value + span * frac

    try:
        set_brightness(_ambient_current_brightness)
    except Exception as e:
        print("gps_time: set_brightness() failed:", e)
        _backlight_working = False


def _sync_rtc_now():
    """Force an immediate RTC sync from the last known GPS fix, regardless
    of RTC_SYNC_INTERVAL. Returns True if a sync actually happened."""
    global gps_last_sync_ticks

    if not rtc_available or not gps_fix_valid or gps_datetime is None:
        return False

    try:
        rtc.datetime(gps_datetime)
        gps_last_sync_ticks = badge.ticks
        return True
    except Exception:
        return False


def _maybe_sync_rtc():
    if not rtc_available or not gps_fix_valid or gps_datetime is None:
        return

    now = badge.ticks

    if gps_last_sync_ticks is None:
        # First sync of this session: let the fix settle for a bit rather
        # than trusting it the instant it appears.
        if _first_valid_fix_ticks is None or (now - _first_valid_fix_ticks) < FIRST_SYNC_DELAY * 1000:
            return
        _sync_rtc_now()
        return

    if (now - gps_last_sync_ticks) >= RTC_SYNC_INTERVAL * 1000:
        _sync_rtc_now()


def _fix_quality_label(fix_type):
    """Maps gps_fix_type (GSA mode2: 1=no fix, 2=2D, 3=3D) to the short
    string used in the CSV log - same 2D/3D/NO vocabulary as the on-screen
    fix status elsewhere in the app."""
    if fix_type == 3:
        return "3D"
    if fix_type == 2:
        return "2D"
    return "NO"


def _next_gps_log_path():
    """Scans GPS_LOG_DIR (the writable /state area) for existing
    gps_log_NNNN.csv files, and returns the full path for the next one in
    sequence (highest existing index + 1, or 0001 if none exist yet). This
    is what gives each power-on its own fresh file."""
    max_index = 0
    try:
        for name in os.listdir(GPS_LOG_DIR):
            if name.startswith(GPS_LOG_PREFIX) and name.endswith(GPS_LOG_SUFFIX):
                middle = name[len(GPS_LOG_PREFIX):-len(GPS_LOG_SUFFIX)]
                try:
                    idx = int(middle)
                    if idx > max_index:
                        max_index = idx
                except ValueError:
                    continue
    except OSError as e:
        print("gps_time: couldn't list", GPS_LOG_DIR, "for existing logs:", e)

    return "{}/{}{:04d}{}".format(GPS_LOG_DIR, GPS_LOG_PREFIX, max_index + 1, GPS_LOG_SUFFIX)


def _init_gps_log():
    """Called once from init(). Picks a fresh, uniquely-numbered CSV path
    for this session, writes its header row, and starts the sample/write
    timers from this moment so the first sample/flush land a full interval
    after boot rather than immediately.

    Also prints os.getcwd() - purely diagnostic, left in so a future path
    problem (like this one) shows up immediately in the console rather
    than needing to be re-discovered by trial and error."""
    global _gps_log_path, _gps_last_sample_ticks, _gps_last_write_ticks

    try:
        print("gps_time: cwd at boot:", os.getcwd())
    except Exception as e:
        print("gps_time: couldn't read cwd:", e)

    _gps_log_path = _next_gps_log_path()
    try:
        with open(_gps_log_path, "w") as f:
            f.write("timestamp,latitude,longitude,fix_quality,hdop,sats_in_use,altitude\n")
        print("gps_time: logging location to", _gps_log_path)
    except OSError as e:
        print("gps_time: failed to create", _gps_log_path, "-", e)
        _gps_log_path = None

    _gps_last_sample_ticks = badge.ticks
    _gps_last_write_ticks = badge.ticks


def _format_log_timestamp():
    """UTC timestamp for the log row, from whichever source the clock
    display itself is currently trusting (RTC if available/plausible, else
    last known GPS time) - so log timestamps stay sane even between fixes.
    Empty string if neither is available yet (e.g. no fix since boot)."""
    dt = _get_display_datetime()
    if dt is None:
        return ""
    year, month, day, hour, minute, second = dt[0], dt[1], dt[2], dt[3], dt[4], dt[5]
    return "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z".format(year, month, day, hour, minute, second)


def _sample_gps_fix():
    """Builds one CSV row from the current GPS state and appends it to
    _gps_log_buffer (RAM only - see _flush_gps_log() for the actual write).
    Fields are blank (not zero) when unknown, e.g. no fix yet, so they're
    not mistaken for real readings of 0.0."""
    timestamp = _format_log_timestamp()
    lat = "" if gps_lat is None else "{:.6f}".format(gps_lat)
    lon = "" if gps_lon is None else "{:.6f}".format(gps_lon)
    fix_quality = _fix_quality_label(gps_fix_type)
    hdop = "" if gps_hdop is None else "{:.1f}".format(gps_hdop)
    altitude = "" if gps_altitude is None else "{:.1f}".format(gps_altitude)

    _gps_log_buffer.append("{},{},{},{},{},{},{}\n".format(
        timestamp, lat, lon, fix_quality, hdop, gps_num_sats, altitude))


def _flush_gps_log():
    """Writes every row currently in _gps_log_buffer to disk in one go,
    then clears the buffer. A single open/write/close for the whole batch,
    rather than one per row, is what actually cuts down on flash writes -
    sampling into RAM alone wouldn't save anything if each sample still hit
    flash individually."""
    global _gps_log_buffer

    if _gps_log_path is None or not _gps_log_buffer:
        return

    try:
        with open(_gps_log_path, "a") as f:
            for row in _gps_log_buffer:
                f.write(row)
        _gps_log_buffer = []
    except OSError as e:
        print("gps_time: failed to flush location log:", e)


def _maybe_log_gps():
    """Called every update() frame. Takes a GPS reading into RAM every
    GPS_LOG_SAMPLE_INTERVAL seconds, and separately flushes everything
    buffered to flash every GPS_LOG_WRITE_INTERVAL seconds - the two
    timers are independent, so a sample and a flush can land in the same
    frame or several frames apart. Does nothing at all while
    gps_logging_enabled is False (Settings screen) - the sample/write
    timers just sit frozen at whatever they last were, so toggling this
    back on resumes into the same per-boot log file rather than starting
    a new one."""
    global _gps_last_sample_ticks, _gps_last_write_ticks

    if not gps_logging_enabled:
        return

    now = badge.ticks

    if _gps_last_sample_ticks is None or (now - _gps_last_sample_ticks) >= GPS_LOG_SAMPLE_INTERVAL * 1000:
        _gps_last_sample_ticks = now
        _sample_gps_fix()

    if _gps_last_write_ticks is None or (now - _gps_last_write_ticks) >= GPS_LOG_WRITE_INTERVAL * 1000:
        _gps_last_write_ticks = now
        _flush_gps_log()


def _i2c_recover():
    """Attempt to un-wedge the I2C bus and reinitialise the GPS connection.

    This is aimed at the common hot-plug failure mode: the QWIIC connector
    makes contact mid-transaction and a slave (the GPS module, or whatever
    else shares the bus) is left holding SDA low. I2C has no native
    hot-plug protocol, so the RP2350's I2C peripheral can otherwise wait
    forever for an ack/clock-stretch that will never come - the classic
    symptom of that is the whole app appearing to lock up.

    The fix is the standard I2C bus-recovery trick: manually toggle SCL
    (up to 9 times) while watching SDA, which walks a wedged slave through
    any partial byte it thinks it's sending until it releases the bus.
    This can only recover a *slave-holding-SDA-low* condition - it can't
    do anything about a genuine blocking hang inside a driver call, since
    by definition we never get back control to run it in that case.
    """
    global _i2c, i2c_scan_result, _i2c_recover_count

    _i2c_recover_count += 1
    print("gps_time: attempting i2c bus recovery (#{})".format(_i2c_recover_count))

    try:
        scl = machine.Pin(I2C_SCL_PIN, machine.Pin.OUT)
        sda = machine.Pin(I2C_SDA_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        scl.value(1)
        for _ in range(9):
            if sda.value():
                break
            scl.value(0)
            time.sleep_us(5)
            scl.value(1)
            time.sleep_us(5)
    except Exception as e:
        print("gps_time: bus recovery clock toggle failed:", e)

    try:
        _i2c = machine.I2C(I2C_ID, sda=machine.Pin(I2C_SDA_PIN), scl=machine.Pin(I2C_SCL_PIN), freq=I2C_FREQ)
        i2c_scan_result = _i2c.scan()
        print("gps_time: i2c devices found after recovery:", i2c_scan_result)
        _configure_gps_for_address()
    except Exception as e:
        _i2c = None
        i2c_scan_result = "recovery failed"
        print("gps_time: i2c reinit after recovery failed:", e)


def _is_plausible_datetime(dt):
    """Sanity-checks a (year, month, day, hour, minute, second, dow) tuple.
    Guards against displaying a bogus-but-plausible-looking timestamp (like
    midnight on 0000-00-00) if an I2C glitch - e.g. from hot-plugging the
    GPS module - causes a read to come back as all zeros, or otherwise out
    of range, instead of raising a clean exception."""
    if dt is None:
        return False
    try:
        year, month, day, hour, minute, second = dt[0], dt[1], dt[2], dt[3], dt[4], dt[5]
    except (TypeError, IndexError):
        return False

    return (
        2024 <= year <= 2100 and
        1 <= month <= 12 and
        1 <= day <= 31 and
        0 <= hour <= 23 and
        0 <= minute <= 59 and
        0 <= second <= 59
    )


def _get_display_datetime():
    """Time source for the clock display: prefer the RTC (kept accurate by
    GPS syncs), fall back to the last known GPS time if there's no RTC."""
    if rtc_available:
        try:
            dt = rtc.datetime()
            if _is_plausible_datetime(dt):
                return dt
        except Exception:
            pass

    if _is_plausible_datetime(gps_datetime):
        return gps_datetime

    return None


# MTK NMEA output config: enable GGA + RMC + GSA + GSV, once per fix, and
# disable GLL/VTG. The PA1010D sends sentences in a burst over its internal
# UART once per second, and trimming unused sentence types keeps that burst
# shorter so RMC (and everything else) is available sooner after the true
# GPS second. GSA is included because its mode2 field is the only thing
# that tells us 2D vs 3D fix - GGA's fix-quality field doesn't carry that.
# GSV is included (despite being the biggest addition to the burst, since
# it needs one sentence per ~4 satellites in view) because it's the only
# source of per-satellite elevation/azimuth/SNR - needed for the sky plot
# on the third GPS info page. At 115200 baud the extra bytes cost single-
# digit milliseconds, which is why this is affordable now when it wouldn't
# have been worth it back at the original 9600 baud default.
_PMTK_SET_NMEA_OUTPUT_RMCGGAGSAGSV = b"$PMTK314,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
_PMTK_SET_NMEA_UPDATE_1HZ = b"$PMTK220,1000*1F\r\n"

# Raise the module's internal NMEA generation rate from the 9600 baud
# default to 115200. Even though we talk to it over I2C, not a physical
# UART, the I2C interface is a bridge over that same internal generation -
# I2C reads just drain whatever's in the buffer, filler bytes and all (see
# _poll_gps()). A slower internal baud means the once-a-second GGA+RMC+GSA
# burst takes measurably longer to finish being generated (~145ms at 9600
# baud for our ~140-byte burst, vs ~12ms at 115200) - all of which is
# latency between the true GPS second and RMC actually being parseable.
# This doesn't touch I2C_FREQ (the physical I2C clock, already far faster
# than either baud) or the 1Hz fix rate above - only how fast each burst
# is assembled internally.
#
# Sent last, after the sentence-selection and update-rate commands: since
# I2C writes aren't baud-gated the way physical UART commands would be,
# ordering doesn't matter functionally, but doing the "most disruptive"
# change last keeps the other two commands' effect unambiguous if this one
# turns out not to be honoured in I2C-bridge mode (unconfirmed against
# GlobalTop's own docs, which only cover the UART use case - if this
# doesn't help, it should still be harmless to leave in). If you ever need
# to walk it back, resend this same command with 9600 in place of 115200 -
# recovery doesn't depend on guessing the current baud, since I2C traffic
# itself was never baud-limited.
_PMTK_SET_BAUD_115200 = b"$PMTK251,115200*1F\r\n"


def _configure_gps_pmtk():
    """PA1010D (and other MTK-chipset modules) configuration - the original
    _configure_gps() body, just renamed now that there's more than one
    module type to configure. See the PMTK constants above for what each
    command does."""
    if _i2c is None:
        return
    try:
        _i2c.writeto(gps_i2c_addr, _PMTK_SET_NMEA_OUTPUT_RMCGGAGSAGSV)
    except OSError as e:
        print("gps_time: PMTK314 write failed:", e)
    try:
        _i2c.writeto(gps_i2c_addr, _PMTK_SET_NMEA_UPDATE_1HZ)
    except OSError as e:
        print("gps_time: PMTK220 write failed:", e)
    try:
        _i2c.writeto(gps_i2c_addr, _PMTK_SET_BAUD_115200)
    except OSError as e:
        print("gps_time: PMTK251 write failed:", e)


# ---------------------------------------------------------------------------
# u-blox (UBX protocol) configuration - used when gps_i2c_addr is 0x42.
# Confirmed working with a MAX-M10S (Sean Hodgins' PPS Watch) over Qwiic.
#
# u-blox M8/M9/M10-gen modules use a key/value config interface rather than
# MTK-style ASCII commands. Only one key is needed: CFG-I2COUTPROT-NMEA
# (0x10720002), which enables NMEA sentences over I2C/DDC so _poll_gps()
# has something to parse. Sent to the RAM layer only (not BBR/Flash), so
# it doesn't permanently alter a borrowed/shared GPS module.
# ---------------------------------------------------------------------------

_UBX_SYNC = b"\xb5\x62"
_UBX_CLASS_CFG = 0x06
_UBX_ID_CFG_VALSET = 0x8A
_UBX_LAYER_RAM = 0x01

_UBX_KEY_CFG_I2COUTPROT_NMEA = 0x10720002


def _ubx_checksum(data):
    """8-bit Fletcher checksum over the class/id/length/payload bytes, per
    the UBX frame spec (not including the 0xB5 0x62 sync bytes)."""
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def _ubx_valset_bool(key_id, value, layers=_UBX_LAYER_RAM):
    """Builds a complete UBX-CFG-VALSET frame setting a single 1-byte
    ("L"/bool-type) configuration item to 0 or 1. Only single-item, non-
    transactional VALSET is needed here, so the rarely-used fields
    (version, reserved, transaction id) are just left at 0."""
    payload = bytearray()
    payload.append(0x00)          # version: 0 = no transaction
    payload.append(layers & 0xFF)  # which layer(s) to write
    payload += b"\x00\x00"          # reserved
    payload += key_id.to_bytes(4, "little")
    payload.append(1 if value else 0)

    length = len(payload)
    frame_body = bytes([_UBX_CLASS_CFG, _UBX_ID_CFG_VALSET]) + length.to_bytes(2, "little") + bytes(payload)
    ck_a, ck_b = _ubx_checksum(frame_body)

    return _UBX_SYNC + frame_body + bytes([ck_a, ck_b])


def _configure_gps_ubx():
    """u-blox module configuration, sent when gps_i2c_addr is 0x42. Enables
    NMEA output on I2C so _poll_gps() (NMEA-only, no UBX binary parsing)
    has something to read. Verified against a real MAX-M10S."""
    if _i2c is None:
        return
    try:
        _i2c.writeto(gps_i2c_addr, _ubx_valset_bool(_UBX_KEY_CFG_I2COUTPROT_NMEA, True))
    except OSError as e:
        print("gps_time: UBX-CFG-VALSET (I2COUTPROT-NMEA) write failed:", e)


def _configure_gps_for_address():
    """Dispatches to the right configuration routine for whichever module
    is currently selected at gps_i2c_addr (set from the Settings screen, or
    left at the GPS_I2C_ADDR default at boot)."""
    if gps_i2c_addr == 0x42:
        _configure_gps_ubx()
    else:
        _configure_gps_pmtk()


# ---------------------------------------------------------------------------
# App lifecycle
# ---------------------------------------------------------------------------

def init():
    global _i2c, rtc_available, i2c_scan_result

    try:
        _i2c = machine.I2C(I2C_ID, sda=machine.Pin(I2C_SDA_PIN), scl=machine.Pin(I2C_SCL_PIN), freq=I2C_FREQ)
        i2c_scan_result = _i2c.scan()
        print("gps_time: i2c devices found:", i2c_scan_result)
        _configure_gps_for_address()
    except Exception as e:
        _i2c = None
        i2c_scan_result = "init failed"
        print("gps_time: i2c init failed:", e)

    try:
        rtc.datetime()
        rtc_available = True
    except Exception:
        rtc_available = False
    print("gps_time: rtc available:", rtc_available)

    _load_cog_image()
    _load_home_image()

    _init_gps_log()


_ARROW_W = 10
_ARROW_H = 7

# Deliberately small and fixed rather than filling available screen space -
# see the comment in _draw_info_screen_1() for why.
SKY_PLOT_MAX_RADIUS = 44


def _draw_chevron(cx, cy, pointing_down):
    """V-shaped (or inverted-V) chevron made of two line segments, centred
    at (cx, cy). Drawn twice with a 1px offset to fake a ~2px stroke,
    since line() taking a width/thickness argument isn't something I've
    been able to confirm for this API - only the plain 4-argument form is
    documented, so this only relies on that."""
    screen.pen = color.rgb(255, 255, 255, LOCAL_ALPHA)
    hw = _ARROW_W / 2
    hh = _ARROW_H / 2

    if pointing_down:
        left_y, tip_y = cy - hh, cy + hh
    else:
        left_y, tip_y = cy + hh, cy - hh

    left_x, right_x, tip_x = cx - hw, cx + hw, cx

    for dy in (0, 1):
        screen.line(left_x, left_y + dy, tip_x, tip_y + dy)
        screen.line(tip_x, tip_y + dy, right_x, left_y + dy)


def _draw_down_arrow(cx, cy):
    """2px-ish down-pointing chevron centred at (cx, cy)."""
    _draw_chevron(cx, cy, pointing_down=True)


def _draw_up_arrow(cx, cy):
    """2px-ish up-pointing chevron centred at (cx, cy)."""
    _draw_chevron(cx, cy, pointing_down=False)


_COG_RADIUS = 5
_COG_HOLE_RADIUS = 2
_COG_TOOTH = 2


def _draw_cog_icon_fallback(cx, cy):
    """Small grey cog/gear icon centred at (cx, cy), built from primitive
    shapes - used if the cog.png sprite (see below) doesn't load. Blocky
    by nature (4 teeth is about the limit of what still reads as a gear at
    this size using rectangles), which is the whole reason cog.png exists -
    a real drawn icon looks much cleaner than this."""
    screen.pen = color.smoke

    screen.circle(cx, cy, _COG_RADIUS)

    screen.rectangle(cx - 1, cy - _COG_RADIUS - _COG_TOOTH, 2, _COG_TOOTH + 1)
    screen.rectangle(cx - 1, cy + _COG_RADIUS - 1, 2, _COG_TOOTH + 1)
    screen.rectangle(cx - _COG_RADIUS - _COG_TOOTH, cy - 1, _COG_TOOTH + 1, 2)
    screen.rectangle(cx + _COG_RADIUS - 1, cy - 1, _COG_TOOTH + 1, 2)

    # hollow centre hole, same double-shape trick used elsewhere (battery
    # icon, sky-plot rings) - draw the background colour on top to punch
    # a hole rather than leaving a solid disc
    screen.pen = color.black
    screen.circle(cx, cy, _COG_HOLE_RADIUS)


# cog.png: a 20x20 transparent PNG, 6-spoke grey cog at 50% alpha, generated
# to replace the blocky primitive-drawn version above. Ships alongside
# __init__.py in the app folder - the sync script needs to copy it there
# too, not just __init__.py.
_COG_IMAGE_SIZE = 18
_cog_image = None


def _load_cog_image():
    """Attempts to load cog.png once at startup. Falls back to the drawn
    icon (_draw_cog_icon_fallback) if it's missing or won't load - this is
    a best-effort based on Image.load(path) as documented for Badgeware's
    sibling badge platforms; I haven't been able to confirm the exact call
    signature against this specific Tufty firmware build, so treat this as
    "should work" rather than guaranteed."""
    global _cog_image
    try:
        _cog_image = image.load("cog.png")
        print("gps_time: loaded cog.png")
    except Exception as e:
        _cog_image = None
        print("gps_time: could not load cog.png, using drawn fallback:", e)


def _draw_cog_icon(cx, cy):
    """Settings-button hint icon, centred at (cx, cy): the loaded cog.png
    sprite if available, otherwise the hand-drawn fallback above."""
    if _cog_image is not None:
        half = _COG_IMAGE_SIZE / 2
        screen.blit(_cog_image, rect(int(cx - half), int(cy - half), _COG_IMAGE_SIZE, _COG_IMAGE_SIZE))
    else:
        _draw_cog_icon_fallback(cx, cy)


def _draw_home_icon_fallback(cx, cy):
    """Simple grey house silhouette from primitives (triangle roof,
    rectangle body) - used if home.png doesn't load."""
    screen.pen = color.smoke

    roof_w = 10
    roof_h = 6
    screen.triangle(cx - roof_w / 2, cy - 1, cx + roof_w / 2, cy - 1, cx, cy - 1 - roof_h)

    body_w = 8
    body_h = 6
    screen.rectangle(int(cx - body_w / 2), int(cy - 1), body_w, body_h)


# home.png: a 20x20 transparent PNG, same grey/alpha as cog.png, generated
# to match its style. Ships alongside __init__.py in the app folder, same
# as cog.png - the sync script needs to copy this one too.
_HOME_IMAGE_SIZE = 20
_home_image = None


def _load_home_image():
    """Same best-effort approach as _load_cog_image() - see its comment
    for the caveat on Image.load(path)."""
    global _home_image
    try:
        _home_image = image.load("home.png")
        print("gps_time: loaded home.png")
    except Exception as e:
        _home_image = None
        print("gps_time: could not load home.png, using drawn fallback:", e)


def _draw_home_icon(cx, cy):
    """Home-button hint icon, centred at (cx, cy): the loaded home.png
    sprite if available, otherwise the hand-drawn fallback above."""
    if _home_image is not None:
        half = _HOME_IMAGE_SIZE / 2
        screen.blit(_home_image, rect(int(cx - half), int(cy - half), _HOME_IMAGE_SIZE, _HOME_IMAGE_SIZE))
    else:
        _draw_home_icon_fallback(cx, cy)


def _go_home():
    """Syncs the RTC from GPS (if we have a fix), then returns to the
    badge's home menu.

    There's no documented "return to menu" call for apps using the normal
    update() loop - the only things that lead back to the menu are the
    physical HOME button (intercepted by the firmware, never reaching our
    code) or main.py restarting, which the docs describe as happening "on
    the default firmware" after badge.sleep(). We actually tried sleep()
    for exactly this earlier in this app's history and it hard-froze the
    badge instead of waking itself back up on schedule - a real firmware
    quirk, not something fixable from application code, which is why that
    attempt got backed out.

    machine.reset() is a plainer tool: a full watchdog reset rather than a
    sleep/wake cycle, so it never goes through whatever in sleep() broke
    last time. It should land on the same "main.py restarts, menu
    reappears" outcome, just via a full reboot rather than a graceful
    hand-off - expect a brief reboot flicker, not an instant transition.
    I can't fully rule out this having its own surprises without testing
    on real hardware, but it's the more standard, lower-risk tool of the
    two for this job.

    Also flushes the location log buffer first - a reset here is the one
    "leaving the app" path we get to intercept before a real power-off,
    so it's a free chance to save any samples taken since the last
    scheduled GPS_LOG_WRITE_INTERVAL flush rather than losing them."""
    _sync_rtc_now()
    _flush_gps_log()
    machine.reset()


def _draw_clock_screen():
    screen.pen = color.black
    screen.clear()

    dt = _get_display_datetime()

    if dt is not None:
        year, month, day, hour, minute, second, _dow = dt
        utc_str = _format_utc(hour, minute, second)

        ly, lm, ld, lh, lmin, lsec = _add_offset(year, month, day, hour, minute, second, local_offset)
        if time_format_24h:
            local_str = "Local " + _format_24h(lh, lmin, lsec)
        else:
            local_str = "Local " + _format_12h(lh, lmin, lsec)
    else:
        utc_str = "--:--:--Z"
        local_str = "Local --:--:--" if time_format_24h else "Local --:--:-- --"

    # UTC colour is a three-state trust indicator at a glance:
    # orange = no GPS fix, yellow = fix but the RTC hasn't been synced
    # from it yet, white = fix and RTC synced at least once.
    if not gps_fix_valid:
        utc_color = color.orange
    elif gps_last_sync_ticks is None:
        utc_color = color.yellow
    else:
        utc_color = color.white

    # -- UTC clock: render at native size to an offscreen buffer, then
    # blit that buffer scaled up so it's bigger than any built-in font --
    screen.font = _time_font
    tw, th = screen.measure_text(utc_str)
    tw, th = int(tw), int(th)

    global _utc_buf, _utc_buf_size
    if _utc_buf is None or _utc_buf_size != (tw, th):
        _utc_buf = image(tw, th)
        _utc_buf_size = (tw, th)

    _utc_buf.pen = color.black
    _utc_buf.clear()
    _utc_buf.pen = utc_color
    _utc_buf.font = _time_font
    _utc_buf.text(utc_str, 0, 0)

    dest_w = int(tw * UTC_SCALE)
    dest_h = int(th * UTC_SCALE)
    x = int((screen.width - dest_w) / 2)
    y = int((screen.height - dest_h) / 2)
    screen.blit(_utc_buf, rect(x, y, dest_w, dest_h))

    # -- owner name: same size/style as the local time line, just above the
    # UTC clock --
    screen.font = _local_font
    screen.pen = color.rgb(255, 255, 255, LOCAL_ALPHA)
    wn, hn = screen.measure_text(OWNER_NAME)
    wn = int(wn)
    hn = int(hn)
    xn = int((screen.width - wn) / 2)
    name_gap = 4
    yn = y - hn - name_gap
    screen.text(OWNER_NAME, xn, yn)

    # -- WOPR-style scanner: a small block sweeping left-right, halfway
    # between the name and the UTC clock. Position comes directly from
    # the displayed second value (a slow back-and-forth bounce, not a
    # full sweep every second) - phase-locked to the actual displayed
    # time rather than free-running off badge.ticks. Colour matches the
    # UTC clock's fix-status colour (orange/yellow/white). A nod to
    # WOPR's countdown display in WarGames.
    if dt is not None:
        box_w = 6
        box_h = 4
        margin = 12
        track_w = screen.width - 2 * margin - box_w

        # Clean 20-second triangle wave: left edge at second % 20 == 0,
        # right edge at second % 20 == 10, ten equal one-second steps each
        # way. (The previous version used a 38-second period that wasn't
        # symmetric between its up and down legs - technically correct
        # but landed on odd second values instead of round ones, which
        # read as "wandering" rather than a clean bounce.)
        cycle_pos = second % 20
        if cycle_pos <= 10:
            frac = cycle_pos / 10
        else:
            frac = (20 - cycle_pos) / 10
        box_x = int(margin + frac * track_w)

        gap_top = yn + hn
        gap_bottom = y
        scan_cy = (gap_top + gap_bottom) // 2
        box_y = int(scan_cy - box_h / 2) + 4

        screen.pen = utc_color
        screen.rectangle(box_x, box_y, box_w, box_h)

    # -- local time: dimmed a touch, in the lower part of the screen --
    screen.font = _local_font
    screen.pen = color.rgb(255, 255, 255, LOCAL_ALPHA)
    w2, h2 = screen.measure_text(local_str)
    w2 = int(w2)
    h2 = int(h2)
    x2 = int((screen.width - w2) / 2)
    y2 = int(screen.height * 0.75 - h2 / 2)
    screen.text(local_str, x2, y2)

    # -- top-left: fix status, always shown (not just when there's no fix) -
    # same NO FIX / 2D / 3D logic as GPS Info 1, so the two stay consistent --
    screen.font = _label_font

    if not gps_fix_valid or gps_fix_type <= 1:
        fix_label = "NO FIX"
        fix_label_color = color.red
    elif gps_fix_type == 2:
        fix_label = "2D FIX"
        fix_label_color = color.orange
    else:
        fix_label = "3D FIX"
        fix_label_color = color.green

    screen.pen = fix_label_color
    screen.text(fix_label, 4, 4)

    # -- top-middle: satellites in view (gps_num_sats, from GGA), coloured
    # to match the fix status label at top-left rather than its own
    # sat-count thresholds - ties it to "is this trustworthy" instead of
    # a raw visibility count that says nothing about fix quality on its
    # own (see the earlier NO FIX-with-5-sats conversation) --
    sats_str = "{} sats".format(gps_num_sats)
    screen.pen = fix_label_color

    sw, _sh = screen.measure_text(sats_str)
    sw = int(sw)
    screen.text(sats_str, int((screen.width - sw) / 2), 4)

    # -- top-right: battery gauge --
    _draw_battery_icon(screen.width - _BATTERY_W - _BATTERY_NUB_W - 4, 4)

    # -- bottom-right: down arrow hints at the GPS info page below --
    _draw_down_arrow(screen.width - 12, screen.height - 10)

    # -- bottom-middle: cog icon over button B hints at Settings --
    _draw_cog_icon(screen.width // 2, screen.height - 10)

    # -- bottom-left: home icon over button A hints at returning to the
    # badge's home menu --
    _draw_home_icon(29, screen.height - 10)

    # -- debug overlay --
    if DEBUG_OVERLAY:
        screen.font = _debug_font
        screen.pen = color.smoke

        line1 = "i2c:{} rtc:{} b:{} e:{}".format(
            i2c_scan_result, "T" if rtc_available else "F", i2c_bytes_read, i2c_read_errors
        )
        line2 = "ln:{} gga:{} rmc:{}".format(
            _format_compact_count(nmea_lines_seen),
            _format_compact_count(gga_count),
            _format_compact_count(rmc_count),
        )

        _, lh = screen.measure_text(line1)
        lh = int(lh)
        screen.text(line1, 4, screen.height - (lh * 2) - 6)
        screen.text(line2, 4, screen.height - lh - 3)


def _draw_info_header(title, show_down_arrow):
    """Shared header for GPS info pages: title, battery, up arrow (every
    info page can page back up), and an optional down arrow for pages that
    have another one below them. Returns the y-coordinate below the header
    where page-specific content can start."""
    screen.pen = color.black
    screen.clear()

    screen.font = _label_font
    screen.pen = color.white
    screen.text(title, 4, 4)

    battery_x = screen.width - _BATTERY_W - _BATTERY_NUB_W - 4
    _draw_battery_icon(battery_x, 4)

    # up arrow, centred under the battery icon, hints at paging back up
    battery_center_x = battery_x + (_BATTERY_W + _BATTERY_NUB_W) / 2
    _draw_up_arrow(battery_center_x, 4 + _BATTERY_H + 2 + _ARROW_H / 2)

    if show_down_arrow:
        _draw_down_arrow(screen.width - 12, screen.height - 10)

    return 4 + _BATTERY_H + 2 + _ARROW_H + 4


def _draw_info_page(title, lines, show_down_arrow):
    """Shared renderer for label/value GPS info pages. `lines` is a list of
    (label, value, value_color) tuples."""
    start_y = _draw_info_header(title, show_down_arrow)

    screen.font = _debug_font
    _, sample_h = screen.measure_text("Ay")
    row_h = int(sample_h) + 4
    right_margin = 6

    for i, (label, value, value_color) in enumerate(lines):
        row_y = start_y + i * row_h
        screen.pen = color.smoke
        screen.text(label, 6, row_y)

        # Right-align the value so it never runs off the edge of the
        # screen, regardless of display resolution.
        screen.pen = value_color if value_color is not None else color.white
        vw, _vh = screen.measure_text(value)
        screen.text(value, screen.width - int(vw) - right_margin, row_y)


def _draw_info_screen_2():
    """GPS info, page 2 ("GPS Info 1" on screen): the fix itself."""
    fix_quality_names = {0: "none", 1: "GPS", 2: "DGPS"}
    fix_quality_str = fix_quality_names.get(gps_fix_quality, str(gps_fix_quality))

    # gps_fix_valid tracks whether we have a recent, usable time fix (from
    # RMC); gps_fix_type is the GSA dimensionality (2D/3D) of that fix. A
    # stale/lost fix always shows as NO FIX regardless of the last-known
    # GSA mode, since gps_fix_valid already accounts for GPS_FIX_TIMEOUT.
    if not gps_fix_valid or gps_fix_type <= 1:
        fix_status_str = "NO FIX"
        fix_status_color = color.red
    elif gps_fix_type == 2:
        fix_status_str = "2D"
        fix_status_color = color.orange
    else:
        fix_status_str = "3D"
        fix_status_color = color.green

    if gps_hdop is None:
        hdop_str = "--"
        hdop_color = None
    else:
        hdop_str = "{:.1f}".format(gps_hdop)
        if gps_hdop <= 2:
            hdop_color = color.green
        elif gps_hdop <= 5:
            hdop_color = color.orange
        else:
            hdop_color = color.red

    lines = [
        ("Fix status", fix_status_str, fix_status_color),
        ("Satellites", "{}".format(gps_num_sats), None),
        ("Fix quality", fix_quality_str, None),
        ("HDOP", hdop_str, hdop_color),
        ("Altitude", "{:.1f} m".format(gps_altitude) if gps_altitude is not None else "--", None),
    ]

    _draw_info_page("GPS Info 1", lines, show_down_arrow=True)


def _draw_info_screen_3():
    """GPS info, page 3 ("GPS Info 2" on screen, last): position + sync detail."""
    if gps_last_sync_ticks is not None:
        sync_age_s = (badge.ticks - gps_last_sync_ticks) / 1000
        rtc_sync_str = "{:.0f}s ago".format(sync_age_s)
    else:
        rtc_sync_str = "never"

    try:
        light_level_str = str(badge.light_level())
    except Exception:
        light_level_str = "--"

    lines = [
        ("RTC SYNC", rtc_sync_str, None),
        ("GGA / RMC seen", "{} / {}".format(_format_compact_count(gga_count), _format_compact_count(rmc_count)), None),
        ("Latitude", "{:.5f}".format(gps_lat) if gps_lat is not None else "--", None),
        ("Longitude", "{:.5f}".format(gps_lon) if gps_lon is not None else "--", None),
        ("Light level", light_level_str, None),
    ]

    _draw_info_page("GPS Info 2", lines, show_down_arrow=False)


def _draw_sky_plot(cx, cy, radius):
    """Polar sky plot: centre = zenith (elevation 90), edge = horizon
    (elevation 0), azimuth measured clockwise from N at the top - same
    convention as a standard GPS sky-view chart."""

    # Reference rings at elevation 60/30/0, using the same double-circle
    # outline trick as the battery icon: draw the ring colour, then a
    # slightly smaller circle in the background colour on top to hollow
    # out the middle rather than leaving a filled disc.
    for elevation in (60, 30, 0):
        r = int(radius * (90 - elevation) / 90)
        if r <= 0:
            continue
        screen.pen = color.smoke
        screen.circle(cx, cy, r)
        screen.pen = color.black
        screen.circle(cx, cy, max(0, r - 1))

    # Cardinal direction labels around the horizon ring.
    screen.font = _debug_font
    screen.pen = color.smoke
    for label, az in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
        rad = math.radians(az)
        tx = cx + (radius + 8) * math.sin(rad)
        ty = cy - (radius + 8) * math.cos(rad)
        tw, th = screen.measure_text(label)
        screen.text(label, int(tx - tw / 2), int(ty - th / 2))

    # Satellites: circles for GPS ("GP"), triangles for anything else
    # (e.g. GLONASS "GL") - mirrors the circle-vs-triangle convention from
    # typical sky-plot tools. Green if used in the current fix solution,
    # dim grey if just visible but not used.
    for (talker, prn), (elevation, azimuth, _snr, _last_seen) in gps_sats.items():
        r = radius * (90 - elevation) / 90
        rad = math.radians(azimuth)
        x = cx + r * math.sin(rad)
        y = cy - r * math.cos(rad)

        screen.pen = color.green if prn in gps_sats_used else color.smoke

        if talker == "GP":
            screen.circle(int(x), int(y), 3)
        else:
            screen.triangle(x - 3, y + 3, x + 3, y + 3, x, y - 3)


_LEGEND_MARKER_R = 3  # matches the satellite marker size in the plot itself


def _draw_sky_plot_legend(x, y, row_h):
    """Small legend explaining the sky plot's two independent visual
    dimensions: colour (used in fix vs. just visible) and shape
    (constellation). Short labels on purpose - there's only ever half the
    screen width to work with here, and this display's actual usable
    width has surprised us before, so labels stay short rather than
    relying on there being room."""
    screen.font = _debug_font

    entries = [
        (color.green, "circle", "Used"),
        (color.smoke, "circle", "Visible"),
        (color.white, "circle", "GPS"),
        (color.white, "triangle", "Other"),
    ]

    for i, (marker_color, shape_kind, label) in enumerate(entries):
        row_y = y + i * row_h
        marker_cx = x + _LEGEND_MARKER_R
        marker_cy = row_y + _LEGEND_MARKER_R

        screen.pen = marker_color
        if shape_kind == "circle":
            screen.circle(marker_cx, marker_cy, _LEGEND_MARKER_R)
        else:
            screen.triangle(
                marker_cx - _LEGEND_MARKER_R, marker_cy + _LEGEND_MARKER_R,
                marker_cx + _LEGEND_MARKER_R, marker_cy + _LEGEND_MARKER_R,
                marker_cx, marker_cy - _LEGEND_MARKER_R,
            )

        screen.pen = color.smoke
        _, th = screen.measure_text(label)
        screen.text(label, x + _LEGEND_MARKER_R * 2 + 4, int(row_y + _LEGEND_MARKER_R - th / 2))


_SKY_PLOT_LEGEND_LABELS = ["Used", "Visible", "GPS", "Other"]


def _draw_info_screen_1():
    """GPS info, page 1 (first, "Sky View" on screen): sky plot of
    satellites currently in view (left), with a legend (right) explaining
    the marker colours/shapes."""
    start_y = _draw_info_header("Sky View", show_down_arrow=True)

    available_h = screen.height - start_y - 4

    # Measure the legend's actual width first and anchor it to the real
    # right edge of the screen, rather than assuming a layout and hoping
    # the legend fits after - this display's real width has been wrong
    # more than once already (see the info-page column fix, the sky plot
    # radius fix). Only once we know how much room the legend genuinely
    # needs do we size the plot with whatever's left.
    screen.font = _debug_font
    legend_max_label_w = max(int(screen.measure_text(t)[0]) for t in _SKY_PLOT_LEGEND_LABELS)
    legend_gap = 4
    legend_w = _LEGEND_MARKER_R * 2 + legend_gap + legend_max_label_w
    right_margin = 4
    plot_to_legend_gap = 8

    # Deliberately conservative rather than trying to fill available
    # space: SKY_PLOT_MAX_RADIUS is a small, fixed cap, and the fit-to-
    # screen numbers below can only ever shrink it further, never grow it
    # past that cap. Reserve an extra 14px beyond the ring itself for the
    # N/E/S/W labels drawn just outside it.
    label_margin = 14
    plot_budget_w = screen.width - legend_w - right_margin - plot_to_legend_gap
    fits_width = (plot_budget_w - 2 * label_margin) // 2
    fits_height = (available_h - 2 * label_margin) // 2
    radius = min(SKY_PLOT_MAX_RADIUS, fits_width, fits_height)
    radius = max(10, radius)

    plot_footprint = radius + label_margin
    cx = plot_footprint + 6
    cy = start_y + available_h // 2

    _draw_sky_plot(cx, cy, radius)

    legend_x = screen.width - legend_w - right_margin
    _, sample_h = screen.measure_text("Ay")
    legend_row_h = int(sample_h) + 4
    _draw_sky_plot_legend(legend_x, start_y + 4, legend_row_h)

    if not gps_sats:
        screen.font = _debug_font
        screen.pen = color.smoke
        msg = "no data yet"
        mw, mh = screen.measure_text(msg)
        screen.text(msg, int(cx - mw / 2), int(cy - mh / 2))


MIN_LOCAL_OFFSET = -12.0  # Baker Island, UTC-12 - the most negative in real-world use
MAX_LOCAL_OFFSET = 14.0   # Kiribati (Line Islands), UTC+14 - the most positive in real-world use


def _adjust_selected_setting(direction):
    """direction is +1 or -1. Local offset adjusts by 0.5h per press,
    clamped to the real-world UTC offset range. Time format just flips
    between its two states. GPS I2C address cycles _GPS_I2C_ADDR_CHOICES
    and calls _configure_gps_for_address() to reconfigure the module at
    the new address. GPS Logging just flips on/off - see _maybe_log_gps()
    for how it's checked; toggling it doesn't start a new log file or
    touch what's already been written, it just pauses/resumes sampling
    into the existing session's file."""
    global local_offset, time_format_24h, gps_i2c_addr, gps_logging_enabled

    if _settings_selected_index == 0:
        local_offset += 0.5 * direction
        if local_offset > MAX_LOCAL_OFFSET:
            local_offset = MAX_LOCAL_OFFSET
        elif local_offset < MIN_LOCAL_OFFSET:
            local_offset = MIN_LOCAL_OFFSET
    elif _settings_selected_index == 1:
        time_format_24h = not time_format_24h
    elif _settings_selected_index == 2:
        idx = _GPS_I2C_ADDR_CHOICES.index(gps_i2c_addr)
        idx = (idx + direction) % len(_GPS_I2C_ADDR_CHOICES)
        gps_i2c_addr = _GPS_I2C_ADDR_CHOICES[idx]
        _configure_gps_for_address()
    else:
        gps_logging_enabled = not gps_logging_enabled


def _draw_settings_screen():
    screen.pen = color.black
    screen.clear()

    screen.font = _label_font
    screen.pen = color.white
    screen.text("Settings", 4, 4)

    _draw_battery_icon(screen.width - _BATTERY_W - _BATTERY_NUB_W - 4, 4)

    start_y = 4 + _BATTERY_H + 2 + 10

    screen.font = _debug_font
    _, sample_h = screen.measure_text("Ay")
    row_h = int(sample_h) + 6
    right_margin = 6

    offset_str = "{:+.1f}h".format(local_offset)
    format_str = "24H" if time_format_24h else "12H"
    addr_str = "0x{:02X}".format(gps_i2c_addr)
    logging_str = "ON" if gps_logging_enabled else "OFF"

    rows = [
        ("Local UTC offset", offset_str),
        ("Time format", format_str),
        ("GPS I2C Address", addr_str),
        ("GPS Logging", logging_str),
    ]

    for i, (label, value) in enumerate(rows):
        row_y = start_y + i * row_h
        selected = (i == _settings_selected_index)
        row_color = color.white if selected else color.smoke

        screen.pen = row_color
        screen.text(("> " if selected else "  ") + label, 6, row_y)

        vw, _vh = screen.measure_text(value)
        screen.text(value, screen.width - int(vw) - right_margin, row_y)

    hint = "C:select  UP/DN:adjust  B:back"
    screen.pen = color.smoke
    hw, hh = screen.measure_text(hint)
    if hw > screen.width - 8:
        hint = "C:sel UP/DN:adj B:back"
        hw, hh = screen.measure_text(hint)
    screen.text(hint, 4, screen.height - int(hh) - 3)


def update():
    global _screen

    _poll_gps()
    _update_fix_status()
    _maybe_sync_rtc()
    _maybe_log_gps()
    _update_leds()
    _update_ambient_dimming()
    _apply_ambient_level()

    if _screen == "clock":
        if badge.pressed(BUTTON_DOWN):
            _screen = "info1"
            return
        if badge.pressed(BUTTON_B):
            _screen = "settings"
            return
        if badge.pressed(BUTTON_A):
            _go_home()
            return
        _draw_clock_screen()
    elif _screen == "info1":
        if badge.pressed(BUTTON_DOWN):
            _screen = "info2"
            return
        if badge.pressed(BUTTON_UP):
            _screen = "clock"
            return
        _draw_info_screen_1()
    elif _screen == "info2":
        if badge.pressed(BUTTON_DOWN):
            _screen = "info3"
            return
        if badge.pressed(BUTTON_UP):
            _screen = "info1"
            return
        _draw_info_screen_2()
    elif _screen == "info3":
        if badge.pressed(BUTTON_UP):
            _screen = "info2"
            return
        _draw_info_screen_3()
    else:  # "settings"
        global _settings_selected_index

        if badge.pressed(BUTTON_B):
            _screen = "clock"
            return
        if badge.pressed(BUTTON_C):
            _settings_selected_index = (_settings_selected_index + 1) % _SETTINGS_FIELD_COUNT
            return
        if badge.pressed(BUTTON_UP):
            _adjust_selected_setting(1)
            return
        if badge.pressed(BUTTON_DOWN):
            _adjust_selected_setting(-1)
            return
        _draw_settings_screen()


# Call this explicitly rather than relying solely on the app loader to find
# it - on 2.0.2 the automatic init() detection doesn't always fire.
init()

run(update)