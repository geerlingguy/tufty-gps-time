# gps_time - GPS-synced clock for Badgeware (Tufty 2350)
#
# Shows UTC (large, ISO-style with trailing "Z") + local time below it,
# sourced from the internal RTC (kept accurate by periodic syncs from an
# Adafruit PA1010D GPS module over QWIIC/I2C).
#
# Top-left  : "NO FIX" (red) until the GPS has a valid time fix
# Top-right : "N sats" - red @ 0, orange @ 1-4, green @ 5+

import machine

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Hours to add to UTC to get local time, e.g. -5 for US Central.
# Half-hour offsets (e.g. 5.5) are fine too.
LOCAL_OFFSET = -5

# How often (in seconds) to push GPS time into the RTC once we have a fix.
RTC_SYNC_INTERVAL = 3600  # once per hour

# How long (seconds) we'll keep showing a fix as "valid" after the last
# good RMC sentence before falling back to "NO FIX".
GPS_FIX_TIMEOUT = 10

# PA1010D I2C address (fixed by the module, not configurable on the device).
GPS_I2C_ADDR = 0x10

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


# ---------------------------------------------------------------------------
# GPS / NMEA handling
# ---------------------------------------------------------------------------

def _handle_nmea_sentence(line):
    global gps_num_sats, gps_fix_valid, gps_last_fix_ticks, gps_datetime
    global gga_count, rmc_count, last_nmea_line

    last_nmea_line = line

    body = line.split("*")[0]
    fields = body.split(",")
    if not fields:
        return

    sentence_id = fields[0]

    if sentence_id.endswith("GGA"):
        gga_count += 1
        # $--GGA,time,lat,NS,lon,EW,fixquality,numSV,HDOP,alt,...
        if len(fields) > 7 and fields[7]:
            try:
                gps_num_sats = int(fields[7])
            except ValueError:
                pass

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

                dow = _day_of_week(year, month, day)

                gps_datetime = (year, month, day, hour, minute, second, dow)
                gps_fix_valid = True
                gps_last_fix_ticks = badge.ticks
            except ValueError:
                pass


def _poll_gps():
    global _nmea_buffer, i2c_read_errors, i2c_bytes_read, nmea_lines_seen

    if _i2c is None:
        return

    # Drain everything currently waiting instead of reading one fixed-size
    # chunk. The module streams continuously (~960 bytes/sec at 9600 baud)
    # regardless of how often we poll - if update() runs slower than that
    # (e.g. because rendering the scaled clock takes a while), reading only
    # one 32-byte chunk per frame falls further behind every second,
    # showing up as a growing display lag rather than a fixed latency.
    # Stop once a chunk comes back as all filler (nothing left to read) or
    # we hit the cap, so we don't spend forever draining on a bad frame.
    for _ in range(_I2C_MAX_READS_PER_POLL):
        try:
            chunk = _i2c.readfrom(GPS_I2C_ADDR, 32)
        except OSError:
            i2c_read_errors += 1
            return

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
    if gps_fix_valid and gps_last_fix_ticks is not None:
        if (badge.ticks - gps_last_fix_ticks) > GPS_FIX_TIMEOUT * 1000:
            gps_fix_valid = False


def _maybe_sync_rtc():
    global gps_last_sync_ticks

    if not rtc_available or not gps_fix_valid or gps_datetime is None:
        return

    now = badge.ticks
    if gps_last_sync_ticks is None or (now - gps_last_sync_ticks) >= RTC_SYNC_INTERVAL * 1000:
        try:
            rtc.datetime(gps_datetime)
            gps_last_sync_ticks = now
        except Exception:
            pass


def _get_display_datetime():
    """Time source for the clock display: prefer the RTC (kept accurate by
    GPS syncs), fall back to the last known GPS time if there's no RTC."""
    if rtc_available:
        try:
            return rtc.datetime()
        except Exception:
            pass
    return gps_datetime


# MTK NMEA output config: enable only GGA + RMC, once per fix, and disable
# GLL/VTG/GSA/GSV. The PA1010D sends sentences in a burst over its internal
# UART (9600 baud by default) once per second, and RMC is normally near the
# back of that burst - trimming it down to just the two sentences we need
# means RMC shows up much closer to the actual PPS edge instead of after
# several extra sentences worth of transmission time.
_PMTK_SET_NMEA_OUTPUT_RMCGGA = b"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
_PMTK_SET_NMEA_UPDATE_1HZ = b"$PMTK220,1000*1F\r\n"


def _configure_gps():
    if _i2c is None:
        return
    try:
        _i2c.writeto(GPS_I2C_ADDR, _PMTK_SET_NMEA_OUTPUT_RMCGGA)
    except OSError as e:
        print("gps_time: PMTK314 write failed:", e)
    try:
        _i2c.writeto(GPS_I2C_ADDR, _PMTK_SET_NMEA_UPDATE_1HZ)
    except OSError as e:
        print("gps_time: PMTK220 write failed:", e)


# ---------------------------------------------------------------------------
# App lifecycle
# ---------------------------------------------------------------------------

def init():
    global _i2c, rtc_available, i2c_scan_result

    try:
        _i2c = machine.I2C(I2C_ID, sda=machine.Pin(I2C_SDA_PIN), scl=machine.Pin(I2C_SCL_PIN), freq=I2C_FREQ)
        i2c_scan_result = _i2c.scan()
        print("gps_time: i2c devices found:", i2c_scan_result)
        _configure_gps()
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


def update():
    _poll_gps()
    _update_fix_status()
    _maybe_sync_rtc()

    screen.pen = color.navy
    screen.clear()

    dt = _get_display_datetime()

    if dt is not None:
        year, month, day, hour, minute, second, _dow = dt
        utc_str = _format_utc(hour, minute, second)

        ly, lm, ld, lh, lmin, lsec = _add_offset(year, month, day, hour, minute, second, LOCAL_OFFSET)
        local_str = "Local " + _format_12h(lh, lmin, lsec)
    else:
        utc_str = "--:--:--Z"
        local_str = "Local --:--:-- --"

    screen.pen = color.white

    # -- UTC clock: render at native size to an offscreen buffer, then
    # blit that buffer scaled up so it's bigger than any built-in font --
    screen.font = _time_font
    tw, th = screen.measure_text(utc_str)
    tw, th = int(tw), int(th)

    global _utc_buf, _utc_buf_size
    if _utc_buf is None or _utc_buf_size != (tw, th):
        _utc_buf = image(tw, th)
        _utc_buf_size = (tw, th)

    _utc_buf.pen = color.navy
    _utc_buf.clear()
    _utc_buf.pen = color.white
    _utc_buf.font = _time_font
    _utc_buf.text(utc_str, 0, 0)

    dest_w = int(tw * UTC_SCALE)
    dest_h = int(th * UTC_SCALE)
    x = int((screen.width - dest_w) / 2)
    y = int((screen.height - dest_h) / 2)
    screen.blit(_utc_buf, rect(x, y, dest_w, dest_h))

    # -- local time: dimmed a touch, in the lower part of the screen --
    screen.font = _local_font
    screen.pen = color.rgb(255, 255, 255, LOCAL_ALPHA)
    w2, h2 = screen.measure_text(local_str)
    w2 = int(w2)
    h2 = int(h2)
    x2 = int((screen.width - w2) / 2)
    y2 = int(screen.height * 0.75 - h2 / 2)
    screen.text(local_str, x2, y2)

    # -- corner indicators --
    screen.font = _label_font

    if not gps_fix_valid:
        screen.pen = color.red
        screen.text("NO FIX", 4, 4)

    sats_str = "{} sats".format(gps_num_sats)
    if gps_num_sats == 0:
        screen.pen = color.red
    elif gps_num_sats < 5:
        screen.pen = color.orange
    else:
        screen.pen = color.green

    sw, _sh = screen.measure_text(sats_str)
    sw = int(sw)
    screen.text(sats_str, screen.width - sw - 4, 4)

    # -- debug overlay --
    if DEBUG_OVERLAY:
        screen.font = _debug_font
        screen.pen = color.smoke

        line1 = "i2c:{} rtc:{} b:{} e:{}".format(
            i2c_scan_result, "T" if rtc_available else "F", i2c_bytes_read, i2c_read_errors
        )
        line2 = "ln:{} gga:{} rmc:{}".format(nmea_lines_seen, gga_count, rmc_count)

        _, lh = screen.measure_text(line1)
        lh = int(lh)
        screen.text(line1, 4, screen.height - (lh * 2) - 6)
        screen.text(line2, 4, screen.height - lh - 3)


# Call this explicitly rather than relying solely on the app loader to find
# it - on 2.0.2 the automatic init() detection doesn't always fire.
init()

run(update)