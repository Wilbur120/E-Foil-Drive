# main.py - eFoil Handheld Remote
# Pico 2 W: BLE throttle control + 0.96" ST7735 80x160 IPS display
#
# ONE PAGE. The layout never changes with the FOC state. Only values and
# colours change, so there are no full-screen redraws after boot.
#
# ========== WIRING ==========
#
#   Display         Pico 2 W
#   -------         --------
#   VCC        ->   3V3(OUT)  (pin 36)
#   GND        ->   GND       (pin 38 or any GND)
#   SCL (SCK)  ->   GP18      (SPI0 SCK, pin 24)
#   SDA (MOSI) ->   GP19      (SPI0 TX,  pin 25)
#   CS         ->   GP17      (SPI0 CSn, pin 22)
#   DC         ->   GP16      (pin 21)
#   RST        ->   GP20      (pin 26)
#   BLK        ->   GP21      (pin 27, PWM brightness control)
#
#   DRV5055A3QLPGM (TO-92, LPG package)
#   -----------------------------------
#   VCC        ->   3V3(OUT)  (pin 36), 100 nF to GND right at the sensor
#   GND        ->   GND
#   OUT        ->   GP26      (ADC0, pin 31), 1k series + 10 nF to GND at
#                             the Pico end if the run is longer than a few cm
#
#   Confirm pin order against the LPG drawing in the datasheet before you
#   solder. Magnet polarity does not matter: the scaling below is a signed
#   interpolation between two calibrated points, so a magnet facing the
#   other way just gives you a negative span.
#
# ========== LAYOUT ==========
#
#     +--------------------------------------------------+ y=0
#     | RUN              LAB               LINK          |
#     +--------------------------------------------------+ y=11
#     | THR  87%     | Vbus  48.6V                       |
#     | [=======   ] | Idc     52A                       |
#     |              | Pwr    2.5kW                      |
#     | RPM 2550     | Irms   118A                       |
#     |  estimated   | FET    45.2C                      |
#     +--------------------------------------------------+ y=79
#
# ========== PROTOCOL (matches the STM32 build as is) ==========
#
#   Pico -> STM32   "T{:05.1f}\n"   20 Hz, throttle percent
#
#   STM32 -> Pico   "<lead>,<st><fl>,<t10>,<rpm>,<idc>,<vb10>,<ia>,<ib>,<ic>\n"
#     lead   'S' normal build, 'L' LAB_TEST_MODE build
#     st     FOC state 0..5 (CAL IDLE ALIGN OPEN_LOOP CLOSED_LOOP FAULT)
#     fl     fault code 0..4 (none PHASE_OC OVERTEMP ACS_SAT DCLINK_OC)
#     t10    module temperature, tenths of a degree, 9999 = NTC fault
#     rpm    mechanical rpm, see RPM below
#     idc    DC link amps, -9999 = HASS never calibrated
#     vb10   bus volts in tenths, negative = bus sense fault
#     ia/ib/ic  per-phase RMS amps over a 100 ms window
#
#   The STM32 also sends "D,..." debug frames. This remote ignores them on
#   purpose. Nothing on this screen depends on them.
#
# ========== RPM, WHAT THE NUMBER ACTUALLY IS ==========
#
#   There is no encoder, so no RPM on this board is a measurement.
#     OPEN_LOOP    firmware sends the COMMANDED ramp speed. The rotor is
#                  being dragged and may not be following it.
#                  Shown as "commanded".
#     CLOSED_LOOP  firmware sends the PLL speed from the back-EMF observer.
#                  Shown as "estimated".
#     other states firmware sends 0 whether or not the prop is turning, so a
#                  coasting prop would read 0. Shown as "--" and
#                  "not tracked" rather than a false zero.
#
# ========== THROTTLE SENSING ==========
#
#   A DRV5055A3 is ratiometric and bipolar. With no field it sits at VCC/2
#   and moves 0.5 % of VCC per mT (the A3 is 25 mV/mT at 5 V). The sensor
#   runs off the same 3V3 rail that feeds ADC_VREF, so what the ADC sees is
#   that ratio and supply drift cancels. That is why COUNTS_PER_MT below has
#   no 3.3 in it and why the rail being 3.28 V does not shift the throttle.
#
#   Unlike a pot there are no end stops defining the range, so both ends of
#   travel are numbers you calibrate once and anything outside them is a
#   fault, not a throttle position. That is the one real gain over the pot:
#   a pot that loses its wiper wire reads as a valid position, this cannot.
#
# ========== CALIBRATING THE HALL THROTTLE ==========
#
#   Set HALL_CAL = True, run, watch the REPL.
#     1. Trigger released     -> B_REST_MT
#     2. Trigger at the stop  -> B_FULL_MT
#   Either can be negative and B_FULL can be below B_REST.
#   Aim for at least 30 mT of span. Keep BOTH ends at least 10 mT away from
#   0 mT, because a rest point sitting near 0 mT is indistinguishable from a
#   magnet that has fallen off its carrier. Keep the whole band inside
#   +-85 mT, which is where an A3 stays linear on a 3.3 V supply.
#   Then set HALL_CAL = False.
#
# ========== THROTTLE LOCK ==========
#
#   After every (re)connect the remote sends 0 % until the throttle has been
#   released below ARM_BELOW_PCT. Without this, a BLE dropout while holding
#   the throttle open would restart the drive the instant the link came
#   back. The display shows LOCK while this is holding.
#
#   A Hall fault also disarms, so clearing one needs a fresh release too.
#
# ========== WHAT KEEPS THIS FROM BREAKING ==========
#
#   1. aioble keeps only ONE pending notification per characteristic and
#      overwrites it. The HM-10 sends 20-byte notifications, so every frame
#      arrives in two or more pieces, often in the same connection event, and
#      the first piece is overwritten before any task can read it.
#      The queue is widened to NOTIFY_QUEUE_LEN, BUT char.notified() MUST NOT
#      be used with a widened queue. It only raises its wake flag when the
#      queue goes from empty to one item, and the flag is cleared when the
#      waiting task resumes. A second piece that lands between the IRQ and
#      that resume leaves one item queued with the flag clear, the next
#      notified() waits forever, and no later notification can wake it
#      because the queue is never empty again. Telemetry freezes while the
#      throttle writes keep working. That is the NO TLM fault.
#      So the receiver POLLS the queue every RX_POLL_MS and never waits on
#      the flag. If a future aioble renames the queue, it falls back to
#      notified() on the default one-slot queue: lossy, but cannot deadlock.
#      Glyphs are cached so a redraw does not starve the poll.
#   2. The old disconnect call passed timeout_ms to asyncio.wait_for, which
#      raised TypeError into a bare except, so the Pico never actually
#      disconnected. The HM-10 then stayed connected and stopped advertising,
#      and the rescan could not find it. Now awaited directly.
#   3. A malformed byte in a notification used to raise out of the receiver
#      and kill it silently while the throttle kept writing, leaving the
#      screen stale forever. Bad lines are now dropped one at a time, and if
#      the receiver ever does exit the link is torn down and rebuilt.
#   4. A single failed throttle write no longer drops the link.
#   4b. Backstop: if no valid status frame has arrived for RX_SILENT_MS AND
#      the throttle being sent is closed, the link is rebuilt. It never does
#      this with the throttle open, because silent telemetry is a display
#      fault and cutting thrust on the water over it would be worse.
#   5. The screen repaints all furniture and fields every REFRESH_MS without
#      clearing it, so a corrupted cell cannot persist.
#   6. Every Hall sample is a median of three reads and is range checked.
#      A reading outside the calibrated travel plus B_MARGIN_MT for
#      HALL_BAD_N samples in a row means lost supply, open OUT, dead sensor
#      or a lost magnet: throttle goes to 0, the arm flag drops and the
#      screen shows HALL in red. Clearing it needs HALL_GOOD_N good samples
#      AND a release, and the filter is seeded with the live reading on
#      recovery so a stale 0 % can never be what arms it. The BLE link is
#      deliberately left up, because 0 % at 20 Hz stops the drive more
#      gracefully than dropping the link and waiting out the dead-man.
#
# ========== FILE LAYOUT ON PICO ==========
#
#   /lib/ST7735S/ST7735.py    (existing driver)
#   /lib/ST7735S/sysfont.py   (existing font)
#   /main.py                  (this file)
#

import sys
sys.path.append("/lib/ST7735S")

import aioble
import bluetooth
import asyncio
import time
from collections import deque
from machine import ADC, Pin, SPI, PWM
from ST7735 import TFT, TFTColor
from sysfont import sysfont

# ----------------------------------------------------------------
#  BLE configuration (HM-10 / AT-09 on the eFoil board)
# ----------------------------------------------------------------
HM10_SERVICE = bluetooth.UUID(0xFFE0)
HM10_CHAR    = bluetooth.UUID(0xFFE1)
HM10_NAME    = "HMSoft"

THR_SEND_MS      = 50       # 20 Hz. STM32 dead-man is 500 ms.
WRITE_FAIL_MAX   = 4        # consecutive failed writes before reconnecting
NOTIFY_QUEUE_LEN = 32       # see note 1 in the header
RX_POLL_MS       = 10       # ~30 notifications/s running, so 1 or 0 per poll
RX_BUF_MAX       = 160      # bytes without a newline before resync
RX_SILENT_MS     = 3000     # no valid frame this long, throttle closed: rebuild
THR_CLOSED_PCT   = 5.0      # matches the STM32 THR_STOP_PCT

TELEM_STALE_MS = 1000       # status arrives every 100 ms running, 250 idle
IDC_ABSENT     = -9999

# ----------------------------------------------------------------
#  Throttle: DRV5055A3QLPGM on ADC0
# ----------------------------------------------------------------
ARM_BELOW_PCT = 3.0         # below the STM32 stop threshold of 5 %

hall = ADC(Pin(26))

ADC_FS        = 65535
COUNTS_PER_MT = 0.005 * ADC_FS   # A3 is 0.5 %VCC per mT -> 327.7 counts/mT
ZERO_COUNTS   = ADC_FS // 2      # nominal VCC/2. The sensor's own quiescent
                                 # offset is a couple of mT and is absorbed
                                 # by B_REST_MT, which is measured in place.

# --- calibration, see the header ---
B_REST_MT = 53.0            # field with the trigger released
B_FULL_MT = 67.0            # field at the full throttle stop

B_MARGIN_MT = 12.0          # slack outside travel before it counts as a fault
HALL_BAD_N  = 4             # consecutive bad samples (25 ms each) to fault
HALL_GOOD_N = 8             # consecutive good samples to clear it

HALL_CAL = False            # True prints live mT and counts at 4 Hz

THR_IIR = 0.35

_B_SPAN = B_FULL_MT - B_REST_MT
_B_LO   = min(B_REST_MT, B_FULL_MT) - B_MARGIN_MT
_B_HI   = max(B_REST_MT, B_FULL_MT) + B_MARGIN_MT

# ----------------------------------------------------------------
#  Colour thresholds. Display only, copies of firmware and pack numbers.
# ----------------------------------------------------------------
TEMP_WARN_C    = 60.0
TEMP_ALERT_C   = 80.0       # firmware trips at 90 C
IDC_WARN_A     = 150.0
IDC_ALERT_A    = 200.0      # firmware IDC_TRIP_A
IPH_RMS_CONT_A = 115.5      # firmware single DC link cap continuous limit
IMBALANCE_FRAC = 0.25
IMBALANCE_MIN_A = 10.0
PACK_EMPTY_V   = 39.6       # 12S at 3.3 V/cell
PACK_LOW_V     = 42.0       # 12S at 3.5 V/cell

DISPLAY_MS = 100
REFRESH_MS = 10000

# ----------------------------------------------------------------
#  Display setup - SPI0 on Pico 2 W
# ----------------------------------------------------------------
spi = SPI(0, baudrate=20000000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19))

PIN_DC  = 16
PIN_RST = 20
PIN_CS  = 17

backlight = PWM(Pin(21))
backlight.freq(1000)

def set_brightness(percent):
    duty = int(percent / 100.0 * 65535)
    backlight.duty_u16(max(0, min(65535, duty)))

set_brightness(100)

tft = TFT(spi, PIN_DC, PIN_RST, PIN_CS)

tft._size = (80, 160)
tft._offset[0] = 26
tft._offset[1] = 1
tft.initr()
tft.invertcolor(True)
tft.rgb(False)
tft.rotation(1)
tft._offset[0] = 1
tft._offset[1] = 26

SCR_W, SCR_H = 160, 80

# ----------------------------------------------------------------
#  Palette
# ----------------------------------------------------------------
COL_BG    = TFT.BLACK
COL_VAL   = TFT.WHITE
COL_LABEL = TFTColor(0x78, 0x80, 0x90)
COL_DIV   = TFTColor(0x28, 0x2C, 0x34)
COL_OK    = TFTColor(0x00, 0xE0, 0x60)
COL_WARN  = TFTColor(0xFF, 0xC0, 0x00)
COL_ALERT = TFTColor(0xFF, 0x30, 0x30)
COL_ACC   = TFTColor(0x00, 0xC8, 0xFF)

# ----------------------------------------------------------------
#  Text rendering with a glyph cache
#
#  Each character cell (glyph plus background) is built once per
#  (char, colour, scale) and pushed with a single image() call. After the
#  first few seconds almost every draw is a cache hit, which is what keeps
#  the event loop free for the BLE receiver.
# ----------------------------------------------------------------
_FW     = sysfont["Width"]
_FH     = sysfont["Height"]
_FSTART = sysfont["Start"]
_FEND   = sysfont["End"]
_FDATA  = sysfont["Data"]

_GLYPH_MAX = 300
_glyphs = {}

def _glyph(ch, color, scale):
    key = (ch, color, scale)
    g = _glyphs.get(key)
    if g is not None:
        return g
    if len(_glyphs) >= _GLYPH_MAX:
        _glyphs.clear()

    cw = (_FW + 1) * scale
    chh = _FH * scale
    buf = bytearray(2 * cw * chh)
    bh = COL_BG >> 8
    bl = COL_BG & 0xFF
    if bh or bl:
        for i in range(0, len(buf), 2):
            buf[i] = bh
            buf[i + 1] = bl

    hi = color >> 8
    lo = color & 0xFF
    row = cw * 2
    code = ord(ch)
    if code < _FSTART or code > _FEND:
        code = 63                               # '?'
    ci = (code - _FSTART) * _FW
    for q in range(_FW):
        bits = _FDATA[ci + q]
        xo = q * scale * 2
        for r in range(_FH):
            if bits & 1:
                for sy in range(scale):
                    p = (r * scale + sy) * row + xo
                    for sx in range(scale):
                        buf[p] = hi
                        buf[p + 1] = lo
                        p += 2
            bits >>= 1

    g = (buf, cw, chh)
    _glyphs[key] = g
    return g

def draw_text(x, y, s, color, scale=1):
    for ch in s:
        buf, cw, chh = _glyph(ch, color, scale)
        tft.image(x, y, x + cw - 1, y + chh - 1, buf)
        x += cw

# Field cache: only redraw when text or colour changed. The width is
# enforced here, so a short string can never leave fragments of a longer
# one behind. The cache entry is written AFTER the draw, so a draw that
# raises is retried next pass instead of being remembered as done.
_last = {}

def field(key, x, y, s, color, width, scale=1):
    n = len(s)
    if n < width:
        s = " " * (width - n) + s
    elif n > width:
        s = s[:width]
    prev = _last.get(key)
    if prev is not None and prev[0] == s and prev[1] == color:
        return
    draw_text(x, y, s, color, scale)
    _last[key] = (s, color)

# ----------------------------------------------------------------
#  Layout
# ----------------------------------------------------------------
HDR_Y = 11
DIV_X = 86

STATE_X, STATE_Y = 2, 2         # 6 chars -> 2..37
LAB_X,   LAB_Y   = 62, 2        # 4 chars -> 62..85
LINK_X,  LINK_Y  = 124, 2       # 6 chars -> 124..159

THR_LX, THR_LY = 2, 18
THR_VX, THR_VY = 26, 14         # scale 2, 4 chars -> 26..73, 14..29
BAR_X, BAR_Y, BAR_W, BAR_H = 3, 33, 80, 6

RPM_LX, RPM_LY = 2, 49
RPM_VX, RPM_VY = 26, 45         # scale 2, 4 chars -> 26..73, 45..60
SRC_X,  SRC_Y  = 2, 67          # 12 chars -> 2..73

R_LX, R_VX = 92, 118            # label 92..115, value 118..153
ROW_Y = (15, 28, 41, 54, 67)
R_LABELS = ("Vbus", "Idc", "Pwr", "Irms", "FET")

ST_OPENLP = 3
ST_CLOSED = 4

STATE_TXT = ("CAL", "IDLE", "ALIGN", "START", "RUN", "FAULT")
FAULT_TXT = ("", "OC PHS", "OVRTMP", "SENSOR", "OC DC")

def draw_static():
    """Furniture only. Drawn over the top of itself on every refresh, never
    cleared, so it does not flicker."""
    tft.hline((0, HDR_Y), SCR_W, COL_DIV)
    tft.vline((DIV_X, HDR_Y + 2), SCR_H - HDR_Y - 2, COL_DIV)
    tft.rect((BAR_X - 1, BAR_Y - 1), (BAR_W + 2, BAR_H + 2), COL_DIV)
    draw_text(THR_LX, THR_LY, "THR", COL_LABEL)
    draw_text(RPM_LX, RPM_LY, "RPM", COL_LABEL)
    for i in range(len(R_LABELS)):
        draw_text(R_LX, ROW_Y[i], R_LABELS[i], COL_LABEL)

# ---- throttle bar: only the changed slice is repainted ----
_bar_w   = 0
_bar_col = None

def reset_bar():
    global _bar_w, _bar_col
    _bar_w = 0
    _bar_col = None

def draw_bar(pct, col):
    global _bar_w, _bar_col
    w = int(pct * BAR_W / 100.0)
    w = 0 if w < 0 else (BAR_W if w > BAR_W else w)
    if col != _bar_col:
        if w:
            tft.fillrect((BAR_X, BAR_Y), (w, BAR_H), col)
        if w < BAR_W:
            tft.fillrect((BAR_X + w, BAR_Y), (BAR_W - w, BAR_H), COL_BG)
        _bar_col = col
        _bar_w = w
        return
    if w > _bar_w:
        tft.fillrect((BAR_X + _bar_w, BAR_Y), (w - _bar_w, BAR_H), col)
    elif w < _bar_w:
        tft.fillrect((BAR_X + w, BAR_Y), (_bar_w - w, BAR_H), COL_BG)
    _bar_w = w

# ----------------------------------------------------------------
#  Shared state
# ----------------------------------------------------------------
throttle_pct  = 0.0
thr_armed     = False
hall_ok       = False       # starts false: the sensor has to prove itself
ble_connected = False
last_tel_ms   = None

tel = {}

def clear_telemetry():
    global last_tel_ms
    tel["state"] = None
    tel["fault"] = 0
    tel["lab"] = False
    tel["temp"] = None
    tel["temp_err"] = False
    tel["rpm"] = None
    tel["idc"] = None
    tel["vbus"] = None
    tel["vbus_err"] = False
    tel["ia"] = None
    tel["ib"] = None
    tel["ic"] = None
    last_tel_ms = None

clear_telemetry()

# ----------------------------------------------------------------
#  Frame parsing. Status frames only.
# ----------------------------------------------------------------
def _parse_line(line):
    global last_tel_ms
    if len(line) < 2:
        return
    lead = line[0]
    if lead != "S" and lead != "L":
        return                      # 'D' debug frames dropped on purpose

    p = line.split(",")
    if len(p) != 9 or len(p[1]) != 2:
        return
    try:
        st   = int(p[1][0])
        fl   = int(p[1][1])
        t10  = int(p[2])
        rpm  = int(p[3])
        idc  = int(p[4])
        vb10 = int(p[5])
        ia   = int(p[6])
        ib   = int(p[7])
        ic   = int(p[8])
    except (ValueError, IndexError):
        return
    if st > 5 or fl > 4:
        return

    # Whole frame validated before anything is written, so the display can
    # never see half of one frame and half of the previous.
    temp_err = (t10 == 9999) or (abs(t10) >= 2000)
    tel["lab"] = (lead == "L")
    tel["state"] = st
    tel["fault"] = fl
    tel["temp_err"] = temp_err
    tel["temp"] = None if temp_err else t10 / 10.0
    tel["vbus_err"] = vb10 < 0
    tel["vbus"] = None if vb10 < 0 else vb10 / 10.0
    tel["idc"] = None if idc <= IDC_ABSENT else idc
    tel["rpm"] = rpm
    tel["ia"] = ia
    tel["ib"] = ib
    tel["ic"] = ic
    last_tel_ms = time.ticks_ms()

# ----------------------------------------------------------------
#  Formatting and colour rules
# ----------------------------------------------------------------
def _fmt_pwr(vb, idc):
    kw = vb * idc / 1000.0
    if abs(kw) < 9.95:
        return "{:4.1f}kW".format(kw)
    kw = -99.0 if kw < -99.0 else (99.0 if kw > 99.0 else kw)
    return "{:4.0f}kW".format(kw)

def _col_vbus(vb, lab):
    # A bench supply is deliberately below pack voltage, so pack
    # thresholds are meaningless in lab mode.
    if lab:
        return COL_VAL
    if vb < PACK_EMPTY_V:
        return COL_ALERT
    if vb < PACK_LOW_V:
        return COL_WARN
    return COL_VAL

def _col_idc(idc):
    if idc > IDC_ALERT_A:
        return COL_ALERT
    if idc > IDC_WARN_A:
        return COL_WARN
    return COL_ACC

def _col_irms(ia, ib, ic):
    """Amber if a phase is out of balance (dropped phase or dead sensor) or
    the hottest phase is above what one DC link cap carries continuously.
    The firmware derates for the second one on its own; this just shows it."""
    m = max(ia, ib, ic)
    if m > IPH_RMS_CONT_A:
        return COL_WARN
    mean = (ia + ib + ic) / 3.0
    if mean >= IMBALANCE_MIN_A:
        lim = IMBALANCE_FRAC * mean
        if abs(ia - mean) > lim or abs(ib - mean) > lim or abs(ic - mean) > lim:
            return COL_WARN
    return COL_VAL

def _col_temp(t):
    if t >= TEMP_ALERT_C:
        return COL_ALERT
    if t >= TEMP_WARN_C:
        return COL_WARN
    return COL_VAL

# ----------------------------------------------------------------
#  Rendering
# ----------------------------------------------------------------
async def render():
    now = time.ticks_ms()
    t = tel.copy()                  # one consistent frame for this pass
    fresh = (ble_connected and last_tel_ms is not None and
             time.ticks_diff(now, last_tel_ms) < TELEM_STALE_MS)
    lab = fresh and t["lab"]
    st = t["state"] if fresh else None
    fl = t["fault"] if fresh else 0

    # ---- throttle, every pass ----
    # HALL outranks everything else here. A dead sensor is not a throttle
    # position, and 0 % on its own would look like a released trigger.
    pct = throttle_pct
    if not hall_ok:
        thr_s, thr_c, bar_c = "HALL", COL_ALERT, COL_LABEL
    elif not ble_connected:
        thr_s, thr_c, bar_c = "{:3d}%".format(int(pct + 0.5)), COL_LABEL, COL_LABEL
    elif not thr_armed:
        thr_s, thr_c, bar_c = "LOCK", COL_WARN, COL_LABEL
    else:
        thr_s, thr_c = "{:3d}%".format(int(pct + 0.5)), COL_VAL
        bar_c = COL_OK if pct < 50 else (COL_WARN if pct < 80 else COL_ALERT)
    field("thr", THR_VX, THR_VY, thr_s, thr_c, 4, 2)
    draw_bar(pct, bar_c)
    await asyncio.sleep_ms(0)

    # ---- header ----
    if not ble_connected:
        link_s, link_c = "NO BLE", COL_ALERT
    elif not fresh:
        link_s, link_c = "NO TLM", COL_WARN
    else:
        link_s, link_c = "LINK", COL_OK
    field("link", LINK_X, LINK_Y, link_s, link_c, 6)
    field("lab", LAB_X, LAB_Y, "LAB" if lab else "", COL_WARN, 4)

    if st is None:
        st_s, st_c = "--", COL_LABEL
    elif fl:
        st_s, st_c = FAULT_TXT[fl], COL_ALERT
    else:
        st_s = STATE_TXT[st]
        st_c = COL_OK if st == ST_CLOSED else (
               COL_ACC if st == ST_OPENLP else COL_LABEL)
    # Left aligned: pad on the right by hand, field() pads on the left.
    field("state", STATE_X, STATE_Y, (st_s + "      ")[:6], st_c, 6)
    await asyncio.sleep_ms(0)

    # ---- RPM and where it came from ----
    if st == ST_CLOSED:
        rpm_s, rpm_c = "{:4d}".format(min(max(t["rpm"], 0), 9999)), COL_VAL
        src_s, src_c = " estimated  ", COL_OK
    elif st == ST_OPENLP:
        rpm_s, rpm_c = "{:4d}".format(min(max(t["rpm"], 0), 9999)), COL_VAL
        src_s, src_c = " commanded  ", COL_ACC
    else:
        rpm_s, rpm_c = "--", COL_LABEL
        src_s, src_c = "not tracked ", COL_LABEL
    field("rpm", RPM_VX, RPM_VY, rpm_s, rpm_c, 4, 2)
    field("src", SRC_X, SRC_Y, src_s, src_c, 12)
    await asyncio.sleep_ms(0)

    # ---- right column ----
    # Vbus. NO V means the STM32 cannot measure the bus at all, which is
    # different from a flat pack. The drive will not start while it shows.
    vb = t["vbus"] if fresh else None
    if not fresh:
        v_s, v_c = "--", COL_LABEL
    elif t["vbus_err"]:
        v_s, v_c = "NO V", COL_ALERT
    else:
        v_s, v_c = "{:5.1f}V".format(vb), _col_vbus(vb, lab)
    field("vbus", R_VX, ROW_Y[0], v_s, v_c, 6)

    # Idc. "--" covers both a stale link and an uncalibrated HASS.
    idc = t["idc"] if fresh else None
    if idc is None:
        field("idc", R_VX, ROW_Y[1], "--", COL_LABEL, 6)
    else:
        field("idc", R_VX, ROW_Y[1], "{:d}A".format(idc), _col_idc(idc), 6)

    # Power, only when both inputs are real measurements.
    if vb is None or idc is None:
        field("pwr", R_VX, ROW_Y[2], "--", COL_LABEL, 6)
    else:
        field("pwr", R_VX, ROW_Y[2], _fmt_pwr(vb, idc), COL_VAL, 6)
    await asyncio.sleep_ms(0)

    # Irms, the highest of the three phases.
    if fresh and t["ia"] is not None:
        ia, ib, ic = t["ia"], t["ib"], t["ic"]
        field("irms", R_VX, ROW_Y[3], "{:d}A".format(max(ia, ib, ic)),
              _col_irms(ia, ib, ic), 6)
    else:
        field("irms", R_VX, ROW_Y[3], "--", COL_LABEL, 6)

    # Module temperature. In lab mode an unplugged thermistor is expected,
    # so it is grey OFF rather than red ERR.
    if not fresh:
        tp_s, tp_c = "--", COL_LABEL
    elif t["temp_err"]:
        tp_s, tp_c = ("OFF", COL_LABEL) if lab else ("ERR", COL_ALERT)
    else:
        tp_s, tp_c = "{:5.1f}C".format(t["temp"]), _col_temp(t["temp"])
    field("temp", R_VX, ROW_Y[4], tp_s, tp_c, 6)

# ----------------------------------------------------------------
#  Tasks
# ----------------------------------------------------------------
def read_hall():
    """Median of three ADC reads, returned as (counts, millitesla).

    The median is not about magnetic noise, it is about single ADC samples
    landing well away from the others. One of those getting through the
    range check would be enough to trip a fault on a healthy sensor."""
    a = hall.read_u16()
    b = hall.read_u16()
    c = hall.read_u16()
    raw = a + b + c - min(a, b, c) - max(a, b, c)
    return raw, (raw - ZERO_COUNTS) / COUNTS_PER_MT

async def throttle_task():
    global throttle_pct, thr_armed, hall_ok
    bad = 0
    good = 0
    cal_ms = time.ticks_ms()
    while True:
        try:
            raw, bmt = read_hall()

            if _B_LO <= bmt <= _B_HI:
                pct = (bmt - B_REST_MT) * 100.0 / _B_SPAN
                pct = 0.0 if pct < 0.0 else (100.0 if pct > 100.0 else pct)
                bad = 0
                if hall_ok:
                    throttle_pct = throttle_pct + THR_IIR * (pct - throttle_pct)
                else:
                    # Coming back from a fault there is no filter history
                    # worth keeping. Seed it with the live reading so the
                    # arming test below cannot see a stale 0 % while the
                    # trigger is actually held open.
                    throttle_pct = pct
                    good += 1
                    if good >= HALL_GOOD_N:
                        good = 0
                        hall_ok = True
            else:
                good = 0
                if bad < HALL_BAD_N:
                    bad += 1
                    if bad >= HALL_BAD_N and hall_ok:
                        hall_ok = False
                        thr_armed = False
                        print("hall out of range: {:.1f} mT".format(bmt))
                if not hall_ok:
                    throttle_pct = 0.0

            # Arms on the FILTERED value, so it takes a real release to arm,
            # not one noisy sample.
            if (ble_connected and hall_ok and not thr_armed and
                    throttle_pct < ARM_BELOW_PCT):
                thr_armed = True

            if HALL_CAL and time.ticks_diff(time.ticks_ms(), cal_ms) >= 250:
                cal_ms = time.ticks_ms()
                print("B {:7.2f} mT   counts {:5d}".format(bmt, raw))
        except Exception as e:
            print("throttle:", e)
        await asyncio.sleep_ms(25)

async def display_task():
    need_refresh = True
    last_refresh = time.ticks_ms()
    while True:
        try:
            now = time.ticks_ms()
            if need_refresh or time.ticks_diff(now, last_refresh) >= REFRESH_MS:
                need_refresh = False
                last_refresh = now
                _last.clear()
                reset_bar()
                draw_static()
                await asyncio.sleep_ms(0)
            await render()
        except Exception as e:
            print("render:", e)
            need_refresh = True     # repaint everything next pass
        await asyncio.sleep_ms(DISPLAY_MS)

async def find_hm10():
    print("Scanning for HM-10...")
    try:
        async with aioble.scan(duration_ms=10000, interval_us=30000,
                               window_us=30000, active=True) as scanner:
            async for result in scanner:
                if result.name() == HM10_NAME:
                    return result.device
    except Exception as e:
        print("scan:", e)
    return None

async def telem_receiver(char, queue, link):
    """Reassembles lines from 20-byte HM-10 notifications. A bad line is
    dropped on its own and never stops the loop. If this task ever exits,
    link[0] goes False and main() rebuilds the connection.

    queue is the widened aioble notify deque, read directly. It is filled by
    the BLE IRQ, which runs between bytecodes on this same thread, so a
    len() check followed by popleft() cannot be interleaved with an append.
    queue is None only if aioble has renamed its internals, in which case
    the default one-slot queue and notified() are used instead."""
    rx = b""
    try:
        await char.subscribe(notify=True)
        while True:
            if queue is None:
                data = await char.notified()
                chunks = (data,) if data else ()
            else:
                if len(queue) == 0:
                    await asyncio.sleep_ms(RX_POLL_MS)
                    continue
                chunks = []
                while len(queue):
                    chunks.append(queue.popleft())

            for data in chunks:
                rx += bytes(data)
            while True:
                i = rx.find(b"\n")
                if i < 0:
                    break
                line = rx[:i]
                rx = rx[i + 1:]
                try:
                    _parse_line(line.decode().strip())
                except Exception:
                    pass
            if len(rx) > RX_BUF_MAX:
                rx = b""            # no newline in sight, resync
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print("telem_receiver:", e)
    finally:
        link[0] = False

# ----------------------------------------------------------------
#  Main
# ----------------------------------------------------------------
async def main():
    global ble_connected, thr_armed

    tft.fill(COL_BG)                # the only full-screen clear, at boot
    asyncio.create_task(throttle_task())
    asyncio.create_task(display_task())

    while True:
        device = await find_hm10()
        if device is None:
            await asyncio.sleep_ms(1000)
            continue

        try:
            connection = await device.connect(timeout_ms=10000)
        except Exception as e:
            print("connect:", e)
            await asyncio.sleep_ms(2000)
            continue

        recv_task = None
        link = [True]               # per-connection, so an old receiver
                                    # can never mark a new link dead
        try:
            service = await connection.service(HM10_SERVICE)
            if service is None:
                raise RuntimeError("no FFE0 service")
            char = await service.characteristic(HM10_CHAR)
            if char is None:
                raise RuntimeError("no FFE1 characteristic")

            # aioble default is a one-slot queue that overwrites. Widen it
            # before subscribing, and hand the SAME object to the receiver,
            # which polls it. See note 1 for why notified() is not used.
            if hasattr(char, "_notify_queue"):
                queue = deque((), NOTIFY_QUEUE_LEN)
                char._notify_queue = queue
            else:
                queue = None
                print("aioble internals changed, using lossy notified()")

            thr_armed = False
            clear_telemetry()
            ble_connected = True
            recv_task = asyncio.create_task(telem_receiver(char, queue, link))
            t_connect = time.ticks_ms()
            print("Connected")

            fails = 0
            while connection.is_connected() and link[0]:
                # hall_ok is checked here as well as in throttle_task. The
                # task zeroes throttle_pct on a fault, this makes sure a
                # fault between the two can still only send 0 %.
                cmd = throttle_pct if (thr_armed and hall_ok) else 0.0
                cmd = 0.0 if cmd < 0.0 else (100.0 if cmd > 100.0 else cmd)

                # Backstop, note 4b. Only with the throttle closed.
                now = time.ticks_ms()
                t_ref = t_connect if last_tel_ms is None else last_tel_ms
                if (cmd < THR_CLOSED_PCT and
                        time.ticks_diff(now, t_ref) > RX_SILENT_MS):
                    print("telemetry silent, rebuilding link")
                    break
                try:
                    await char.write("T{:05.1f}\n".format(cmd).encode(),
                                     response=False)
                    fails = 0
                except Exception as e:
                    fails += 1
                    print("write:", e)
                    if fails >= WRITE_FAIL_MAX:
                        break
                await asyncio.sleep_ms(THR_SEND_MS)

        except Exception as e:
            print("link:", e)
        finally:
            ble_connected = False
            thr_armed = False
            clear_telemetry()
            if recv_task is not None:
                recv_task.cancel()
            try:
                await connection.disconnect(timeout_ms=2000)
            except Exception:
                pass
            await asyncio.sleep_ms(1000)

asyncio.run(main())

