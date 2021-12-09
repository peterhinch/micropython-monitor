# analyser.py Low cost back end for micropython-monitor

import hardware_setup  # Create a display instance
from array import array
import gc
gc.collect()
EVLEN = const(2048)  # Must be 2**N for modulo with EVMSK
TAIL = const(1024)  # No of events to store after trigger
EVMSK = const(EVLEN - 1)
evt_buf = array('I', (0 for _ in range(EVLEN)))

from gui.core.ugui import Screen, ssd, LinearIO
from gui.widgets.label import Label
from gui.widgets.buttons import Button, CloseButton, CIRCLE
from gui.widgets.dropdown import Dropdown
from gui.widgets.adjuster import Adjuster, FloatAdj

from gui.core.writer import CWriter
# Font for CWriter
import gui.fonts.arial10 as arial10
from gui.core.colors import *

from time import ticks_us, ticks_diff
import rp2
from machine import UART, Pin, Timer
from random import getrandbits
import uasyncio as asyncio
gc.collect()

dolittle = lambda *_ : None
GRIDCOLOR = create_color(12, 50, 150, 50)

# ****** SPI support ******
@rp2.asm_pio(autopush=True, in_shiftdir=rp2.PIO.SHIFT_LEFT, push_thresh=8)
def spi_in():
    label("escape")
    set(x, 0)
    mov(isr, x)  # Zero after DUT crash
    wrap_target()
    wait(1, pins, 2)  # CS/ False
    wait(0, pins, 2)  # CS/ True
    set(x, 7)
    label("bit")
    wait(0, pins, 1)
    wait(1, pins, 1)
    in_(pins, 1)
    jmp(pin, "escape")  # DUT crashed. On restart it sends a char with CS high.
    jmp(x_dec, "bit")  # Post decrement
    wrap()


class PIOSPI:
    def __init__(self):
        self._sm = rp2.StateMachine(
            0,
            spi_in,
            in_shiftdir=rp2.PIO.SHIFT_LEFT,
            push_thresh=8,
            in_base=Pin(0),
            jmp_pin=Pin(2, Pin.IN, Pin.PULL_UP),
        )
        self._sm.active(1)

    # Nonblocking read of 1 char. Returns ord(ch) or None.
    def read(self):
        if self._sm.rx_fifo():
            return self._sm.get() & 0xFF


# ****** Monitor ******

class Channel:
    def __init__(self, graph, nbit):
        self.graph = graph
        self.row = graph.row + nbit * graph.dr
        self.nbit = nbit
        self.height = graph.dr - 2  # Allow for spacing between lines

    def show(self):
        width = self.graph.width
        c = self.graph.fgcolor
        x = self.graph.col
        ht = self.height
        yon = self.row + 2
        yoff = self.row + ht - 2
        ssd.hline(x, self.row + ht + 1, width, self.graph.gridcolor)
        on = self.graph.on
        off = self.graph.off
        mask = 1 << self.nbit
        state = 0  # 0 not yet known, 1 high, 2 low
        pstate = 0
        for n in range(width):
            if on[n] & mask:
                state = 1
                ssd.pixel(x + n, yon, c)
            if off[n] & mask:
                state = 2
                ssd.pixel(x + n, yoff, c)
            # Conditions for a vertical line
            # Draw if prior state was known and state has changed.
            if (pstate and (state ^ pstate)) or (off[n] & on[n] & mask):
                ssd.vline(x + n, yon, ht - 4, c)
            pstate = state


# The LA class provides a logic analyser display of 8 channels based on two
# bytearrys. Each column maps onto an ON byte and an OFF byte, with each bit
# corresponding to a channel. A bit may be ON, OFF, neither (unknown) or both
# (aliased data).
class LA(LinearIO):
    def __init__(self, writer, row, col, height, width,
                 fgcolor, bgcolor, bdcolor, gridcolor, curs_color, trig_color,
                 callback, args):
        fw = writer.font.max_width()  # Width of channel labels
        width -= fw
        lcol = col
        col += fw
        super().__init__(writer, row, col, height, width,
                         fgcolor, bgcolor, bdcolor, 0, True, None)
        super()._set_callbacks(callback, args)
        self.cpos = [-1, -1]  # Cursor position (column)
        self.trig_pos = -1
        self.ccolor = curs_color if curs_color is not None else self.fgcolor
        self.tcolor = trig_color if trig_color is not None else self.fgcolor
        self.gridcolor = gridcolor if gridcolor is not None else self.fgcolor
        self.on = bytearray(width)  # Data arrays: on pixels
        self.off = bytearray(width)  # Off pixels
        self.nrows = 8
        self.dr = round(height/self.nrows)
        hh = round(self.dr / 3)
        for n in range(self.nrows):
            Label(writer, row + self.dr * n + hh, lcol, chr(0x30 + n))
        self.lines = [Channel(self, n) for n in range(self.nrows)]

    def show(self):  # 56ms regardless of freq
        if super().show():  # Clear working area
            for line in self.lines:
                line.show()
        for x in range(2):
            if self.cpos[x] >= 0:  # Draw cursors if in window
                ssd.vline(self.cpos[x], self.row, self.height, self.ccolor)
        if self.trig_pos >= 0:
            ssd.vline(self.trig_pos, self.row, self.height, self.tcolor)

    def set_point(self, n, onoff):
        self.on[n] = onoff & 0xFF
        self.off[n] = onoff >> 8

    def clear(self):
        for n in range(self.width):
            self.on[n] = 0
            self.off[n] = 0

    def v_to_col(self, v):  # Given value 0..1 return equivalent column
        return self.col + round(v * (self.width -1))


class Monitor(LA):

    """An event consists of a 32 bit integer. Bits 31..4 comprise the time in us from
    the first sample captured (which may be lost). Bit 3 indicates bit set (1) or
    clear (0), bits 0-2 are the channel. This means the LS 4 bits of time are garbage
    but 16μs is negligible so time comparisons are done ignoring this.
    The onoff value stores the current state of all channels, bits 0-7 indicating ON
    channels and 8-15 OFF ones. A channel can be unknown with neither set or aliased
    with both set.
    sonoff takes current onoff value and modifies it with an event."""
    @staticmethod
    def sonoff(onoff, evt):
        bitset = 1 << (evt & 7)  # ON bit for current channel
        bitclr = 1 << ((evt & 7) + 8)  # OFF bit for current channel
        on = evt & 8
        bset = bitset if on else bitclr
        bclr = bitset if not on else bitclr
        return (onoff | bset) & ~bclr

    @staticmethod
    def mod_onoff(onoff, adt):  # Set both on and off bits for aliased channels
        if adt:
            for chan in range(8):
                if adt & 1:  # Alias detected for this channel
                    onoff |= (1 << chan) | (1 << (chan + 8))  # Set on and off bits
                adt >>= 1
        return onoff

    def __init__(self, writer, row, col, *, height, width, evtbuf,
                 fgcolor=None, bgcolor=None, bdcolor=None, gridcolor=None,
                 curs_color=None, trig_color=None, callback=dolittle, args=()):
        evtbuf.set_monitor(self)
        self.evtbuf = evtbuf
        # Times in μs from 1st sample. Set by EventBuf.acquire.
        self.t_end = 1_000_000  # Time of last sample
        self.t_ws = 0  # Display window start
        self.t_we = 100_000  # Display window end
        self.t_ww = 100_000  # Window width
        self.can_zoomout = True
        self.ttrig = 0  # Trigger time
        self.tcurs = [0, 0]  # Cursor times
        super().__init__(writer, row, col, height, width,
                 fgcolor, bgcolor, bdcolor, gridcolor, curs_color, trig_color,
                 callback, args)


    # Alias detection can be confusing, notably with trigger events which issue a short pulse.
    # This can be aliased if both transitions occur in one pixel.
    def redraw(self):
        self.draw = True  # Ensure physical refresh on return
        npoints = self.width
        uspp = round(self.t_ww / npoints)  # μs per point
        onoff = 0  # byte 0 = on bits, byte 1 = off bits
        for evt in (get:= self.evtbuf.get_event()):
            if evt:  # Ignore empty elements
                evt_time = (evt & ~0xF) | 8  # Mean time: replace channel and bit with avg
                if evt_time >= self.t_ws:
                    break  # Reached the window
                onoff = self.sonoff(onoff, evt)
        else:  # No data
            self.clear()  # Display an empty graph
            self.show()
            return
        # Reached 1st event in display window.
        for point in range(npoints):
            tp = self.t_ws + point * uspp  # Time corresponding to current point
            if evt == 0:  # Either reached end of buf or rest of contents are zero
                self.set_point(point, 0)  # No data
            else:
                aa = 0
                adt = 0
                while evt_time <= tp:  # Process events at or before current point
                    # Note that onoff tracks all events, maintaining current state of
                    # all channels even if aliasing occurs.
                    onoff = self.sonoff(onoff, evt)  # Update onoff and get next event
                    abit = (1 << (evt & 7))  # Unique bit for channel
                    if aa & abit:
                        adt |= abit  # Log all channels aliased on this point
                    aa |= abit
                    try:
                        evt = next(get)
                        evt_time = (evt & ~0xF) | 8  # Mean time
                    except StopIteration:
                        evt = 0  # Flag out of data
                        break
                # Set all channels for this point, modifying for aliased channels
                self.set_point(point, self.mod_onoff(onoff, adt))

        for n in range(2):
            t = self.tcurs[n]
            if self.t_ws <= t <= self.t_we:  # Trigger is in window
                self.cpos[n] = round(self.v_to_col((t - self.t_ws) / self.t_ww))
            else:
                self.cpos[n] = -1  # Not visible
        if self.t_ws <= self.ttrig <= self.t_we:  # Trigger is in window
            self.trig_pos = round(self.v_to_col((self.ttrig - self.t_ws) / self.t_ww))
        else:
            self.trig_pos = -1  # Not visible

    def zoom_minus(self):  # Widen window
        dt = self.t_ww // 2  # Width added at each end
        self.t_ww *= 2  # New width
        if (self.t_ws - dt) < 0:
            self.value(0)
            self.can_zoomout = False
        elif (self.t_we + dt) > self.t_end:
            self.value(1)
            self.can_zoomout = False
        else:
            self.scroll()

    def zoom_plus(self):  # Narrow window
        dt = self.t_ww // 4
        self.t_ws += dt
        self.t_we -= dt
        self.t_ww = self.t_we - self.t_ws
        self.can_zoomout = True

    def scroll(self):  # Move window
        self.t_ws = round(self.value() * (self.t_end - self.t_ww))  # Value expressed as a time
        self.t_we = self.t_ws + self.t_ww
        self.can_zoomout = True

    def home(self):  # Attempt to place trigger at screen centre
        t_ws = max(0, self.ttrig - round(self.t_ww / 2))
        t_we = t_ws + self.t_ww
        if t_we > self.t_end:
            t_ws = self.t_end - self.t_ww
        self.value(t_ws / (self.t_end - self.t_ww))

    def cursor(self, x, v):  # Adjuster has moved
        t = round(v * self.t_end)  # Target time
        if self.tcurs[x] == -1 and not (self.t_ws <= t <= self.t_we):
            # Moved for first time: put in middle of window
            t = self.t_ws + (self.t_we - self.t_ws) // 2
            return t / self.t_end
        self.tcurs[x] = t
        return self.tcurs

    def kill_cursors(self):
        self.tcurs[0] = -1
        self.tcurs[1] = -1


# ****** Acquisition ******

class EventBuf:

    def __init__(self, screen, device):
        self.screen = screen
        self.device = device
        self.wptr = 0  # Buffer write pointer
        self.tim = Timer()
        self.run = False
        self.monitor = None  # Point to Monitor instance
        hardware_setup.nxt.irq(handler=self.tcb, trigger=Pin.IRQ_FALLING)  # Next button stops capture
        if device == "uart" or device == "demo":
            uart = UART(0, 1_000_000)  # rx on GPIO 1

            def read():
                if uart.any():  # Nonblocking read
                    return ord(uart.read(1))

        elif device == "spi":
            pio = PIOSPI()

            def read():  # Nonblocking
                return pio.read()

        else:
            raise ValueError("Unsupported device:", device)
        self.read = read

    def set_monitor(self, mon):  # Called by Monitor ctor
        self.monitor = mon

    def tcb(self, _):  # Timer callback: normal termination
        self.run = False

    # ***** Process event buffer *****
    # This is a ringbuf. Writing may overwrite old samples until a trigger occurs,
    # when the contents are frozen. The oldest sample is then one after the write
    # pointer (modulo length).
    # Return an event, oldest first.
    def get_event(self):
        rptr = (self.wptr + 1) & EVMSK
        while rptr != self.wptr:
            yield evt_buf[rptr]
            rptr = (rptr + 1) & EVMSK

    def get_idx(self):  # Return index into evt_buf, oldest entry first
        rptr = (self.wptr + 1) & EVMSK
        while rptr != self.wptr:
            yield rptr
            rptr = (rptr + 1) & EVMSK

    def clear(self):
        for x in range(EVLEN):
            evt_buf[x] = 0
        self.wptr = 0

    def acquire(self, mode, t_ms):
        if self.device == "demo":
            self.demo_acquire(mode, t_ms)
        else:
            self.real_acquire(mode, t_ms)

    # Dummy data acquisition.
    def demo_acquire(self, mode, t_ms):
        for x in range(EVLEN // 2):
            evt_buf[x] = 512 * x + getrandbits(4)
        mon = self.monitor
        mon.t_end = 512 * x  # Time of last sample relative to 1st
        mon.t_ws = 0
        mon.t_we = mon.t_end // 10
        mon.t_ww = mon.t_we - mon.t_ws
        mon.can_zoomout = True
        mon.ttrig = mon.t_end // 2
        mon.kill_cursors()
        self.wptr = x  # Read from index 0

    # Adjust times of a buffer of samples such that oldest has time 0
    def normalise(self):
        ts = 0  # Time of oldest sample before normalisation
        for tp in self.get_idx():  # Generator returns index into evt_buf
            if ts:
                evt_buf[tp] -= ts
            elif (ts := (evt_buf[tp] & ~0xF)):  # Early samples may be 0 (no data)
                evt_buf[tp] -= ts
        dt = evt_buf[tp]  # Time of most recent sample relative to oldest
        mon = self.monitor
        mon.ttrig -= ts
        mon.t_end = dt
        mon.t_ws = 0
        mon.t_we = dt // 10  # Window 1/10 of data initially
        mon.t_ww = mon.t_we - mon.t_ws
        mon.can_zoomout = True
        mon.kill_cursors()
        print('Capture complete.')

    # Incoming data has channels up to 21. Store ch 0-7 in buffer.
    def save_event(self, x, tarr_us, t_start):
        if (x & 0x1f) < 8:
            dt = ticks_diff(tarr_us, t_start) & ~0xF  # Ensure no rollover
            evt_buf[self.wptr] = dt | x & 7 | (0 if (x & 0x20) else 8)
            self.wptr += 1
            self.wptr &= EVMSK

    # mode defines stop conditions. In any mode .abrt terminates immediately.
    # 0: Run forever (until .abrt)
    # 1: When Ch 0 width > t_ms (check program segment duration)
    # 2: When Ch 0 inactive for > t_ms (timer timeout sets .run False)
    # 3: Ch 0 inactive for > t_ms, stop when it is next active
    def real_acquire(self, mode, t_ms):
        self.clear()  # Zero all samples.
        self.run = True
        tim = self.tim
        t_us = t_ms * 1000
        while self.read() is not None:
            pass  # Discard any buffered characters
        h_start = -1  # Absolute time of last Ch0 leading edge: invalidate.
        t_start = ticks_us()  # Time reference for buffer
        tarr_us = t_start  # Arrival time of latest byte
        while self.run:
            if (x := self.read()) is not None:
                tarr_us = ticks_us()
                if x == 0x7A:  # Init: program under test has restarted
                    print("Got communication.")
                    h_start = -1
                    self.clear()
                    t_start = ticks_us()
                    continue

                self.save_event(x, tarr_us, t_start)  # Store in evt_buf.
                if not (x & 0x1F):  # Edge on channel 0
                    # Possible timeout on channel 0
                    ptout = h_start != -1 and ticks_diff(tarr_us, h_start) > t_us
                    if x == 0x40:  # Leading edge on ch 0.
                        if mode == 2:  # .run = False on absence of activity > t_ms
                            tim.init(period=t_ms, mode=Timer.ONE_SHOT, callback=self.tcb)
                        elif mode == 3 and ptout:
                            break  # Ch 0 edge, but inactivity exceeded threshold
                        h_start = tarr_us
                    if x == 0x60 and mode == 1:  # Ch 0 trailing edge: check pulse width
                        if ptout:
                            break  # Stop capture

        # Acquistion has ended. Collect 1s/half a buffer full of data.
        # Trigger time is arrival time of last byte or timer timeout.
        self.monitor.ttrig = ticks_diff(ticks_us(), t_start) & ~0xF
        begin = ticks_us()
        nch = 0
        while (ticks_diff(ticks_us(), begin) < 1_000_000) and (nch < TAIL):
            while (x:= self.read()) is not None:
                self.save_event(x, ticks_us(), t_start)
                nch += 1
        self.normalise()


# ***** GUI *****
class BaseScreen(Screen):  # Screen is 240*320

    def __init__(self, device):  # "uart", "spi" or "demo"
        super().__init__()
        self.initialised = False
        wri = CWriter(ssd, arial10, GREEN, BLACK, False)
        self.evtbuf = EventBuf(self, device)  # Store incoming data
        col = 2
        row = 2
        lw = 50  # Label width
        self.la = Monitor(wri, row, col, height=170, width=316,
                          bdcolor=GRIDCOLOR, gridcolor=GRIDCOLOR,
                          curs_color=RED, trig_color=BLUE, 
                          callback=self.cb, evtbuf=self.evtbuf)

        row = self.la.mrow + 5
        self.lblstart = Label(wri, row, col, lw, bdcolor=GREEN) 
        self.lblend = Label(wri, row, ssd.width - lw - 3, lw, bdcolor=GREEN)
        self.ba = Button(wri, self.lblstart.mrow + 5, col, height=15,
                         bgcolor=DARKGREEN, litcolor=WHITE, shape=CLIPPED_RECT,
                         text='Acquire', callback=self.btncb)
        self.bh = Button(wri, self.ba.mrow + 5, col, height=15,
                         bgcolor=DARKGREEN, litcolor=WHITE, shape=CLIPPED_RECT,
                         text='Home', callback=self.homecb)
        col = self.ba.mcol + 10
        self.zp = Button(wri, row, col, shape=CIRCLE, bgcolor=LIGHTRED,
                         litcolor=WHITE, text='+', callback=self.zplus)
        self.zm = Button(wri, row + 25, col, shape=CIRCLE, bgcolor=LIGHTRED,
                         litcolor=WHITE, text='-', callback=self.zminus)
        col = self.zp.mcol + 20
        els = ("Next", "Ch 0", "Timer", "Hog end")
        self.dd = Dropdown(wri, row, col, elements = els,
                           bdcolor = BLUE, fgcolor=BLUE, fontcolor=GREEN)
        self.lbltim = Label(wri, self.dd.mrow + 3, col, "Duration")
        self.fa = FloatAdj(wri, self.lbltim.mrow + 3, col, color=BLUE, lbl_width=lw,
                           map_func=lambda x: 1 + round(x * 1000),
                           value=0.5, fstr="{:3d}ms")
        col = self.dd.mcol + 20
        rowa = row + 15
        self.ca0 = Adjuster(wri, rowa, col, fgcolor=CYAN,
                           value=0.5, callback=self.curs, args=(0,))
        self.lblcurs = Label(wri, row, col, "Cursors")
        self.ca1 = Adjuster(wri, self.ca0.mrow + 5, col, fgcolor=CYAN,
                           value=0.5, callback=self.curs, args=(1,))
        self.lblcursa = Label(wri, rowa, self.ca1.mcol + 5, lw, bdcolor=CYAN)
        Label(wri, rowa, self.lblcursa.mcol + 3, "Absolute")
        self.lblcursv = Label(wri, self.lblcursa.mrow + 5, self.ca1.mcol + 5, lw, bdcolor=CYAN)
        Label(wri, self.lblcursa.mrow + 5, self.lblcursv.mcol + 3, "Diff")

    def cb(self, la):
        la.scroll()  # Adjust window to match current value
        self.refresh(la)

    def curs(self, adj, n):  # Args: adjuster, cursor no
        if not self.initialised:
            return
        res = self.la.cursor(n, adj())
        if isinstance(res, float):
            adj.value(res)
            return  # Will re-enter
        v0, v1 = res
        if v0 >= 0:  # Cursor 0 has been set
            self.lblcursa.value("{:6.3f}ms".format(v0 / 1000))
            if v1 >= 0:  # Both set
                self.lblcursv.value("{:6.3f}ms".format((v1 - v0) / 1000))
        self.la.redraw()


    def refresh(self, la):
        la.redraw()
        self.lblstart.value(f'{la.t_ws / 1000:5.1f}ms')
        self.lblend.value(f'{la.t_we / 1000:5.1f}ms')
        if la.tcurs[0] == -1:
            self.lblcursv.value("")
            self.lblcursa.value("")
        self.zm.greyed_out(not self.la.can_zoomout)

    def btncb(self, _):
        asyncio.create_task(self.do_get())

    async def do_get(self):
        self.evtbuf.clear()
        self.refresh(self.la)
        await asyncio.sleep_ms(200)  # Physical refresh of blank display
        self.evtbuf.acquire(self.dd.value(), self.fa.mapped_value())  # Args: mode and timer value
        self.refresh(self.la)

    def homecb(self, _):
        self.la.home()
        self.refresh(self.la)

    def zplus(self, _):
        self.la.zoom_plus()
        self.refresh(self.la)
        #gc.collect()  # TEST
        #print(gc.mem_free())

    def zminus(self, _):
        self.la.zoom_minus()
        self.refresh(self.la)

    def after_open(self):
        self.initialised = True

# ***** End *****
def run(device="demo"):  # Options "uart" or "spi"
    print("Analyser mode:", device)
    Screen.change(BaseScreen, args=(device,))

# run()
