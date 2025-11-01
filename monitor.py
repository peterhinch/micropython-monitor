# monitor.py
# Monitor an asynchronous program by sending single bytes down an interface.

# Copyright (c) 2021-2024 Peter Hinch
# Released under the MIT License (MIT) - see LICENSE file
# V0.2 Supports monitoring dual-core applications on RP2

import asyncio
from machine import UART, SPI, Pin, disable_irq, enable_irq
from time import sleep_us
from sys import exit

_lck = False
try:
    import _thread

    _lock = _thread.allocate_lock()
    _acquire = _lock.acquire
    _release = _lock.release
except ImportError:  # Only source of hard concurrency is IRQ

    def _acquire():  # Crude blocking lock. Held only for short periods.
        global _lck
        istate = disable_irq()
        while _lck:
            pass
        _lck = True
        enable_irq(istate)

    def _release():
        global _lck
        _lck = False


# Quit with an error message rather than throw.
def _quit(s):
    print("Monitor", s)
    exit(0)


_write = lambda _: _quit("must run set_device")
_ifrst = lambda: None  # Reset interface. If UART do nothing.

# For UART pass initialised UART. Baudrate must be 1_000_000.
# For SPI pass initialised instance SPI. Can be any baudrate, but
# must be default in other respects.
def set_device(dev, cspin=None):
    global _write, _ifrst
    if isinstance(dev, UART) and cspin is None:  # UART

        def uwrite(data, buf=bytearray(1)):
            _acquire()
            buf[0] = data
            dev.write(buf)
            _release()

        _write = uwrite
    elif isinstance(dev, SPI) and isinstance(cspin, Pin):
        cspin(1)

        def spiwrite(data, buf=bytearray(1)):
            _acquire()
            buf[0] = data
            cspin(0)
            dev.write(buf)
            cspin(1)
            _release()

        _write = spiwrite

        def clear_sm():  # Set Pico SM to its initial state
            cspin(1)
            dev.write(b"\0")  # SM is now waiting for CS low.

        _ifrst = clear_sm
    else:
        _quit("set_device: invalid args.")


# Valid idents are 0..21
# Looping: some idents may be repeatedly instantiated. This can occur
# if decorator is run in looping code. A CM is likely to be used in a
# loop. In these cases only validate on first use.


def _validate(ident, num=1, looping=False, loopers=set(), available=set(range(0, 22))):
    if ident >= 0 and ident + num <= 22:
        try:
            for x in range(ident, ident + num):
                if looping:
                    if x not in loopers:
                        available.remove(x)
                        loopers.add(x)
                else:
                    available.remove(x)
        except KeyError:
            _quit("error - ident {:02d} already allocated.".format(x))
    else:
        _quit("error - ident {:02d} out of range.".format(ident))


# /mnt/qnap2/data/Projects/Python/AssortedTechniques/decorators
# asynchronous monitor
def asyn(ident, max_instances=1, verbose=True, looping=False):
    def decorator(coro):
        _validate(ident, max_instances, looping)
        instance = 0

        async def wrapped_coro(*args, **kwargs):
            nonlocal instance
            d = 0x40 + ident + min(instance, max_instances - 1)
            instance += 1
            if verbose and instance > max_instances:  # Warning only.
                print("Monitor ident: {:02d} instances: {}.".format(ident, instance))
            _write(d)
            try:
                res = await coro(*args, **kwargs)
            except asyncio.CancelledError:
                raise  # Other exceptions produce traceback.
            finally:
                _write(d | 0x20)
                instance -= 1
            return res

        return wrapped_coro

    return decorator


# If SPI, clears the state machine in case prior test resulted in the DUT
# crashing. It does this by sending a byte with CS\ False (high).
def init(hog_detect=False):
    # Optionally run hog detection to show up periods of blocking behaviour
    async def hd(s=(0x40, 0x60)):
        while True:
            for v in s:
                _write(v)
                await asyncio.sleep_ms(0)

    _ifrst()  # Reset interface. Does nothing if UART.
    _write(ord("z"))  # Clear Pico's instance counters etc.
    if hog_detect:
        if asyncio.current_task is None:
            print("Warning: attempt to run hog_detect before asyncio started.")
        asyncio.create_task(hd())


# Monitor a synchronous function definition
def sync(ident, looping=False):
    def decorator(func):
        _validate(ident, 1, looping)

        def wrapped_func(*args, **kwargs):
            _write(0x40 + ident)
            res = func(*args, **kwargs)
            _write(0x60 + ident)
            return res

        return wrapped_func

    return decorator


# Monitor using a context manager
class Monitor:
    def __init__(self, ident):
        # looping: a CM may be instantiated many times
        _validate(ident, 1, True)
        self.ident = ident

    def __enter__(self):
        _write(0x40 + self.ident)
        return self

    def __exit__(self, type, value, traceback):
        _write(0x60 + self.ident)
        return False  # Don't silence exceptions


mon_call = Monitor  # Old version

# Either cause pico ident n to produce a brief (~80μs) pulse or turn it
# on or off on demand. No looping: docs suggest instantiating at start.
def trigger(ident):
    _validate(ident)

    def wrapped(state=None):
        if state is None:
            _write(0x40 + ident)
            sleep_us(20)
            _write(0x60 + ident)
        else:
            _write(ident + (0x40 if state else 0x60))

    return wrapped


# Track the state of a Lock instance
class MonLock(asyncio.Lock):
    def __init__(self, ident):
        _validate(ident)  # Cannot instantiate in a loop
        self.ident = ident
        super().__init__()

    async def acquire(self):
        if not self.state:  # Currently unlocked
            _write(0x40 + self.ident)  # Lock will be set
        await super().acquire()

    def release(self):
        super().release()  # But a pending task may be scheduled
        if not self.state:  # No pending task: the lock is free.
            _write(0x60 + self.ident)


# Track the state of an Event instance
class MonEvent(asyncio.Event):
    def __init__(self, ident):
        _validate(ident)
        self.ident = ident
        super().__init__()

    def set(self):
        if not self.state:  # Currently unset
            _write(0x40 + self.ident)  # Event will be set
        super().set()

    def clear(self):
        if self.state:  # Currently set
            _write(0x60 + self.ident)
        super().clear()
