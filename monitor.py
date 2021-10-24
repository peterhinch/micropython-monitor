# monitor.py
# Monitor an asynchronous program by sending single bytes down an interface.

# Copyright (c) 2021 Peter Hinch
# Released under the MIT License (MIT) - see LICENSE file

import uasyncio as asyncio
from machine import UART, SPI, Pin
from time import sleep_us
from sys import exit

buf = bytearray(1)  # Pre-allocate UART/SPI buffer
# Quit with an error message rather than throw.
def _quit(s):
    print("Monitor " + s)
    exit(0)


_write = lambda _: _quit("must run set_device")
_ifrst = lambda: None  # Reset interface. If UART do nothing.

# For UART pass initialised UART. Baudrate must be 1_000_000.
# For SPI pass initialised instance SPI. Can be any baudrate, but
# must be default in other respects.
def set_device(dev, cspin=None):
    global _write
    global _ifrst
    if isinstance(dev, UART) and cspin is None:  # UART

        def uwrite(data):
            buf[0] = data
            dev.write(buf)

        _write = uwrite
    elif isinstance(dev, SPI) and isinstance(cspin, Pin):
        cspin(1)

        def spiwrite(data):
            cspin(0)
            buf[0] = data
            dev.write(buf)
            cspin(1)

        _write = spiwrite

        def clear_sm():  # Set Pico SM to its initial state
            cspin(1)
            dev.write(b"\0")  # SM is now waiting for CS low.

        _ifrst = clear_sm
    else:
        _quit("set_device: invalid args.")


# /mnt/qnap2/data/Projects/Python/AssortedTechniques/decorators
_available = set(range(0, 22))  # Valid idents are 0..21
# Looping: some idents may be repeatedly instantiated. This can occur
# if decorator is run in looping code. A CM is likely to be used in a
# loop. In these cases only validate on first use.
_loopers = set()


def _validate(ident, num=1, looping=False):
    if ident >= 0 and ident + num < 22:
        try:
            for x in range(ident, ident + num):
                if looping:
                    if x not in _loopers:
                        _available.remove(x)
                        _loopers.add(x)
                else:
                    _available.remove(x)
        except KeyError:
            _quit("error - ident {:02d} already allocated.".format(x))
    else:
        _quit("error - ident {:02d} out of range.".format(ident))


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
def init():
    _ifrst()  # Reset interface. Does nothing if UART.
    _write(ord("z"))  # Clear Pico's instance counters etc.


# Optionally run this to show up periods of blocking behaviour
async def hog_detect(s=(0x40, 0x60)):
    while True:
        for v in s:
            _write(v)
            await asyncio.sleep_ms(0)


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


# Monitor a function call
class mon_call:
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
