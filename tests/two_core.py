# two_core.py Test monitoring dual-core code on RP2.
# Firmware should have patch
# https://github.com/micropython/micropython/issues/7977
# Should see hog detect on 0, approximate square waves on 1-3

import _thread
from time import sleep_ms
import asyncio
from machine import UART, SPI, Pin
import gc
import monitor

# monitor.set_device(UART(0, 1_000_000))  # UART must be 1MHz. O/P on GPIO 0
monitor.set_device(SPI(0, baudrate=5_000_000), Pin(5, Pin.OUT))
trig1 = monitor.trigger(1)
trig2 = monitor.trigger(2)


def other():
    while True:
        trig1(True)
        sleep_ms(10)
        trig1(False)
        sleep_ms(9)


@monitor.asyn(3)
async def bar():
    await asyncio.sleep_ms(10)


async def foo():
    while True:
        trig2(True)
        await asyncio.sleep_ms(10)
        trig2(False)
        await asyncio.sleep_ms(10)


async def main():
    monitor.init(True)  # Run hog_detect task
    print("Running...")
    asyncio.create_task(foo())
    _thread.start_new_thread(other, ())  # Tuple of args
    while True:
        await bar()
        await asyncio.sleep_ms(11)


asyncio.run(main())
