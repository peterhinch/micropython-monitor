# isr.py
# Tests case where a trigger is operated in a hard ISR.
# This test is Pyboard specific.

# Copyright (c) 2021-2022 Peter Hinch
# Released under the MIT License (MIT) - see LICENSE file


import asyncio
from machine import Pin, UART, SPI
from pyb import Timer
import monitor


monitor.set_device(UART(2, 1_000_000))  # UART must be 1MHz. O/P on X3
trig1 = monitor.trigger(1)
trig2 = monitor.trigger(2)

tim = Timer(1)
tsf = asyncio.ThreadSafeFlag()


def tcb(_):
    trig1()
    tsf.set()


async def main():
    monitor.init()
    tim.init(freq=50, callback=tcb)
    while True:
        await tsf.wait()
        trig2()  # Latency 276us on Pyboad D SF2W


try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()
