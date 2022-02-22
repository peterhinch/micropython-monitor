# two_core.py Test monitoring dual-core code on RP2.
# Firmware should have patch
# https://github.com/micropython/micropython/issues/7977
import _thread
from time import sleep_ms
import uasyncio as asyncio
from machine import UART
import gc
import monitor

monitor.set_device(UART(0, 1_000_000))  # UART must be 1MHz. O/P on GPIO 0
trig1 = monitor.trigger(1)
trig2 = monitor.trigger(2)
g = 0
def other():
    global g
    while True:
        trig1(True)
        sleep_ms(10)
        trig1(False)
        sleep_ms(9)
        g += 1

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
    monitor.init()
    asyncio.create_task(monitor.hog_detect())
    asyncio.create_task(foo())
    _thread.start_new_thread(other, ())  # Tuple of args
    while True:
        await bar()
        await asyncio.sleep_ms(11)
        #print(g)
        #gc.collect()  # ???

asyncio.run(main())
