# lock_event.py
# Tests the monitoring of Lock and Event objects.

# Copyright (c) 2025 Peter Hinch
# Released under the MIT License (MIT) - see LICENSE file

# Host assumed to be a Pico

import asyncio
from machine import UART
import monitor
from pins import pico  # PCB pin nos.

# Define interface to use
monitor.set_device(UART(0, 1_000_000))  # GPIO 0, pin 1. UART must be 1MHz

lock = monitor.MonLock(pico(6))  # GPIO4 pin 6
event = monitor.MonEvent(pico(7))  # GPIO 5 pin 7


async def locker(n):
    print(f"locker {n} awaiting event.")
    await event.wait()
    await asyncio.sleep(1)
    print(f"locker {n} awaiting lock.")
    async with lock:
        print(f"locker {n} got lock.")
        await asyncio.sleep(1)
    print(f"locker {n} released lock.")


async def start():
    await asyncio.sleep(1)
    event.set()


async def main():
    monitor.init()
    coros = [locker(x) for x in range(4)]
    asyncio.create_task(start())
    await asyncio.gather(*coros)
    print("Pause 1s")
    await asyncio.sleep(1)
    event.clear()
    print("Event cleared.")
    await asyncio.sleep(1)
    print("Done")


try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()

# Sequence:
# Lock 0, Event 0 All tasks waiting on event
# Lock 0, Event 1 Tasks got event, paused
# Lock 1, Event 1 Persists for 4 secs. Tasks wait on lock, each runs for 1s
# Lock 0, Event 1 Last task releases lock
# Lock 0, Event 0 Main code clears event.
