# MIT License
#
# Copyright (c) 2018, Kevin Köck
# Copyright (c) 2019, Mike Teachman
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

'''
Created on 16.06.2018

@author: Kevin Köck
'''

# based on circuitpython-code of adafruit: https://learn.adafruit.com/pm25-air-quality-sensor/circuitpython-code
# and some inspiration from https://github.com/RigacciOrg/AirPi/blob/master/lib/pms5003

# command responses are only processed generally but can't distinguish between different responses

__updated__ = "2019-03-21"
__version__ = "1.9.10"

import uasyncio as asyncio
import time

try:
    import struct
except ImportError:
    import ustruct as struct

# Normal data frame length.
DATA_FRAME_LENGTH = 28
# Command response frame length.
CMD_FRAME_LENGTH = 4

# Maximum tries after which the device is being reset, the actual tries are twice as much as library tries 2 times
MAX_COMMAND_FAILS = 3

DEBUG = False


def set_debug(debug):
    global DEBUG
    DEBUG = debug


class PMS5003:
    def __init__(self, uart, lock, event=None):
        self._uart = uart  # accepts a uart object
        self._sreader = asyncio.StreamReader(uart)
        self._lock = lock
        self._event = event
        self._timestamp = None
        self._invalidateMeasurements()

    @staticmethod
    def _error(message):
        # Default logging implementation, to be overriden in subclasses if logging required
        print(message)

    @staticmethod
    def _warn(message):
        # Default logging implementation, to be overriden in subclasses if logging required
        print(message)

    @staticmethod
    def _debug(message):
        # Default logging implementation, to be overriden in subclasses if logging required
        if DEBUG:
            print(message)

    async def setPassiveMode(self, interval=None):
        self._debug("setPassiveMode")
        async with self._lock:
            self._debug("setPassiveMode got lock")
            res = await self._sendCommand(0xe1, 0x00)
            if res is None:
                await asyncio.sleep_ms(100)
                res = await self._sendCommand(0xe1, 0x00)
                if res is None:
                    self._error("Error putting device in passive mode")
                    self._lock.release()  # workaround until bug fixed
                    return False
            self._uart.clear_rx()
        self._debug("setPassiveMode done")
        return True

    async def _sendCommand(self, command, data, expect_command=True, delay=1000, wait=None):
        self._debug("Sending command: {!s},{!s},{!s},{!s}".format(command, data, expect_command, delay))
        arr = bytearray(7)
        arr[0] = 0x42
        arr[1] = 0x4d
        arr[2] = command
        arr[3] = 0x00
        arr[4] = data
        s = sum(arr[:5])
        arr[5] = int(s / 256)
        arr[6] = s % 256
        self._uart.clear_rx()
        self._uart.write(arr)
        et = time.ticks_ms() + delay + (wait if wait else 0)
        frame_len = CMD_FRAME_LENGTH + 4 if expect_command else DATA_FRAME_LENGTH + 4
        # self._debug("Expecting {!s}".format(frame_len))
        if wait:
            self._debug("waiting {!s}s".format(wait / 1000))
            await asyncio.sleep_ms(wait)
            self._uart.clear_rx()
        while time.ticks_ms() < et:
            await asyncio.sleep_ms(100)
            if self._uart.any() >= frame_len:
                # going through all pending data frames
                res = await self._read_frame()
                if res is True and expect_command:
                    self._debug("Got True")
                    return True
                elif res is not None:
                    self._debug("Got {!s}".format(res))
                    return res
                else:
                    pass  # try again until found a valid one or timeout
                await asyncio.sleep_ms(100)
        self._debug("Got no available bytes")
        return None

    def print(self):
        if self._active and self._timestamp is not None:
            print("")
            print("---------------------------------------------")
            t = time.localtime()
            print("Measurement {!s}ms ago at {}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
                time.ticks_ms() - self._timestamp, t[0], t[1], t[2], t[3], t[4], t[5]))
            print("---------------------------------------------")
            print("Concentration Units (standard)")
            print("---------------------------------------------")
            print("PM 1.0: %d\tPM2.5: %d\tPM10: %d" % (self._pm10_standard, self._pm25_standard, self._pm100_standard))
            print("Concentration Units (environmental)")
            print("---------------------------------------------")
            print("PM 1.0: %d\tPM2.5: %d\tPM10: %d" % (self._pm10_env, self._pm25_env, self._pm100_env))
            print("---------------------------------------------")
            print("Particles > 0.3um / 0.1L air:", self._particles_03um)
            print("Particles > 0.5um / 0.1L air:", self._particles_05um)
            print("Particles > 1.0um / 0.1L air:", self._particles_10um)
            print("Particles > 2.5um / 0.1L air:", self._particles_25um)
            print("Particles > 5.0um / 0.1L air:", self._particles_50um)
            print("Particles > 10 um / 0.1L air:", self._particles_100um)
            print("---------------------------------------------")
            print("")
        else:
            print("PMS5003 Sensor not active")

    async def read(self):
        async with self._lock:
            self._debug("_read got lock")
            frame = None
            counter = 0
            while frame is None and counter < 5:
                frame = await self._sendCommand(0xe2, 0x00, False, delay=10000)
                if frame is not None:
                    self._pm10_standard = frame[0]
                    self._pm25_standard = frame[1]
                    self._pm100_standard = frame[2]
                    self._pm10_env = frame[3]
                    self._pm25_env = frame[4]
                    self._pm100_env = frame[5]
                    self._particles_03um = frame[6]
                    self._particles_05um = frame[7]
                    self._particles_10um = frame[8]
                    self._particles_25um = frame[9]
                    self._particles_50um = frame[10]
                    self._particles_100um = frame[11]
                    self._timestamp = time.ticks_ms()
                    if self._event is not None:
                        self._event.set()
                    last_reading = time.ticks_ms()
                counter += 1
                await asyncio.sleep_ms(100)

    async def _read_frame(self, with_lock=False, with_async=False):
        # using lock to prevent multiple coroutines from reading at the same time
        self._debug("readFrame {!s} {!s}".format(with_lock, with_async))
        if with_lock:
            async with self._lock:
                self._debug("readFrame got lock")
                res = await self.__read_frame(with_async)  # can be None
                self._debug("readFrame got: {!s}".format(res))
                return res
        else:
            res = await self.__read_frame(with_async)  # can be None
            self._debug("readFrame got: {!s}".format(res))
            return res

    async def __await_bytes(self, count, timeout):
        st = time.ticks_ms()
        while self._uart.any() < count:
            await asyncio.sleep_ms(20)
            if time.ticks_ms() - st > timeout:
                return

    async def __read_frame(self, with_async):
        buffer = []
        start = time.ticks_ms()
        timeout = 200
        self._debug("__read_frame")
        available = self._uart.any()
        if available > 32 and available % 32 == 0:
            self._uart.read(available - 32)  # just throw away the oldest data_frames
            self._debug("Throwing away old data_frames, #bytes {!s}".format(available - 32))
        while True:
            if with_async is False and time.ticks_ms() - start > timeout:
                self._debug(
                    "Reading a lot of noise on RX line to exceed timeout of {!s}ms, availble bytes {!s}".format(
                        timeout, self._uart.any()))
                return None
            preframe_len = 4 + CMD_FRAME_LENGTH - len(buffer)
            if with_async:
                # StreamReader seems to have problems reading the correct amount of bytes
                data = b""
                count = 0
                while len(data) < preframe_len:
                    data += await self._sreader.read(preframe_len - len(data))
                    if count > 5:
                        break
                    count += 1
            else:
                data = b""
                await self.__await_bytes(preframe_len, 100)
                data = self._uart.read(preframe_len)
            if data == None:
                return None
            if len(data) != preframe_len and len(data) > 0:
                self._error("Short read, expected {!s} bytes, got {!s}".format(preframe_len, len(data)))
                return None
            if data == b'':
                return None
            buffer += list(data)
            while len(buffer) >= 2 and buffer[0] != 0x42 and buffer[1] != 0x4d:
                buffer.pop(0)
            if len(buffer) == 0:
                continue
            elif len(buffer) < 4:
                continue
            frame_len = struct.unpack(">H", bytes(buffer[2:4]))[0]
            if frame_len == DATA_FRAME_LENGTH:
                if with_async:
                    # StreamReader seems to have problems reading the correct amount of bytes
                    data = b""
                    count = 0
                    while len(data) < frame_len - CMD_FRAME_LENGTH:
                        data += await self._sreader.read(frame_len - CMD_FRAME_LENGTH - len(data))
                        if count > 5:
                            break
                        count += 1
                else:
                    await self.__await_bytes(frame_len - CMD_FRAME_LENGTH, 100)
                    data = self._uart.read(frame_len - CMD_FRAME_LENGTH)
                if len(data) != DATA_FRAME_LENGTH - CMD_FRAME_LENGTH:
                    self._error("Short read, expected {!s} bytes, got {!s}".format(frame_len, len(data)))
                    return None
                buffer += list(data)
                check = buffer[-2] * 256 + buffer[-1]
                checksum = sum(buffer[0:frame_len + 2])
                if check == checksum:
                    if self._uart.any() > 32:
                        self._uart.clear_rx()  # just to prevent getting flooded if a callback took too long
                        self._warn("Getting too many new data frames, callback too slow")
                    frame = struct.unpack(">HHHHHHHHHHHHHH", bytes(buffer[4:]))
                    return frame
            elif frame_len == CMD_FRAME_LENGTH:
                check = buffer[-2] * 256 + buffer[-1]
                checksum = sum(buffer[0:frame_len + 2])
                if check == checksum:
                    self._debug("Received command response frame: {!s}".format(buffer))
                    return True
                else:
                    return None
            elif frame_len == 0:
                pass  # wrong frame/bytes received
            else:
                self._warn("Unexpected frame_len {!s}, probably random or scrambled bytes".format(frame_len))

            buffer = []
            continue

            # pm10_standard, pm25_standard, pm100_standard, pm10_env,
            # pm25_env, pm100_env, particles_03um, particles_05um, particles_10um,
            # particles_25um, particles_50um, particles100um, skip, checksum=frame

    def _invalidateMeasurements(self):
        self._pm10_standard = None
        self._pm25_standard = None
        self._pm100_standard = None
        self._pm10_env = None
        self._pm25_env = None
        self._pm100_env = None
        self._particles_03um = None
        self._particles_05um = None
        self._particles_10um = None
        self._particles_25um = None
        self._particles_50um = None
        self._particles_100um = None

    @property
    def pm10_standard(self):
        return self._pm10_standard

    @property
    def pm25_standard(self):
        return self._pm25_standard

    @property
    def pm100_standard(self):
        return self._pm100_standard

    @property
    def pm10_env(self):
        return self._pm10_env

    @property
    def pm25_env(self):
        return self._pm25_env

    @property
    def pm100_env(self):
        return self._pm100_env

    @property
    def particles_03um(self):
        return self._particles_03um

    @property
    def particles_05um(self):
        return self._particles_05um

    @property
    def particles_10um(self):
        return self._particles_10um

    @property
    def particles_25um(self):
        return self._particles_25um

    @property
    def particles_50um(self):
        return self._particles_50um

    @property
    def particles_100um(self):
        return self._particles_100um

    @property
    def timestamp(self):
        return self._timestamp