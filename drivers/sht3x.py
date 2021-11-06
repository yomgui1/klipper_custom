# SHT3x i2c based temperature sensors support
#
# Copyright (C) 2021 Guillaume Roguez <yomgui1@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
import logging
from . import bus

# Tested on SHT31

SHT3X_I2C_ADDR= 0x44

class SHT3X:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=SHT3X_I2C_ADDR, default_speed=100000)
        self.report_time = config.getint('sht3x_report_time',5,minval=2)
        self.temperature = self.min_temp = self.max_temp = self.humidity = 0.
        self.sample_timer = self.reactor.register_timer(self._sample_sht3x)
        self.printer.add_object("sht3x " + self.name, self)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

    def handle_connect(self):
        self._init_sht3x()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def _init_sht3x(self):
        # Device Soft Reset
        self.i2c.i2c_write([0x30, 0xA2])
        # Wait after reset
        self.reactor.pause(self.reactor.monotonic() + .5)

    def _sample_sht3x(self, eventtime):
        try:
            # Read Oneshot
            params = self.i2c.i2c_write([0x2C, 0x06])

            # Wait
            self.reactor.pause(self.reactor.monotonic() + .01)

            params = self.i2c.i2c_read([], 6)
            response = bytearray(params['response'])

            # Temperature
            rTemp = (response[0] << 8) + response[1]
            logging.debug("sht3x: [%d, %d]" % (response[0], response[1]))
            if not self._checkCRC8(rTemp, response[2]):
                logging.warn("sht3x: Checksum error on Temperature reading!")
            else:
                self.temperature = -45 + (175 * rTemp / 65535.0)
                logging.debug("sht3x: Temperature %.2f " % self.temperature)

            # Humidity
            rHumid = (response[3] << 8) + response[4]
            if not self._checkCRC8(rHumid, response[5]):
                logging.warn("sht3x: Checksum error on Humidity reading!")
            else:
                self.humidity = 100 * rHumid / 65535.0
                if (self.humidity < 0):
                    self.humidity = 0
                elif (self.humidity > 100):
                    self.humidity = 100
                logging.debug("sht3x: Humidity %.2f " % self.humidity)
        except Exception:
            logging.exception("sht3x: Error reading data")
            self.temperature = self.humidity = .0
            return self.reactor.NEVER

        if self.temperature < self.min_temp or self.temperature > self.max_temp:
            self.printer.invoke_shutdown(
                "SHT3X temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temperature, self.min_temp, self.max_temp))

        measured_time = self.reactor.monotonic()
        print_time = self.i2c.get_mcu().estimated_print_time(measured_time)
        self._callback(print_time, self.temperature)
        return measured_time + self.report_time

    def _checkCRC8(self, data, crc):
        # TODO
        return True

    def get_status(self, eventtime):
        return {
            'temperature': round(self.temperature, 2),
            'humidity': self.humidity,
        }


def load_config(config):
    # Register sensor
    pheater = config.get_printer().lookup_object("heaters")
    pheater.add_sensor_factory("SHT3X", SHT3X)
