"""
CMPS12 I2C Controller (smbus2)
================================
A simple Python driver for the SparkFun/Robot Electronics CMPS12 tilt-compensated
compass module, operating in I2C mode.

Wiring (I2C mode):
    - Mode pin -> left open, or pulled to supply voltage (selects I2C mode at
      power-up; do NOT ground it, that selects serial mode instead)
    - SDA -> SDA (with pull-up resistor, usually already present on Pi/dev boards)
    - SCL -> SCL (with pull-up resistor)
    - GND -> GND
    - VCC -> 3.3-5V

Note on addressing: the datasheet's register table uses the 8-bit write
address (default 0xC0). smbus2, like most Linux I2C stacks, wants the 7-bit
address instead, so 0xC0 becomes 0x60 (0xC0 >> 1). If you've changed the
module's address, shift it the same way.

Requires: smbus2  (pip install smbus2 --break-system-packages)

For fastest readout, use read_all_fast() (a single bulk transaction covering
every register) rather than calling individual get_x() methods in a loop -
see the class docstring below for details. Bus clock speed is a separate,
larger lever that has to be set outside Python; see the note above
read_all_fast().
"""

import time
import struct
from smbus2 import SMBus, i2c_msg


class CMPS12:
    # Register map
    REG_COMMAND = 0x00           # write: command register / read: software version
    REG_BEARING_8BIT = 0x01
    REG_BEARING_16BIT_HIGH = 0x02   # 0x02, 0x03 -> 0-3599 (0.1 degree units)
    REG_PITCH_90 = 0x04             # signed byte, +/- 90
    REG_ROLL_90 = 0x05              # signed byte, +/- 90
    REG_MAG_X_HIGH = 0x06           # 0x06-0x0B, signed 16-bit XYZ
    REG_ACCEL_X_HIGH = 0x0C         # 0x0C-0x11, signed 16-bit XYZ
    REG_GYRO_X_HIGH = 0x12          # 0x12-0x17, signed 16-bit XYZ
    REG_TEMP_HIGH = 0x18            # 0x18, 0x19, signed 16-bit, degrees C
    REG_BOSCH_BEARING_HIGH = 0x1A   # 0x1A, 0x1B -> 0-5759 (divide by 16 for degrees)
    REG_PITCH_180_HIGH = 0x1C       # 0x1C, 0x1D, signed 16-bit, +/- 180
    REG_CAL_STATE = 0x1E

    # Command register (0x00) sequences
    SEQ_STORE_CAL = (0xF0, 0xF5, 0xF6)
    SEQ_DELETE_CAL = (0xE0, 0xE5, 0xE2)
    SEQ_CHANGE_ADDR = (0xA0, 0xAA, 0xA5)  # followed by the new 8-bit address

    DEFAULT_ADDR_8BIT = 0xC0

    def __init__(self, bus_number=1, address_8bit=DEFAULT_ADDR_8BIT):
        """
        bus_number: I2C bus number (1 on most Raspberry Pi boards)
        address_8bit: the module's address as printed in the datasheet
                      (0xC0 default). Automatically converted to 7-bit.
        """
        self.address = address_8bit >> 1
        self.bus = SMBus(bus_number)

    def close(self):
        self.bus.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # ------------------------------------------------------------------
    # Low-level helpers
    # ------------------------------------------------------------------
    def _read_bytes(self, register, n):
        """Read n bytes starting at register using a repeated-start transaction."""
        write = i2c_msg.write(self.address, [register])
        read = i2c_msg.read(self.address, n)
        self.bus.i2c_rdwr(write, read)
        return bytes(read)

    def _write_byte(self, register, value):
        self.bus.write_byte_data(self.address, register, value)

    @staticmethod
    def _to_signed16(high, low):
        return struct.unpack('>h', bytes([high, low]))[0]

    @staticmethod
    def _to_signed8(byte_val):
        return struct.unpack('b', bytes([byte_val]))[0]

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def get_version(self):
        """Software version (reading register 0x00 returns this)."""
        data = self._read_bytes(self.REG_COMMAND, 1)
        return data[0]

    def get_bearing_8bit(self):
        """Bearing scaled to 0-255 for a full circle."""
        data = self._read_bytes(self.REG_BEARING_8BIT, 1)
        return data[0]

    def get_bearing_16bit(self):
        """Bearing in degrees, 0.0-359.9 (processor-calculated)."""
        data = self._read_bytes(self.REG_BEARING_16BIT_HIGH, 2)
        raw = (data[0] << 8) | data[1]
        return raw / 10.0

    def get_pitch(self):
        """Pitch angle, +/- 90 degrees, signed byte."""
        data = self._read_bytes(self.REG_PITCH_90, 1)
        return self._to_signed8(data[0])

    def get_roll(self):
        """Roll angle, +/- 90 degrees, signed byte."""
        data = self._read_bytes(self.REG_ROLL_90, 1)
        return self._to_signed8(data[0])

    def get_pitch_180(self):
        """Pitch angle, +/- 180 degrees, 16-bit signed, high byte first."""
        data = self._read_bytes(self.REG_PITCH_180_HIGH, 2)
        return self._to_signed16(data[0], data[1])

    def get_mag_raw(self):
        """Raw magnetometer XYZ, 16-bit signed integers."""
        data = self._read_bytes(self.REG_MAG_X_HIGH, 6)
        return (
            self._to_signed16(data[0], data[1]),
            self._to_signed16(data[2], data[3]),
            self._to_signed16(data[4], data[5]),
        )

    def get_accel_raw(self):
        """Raw accelerometer XYZ, 16-bit signed integers."""
        data = self._read_bytes(self.REG_ACCEL_X_HIGH, 6)
        return (
            self._to_signed16(data[0], data[1]),
            self._to_signed16(data[2], data[3]),
            self._to_signed16(data[4], data[5]),
        )

    def get_gyro_raw(self):
        """Raw gyro XYZ, 16-bit signed integers."""
        data = self._read_bytes(self.REG_GYRO_X_HIGH, 6)
        return (
            self._to_signed16(data[0], data[1]),
            self._to_signed16(data[2], data[3]),
            self._to_signed16(data[4], data[5]),
        )

    def get_temperature(self):
        """Temperature in degrees C."""
        data = self._read_bytes(self.REG_TEMP_HIGH, 2)
        return self._to_signed16(data[0], data[1])

    def get_bosch_bearing_16bit(self):
        """Bosch-calculated bearing in degrees (0-359.9)."""
        data = self._read_bytes(self.REG_BOSCH_BEARING_HIGH, 2)
        raw = (data[0] << 8) | data[1]
        return raw / 16.0

    def get_calibration_state(self):
        """
        Returns dict with calibration levels 0-3 for system, gyro,
        accelerometer, and magnetometer (3 = fully calibrated).
        """
        data = self._read_bytes(self.REG_CAL_STATE, 1)
        val = data[0]
        return {
            "system": (val >> 6) & 0x03,
            "gyro": (val >> 4) & 0x03,
            "accelerometer": (val >> 2) & 0x03,
            "magnetometer": val & 0x03,
        }

    def get_all(self):
        """Convenience read: bearing (0-359.9), pitch (+/-90), roll (+/-90).
        Uses 3 separate transactions - see read_all_fast() for a single-
        transaction version that pulls every register in one go."""
        return {
            "bearing": self.get_bearing_16bit(),
            "pitch": self.get_pitch(),
            "roll": self.get_roll(),
        }

    # ------------------------------------------------------------------
    # Fast bulk read
    # ------------------------------------------------------------------
    # Registers 0x01-0x1E are contiguous, so the whole sensor block can be
    # pulled in ONE I2C transaction instead of one transaction per value.
    # This is the single biggest speed win available: each transaction has
    # fixed overhead (start condition, address+ack, stop condition) that
    # usually costs more than the extra bytes do, so collapsing ~13
    # transactions into 1 is far faster than reading fewer bytes per call.
    _BULK_START_REG = 0x01
    _BULK_LENGTH = 0x1E - 0x01 + 1  # 30 bytes, registers 0x01 through 0x1E

    def read_all_fast(self):
        """
        Read every register (0x01-0x1E) in a single I2C transaction and
        parse all values from the resulting buffer. This is the fastest
        way to poll the sensor - use this instead of calling multiple
        individual get_x() methods in a loop.

        Returns a dict with every value the module provides.
        """
        data = self._read_bytes(self._BULK_START_REG, self._BULK_LENGTH)
        # data[i] corresponds to register (_BULK_START_REG + i), i.e.
        # data[0] = register 0x01, data[1] = register 0x02, etc.

        bearing_8bit = data[0x01 - 0x01]
        bearing_16bit_raw = (data[0x02 - 0x01] << 8) | data[0x03 - 0x01]
        pitch_90 = self._to_signed8(data[0x04 - 0x01])
        roll_90 = self._to_signed8(data[0x05 - 0x01])

        mag = (
            self._to_signed16(data[0x06 - 0x01], data[0x07 - 0x01]),
            self._to_signed16(data[0x08 - 0x01], data[0x09 - 0x01]),
            self._to_signed16(data[0x0A - 0x01], data[0x0B - 0x01]),
        )
        accel = (
            self._to_signed16(data[0x0C - 0x01], data[0x0D - 0x01]),
            self._to_signed16(data[0x0E - 0x01], data[0x0F - 0x01]),
            self._to_signed16(data[0x10 - 0x01], data[0x11 - 0x01]),
        )
        gyro = (
            self._to_signed16(data[0x12 - 0x01], data[0x13 - 0x01]),
            self._to_signed16(data[0x14 - 0x01], data[0x15 - 0x01]),
            self._to_signed16(data[0x16 - 0x01], data[0x17 - 0x01]),
        )

        temperature = self._to_signed16(data[0x18 - 0x01], data[0x19 - 0x01])
        bosch_bearing_raw = (data[0x1A - 0x01] << 8) | data[0x1B - 0x01]
        pitch_180 = self._to_signed16(data[0x1C - 0x01], data[0x1D - 0x01])
        cal_byte = data[0x1E - 0x01]

        return {
            "bearing_8bit": bearing_8bit,
            "bearing": bearing_16bit_raw / 10.0,
            "pitch": pitch_90,
            "roll": roll_90,
            "mag_raw": mag,
            "accel_raw": accel,
            "gyro_raw": gyro,
            "temperature": temperature,
            "bosch_bearing": bosch_bearing_raw / 16.0,
            "pitch_180": pitch_180,
            "calibration": {
                "system": (cal_byte >> 6) & 0x03,
                "gyro": (cal_byte >> 4) & 0x03,
                "accelerometer": (cal_byte >> 2) & 0x03,
                "magnetometer": cal_byte & 0x03,
            },
        }

    # ------------------------------------------------------------------
    # Calibration profile storage
    # ------------------------------------------------------------------
    def store_calibration_profile(self, delay=0.02):
        """Persist the current calibration profile to non-volatile memory."""
        for byte in self.SEQ_STORE_CAL:
            self._write_byte(self.REG_COMMAND, byte)
            time.sleep(delay)

    def delete_calibration_profile(self, delay=0.02):
        """Erase the stored calibration profile."""
        for byte in self.SEQ_DELETE_CAL:
            self._write_byte(self.REG_COMMAND, byte)
            time.sleep(delay)

    # ------------------------------------------------------------------
    # I2C address change
    # ------------------------------------------------------------------
    def change_i2c_address(self, new_address_8bit, delay=0.02):
        """
        Change the module's I2C address. Only ONE CMPS12 may be on the bus
        when doing this. new_address_8bit must be one of the values listed
        in the datasheet's address table (0xC0, 0xC2, 0xC4, ... 0xCE).

        After this call, self.address is updated to match, and the module
        must be power-cycled for the new address to take effect.
        """
        for byte in self.SEQ_CHANGE_ADDR:
            self._write_byte(self.REG_COMMAND, byte)
            time.sleep(delay)
        self._write_byte(self.REG_COMMAND, new_address_8bit)
        time.sleep(delay)
        self.address = new_address_8bit >> 1
        print(
            f"Address changed to 0x{new_address_8bit:02X} (7-bit: "
            f"0x{self.address:02X}). Power-cycle the module for it to take effect."
        )


# ------------------------------------------------------------------------
# Example usage
# ------------------------------------------------------------------------
if __name__ == "__main__":
    BUS_NUMBER = 1  # I2C bus number, typically 1 on a Raspberry Pi

    with CMPS12(bus_number=BUS_NUMBER) as compass:
        print("Software version:", compass.get_version())

        # Quick benchmark: how many single-transaction bulk reads/sec we get
        n = 200
        t0 = time.perf_counter()
        for _ in range(n):
            compass.read_all_fast()
        elapsed = time.perf_counter() - t0
        print(f"read_all_fast(): {n / elapsed:.1f} reads/sec "
              f"({elapsed / n * 1000:.2f} ms/read)")

        try:
            while True:
                d = compass.read_all_fast()
                cal = d["calibration"]

                print(
                    f"Bearing: {d['bearing']:6.1f}  "
                    f"Pitch: {d['pitch']:4d}  "
                    f"Roll: {d['roll']:4d}  "
                    f"Temp: {d['temperature']:3d}C  "
                    f"Cal[sys/gyro/acc/mag]: "
                    f"{cal['system']}/{cal['gyro']}/{cal['accelerometer']}/{cal['magnetometer']}"
                )
                # No sleep - read_all_fast() is cheap enough to run flat-out.
                # Add a time.sleep() here if you want to throttle to a fixed rate.
        except KeyboardInterrupt:
            print("\nStopped.")