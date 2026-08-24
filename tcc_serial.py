"""
CMPS12 Serial Controller
=========================
A simple Python driver for the SparkFun/Robot Electronics CMPS12 tilt-compensated
compass module, operating in Serial (UART) mode.

Wiring (Serial mode):
    - Mode pin -> 0V (ground)  -- selects serial mode at power-up
    - TX (module) -> RX (your USB-serial adapter)
    - RX (module) -> TX (your USB-serial adapter)
    - GND -> GND
    - VCC -> 3.3-5V

Default UART settings: 9600 baud, 8 data bits, no parity, 1 stop bit.

Requires: pyserial  (pip install pyserial --break-system-packages)
"""

import serial
import time
import struct


class CMPS12:
    # Command bytes
    CMD_GET_VERSION = 0x11
    CMD_GET_BEARING_8BIT = 0x12
    CMD_GET_BEARING_16BIT = 0x13
    CMD_GET_PITCH = 0x14
    CMD_GET_ROLL = 0x15
    CMD_GET_MAG_RAW = 0x19
    CMD_GET_ACCEL_RAW = 0x20
    CMD_GET_GYRO_RAW = 0x21
    CMD_GET_TEMP = 0x22
    CMD_GET_ALL = 0x23
    CMD_GET_CALIBRATION_STATE = 0x24
    CMD_GET_BOSCH_BEARING_16BIT = 0x25
    CMD_GET_PITCH_180 = 0x26

    CMD_STORE_CAL_1 = 0xF0
    CMD_STORE_CAL_2 = 0xF5
    CMD_STORE_CAL_3 = 0xF6

    CMD_DELETE_CAL_1 = 0xE0
    CMD_DELETE_CAL_2 = 0xE5
    CMD_DELETE_CAL_3 = 0xE2

    CMD_BAUD_19200 = 0xA0
    CMD_BAUD_38400 = 0xA1

    OK_BYTE = 0x55

    def __init__(self, port, baudrate=9600, timeout=1.0):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout,
                                  bytesize=serial.EIGHTBITS,
                                  parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE)
        # Let the module settle after opening the port
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # ------------------------------------------------------------------
    # Low-level helpers
    # ------------------------------------------------------------------
    def _send_command(self, cmd_byte, n_bytes_expected):
        """Send a single command byte and read back n_bytes_expected bytes."""
        self.ser.reset_input_buffer()
        self.ser.write(bytes([cmd_byte]))
        data = self.ser.read(n_bytes_expected)
        if len(data) != n_bytes_expected:
            raise IOError(
                f"CMPS12: expected {n_bytes_expected} bytes for command "
                f"0x{cmd_byte:02X}, got {len(data)}"
            )
        return data

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
        """Return the software version byte."""
        data = self._send_command(self.CMD_GET_VERSION, 1)
        return data[0]

    def get_bearing_8bit(self):
        """Bearing scaled to 0-255 for a full circle."""
        data = self._send_command(self.CMD_GET_BEARING_8BIT, 1)
        return data[0]

    def get_bearing_16bit(self):
        """Bearing in degrees, 0.0-359.9."""
        data = self._send_command(self.CMD_GET_BEARING_16BIT, 2)
        raw = (data[0] << 8) | data[1]
        return raw / 10.0

    def get_pitch(self):
        """Pitch angle, +/- 90 degrees, signed byte."""
        data = self._send_command(self.CMD_GET_PITCH, 1)
        return self._to_signed8(data[0])

    def get_roll(self):
        """Roll angle, +/- 90 degrees, signed byte."""
        data = self._send_command(self.CMD_GET_ROLL, 1)
        return self._to_signed8(data[0])

    def get_pitch_180(self):
        """Pitch angle, +/- 180 degrees, 16-bit signed, high byte first."""
        data = self._send_command(self.CMD_GET_PITCH_180, 2)
        return self._to_signed16(data[0], data[1])

    def get_mag_raw(self):
        """Raw magnetometer XYZ, 16-bit signed integers."""
        data = self._send_command(self.CMD_GET_MAG_RAW, 6)
        x = self._to_signed16(data[0], data[1])
        y = self._to_signed16(data[2], data[3])
        z = self._to_signed16(data[4], data[5])
        return x, y, z

    def get_accel_raw(self):
        """Raw accelerometer XYZ, 16-bit signed integers."""
        data = self._send_command(self.CMD_GET_ACCEL_RAW, 6)
        x = self._to_signed16(data[0], data[1])
        y = self._to_signed16(data[2], data[3])
        z = self._to_signed16(data[4], data[5])
        return x, y, z

    def get_gyro_raw(self):
        """Raw gyro XYZ, 16-bit signed integers."""
        data = self._send_command(self.CMD_GET_GYRO_RAW, 6)
        x = self._to_signed16(data[0], data[1])
        y = self._to_signed16(data[2], data[3])
        z = self._to_signed16(data[4], data[5])
        return x, y, z

    def get_temperature(self):
        """Temperature in degrees C."""
        data = self._send_command(self.CMD_GET_TEMP, 2)
        return self._to_signed16(data[0], data[1])

    def get_all(self):
        """Returns dict with bearing (0-359.9), pitch (+/-90), roll (+/-90)."""
        data = self._send_command(self.CMD_GET_ALL, 4)
        bearing_raw = (data[0] << 8) | data[1]
        pitch = self._to_signed8(data[2])
        roll = self._to_signed8(data[3])
        return {
            "bearing": bearing_raw / 10.0,
            "pitch": pitch,
            "roll": roll,
        }

    def get_calibration_state(self):
        """
        Returns dict with calibration levels 0-3 for system, gyro,
        accelerometer, and magnetometer (3 = fully calibrated).
        """
        data = self._send_command(self.CMD_GET_CALIBRATION_STATE, 1)
        val = data[0]
        return {
            "system": (val >> 6) & 0x03,
            "gyro": (val >> 4) & 0x03,
            "accelerometer": (val >> 2) & 0x03,
            "magnetometer": val & 0x03,
        }

    def get_bosch_bearing_16bit(self):
        """Bosch-calculated bearing in degrees (0-359.9)."""
        data = self._send_command(self.CMD_GET_BOSCH_BEARING_16BIT, 2)
        raw = (data[0] << 8) | data[1]
        return raw / 16.0

    # ------------------------------------------------------------------
    # Calibration profile storage
    # ------------------------------------------------------------------
    def _send_ok_sequence(self, cmd_bytes, delay=0.02):
        """Send a sequence of command bytes, expecting an OK (0x55) after each."""
        for cmd in cmd_bytes:
            data = self._send_command(cmd, 1)
            if data[0] != self.OK_BYTE:
                raise IOError(f"CMPS12: expected OK (0x55) after 0x{cmd:02X}, "
                              f"got 0x{data[0]:02X}")
            time.sleep(delay)

    def store_calibration_profile(self):
        """Persist the current calibration profile to non-volatile memory."""
        self._send_ok_sequence([self.CMD_STORE_CAL_1,
                                 self.CMD_STORE_CAL_2,
                                 self.CMD_STORE_CAL_3])

    def delete_calibration_profile(self):
        """Erase the stored calibration profile."""
        self._send_ok_sequence([self.CMD_DELETE_CAL_1,
                                 self.CMD_DELETE_CAL_2,
                                 self.CMD_DELETE_CAL_3])

    # ------------------------------------------------------------------
    # Baud rate change
    # ------------------------------------------------------------------
    def set_baud_19200(self):
        """
        Switch the module to 19200 baud. The OK byte is returned at the NEW
        baud rate, so the host's serial port must be reconfigured immediately
        after sending this command.
        """
        self.ser.reset_input_buffer()
        self.ser.write(bytes([self.CMD_BAUD_19200]))
        self.ser.baudrate = 19200
        data = self.ser.read(1)
        if len(data) != 1 or data[0] != self.OK_BYTE:
            raise IOError("CMPS12: failed to confirm baud rate change to 19200")

    def set_baud_38400(self):
        """Switch the module to 38400 baud (see note in set_baud_19200)."""
        self.ser.reset_input_buffer()
        self.ser.write(bytes([self.CMD_BAUD_38400]))
        self.ser.baudrate = 38400
        data = self.ser.read(1)
        if len(data) != 1 or data[0] != self.OK_BYTE:
            raise IOError("CMPS12: failed to confirm baud rate change to 38400")


# ------------------------------------------------------------------------
# Example usage
# ------------------------------------------------------------------------
if __name__ == "__main__":
    PORT = "COM7"   # change to your port, e.g. "COM5" on Windows

    with CMPS12(PORT, baudrate=9600) as compass:
        print("Software version:", compass.get_version())

        try:
            while True:
                all_data = compass.get_all()
                cal = compass.get_calibration_state()
                temp = compass.get_temperature()

                print(
                    f"Bearing: {all_data['bearing']:6.1f}  "
                    f"Pitch: {all_data['pitch']:4d}  "
                    f"Roll: {all_data['roll']:4d}  "
                    f"Temp: {temp:3d}C  "
                    f"Cal[sys/gyro/acc/mag]: "
                    f"{cal['system']}/{cal['gyro']}/{cal['accelerometer']}/{cal['magnetometer']}"
                )
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\nStopped.")