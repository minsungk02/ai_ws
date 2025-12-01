"""
Utility helpers for communicating with the Arduino-based steering/throttle
controller.  Every perception/control node should import this module instead of
duplicating Serial code.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import rospy

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover - dependency error should be obvious
    raise RuntimeError("pyserial is required for common_serial") from exc


DEFAULT_PORT = "/dev/arduino"
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT = 0.5
INITIAL_CONNECT_DELAY = 2.5  # allow Arduino/ESC to fully reset/arm after port open


def clamp(value: int, minimum: int = 45, maximum: int = 135) -> int:
    """Clamp steering values into the Arduino-safe range."""
    return max(minimum, min(maximum, value))


def create_command(steering: int, speed: int) -> bytearray:
    """
    Construct the 0xEA framed command packet the Arduino firmware expects.

    Args:
        steering: Servo target (45~135).
        speed: ESC value (0~180 typical, 90=stop).
    """
    steering = clamp(steering)
    speed = clamp(speed, minimum=0, maximum=180)
    STX = 0xEA
    ETX = 0x03
    length = 0x04
    dummy1 = 0x00
    dummy2 = 0x00
    checksum = ((~(length + steering + speed + dummy1 + dummy2)) & 0xFF) + 1
    return bytearray([STX, length, steering, speed, dummy1, dummy2, checksum, ETX])


class SerialBridge:
    """
    Thread-safe wrapper around pyserial that automatically sends stop commands on
    shutdown and exposes helper utilities for other nodes.
    """

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baudrate: int = DEFAULT_BAUD,
        timeout: float = DEFAULT_TIMEOUT,
        auto_connect: bool = True,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        if auto_connect:
            self.connect()
        rospy.on_shutdown(self.stop_vehicle)

    # --------------------------------------------------------------------- utils
    def connect(self) -> None:
        """Open the serial port if it is not already open."""
        if self.is_open:
            return

        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
            )
            rospy.loginfo("SerialBridge connected to %s @ %d", self.port, self.baudrate)
            time.sleep(INITIAL_CONNECT_DELAY)  # allow Arduino/ESC to reset and arm
        except serial.SerialException as exc:
            rospy.logerr("Failed to open %s: %s", self.port, exc)
            raise

    def close(self) -> None:
        """Close the serial port."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            rospy.loginfo("SerialBridge closed %s", self.port)

    @property
    def is_open(self) -> bool:
        return bool(self._serial and self._serial.is_open)

    # ----------------------------------------------------------------- operations
    def send(self, steering: int, speed: int) -> None:
        """Serialize and transmit a command packet."""
        packet = create_command(steering, speed)
        with self._lock:
            if not self.is_open:
                self.connect()
            assert self._serial is not None  # self.is_open ensures this
            self._serial.write(packet)

    def write_raw(self, payload: bytes) -> None:
        """Low-level passthrough for special commands."""
        with self._lock:
            if not self.is_open:
                self.connect()
            assert self._serial is not None
            self._serial.write(payload)

    def readline(self) -> str:
        """Read a single line from the serial port (used for logging datasets)."""
        if not self.is_open:
            self.connect()
        assert self._serial is not None
        return self._serial.readline().decode(errors="ignore").strip()

    def stop_vehicle(self) -> None:
        """
        Send a neutral steering/stop command. This is automatically invoked when
        ROS shuts down to keep the vehicle safe.
        """
        try:
            if self.is_open:
                self._serial.write(create_command(90, 90))  # type: ignore[arg-type]
                self._serial.flush()
        except serial.SerialException as exc:
            rospy.logwarn("Failed to send stop command: %s", exc)
        finally:
            self.close()
