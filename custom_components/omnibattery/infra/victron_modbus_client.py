"""Async Modbus TCP client for a Victron GX device (Cerbo GX / Venus GX / Venus OS).

Victron's GX device runs a Modbus-TCP server that fans a single TCP connection
out to every D-Bus service on the box: the Multiplus/Quattro (``com.
victronenergy.vebus``), each communicating battery (``com.victronenergy.
battery``), the system aggregate (``com.victronenergy.system``), and more.
Unlike Marstek or Huawei — where one Modbus client addresses exactly one
physical device — the *unit id* here selects which service answers, over one
shared connection. This client is therefore unit-id-per-call rather than
unit-id-per-instance; the driver opens one connection and reads/writes several
unit ids across it, mirroring Huawei's separate inverter/EMMA clients but
without needing a second socket.

Registers themselves (which one is which, and what each means) live in
``drivers/victron.py``; this module only moves bytes.

No post-connect pause or single-in-flight constraint is applied here, unlike
``HuaweiModbusClient`` — that workaround is specific to the SUN2000's own
Modbus stack and there is no equivalent evidence for Venus OS's. If a GX
device turns out to need similar pacing, add it here once measured, not
speculatively.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Optional

from pymodbus.client import AsyncModbusTcpClient
from pymodbus.exceptions import ConnectionException, ModbusIOException

from .modbus_client import _detect_slave_kwarg

_LOGGER = logging.getLogger(__name__)

_REQUEST_TIMEOUT_S = 10.0
# Minimum spacing between requests on the shared connection. No hardware
# evidence sets this; it only exists so a 2 s control tick issuing several
# reads plus a write never pipelines faster than a generic Modbus-TCP server
# is guaranteed to tolerate.
_MESSAGE_WAIT_S = 0.02


def decode_u16(regs: list[int], offset: int = 0) -> int:
    return regs[offset] & 0xFFFF


def decode_i16(regs: list[int], offset: int = 0) -> int:
    value = regs[offset] & 0xFFFF
    return value - 0x10000 if value >= 0x8000 else value


def encode_i16(value: int) -> int:
    """Pack a signed value into one 16-bit Modbus register word."""
    value = max(-0x8000, min(0x7FFF, int(value)))
    return value & 0xFFFF


def decode_ascii(regs: list[int]) -> Optional[str]:
    """Decode a ``string[N]`` register run: 2 ASCII bytes per register, NUL-padded.

    Same convention as Huawei's string registers (HuaweiModbusClient.decode_string)
    — standard Modbus string packing, not yet confirmed against a real Victron
    device since every other register this driver reads is numeric.
    """
    raw = bytearray()
    for reg in regs:
        raw.append((reg >> 8) & 0xFF)
        raw.append(reg & 0xFF)
    text = raw.split(b"\x00")[0].decode("ascii", "replace").strip()
    return text or None


class VictronModbusClient:
    """Thin async Modbus TCP transport shared across a GX device's unit ids.

    One instance owns one TCP connection to one GX device. Every read/write
    takes the target ``unit_id`` as an explicit argument, since the same
    connection serves the vebus, battery, and system services at different
    unit ids (Victron's own convention — see the GX Modbus-TCP manual).
    """

    def __init__(self, host: str, port: int = 502, timeout: float = _REQUEST_TIMEOUT_S) -> None:
        self.host = host
        self.port = port
        self._timeout = timeout
        self._client: Optional[AsyncModbusTcpClient] = None
        self._slave_kwarg = "slave"
        self._is_shutting_down = False
        self._lock = asyncio.Lock()

    @property
    def connected(self) -> bool:
        client = self._client
        return bool(client is not None and getattr(client, "connected", False))

    def set_shutting_down(self, value: bool) -> None:
        self._is_shutting_down = bool(value)

    async def async_connect(self) -> bool:
        await self.async_close()
        client = AsyncModbusTcpClient(host=self.host, port=self.port, timeout=self._timeout)
        try:
            connected = await client.connect()
        except Exception as err:
            _LOGGER.error("Victron Modbus connect failed %s:%s: %s", self.host, self.port, err)
            await _close_quietly(client)
            return False
        if not connected:
            _LOGGER.error("Victron Modbus connect refused %s:%s", self.host, self.port)
            await _close_quietly(client)
            return False
        self._client = client
        self._slave_kwarg = _detect_slave_kwarg(client)
        return True

    async def async_close(self) -> None:
        client = self._client
        self._client = None
        if client is not None:
            await _close_quietly(client)

    async def async_read_holding_block(
        self, unit_id: int, start: int, count: int
    ) -> Optional[list[int]]:
        """FC03: read ``count`` contiguous holding registers from ``unit_id``."""
        if not (0 <= start <= 0xFFFF) or not (1 <= count <= 125):
            _LOGGER.error("Invalid Victron Modbus read start=%s count=%s", start, count)
            return None
        if self._client is None or not self.connected:
            return None

        kwargs = {self._slave_kwarg: unit_id}
        async with self._lock:
            try:
                try:
                    result = await asyncio.wait_for(
                        self._client.read_holding_registers(address=start, count=count, **kwargs),
                        timeout=self._timeout,
                    )
                finally:
                    if not self._is_shutting_down:
                        await asyncio.sleep(_MESSAGE_WAIT_S)
            except (ConnectionException, ModbusIOException, asyncio.TimeoutError):
                if not self._is_shutting_down:
                    _LOGGER.debug(
                        "Victron Modbus connection error reading unit %s @ %d (count=%d)",
                        unit_id, start, count,
                    )
                return None
            except Exception as err:
                if not self._is_shutting_down:
                    _LOGGER.exception(
                        "Victron Modbus exception reading unit %s @ %d: %s", unit_id, start, err
                    )
                return None

        if result.isError():
            if not self._is_shutting_down:
                _LOGGER.debug(
                    "Victron Modbus read error on unit %s @ %d (count=%d)", unit_id, start, count
                )
            return None
        regs = getattr(result, "registers", None)
        if regs is None or len(regs) < count:
            if not self._is_shutting_down:
                _LOGGER.warning(
                    "Victron Modbus incomplete read on unit %s @ %d: got %s expected %d",
                    unit_id, start, len(regs) if regs else 0, count,
                )
            return None
        return list(regs[:count])

    async def async_write_registers(self, unit_id: int, start: int, values: list[int]) -> bool:
        """FC16: write ``values`` to consecutive holding registers from ``start``.

        One function code covers every write here, including the single-
        register ``AcPowerSetpoint`` write — FC16 is a superset of FC06 and
        every Modbus-TCP server is required to support it.
        """
        if not (0 <= start <= 0xFFFF) or not (1 <= len(values) <= 123):
            _LOGGER.error("Invalid Victron Modbus write start=%s count=%s", start, len(values))
            return False
        if self._client is None or not self.connected:
            return False

        kwargs = {self._slave_kwarg: unit_id}
        async with self._lock:
            try:
                try:
                    result = await asyncio.wait_for(
                        self._client.write_registers(address=start, values=list(values), **kwargs),
                        timeout=self._timeout,
                    )
                finally:
                    if not self._is_shutting_down:
                        await asyncio.sleep(_MESSAGE_WAIT_S)
            except (ConnectionException, ModbusIOException, asyncio.TimeoutError) as err:
                if not self._is_shutting_down:
                    _LOGGER.warning(
                        "Victron Modbus write to unit %s @ %d failed: %s", unit_id, start, err
                    )
                return False
            except Exception as err:
                if not self._is_shutting_down:
                    _LOGGER.exception(
                        "Victron Modbus exception writing unit %s @ %d: %s", unit_id, start, err
                    )
                return False

        if result.isError():
            if not self._is_shutting_down:
                _LOGGER.warning(
                    "Victron Modbus write rejected on unit %s @ %d (values=%s)",
                    unit_id, start, values,
                )
            return False
        return True


async def _close_quietly(client) -> None:
    try:
        result = client.close()
        if asyncio.iscoroutine(result):
            await result
    except Exception as err:
        _LOGGER.debug("Error closing Victron Modbus connection: %s", err)
