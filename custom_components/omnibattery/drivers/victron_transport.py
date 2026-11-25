"""How the Victron driver talks to a GX device: in D-Bus terms, not registers.

A GX device (Cerbo GX, Venus GX, Venus OS) publishes every value of the
installation on D-Bus as a *service* (``com.victronenergy.vebus`` for the
Multiplus/Quattro, ``.system`` for the aggregate, ``.settings``) plus a *path*
(``/Ac/Out/L1/P``). It forwards those values over more than one protocol —
Modbus-TCP and MQTT among them — each with its own addressing and quirks. The
driver only ever names service + path and works in physical units; a
:class:`GxTransport` maps that onto one protocol.

The contract every transport keeps:

* ``read`` returns ``None`` when the read itself failed (no answer, connection
  down): nothing is known, so nothing changes.
* A value of ``None`` inside a returned dict is the GX's own "I have no valid
  value for this" — D-Bus's invalid value. A transport that cannot express it
  natively (Modbus answers it with a plain 0) must recognise it and report
  ``None``; it must never hand the driver a placeholder as a real reading.
* Values are physical: W, V, %, 0/1 flags, plain ints for enums, str for text.

:class:`ModbusGxTransport` is the Modbus-TCP implementation. MQTT, where the GX
marks an invalid value explicitly, can be added as a second one without
touching the driver's control logic.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Protocol, Sequence

from ..infra.victron_modbus_client import (
    VictronModbusClient,
    decode_ascii,
    decode_i16,
    decode_u16,
    encode_i16,
)

_LOGGER = logging.getLogger(__name__)

# Services, by the last part of their D-Bus name.
SYSTEM = "system"
VEBUS = "vebus"
SETTINGS = "settings"

# Paths the driver uses.
SYSTEM_SERIAL = "/Serial"
SYSTEM_BATTERY_VOLTAGE = "/Dc/Battery/Voltage"
SYSTEM_BATTERY_POWER = "/Dc/Battery/Power"
SYSTEM_BATTERY_SOC = "/Dc/Battery/Soc"
VEBUS_AC_IN_L1_POWER = "/Ac/ActiveIn/L1/P"
VEBUS_AC_OUT_L1_POWER = "/Ac/Out/L1/P"
VEBUS_DC_VOLTAGE = "/Dc/0/Voltage"
VEBUS_AC_POWER_SETPOINT_L1 = "/Hub4/L1/AcPowerSetpoint"
VEBUS_DISABLE_CHARGE = "/Hub4/DisableCharge"
VEBUS_DISABLE_FEEDIN = "/Hub4/DisableFeedIn"
SETTINGS_HUB4_MODE = "/Settings/CGwacs/Hub4Mode"

# The path that vouches for a service's data: no running Multi sits on a 0 V
# battery, so a service that reports its DC voltage as invalid has no valid
# data at all (see ModbusGxTransport for why Modbus needs this).
SERVICE_WITNESS = {
    SYSTEM: SYSTEM_BATTERY_VOLTAGE,
    VEBUS: VEBUS_DC_VOLTAGE,
}
# Whose witness a service's values follow. Settings has no voltage of its own;
# it shares the system unit id and restarts with it, and during a restart its
# Hub4Mode reads 0 — not a mode — so it follows the system service.
TRUSTED_WITH = {
    SYSTEM: SYSTEM,
    SETTINGS: SYSTEM,
    VEBUS: VEBUS,
}


class GxTransport(Protocol):
    """One connection to one GX device, addressed by D-Bus service + path."""

    @property
    def connected(self) -> bool: ...

    async def connect(self) -> bool:
        """Open the connection; every service starts untrusted again."""

    async def close(self) -> None: ...

    def set_shutting_down(self, value: bool) -> None: ...

    async def read(
        self, service: str, paths: Sequence[str]
    ) -> Optional[dict[str, Any]]:
        """``{path: value}`` for every path asked, or None if the read failed."""

    async def write(self, service: str, values: Mapping[str, Any]) -> bool:
        """Write ``{path: value}`` together; True when the GX accepted it."""


# --- Modbus-TCP --------------------------------------------------------------

@dataclass(frozen=True)
class _Register:
    """Where one path lives on the GX's Modbus-TCP server, and its encoding."""

    address: int
    kind: str           # "u16", "i16" or "string"
    scale: float = 1    # physical value = raw * scale (numbers only)
    count: int = 1      # registers occupied (strings span several)


# Every register below is verified against attributes.csv from
# victronenergy/dbus_modbustcp — the actual table the server is built from.
_REGISTERS: dict[str, dict[str, _Register]] = {
    SYSTEM: {
        # The GX device's own serial, not a battery-pack serial.
        SYSTEM_SERIAL: _Register(800, "string", count=6),
        # 840-843 are contiguous (841 is the current, unused): one read.
        SYSTEM_BATTERY_VOLTAGE: _Register(840, "u16", scale=0.1),
        SYSTEM_BATTERY_POWER: _Register(842, "i16"),    # W, +charge/-discharge
        SYSTEM_BATTERY_SOC: _Register(843, "u16"),      # %
    },
    VEBUS: {
        # 12-26 are contiguous (AC-in and AC-out voltage, current, frequency
        # and L2/L3 power between them): one read, so the DC voltage that
        # vouches for both AC figures comes with them.
        VEBUS_AC_IN_L1_POWER: _Register(12, "i16", scale=10),   # 10 W per count, +import
        VEBUS_AC_OUT_L1_POWER: _Register(23, "i16", scale=10),  # 10 W per count
        VEBUS_DC_VOLTAGE: _Register(26, "u16", scale=0.01),
        # 37-39 are contiguous. They are marked write-only in attributes.csv;
        # the server's read path (verified against its source) applies no such
        # check, so reading them back works. L2/L3 setpoints (40/41) are not
        # used: single-phase only.
        VEBUS_AC_POWER_SETPOINT_L1: _Register(37, "i16"),  # W, +import/-export
        VEBUS_DISABLE_CHARGE: _Register(38, "u16"),
        VEBUS_DISABLE_FEEDIN: _Register(39, "u16"),
    },
    SETTINGS: {
        SETTINGS_HUB4_MODE: _Register(2902, "u16"),     # 3 = External control
    },
}


def _number(raw: int, scale: float) -> int | float:
    value = raw * scale
    return int(value) if float(scale).is_integer() else value


class ModbusGxTransport:
    """A GX device over Modbus-TCP: one connection, a unit id per service.

    The system and settings services share the system unit id (100 by
    default); the Multiplus/Quattro answers on its own vebus unit id.

    The invalid-value placeholder. dbus_modbustcp answers a register whose
    D-Bus value is invalid with a plain 0 — a successful read, no Modbus
    exception (Mappings::getValue: ``if (!dbusValue.isValid()) return 0;``).
    That happens while the GX's services restart, e.g. right after its
    Modbus-TCP port refused connections: on real hardware SOC went 67% -> 0% ->
    67% within 12s, and the fleet planned a grid charge for the missing 2.2 kWh.
    Each witnessed service (``SERVICE_WITNESS``) is therefore trusted only
    while its DC voltage reads above 0 V: a read that includes the witness
    sets the service's state, and a read that does not (37-39 have no voltage
    of their own) uses the state the last one left. Settings follow the
    system service (``TRUSTED_WITH``). While a service is not trusted every
    value from it is reported as None. Both witnessed services start untrusted
    on each (re)connect; a failed read changes nothing.
    """

    def __init__(
        self,
        host: str,
        port: int = 502,
        *,
        vebus_unit_id: int,
        system_unit_id: int = 100,
        client: Optional[VictronModbusClient] = None,
    ) -> None:
        self._client = client if client is not None else VictronModbusClient(host, port)
        self._unit_ids = {
            SYSTEM: int(system_unit_id),
            SETTINGS: int(system_unit_id),
            VEBUS: int(vebus_unit_id),
        }
        self._trusted = {service: False for service in SERVICE_WITNESS}

    @property
    def connected(self) -> bool:
        return self._client.connected

    async def connect(self) -> bool:
        if not await self._client.async_connect():
            return False
        for service in self._trusted:
            self._trusted[service] = False
        return True

    async def close(self) -> None:
        await self._client.async_close()

    def set_shutting_down(self, value: bool) -> None:
        self._client.set_shutting_down(value)

    def _span(self, service: str, paths: Sequence[str]) -> tuple[int, int]:
        regs = [_REGISTERS[service][path] for path in paths]
        start = min(r.address for r in regs)
        end = max(r.address + r.count for r in regs)
        return start, end - start

    async def read(
        self, service: str, paths: Sequence[str]
    ) -> Optional[dict[str, Any]]:
        # The paths of one call are read as one block spanning all of them:
        # callers ask for neighbours (see the register comments above), and
        # every register in between exists on the server.
        start, count = self._span(service, paths)
        regs = await self._client.async_read_holding_block(
            self._unit_ids[service], start, count
        )
        if regs is None:
            return None

        values: dict[str, Any] = {}
        for path in paths:
            reg = _REGISTERS[service][path]
            offset = reg.address - start
            if reg.kind == "string":
                values[path] = decode_ascii(regs[offset:offset + reg.count])
            elif reg.kind == "i16":
                values[path] = _number(decode_i16(regs, offset), reg.scale)
            else:
                values[path] = _number(decode_u16(regs, offset), reg.scale)

        witness = SERVICE_WITNESS.get(service)
        if witness is not None and witness in values:
            self._set_trusted(service, values[witness] != 0)
        if not self._trusted[TRUSTED_WITH[service]]:
            # Text needs no witness: its placeholder is all NULs, which
            # already decodes to None. That keeps the serial readable right
            # after connect, before any witness has been read.
            return {
                path: (value if _REGISTERS[service][path].kind == "string" else None)
                for path, value in values.items()
            }
        return values

    def _set_trusted(self, service: str, trusted: bool) -> None:
        if trusted != self._trusted[service]:
            if trusted:
                _LOGGER.info("Victron GX: %s service reports valid data again", service)
            else:
                _LOGGER.warning(
                    "Victron GX: %s service reports 0 V DC — it has no valid "
                    "data (services restarting?); its readings are reported as "
                    "unknown until it recovers",
                    service,
                )
        self._trusted[service] = trusted

    async def write(self, service: str, values: Mapping[str, Any]) -> bool:
        start, count = self._span(service, list(values))
        if count != len(values):
            raise ValueError(f"Victron GX: {list(values)} are not contiguous registers")
        words = [0] * count
        for path, value in values.items():
            reg = _REGISTERS[service][path]
            raw = round(value / reg.scale)
            words[reg.address - start] = (
                encode_i16(raw) if reg.kind == "i16" else max(0, min(0xFFFF, int(raw)))
            )
        return await self._client.async_write_registers(
            self._unit_ids[service], start, words
        )
