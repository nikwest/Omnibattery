"""Victron ESS external-control driver (Multiplus/Quattro behind a GX device).

Unlike every other brand here, "the battery" is not the thing this driver
talks to for control. A Multiplus II is an inverter/charger, not a battery
with its own BMS; the signed command this driver issues is a *grid-point*
power target (Victron's ESS "external control", Hub-4 Mode 3), and the GX
device's own fast loop converts that into whatever charge/discharge the
battery behind it needs, automatically netting out whatever AC load the
Multi is serving. See ``docs/research/victron/driver-assessment-victron.md``
for the full writeup; the short version:

* ``Hub4/L1/AcPowerSetpoint`` (register 37) is the control point. Positive
  draws from the grid, negative feeds it — **confirmed against Victron's own
  "ESS mode 2 and 3" documentation**, not merely a community reference.
* It must be rewritten at least once every 60 s or the Multi reverts to
  passthrough (also Victron's own documentation). That is a *safety*
  property, not a nuisance: if Omnibattery stops running, control is handed
  back within a minute without anything else having to notice. See
  ``DriverCapabilities.setpoint_refresh_interval_s``.
* Net power maps onto ``AcPowerSetpoint`` with **no sign flip** — but only
  because this installation's Multi carries the whole house's AC-out load at
  the same point the grid sensor measures. That assumption was reasoned
  through, not measured; confirm it with a small step command (+200 W then
  -200 W) against the real grid sensor before trusting this for automatic
  control. A parallel-metered topology (the Multi is one of several
  AC-coupled sources, not the single path to the meter) would need a
  different translation and is explicitly out of scope here.
* Three-phase systems have independent setpoints per phase (registers 37/40/
  41 for L1/L2/L3). This driver writes L1 only — single-phase installations
  only, for now.

Telemetry comes from ``com.victronenergy.system`` (unit id 100, Victron's own
documented default for the system aggregate), not from a per-battery
``com.victronenergy.battery`` service. That is deliberate, not a fallback: a
GX device always exposes the system aggregate, while whether a separate
per-battery Modbus service exists at all depends on the BMS and how Venus OS
enumerates it — DVCC sees a CAN-bus battery regardless, but the Modbus-TCP
bridge does not necessarily forward it as its own service. Registers 842
(``Dc/Battery/Power``, int16, W, +charge/-discharge) and 843 (``Dc/Battery/
Soc``, uint16, 0-100%) were confirmed against a real installation's register
map during review — not merely a secondhand reference — which is why this
is the primary telemetry path rather than a documented fallback.
"""

from __future__ import annotations

import logging
from dataclasses import replace
from typing import Optional

from ..infra.victron_modbus_client import (
    VictronModbusClient,
    decode_i16,
    decode_u16,
    encode_i16,
)
from .base import (
    BatteryDriver,
    DriverCapabilities,
    ReadGroup,
    SetpointResult,
    TelemetrySnapshot,
)

_LOGGER = logging.getLogger(__name__)

# --- com.victronenergy.vebus (the Multiplus/Quattro itself) -----------------
# CONFIRMED against https://www.victronenergy.com/live/ess:ess_mode_2_and_3
_REG_VEBUS_AC_POWER_SETPOINT_L1 = 37   # int16, W, +import/-export (Hub4/L1/AcPowerSetpoint)
_REG_VEBUS_AC_POWER_SETPOINT_L2 = 40   # int16, W (L2, three-phase only — not written by this driver)
_REG_VEBUS_AC_POWER_SETPOINT_L3 = 41   # int16, W (L3, three-phase only — not written by this driver)
_REG_VEBUS_DISABLE_CHARGE = 38         # uint16, 0/1 (Hub4/DisableCharge)
_REG_VEBUS_DISABLE_FEEDIN = 39         # uint16, 0/1 (Hub4/DisabledFeedIn)

# --- com.victronenergy.system, unit id 100 (Victron's documented default) --
# CONFIRMED against a real installation's register map, not a secondhand
# reference. This is the system-wide DC battery aggregate, read regardless of
# whether this specific battery also answers as its own Modbus service.
_REG_SYSTEM_BATTERY_POWER = 842  # int16, W, +charge/-discharge, no scaling
_REG_SYSTEM_BATTERY_SOC = 843    # uint16, 0-100%, no scaling

# Hard-coded by the Multi's own regulation, not configurable. CONFIRMED
# against the same ESS Mode 2/3 documentation as the setpoint register.
_RAMP_LIMIT_W_PER_S = 400.0

# Also CONFIRMED there: the watchdog that falls back to passthrough.
_HARDWARE_WATCHDOG_S = 60.0
# Rewritten this much sooner than the hardware deadline. A 2 s control tick
# crosses this many times before the real deadline, so losing one write to a
# transient failure is not a near miss.
_SETPOINT_REFRESH_INTERVAL_S = 45.0

# No measured latency exists for this driver yet (see module docstring); this
# is a reasoned floor, not a measurement; see model_label for how this reads
# in review.
_MIN_ACTUATOR_LATENCY_S = 2.0

_WRITE_CONTROLS = {
    "disable_charge": _REG_VEBUS_DISABLE_CHARGE,
    "disable_feedin": _REG_VEBUS_DISABLE_FEEDIN,
}

SENSOR_DEFINITIONS: list[dict] = [
    {"key": "battery_soc", "name": "Battery SOC", "unit": "%", "device_class": "battery",
     "state_class": "measurement", "scale": 1, "precision": 1, "scan_interval": "high",
     "enabled_by_default": True},
    {"key": "battery_power", "name": "Battery Power", "unit": "W", "device_class": "power",
     "state_class": "measurement", "scale": 1, "precision": 0, "scan_interval": "high",
     "enabled_by_default": True},
    {"key": "ac_power_setpoint", "name": "ESS AC Power Setpoint", "unit": "W",
     "device_class": "power", "state_class": "measurement", "scale": 1, "precision": 0,
     "category": "diagnostic", "scan_interval": "low", "enabled_by_default": True},
]
NUMBER_DEFINITIONS: list[dict] = []
SELECT_DEFINITIONS: list[dict] = []
SWITCH_DEFINITIONS: list[dict] = [
    {"key": "disable_charge", "name": "ESS Disable Charge", "icon": "mdi:battery-charging-off",
     "category": "diagnostic", "enabled_by_default": True, "command_on": 1, "command_off": 0},
    {"key": "disable_feedin", "name": "ESS Disable Feed-in", "icon": "mdi:transmission-tower-off",
     "category": "diagnostic", "enabled_by_default": True, "command_on": 1, "command_off": 0},
]
BINARY_SENSOR_DEFINITIONS: list[dict] = []
BUTTON_DEFINITIONS: list[dict] = []


class VictronEssDriver(BatteryDriver):
    """One Victron GX device's Multiplus/Quattro, driven through ESS Mode 3."""

    def __init__(
        self,
        host: str,
        port: int = 502,
        *,
        vebus_unit_id: int,
        system_unit_id: int = 100,
        max_charge_power_w: int,
        max_discharge_power_w: int,
        client: Optional[VictronModbusClient] = None,
    ) -> None:
        self.host = host
        self.port = port
        self._vebus_unit_id = int(vebus_unit_id)
        self._system_unit_id = int(system_unit_id)
        self._client = client if client is not None else VictronModbusClient(host, port)
        self._shutting_down = False
        # Last value actually written, so net_power_from_data has something to
        # report even before the first telemetry poll confirms it, and so a
        # failed write is never mistaken for an applied one.
        self._last_written_w: Optional[int] = None

        max_charge_power_w = max(0, int(max_charge_power_w))
        max_discharge_power_w = max(0, int(max_discharge_power_w))
        # Worst case is a full reversal between the two configured ceilings,
        # at the Multi's own fixed 400 W/s ramp. Measured hardware would very
        # likely show something lower for any realistic PD command size; this
        # is the conservative, reasoned bound until it is.
        reversal_s = (max_charge_power_w + max_discharge_power_w) / _RAMP_LIMIT_W_PER_S
        actuator_latency_s = max(_MIN_ACTUATOR_LATENCY_S, reversal_s)

        self._capabilities = DriverCapabilities(
            # No confirmed register ties the user's configured SOC window to
            # anything the BMS enforces on its own; software owns the floor
            # and ceiling here, same as most brands. The BMS's own protections
            # remain active regardless — that is DVCC's job, not this driver's.
            hardware_soc_cutoff=False,
            has_force_mode=False,
            push_telemetry=False,
            max_charge_power_w=max_charge_power_w,
            max_discharge_power_w=max_discharge_power_w,
            has_mppt_pv=False,
            has_alarm_registers=False,
            # Toggling ESS Mode itself (Hub4Mode) is a Settings-service write
            # whose Modbus-TCP exposure is unconfirmed — see the module
            # docstring. Claiming this capability would promise a gate this
            # driver cannot actually operate.
            has_rs485_control=False,
            has_energy_counters=False,
            has_daily_energy_counters=False,
            has_nominal_capacity=False,
            setpoint_confirm_reliable=True,
            actuator_latency_s=actuator_latency_s,
            readback_latency_s=actuator_latency_s,
            setpoint_refresh_interval_s=_SETPOINT_REFRESH_INTERVAL_S,
        )
        self._read_groups = [
            ReadGroup("high", ("battery_soc", "battery_power")),
            ReadGroup("low", ("ac_power_setpoint", "disable_charge", "disable_feedin")),
        ]

    # --- identity -------------------------------------------------------

    @property
    def capabilities(self) -> DriverCapabilities:
        return self._capabilities

    @property
    def model_label(self) -> Optional[str]:
        return "Victron ESS (Multiplus/Quattro)"

    # --- connection lifecycle --------------------------------------------

    @property
    def connected(self) -> bool:
        return self._client.connected

    async def connect(self) -> bool:
        return await self._client.async_connect()

    async def close(self) -> None:
        await self._client.async_close()

    def set_shutting_down(self, value: bool) -> None:
        self._shutting_down = bool(value)
        self._client.set_shutting_down(value)

    # --- telemetry (read) -------------------------------------------------

    @property
    def read_groups(self) -> list[ReadGroup]:
        return self._read_groups

    async def read_telemetry(self, keys: Optional[list[str]] = None) -> TelemetrySnapshot:
        requested = set(keys) if keys is not None else None
        snapshot: TelemetrySnapshot = {}

        if requested is None or {"battery_soc", "battery_power"} & requested:
            regs = await self._client.async_read_holding_block(
                self._system_unit_id, _REG_SYSTEM_BATTERY_SOC, 1
            )
            if regs is not None:
                soc = decode_u16(regs, 0)
                if 0 <= soc <= 100:
                    snapshot["battery_soc"] = soc
                else:
                    _LOGGER.warning(
                        "Victron driver: dropping battery_soc=%s — outside 0-100%%. "
                        "Confirm the system unit id (default 100) is correct for "
                        "this GX device; see "
                        "docs/research/victron/driver-assessment-victron.md",
                        soc,
                    )
            regs = await self._client.async_read_holding_block(
                self._system_unit_id, _REG_SYSTEM_BATTERY_POWER, 1
            )
            if regs is not None:
                snapshot["battery_power"] = decode_i16(regs, 0)

        if requested is None or "ac_power_setpoint" in requested:
            regs = await self._client.async_read_holding_block(
                self._vebus_unit_id, _REG_VEBUS_AC_POWER_SETPOINT_L1, 1
            )
            if regs is not None:
                snapshot["ac_power_setpoint"] = decode_i16(regs, 0)

        if requested is None or {"disable_charge", "disable_feedin"} & requested:
            regs = await self._client.async_read_holding_block(
                self._vebus_unit_id, _REG_VEBUS_DISABLE_CHARGE, 2
            )
            if regs is not None:
                snapshot["disable_charge"] = decode_u16(regs, 0)
                snapshot["disable_feedin"] = decode_u16(regs, 1)

        return snapshot

    @property
    def control_dependency_keys(self) -> frozenset:
        return frozenset({"battery_soc", "battery_power", "ac_power_setpoint"})

    # --- control (write) ----------------------------------------------------

    def _echo(self, applied: int) -> dict:
        return {"ac_power_setpoint": applied}

    async def apply_setpoint(
        self,
        net_power_w: int,
        *,
        mode_hint: Optional[str] = None,
        read_back: bool = True,
    ) -> SetpointResult:
        applied = max(
            -self._capabilities.max_discharge_power_w,
            min(self._capabilities.max_charge_power_w, int(net_power_w)),
        )
        ok = await self._client.async_write_registers(
            self._vebus_unit_id, _REG_VEBUS_AC_POWER_SETPOINT_L1, [encode_i16(applied)]
        )
        if not ok:
            return SetpointResult(
                ok=False, net_power_w=applied, confirmed=False, failure_reason="write_failed"
            )
        self._last_written_w = applied

        if not read_back:
            return SetpointResult(
                ok=True, net_power_w=applied, confirmed=False, applied=self._echo(applied)
            )

        regs = await self._client.async_read_holding_block(
            self._vebus_unit_id, _REG_VEBUS_AC_POWER_SETPOINT_L1, 1
        )
        if regs is None:
            # The write itself succeeded; only the confirmation read failed.
            return SetpointResult(
                ok=True, net_power_w=applied, confirmed=False, applied=self._echo(applied)
            )
        echoed = decode_i16(regs, 0)
        return SetpointResult(
            ok=True,
            net_power_w=applied,
            confirmed=echoed == applied,
            battery_power_w=None,
            applied=self._echo(echoed),
        )

    async def write_control(self, key: str, value: int) -> bool:
        register = _WRITE_CONTROLS.get(key)
        if register is None:
            return False
        return await self._client.async_write_registers(
            self._vebus_unit_id, register, [1 if value else 0]
        )

    def net_power_from_data(self, data: dict) -> Optional[int]:
        value = data.get("ac_power_setpoint")
        if value is None:
            return None
        return int(value)

    async def apply_config(
        self,
        max_soc_pct: Optional[float] = None,
        min_soc_pct: Optional[float] = None,
        max_charge_power_w: Optional[int] = None,
        max_discharge_power_w: Optional[int] = None,
    ) -> None:
        """Update the software power ceiling. SOC window is software-owned
        elsewhere (see ``hardware_soc_cutoff``) and deliberately ignored here."""
        updates = {}
        if max_charge_power_w is not None:
            updates["max_charge_power_w"] = max(0, int(max_charge_power_w))
        if max_discharge_power_w is not None:
            updates["max_discharge_power_w"] = max(0, int(max_discharge_power_w))
        if updates:
            self._capabilities = replace(self._capabilities, **updates)

    async def standby(self) -> bool:
        """Release control the only way this driver can currently guarantee.

        This does **not** restore the Multi's own ESS mode — Hub4Mode's
        Modbus-TCP writability is unconfirmed (see module docstring). It stops
        writing and relies on the hardware's own watchdog: the Multi falls
        back to passthrough within ``_HARDWARE_WATCHDOG_S`` of the last
        AcPowerSetpoint write on its own. Restoring the GX device's ESS mode
        to its own control, if that turns out to be writable, is a follow-up.
        """
        _LOGGER.info(
            "Victron driver: standby — no further AcPowerSetpoint writes will be "
            "issued; the Multi returns to passthrough within %.0fs on its own",
            _HARDWARE_WATCHDOG_S,
        )
        return True

    async def set_charge_cutoff(self, soc_pct: float) -> bool:
        return False

    async def set_rs485_control(self, enable: bool) -> bool:
        return False

    # --- entity definitions ------------------------------------------------

    @property
    def sensor_definitions(self) -> list[dict]:
        return SENSOR_DEFINITIONS

    @property
    def number_definitions(self) -> list[dict]:
        return NUMBER_DEFINITIONS

    @property
    def select_definitions(self) -> list[dict]:
        return SELECT_DEFINITIONS

    @property
    def switch_definitions(self) -> list[dict]:
        return SWITCH_DEFINITIONS

    @property
    def binary_sensor_definitions(self) -> list[dict]:
        return BINARY_SENSOR_DEFINITIONS

    @property
    def button_definitions(self) -> list[dict]:
        return BUTTON_DEFINITIONS

    @property
    def all_definitions(self) -> list[dict]:
        return (
            self.sensor_definitions
            + NUMBER_DEFINITIONS
            + SELECT_DEFINITIONS
            + SWITCH_DEFINITIONS
            + BINARY_SENSOR_DEFINITIONS
            + BUTTON_DEFINITIONS
        )

    # --- setup helpers -------------------------------------------------------

    @classmethod
    async def probe(
        cls, host: str, port: int, vebus_unit_id: int, system_unit_id: int = 100
    ) -> bool:
        """Best-effort connectivity check for the config flow.

        Confirms both unit ids answer a read at all — it does not confirm the
        register *map* is correct for this Venus OS version, which is a
        hardware-validation step this probe cannot perform.
        """
        client = VictronModbusClient(host, port)
        try:
            if not await client.async_connect():
                return False
            vebus_regs = await client.async_read_holding_block(
                int(vebus_unit_id), _REG_VEBUS_AC_POWER_SETPOINT_L1, 1
            )
            system_regs = await client.async_read_holding_block(
                int(system_unit_id), _REG_SYSTEM_BATTERY_SOC, 1
            )
            return vebus_regs is not None and system_regs is not None
        finally:
            await client.async_close()
