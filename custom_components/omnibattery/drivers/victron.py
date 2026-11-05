"""Victron ESS external-control driver (Multiplus/Quattro behind a GX device).

Unlike every other brand here, "the battery" is not the thing this driver
writes to for control. A Multiplus II is an inverter/charger, not a battery
with its own BMS; the register this driver writes is a *grid-point* (AC-input)
power target (Victron's ESS "external control", Hub-4 Mode 3), not a battery
power target. See ``docs/research/victron/driver-assessment-victron.md`` for
the full writeup; the short version:

* ``Hub4/L1/AcPowerSetpoint`` (register 37) is the control point. Positive
  draws from the grid, negative feeds it — **confirmed against Victron's own
  "ESS mode 2 and 3" documentation**.
* **The register targets AC-input power, not battery power — these are not
  the same quantity whenever AC-out load is nonzero.** Energy balance at the
  Multi is ``AC_in = AC_out + battery_power`` (+battery = charging). Writing
  the desired battery power straight into the register — this driver's first
  version — asks the Multi to make ``AC_in`` equal that number, which only
  equals the intended battery command when ``AC_out`` happens to be zero.
  With a 2000 W house load and a "charge at 500 W" intent, the earlier code
  asked for ``AC_in = 500``, which the Multi can only reach by *discharging*
  1500 W (``500 - 2000 = -1500``) — the opposite of what was asked, confirmed
  by Victron's own community documentation: "the point of control is at the
  AC-input of the Multi, and if there are loads on the output, your control
  loop needs to compensate for that." This driver now reads AC consumption
  (register 817, system aggregate) and writes
  ``AcPowerSetpoint = battery_power_wanted + ac_consumption_now`` instead of
  the battery figure alone. A reading of ``0 W`` wanted now correctly means
  battery-idle (``AC_in`` tracks consumption exactly, battery untouched) —
  it did not before this fix.
* It must be rewritten at least once every 60 s or the Multi reverts to
  passthrough (also Victron's own documentation). That is a *safety*
  property: if Omnibattery stops running, control is handed back within a
  minute without anything else having to notice. See
  ``DriverCapabilities.setpoint_refresh_interval_s``.
* Switching ESS Mode to External control **disables BatteryLife** — Victron's
  own battery-protection/optimisation behaviour — for as long as external
  control is active. This is Victron's documented design, not a side effect
  of this driver, but it is a real consequence the operator should know about.
* Three-phase systems have independent setpoints per phase (registers 37/40/
  41 for L1/L2/L3, and per-phase consumption registers). This driver writes
  and compensates for L1 only — single-phase installations only, for now.
* A write's "confirmed" flag means the register echoed back what was
  written — **it does not mean the Multi achieved that power.** Victron's own
  documentation recommends reading the Multi's actual delivered power rather
  than trusting the command was achievable; a managed battery's own
  charge/discharge current limit can cap what the Multi can actually do
  regardless of what was asked. This driver reads the battery power register
  (842) on every confirmed write and reports it as ``battery_power_w`` on the
  result, which is what Omnibattery's existing non-responsive-delivery
  tracking (shared across every brand) uses to detect exactly this gap — it
  is not something this driver has to re-implement itself.

Telemetry comes from ``com.victronenergy.system`` (unit id 100, Victron's own
documented default for the system aggregate), not from a per-battery
``com.victronenergy.battery`` service. That is deliberate, not a fallback: a
GX device always exposes the system aggregate, while whether a separate
per-battery Modbus service exists at all depends on the BMS and how Venus OS
enumerates it — DVCC sees a CAN-bus battery regardless, but the Modbus-TCP
bridge does not necessarily forward it as its own service. Every register
this driver reads or writes (37, 38, 39, 817, 842, 843, 930) is verified
against ``attributes.csv`` from the ``victronenergy/dbus_modbustcp`` GitHub
repository — the actual table the Modbus-TCP server is generated from — not
a secondhand transcription.

``/SystemState/State`` (register 930, same unit id; ``252`` means External
control) is read on a best-effort basis to confirm ESS is actually in
external control before writing, and to detect when it visibly is not (e.g.
``Passthru`` — the watchdog already fired). A failed read is treated as
unknown, not as a failure. Writing ESS Mode back on ``standby()`` is not
implemented: no source consulted confirms a write to that settings register
actually succeeds, and guessing wrong risks leaving the system in a worse
state than just waiting out the 60 s watchdog.
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

# Every register below (type, scale, access) is verified against the
# authoritative source: attributes.csv from the victronenergy/dbus_modbustcp
# GitHub repo — the actual table its Modbus-TCP server is built from, not a
# secondhand transcription. All of them use scale 1 (no fixed-point decoding
# needed); the register number is the whole translation.

# --- com.victronenergy.vebus (the Multiplus/Quattro itself) -----------------
_REG_VEBUS_AC_POWER_SETPOINT_L1 = 37   # int16, W, +import/-export (Hub4/L1/AcPowerSetpoint)
_REG_VEBUS_AC_POWER_SETPOINT_L2 = 40   # int16, W (L2, three-phase only — not written by this driver)
_REG_VEBUS_AC_POWER_SETPOINT_L3 = 41   # int16, W (L3, three-phase only — not written by this driver)
_REG_VEBUS_DISABLE_CHARGE = 38         # uint16, 0/1 (Hub4/DisableCharge)
_REG_VEBUS_DISABLE_FEEDIN = 39         # uint16, 0/1 (Hub4/DisableFeedIn)
# attributes.csv marks 37/38/39 write-only ("W"), which only means the
# server's write path enforces it — its read path (Mappings::getValues)
# applies no such check, and the Multi's own control loop has to continuously
# read these to act on them. Reading them back, as this driver does, works.

# --- com.victronenergy.system, unit id 100 (Victron's documented default) --
# The system-wide DC battery aggregate, read regardless of whether this
# specific battery also answers as its own Modbus service.
_REG_SYSTEM_BATTERY_POWER = 842       # int16, W, +charge/-discharge
_REG_SYSTEM_BATTERY_SOC = 843         # uint16, 0-100%
_REG_SYSTEM_AC_CONSUMPTION_L1 = 817   # uint16, W — house load the Multi is serving on AC-out,
                                       # the feedforward term apply_setpoint needs (see module docstring)
# Aggregated operating state; 252 specifically means External control (DVCC
# terminology) and a real-world report confirms it holds at 252 throughout —
# it does not cycle through the normal charge-state values while external
# control is active, which is what makes it a clean signal rather than a
# flaky one. Read-only, so there is no ambiguity about whether this is a
# writable-only path the way there was a reason to double-check above.
_REG_SYSTEM_STATE = 930
_SYSTEM_STATE_EXTERNAL_CONTROL = 252
# The handful of other values worth naming for the diagnostic sensor — in
# particular 8 (Passthru), which is exactly what seeing this value means the
# 60s watchdog already fired. Not exhaustive; an unlisted value still prints
# as "Not external control (state N)" rather than being hidden.
_SYSTEM_STATE_LABELS = {
    0: "Off", 1: "Low power", 2: "VE.Bus fault", 8: "Passthru", 9: "Inverting",
}

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
# is a reasoned floor, not a measurement.
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
    {"key": "ac_consumption", "name": "AC Consumption", "unit": "W",
     "device_class": "power", "state_class": "measurement", "scale": 1, "precision": 0,
     "category": "diagnostic", "scan_interval": "high", "enabled_by_default": True},
    {"key": "ess_control_mode", "name": "ESS Control Mode", "data_type": "char",
     "icon": "mdi:state-machine", "category": "diagnostic", "scan_interval": "low",
     "enabled_by_default": True},
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
        # AC consumption feedforward term apply_setpoint needs to translate a
        # battery-power intent into the grid-point register (see module
        # docstring). None until the first successful read — apply_setpoint
        # refuses to write rather than guess while this is unknown, since a
        # wrong guess reproduces the exact bug this field exists to fix.
        self._ac_consumption_w: Optional[float] = None
        # Best-effort ESS-mode confirmation (SystemState/State == 252). None =
        # unknown (not yet read) — treated as "proceed, we cannot tell"; False
        # = positively confirmed NOT external control — treated as "refuse to
        # write, it would be pointless or misleading".
        self._ess_mode_confirmed_external: Optional[bool] = None

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
            # SystemState/State is read for a safety check (is ESS actually in
            # External control?), but this driver has no confirmed way to
            # *write* ESS Mode back — see the module docstring. Claiming this
            # capability would promise a toggle/confirm gate this driver
            # cannot actually operate.
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
            # ac_consumption is in the fast group, not "low": apply_setpoint
            # needs a fresh value every write, not whatever was last read tens
            # of seconds ago.
            ReadGroup("high", ("battery_soc", "battery_power", "ac_consumption")),
            ReadGroup("low", ("ac_power_setpoint", "disable_charge", "disable_feedin", "ess_control_mode")),
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

        if requested is None or "ac_consumption" in requested:
            regs = await self._client.async_read_holding_block(
                self._system_unit_id, _REG_SYSTEM_AC_CONSUMPTION_L1, 1
            )
            if regs is not None:
                consumption = decode_u16(regs, 0)
                self._ac_consumption_w = consumption
                snapshot["ac_consumption"] = consumption
            # A failed read leaves self._ac_consumption_w at its last known
            # value rather than clearing it to None — a single missed poll
            # should not immediately refuse every write; apply_setpoint only
            # ever sees None here before the very first successful read.

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

        if requested is None or "ess_control_mode" in requested:
            regs = await self._client.async_read_holding_block(
                self._system_unit_id, _REG_SYSTEM_STATE, 1
            )
            if regs is not None:
                state = decode_u16(regs, 0)
                self._ess_mode_confirmed_external = state == _SYSTEM_STATE_EXTERNAL_CONTROL
                snapshot["ess_control_mode"] = (
                    "External control" if self._ess_mode_confirmed_external
                    else _SYSTEM_STATE_LABELS.get(state, f"Not external control (state {state})")
                )
            # A failed read leaves self._ess_mode_confirmed_external at its
            # last known value: unknown before the first read, not refused.
            # The sensor key is omitted rather than publishing a guess.

        return snapshot

    @property
    def control_dependency_keys(self) -> frozenset:
        return frozenset({
            "battery_soc", "battery_power", "ac_power_setpoint",
            "ac_consumption", "ess_control_mode",
        })

    # --- control (write) ----------------------------------------------------

    def _echo(self, battery_setpoint_w: int, ac_power_setpoint: int) -> dict:
        # Two different quantities, both worth keeping in the coordinator
        # cache: ac_power_setpoint is the raw grid-point register (what the
        # Multi was actually told), battery_setpoint_w is the battery-power
        # intent net_power_from_data reconstructs — they differ by whatever
        # AC consumption was at write time, and conflating them was the bug.
        return {"ac_power_setpoint": ac_power_setpoint, "battery_setpoint_w": battery_setpoint_w}

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

        if self._ess_mode_confirmed_external is False:
            # Writing would be pointless (the Multi's own loop is driving it,
            # not this register) and reporting ok=True here would tell the
            # control layer a command landed when it did not.
            return SetpointResult(
                ok=False, net_power_w=applied, confirmed=False,
                failure_reason="ess_mode_not_external",
            )
        if self._ac_consumption_w is None:
            # Without a consumption reading there is no safe way to translate
            # a battery-power intent into the grid-point register — writing
            # the raw figure anyway is exactly the bug this field exists to
            # prevent (see module docstring). Refuse rather than guess; the
            # 60s hardware watchdog is the backstop if this persists.
            return SetpointResult(
                ok=False, net_power_w=applied, confirmed=False,
                failure_reason="consumption_unavailable",
            )

        # Clamped through the same int16 round-trip the register itself
        # enforces, so the comparison below is against what the hardware can
        # actually echo rather than an unclamped intermediate value.
        ac_in_target = decode_i16([encode_i16(applied + round(self._ac_consumption_w))])

        ok = await self._client.async_write_registers(
            self._vebus_unit_id, _REG_VEBUS_AC_POWER_SETPOINT_L1, [encode_i16(ac_in_target)]
        )
        if not ok:
            return SetpointResult(
                ok=False, net_power_w=applied, confirmed=False, failure_reason="write_failed"
            )

        if not read_back:
            return SetpointResult(
                ok=True, net_power_w=applied, confirmed=False,
                applied=self._echo(applied, ac_in_target),
            )

        regs = await self._client.async_read_holding_block(
            self._vebus_unit_id, _REG_VEBUS_AC_POWER_SETPOINT_L1, 1
        )
        # A register echo only confirms the write landed, never that the Multi
        # achieved it — Victron's own guidance is to read the Multi's actual
        # power instead of trusting a command was achievable (a managed
        # battery's own charge/discharge limit can cap it regardless of what
        # was asked). Read it here so the existing non-responsive-delivery
        # tracking — shared across every brand, not reimplemented per-driver —
        # has something real to judge delivery against.
        power_regs = await self._client.async_read_holding_block(
            self._system_unit_id, _REG_SYSTEM_BATTERY_POWER, 1
        )
        battery_power_w = decode_i16(power_regs, 0) if power_regs is not None else None

        if regs is None:
            # The write itself succeeded; only the confirmation read failed.
            return SetpointResult(
                ok=True, net_power_w=applied, confirmed=False,
                battery_power_w=battery_power_w,
                applied=self._echo(applied, ac_in_target),
            )
        echoed = decode_i16(regs, 0)
        return SetpointResult(
            ok=True,
            net_power_w=applied,
            confirmed=echoed == ac_in_target,
            battery_power_w=battery_power_w,
            applied=self._echo(applied, echoed),
        )

    async def write_control(self, key: str, value: int) -> bool:
        register = _WRITE_CONTROLS.get(key)
        if register is None:
            return False
        return await self._client.async_write_registers(
            self._vebus_unit_id, register, [1 if value else 0]
        )

    def net_power_from_data(self, data: dict) -> Optional[int]:
        value = data.get("battery_setpoint_w")
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

        This does **not** restore the Multi's own ESS mode. Writing
        ``Settings/CGwacs/Hub4Mode`` back to 1 or 2 would do that, but no
        source consulted confirms a write to it actually succeeds, and the
        settings service is not forwarded over Modbus-TCP on every Venus OS
        configuration. Guessing wrong on a settings register risks leaving the
        system in a worse state than the alternative, which is simply stopping
        writes and letting the hardware's own 60 s watchdog revert the Multi
        to passthrough on its own. Passthrough is not the same as the GX
        device's normal ESS behaviour — BatteryLife stays disabled for as long
        as ESS Mode itself remains set to External control, regardless of
        whether this driver is writing.
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
