"""Victron ESS external-control driver (Multiplus/Quattro behind a GX device).

Controls the Multi through ESS "External control" (Hub-4 Mode 3). The
register this driver writes (37, AcPowerSetpoint) targets AC-*input* power,
not battery power, so every command is translated through the AC-out load
the Multi is currently serving: ``AcPowerSetpoint = battery_power_wanted +
ac_out_load``. See the register constants below for the full map, each
checked against ``attributes.csv`` from ``victronenergy/dbus_modbustcp``.

Two refusals in ``apply_setpoint`` are deliberate, not missing error
handling: it writes only once the AC-out load has been read at least once,
and only once ``Hub4Mode`` is positively confirmed External control — an
unreadable or unread check refuses exactly like a confirmed-wrong one. Both
checks also expire: a value that stops refreshing (the register starts
failing after having answered before) is treated the same as one that was
never read at all once it is older than ``_HARDWARE_WATCHDOG_S`` — a stale
"yes" must not outlive the same window the hardware itself uses to give up
on an unwritten setpoint.

Design history, the bugs this went through, and what remains unverified on
real hardware are in the project guide page's technical notes
(``site-docs/configuration/batteries/victron.md``), not repeated here.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import replace
from typing import Optional

from ..infra.victron_modbus_client import (
    VictronModbusClient,
    decode_i16,
    decode_i32,
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

# Every register below is verified against attributes.csv from
# victronenergy/dbus_modbustcp — the actual table the server is built from.

# --- com.victronenergy.vebus (the Multiplus/Quattro itself) -----------------
_REG_VEBUS_AC_POWER_SETPOINT_L1 = 37   # int16, W, +import/-export (Hub4/L1/AcPowerSetpoint)
_REG_VEBUS_AC_POWER_SETPOINT_L2 = 40   # int16, W (L2, three-phase — not written)
_REG_VEBUS_AC_POWER_SETPOINT_L3 = 41   # int16, W (L3, three-phase — not written)
_REG_VEBUS_DISABLE_CHARGE = 38         # uint16, 0/1 (Hub4/DisableCharge)
_REG_VEBUS_DISABLE_FEEDIN = 39         # uint16, 0/1 (Hub4/DisableFeedIn)
# 37/38/39 are marked write-only in attributes.csv; the server's read path
# (verified against its source) applies no such check, so reading them back
# for confirmation works.

# --- com.victronenergy.system, unit id 100 (Victron's documented default) --
_REG_SYSTEM_BATTERY_POWER = 842       # int16, W, +charge/-discharge
_REG_SYSTEM_BATTERY_SOC = 843         # uint16, 0-100%
_REG_SYSTEM_AC_CONSUMPTION_ON_OUTPUT_L1 = 878  # int32 (2 registers), W — AC-out
                                                # load specifically, not the
                                                # generic Ac/Consumption (817)
_REG_SETTINGS_HUB4_MODE = 2902  # uint16; 3 = External control
_HUB4_MODE_EXTERNAL_CONTROL = 3
_HUB4_MODE_LABELS = {
    1: "ESS with phase compensation", 2: "ESS without phase compensation",
    3: "External control",
}

# Confirmed by Victron's own ESS Mode 2/3 documentation.
_RAMP_LIMIT_W_PER_S = 400.0       # the Multi's own fixed ramp rate
_HARDWARE_WATCHDOG_S = 60.0       # passthrough fallback after this long unwritten
_SETPOINT_REFRESH_INTERVAL_S = 45.0  # rewrite margin ahead of the watchdog
_MIN_ACTUATOR_LATENCY_S = 2.0     # reasoned floor; not a measurement

# How long a confirmed value may go without a fresh read before apply_setpoint
# treats it as unconfirmed again (see VictronEssDriver._is_fresh). Each needs
# its own margin over its own polling cadence, not the Multi's watchdog
# number: ac_out_load polls every ~2s, so _HARDWARE_WATCHDOG_S (60s) is ~30x
# that and safely clear of ordinary jitter. ess_control_mode polls only every
# ~30s — reusing the same 60s bound gave it just 2x margin, so a single
# missed poll could push the gap past the threshold and trip the gate. That
# is exactly what produced a write/passthrough/rewrite oscillation on real
# hardware: the gate refuses, the Multi's own watchdog reverts to passthrough
# within the same window, the next successful poll reopens the gate, repeat.
_ESS_MODE_STALE_S = 150.0  # 5x its ~30s cadence

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
    {"key": "ac_out_load", "name": "AC Output Load", "unit": "W",
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
        # AC-out load, for the compensation formula, plus when it was last
        # refreshed (monotonic). None until first read; apply_setpoint
        # refuses to write while it is None or stale — see _is_fresh.
        self._ac_out_load_w: Optional[float] = None
        self._ac_out_load_monotonic = 0.0
        # Hub4Mode == 3, confirmed, plus when it was last refreshed.
        # Fail-closed: apply_setpoint writes only when this is True *and*
        # fresh; None, False, and stale-True all refuse.
        self._ess_mode_confirmed_external: Optional[bool] = None
        self._ess_mode_monotonic = 0.0

        max_charge_power_w = max(0, int(max_charge_power_w))
        max_discharge_power_w = max(0, int(max_discharge_power_w))
        # Worst case: a full reversal between the two ceilings at the Multi's
        # fixed ramp rate. Conservative; a real PD command is usually smaller.
        reversal_s = (max_charge_power_w + max_discharge_power_w) / _RAMP_LIMIT_W_PER_S
        actuator_latency_s = max(_MIN_ACTUATOR_LATENCY_S, reversal_s)

        self._capabilities = DriverCapabilities(
            # Software owns the SOC floor/ceiling; no confirmed register ties
            # it to BMS enforcement. DVCC/the BMS protect the battery regardless.
            hardware_soc_cutoff=False,
            has_force_mode=False,
            push_telemetry=False,
            max_charge_power_w=max_charge_power_w,
            max_discharge_power_w=max_discharge_power_w,
            has_mppt_pv=False,
            has_alarm_registers=False,
            # Hub4Mode is read for the safety gate, but writing it back is
            # unconfirmed — declaring this True would promise a toggle this
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
            # ac_out_load is fast, not "low": apply_setpoint needs it fresh.
            ReadGroup("high", ("battery_soc", "battery_power", "ac_out_load")),
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
                        "Victron driver: dropping battery_soc=%s — outside "
                        "0-100%%. Confirm the system unit id (default 100) "
                        "is correct for this GX device.",
                        soc,
                    )
            regs = await self._client.async_read_holding_block(
                self._system_unit_id, _REG_SYSTEM_BATTERY_POWER, 1
            )
            if regs is not None:
                snapshot["battery_power"] = decode_i16(regs, 0)

        if requested is None or "ac_out_load" in requested:
            regs = await self._client.async_read_holding_block(
                self._system_unit_id, _REG_SYSTEM_AC_CONSUMPTION_ON_OUTPUT_L1, 2
            )
            if regs is not None:
                load = decode_i32(regs, 0)
                self._ac_out_load_w = load
                self._ac_out_load_monotonic = asyncio.get_running_loop().time()
                snapshot["ac_out_load"] = load
            # A failed read keeps the last known value rather than clearing it
            # to None, so one missed poll doesn't immediately refuse writes —
            # but it also doesn't refresh the timestamp, so apply_setpoint's
            # freshness check still catches a value that has gone stale.

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
                self._system_unit_id, _REG_SETTINGS_HUB4_MODE, 1
            )
            if regs is not None:
                mode = decode_u16(regs, 0)
                self._ess_mode_confirmed_external = mode == _HUB4_MODE_EXTERNAL_CONTROL
                self._ess_mode_monotonic = asyncio.get_running_loop().time()
                snapshot["ess_control_mode"] = _HUB4_MODE_LABELS.get(
                    mode, f"Unknown ESS mode ({mode})"
                )
            # A failed read keeps the last known value without refreshing the
            # timestamp, so a confirmation that stops being renewed goes
            # stale and apply_setpoint's freshness check refuses on it.

        return snapshot

    @property
    def control_dependency_keys(self) -> frozenset:
        return frozenset({
            "battery_soc", "battery_power", "ac_power_setpoint",
            "ac_out_load", "ess_control_mode",
        })

    # --- control (write) ----------------------------------------------------

    def _echo(self, battery_setpoint_w: int, ac_power_setpoint: int) -> dict:
        # ac_power_setpoint is the raw register (what the Multi was told);
        # battery_setpoint_w is the battery intent net_power_from_data reads.
        return {"ac_power_setpoint": ac_power_setpoint, "battery_setpoint_w": battery_setpoint_w}

    @staticmethod
    def _is_fresh(last_success_monotonic: float, max_age_s: float) -> bool:
        """Whether a value refreshed at this monotonic time is still trusted.

        Caps how long a cached "last known good" value may be relied on once
        its register stops answering — otherwise a value that was confirmed
        once but never updates again (reads silently failing) would be
        trusted forever. ``max_age_s`` is the caller's own margin over its own
        polling cadence, not a single shared number — see
        ``_ESS_MODE_STALE_S`` for why that distinction matters.
        """
        return asyncio.get_running_loop().time() - last_success_monotonic <= max_age_s

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

        if self._ess_mode_confirmed_external is not True or not self._is_fresh(
            self._ess_mode_monotonic, _ESS_MODE_STALE_S
        ):
            # Fail-closed: only a positively-confirmed *and fresh* True
            # proceeds — a stale True (the register stopped answering after
            # once confirming) refuses exactly like an outright False.
            return SetpointResult(
                ok=False, net_power_w=applied, confirmed=False,
                failure_reason="ess_mode_not_confirmed_external",
            )
        if self._ac_out_load_w is None or not self._is_fresh(
            self._ac_out_load_monotonic, _HARDWARE_WATCHDOG_S
        ):
            # No safe way to translate battery power into the grid-point
            # register without a fresh reading. Refuse rather than guess.
            return SetpointResult(
                ok=False, net_power_w=applied, confirmed=False,
                failure_reason="consumption_unavailable",
            )

        # Round-tripped through the register's own int16 clamp, so the
        # comparison below is against what the hardware can actually echo.
        ac_in_target = decode_i16([encode_i16(applied + round(self._ac_out_load_w))])

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
        # The echo only confirms the write landed, not that the Multi
        # achieved it, so also read actual battery power for the shared
        # non-responsive-delivery tracker to judge delivery against.
        power_regs = await self._client.async_read_holding_block(
            self._system_unit_id, _REG_SYSTEM_BATTERY_POWER, 1
        )
        battery_power_w = decode_i16(power_regs, 0) if power_regs is not None else None

        if regs is None:
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
        """Stop writing and let the hardware watchdog revert to passthrough.

        Does not restore the Multi's own ESS mode — writing Hub4Mode back is
        unconfirmed, and guessing wrong on a settings register risks a worse
        state than just waiting out the watchdog. BatteryLife stays disabled
        until ESS Mode is switched back by hand, independent of this call.
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
