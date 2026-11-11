"""Victron ESS external-control driver (Multiplus/Quattro behind a GX device).

Controls the Multi through ESS "External control" (Hub-4 Mode 3). The
register this driver writes (37, AcPowerSetpoint) targets AC-*input* power,
not battery power, so every command is translated through the power the
Multi currently passes on its AC-out: ``AcPowerSetpoint = battery_power_wanted
+ ac_out_power``. See the register constants below for the full map, each
checked against ``attributes.csv`` from ``victronenergy/dbus_modbustcp``.

The Multi holds the last written AC-in figure, not the battery intent behind
it, so the translation has to follow the AC-out power as it moves: every
fresh AC-out reading (``read_telemetry``, ~2 s) re-derives the register value
for the held battery target and rewrites it when it changed. The same
rewrite doubles as the keepalive for the Multi's 60 s setpoint watchdog.

Two refusals are deliberate, not missing error handling: nothing is written
until the AC-out power has a fresh reading, nor until ``Hub4Mode`` is
positively confirmed External control — an unreadable or unread check
refuses exactly like a confirmed-wrong one, and a confirmation that stops
refreshing expires.

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
    decode_ascii,
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

# Every register below is verified against attributes.csv from
# victronenergy/dbus_modbustcp — the actual table the server is built from.

# --- com.victronenergy.vebus (the Multiplus/Quattro itself) -----------------
_REG_VEBUS_AC_OUT_L1_POWER = 23        # int16, 10 W per count (Ac/Out/L1/P),
                                       # signed: negative when something on
                                       # AC-out feeds back into the Multi
_AC_OUT_POWER_W_PER_COUNT = 10
_REG_VEBUS_AC_POWER_SETPOINT_L1 = 37   # int16, W, +import/-export (Hub4/L1/AcPowerSetpoint)
_REG_VEBUS_DISABLE_CHARGE = 38         # uint16, 0/1 (Hub4/DisableCharge)
_REG_VEBUS_DISABLE_FEEDIN = 39         # uint16, 0/1 (Hub4/DisableFeedIn)
# 37/38/39 are contiguous and read back as one block. They are marked
# write-only in attributes.csv; the server's read path (verified against its
# source) applies no such check, so reading them back works.
# L2/L3 setpoints (40/41) are not written: single-phase only.

# --- com.victronenergy.system / .settings, both unit id 100 by default ------
_REG_SYSTEM_SERIAL = 800              # string[6] (12 bytes) — the GX device's
                                       # own serial, not a battery-pack serial;
                                       # there is no separate battery service
                                       # on every installation (see below)
_REG_SYSTEM_BATTERY_VOLTAGE = 840     # uint16, V, scale 10 — first of 4
                                       # contiguous registers through 843, read
                                       # as one block below
_REG_SYSTEM_BATTERY_POWER = 842       # int16, W, +charge/-discharge
_REG_SYSTEM_BATTERY_SOC = 843         # uint16, 0-100%
_REG_SETTINGS_HUB4_MODE = 2902  # uint16; 3 = External control
_HUB4_MODE_EXTERNAL_CONTROL = 3
_HUB4_MODE_LABELS = {
    1: "ESS with phase compensation", 2: "ESS without phase compensation",
    3: "External control",
}

# Confirmed by Victron's own ESS Mode 2/3 documentation.
_RAMP_LIMIT_W_PER_S = 400.0       # the Multi's own fixed ramp rate
_HARDWARE_WATCHDOG_S = 60.0       # passthrough fallback after this long unwritten
_SETPOINT_REFRESH_INTERVAL_S = 15.0  # rewrite an unchanged value this often;
# well under the 60s deadline so one dropped write still leaves room to retry
_MIN_ACTUATOR_LATENCY_S = 2.0     # reasoned floor; not a measurement
# How long the command must have stayed at 0 W before the battery is locked
# idle (see _is_idle). The driver is not told whether the controller is
# using this battery; a battery in use passes through zero for seconds at a
# time, one that was handed off stays there.
_IDLE_LOCK_DELAY_S = 30.0

# How long a value may go without a fresh read before it stops being trusted
# (see VictronEssDriver._refusal). Each is ~5x its own polling cadence: wide
# enough that one missed poll doesn't trip the gate (a 2x margin on
# ess_control_mode did exactly that on real hardware, and the resulting
# refuse/passthrough/rewrite cycle looked like power oscillation), tight
# enough that the compensation never runs on an AC-out figure that is
# seconds out of date.
_AC_OUT_STALE_S = 10.0     # polled every ~2s
_ESS_MODE_STALE_S = 150.0  # polled every ~30s

SENSOR_DEFINITIONS: list[dict] = [
    {"key": "battery_soc", "name": "Battery SOC", "unit": "%", "device_class": "battery",
     "state_class": "measurement", "scale": 1, "precision": 1, "scan_interval": "high",
     "enabled_by_default": True},
    {"key": "battery_power", "name": "Battery Power", "unit": "W", "device_class": "power",
     "state_class": "measurement", "scale": 1, "precision": 0, "scan_interval": "high",
     "enabled_by_default": True},
    {"key": "battery_voltage", "name": "Battery Voltage", "unit": "V",
     "device_class": "voltage", "state_class": "measurement", "scale": 0.1,
     "precision": 1, "category": "diagnostic", "scan_interval": "high",
     "enabled_by_default": True},
    {"key": "ac_power_setpoint", "name": "ESS AC Power Setpoint", "unit": "W",
     "device_class": "power", "state_class": "measurement", "scale": 1, "precision": 0,
     "category": "diagnostic", "scan_interval": "low", "enabled_by_default": True},
    {"key": "ac_out_load", "name": "AC Output Power", "unit": "W",
     "device_class": "power", "state_class": "measurement", "scale": 1, "precision": 0,
     "category": "diagnostic", "scan_interval": "high", "enabled_by_default": True},
    {"key": "ess_control_mode", "name": "ESS Control Mode", "data_type": "char",
     "icon": "mdi:state-machine", "category": "diagnostic", "scan_interval": "low",
     "enabled_by_default": True},
]
NUMBER_DEFINITIONS: list[dict] = []
SELECT_DEFINITIONS: list[dict] = []
# The switches show what the user asked for, not the raw registers: the
# driver additionally sets both flags while idle (see _sync_disable_flags).
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
        # AC-out power, for the compensation formula, plus when it was last
        # refreshed (monotonic). None until first read.
        self._ac_out_load_w: Optional[int] = None
        self._ac_out_load_monotonic = 0.0
        # Hub4Mode == 3, confirmed, plus when it was last refreshed.
        # Fail-closed: None, False, and stale-True all refuse — see _refusal.
        self._ess_mode_confirmed_external: Optional[bool] = None
        self._ess_mode_monotonic = 0.0
        # The battery power the controller last asked for. None until the
        # first command and again after standby: nothing is written, and the
        # Multi stays in (or falls back to) passthrough.
        self._target_w: Optional[int] = None
        # What register 37 was last told, and when — to skip a rewrite that
        # would change nothing while still feeding the watchdog on time.
        self._written_ac_in_w: Optional[int] = None
        self._written_monotonic = 0.0
        # When the target last became 0 W (monotonic) — see _is_idle.
        self._zero_since = 0.0
        # What the user's two switches ask for. Not persisted: both start
        # off after a restart.
        self._user_disable = {"disable_charge": False, "disable_feedin": False}
        # Registers 38/39 as last written or read back; None when unknown.
        self._disable_flags: Optional[tuple[int, int]] = None
        # The GX device's own serial (register 800), read once on connect —
        # not polled telemetry. None until read; also keys the synthetic-
        # energy backup so a removed/re-added battery reclaims its totals.
        self._serial: Optional[str] = None

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
        )
        self._read_groups = [
            # ac_out_load is fast, not "low": every setpoint is derived from it.
            ReadGroup("high", ("battery_soc", "battery_power", "battery_voltage", "ac_out_load")),
            ReadGroup("low", ("ac_power_setpoint", "disable_charge", "disable_feedin", "ess_control_mode")),
        ]

    # --- identity -------------------------------------------------------

    @property
    def capabilities(self) -> DriverCapabilities:
        return self._capabilities

    @property
    def model_label(self) -> Optional[str]:
        return "Victron ESS (Multiplus/Quattro)"

    @property
    def serial(self) -> Optional[str]:
        return self._serial

    # --- connection lifecycle --------------------------------------------

    @property
    def connected(self) -> bool:
        return self._client.connected

    async def connect(self) -> bool:
        if not await self._client.async_connect():
            return False
        # Read once, not polled: a static identifier, and a failed read here
        # (older Venus OS, or the register just not forwarded) must not fail
        # the connection — self._serial simply stays None, same as any driver
        # with no serial to report.
        regs = await self._client.async_read_holding_block(
            self._system_unit_id, _REG_SYSTEM_SERIAL, 6
        )
        if regs is not None:
            self._serial = decode_ascii(regs)
        return True

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

        if requested is None or {"battery_soc", "battery_power", "battery_voltage"} & requested:
            # One block covers Voltage(840)/Current(841, unused)/Power(842)/
            # Soc(843) — contiguous on the system service, so this is a single
            # read instead of two.
            regs = await self._client.async_read_holding_block(
                self._system_unit_id, _REG_SYSTEM_BATTERY_VOLTAGE, 4
            )
            if regs is not None:
                snapshot["battery_voltage"] = decode_u16(regs, 0)
                snapshot["battery_power"] = decode_i16(regs, 2)
                soc = decode_u16(regs, 3)
                if 0 <= soc <= 100:
                    snapshot["battery_soc"] = soc
                else:
                    _LOGGER.warning(
                        "Victron driver: dropping battery_soc=%s — outside "
                        "0-100%%. Confirm the system unit id (default 100) "
                        "is correct for this GX device.",
                        soc,
                    )

        if requested is None or "ac_out_load" in requested:
            regs = await self._client.async_read_holding_block(
                self._vebus_unit_id, _REG_VEBUS_AC_OUT_L1_POWER, 1
            )
            if regs is not None:
                load = decode_i16(regs, 0) * _AC_OUT_POWER_W_PER_COUNT
                self._ac_out_load_w = load
                self._ac_out_load_monotonic = asyncio.get_running_loop().time()
                snapshot["ac_out_load"] = load
                await self._hold_target()
            # A failed read keeps the last known value without refreshing the
            # timestamp: one missed poll doesn't refuse writes, a value that
            # has gone stale does.

        if requested is None or {"ac_power_setpoint", "disable_charge", "disable_feedin"} & requested:
            regs = await self._client.async_read_holding_block(
                self._vebus_unit_id, _REG_VEBUS_AC_POWER_SETPOINT_L1, 3
            )
            if regs is not None:
                snapshot["ac_power_setpoint"] = decode_i16(regs, 0)
                # Remembered so a readback that disagrees with what is wanted
                # gets corrected on the next poll.
                self._disable_flags = (
                    int(bool(decode_u16(regs, 1))), int(bool(decode_u16(regs, 2)))
                )
                snapshot.update({k: int(v) for k, v in self._user_disable.items()})

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
            # Same failed-read rule as ac_out_load above.

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

    def _clamp(self, net_power_w: int) -> int:
        return max(
            -self._capabilities.max_discharge_power_w,
            min(self._capabilities.max_charge_power_w, int(net_power_w)),
        )

    def _refusal(self) -> Optional[str]:
        """Why register 37 must not be written right now, or None if it may.

        Fail-closed on both inputs: only a positively-confirmed *and fresh*
        value passes. A cached "yes" that stopped being renewed (its register
        went quiet after once answering) refuses exactly like one never read.
        """
        now = asyncio.get_running_loop().time()
        if (
            self._ess_mode_confirmed_external is not True
            or now - self._ess_mode_monotonic > _ESS_MODE_STALE_S
        ):
            return "ess_mode_not_confirmed_external"
        if (
            self._ac_out_load_w is None
            or now - self._ac_out_load_monotonic > _AC_OUT_STALE_S
        ):
            # No safe way to translate battery power into the AC-in register
            # without a fresh AC-out reading. Refuse rather than guess.
            return "consumption_unavailable"
        return None

    def _ac_in_target(self, battery_w: int) -> int:
        # Round-tripped through the register's own int16 clamp, so a readback
        # is compared against what the hardware can actually echo.
        return decode_i16([encode_i16(battery_w + self._ac_out_load_w)])

    async def _write_ac_in(self, ac_in_w: int) -> bool:
        ok = await self._client.async_write_registers(
            self._vebus_unit_id, _REG_VEBUS_AC_POWER_SETPOINT_L1, [encode_i16(ac_in_w)]
        )
        _LOGGER.debug(
            "Victron driver: AcPowerSetpoint=%sW (battery target %sW + AC-out %sW)%s",
            ac_in_w, self._target_w, self._ac_out_load_w, "" if ok else " — write failed",
        )
        if ok:
            self._written_ac_in_w = ac_in_w
            self._written_monotonic = asyncio.get_running_loop().time()
        return ok

    def _is_idle(self) -> bool:
        """Whether the battery should be locked idle: commanded 0 W, and for
        long enough that it is evidently not the battery in use.

        A non-zero command always unlocks at once. The delay keeps the lock
        off a battery that is merely passing through zero, where it would
        flap every few seconds.
        """
        if self._target_w is None or self._clamp(self._target_w) != 0:
            return False
        return asyncio.get_running_loop().time() - self._zero_since >= _IDLE_LOCK_DELAY_S

    async def _sync_disable_flags(self, idle: bool) -> bool:
        """Write DisableCharge/DisableFeedIn: each is set if the user's
        switch asks for it, and both are set while the target is 0 W.

        Matching AC-in to AC-out is not a true idle: the 10 W AC-out
        resolution and the Multi's own conversion losses still come out of
        the battery (30-50 W on real hardware). With both flags set the Multi
        cannot touch the battery at all, whatever AC-out does — the same
        flags the GX device's own ESS uses to idle it. A failed write leaves
        the state unknown, so the next poll tries again.
        """
        wanted = (
            int(idle or self._user_disable["disable_charge"]),
            int(idle or self._user_disable["disable_feedin"]),
        )
        if self._disable_flags == wanted:
            return True
        ok = await self._client.async_write_registers(
            self._vebus_unit_id, _REG_VEBUS_DISABLE_CHARGE, list(wanted)
        )
        _LOGGER.debug(
            "Victron driver: DisableCharge=%s DisableFeedIn=%s (idle lock %s, was %s)%s",
            wanted[0], wanted[1], "on" if idle else "off", self._disable_flags,
            "" if ok else " — write failed",
        )
        self._disable_flags = wanted if ok else None
        return ok

    async def _hold_target(self) -> None:
        """Keep the held battery target true against a fresh AC-out reading.

        Called from the telemetry poll right after AC-out power was read. The
        Multi only remembers the AC-in figure, so whenever AC-out moves the
        battery silently absorbs the difference until that figure is
        re-derived — and an unchanged figure still has to be rewritten ahead
        of the 60 s watchdog. Living here rather than in the controller means
        neither depends on the controller having a reason to call in.
        """
        if self._target_w is None or self._shutting_down or self._refusal() is not None:
            return
        target_w = self._clamp(self._target_w)
        await self._sync_disable_flags(self._is_idle())
        ac_in_w = self._ac_in_target(target_w)
        age_s = asyncio.get_running_loop().time() - self._written_monotonic
        if ac_in_w == self._written_ac_in_w and age_s < _SETPOINT_REFRESH_INTERVAL_S:
            return
        await self._write_ac_in(ac_in_w)

    async def apply_setpoint(
        self,
        net_power_w: int,
        *,
        mode_hint: Optional[str] = None,
        read_back: bool = True,
    ) -> SetpointResult:
        applied = self._clamp(net_power_w)
        # Held even when the write below is refused or fails: it is still
        # what the controller wants, and _hold_target applies it as soon as
        # the gate reopens.
        if applied == 0 and self._target_w != 0:
            self._zero_since = asyncio.get_running_loop().time()
        self._target_w = applied

        failure_reason = self._refusal()
        if failure_reason is not None:
            return SetpointResult(
                ok=False, net_power_w=applied, confirmed=False,
                failure_reason=failure_reason,
            )

        # Unlock before a non-zero setpoint, lock before the idle one: either
        # order is safe (the flags only ever restrict), this one is prompt.
        await self._sync_disable_flags(self._is_idle())
        ac_in_target = self._ac_in_target(applied)
        if not await self._write_ac_in(ac_in_target):
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
        if key not in self._user_disable:
            return False
        self._user_disable[key] = bool(value)
        return await self._sync_disable_flags(self._is_idle())

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
        """Stop writing, release the idle lock, and let the hardware
        watchdog revert to passthrough.

        Does not restore the Multi's own ESS mode — writing Hub4Mode back is
        unconfirmed, and guessing wrong on a settings register risks a worse
        state than just waiting out the watchdog. BatteryLife stays disabled
        until ESS Mode is switched back by hand, independent of this call.
        """
        self._target_w = None
        # Drop the idle lock (the user's own switches stay as set): it would
        # otherwise outlive this integration and keep the battery idle under
        # whatever drives it next.
        await self._sync_disable_flags(False)
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
