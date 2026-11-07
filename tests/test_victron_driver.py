"""Unit tests for the Victron ESS external-control driver.

A fake Modbus client, throughout: this proves the driver's arithmetic and
control-flow logic (clamping, the compensation formula, which refusal fires
when) — it cannot prove a real GX device answers a given register, or that
one register reports what its name implies versus a similarly-named
neighbour. See the "Technical notes" section of the guide page
(site-docs/configuration/batteries/victron.md) for what only a real device
has caught so far, and what is still unverified.
"""
from __future__ import annotations

import time
from unittest.mock import AsyncMock, Mock

import pytest

from custom_components.omnibattery.drivers import DriverCapabilities, VictronEssDriver
from custom_components.omnibattery.drivers.victron import (
    _ESS_MODE_STALE_S,
    _HARDWARE_WATCHDOG_S,
    _HUB4_MODE_EXTERNAL_CONTROL,
    _REG_SETTINGS_HUB4_MODE,
    _REG_SYSTEM_AC_CONSUMPTION_ON_OUTPUT_L1,
    _REG_SYSTEM_BATTERY_POWER,
    _REG_SYSTEM_BATTERY_SOC,
    _REG_SYSTEM_BATTERY_VOLTAGE,
    _REG_SYSTEM_SERIAL,
    _REG_VEBUS_AC_POWER_SETPOINT_L1,
    _REG_VEBUS_DISABLE_CHARGE,
)
from custom_components.omnibattery.infra.victron_modbus_client import (
    decode_ascii,
    decode_i16,
    decode_i32,
    decode_u16,
    encode_i16,
)


def _fake_client():
    client = AsyncMock()
    client.connected = True
    client.async_connect = AsyncMock(return_value=True)
    client.async_close = AsyncMock()
    client.set_shutting_down = Mock()
    client.async_read_holding_block = AsyncMock(return_value=None)
    client.async_write_registers = AsyncMock(return_value=True)
    return client


def _driver(
    client=None, consumption=0, ess_mode_external=True,
    consumption_age_s=0.0, ess_mode_age_s=0.0, **kw,
):
    """Build a driver with sane defaults for the control-path tests.

    ``consumption=0`` makes the AC-consumption compensation a no-op by
    default, so tests that don't care about it see the same numbers the
    pre-compensation code produced. Tests that do care pass a real value.
    ``ess_mode_external=True`` is NOT the driver's own starting state (that's
    None, before any telemetry poll) — it's set here so tests exercising the
    write path don't also have to care about the fail-closed ESS-mode gate;
    the gate itself has its own dedicated tests below, covering None, False,
    and staleness explicitly. ``*_age_s`` back-dates the matching monotonic
    timestamp by that many seconds, to simulate a value that stopped
    refreshing; 0.0 (the default) means "just read, fully fresh". asyncio's
    default event loop implements ``loop.time()`` as ``time.monotonic()``, so
    stamping with the latter here is directly comparable to what the driver
    itself stamps.
    """
    kw.setdefault("vebus_unit_id", 227)
    kw.setdefault("system_unit_id", 100)
    kw.setdefault("max_charge_power_w", 5000)
    kw.setdefault("max_discharge_power_w", 5000)
    driver = VictronEssDriver("1.2.3.4", 502, client=client or _fake_client(), **kw)
    driver._ac_out_load_w = consumption
    driver._ac_out_load_monotonic = time.monotonic() - consumption_age_s
    driver._ess_mode_confirmed_external = ess_mode_external
    driver._ess_mode_monotonic = time.monotonic() - ess_mode_age_s
    return driver


# ----------------------------------------------------------------------
# encode/decode helpers
# ----------------------------------------------------------------------

def test_encode_i16_round_trips_negative_values():
    assert decode_i16([encode_i16(-200)]) == -200
    assert decode_i16([encode_i16(200)]) == 200
    assert decode_i16([encode_i16(0)]) == 0


def test_encode_i16_clamps_to_register_range():
    assert encode_i16(100000) == 0x7FFF
    assert encode_i16(-100000) == 0x8000
    assert decode_i16([encode_i16(100000)]) == 32767
    assert decode_i16([encode_i16(-100000)]) == -32768


def test_decode_u16_is_unsigned():
    assert decode_u16([650]) == 650


# ----------------------------------------------------------------------
# capabilities
# ----------------------------------------------------------------------

def test_capabilities_static_fields():
    caps = _driver().capabilities
    assert isinstance(caps, DriverCapabilities)
    assert caps.hardware_soc_cutoff is False
    assert caps.has_force_mode is False
    assert caps.push_telemetry is False
    assert caps.has_rs485_control is False
    assert caps.has_mppt_pv is False
    assert caps.has_nominal_capacity is False
    assert caps.has_energy_counters is False
    assert caps.setpoint_confirm_reliable is True
    assert caps.max_charge_power_w == 5000
    assert caps.max_discharge_power_w == 5000


def test_capabilities_declares_the_ess_watchdog_refresh():
    """The 60s hardware watchdog is Victron-specific; every other driver's
    capability stays None and must be unaffected by this one existing."""
    caps = _driver().capabilities
    assert caps.setpoint_refresh_interval_s is not None
    assert caps.setpoint_refresh_interval_s < 60.0


def test_actuator_latency_scales_with_configured_power():
    small = _driver(max_charge_power_w=1000, max_discharge_power_w=1000).capabilities
    large = _driver(max_charge_power_w=8000, max_discharge_power_w=8000).capabilities
    assert large.actuator_latency_s > small.actuator_latency_s
    assert small.readback_latency_s == small.actuator_latency_s


def test_model_label():
    assert _driver().model_label == "Victron ESS (Multiplus/Quattro)"


# ----------------------------------------------------------------------
# telemetry
# ----------------------------------------------------------------------

@pytest.mark.asyncio
async def test_read_telemetry_decodes_voltage_soc_and_power():
    """Voltage(840)/Current(841, unused)/Power(842)/Soc(843) are one
    contiguous block, read once starting at _REG_SYSTEM_BATTERY_VOLTAGE."""
    client = _fake_client()

    async def fake_read(unit_id, start, count):
        if unit_id == 100 and start == _REG_SYSTEM_BATTERY_VOLTAGE and count == 4:
            return [530, 0, encode_i16(-320) & 0xFFFF, 65]
        return None

    client.async_read_holding_block = AsyncMock(side_effect=fake_read)
    driver = _driver(client=client)

    data = await driver.read_telemetry(["battery_soc", "battery_power", "battery_voltage"])
    assert data["battery_voltage"] == 530  # scaled 0.1 -> 53.0V at the sensor layer
    assert data["battery_power"] == -320
    assert data["battery_soc"] == 65


@pytest.mark.asyncio
async def test_read_telemetry_rejects_out_of_range_soc():
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(
        side_effect=lambda unit_id, start, count: (
            [0, 0, 0, 0xFFFF] if start == _REG_SYSTEM_BATTERY_VOLTAGE else None
        )
    )
    driver = _driver(client=client)

    data = await driver.read_telemetry(["battery_soc"])
    assert "battery_soc" not in data


@pytest.mark.asyncio
async def test_read_telemetry_omits_keys_on_failed_block():
    """A None block (timeout/connection loss) must omit the key, never fabricate 0."""
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(return_value=None)
    driver = _driver(client=client)

    data = await driver.read_telemetry(None)
    assert "battery_soc" not in data
    assert "battery_power" not in data
    assert "battery_voltage" not in data
    assert "ac_power_setpoint" not in data
    assert "ac_out_load" not in data
    assert "ess_control_mode" not in data


@pytest.mark.asyncio
async def test_read_telemetry_disable_flags():
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(
        side_effect=lambda unit_id, start, count: [1, 0] if start == _REG_VEBUS_DISABLE_CHARGE else None
    )
    driver = _driver(client=client)

    data = await driver.read_telemetry(["disable_charge", "disable_feedin"])
    assert data["disable_charge"] == 1
    assert data["disable_feedin"] == 0


@pytest.mark.asyncio
async def test_read_telemetry_tracks_ac_out_load_for_apply_setpoint():
    """Register 878 is int32 — two registers, not the uint16/one-register
    shape of the wrong register (817) an earlier version used."""
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(
        side_effect=lambda unit_id, start, count: (
            [0, 1800] if start == _REG_SYSTEM_AC_CONSUMPTION_ON_OUTPUT_L1 else None
        )
    )
    driver = _driver(client=client, consumption=None)

    data = await driver.read_telemetry(["ac_out_load"])
    assert data["ac_out_load"] == 1800
    assert driver._ac_out_load_w == 1800


@pytest.mark.parametrize(
    "mode, expected_label, expected_confirmed",
    [
        (_HUB4_MODE_EXTERNAL_CONTROL, "External control", True),
        (1, "ESS with phase compensation", False),  # real hardware read this, not a made-up value
        (9, "Unknown ESS mode (9)", False),  # unmapped value: labelled, not hidden
    ],
)
@pytest.mark.asyncio
async def test_read_telemetry_reports_ess_control_mode(mode, expected_label, expected_confirmed):
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(
        side_effect=lambda unit_id, start, count: [mode] if start == _REG_SETTINGS_HUB4_MODE else None
    )
    driver = _driver(client=client)

    data = await driver.read_telemetry(["ess_control_mode"])
    assert data["ess_control_mode"] == expected_label
    assert driver._ess_mode_confirmed_external is expected_confirmed


# ----------------------------------------------------------------------
# control: the AC-consumption compensation (the correctness bug fix)
# ----------------------------------------------------------------------

@pytest.mark.asyncio
async def test_apply_setpoint_writes_and_confirms_with_zero_consumption():
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(return_value=[encode_i16(1200)])
    driver = _driver(client=client, consumption=0)

    result = await driver.apply_setpoint(1200)

    client.async_write_registers.assert_awaited_once_with(
        227, _REG_VEBUS_AC_POWER_SETPOINT_L1, [encode_i16(1200)]
    )
    assert result.ok is True
    assert result.net_power_w == 1200
    assert result.confirmed is True
    assert result.applied == {"ac_power_setpoint": 1200, "battery_setpoint_w": 1200}


@pytest.mark.parametrize(
    "net_power_w, consumption, expected_register_value",
    [
        (500, 2000, 2500),    # charge 500W against a 2000W load -> write 2500, not 500
        (-500, 2000, 1500),   # discharge 500W against the same load -> write 1500
        (0, 2000, 2000),      # 0W intent -> import exactly matches the load; battery untouched
    ],
)
@pytest.mark.asyncio
async def test_apply_setpoint_compensates_for_ac_out_load(net_power_w, consumption, expected_register_value):
    """The bug the review caught: writing the battery figure straight into
    the grid-point register asks for the opposite of what was intended
    whenever there's a load on AC-out."""
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(return_value=[encode_i16(expected_register_value)])
    driver = _driver(client=client, consumption=consumption)

    result = await driver.apply_setpoint(net_power_w)

    client.async_write_registers.assert_awaited_once_with(
        227, _REG_VEBUS_AC_POWER_SETPOINT_L1, [encode_i16(expected_register_value)]
    )
    assert result.net_power_w == net_power_w  # the battery intent, unchanged
    assert result.applied["ac_power_setpoint"] == expected_register_value
    assert result.applied["battery_setpoint_w"] == net_power_w


@pytest.mark.asyncio
async def test_apply_setpoint_refuses_when_consumption_unknown():
    client = _fake_client()
    driver = _driver(client=client, consumption=None)

    result = await driver.apply_setpoint(500)

    client.async_write_registers.assert_not_awaited()
    assert result.ok is False
    assert result.failure_reason == "consumption_unavailable"


@pytest.mark.parametrize("ess_mode_external", [False, None])
@pytest.mark.asyncio
async def test_apply_setpoint_refuses_unless_positively_confirmed_external(ess_mode_external):
    """Fail-closed: False (confirmed wrong) and None (unread, or a
    persistently failed read) both refuse — the fix for a real near-miss. An
    earlier version treated None as "nothing to act on, proceed," and the
    register it then relied on failed to read on real hardware while the
    real ESS mode (read via a different, working register) was plain ESS;
    the permissive default would have written anyway."""
    client = _fake_client()
    driver = _driver(client=client, consumption=0, ess_mode_external=ess_mode_external)

    result = await driver.apply_setpoint(500)

    client.async_write_registers.assert_not_awaited()
    assert result.ok is False
    assert result.failure_reason == "ess_mode_not_confirmed_external"


@pytest.mark.asyncio
async def test_apply_setpoint_proceeds_only_when_positively_confirmed_external():
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(return_value=[encode_i16(500)])
    driver = _driver(client=client, consumption=0, ess_mode_external=True)

    result = await driver.apply_setpoint(500)

    client.async_write_registers.assert_awaited_once()
    assert result.ok is True


@pytest.mark.asyncio
async def test_apply_setpoint_refuses_when_ess_mode_confirmation_is_stale():
    """A True that stopped being refreshed must not be trusted forever: once
    the register has gone unanswered for longer than its own staleness
    margin, a cached 'yes' is as good as no confirmation at all. This uses
    its own constant (not _HARDWARE_WATCHDOG_S) because ess_control_mode
    polls on a slower ~30s cadence than ac_out_load's ~2s — reusing the same
    60s bound here gave only 2x margin and tripped on a single missed poll,
    producing a write/passthrough/rewrite oscillation on real hardware."""
    client = _fake_client()
    driver = _driver(
        client=client, consumption=0, ess_mode_external=True,
        ess_mode_age_s=_ESS_MODE_STALE_S + 1,
    )

    result = await driver.apply_setpoint(500)

    client.async_write_registers.assert_not_awaited()
    assert result.ok is False
    assert result.failure_reason == "ess_mode_not_confirmed_external"


@pytest.mark.asyncio
async def test_apply_setpoint_refuses_when_ac_out_load_is_stale():
    client = _fake_client()
    driver = _driver(
        client=client, consumption=2000,
        consumption_age_s=_HARDWARE_WATCHDOG_S + 1,
    )

    result = await driver.apply_setpoint(500)

    client.async_write_registers.assert_not_awaited()
    assert result.ok is False
    assert result.failure_reason == "consumption_unavailable"


@pytest.mark.asyncio
async def test_read_telemetry_refreshes_the_freshness_timestamps():
    """The mechanism the staleness tests above rely on: a successful read
    must actually stamp 'now', not just cache the value."""
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(
        side_effect=lambda unit_id, start, count: (
            [0, 1800] if start == _REG_SYSTEM_AC_CONSUMPTION_ON_OUTPUT_L1
            else [_HUB4_MODE_EXTERNAL_CONTROL] if start == _REG_SETTINGS_HUB4_MODE
            else None
        )
    )
    driver = _driver(client=client, consumption=None, ess_mode_external=None)

    before = time.monotonic()
    await driver.read_telemetry(["ac_out_load", "ess_control_mode"])

    assert driver._ac_out_load_monotonic >= before
    assert driver._ess_mode_monotonic >= before


@pytest.mark.asyncio
async def test_apply_setpoint_reports_actual_battery_power_separately_from_the_echo():
    """'Confirmed' only means the setpoint register echoed back — it does not
    mean the Multi achieved that power. battery_power_w must come from a real
    reading of the system aggregate, not be assumed equal to the command."""
    client = _fake_client()

    async def fake_read(unit_id, start, count):
        if unit_id == 227 and start == _REG_VEBUS_AC_POWER_SETPOINT_L1:
            return [encode_i16(1200)]  # the setpoint echo
        if unit_id == 100 and start == _REG_SYSTEM_BATTERY_POWER:
            return [encode_i16(300)]  # what the Multi actually achieved so far
        return None

    client.async_read_holding_block = AsyncMock(side_effect=fake_read)
    driver = _driver(client=client, consumption=0)

    result = await driver.apply_setpoint(1200)

    assert result.confirmed is True        # the register took the write
    assert result.battery_power_w == 300   # but delivery is still ramping


@pytest.mark.asyncio
async def test_apply_setpoint_clamps_to_envelope():
    driver = _driver(max_charge_power_w=2000, max_discharge_power_w=1500, consumption=0)
    driver._client.async_read_holding_block = AsyncMock(return_value=[encode_i16(2000)])

    over_charge = await driver.apply_setpoint(9000)
    assert over_charge.net_power_w == 2000

    over_discharge = await driver.apply_setpoint(-9000)
    assert over_discharge.net_power_w == -1500


@pytest.mark.asyncio
async def test_apply_setpoint_write_failure_is_reported():
    client = _fake_client()
    client.async_write_registers = AsyncMock(return_value=False)
    driver = _driver(client=client, consumption=0)

    result = await driver.apply_setpoint(500)
    assert result.ok is False
    assert result.failure_reason == "write_failed"


@pytest.mark.asyncio
async def test_apply_setpoint_skips_readback_when_not_requested():
    client = _fake_client()
    driver = _driver(client=client, consumption=0)

    result = await driver.apply_setpoint(500, read_back=False)
    client.async_read_holding_block.assert_not_awaited()
    assert result.ok is True
    assert result.confirmed is False
    assert result.applied == {"ac_power_setpoint": 500, "battery_setpoint_w": 500}


@pytest.mark.asyncio
async def test_apply_setpoint_unconfirmed_when_readback_fails():
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(return_value=None)
    driver = _driver(client=client, consumption=0)

    result = await driver.apply_setpoint(500)
    assert result.ok is True
    assert result.confirmed is False
    assert result.battery_power_w is None


def test_net_power_from_data():
    driver = _driver()
    assert driver.net_power_from_data({}) is None
    assert driver.net_power_from_data({"battery_setpoint_w": -400}) == -400
    # The raw register alone (without the companion key) must not be
    # mistaken for the battery intent — that was the bug.
    assert driver.net_power_from_data({"ac_power_setpoint": -400}) is None


@pytest.mark.asyncio
async def test_write_control_disable_charge_and_feedin():
    client = _fake_client()
    driver = _driver(client=client)

    assert await driver.write_control("disable_charge", 1) is True
    client.async_write_registers.assert_awaited_with(227, _REG_VEBUS_DISABLE_CHARGE, [1])

    assert await driver.write_control("disable_feedin", 0) is True
    assert await driver.write_control("not_a_real_key", 1) is False


@pytest.mark.asyncio
async def test_standby_does_not_write_and_returns_true():
    client = _fake_client()
    driver = _driver(client=client)

    assert await driver.standby() is True
    client.async_write_registers.assert_not_awaited()


@pytest.mark.asyncio
async def test_unsupported_hardware_controls_report_false():
    driver = _driver()
    assert await driver.set_charge_cutoff(50) is False
    assert await driver.set_rs485_control(True) is False


@pytest.mark.asyncio
async def test_apply_config_updates_software_power_ceiling():
    driver = _driver(max_charge_power_w=3000, max_discharge_power_w=3000)

    await driver.apply_config(max_charge_power_w=4500, max_discharge_power_w=1000)

    assert driver.capabilities.max_charge_power_w == 4500
    assert driver.capabilities.max_discharge_power_w == 1000


# ----------------------------------------------------------------------
# connection lifecycle
# ----------------------------------------------------------------------

@pytest.mark.asyncio
async def test_connect_close_delegate_to_client():
    client = _fake_client()
    driver = _driver(client=client)

    assert await driver.connect() is True
    client.async_connect.assert_awaited_once()

    await driver.close()
    client.async_close.assert_awaited_once()

    driver.set_shutting_down(True)
    client.set_shutting_down.assert_called_once_with(True)
    assert driver.connected is True


@pytest.mark.asyncio
async def test_connect_reads_the_serial_once():
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(
        side_effect=lambda unit_id, start, count: (
            [0x4858, 0x3132, 0x3334, 0, 0, 0] if start == _REG_SYSTEM_SERIAL else None
        )
    )
    driver = _driver(client=client)

    assert await driver.connect() is True
    assert driver.serial == "HX1234"


@pytest.mark.asyncio
async def test_connect_tolerates_an_unreadable_serial():
    """An older Venus OS, or the register just not forwarded, must not fail
    the connection — serial simply stays None, like any driver with none."""
    client = _fake_client()
    driver = _driver(client=client)

    assert await driver.connect() is True
    assert driver.serial is None


@pytest.mark.asyncio
async def test_probe_requires_both_unit_ids_to_answer(monkeypatch):
    from custom_components.omnibattery.drivers import victron as victron_mod

    fake = _fake_client()
    fake.async_read_holding_block = AsyncMock(return_value=[1])
    monkeypatch.setattr(victron_mod, "VictronModbusClient", lambda *a, **k: fake)

    assert await VictronEssDriver.probe("1.2.3.4", 502, 227, 100) is True

    fake_fail = _fake_client()
    fake_fail.async_read_holding_block = AsyncMock(return_value=None)
    monkeypatch.setattr(victron_mod, "VictronModbusClient", lambda *a, **k: fake_fail)
    assert await VictronEssDriver.probe("1.2.3.4", 502, 227, 100) is False


# ----------------------------------------------------------------------
# control_dependency_keys / definitions
# ----------------------------------------------------------------------

def test_control_dependency_keys():
    keys = _driver().control_dependency_keys
    assert {
        "battery_soc", "battery_power", "ac_power_setpoint",
        "ac_out_load", "ess_control_mode",
    } <= keys


def test_entity_definitions_expose_expected_keys():
    driver = _driver()
    sensor_keys = {d["key"] for d in driver.sensor_definitions}
    switch_keys = {d["key"] for d in driver.switch_definitions}
    assert {
        "battery_soc", "battery_power", "battery_voltage", "ac_power_setpoint",
        "ac_out_load", "ess_control_mode",
    } <= sensor_keys
    assert {"disable_charge", "disable_feedin"} == switch_keys
    assert driver.number_definitions == []
    assert driver.all_definitions == (
        driver.sensor_definitions + driver.switch_definitions
    )


def test_switch_definitions_declare_command_values():
    for definition in _driver().switch_definitions:
        assert definition["command_on"] == 1
        assert definition["command_off"] == 0


# ----------------------------------------------------------------------
# device registry
# ----------------------------------------------------------------------

def test_device_info_reports_victron_as_the_manufacturer():
    """The brand chain used to fall through to Marstek for every new brand
    that forgot to add itself (same bug fixed for Huawei) — a Victron
    battery read device card "Marstek / Victron ESS (Multiplus/Quattro)"."""
    from types import SimpleNamespace

    from custom_components.omnibattery.infra.coordinator import (
        MarstekVenusDataUpdateCoordinator,
    )

    coordinator = SimpleNamespace(
        device_key="192.0.2.1_502_227",
        name="Victron ESS",
        brand="victron",
        driver=SimpleNamespace(
            model_label="Victron ESS (Multiplus/Quattro)", serial="HQ1234ABCDE",
        ),
        host="192.0.2.1",
        port=502,
        data={},
    )
    info = MarstekVenusDataUpdateCoordinator.battery_device_info.fget(coordinator)
    assert info["manufacturer"] == "Victron"
    assert info["model"] == "Victron ESS (Multiplus/Quattro)"
    assert info["serial_number"] == "HQ1234ABCDE"
