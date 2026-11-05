"""Unit tests for the Victron ESS external-control driver.

Everything here exercises translation logic against a fake Modbus client —
it proves the driver's decode/encode math and control-loop contract, not the
vendor's register map. See docs/research/victron/driver-assessment-victron.md
for what still needs confirming against real hardware.
"""
from __future__ import annotations

from unittest.mock import AsyncMock, Mock

import pytest

from custom_components.omnibattery.drivers import DriverCapabilities, VictronEssDriver
from custom_components.omnibattery.drivers.victron import (
    _REG_SYSTEM_BATTERY_POWER,
    _REG_SYSTEM_BATTERY_SOC,
    _REG_VEBUS_AC_POWER_SETPOINT_L1,
    _REG_VEBUS_DISABLE_CHARGE,
)
from custom_components.omnibattery.infra.victron_modbus_client import (
    decode_i16,
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


def _driver(client=None, **kw):
    kw.setdefault("vebus_unit_id", 227)
    kw.setdefault("system_unit_id", 100)
    kw.setdefault("max_charge_power_w", 5000)
    kw.setdefault("max_discharge_power_w", 5000)
    return VictronEssDriver("1.2.3.4", 502, client=client or _fake_client(), **kw)


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
async def test_read_telemetry_decodes_soc_and_power():
    client = _fake_client()

    async def fake_read(unit_id, start, count):
        if unit_id == 100 and start == _REG_SYSTEM_BATTERY_SOC:
            return [65]  # 65%, no scaling on this register
        if unit_id == 100 and start == _REG_SYSTEM_BATTERY_POWER:
            return [encode_i16(-320)]  # discharging 320W
        return None

    client.async_read_holding_block = AsyncMock(side_effect=fake_read)
    driver = _driver(client=client)

    data = await driver.read_telemetry(["battery_soc", "battery_power"])
    assert data["battery_soc"] == 65
    assert data["battery_power"] == -320


@pytest.mark.asyncio
async def test_read_telemetry_rejects_out_of_range_soc():
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(
        side_effect=lambda unit_id, start, count: [0xFFFF] if start == _REG_SYSTEM_BATTERY_SOC else None
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
    assert "ac_power_setpoint" not in data


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


# ----------------------------------------------------------------------
# control
# ----------------------------------------------------------------------

@pytest.mark.asyncio
async def test_apply_setpoint_writes_and_confirms():
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(return_value=[encode_i16(1200)])
    driver = _driver(client=client)

    result = await driver.apply_setpoint(1200)

    client.async_write_registers.assert_awaited_once_with(
        227, _REG_VEBUS_AC_POWER_SETPOINT_L1, [encode_i16(1200)]
    )
    assert result.ok is True
    assert result.net_power_w == 1200
    assert result.confirmed is True
    assert result.applied == {"ac_power_setpoint": 1200}


@pytest.mark.asyncio
async def test_apply_setpoint_clamps_to_envelope():
    driver = _driver(max_charge_power_w=2000, max_discharge_power_w=1500)
    driver._client.async_read_holding_block = AsyncMock(return_value=[encode_i16(2000)])

    over_charge = await driver.apply_setpoint(9000)
    assert over_charge.net_power_w == 2000

    over_discharge = await driver.apply_setpoint(-9000)
    assert over_discharge.net_power_w == -1500


@pytest.mark.asyncio
async def test_apply_setpoint_write_failure_is_reported():
    client = _fake_client()
    client.async_write_registers = AsyncMock(return_value=False)
    driver = _driver(client=client)

    result = await driver.apply_setpoint(500)
    assert result.ok is False
    assert result.failure_reason == "write_failed"


@pytest.mark.asyncio
async def test_apply_setpoint_skips_readback_when_not_requested():
    client = _fake_client()
    driver = _driver(client=client)

    result = await driver.apply_setpoint(500, read_back=False)
    client.async_read_holding_block.assert_not_awaited()
    assert result.ok is True
    assert result.confirmed is False
    assert result.applied == {"ac_power_setpoint": 500}


@pytest.mark.asyncio
async def test_apply_setpoint_unconfirmed_when_readback_fails():
    client = _fake_client()
    client.async_read_holding_block = AsyncMock(return_value=None)
    driver = _driver(client=client)

    result = await driver.apply_setpoint(500)
    assert result.ok is True
    assert result.confirmed is False


def test_net_power_from_data():
    driver = _driver()
    assert driver.net_power_from_data({}) is None
    assert driver.net_power_from_data({"ac_power_setpoint": -400}) == -400


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
    assert {"battery_soc", "battery_power", "ac_power_setpoint"} <= keys


def test_entity_definitions_expose_expected_keys():
    driver = _driver()
    sensor_keys = {d["key"] for d in driver.sensor_definitions}
    switch_keys = {d["key"] for d in driver.switch_definitions}
    assert {"battery_soc", "battery_power", "ac_power_setpoint"} <= sensor_keys
    assert {"disable_charge", "disable_feedin"} == switch_keys
    assert driver.number_definitions == []
    assert driver.all_definitions == (
        driver.sensor_definitions + driver.switch_definitions
    )


def test_switch_definitions_declare_command_values():
    for definition in _driver().switch_definitions:
        assert definition["command_on"] == 1
        assert definition["command_off"] == 0
