"""Config flow: reconfiguring a Victron battery.

Without its own reconfigure step a Victron battery fell through to the
Marstek form, whose Marstek probe can never pass against a GX device.
"""
from __future__ import annotations

import glob
import json
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

import pytest

from custom_components.omnibattery.config_flow import (
    MarstekVenusConfigFlow,
    OptionsFlowHandler,
)
from custom_components.omnibattery.drivers.victron import VictronEssDriver

_STORED = {
    "brand": "victron",
    "name": "Victron ESS",
    "host": "10.0.1.31",
    "port": 502,
    "victron_vebus_unit_id": 227,
    "victron_system_unit_id": 100,
    "max_charge_power": 3000,
    "max_discharge_power": 2500,
    "min_soc": 20,
    "battery_capacity_kwh": 4.7,
}


def _flow(stored=None):
    entry = SimpleNamespace(entry_id="test-entry", data={"batteries": [dict(stored or _STORED)]})
    flow = MarstekVenusConfigFlow()
    flow.battery_index = 0
    flow._reconfigure_batteries = []
    flow._get_reconfigure_entry = lambda: entry
    flow._migrate_battery_registry_ids = MagicMock()
    flow.async_update_reload_and_abort = MagicMock(return_value={"type": "abort"})
    return flow, entry


def _submit(**changes):
    data = {
        "name": "Victron ESS",
        "host": "10.0.1.31",
        "port": 502,
        "victron_vebus_unit_id": 227,
        "victron_system_unit_id": 100,
    }
    data.update(changes)
    return data


@pytest.mark.asyncio
async def test_reconfigure_routes_victron_to_its_own_form():
    flow, _ = _flow()

    form = await flow.async_step_reconfigure_battery()

    assert form["step_id"] == "reconfigure_battery_victron"
    fields = {marker.schema: marker for marker in form["data_schema"].schema}
    # Connection only: the power ceilings stay in the options flow.
    assert set(fields) == {
        "name", "host", "port", "victron_vebus_unit_id", "victron_system_unit_id",
    }
    assert fields["host"].default() == "10.0.1.31"
    assert fields["victron_vebus_unit_id"].default() == 227


@pytest.mark.asyncio
async def test_reconfigure_moves_the_gx_and_keeps_every_other_setting(monkeypatch):
    diagnose = AsyncMock(return_value=None)
    monkeypatch.setattr(VictronEssDriver, "diagnose", diagnose)
    flow, entry = _flow()

    result = await flow.async_step_reconfigure_battery_victron(
        _submit(host="10.0.1.40", victron_vebus_unit_id=246)
    )

    assert result == {"type": "abort"}
    diagnose.assert_awaited_once_with("10.0.1.40", 502, 246, 100)
    flow._migrate_battery_registry_ids.assert_called_once_with(
        entry, "10.0.1.31", 502, "10.0.1.40", 502
    )
    assert flow._reconfigure_batteries == [
        _STORED | {"host": "10.0.1.40", "victron_vebus_unit_id": 246}
    ]


@pytest.mark.asyncio
async def test_reconfigure_unit_id_change_alone_keeps_the_registry_ids(monkeypatch):
    """The unit ids select services on the same GX device; they are not
    part of the battery's identity, so its entities stay where they are."""
    monkeypatch.setattr(VictronEssDriver, "diagnose", AsyncMock(return_value=None))
    flow, _ = _flow()

    await flow.async_step_reconfigure_battery_victron(_submit(victron_system_unit_id=0))

    flow._migrate_battery_registry_ids.assert_not_called()
    assert flow._reconfigure_batteries[0]["victron_system_unit_id"] == 0


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "error",
    ["cannot_connect", "victron_system_unit_id", "victron_vebus_unit_id", "victron_no_ess"],
)
async def test_reconfigure_failed_probe_keeps_the_form_and_changes_nothing(monkeypatch, error):
    """The form names what did not answer, not just "cannot connect"."""
    monkeypatch.setattr(VictronEssDriver, "diagnose", AsyncMock(return_value=error))
    flow, _ = _flow()

    form = await flow.async_step_reconfigure_battery_victron(_submit(host="10.0.1.99"))

    assert form["step_id"] == "reconfigure_battery_victron"
    assert form["errors"] == {"base": error}
    assert flow._reconfigure_batteries == []
    flow._migrate_battery_registry_ids.assert_not_called()
    flow.async_update_reload_and_abort.assert_not_called()


# ----------------------------------------------------------------------
# Adding a Victron battery (config flow) and editing it (options flow)
# ----------------------------------------------------------------------

_CONNECTION = {
    "name": "Victron ESS",
    "host": "10.0.1.31",
    "port": 502,
    "victron_vebus_unit_id": 227,
    "victron_system_unit_id": 100,
    "max_charge_power": 3000,
    "max_discharge_power": 2500,
}


def _markers(form):
    return {marker.schema: marker for marker in form["data_schema"].schema}


@pytest.mark.asyncio
async def test_setup_flow_stores_the_connection_and_the_limits(monkeypatch):
    """The power ceilings come from the connection step (there is no device
    value to offer), the capacity is required (the GX does not report one),
    and the Marstek-only voltage taper is neither offered nor stored."""
    diagnose = AsyncMock(return_value=None)
    monkeypatch.setattr(VictronEssDriver, "diagnose", diagnose)
    flow = MarstekVenusConfigFlow()
    flow.config_data = {"num_batteries": 1}
    flow._current_battery_data = {"brand": "victron"}

    limits = await flow.async_step_battery_connection_victron(dict(_CONNECTION))

    diagnose.assert_awaited_once_with("10.0.1.31", 502, 227, 100)
    assert limits["step_id"] == "battery_limits"
    markers = _markers(limits)
    assert "max_charge_power" not in markers
    assert "max_discharge_power" not in markers
    assert "full_charge_voltage_taper_enabled" not in markers
    assert "battery_capacity_kwh" in markers
    assert not callable(markers["battery_capacity_kwh"].default)   # required, no default
    assert markers["min_soc"].default() == 10

    flow.async_step_time_slots = AsyncMock(return_value={"type": "form", "step_id": "time_slots"})
    await flow.async_step_battery_limits({
        "max_soc": 100, "min_soc": 20, "charge_hysteresis_percent": 5,
        "backup_offgrid_threshold": 50, "battery_capacity_kwh": 4.7,
    })

    stored = flow.config_data["batteries"][0]
    assert stored["brand"] == "victron"
    assert stored["host"] == "10.0.1.31"
    assert stored["victron_vebus_unit_id"] == 227
    assert stored["victron_system_unit_id"] == 100
    assert stored["max_charge_power"] == 3000
    assert stored["max_discharge_power"] == 2500
    assert stored["battery_capacity_kwh"] == 4.7
    assert stored["full_charge_voltage_taper_enabled"] is False


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "error", ["cannot_connect", "victron_system_unit_id", "victron_vebus_unit_id", "victron_no_ess"],
)
async def test_setup_flow_names_what_did_not_answer(monkeypatch, error):
    monkeypatch.setattr(VictronEssDriver, "diagnose", AsyncMock(return_value=error))
    flow = MarstekVenusConfigFlow()
    flow.config_data = {"num_batteries": 1}
    flow._current_battery_data = {"brand": "victron"}

    form = await flow.async_step_battery_connection_victron(dict(_CONNECTION))

    assert form["step_id"] == "battery_connection_victron"
    assert form["errors"] == {"base": error}


@pytest.mark.asyncio
async def test_options_flow_edits_a_victron_battery(monkeypatch):
    entry = SimpleNamespace(entry_id="test-entry", data={"batteries": [dict(_STORED)]})
    flow = OptionsFlowHandler(entry)
    flow.hass = SimpleNamespace(
        config_entries=SimpleNamespace(
            async_get_known_entry=lambda entry_id: entry if entry_id == entry.entry_id else None
        )
    )
    flow.handler = entry.entry_id

    form = await flow.async_step_battery_brand({"brand": "victron"})

    assert form["step_id"] == "battery_connection_victron"
    markers = _markers(form)
    assert markers["host"].default() == "10.0.1.31"           # the stored battery
    assert markers["victron_vebus_unit_id"].default() == 227
    assert markers["max_charge_power"].default() == 3000

    monkeypatch.setattr(VictronEssDriver, "diagnose", AsyncMock(return_value=None))
    flow.async_step_battery_limits = AsyncMock(return_value={"type": "form", "step_id": "battery_limits"})
    form = await flow.async_step_battery_connection_victron(
        dict(_CONNECTION, victron_vebus_unit_id=246, max_charge_power=3500)
    )

    assert form["step_id"] == "battery_limits"
    assert flow._current_battery_data["victron_vebus_unit_id"] == 246
    assert flow._current_battery_data["max_charge_power"] == 3500


def test_every_victron_string_exists_in_every_language():
    paths = ["custom_components/omnibattery/strings.json"] + sorted(
        glob.glob("custom_components/omnibattery/translations/*.json")
    )
    assert len(paths) == 7
    for path in paths:
        data = json.load(open(path, encoding="utf-8"))
        for section, step in (
            ("config", "battery_connection_victron"),
            ("config", "reconfigure_battery_victron"),
            ("options", "battery_connection_victron"),
        ):
            texts = data[section]["step"][step]
            assert texts["title"] and texts["description"], (path, section, step)
            assert "{battery_num}" in texts["title"], (path, section, step)
            for key in ("name", "host", "port", "victron_vebus_unit_id", "victron_system_unit_id"):
                assert texts["data"][key], (path, section, step, key)
        for section in ("config", "options"):
            for error in ("victron_system_unit_id", "victron_vebus_unit_id", "victron_no_ess"):
                assert data[section]["error"][error], (path, section, error)
        for platform, keys in (
            ("sensor", ("ac_power_setpoint", "ac_out_load", "ess_control_mode")),
            ("switch", ("disable_charge", "disable_feedin")),
        ):
            for key in keys:
                assert data["entity"][platform][key]["name"], (path, platform, key)
