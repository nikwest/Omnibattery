"""Config flow: reconfiguring a Victron battery.

Without its own reconfigure step a Victron battery fell through to the
Marstek form, whose Marstek probe can never pass against a GX device.
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

import pytest

from custom_components.omnibattery.config_flow import MarstekVenusConfigFlow
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
    probe = AsyncMock(return_value=True)
    monkeypatch.setattr(VictronEssDriver, "probe", probe)
    flow, entry = _flow()

    result = await flow.async_step_reconfigure_battery_victron(
        _submit(host="10.0.1.40", victron_vebus_unit_id=246)
    )

    assert result == {"type": "abort"}
    probe.assert_awaited_once_with("10.0.1.40", 502, 246, 100)
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
    monkeypatch.setattr(VictronEssDriver, "probe", AsyncMock(return_value=True))
    flow, _ = _flow()

    await flow.async_step_reconfigure_battery_victron(_submit(victron_system_unit_id=0))

    flow._migrate_battery_registry_ids.assert_not_called()
    assert flow._reconfigure_batteries[0]["victron_system_unit_id"] == 0


@pytest.mark.asyncio
async def test_reconfigure_failed_probe_keeps_the_form_and_changes_nothing(monkeypatch):
    monkeypatch.setattr(VictronEssDriver, "probe", AsyncMock(return_value=False))
    flow, _ = _flow()

    form = await flow.async_step_reconfigure_battery_victron(_submit(host="10.0.1.99"))

    assert form["step_id"] == "reconfigure_battery_victron"
    assert form["errors"] == {"base": "cannot_connect"}
    assert flow._reconfigure_batteries == []
    flow._migrate_battery_registry_ids.assert_not_called()
    flow.async_update_reload_and_abort.assert_not_called()
