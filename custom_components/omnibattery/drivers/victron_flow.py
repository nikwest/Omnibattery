"""Victron config flow: GX connection + unit ids + power ceilings, required capacity.

Imports only ``drivers.setup``, ``const`` and Home Assistant. ``victron.py``
imports ``VICTRON_SETUP`` from here to set ``VictronEssDriver.SETUP``, so this
module cannot import ``victron.py`` at the top (cycle): the handlers import
``VictronEssDriver`` inside the function. It is the same class object, so
patching ``VictronEssDriver.diagnose`` works as usual.
"""
from __future__ import annotations

import logging
from typing import Any

import voluptuous as vol
from homeassistant.const import CONF_HOST, CONF_NAME, CONF_PORT
from homeassistant.data_entry_flow import FlowResult
from homeassistant.helpers.selector import (
    NumberSelector,
    NumberSelectorConfig,
    NumberSelectorMode,
)

from ..const import CONF_DC_PV_CONNECTED, CONF_FULL_CHARGE_VOLTAGE_TAPER_ENABLED
from .setup import DriverSetup, FlowKind, SetupContext

_LOGGER = logging.getLogger(__name__)

STEP_CONNECTION = "battery_connection_victron"
STEP_RECONFIGURE = "reconfigure_battery_victron"

# No hardware-reported ceiling exists for Victron (unlike Anker/Huawei): the
# installed base spans everything from a single small Multiplus to a stacked
# Quattro system, so the user enters the real ceiling directly in the
# connection step. This only bounds the input form against a typo.
_VICTRON_MAX_POWER_W = 20000
_DEFAULT_POWER_W = 5000


def _connection_schema(
    battery_num: int, saved: dict | None = None, *, include_power: bool = True
) -> vol.Schema:
    """The connection form for adding, editing and reconfiguring.

    ``saved`` pre-fills an edit. Reconfigure passes ``include_power=False``: it
    only moves the connection, and the power ceilings stay in the options flow.
    """
    saved = saved or {}
    unit_id = vol.All(
        NumberSelector(NumberSelectorConfig(min=0, max=247, step=1, mode=NumberSelectorMode.BOX)),
        vol.Coerce(int),
    )
    power = NumberSelector(
        NumberSelectorConfig(
            min=100, max=_VICTRON_MAX_POWER_W, step=50,
            unit_of_measurement="W", mode=NumberSelectorMode.BOX,
        )
    )
    # No default for the vebus unit id on a new battery: it is not guessable.
    vebus_key = (
        vol.Required("victron_vebus_unit_id", default=saved["victron_vebus_unit_id"])
        if "victron_vebus_unit_id" in saved else vol.Required("victron_vebus_unit_id")
    )
    host_key = (
        vol.Required(CONF_HOST, default=saved[CONF_HOST])
        if saved.get(CONF_HOST) else vol.Required(CONF_HOST)
    )
    fields = {
        vol.Required(CONF_NAME, default=saved.get(CONF_NAME, f"Victron ESS {battery_num}")): str,
        host_key: str,
        vol.Optional(CONF_PORT, default=saved.get(CONF_PORT, 502)): vol.All(
            vol.Coerce(int), vol.Range(min=1, max=65535)
        ),
        vebus_key: unit_id,
        vol.Required("victron_system_unit_id", default=saved.get("victron_system_unit_id", 100)): unit_id,
    }
    if include_power:
        fields[vol.Required(
            "max_charge_power", default=saved.get("max_charge_power", _DEFAULT_POWER_W)
        )] = power
        fields[vol.Required(
            "max_discharge_power", default=saved.get("max_discharge_power", _DEFAULT_POWER_W)
        )] = power
    return vol.Schema(fields)


def _power_ceilings(battery_data: dict) -> tuple[int, int]:
    """The ceilings the user entered in the connection step, bounded like the form.

    Unlike Anker/Huawei there is no device-reported value to clamp against.
    """
    def bounded(key: str) -> int:
        return max(100, min(_VICTRON_MAX_POWER_W, int(battery_data.get(key, _VICTRON_MAX_POWER_W))))

    return bounded("max_charge_power"), bounded("max_discharge_power")


async def _diagnose(host: str, port: int, vebus_unit_id: int, system_unit_id: int) -> str | None:
    from .victron import VictronEssDriver  # local: victron.py imports this module

    _LOGGER.info(
        "Probing Victron GX device at %s:%s (vebus unit %s, system unit %s)",
        host, port, vebus_unit_id, system_unit_id,
    )
    return await VictronEssDriver.diagnose(host, port, vebus_unit_id, system_unit_id)


def _unit_ids(user_input: dict) -> tuple[int, int]:
    return int(user_input["victron_vebus_unit_id"]), int(user_input.get("victron_system_unit_id", 100))


async def _connection(flow, user_input: dict[str, Any] | None, ctx: SetupContext) -> FlowResult | dict:
    """Add (config flow) or edit (options flow) a Multiplus/Quattro behind a GX device.

    host/port address the GX device itself, not the Multi — it has no network
    connection of its own. The vebus unit id is not guessable; it is listed
    under Settings -> Services -> Modbus/TCP -> Available services on the GX.
    """
    errors: dict[str, str] = {}
    battery_num = flow.battery_index + 1

    if user_input is not None:
        host = user_input[CONF_HOST]
        port = int(user_input.get(CONF_PORT, 502))
        vebus_unit_id, system_unit_id = _unit_ids(user_input)
        error = await _diagnose(host, port, vebus_unit_id, system_unit_id)
        if error is None:
            return {
                CONF_NAME: user_input[CONF_NAME],
                CONF_HOST: host,
                CONF_PORT: port,
                "victron_vebus_unit_id": vebus_unit_id,
                "victron_system_unit_id": system_unit_id,
                "max_charge_power": int(user_input["max_charge_power"]),
                "max_discharge_power": int(user_input["max_discharge_power"]),
                "brand": "victron",
            }
        errors["base"] = error

    saved = ctx.battery if ctx.kind is FlowKind.OPTIONS else None
    return flow.async_show_form(
        step_id=STEP_CONNECTION,
        data_schema=_connection_schema(battery_num, saved),
        errors=errors,
        description_placeholders={"battery_num": str(battery_num)},
    )


async def _reconfigure(flow, user_input: dict[str, Any] | None, ctx: SetupContext) -> FlowResult | dict:
    """Move a Victron battery to another GX address or unit ids (reconfigure source).

    Connection only, like the other brands' reconfigure steps: the power
    ceilings and every other setting stay as stored (the options flow edits
    them).
    """
    current = ctx.battery or {}
    battery_num = flow.battery_index + 1
    errors: dict[str, str] = {}

    if user_input is not None:
        new_host = user_input[CONF_HOST]
        new_port = int(user_input.get(CONF_PORT, 502))
        vebus_unit_id, system_unit_id = _unit_ids(user_input)
        error = await _diagnose(new_host, new_port, vebus_unit_id, system_unit_id)
        if error is not None:
            errors["base"] = error
        else:
            old_host = current.get(CONF_HOST)
            old_port = current.get(CONF_PORT)
            # The unit ids select services on the same GX device; they are not
            # part of the battery's identity. Only a moved GX device (host/port)
            # moves the registry ids.
            if old_host and old_port and (old_host != new_host or old_port != new_port):
                flow._migrate_battery_registry_ids(
                    ctx.entry, old_host, old_port, new_host, new_port
                )
            return {
                CONF_NAME: user_input[CONF_NAME],
                CONF_HOST: new_host,
                CONF_PORT: new_port,
                "victron_vebus_unit_id": vebus_unit_id,
                "victron_system_unit_id": system_unit_id,
                "brand": "victron",
            }

    return flow.async_show_form(
        step_id=STEP_RECONFIGURE,
        data_schema=_connection_schema(battery_num, current, include_power=False),
        errors=errors,
        description_placeholders={"battery_num": str(battery_num)},
    )


class _VictronSetup(DriverSetup):
    def power_ceilings(self, battery_data: dict) -> tuple[int, int]:
        return _power_ceilings(battery_data)

    def limits_schema(self, battery_data: dict, base_fields: dict, ctx: SetupContext) -> dict:
        """Base fields without the power ceilings (already asked in the connection
        step), plus a required capacity: the GX device reports none.

        Options flow: the capacity defaults to the one stored for this slot.
        """
        fields = {
            key: selector for key, selector in base_fields.items()
            if str(key.schema) not in ("max_charge_power", "max_discharge_power")
        }
        saved = 0.0
        if ctx.kind is FlowKind.OPTIONS and ctx.battery is not None:
            saved = float((ctx.battery | battery_data).get("battery_capacity_kwh", 0.0))
        capacity_field = (
            vol.Required("battery_capacity_kwh", default=saved)
            if saved > 0 else vol.Required("battery_capacity_kwh")
        )
        fields[capacity_field] = NumberSelector(
            NumberSelectorConfig(
                min=0.01, max=100, step=0.01, unit_of_measurement="kWh", mode=NumberSelectorMode.BOX
            )
        )
        return fields

    def finalize_limits(
        self, merged: dict, user_input: dict, battery_data: dict, ctx: SetupContext
    ) -> None:
        # The ceilings were entered in the connection step. In the options flow
        # ``merged`` is the stored battery with this flow's connection data on top.
        source = merged if ctx.kind is FlowKind.OPTIONS else battery_data
        merged["max_charge_power"], merged["max_discharge_power"] = _power_ceilings(source)
        merged[CONF_DC_PV_CONNECTED] = False
        merged[CONF_FULL_CHARGE_VOLTAGE_TAPER_ENABLED] = False
        merged["battery_capacity_kwh"] = round(float(user_input["battery_capacity_kwh"]), 2)


VICTRON_SETUP = _VictronSetup(
    steps={STEP_CONNECTION: _connection, STEP_RECONFIGURE: _reconfigure},
    entry_step=STEP_CONNECTION,
    reconfigure_entry_step=STEP_RECONFIGURE,
)
