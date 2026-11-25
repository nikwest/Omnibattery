"""The Victron GX transport seam.

Two halves: the Modbus-TCP transport on its own (register map, scaling, block
spans, the invalid-value placeholder), and the driver running on a transport
that is not Modbus at all — proof that the control logic depends only on the
D-Bus-level contract in drivers/victron_transport.py.
"""
from __future__ import annotations

from unittest.mock import Mock

import pytest

from custom_components.omnibattery.drivers import victron_transport as gx
from custom_components.omnibattery.drivers.victron import VictronEssDriver
from custom_components.omnibattery.drivers.victron_transport import ModbusGxTransport
from custom_components.omnibattery.infra.victron_modbus_client import encode_i16


# ----------------------------------------------------------------------
# ModbusGxTransport
# ----------------------------------------------------------------------

class _FakeModbus:
    """A GX Modbus-TCP server holding a sparse register image per unit id."""

    def __init__(self, image: dict[int, dict[int, int]]):
        self.image = image
        self.connected = True
        self.reads: list[tuple[int, int, int]] = []
        self.writes: list[tuple[int, int, list[int]]] = []
        self.fail = False

    async def async_connect(self):
        return True

    async def async_close(self):
        pass

    def set_shutting_down(self, value):
        pass

    async def async_read_holding_block(self, unit_id, start, count):
        self.reads.append((unit_id, start, count))
        if self.fail:
            return None
        regs = self.image.get(unit_id, {})
        return [regs.get(start + i, 0) for i in range(count)]

    async def async_write_registers(self, unit_id, start, values):
        self.writes.append((unit_id, start, list(values)))
        return True


def _modbus(image=None):
    fake = _FakeModbus(image or {
        100: {840: 532, 842: encode_i16(-750), 843: 67, 2902: 3},
        227: {23: encode_i16(-60), 26: 5318, 37: encode_i16(1200), 38: 1, 39: 0},
    })
    transport = ModbusGxTransport(
        "1.2.3.4", vebus_unit_id=227, system_unit_id=100, client=fake,
    )
    return transport, fake


@pytest.mark.asyncio
async def test_modbus_reads_neighbours_as_one_block_in_physical_units():
    transport, fake = _modbus()

    values = await transport.read(gx.SYSTEM, [
        gx.SYSTEM_BATTERY_VOLTAGE, gx.SYSTEM_BATTERY_POWER, gx.SYSTEM_BATTERY_SOC,
    ])

    assert values == {
        gx.SYSTEM_BATTERY_VOLTAGE: pytest.approx(53.2),
        gx.SYSTEM_BATTERY_POWER: -750,
        gx.SYSTEM_BATTERY_SOC: 67,
    }
    assert fake.reads == [(100, 840, 4)]   # 840-843 in one request


@pytest.mark.asyncio
async def test_modbus_ac_out_power_is_10_w_per_count_and_signed():
    transport, fake = _modbus()

    values = await transport.read(gx.VEBUS, [gx.VEBUS_AC_OUT_L1_POWER, gx.VEBUS_DC_VOLTAGE])

    assert values[gx.VEBUS_AC_OUT_L1_POWER] == -600
    assert values[gx.VEBUS_DC_VOLTAGE] == pytest.approx(53.18)
    assert fake.reads == [(227, 23, 4)]


@pytest.mark.asyncio
async def test_modbus_settings_share_the_system_unit_id_and_its_trust():
    """Hub4Mode has no voltage of its own; it lives on the system unit id and
    restarts with it (reading 0, not a mode), so it follows that witness."""
    transport, fake = _modbus()

    assert await transport.read(gx.SETTINGS, [gx.SETTINGS_HUB4_MODE]) == {
        gx.SETTINGS_HUB4_MODE: None   # nothing has vouched for the system yet
    }
    await transport.read(gx.SYSTEM, [gx.SYSTEM_BATTERY_VOLTAGE])
    assert await transport.read(gx.SETTINGS, [gx.SETTINGS_HUB4_MODE]) == {
        gx.SETTINGS_HUB4_MODE: 3
    }
    assert fake.reads[-1] == (100, 2902, 1)

    fake.image[100].update({840: 0, 2902: 0})          # the GX restarting
    await transport.read(gx.SYSTEM, [gx.SYSTEM_BATTERY_VOLTAGE])
    assert await transport.read(gx.SETTINGS, [gx.SETTINGS_HUB4_MODE]) == {
        gx.SETTINGS_HUB4_MODE: None
    }


@pytest.mark.asyncio
async def test_modbus_failed_read_is_none_not_zeros():
    transport, fake = _modbus()
    fake.fail = True

    assert await transport.read(gx.SYSTEM, [gx.SYSTEM_BATTERY_SOC]) is None


@pytest.mark.asyncio
async def test_modbus_placeholder_zero_is_reported_unknown_for_the_whole_service():
    """dbus_modbustcp answers an invalid D-Bus value with 0. A 0 V witness
    means the service has no valid data: every value it served is None, and
    so is every later read of it without a witness, until a real voltage."""
    transport, fake = _modbus()
    vebus = fake.image[227]
    vebus.update({23: 0, 26: 0, 37: 0, 38: 0, 39: 0})

    ac_out = await transport.read(gx.VEBUS, [gx.VEBUS_AC_OUT_L1_POWER, gx.VEBUS_DC_VOLTAGE])
    hub4 = await transport.read(gx.VEBUS, [
        gx.VEBUS_AC_POWER_SETPOINT_L1, gx.VEBUS_DISABLE_CHARGE, gx.VEBUS_DISABLE_FEEDIN,
    ])

    assert ac_out == {gx.VEBUS_AC_OUT_L1_POWER: None, gx.VEBUS_DC_VOLTAGE: None}
    assert set(hub4.values()) == {None}

    vebus.update({23: encode_i16(15), 26: 5300, 37: encode_i16(150)})
    ac_out = await transport.read(gx.VEBUS, [gx.VEBUS_AC_OUT_L1_POWER, gx.VEBUS_DC_VOLTAGE])
    hub4 = await transport.read(gx.VEBUS, [gx.VEBUS_AC_POWER_SETPOINT_L1])
    assert ac_out[gx.VEBUS_AC_OUT_L1_POWER] == 150
    assert hub4 == {gx.VEBUS_AC_POWER_SETPOINT_L1: 150}


@pytest.mark.asyncio
async def test_modbus_services_start_untrusted_on_every_connect():
    transport, _ = _modbus()
    await transport.read(gx.VEBUS, [gx.VEBUS_AC_OUT_L1_POWER, gx.VEBUS_DC_VOLTAGE])
    assert (await transport.read(gx.VEBUS, [gx.VEBUS_AC_POWER_SETPOINT_L1]))[
        gx.VEBUS_AC_POWER_SETPOINT_L1
    ] == 1200

    await transport.connect()

    # No witness read since connecting: the setpoint readback is unknown...
    assert (await transport.read(gx.VEBUS, [gx.VEBUS_AC_POWER_SETPOINT_L1]))[
        gx.VEBUS_AC_POWER_SETPOINT_L1
    ] is None
    # ...and so is the ESS mode, which follows the system service.
    assert (await transport.read(gx.SETTINGS, [gx.SETTINGS_HUB4_MODE])) == {
        gx.SETTINGS_HUB4_MODE: None
    }


@pytest.mark.asyncio
async def test_modbus_serial_is_readable_before_any_witness():
    """Read right after connect: text needs no witness, its placeholder is
    all NULs and decodes to None by itself."""
    serial = [0x4843, 0x3132, 0x3334, 0x4142, 0, 0]  # "HC1234AB"
    transport, _ = _modbus({100: dict(zip(range(800, 806), serial))})

    assert await transport.read(gx.SYSTEM, [gx.SYSTEM_SERIAL]) == {gx.SYSTEM_SERIAL: "HC1234AB"}


@pytest.mark.asyncio
async def test_modbus_writes_encode_each_register_and_go_out_together():
    transport, fake = _modbus()

    assert await transport.write(gx.VEBUS, {gx.VEBUS_AC_POWER_SETPOINT_L1: -1500})
    assert await transport.write(gx.VEBUS, {gx.VEBUS_DISABLE_CHARGE: 1, gx.VEBUS_DISABLE_FEEDIN: 0})

    assert fake.writes == [
        (227, 37, [encode_i16(-1500)]),
        (227, 38, [1, 0]),          # one request, never one flag without the other
    ]


@pytest.mark.asyncio
async def test_modbus_refuses_to_write_registers_that_are_not_neighbours():
    transport, fake = _modbus()

    with pytest.raises(ValueError):
        await transport.write(gx.VEBUS, {
            gx.VEBUS_AC_POWER_SETPOINT_L1: 0, gx.VEBUS_DISABLE_FEEDIN: 1,
        })
    assert fake.writes == []


# ----------------------------------------------------------------------
# The driver on a transport that is not Modbus
# ----------------------------------------------------------------------

@pytest.mark.asyncio
async def test_modbus_reads_ac_in_and_ac_out_as_one_block():
    transport, fake = _modbus({227: {12: encode_i16(19), 23: encode_i16(15), 26: 5318}})

    values = await transport.read(gx.VEBUS, [
        gx.VEBUS_AC_IN_L1_POWER, gx.VEBUS_AC_OUT_L1_POWER, gx.VEBUS_DC_VOLTAGE,
    ])

    assert values[gx.VEBUS_AC_IN_L1_POWER] == 190
    assert values[gx.VEBUS_AC_OUT_L1_POWER] == 150
    assert fake.reads == [(227, 12, 15)]   # 12-26 in one request


class _PathTransport:
    """A transport in plain D-Bus terms, the way MQTT presents a GX device:
    every value addressed by service + path, and an invalid one already None
    (MQTT publishes ``{"value": null}``) — no registers, no placeholder."""

    def __init__(self):
        self.connected = True
        self.values = {
            gx.SYSTEM: {
                gx.SYSTEM_SERIAL: "HC1234AB",
                gx.SYSTEM_BATTERY_VOLTAGE: 53.2,
                gx.SYSTEM_BATTERY_POWER: -750,
                gx.SYSTEM_BATTERY_SOC: 67.5,
            },
            gx.VEBUS: {
                gx.VEBUS_AC_IN_L1_POWER: 430,
                gx.VEBUS_AC_OUT_L1_POWER: 430,
                gx.VEBUS_DC_VOLTAGE: 53.18,
                gx.VEBUS_AC_POWER_SETPOINT_L1: 0,
                gx.VEBUS_DISABLE_CHARGE: 0,
                gx.VEBUS_DISABLE_FEEDIN: 0,
            },
            gx.SETTINGS: {gx.SETTINGS_HUB4_MODE: 3},
        }
        self.writes: list[tuple[str, dict]] = []
        self.set_shutting_down = Mock()

    async def connect(self):
        return True

    async def close(self):
        pass

    async def read(self, service, paths):
        return {path: self.values[service][path] for path in paths}

    async def write(self, service, values):
        self.writes.append((service, dict(values)))
        self.values[service].update(values)
        return True


def _driver_on(transport):
    return VictronEssDriver(
        "gx.local", vebus_unit_id=0, max_charge_power_w=3000,
        max_discharge_power_w=3000, transport=transport,
    )


@pytest.mark.asyncio
async def test_driver_runs_on_a_path_transport_end_to_end():
    transport = _PathTransport()
    driver = _driver_on(transport)
    assert await driver.connect()
    assert driver.serial == "HC1234AB"

    data = await driver.read_telemetry(None)

    assert data["battery_soc"] == 67.5
    assert data["battery_voltage"] == 53.2
    assert data["ac_out_load"] == 430
    assert data["ess_control_mode"] == "External control"

    result = await driver.apply_setpoint(-1000)

    assert result.ok and result.confirmed
    # The AC-out compensation works the same: AC-in = battery + AC-out.
    assert (gx.VEBUS, {gx.VEBUS_AC_POWER_SETPOINT_L1: -570}) in transport.writes
    assert result.battery_power_w == -750


@pytest.mark.asyncio
async def test_driver_on_a_path_transport_treats_null_as_unknown_not_zero():
    """The incident, as MQTT would present it: the system service's values
    go null instead of 0. Nothing is reported as 0%, and no setpoint is
    written until the service is back."""
    transport = _PathTransport()
    driver = _driver_on(transport)
    await driver.connect()
    await driver.read_telemetry(None)
    transport.writes.clear()

    transport.values[gx.SYSTEM].update({
        gx.SYSTEM_BATTERY_VOLTAGE: None, gx.SYSTEM_BATTERY_POWER: None,
        gx.SYSTEM_BATTERY_SOC: None,
    })
    data = await driver.read_telemetry(["battery_soc", "battery_power", "battery_voltage"])
    result = await driver.apply_setpoint(500)

    assert data == {}
    assert result.failure_reason == "gx_data_invalid"
    assert not [w for w in transport.writes if gx.VEBUS_AC_POWER_SETPOINT_L1 in w[1]]

    transport.values[gx.SYSTEM].update({
        gx.SYSTEM_BATTERY_VOLTAGE: 53.2, gx.SYSTEM_BATTERY_POWER: 0,
        gx.SYSTEM_BATTERY_SOC: 67,
    })
    assert (await driver.read_telemetry(["battery_soc"]))["battery_soc"] == 67
    assert (await driver.apply_setpoint(500)).ok


@pytest.mark.asyncio
async def test_driver_on_a_path_transport_holds_the_target_on_ac_out_changes():
    transport = _PathTransport()
    driver = _driver_on(transport)
    await driver.connect()
    await driver.read_telemetry(None)
    await driver.apply_setpoint(800)
    transport.writes.clear()

    transport.values[gx.VEBUS][gx.VEBUS_AC_OUT_L1_POWER] = 1200
    await driver.read_telemetry(["ac_out_load"])

    assert transport.writes == [(gx.VEBUS, {gx.VEBUS_AC_POWER_SETPOINT_L1: 2000})]


@pytest.mark.asyncio
async def test_driver_on_a_path_transport_reports_unknown_ac_in_as_none():
    transport = _PathTransport()
    driver = _driver_on(transport)
    await driver.connect()
    transport.values[gx.VEBUS][gx.VEBUS_AC_IN_L1_POWER] = None

    data = await driver.read_telemetry(["ac_out_load", "ac_delivered_power"])

    assert data["ac_out_load"] == 430
    assert data["ac_delivered_power"] is None
