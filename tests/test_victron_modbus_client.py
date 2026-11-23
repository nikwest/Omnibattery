"""The Victron client drops a connection a request failed on.

A timed-out request leaves its reply in flight on the socket; keeping the
socket hands that reply to the next request, or every later request times out
on a half-open connection. The driver reconnects (see test_victron_driver).
"""
from __future__ import annotations

import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest
from pymodbus.exceptions import ConnectionException, ModbusIOException

from custom_components.omnibattery.drivers import victron_client as module
from custom_components.omnibattery.drivers.victron_client import VictronModbusClient


@pytest.fixture(autouse=True)
def _no_message_wait(monkeypatch):
    monkeypatch.setattr(module, "_MESSAGE_WAIT_S", 0)


def _client_with(socket):
    client = VictronModbusClient("1.2.3.4", 502, timeout=0.05)
    client._client = socket
    return client


def _socket(**calls):
    return SimpleNamespace(connected=True, close=Mock(), **calls)


def _ok(registers=None):
    return SimpleNamespace(isError=lambda: False, registers=registers or [1])


def _rejected():
    return SimpleNamespace(isError=lambda: True)


@pytest.mark.parametrize("error", [
    ConnectionException("gone"), ModbusIOException("no reply"), asyncio.TimeoutError(),
])
async def test_a_failed_read_drops_the_connection(error):
    socket = _socket(read_holding_registers=AsyncMock(side_effect=error))
    client = _client_with(socket)
    assert await client.async_read_holding_block(100, 840, 1) is None
    socket.close.assert_called_once()
    assert client.connected is False


async def test_a_read_that_never_answers_drops_the_connection():
    async def hang(**_):
        await asyncio.sleep(1)

    socket = _socket(read_holding_registers=hang)
    client = _client_with(socket)
    assert await client.async_read_holding_block(100, 840, 1) is None
    socket.close.assert_called_once()
    assert client.connected is False


@pytest.mark.parametrize("error", [ModbusIOException("no reply"), asyncio.TimeoutError()])
async def test_a_failed_write_drops_the_connection(error):
    socket = _socket(write_registers=AsyncMock(side_effect=error))
    client = _client_with(socket)
    assert await client.async_write_registers(227, 37, [0]) is False
    socket.close.assert_called_once()
    assert client.connected is False


async def test_a_rejected_request_keeps_the_connection():
    # An exception response (wrong unit id, unknown register) is an answer:
    # the socket is in sync.
    socket = _socket(
        read_holding_registers=AsyncMock(return_value=_rejected()),
        write_registers=AsyncMock(return_value=_rejected()),
    )
    client = _client_with(socket)
    assert await client.async_read_holding_block(100, 840, 1) is None
    assert await client.async_write_registers(227, 37, [0]) is False
    socket.close.assert_not_called()
    assert client.connected is True


async def test_a_successful_read_keeps_the_connection():
    socket = _socket(read_holding_registers=AsyncMock(return_value=_ok([4850])))
    client = _client_with(socket)
    assert await client.async_read_holding_block(100, 840, 1) == [4850]
    socket.close.assert_not_called()


async def test_a_late_failure_leaves_a_newer_connection_alone():
    newer = _socket()
    client = VictronModbusClient("1.2.3.4", 502, timeout=0.05)

    async def fail_after_reconnect(**_):
        client._client = newer  # a reconnect swapped the socket meanwhile
        raise ModbusIOException("no reply")

    old = _socket(read_holding_registers=fail_after_reconnect)
    client._client = old
    assert await client.async_read_holding_block(100, 840, 1) is None
    old.close.assert_called_once()
    newer.close.assert_not_called()
    assert client._client is newer
