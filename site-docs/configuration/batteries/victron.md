# Victron Multiplus/Quattro (ESS external control)

Omnibattery drives a Victron Multiplus or Quattro behind a GX device (Cerbo GX, Venus GX, or Venus OS) through ESS **external control** (Hub-4 Mode 3): it writes the AC power setpoint directly over Modbus TCP instead of relying on the GX device's own self-consumption loop.

!!! warning "Experimental — still pending a live-hardware test"
    Every register this driver reads or writes was checked against `attributes.csv` from the official `victronenergy/dbus_modbustcp` GitHub repository — the actual table Victron's own Modbus-TCP server is generated from — not a secondhand transcription. That still didn't catch everything: a reviewer running read-only checks against their own GX device found two real problems before any control was ever enabled — a wrong consumption register (fixed: see "What you will see" below) and an ESS-mode safety check that would have stayed silently inert on their hardware (fixed: the check now refuses to write whenever it can't positively confirm External control, not just when it can positively confirm the opposite). None of this replaces a live test: start with manual, low-power commands (e.g. +200 W, then -200 W) with the battery otherwise idle, and confirm the grid sensor and battery power move the way you expect before enabling automatic control.

    One behavioural note worth knowing before you start: switching **Settings → ESS → Mode** to **External control** disables Victron's **BatteryLife** feature for as long as external control is active — this is Victron's own documented design, not something Omnibattery does.

## Do I need it?

**Use it if** your Multiplus/Quattro's AC-out carries the house load at the same point your Home Assistant grid sensor measures. The battery's state of charge (SOC) and power come from the GX device's system aggregate, which exists on every installation regardless of whether the battery also answers as its own Modbus service.

**Do not use it if** your installation's topology is different — a Multi that is one of several parallel AC-coupled sources, rather than the single path to the meter, needs a different control mapping than this driver provides.

| Supported equipment | Connection | Automatic control | Important limit |
|---|---|---|---|
| Multiplus/Quattro behind a GX device | Modbus TCP to the GX device (not the Multi directly) | Yes, via ESS external control | Single-phase only; the AC power setpoint must be rewritten at least once every 60 s or the Multi reverts to passthrough — Omnibattery does this automatically. |

Omnibattery reads battery SOC and power from the system aggregate, the AC-out load specifically (not total house consumption — see below), and the ESS AC power setpoint readback. The AC-out reading matters for correctness, not just display: the setpoint register controls power at the Multi's AC-input, not the battery, so every write adds the current AC-out load to the requested battery power before sending it — without that compensation, a "charge" command could discharge the battery instead whenever there's a load on the output. Omnibattery writes the setpoint and, optionally, the `DisableCharge`/`DisableFeedIn` safety switches.

## Before you start

- Reach the GX device's Modbus TCP server from Home Assistant: **Settings → Services → Modbus/TCP** on the GX device, enabled.
- Find the Multiplus/Quattro (`vebus`) unit id under **Settings → Services → Modbus/TCP → Available services** on the GX device. It is not guessable and varies per installation. The **system unit id** defaults to 100 — Victron's own documented default — and rarely needs changing.
- Set **Settings → ESS → Mode** to **External control** on the GX device. Do this only once you are ready to let Omnibattery drive it — this disables the GX device's own self-consumption loop.
- Decide your charge/discharge power ceiling yourself: unlike most other brands, there is no hardware-reported value to check this against, so enter it deliberately in the wizard.

## How to add it

1. Open **Settings → Devices & services → Add integration** and select **Omnibattery**. For an existing installation, open Omnibattery and select **Configure**.
2. At **Battery _n_ — Brand**, select **Victron Multiplus/Quattro (ESS external control)**.
3. At **Configure battery _n_ — Connection (Victron)**, enter **Name**, the GX device's **IP Address** and **Modbus Port**, the **vebus unit id**, leave **System unit id** at 100 unless you know otherwise, and set your **Max Charge Power** / **Max Discharge Power**.
4. Finish the shared power and SOC limits, including nominal battery capacity — this route does not report it natively.

The wizard confirms both unit ids answer over Modbus; it cannot confirm the register map itself is correct for your Venus OS version.

## What you will see

You will see automatic battery control and the shared manual power controls, plus **ESS Disable Charge** and **ESS Disable Feed-in** switches, an **ESS AC Power Setpoint** sensor (the raw grid-point register, after AC-out compensation — not the battery power), and two more diagnostic sensors: **AC Output Load** (specifically the load on the Multi's AC-out port — deliberately not Victron's generic "consumption" figure, which can also include load wired directly on AC-in and would overstate this term wherever that applies) and **ESS Control Mode** (reports "External control" only when the GX device's own ESS-mode setting agrees).

A zero command means the battery is left alone (import is set to match the AC-out load exactly). Stopping Omnibattery (or losing the connection) releases control after at most 60 seconds, when the Multi's own watchdog reverts it to passthrough; there is currently no confirmed way to restore the GX device's own ESS mode immediately, and BatteryLife stays disabled until you switch ESS Mode back yourself.

Two things make Omnibattery refuse to write a setpoint at all, by design: the **AC Output Load** reading being unavailable (no successful read yet, or the register failed — better to do nothing than send an uncompensated value), and **ESS Control Mode** being anything other than a *positively confirmed* "External control". That second one is deliberately strict: an unreadable or missing mode check refuses exactly like a confirmed-wrong one — a GX device that can't answer this check at all gets no writes, not writes-by-default.

Cell-balance monitoring, voltage taper, and MPPT solar telemetry are not available for this route.

## If it does not work

- Confirm Modbus/TCP is enabled on the GX device and reachable from Home Assistant.
- Re-check the vebus unit id under **Settings → Services → Modbus/TCP → Available services** — it can differ from what another tool or guide assumed. If your GX device's system aggregate does not answer on unit id 100, re-check that value too.
- Confirm **Settings → ESS → Mode** is still **External control**; anything else means the GX device's own loop is driving the Multi, not Omnibattery. The **ESS Control Mode** sensor confirms this directly — and if it's missing entirely, nothing will be written at all, which is the safe failure mode.
- If nothing is being commanded at all, check both **AC Output Load** and **ESS Control Mode** — either one being unavailable or unconfirmed blocks every write by design.
- If the battery SOC or power reading looks wrong, compare against the official Victron GX Home Assistant integration's own sensors for the same battery, if installed.
