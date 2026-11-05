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

??? "Technical notes: register map, design decisions, and what caught two real bugs"
    **Register map** (unit ids: the vebus unit id you entered for the Multi; the system unit id, default 100, for the system aggregate and settings service). Every register below is checked against `attributes.csv` from the official `victronenergy/dbus_modbustcp` GitHub repository — the actual table Victron's own Modbus-TCP server is generated from.

    | Register | Path | Type | Role |
    |---|---|---|---|
    | 37 | `Hub4/L1/AcPowerSetpoint` | int16, W | Write: the control point (AC-input power target) |
    | 38 / 39 | `Hub4/DisableCharge` / `DisableFeedIn` | uint16 | Write: the two safety switches |
    | 842 | `Dc/Battery/Power` | int16, W | Read: battery power, +charge/-discharge |
    | 843 | `Dc/Battery/Soc` | uint16, % | Read: battery SOC |
    | 878 | `Ac/ConsumptionOnOutput/L1/Power` | int32, W | Read: AC-out load, the compensation term |
    | 2902 | `Settings/CGwacs/Hub4Mode` | uint16 | Read: the ESS-mode safety gate (`3` = External control) |

    Registers 37/38/39 are marked write-only in `attributes.csv`; the server's own source (`mappings.cpp`) shows its read path applies no such restriction, so reading them back for confirmation works regardless.

    **Why AC-out load, not total consumption.** `AcPowerSetpoint` targets the Multi's AC-*input*, not the battery: energy balance at the Multi is `AC_in = AC_out + battery_power`. A command that writes the desired battery power straight into the register only works when AC-out load happens to be zero — with any house load present, it asks for the wrong thing entirely (a "charge" intent can command a discharge). The fix reads register 878 specifically, not Victron's generic `Ac/Consumption/L1/Power` (817): that aggregate can include load wired directly on AC-in, bypassing the Multi, which would overstate the compensation term on an installation with such a load.

    **Why the ESS-mode gate is fail-closed.** An early version of this check used `/SystemState/State` instead of `Hub4Mode`, and treated an unanswered check as "nothing to act on, proceed." On real hardware, that register returned a read error while `Hub4Mode` answered fine and read plain ESS, not External control — a permissive default would have written anyway, competing with Victron's own ESS algorithm for the same Multi. The gate now refuses whenever it cannot positively confirm `Hub4Mode == 3`; an unread or unreadable check is treated exactly like a confirmed-wrong one, never as permission.

    **Why a confirmed value still expires.** Confirming `Hub4Mode` or the AC-out load once and then caching it is not enough on its own: if that register's reads start silently failing afterwards — a Modbus hiccup, a GX-side restart of just that service — the driver would otherwise keep trusting a confirmation that may no longer be true, indefinitely. Both values now carry the timestamp of their last successful read, and `apply_setpoint` treats a value older than the same watchdog window the hardware itself uses (60 s) as equivalent to never having been confirmed at all. A single missed poll still doesn't trip this — only a reading that stops refreshing for the full window does.

    **What a mock-based test suite can and can't prove.** The driver's unit tests use a fake Modbus client and prove the arithmetic and control-flow logic — clamping, the compensation formula, which refusal fires when. They cannot prove a given register answers on a given GX device, or that one register reports what its name implies versus a neighbouring one with a similar name. Both real bugs above were caught by running read-only checks against actual hardware, not by the test suite — which is also why automatic control has not yet been enabled on any real installation as of this writing.

    **What's still unverified:** the compensation formula itself has not been exercised with a live command against a real grid sensor (the recommended first test is a manual +200 W / -200 W step with the battery otherwise idle); the maximum simultaneous Modbus-TCP connections a GX device tolerates; and whether `Hub4Mode` is writable (only reading it is confirmed), which is why `standby()` cannot restore the GX device's own ESS mode and instead relies on the hardware's 60-second watchdog.
