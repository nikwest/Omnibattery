# Victron Multiplus/Quattro (ESS external control)

Omnibattery drives a Victron Multiplus or Quattro behind a GX device (Cerbo GX, Venus GX, or Venus OS) through ESS **external control** (Hub-4 Mode 3): it writes the AC power setpoint directly over Modbus TCP instead of relying on the GX device's own self-consumption loop.

!!! warning "Experimental — still pending a live-hardware test"
    Every register this driver reads or writes was checked against `attributes.csv` from the official `victronenergy/dbus_modbustcp` GitHub repository — the actual table Victron's own Modbus-TCP server is generated from — not a secondhand transcription. The AC-consumption compensation formula (see "What you will see" below) matches Victron's own external-control-loop documentation word for word: *"Power to/from AC-input = Power to/from battery + Power to/from AC-output."* None of that replaces a live test, though: start with manual, low-power commands (e.g. +200 W, then -200 W) with the battery otherwise idle, and confirm the grid sensor and battery power move the way you expect before enabling automatic control.

    One behavioural note worth knowing before you start: switching **Settings → ESS → Mode** to **External control** disables Victron's **BatteryLife** feature for as long as external control is active — this is Victron's own documented design, not something Omnibattery does.

## Do I need it?

**Use it if** your Multiplus/Quattro's AC-out carries the house load at the same point your Home Assistant grid sensor measures. The battery's state of charge (SOC) and power come from the GX device's system aggregate, which exists on every installation regardless of whether the battery also answers as its own Modbus service.

**Do not use it if** your installation's topology is different — a Multi that is one of several parallel AC-coupled sources, rather than the single path to the meter, needs a different control mapping than this driver provides.

| Supported equipment | Connection | Automatic control | Important limit |
|---|---|---|---|
| Multiplus/Quattro behind a GX device | Modbus TCP to the GX device (not the Multi directly) | Yes, via ESS external control | Single-phase only; the AC power setpoint must be rewritten at least once every 60 s or the Multi reverts to passthrough — Omnibattery does this automatically. |

Omnibattery reads battery SOC and power from the system aggregate, the current AC consumption (house load), and the ESS AC power setpoint readback. The AC consumption reading matters for correctness, not just display: the setpoint register controls power at the Multi's AC-input, not the battery, so every write adds the current house load to the requested battery power before sending it — without that compensation, a "charge" command could discharge the battery instead whenever there's a load on the output. Omnibattery writes the setpoint and, optionally, the `DisableCharge`/`DisableFeedIn` safety switches.

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

You will see automatic battery control and the shared manual power controls, plus **ESS Disable Charge** and **ESS Disable Feed-in** switches, an **ESS AC Power Setpoint** sensor (the raw grid-point register, after house-load compensation — not the battery power), and two more diagnostic sensors: **AC Consumption** (the house-load reading the compensation uses) and **ESS Control Mode** (reports "External control" when the GX device's own aggregated state agrees — and reports e.g. "Passthru" if the 60-second watchdog has already reverted the Multi, which is worth noticing).

A zero command means the battery is left alone (import is set to match consumption exactly). Stopping Omnibattery (or losing the connection) releases control after at most 60 seconds, when the Multi's own watchdog reverts it to passthrough; there is currently no confirmed way to restore the GX device's own ESS mode immediately, and BatteryLife stays disabled until you switch ESS Mode back yourself.

If the AC Consumption reading is unavailable (no successful read yet, or the register failed), Omnibattery refuses to write a setpoint rather than send an uncompensated value — better to do nothing than repeat the compensation bug described above. The same applies if **ESS Control Mode** positively confirms the GX device is *not* in External control.

Cell-balance monitoring, voltage taper, and MPPT solar telemetry are not available for this route.

## If it does not work

- Confirm Modbus/TCP is enabled on the GX device and reachable from Home Assistant.
- Re-check the vebus unit id under **Settings → Services → Modbus/TCP → Available services** — it can differ from what another tool or guide assumed. If your GX device's system aggregate does not answer on unit id 100, re-check that value too.
- Confirm **Settings → ESS → Mode** is still **External control**; anything else means the GX device's own loop is driving the Multi, not Omnibattery. The **ESS Control Mode** sensor confirms this directly — "Passthru" specifically means the write watchdog already lapsed.
- If nothing is being commanded at all, check the **AC Consumption** sensor — a missing or stale reading there blocks every write by design.
- If the battery SOC or power reading looks wrong, compare against the official Victron GX Home Assistant integration's own sensors for the same battery, if installed.
