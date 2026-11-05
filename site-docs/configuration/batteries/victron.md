# Victron Multiplus/Quattro (ESS external control)

Omnibattery drives a Victron Multiplus or Quattro behind a GX device (Cerbo GX, Venus GX, or Venus OS) through ESS **external control** (Hub-4 Mode 3): it writes the AC power setpoint directly over Modbus TCP instead of relying on the GX device's own self-consumption loop.

!!! warning "Experimental — control path confirmed, telemetry path partially confirmed"
    This route was built from Victron's own ESS external-control documentation. The control registers (`AcPowerSetpoint`, `DisableCharge`, `DisabledFeedIn`) are confirmed against Victron's own documentation. Battery SOC and power are read from the **system aggregate** service (unit id 100, registers 843/842) rather than a separate per-battery service — this was corrected against a real installation's register map, since the installation's Modbus-TCP bridge did not expose a separate communicating-battery service, and an earlier register guess for battery power was wrong (it pointed at starter-battery voltage, not the house battery's power). The sign of the control command is still reasoned from topology, not measured — verify it on your own installation (a small +200 W / -200 W step, watched against your grid sensor) before leaving it unattended.

## Do I need it?

**Use it if** your Multiplus/Quattro's AC-out carries the house load at the same point your Home Assistant grid sensor measures. The battery's state of charge (SOC) and power come from the GX device's system aggregate, which exists on every installation regardless of whether the battery also answers as its own Modbus service.

**Do not use it if** your installation's topology is different — a Multi that is one of several parallel AC-coupled sources, rather than the single path to the meter, needs a different control mapping than this driver provides.

| Supported equipment | Connection | Automatic control | Important limit |
|---|---|---|---|
| Multiplus/Quattro behind a GX device | Modbus TCP to the GX device (not the Multi directly) | Yes, via ESS external control | Single-phase only; the AC power setpoint must be rewritten at least once every 60 s or the Multi reverts to passthrough — Omnibattery does this automatically. |

Omnibattery reads battery SOC and power from the system aggregate, and the ESS AC power setpoint readback. It writes the setpoint and, optionally, the `DisableCharge`/`DisabledFeedIn` safety switches.

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

You will see automatic battery control and the shared manual power controls, plus **ESS Disable Charge** and **ESS Disable Feed-in** switches and an **ESS AC Power Setpoint** diagnostic sensor.

A zero command is not an idle/off state — it asks the Multi to balance to zero grid flow, which is an active command, not a release. Stopping Omnibattery (or losing the connection) releases control after at most 60 seconds, when the Multi's own watchdog reverts it to passthrough; there is currently no confirmed way to restore the GX device's own ESS mode immediately.

Cell-balance monitoring, voltage taper, and MPPT solar telemetry are not available for this route.

## If it does not work

- Confirm Modbus/TCP is enabled on the GX device and reachable from Home Assistant.
- Re-check the vebus unit id under **Settings → Services → Modbus/TCP → Available services** — it can differ from what another tool or guide assumed. If your GX device's system aggregate does not answer on unit id 100, re-check that value too.
- Confirm **Settings → ESS → Mode** is still **External control**; anything else means the GX device's own loop is driving the Multi, not Omnibattery.
- If the battery SOC or power reading looks wrong, compare against the official Victron GX Home Assistant integration's own sensors for the same battery, if installed.
