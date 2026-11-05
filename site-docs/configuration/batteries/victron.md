# Victron Multiplus/Quattro (ESS external control)

Omnibattery drives a Victron Multiplus or Quattro behind a GX device (Cerbo GX, Venus GX, or Venus OS) through ESS **external control** (Hub-4 Mode 3): it writes the AC power setpoint directly over Modbus TCP instead of relying on the GX device's own self-consumption loop.

!!! warning "Experimental — not yet validated on real hardware"
    This route was built from Victron's own ESS external-control documentation plus community references, not from a hardware read. The control registers (`AcPowerSetpoint`, `DisableCharge`, `DisabledFeedIn`) are confirmed against Victron's own documentation; the battery SOC/power telemetry registers are not yet confirmed on a real installation. Verify the sign of the control command on your own installation (a small +200 W / -200 W step, watched against your grid sensor) before leaving it unattended.

## Do I need it?

**Use it if** your Multiplus/Quattro's AC-out carries the house load at the same point your Home Assistant grid sensor measures, and the battery behind it reports its state of charge (SOC) over CAN-bus/VE.Can so the GX device exposes a communicating battery service.

**Do not use it if** the battery has no communicating BMS (no reliable SOC source) or your installation's topology is different — a Multi that is one of several parallel AC-coupled sources, rather than the single path to the meter, needs a different control mapping than this driver provides.

| Supported equipment | Connection | Automatic control | Important limit |
|---|---|---|---|
| Multiplus/Quattro with a communicating battery, behind a GX device | Modbus TCP to the GX device (not the Multi directly) | Yes, via ESS external control | Single-phase only; the AC power setpoint must be rewritten at least once every 60 s or the Multi reverts to passthrough — Omnibattery does this automatically. |

Omnibattery reads battery SOC and power, and the ESS AC power setpoint readback. It writes the setpoint and, optionally, the `DisableCharge`/`DisabledFeedIn` safety switches.

## Before you start

- Reach the GX device's Modbus TCP server from Home Assistant: **Settings → Services → Modbus/TCP** on the GX device, enabled.
- Find the Multiplus/Quattro (`vebus`) and battery unit ids under **Settings → Services → Modbus/TCP → Available services** on the GX device. These are not guessable and vary per installation.
- Set **Settings → ESS → Mode** to **External control** on the GX device. Do this only once you are ready to let Omnibattery drive it — this disables the GX device's own self-consumption loop.
- Decide your charge/discharge power ceiling yourself: unlike most other brands, there is no hardware-reported value to check this against, so enter it deliberately in the wizard.

## How to add it

1. Open **Settings → Devices & services → Add integration** and select **Omnibattery**. For an existing installation, open Omnibattery and select **Configure**.
2. At **Battery _n_ — Brand**, select **Victron Multiplus/Quattro (ESS external control)**.
3. At **Configure battery _n_ — Connection (Victron)**, enter **Name**, the GX device's **IP Address** and **Modbus Port**, the **vebus unit id**, the **battery unit id**, and your **Max Charge Power** / **Max Discharge Power**.
4. Finish the shared power and SOC limits, including nominal battery capacity — this route does not report it natively.

The wizard confirms both unit ids answer over Modbus; it cannot confirm the register map itself is correct for your Venus OS version.

## What you will see

You will see automatic battery control and the shared manual power controls, plus **ESS Disable Charge** and **ESS Disable Feed-in** switches and an **ESS AC Power Setpoint** diagnostic sensor.

A zero command is not an idle/off state — it asks the Multi to balance to zero grid flow, which is an active command, not a release. Stopping Omnibattery (or losing the connection) releases control after at most 60 seconds, when the Multi's own watchdog reverts it to passthrough; there is currently no confirmed way to restore the GX device's own ESS mode immediately.

Cell-balance monitoring, voltage taper, and MPPT solar telemetry are not available for this route.

## If it does not work

- Confirm Modbus/TCP is enabled on the GX device and reachable from Home Assistant.
- Re-check the unit ids under **Settings → Services → Modbus/TCP → Available services** — they can differ from what another tool or guide assumed.
- Confirm **Settings → ESS → Mode** is still **External control**; anything else means the GX device's own loop is driving the Multi, not Omnibattery.
- If the battery SOC or power reading looks wrong, the telemetry register offsets are the most likely cause (see the warning above) — compare against the official Victron GX Home Assistant integration's own sensors for the same battery, if installed.
