# How to get roadrunner and FTC Dashboard Telemetry to work.

From this google query:

"ftc using MultipleTelemetry breaks the canvas update"

We got the following Gemni AI response which did fix our telemetry problem:

## Gemni AI response

Using MultipleTelemetry with FtcDashboard can break the canvas update because of how the `update()`
method is handled, causing only the last packet sent within a single loop to be rendered. The
MultipleTelemetry class is designed to combine several Telemetry objects into one, but its use can
conflict with the way the dashboard processes and displays updates.

### The conflict explained

The FtcDashboard is designed to process and display a single `TelemetryPacket` per loop iteration, 
which it dispatches with the `sendTelemetryPacket()` method.

- How MultipleTelemetry works: When you create an object using `MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())`, a call to `multipleTelemetry.update()` in a single loop causes two `Telemetry.update()` calls to happen internally.

- The update clash:
  - The first update sends a packet to the driver station's telemetry log.
  - The second update, intended for the `FtcDashboard`, also sends a packet. However, because both updates happen within the same loop, the FtcDashboard may only fully process and render the last update it receives.

- Why the canvas breaks: Any canvas drawing operations are part of the telemetry packet sent to the dashboard. If a regular `telemetry.update()` is called after the packet with the canvas is sent, it can overwrite or interfere with the canvas update, making it appear broken or not render at all.

### How to fix the canvas update issue

To ensure that both your text telemetry and canvas graphics update correctly, you should use a single, dedicated TelemetryPacket for each loop and send it to the dashboard. This prevents the multiple, conflicting update calls.

Here is the correct workflow:
1. Instantiate `FtcDashboard` and a `new TelemetryPacket`.

```
FtcDashboard dashboard = FtcDashboard.getInstance();
TelemetryPacket packet = new TelemetryPacket();
```

2. Add text telemetry to the packet. Do not use telemetry.addData() or dashboardTelemetry.addData().

```
packet.put("x_value", 3.7);
packet.put("status", "alive");
```

3. Use the packet's `fieldOverlay()` to draw on the canvas.

```
packet.fieldOverlay().setFill("blue").fillRect(-20, -20, 40, 40);
```

4. Send the complete packet to the dashboard once per loop. This ensures that all updates are bundled and sent together, preventing conflicts.

```
dashboard.sendTelemetryPacket(packet);
```

### What to avoid:
- Mixing MultipleTelemetry with direct `FtcDashboard.sendTelemetryPacket()` calls.
- Calling `telemetry.update()` and `dashboard.sendTelemetryPacket()` within the same loop.

- By using a single, comprehensive TelemetryPacket and sending it explicitly, you avoid race conditions and ensure all dashboard features, including the canvas, update reliably.