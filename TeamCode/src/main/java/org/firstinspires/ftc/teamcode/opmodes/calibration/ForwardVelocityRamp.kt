package org.firstinspires.ftc.teamcode.opmodes.calibration

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.FileLogger

@Autonomous(name = "Forward Velocity Ramp", group = "Calibration")
class ForwardVelocityRamp : OpMode() {
    val startPower = 0.0
    val endPower = 0.9
    val rampTime = 5.0 // seconds
    val step = (endPower - startPower) / rampTime // Power increase per second

    val elapsedTime = ElapsedTime()
    var currentPower = startPower

    override fun init() {
        Bot.initialize(hardwareMap, telemetry)
        Bot.localizer.reset()
        elapsedTime.reset()

        Bot.telemetryPacket.put("Current Power", currentPower)
        Bot.telemetryPacket.put("Current Velocity", Bot.localizer.velocity.y)
        Bot.telemetryPacket.put("Elapsed Time", elapsedTime.seconds())
        Bot.sendTelemetryPacket()
    }

    override fun loop() {
        Bot.update()
        // Calculate the current power based on elapsed time
        val time = elapsedTime.seconds()
        if (time < rampTime) {
            currentPower = startPower + step * time
            Bot.mecanumBase.setDrivePower(0.0, currentPower, 0.0)

            // Add telemetry data
            Bot.telemetryPacket.put("Current Power", currentPower)
            Bot.telemetryPacket.put("Current Velocity", Bot.localizer.velocity.y)
            Bot.telemetryPacket.put("Elapsed Time", time)
            Bot.sendTelemetryPacket()

            // Save the power to velocity mapping
            FileLogger.info("ForwardVelocityRamp", "Power: $currentPower, Velocity: ${Bot.localizer.velocity.y}")
        } else {
            Bot.stop()
            requestOpModeStop()
        }
    }

    override fun stop() {
        FileLogger.flush()
    }
}
