package org.firstinspires.ftc.teamcode.opmodes.calibration

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Bot

@TeleOp(name = "Horizontal Velocity PIDF Tuner", group = "Calibration")
class HorizontalVelocityPIDFTuner : OpMode() {
    val timer = ElapsedTime()
    var targetVelocity: Double = 15.0 // Target velocity in cm/s

    override fun init() {
        Bot.initialize(hardwareMap, telemetry)

        Bot.telemetryPacket.put("Target Velocity", targetVelocity)
        Bot.telemetryPacket.put("Current Velocity", 0)
        Bot.sendTelemetryPacket()
    }

    override fun start() {
        timer.reset()
    }

    override fun loop() {
        Bot.update()

        Bot.mecanumBase.moveVelocity(targetVelocity, 0.0, 0.0)

        if (timer.seconds() < 2.5) {
            targetVelocity += 0.25
        } else if (timer.seconds() > 5.0) {
            targetVelocity -= 0.25
        }

        Bot.telemetryPacket.put("Target Velocity", targetVelocity)
        Bot.telemetryPacket.put("Current Velocity", Bot.localizer.velocity.x)
        Bot.sendTelemetryPacket()
    }
}
