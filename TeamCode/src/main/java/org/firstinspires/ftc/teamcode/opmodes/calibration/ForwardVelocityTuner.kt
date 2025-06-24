package org.firstinspires.ftc.teamcode.opmodes.calibration

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.TelemetryInterface

@Autonomous(name = "Forward Velocity Tuner", group = "Calibration")
class ForwardVelocityTuner : OpMode() {
    private var maxVelocity = 0.0 // Maximum velocity in cm/s
    private var accel = 0.0 // Used to gradually accelerate

    override fun init() {
        Bot.initialize(hardwareMap)
        Bot.setTelemetry(TelemetryInterface(telemetry, FtcDashboard.getInstance().telemetry))
        Bot.localizer.reset() // Reset the localizer to the origin

        // Update telemetry
        Bot.telemetry.addData("Max Velocity (cm/s)", maxVelocity)
        Bot.telemetry.addData("Current Pose", Bot.localizer.pose)
        Bot.telemetry.addData("Current Velocity", Bot.localizer.velocity)
        Bot.telemetry.update()
    }

    override fun loop() {
        // Update bot
        Bot.update()

        // Drive forward 150 cm
        if (Bot.localizer.pose.y < 150) {
            Bot.mecanumBase.moveVector(0.0, 1.0*accel, 0.0, adjustForStrafe = false)
            // Update max velocity
            if (Bot.localizer.velocity.y > maxVelocity) {
                maxVelocity = Bot.localizer.velocity.y
            }
            accel += 0.075
            accel.coerceAtMost(1.0)
        } else {
            Bot.mecanumBase.stop()
            terminateOpModeNow()
        }
        
        // Update telemetry
        Bot.telemetry.addData("Max Velocity (cm/s)", maxVelocity)
        Bot.telemetry.addData("Current Pose", Bot.localizer.pose)
        Bot.telemetry.addData("Current Velocity", Bot.localizer.velocity)
        Bot.telemetry.update()
    }

    override fun stop() {
        Bot.stop()
    }
}
