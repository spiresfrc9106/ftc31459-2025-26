package org.firstinspires.ftc.teamcode.opmodes.calibration

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.Bot

@Autonomous(name = "Horizontal Velocity Tuner", group = "Calibration")
class MaxHorizontalVelocityTuner : OpMode() {
    // TODO: Use pure pursuit to stay in a straight line?

    private var maxVelocity = 0.0 // Maximum velocity in cm/s
    private var accel = 0.0 // Used to gradually accelerate

    override fun init() {
        Bot.initialize(hardwareMap, telemetry)
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

        // Drive right 150 cm
        if (Bot.localizer.pose.x < 150) {
            Bot.mecanumBase.setDrivePower(1.0*accel, 0.0, 0.0, adjustForStrafe = false)
            // Update max velocity
            if (Bot.localizer.velocity.x > maxVelocity) {
                maxVelocity = Bot.localizer.velocity.x
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
