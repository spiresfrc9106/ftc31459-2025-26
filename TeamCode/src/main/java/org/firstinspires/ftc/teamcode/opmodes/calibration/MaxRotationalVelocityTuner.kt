package org.firstinspires.ftc.teamcode.opmodes.calibration

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Bot

@Autonomous(name = "Rotational Velocity Tuner", group = "Calibration")
class MaxRotationalVelocityTuner : OpMode() {
    private val timer = ElapsedTime()

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

    override fun start() {
        timer.reset()
    }

    override fun loop() {
        // Update bot
        Bot.update()

        // Rotate for 5 seconds
        if (timer.seconds() < 5.0) {
            Bot.mecanumBase.moveVector(0.0, 0.0, accel, adjustForStrafe = false)
            // Update max velocity
            if (Bot.localizer.velocity.heading > maxVelocity) {
                maxVelocity = Bot.localizer.velocity.heading
            }
            accel += 0.075
            accel.coerceAtMost(1.0)
        } else {
            Bot.mecanumBase.stop()
            terminateOpModeNow()
        }

        // Update telemetry
        Bot.telemetry.addData("Max Velocity (rad/s)", maxVelocity)
        Bot.telemetry.addData("Current Pose", Bot.localizer.pose)
        Bot.telemetry.addData("Current Velocity", Bot.localizer.velocity)
        Bot.telemetry.update()
    }

    override fun stop() {
        Bot.stop()
    }
}
