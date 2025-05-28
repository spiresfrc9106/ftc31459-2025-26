package org.firstinspires.ftc.teamcode.opmodes.calibration

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.TelemetryInterface

@Autonomous(name = "Horizontal Velocity Tuner", group = "Calibration")
class HorizontalVelocityTuner : OpMode() {
    // TODO: Use pure pursuit to stay in a straight line?

    private val timer: ElapsedTime = ElapsedTime()

    private var dt: Double = 0.0 // Delta time in milliseconds
    private var prevTime = timer.milliseconds()

    private var maxVelocity = 0.0 // Maximum velocity in cm/s
    private var accel = 0.0 // Used to gradually accelerate

    override fun init() {
        Bot.initialize(hardwareMap)
        Bot.setTelemetry(TelemetryInterface(telemetry, FtcDashboard.getInstance().telemetry))
        Bot.localizer.reset() // Reset the localizer to the origin
        timer.reset()

        // Update telemetry
        Bot.telemetry.addData("Max Velocity (cm/s)", maxVelocity)
        Bot.telemetry.addData("Current Pose", Bot.localizer.pose)
        Bot.telemetry.addData("Current Velocity", Bot.localizer.velocity)
        Bot.telemetry.update()
    }

    override fun loop() {
        // Update localizer
        Bot.localizer.update(dt / 1000.0) // Convert milliseconds to seconds

        // Drive right 150 cm
        if (Bot.localizer.pose.x < 150) {
            Bot.mecanumBase.moveVector(1.0*accel, 0.0, 0.0, adjustForStrafe = false)
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

        // Update delta time
        dt = timer.milliseconds() - prevTime
        prevTime = timer.milliseconds()
    }
}
